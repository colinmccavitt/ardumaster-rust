//! Does drift correction actually remove drift?
//!
//! The dead-reckoning test measured the problem: a one-degree-per-second gyro
//! bias walks the attitude off by exactly ten degrees in ten seconds. This
//! runs the same simulation with correction enabled and asks whether that
//! walk stops — and whether the integral term converges on the bias itself,
//! which is the mechanism rather than the symptom.
//!
//! A flight log cannot answer either question. It holds upstream's estimate,
//! so the most it could show is that the port drifts the same way ArduPilot
//! does. Only a simulator that knows the truth can say whether the estimate is
//! right.
//!
//! # The vehicle is level and stationary on purpose
//!
//! With no acceleration the measured gravity vector is unambiguous, so the
//! correction has a clean reference and the bias estimate is the only thing
//! that can explain the error. Adding motion would test more of the code and
//! make the result much harder to attribute.

#![allow(
    clippy::cast_possible_truncation,
    reason = "the simulator works in f64 and the port in f32; narrowing at the \
boundary is the interface being exercised"
)]

use ap_ahrs::{Dcm, DriftCorrector, DriftGains, DriftInputs, DriftOutcome, GpsLagBuffer, MatrixHealth};
use ap_math::matrix3::Matrix3f;
use ap_math::vector3::Vector3f;
use ap_sim::{level, AttitudeSim, V3};

fn to_port(v: V3) -> Vector3f {
    Vector3f::new(v.x as f32, v.y as f32, v.z as f32)
}

/// One estimator: the matrix, the correction, and the accumulator between
/// them. Assembling it here rather than in the library keeps the port's pieces
/// separable, which is what let each be tested on its own.
struct Estimator {
    dcm: Dcm,
    drift: DriftCorrector,
    ra_sum: Vector3f,
    ra_deltat: f32,
    gains: DriftGains,
}

impl Estimator {
    fn new(gains: DriftGains) -> Self {
        Self {
            dcm: Dcm::new(),
            drift: DriftCorrector::new(),
            ra_sum: Vector3f::zero(),
            ra_deltat: 0.0,
            gains,
        }
    }

    /// One IMU step: integrate, renormalise, accumulate acceleration, and
    /// correct once enough has built up.
    ///
    /// The correction interval is upstream's fallback path — a fifth of a
    /// second, which is what it uses without a GPS fix to trigger on.
    fn step(&mut self, accel_body: Vector3f, gyro: Vector3f, delta_angle: Vector3f, dt: f32) {
        self.dcm.matrix_update(
            Some((delta_angle, dt)),
            gyro,
            self.drift.omega_i,
            self.drift.omega_p,
            Vector3f::zero(),
        );
        assert_eq!(self.dcm.normalize(), MatrixHealth::Ok);

        let accel_ef = self.dcm.matrix * accel_body;
        DriftCorrector::accumulate(&mut self.ra_sum, &mut self.ra_deltat, accel_ef, dt);

        if self.ra_deltat >= 0.2 {
            let inputs = DriftInputs {
                ra_sum: self.ra_sum,
                ra_deltat: self.ra_deltat,
                // Stationary and level: no velocity change, so no centrifugal
                // correction is needed or available.
                velocity_delta: None,
                dcm_matrix: self.dcm.matrix,
                omega: self.dcm.omega,
                ins_healthy: true,
                using_gps_corrections: false,
                preselected_error: None,
            };
            let outcome = self.drift.correct(&inputs, &self.gains, &mut GpsLagBuffer::default());
            assert_eq!(outcome, DriftOutcome::Corrected, "correction should run");
            self.ra_sum = Vector3f::zero();
            self.ra_deltat = 0.0;
        }
    }
}

/// Roll error against truth, in degrees.
fn roll_error_deg(sim: &AttitudeSim, dcm: &Dcm) -> f64 {
    let true_roll = sim.true_euler().0;
    let est_roll = f64::from(dcm.matrix.c.y).atan2(f64::from(dcm.matrix.c.z));
    (est_roll - true_roll).to_degrees()
}

fn run_with_bias(bias_deg_s: f64, duration_s: f64, gains: DriftGains) -> (f64, Vector3f, f64) {
    let mut sim = AttitudeSim::new();
    sim.errors.gyro_bias = V3::new(bias_deg_s.to_radians(), 0.0, 0.0);
    let mut est = Estimator::new(gains);

    let dt = 0.0025;
    let steps = (duration_s / dt) as usize;
    let mut worst = 0.0_f64;
    for _ in 0..steps {
        let sample = sim.step(level(sim.time_s), dt);
        est.step(
            to_port(sample.accel),
            to_port(sample.gyro),
            to_port(sample.delta_angle),
            dt as f32,
        );
        worst = worst.max(roll_error_deg(&sim, &est.dcm).abs());
    }
    (roll_error_deg(&sim, &est.dcm), est.drift.omega_i, worst)
}

/// The headline: correction stops the walk that dead reckoning showed.
///
/// It does not drive the error to zero, and the reason is the whole design.
/// The batch clamp holds the bias estimate to the drift rate the *hardware* is
/// specified to have -- half a degree per minute. A one-degree-per-second bias
/// is a hundred and twenty times that, so the estimator is deliberately
/// forbidden from believing it. The proportional term carries what the integral
/// is not allowed to absorb, and it can only do that by standing at an error:
/// `omega_P = error * kp` has to equal the bias, so `error = bias / kp`.
///
/// Bounded and predictable, against thirty degrees and still climbing.
#[test]
fn correction_stops_the_drift_that_dead_reckoning_showed() {
    let (at_30, omega_i_30, _) = run_with_bias(1.0, 30.0, DriftGains::default());
    let (at_60, omega_i_60, _) = run_with_bias(1.0, 60.0, DriftGains::default());

    println!(
        "1 deg/s bias: {at_30:.3} deg at 30s (omega_I.x {:.5}), {at_60:.3} deg at 60s (omega_I.x {:.5})",
        omega_i_30.x, omega_i_60.x
    );

    // Dead reckoning gives 30 and 60 -- the drift is linear in time, and the
    // other test file measures exactly that. This is the contrast.
    assert!(
        at_30.abs() < 6.0 && at_60.abs() < 6.0,
        "error should stay bounded, got {at_30} then {at_60} degrees"
    );
    assert!(
        at_60.abs() <= at_30.abs(),
        "doubling the time should not grow the error -- that is the whole point; got {at_30} then {at_60}"
    );

    // The standing error is the bias the integral has not taken, over the gain.
    // Predicted from first principles, not read off a recorded run.
    let carried = 1.0_f64.to_radians() + f64::from(omega_i_60.x);
    let predicted = (carried / f64::from(DriftGains::default().kp)).to_degrees();
    assert!(
        (at_60.abs() - predicted).abs() < 0.5,
        "the standing error should be the uncorrected bias over the gain: predicted {predicted:.3}, got {:.3}",
        at_60.abs()
    );
}

/// The prediction above, tested where it bites: quadruple the proportional
/// gain and the standing error should quarter. A recorded expectation could not
/// make this claim, and an estimator quietly ignoring `kp` would sail past
/// every other test in this file.
#[test]
fn the_standing_error_is_inversely_proportional_to_the_gain() {
    let soft = DriftGains {
        kp: 0.2,
        ..DriftGains::default()
    };
    let stiff = DriftGains {
        kp: 0.8,
        ..DriftGains::default()
    };

    let (err_soft, _, _) = run_with_bias(1.0, 60.0, soft);
    let (err_stiff, _, _) = run_with_bias(1.0, 60.0, stiff);

    let ratio = err_stiff.abs() / err_soft.abs();
    println!(
        "kp 0.2 -> {:.3} deg, kp 0.8 -> {:.3} deg, ratio {ratio:.3} (theory 0.25)",
        err_soft.abs(),
        err_stiff.abs()
    );
    assert!(
        (ratio - 0.25).abs() < 0.05,
        "four times the gain should give a quarter the error, got {ratio}"
    );
}

/// And the flip side: a bias the hardware could plausibly have *is* absorbed,
/// completely. Half a degree per minute is exactly the drift rate the clamp is
/// sized for, so the integral is permitted to take all of it and the standing
/// error goes away.
///
/// This is the pair to the test above. Together they say the clamp is a
/// deliberate limit on what the estimator will believe, not a failure to
/// converge.
#[test]
fn a_bias_within_the_hardware_spec_is_absorbed_completely() {
    // radians(0.5/60) per second, expressed in degrees per second
    let bias_deg_s = 0.5 / 60.0;
    let (final_err, omega_i, _) = run_with_bias(bias_deg_s, 60.0, DriftGains::default());

    let expected = -bias_deg_s.to_radians();
    println!(
        "{bias_deg_s:.5} deg/s bias: omega_I.x {:.8} (expected {expected:.8}), residual error {final_err:.4} deg",
        omega_i.x
    );
    assert!(
        (f64::from(omega_i.x) - expected).abs() < 0.25 * expected.abs(),
        "the estimate should reach the bias, got {} against {expected}",
        omega_i.x
    );
    assert!(
        final_err.abs() < 0.1,
        "with the bias absorbed there should be almost nothing left, got {final_err} degrees"
    );
}

/// Sixty seconds at 400 Hz is 24,000 samples and 72,000 renormalised rows,
/// which laps upstream's `uint16_t` renorm counter. Nothing should depend on
/// that, and nothing should fall over because of it either -- the first version
/// of this port did, which is how the wrap was found.
#[test]
fn a_long_run_laps_the_renorm_counter_without_incident() {
    let mut sim = AttitudeSim::new();
    let mut est = Estimator::new(DriftGains::default());
    let dt = 0.0025;
    for _ in 0..24_000 {
        let sample = sim.step(level(sim.time_s), dt);
        est.step(
            to_port(sample.accel),
            to_port(sample.gyro),
            to_port(sample.delta_angle),
            dt as f32,
        );
    }
    let (_, count) = est.dcm.renorm_stats();
    println!("after 60s the renorm counter reads {count}, from 72,000 increments");
    // 72_000 - 65_536
    assert_eq!(count, 6_464);
    assert!(roll_error_deg(&sim, &est.dcm).abs() < 0.01);
}

/// The integral term updates in five-second batches, not continuously. That is
/// easy to miss reading the code and very visible here: the estimate holds
/// still and then steps.
#[test]
fn the_bias_estimate_updates_in_batches() {
    let mut sim = AttitudeSim::new();
    sim.errors.gyro_bias = V3::new(1.0_f64.to_radians(), 0.0, 0.0);
    let mut est = Estimator::new(DriftGains::default());

    let dt = 0.0025;
    let mut steps_at_first_change = None;
    for i in 0..4000 {
        let sample = sim.step(level(sim.time_s), dt);
        est.step(
            to_port(sample.accel),
            to_port(sample.gyro),
            to_port(sample.delta_angle),
            dt as f32,
        );
        if steps_at_first_change.is_none() && est.drift.omega_i.x != 0.0 {
            steps_at_first_change = Some(i);
        }
    }

    let first = steps_at_first_change.expect("the estimate should update eventually");
    let seconds = f64::from(first) * dt;
    println!("bias estimate first moved at {seconds:.2}s");
    assert!(
        seconds >= 5.0,
        "the batch is five seconds, so nothing should move before then; moved at {seconds}"
    );
}

/// A hard clamp exists so a burst of error cannot walk the bias estimate
/// somewhere the gyro could never have drifted to. With the drift rate set to
/// zero, the estimate must stay put however large the error.
#[test]
#[allow(
    clippy::float_cmp,
    reason = "exactly zero is the claim -- an epsilon here would let a real \
 movement through, which is the only thing this test is looking for"
)]
fn the_drift_rate_clamps_what_the_estimate_can_absorb() {
    let gains = DriftGains {
        gyro_drift_rate: 0.0,
        ..DriftGains::default()
    };
    let (_, omega_i, _) = run_with_bias(5.0, 30.0, gains);
    println!("with a zero drift-rate limit, omega_I.x = {:.9}", omega_i.x);
    assert_eq!(
        omega_i.x, 0.0,
        "a zero drift-rate limit should clamp every batch to nothing"
    );
}

/// Fast gains multiply the proportional term by eight and leave the integral
/// increment alone.
///
/// Tested on a single call rather than over a run, because over a run they are
/// not independent: a bigger `omega_P` moves the attitude, which changes the
/// error the integral then integrates. The first version of this test asserted
/// the closed-loop integrals matched, and they do not -- that is the loop
/// working, not a bug. The property that actually holds is this direct one.
#[test]
fn fast_gains_scale_the_proportional_term_and_not_the_integral() {
    // A tilted accumulation, so there is a real error to correct.
    let inputs = DriftInputs {
        ra_sum: Vector3f::new(0.5, 0.0, -9.0) * 0.2,
        ra_deltat: 0.2,
        velocity_delta: None,
        dcm_matrix: Matrix3f::identity(),
        omega: Vector3f::zero(),
        ins_healthy: true,
        using_gps_corrections: false,
        preselected_error: None,
    };

    let mut normal = DriftCorrector::new();
    let mut fast = DriftCorrector::new();
    normal.correct(&inputs, &DriftGains::default(), &mut GpsLagBuffer::default());
    fast.correct(
        &inputs,
        &DriftGains {
            fast_gains: true,
            ..DriftGains::default()
        },
        &mut GpsLagBuffer::default(),
    );

    println!(
        "omega_P normal {:?}, fast {:?}",
        normal.omega_p, fast.omega_p
    );
    assert_eq!(fast.omega_p, normal.omega_p * 8.0, "eight times, exactly");
    assert_eq!(
        fast.pending_integral(),
        normal.pending_integral(),
        "the integral increment should not know about fast gains"
    );
    assert_ne!(
        normal.omega_p,
        Vector3f::zero(),
        "the fixture must produce a real correction or this proves nothing"
    );
}

/// Spinning hard, the gravity reference is meaningless and the integral must
/// stop accumulating — otherwise the spin would be written into the bias
/// estimate and outlast it.
#[test]
#[allow(
    clippy::float_cmp,
    reason = "exactly zero is the claim -- an epsilon here would let a real \
 movement through, which is the only thing this test is looking for"
)]
fn a_fast_spin_suspends_the_bias_estimate() {
    let mut sim = AttitudeSim::new();
    let mut est = Estimator::new(DriftGains::default());

    // 100 deg/s in yaw, well past the 20 deg/s limit
    let spin = V3::new(0.0, 0.0, 100.0_f64.to_radians());
    let dt = 0.0025;
    for _ in 0..8000 {
        let sample = sim.step(spin, dt);
        est.step(
            to_port(sample.accel),
            to_port(sample.gyro),
            to_port(sample.delta_angle),
            dt as f32,
        );
    }

    let (pending, pending_time) = est.drift.pending_integral();
    println!(
        "after 20s spinning at 100 deg/s: omega_I {:?}, pending {pending:?} over {pending_time}s",
        est.drift.omega_i
    );
    assert_eq!(
        pending_time, 0.0,
        "no time should have been credited to the integral while spinning"
    );
    assert_eq!(
        est.drift.omega_i,
        Vector3f::zero(),
        "and the bias estimate should not have moved"
    );
}
