//! Does the DCM estimator actually estimate?
//!
//! The log-replay tests answer a different question — whether the port matches
//! ArduPilot. They cannot answer this one, because a flight log contains
//! upstream's *estimate* and not the truth it was estimating. If the port
//! reproduced an upstream bug exactly, replay would pass.
//!
//! So this drives the ported estimator from `ap_sim`, whose true attitude is
//! known exactly and whose truth propagation shares no arithmetic with the
//! port: Rodrigues' formula in `f64`, against the port's first-order step in
//! `f32`.
//!
//! # What is and is not being claimed
//!
//! This slice of DCM is dead reckoning only — matrix integration and
//! renormalisation, with no drift correction, because that is all that is
//! ported so far. So the test asserts what dead reckoning can deliver: that
//! the attitude tracks truth closely over a short run, that the error grows
//! rather than staying at zero, and that a gyro bias produces exactly the
//! drift it should. When drift correction lands, the same simulator will show
//! that error being pulled back, which is the point of having it.

#![allow(
    clippy::cast_possible_truncation,
    reason = "the simulator works in f64 and the port in f32; narrowing at the \
boundary is the interface being exercised"
)]

use ap_ahrs::{Dcm, MatrixHealth};
use ap_math::vector3::Vector3f;
use ap_sim::{level, steady_roll, tumbling, turning, AttitudeSim, RateProfile, V3};

/// Convert a simulator vector to the port's.
fn to_port(v: V3) -> Vector3f {
    Vector3f::new(v.x as f32, v.y as f32, v.z as f32)
}

/// Run the estimator over a profile and return the worst attitude error, in
/// degrees, along with the true and estimated Euler angles at the end.
fn run(
    profile: RateProfile,
    duration_s: f64,
    dt: f64,
    sim: &mut AttitudeSim,
) -> (f64, (f64, f64, f64), (f64, f64, f64)) {
    let mut dcm = Dcm::new();
    let mut worst = 0.0_f64;

    let steps = (duration_s / dt) as usize;
    for _ in 0..steps {
        let rates = profile(sim.time_s);
        let sample = sim.step(rates, dt);

        dcm.matrix_update(
            Some((to_port(sample.delta_angle), sample.delta_angle_dt as f32)),
            to_port(sample.gyro),
            Vector3f::zero(),
            Vector3f::zero(),
            Vector3f::zero(),
        );
        assert_eq!(
            dcm.normalize(),
            MatrixHealth::Ok,
            "the matrix should stay healthy on a well-behaved profile"
        );

        worst = worst.max(attitude_error_deg(sim, &dcm));
    }

    let est = estimated_euler(&dcm);
    (worst, sim.true_euler(), est)
}

/// Roll, pitch and yaw the port's matrix represents, in radians.
fn estimated_euler(dcm: &Dcm) -> (f64, f64, f64) {
    let m = dcm.matrix;
    let pitch = f64::from(-m.c.x).clamp(-1.0, 1.0).asin();
    let roll = f64::from(m.c.y).atan2(f64::from(m.c.z));
    let yaw = f64::from(m.b.x).atan2(f64::from(m.a.x));
    (roll, pitch, yaw)
}

/// The angle between the true and estimated attitudes, in degrees.
///
/// Compared as a rotation angle rather than per-axis Euler differences, which
/// would blow up near a pitch singularity and understate error elsewhere.
fn attitude_error_deg(sim: &AttitudeSim, dcm: &Dcm) -> f64 {
    let t = sim.truth;
    let e = dcm.matrix;
    // trace of truth^T * estimate; the rotation angle between them is
    // acos((trace - 1) / 2)
    let trace = t.a.x * f64::from(e.a.x)
        + t.a.y * f64::from(e.a.y)
        + t.a.z * f64::from(e.a.z)
        + t.b.x * f64::from(e.b.x)
        + t.b.y * f64::from(e.b.y)
        + t.b.z * f64::from(e.b.z)
        + t.c.x * f64::from(e.c.x)
        + t.c.y * f64::from(e.c.y)
        + t.c.z * f64::from(e.c.z);
    ((trace - 1.0) / 2.0).clamp(-1.0, 1.0).acos().to_degrees()
}

/// A vehicle that is not moving should be estimated as not moving. This is the
/// weakest possible check and the one that would catch a sign error or a
/// runaway integration immediately.
#[test]
fn a_stationary_vehicle_holds_its_attitude() {
    let mut sim = AttitudeSim::from_euler(0.2, -0.1, 1.0);
    let mut dcm = Dcm::new();
    dcm.matrix = ap_math::matrix3::Matrix3f {
        a: Vector3f::new(
            sim.truth.a.x as f32,
            sim.truth.a.y as f32,
            sim.truth.a.z as f32,
        ),
        b: Vector3f::new(
            sim.truth.b.x as f32,
            sim.truth.b.y as f32,
            sim.truth.b.z as f32,
        ),
        c: Vector3f::new(
            sim.truth.c.x as f32,
            sim.truth.c.y as f32,
            sim.truth.c.z as f32,
        ),
    };

    for _ in 0..4000 {
        let sample = sim.step(level(sim.time_s), 0.0025);
        dcm.matrix_update(
            Some((to_port(sample.delta_angle), sample.delta_angle_dt as f32)),
            to_port(sample.gyro),
            Vector3f::zero(),
            Vector3f::zero(),
            Vector3f::zero(),
        );
        assert_eq!(dcm.normalize(), MatrixHealth::Ok);
    }
    let err = attitude_error_deg(&sim, &dcm);
    assert!(err < 0.01, "ten seconds stationary drifted {err} degrees");
}

/// Dead reckoning over a real motion, at a realistic IMU rate.
#[test]
fn dead_reckoning_tracks_truth_over_a_short_run() {
    for (name, profile, limit) in [
        ("steady roll", steady_roll as RateProfile, 0.5),
        ("turning", turning as RateProfile, 0.5),
        ("tumbling", tumbling as RateProfile, 1.0),
    ] {
        let mut sim = AttitudeSim::new();
        let (worst, truth, est) = run(profile, 10.0, 0.0025, &mut sim);
        println!(
            "{name}: worst error {worst:.4} deg; truth rpy {:.3} {:.3} {:.3}, est {:.3} {:.3} {:.3}",
            truth.0, truth.1, truth.2, est.0, est.1, est.2
        );
        assert!(
            worst < limit,
            "{name} drifted {worst} degrees over ten seconds, limit {limit}"
        );
    }
}

/// The error is not zero, and saying so matters: a first-order integration
/// step accumulates, and a test that only checked an upper bound would pass
/// just as happily if the estimator were secretly using the truth.
#[test]
fn dead_reckoning_error_is_real_and_grows() {
    let mut sim = AttitudeSim::new();
    let (short, _, _) = run(tumbling, 2.0, 0.0025, &mut sim);
    let mut sim = AttitudeSim::new();
    let (long, _, _) = run(tumbling, 20.0, 0.0025, &mut sim);

    println!("tumbling: {short:.5} deg over 2s, {long:.5} deg over 20s");
    assert!(short > 0.0, "a first-order step cannot be exact");
    assert!(
        long > short,
        "error should accumulate without drift correction: {short} then {long}"
    );
}

/// A coarser IMU rate should integrate worse. This pins the relationship the
/// estimator depends on, and would catch a step that ignored its interval.
#[test]
fn a_coarser_sample_rate_integrates_worse() {
    let mut fast_sim = AttitudeSim::new();
    let (fast, _, _) = run(tumbling, 10.0, 0.0025, &mut fast_sim);
    let mut slow_sim = AttitudeSim::new();
    let (slow, _, _) = run(tumbling, 10.0, 0.02, &mut slow_sim);

    println!("tumbling: {fast:.4} deg at 400 Hz, {slow:.4} deg at 50 Hz");
    assert!(
        slow > fast,
        "50 Hz should be worse than 400 Hz: {fast} then {slow}"
    );
}

/// A gyro bias is exactly what the integral drift term exists to remove, and
/// with no drift correction ported yet it should show up undiminished: about
/// one degree per second of bias, per second.
#[test]
fn a_gyro_bias_drifts_at_the_rate_it_should() {
    let bias_deg_s: f64 = 1.0;
    let mut sim = AttitudeSim::new();
    sim.errors.gyro_bias = V3::new(bias_deg_s.to_radians(), 0.0, 0.0);

    let (_, _, est) = run(level, 10.0, 0.0025, &mut sim);
    let drift_deg = est.0.to_degrees();

    println!("1 deg/s of roll bias for 10s produced {drift_deg:.3} degrees");
    assert!(
        (drift_deg - 10.0).abs() < 0.1,
        "expected about ten degrees of drift, got {drift_deg}"
    );
}
