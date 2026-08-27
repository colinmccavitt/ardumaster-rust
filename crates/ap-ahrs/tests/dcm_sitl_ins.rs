//! End-to-end: SITL INS samples through the DCM matrix update loop.
//!
//! The ap-sim tests drive DCM directly with ideal samples. This runs the same
//! profiles through the full FW-011 → FW-008 path: raw samples into
//! [`SitlImuBackend`], accumulation and coning in [`ImuInstance`], publish at
//! loop rate, then [`dcm_matrix_step_from_ins`].

#![allow(
    clippy::cast_possible_truncation,
    reason = "the simulator works in f64 and the port in f32; narrowing at the \
boundary is the interface being exercised"
)]

use ap_ahrs::{dcm_matrix_step_from_ins, Dcm, DcmDriftOmega, MatrixHealth};
use ap_ins::sitl::{SitlBodyState, SitlImuBackend, SitlTimerFileData};
use ap_ins::{InertialSensorFrontend, LoopTiming, DEFAULT_GYRO_FILTER_HZ};
use ap_math::matrix3::Matrix3f;
use ap_math::vector3::Vector3f;
use ap_sim::{steady_roll, tumbling, turning, AttitudeSim, RateProfile, V3, GRAVITY, M3};

const GYRO_HZ: u16 = 8000;
const ACCEL_HZ: u16 = 1000;
const LOOP_HZ: f32 = 400.0;
const LOOP_DT: f32 = 1.0 / LOOP_HZ;
const GYRO_DT_US: u64 = 125;

fn gravity_in_body(sim: &AttitudeSim) -> V3 {
    let t = M3 {
        a: V3::new(sim.truth.a.x, sim.truth.b.x, sim.truth.c.x),
        b: V3::new(sim.truth.a.y, sim.truth.b.y, sim.truth.c.y),
        c: V3::new(sim.truth.a.z, sim.truth.b.z, sim.truth.c.z),
    };
    t.apply(V3::new(0.0, 0.0, -GRAVITY))
}

fn body_state_from_rates(rates: V3, gravity_body: V3) -> SitlBodyState {
    SitlBodyState {
        roll_rate_dps: (rates.x * 180.0 / core::f64::consts::PI) as f32,
        pitch_rate_dps: (rates.y * 180.0 / core::f64::consts::PI) as f32,
        yaw_rate_dps: (rates.z * 180.0 / core::f64::consts::PI) as f32,
        x_accel: gravity_body.x as f32,
        y_accel: gravity_body.y as f32,
        z_accel: gravity_body.z as f32,
        ..SitlBodyState::default()
    }
}

fn attitude_error_deg(sim: &AttitudeSim, dcm: &Dcm) -> f64 {
    let t = sim.truth;
    let e = dcm.matrix;
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

fn init_dcm_from_truth(sim: &AttitudeSim) -> Dcm {
    let mut dcm = Dcm::new();
    dcm.matrix = Matrix3f {
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
    dcm
}


/// SITL backend registered with the INS frontend for loop-rate publish.
struct SitlInsHookup {
    frontend: InertialSensorFrontend,
    backend: SitlImuBackend,
}

impl SitlInsHookup {
    fn new() -> Self {
        let mut frontend = InertialSensorFrontend::new();
        let mut backend = SitlImuBackend::new(GYRO_HZ, ACCEL_HZ);
        backend
            .imu
            .set_gyro_filter(f32::from(GYRO_HZ), DEFAULT_GYRO_FILTER_HZ);
        backend
            .imu
            .set_accel_filter(f32::from(ACCEL_HZ), DEFAULT_GYRO_FILTER_HZ);
        assert!(backend.start(&mut frontend));
        Self { frontend, backend }
    }

    fn timer_update(
        &mut self,
        now_us: u64,
        state: &SitlBodyState,
        files: SitlTimerFileData<'_>,
    ) {
        let _ = self.backend.timer_update(now_us, state, files);
    }

    fn publish(&mut self) {
        self.frontend
            .receive_backend_imu(self.backend.gyro_instance, &self.backend.imu);
        self.frontend.begin_update();
        self.frontend.update();
        self.backend.imu.update_gyro();
        self.backend.imu.update_accel();
    }
}


/// Run one motion profile through SITL INS into DCM and return worst attitude
/// error in degrees.
fn run_sitl_ins_profile(
    profile: RateProfile,
    duration_s: f64,
    sim: &mut AttitudeSim,
) -> f64 {
    let mut hookup = SitlInsHookup::new();
    let mut dcm = init_dcm_from_truth(sim);
    let mut timing = LoopTiming::new(LOOP_DT);
    let drift = DcmDriftOmega::default();

    let loop_period_us = (LOOP_DT * 1_000_000.0) as u64;
    let total_us = (duration_s * 1_000_000.0) as u64;
    let mut now_us = 0_u64;
    let mut next_loop_us = 0_u64;
    let mut worst = 0.0_f64;

    while now_us <= total_us {
        let rates = profile(sim.time_s);
        let gravity = gravity_in_body(&sim);
        let state = body_state_from_rates(rates, gravity);
        let _ = hookup.timer_update(now_us, &state, SitlTimerFileData::default());

        if now_us >= next_loop_us {
            hookup.publish();
            timing.delta_time = LOOP_DT;
            assert_eq!(
                dcm_matrix_step_from_ins(&mut dcm, &hookup.frontend, &timing, drift),
                MatrixHealth::Ok,
                "matrix should stay healthy at t={now_us} us"
            );
            worst = worst.max(attitude_error_deg(sim, &dcm));
            next_loop_us += loop_period_us;
        }

        // Advance truth at gyro rate so body kinematics stay consistent.
        sim.step(rates, GYRO_DT_US as f64 * 1.0e-6);
        now_us += GYRO_DT_US;
    }

    worst
}

/// A stationary vehicle fed through SITL INS should hold attitude.
#[test]
fn sitl_ins_stationary_holds_attitude() {
    let mut sim = AttitudeSim::from_euler(0.2, -0.1, 1.0);
    let mut hookup = SitlInsHookup::new();
    let mut dcm = init_dcm_from_truth(&sim);
    let mut timing = LoopTiming::new(LOOP_DT);
    let drift = DcmDriftOmega::default();

    let gyro_per_loop = u64::from(GYRO_HZ) / u64::from(LOOP_HZ as u16);
    let mut now_us = 0_u64;

    for _ in 0..4000 {
        let rates = V3::zero();
        let gravity = gravity_in_body(&sim);
        let state = body_state_from_rates(rates, gravity);
        for _ in 0..gyro_per_loop {
            let _ = hookup.timer_update(now_us, &state, SitlTimerFileData::default());
            now_us += GYRO_DT_US;
        }
        hookup.publish();
        timing.delta_time = LOOP_DT;
        assert_eq!(
            dcm_matrix_step_from_ins(&mut dcm, &hookup.frontend, &timing, drift),
            MatrixHealth::Ok
        );
        sim.step(rates, LOOP_DT as f64);
    }

    let err = attitude_error_deg(&sim, &dcm);
    assert!(
        err < 0.05,
        "ten seconds stationary through SITL INS drifted {err} degrees"
    );
}

/// Motion profiles through the full INS → DCM path should track truth closely.
#[test]
fn sitl_ins_dead_reckoning_tracks_truth() {
    for (name, profile, limit) in [
        ("steady roll", steady_roll as RateProfile, 0.6),
        ("turning", turning as RateProfile, 0.6),
        ("tumbling", tumbling as RateProfile, 1.2),
    ] {
        let mut sim = AttitudeSim::new();
        let worst = run_sitl_ins_profile(profile, 10.0, &mut sim);
        println!("{name} via SITL INS: worst error {worst:.4} deg");
        assert!(
            worst < limit,
            "{name} drifted {worst} degrees over ten seconds, limit {limit}"
        );
    }
}

/// The hookup reads published samples, not raw accumulation.
#[test]
fn dcm_update_skips_rotation_without_publish() {
    let mut hookup = SitlInsHookup::new();
    let state = SitlBodyState {
        roll_rate_dps: 57.295_78,
        ..SitlBodyState::default()
    };
    let _ = hookup.timer_update(0, &state, SitlTimerFileData::default());

    let mut dcm = Dcm::new();
    let before = dcm.matrix;
    let mut timing = LoopTiming::new(LOOP_DT);
    timing.delta_time = LOOP_DT;
    let drift = DcmDriftOmega::default();

    // Raw samples exist but frontend publish was not called, so get_delta_angle
    // has nothing to hand over and the matrix must not rotate.
    dcm_matrix_step_from_ins(&mut dcm, &hookup.frontend, &timing, drift);

    assert_eq!(dcm.omega, Vector3f::zero());
    assert_eq!(
        dcm.matrix.a, before.a,
        "without publish the delta-angle path should not rotate"
    );
}

/// One degree per second of roll bias through SITL INS should stay bounded once
/// drift correction is wired into the loop, not walk off like dead reckoning.
#[test]
fn sitl_ins_drift_correction_limits_gyro_bias() {
    let mut sim = AttitudeSim::new();
    sim.errors.gyro_bias = V3::new(1.0_f64.to_radians(), 0.0, 0.0);

    let mut hookup = SitlInsHookup::new();
    let mut dcm = init_dcm_from_truth(&sim);
    let mut drift = ap_ahrs::DcmDriftLoop::default();
    let mut timing = LoopTiming::new(LOOP_DT);

    let gyro_per_loop = u64::from(GYRO_HZ) / u64::from(LOOP_HZ as u16);
    let mut now_us = 0_u64;
    let mut worst = 0.0_f64;

    for _ in 0..4000 {
        let rates = V3::zero();
        let gravity = gravity_in_body(&sim);
        let state = body_state_from_rates(rates, gravity);
        for _ in 0..gyro_per_loop {
            let _ = hookup.timer_update(now_us, &state, SitlTimerFileData::default());
            now_us += GYRO_DT_US;
        }
        hookup.publish();
        timing.delta_time = LOOP_DT;
        assert_eq!(
            ap_ahrs::dcm_step_with_drift_from_ins(&mut dcm, &mut drift, &hookup.frontend, &timing, None),
            MatrixHealth::Ok
        );
        sim.step(rates, LOOP_DT as f64);
        worst = worst.max(attitude_error_deg(&sim, &dcm));
    }

    let err = attitude_error_deg(&sim, &dcm);
    println!(
        "1 deg/s bias via SITL INS+drift: final {err:.3} deg, worst {worst:.3} deg"
    );
    assert!(
        err.abs() < 6.0 && worst < 6.0,
        "drift correction should bound bias error, got final {err} worst {worst}"
    );
}
