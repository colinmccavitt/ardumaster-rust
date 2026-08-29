//! Standalone `PosVelEKF` leftover, upstream `PosVelEKF.cpp`.
//!
//! Tracked as **COP-028**. Algebra matches the C++ 1-D predict / fuse /
//! NIS exactly. `run_output_prediction` is a separate leftover.

use ap_math::scalar::is_equal;
use ap_precland::{PosVelEKF, REMAINING};

fn almost(a: f32, b: f32) {
    assert!(is_equal(a, b), "{a} != {b}");
}

#[test]
fn init_sets_state_and_diagonal_covariance() {
    let mut ekf = PosVelEKF::new();
    ekf.init(1.5, 0.25, -2.0, 4.0);
    almost(ekf.pos(), 1.5);
    almost(ekf.vel(), -2.0);
    assert_eq!(ekf.cov(), [0.25, 0.0, 4.0]);
}

#[test]
fn predict_integrates_velocity_and_delta_vel() {
    let mut ekf = PosVelEKF::new();
    ekf.init(1.0, 0.25, 2.0, 1.0);
    ekf.predict(0.1, 0.5, 0.2);
    // newState = [dt*vel + pos, dVel + vel] = [1.2, 2.5]
    almost(ekf.pos(), 1.2);
    almost(ekf.vel(), 2.5);
    // newCov[0] = dt*P01 + dt*(dt*P11 + P01) + P00 = 0.01 + 0.25
    // newCov[1] = dt*P11 + P01 = 0.1
    // newCov[2] = dVelNoise² + P11 = 0.04 + 1.0
    almost(ekf.cov()[0], 0.26);
    almost(ekf.cov()[1], 0.1);
    almost(ekf.cov()[2], 1.04);
}

#[test]
fn fuse_pos_and_nis_match_upstream_algebra() {
    let mut ekf = PosVelEKF::new();
    ekf.init(1.0, 0.25, 2.0, 1.0);
    ekf.predict(0.1, 0.5, 0.2);
    // Use the live state so f32 rounding matches PosVelEKF.cpp order.
    let pos = ekf.pos();
    let vel = ekf.vel();
    let cov = ekf.cov();
    let innov = 1.3 - pos;
    let s = cov[0] + 0.04;
    almost(ekf.pos_nis(1.3, 0.04), (innov * innov) / s);
    ekf.fuse_pos(1.3, 0.04);
    almost(ekf.pos(), cov[0] * innov / s + pos);
    almost(ekf.vel(), cov[1] * innov / s + vel);
    almost(ekf.cov()[0], cov[0] * 0.04 / s);
    almost(ekf.cov()[1], cov[1] * 0.04 / s);
    almost(ekf.cov()[2], -cov[1] * cov[1] / s + cov[2]);
}

#[test]
fn leftover_catalog_drops_posvelekf() {
    assert!(
        REMAINING.len() >= 8,
        "PosVelEKF slice must not claim the 1,133-loc ticket is done"
    );
    assert!(!REMAINING.contains(&"PosVelEKF"));
    assert!(!REMAINING.contains(&"AC_PrecLand::run_output_prediction"));
    assert!(REMAINING.contains(&"AC_PrecLand_StateMachine::update"));
    assert!(!REMAINING.contains(&"inertial_data_frame_s"));
}
