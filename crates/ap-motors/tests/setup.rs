//! Setup leftover -- set_throttle_factor, set_frame_class_and_type,
//! disable_yaw_torque, get_factors, thrust_compensation. COP-005.
//!
//! These are the helpers that sit around the factor table: scripting
//! may rewrite one motor's throttle before the frame is locked; a
//! disarmed vehicle may rebuild the table when FRAME_CLASS / FRAME_TYPE
//! change; yaw torque can be dropped for a vectored airframe; examples
//! can read the factors back; and a vehicle callback can retouch the
//! mixer thrusts for a tiltrotor.

#![allow(
    clippy::float_cmp,
    reason = "catalog emptiness, disabled-slot yaw zeros, and failed \
writes are exact; factor compares use a tight epsilon"
)]

use ap_motors::armed::REMAINING;
use ap_motors::setup::{
    disable_yaw_torque, get_factors, thrust_compensation, MotorSetup, FRAME_CLASS_SCRIPTING_MATRIX,
};
use ap_motors::{MotorMatrix, MAX_NUM_MOTORS};

const QUAD: u8 = 1;
const HEXA: u8 = 2;
const TYPE_X: u8 = 1;

fn quad_x() -> MotorMatrix {
    let mut m = MotorMatrix::new();
    assert!(m.setup_motors(QUAD, TYPE_X), "QUAD X");
    m
}

#[test]
fn leftover_catalog_is_empty() {
    assert!(REMAINING.is_empty(), "{REMAINING:?}");
    assert!(!REMAINING.contains(&"set_throttle_factor"));
    assert!(!REMAINING.contains(&"set_frame_class_and_type"));
    assert!(!REMAINING.contains(&"disable_yaw_torque"));
    assert!(!REMAINING.contains(&"get_factors"));
    assert!(!REMAINING.contains(&"thrust_compensation"));
}

#[test]
fn get_factors_returns_the_table_and_test_order() {
    let m = quad_x();
    let (f, order) = get_factors(&m, 0).expect("fitted");
    assert_eq!(order, 1);
    assert!((f.roll.abs() - 0.5).abs() < 1e-6, "roll {}", f.roll);
    assert!((f.pitch.abs() - 0.5).abs() < 1e-6, "pitch {}", f.pitch);
    assert!((f.yaw.abs() - 0.5).abs() < 1e-6, "yaw {}", f.yaw);
    assert!((f.throttle - 1.0).abs() < 1e-6, "throttle {}", f.throttle);
    assert!(get_factors(&m, 4).is_none(), "not fitted");
    assert!(get_factors(&m, 99).is_none(), "out of range");
}

#[test]
fn disable_yaw_torque_zeros_every_slot() {
    let mut m = quad_x();
    let before = get_factors(&m, 0).expect("fitted").0;
    assert!(before.yaw.abs() > 0.1, "quad X has yaw");

    disable_yaw_torque(&mut m);

    let after = get_factors(&m, 0).expect("fitted").0;
    assert_eq!(after.yaw, 0.0);
    assert_eq!(after.roll, before.roll);
    assert_eq!(after.pitch, before.pitch);
    assert_eq!(after.throttle, before.throttle);

    for i in 0..MAX_NUM_MOTORS {
        if let Some((f, _)) = get_factors(&m, i as u8) {
            assert_eq!(f.yaw, 0.0, "motor {i}");
        }
    }
}

#[test]
fn set_frame_class_and_type_rebuilds_when_disarmed() {
    let mut setup = MotorSetup::new();
    let mut m = MotorMatrix::new();
    setup.set_frame_class_and_type(&mut m, false, QUAD, TYPE_X);
    assert_eq!(setup.active_frame_class(), QUAD);
    assert_eq!(setup.active_frame_type(), TYPE_X);
    assert!(setup.initialised_ok());
    assert_eq!(m.num_motors(), 4);

    setup.set_frame_class_and_type(&mut m, false, HEXA, TYPE_X);
    assert_eq!(setup.active_frame_class(), HEXA);
    assert!(setup.initialised_ok());
    assert_eq!(m.num_motors(), 6);
}

#[test]
fn set_frame_class_and_type_is_a_noop_when_armed() {
    let mut setup = MotorSetup::new();
    let mut m = MotorMatrix::new();
    setup.set_frame_class_and_type(&mut m, false, QUAD, TYPE_X);
    setup.set_frame_class_and_type(&mut m, true, HEXA, TYPE_X);
    assert_eq!(setup.active_frame_class(), QUAD);
    assert_eq!(setup.active_frame_type(), TYPE_X);
    assert_eq!(m.num_motors(), 4);
}

#[test]
fn set_frame_class_and_type_is_a_noop_when_unchanged() {
    let mut setup = MotorSetup::new();
    let mut m = MotorMatrix::new();
    setup.set_frame_class_and_type(&mut m, false, QUAD, TYPE_X);
    m.remove_motor(0);
    assert_eq!(m.num_motors(), 3, "table was edited");
    setup.set_frame_class_and_type(&mut m, false, QUAD, TYPE_X);
    assert_eq!(m.num_motors(), 3, "same pair must not rebuild");
}

#[test]
fn scripting_class_skips_setup_motors() {
    let mut setup = MotorSetup::new();
    let mut m = MotorMatrix::new();
    m.add_motor(0, 45.0, 1.0, 1);
    setup.set_frame_class_and_type(&mut m, false, FRAME_CLASS_SCRIPTING_MATRIX, 0);
    assert_eq!(setup.active_frame_class(), FRAME_CLASS_SCRIPTING_MATRIX);
    assert!(
        !setup.initialised_ok(),
        "scripting init is a different function"
    );
    assert_eq!(m.num_motors(), 1, "must not wipe the table Lua is filling");
}

#[test]
fn set_throttle_factor_only_answers_for_scripting_before_init() {
    let mut setup = MotorSetup::new();
    let mut m = MotorMatrix::new();
    m.add_motor(0, 45.0, 1.0, 1);
    assert!(!setup.set_throttle_factor(&mut m, 0, 0.4), "wrong class");

    setup.set_frame_class_and_type(&mut m, false, FRAME_CLASS_SCRIPTING_MATRIX, 0);
    assert!(setup.set_throttle_factor(&mut m, 0, 0.4));
    let (f, _) = get_factors(&m, 0).expect("fitted");
    assert_eq!(f.throttle, 0.4);

    setup.set_initialised_ok(true);
    assert!(!setup.set_throttle_factor(&mut m, 0, 0.7), "already setup");
    let (f, _) = get_factors(&m, 0).expect("fitted");
    assert_eq!(f.throttle, 0.4, "must not overwrite after init");
}

#[test]
fn set_throttle_factor_rejects_a_missing_or_out_of_range_motor() {
    let mut setup = MotorSetup::new();
    let mut m = MotorMatrix::new();
    setup.set_frame_class_and_type(&mut m, false, FRAME_CLASS_SCRIPTING_MATRIX, 0);
    assert!(!setup.set_throttle_factor(&mut m, 0, 0.5), "not fitted");
    assert!(!setup.set_throttle_factor(&mut m, -1, 0.5));
    assert!(!setup.set_throttle_factor(&mut m, 99, 0.5));
}

#[test]
fn thrust_compensation_is_a_noop_without_a_callback() {
    let mut thrusts = [0.25_f32; MAX_NUM_MOTORS];
    thrust_compensation(&mut thrusts, None);
    assert!(thrusts.iter().all(|&t| t == 0.25));
}

fn tilt_half(thrust: &mut [f32; MAX_NUM_MOTORS]) {
    if let Some(slot) = thrust.get_mut(0) {
        *slot *= 0.5;
    }
}

#[test]
fn thrust_compensation_invokes_the_vehicle_callback() {
    let mut thrusts = [1.0_f32; MAX_NUM_MOTORS];
    thrust_compensation(&mut thrusts, Some(tilt_half));
    assert_eq!(thrusts[0], 0.5);
    assert_eq!(thrusts[1], 1.0);
}
