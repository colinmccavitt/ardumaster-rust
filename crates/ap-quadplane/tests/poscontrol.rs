//! Control-mode / poscontrol init — upstream `QuadPlane::setup`
//! attitude_control / pos_control allocation and `QuadPlane::mode_enter`.

use ap_quadplane::poscontrol::{PositionControlState, THROTTLE_WAIT_INPUT_MIN};
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

#[test]
fn setup_without_q_enable_does_not_init_cop_controllers() {
    let mut qp = QuadPlane::new();
    assert!(!qp.setup());
    assert!(!qp.attitude_control_inited());
    assert!(!qp.pos_control_inited());
    assert!(!qp.available());
}

#[test]
fn setup_with_q_enable_inits_attitude_and_pos_control() {
    // Upstream allocates AC_AttitudeControl_TS then AC_PosControl
    // after motors. The objects live in COP; this is the non-null flag.
    let mut qp = QuadPlane::with_enable(1);
    assert!(!qp.attitude_control_inited());
    assert!(!qp.pos_control_inited());
    assert!(qp.setup());
    assert!(qp.motors_inited());
    assert!(qp.attitude_control_inited());
    assert!(qp.pos_control_inited());
    assert!(qp.available());
}

#[test]
fn setup_is_idempotent_for_cop_controller_flags() {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    assert!(qp.setup());
    assert!(qp.attitude_control_inited());
    assert!(qp.pos_control_inited());
}

#[test]
fn mode_enter_when_available_resets_lean_angle_max() {
    let mut qp = available_qp();
    qp.set_lean_angle_max_cd(4500);
    assert_eq!(qp.lean_angle_max_cd(), 4500);
    qp.mode_enter();
    assert_eq!(qp.lean_angle_max_cd(), 0);
}

#[test]
fn mode_enter_when_unavailable_leaves_lean_angle_max() {
    // Upstream: `if (available()) { pos_control->set_lean_angle_max_cd(0); }`
    let mut qp = QuadPlane::with_enable(1);
    assert!(!qp.available());
    qp.set_lean_angle_max_cd(4500);
    qp.mode_enter();
    assert_eq!(qp.lean_angle_max_cd(), 4500);
}

#[test]
fn mode_enter_resets_poscontrol_to_qpos_none() {
    let mut qp = available_qp();
    qp.poscontrol_mut()
        .set_state(PositionControlState::Approach);
    qp.poscontrol_mut().set_correction_ne_m(12.0, -3.0);
    qp.poscontrol_mut().set_velocity_match_ms(4.0, 1.0, 1_234);
    qp.poscontrol_mut().set_pilot_correction(true, true);
    qp.poscontrol_mut().set_target_vel_ms(2.0, -1.0, 0.5);
    assert!(!qp.poscontrol().mode_enter_cleared());

    qp.mode_enter();

    assert_eq!(qp.poscontrol().state(), PositionControlState::None);
    assert!(qp.poscontrol().mode_enter_cleared());
}

#[test]
fn mode_enter_resets_poscontrol_even_when_unavailable() {
    let mut qp = QuadPlane::new();
    qp.poscontrol_mut()
        .set_state(PositionControlState::LandDescend);
    qp.poscontrol_mut().set_correction_ne_m(1.0, 1.0);
    qp.mode_enter();
    assert_eq!(qp.poscontrol().state(), PositionControlState::None);
    assert!(qp.poscontrol().mode_enter_cleared());
}

#[test]
fn mode_enter_snapshots_then_clears_guided_wait_takeoff() {
    let mut qp = available_qp();
    qp.set_guided_wait_takeoff(true);
    assert!(qp.guided_wait_takeoff());
    assert!(!qp.guided_wait_takeoff_on_mode_enter());

    qp.mode_enter();

    assert!(!qp.guided_wait_takeoff());
    assert!(qp.guided_wait_takeoff_on_mode_enter());

    qp.mode_enter();
    assert!(!qp.guided_wait_takeoff());
    assert!(!qp.guided_wait_takeoff_on_mode_enter());
}

#[test]
fn init_throttle_wait_sets_wait_on_the_ground_at_idle() {
    let mut qp = available_qp();
    qp.init_throttle_wait(0, false);
    assert!(qp.throttle_wait());
    qp.init_throttle_wait(9, false);
    assert!(qp.throttle_wait());
}

#[test]
fn init_throttle_wait_clears_when_stick_or_flying() {
    let mut qp = available_qp();
    qp.init_throttle_wait(THROTTLE_WAIT_INPUT_MIN, false);
    assert!(!qp.throttle_wait());
    qp.set_throttle_wait(true);
    qp.init_throttle_wait(0, true);
    assert!(!qp.throttle_wait());
}

#[test]
fn mode_enter_does_not_touch_throttle_wait() {
    // `throttle_wait` is Q-mode `_enter` (`init_throttle_wait` or a
    // forced false), not `mode_enter`.
    let mut qp = available_qp();
    qp.set_throttle_wait(true);
    qp.mode_enter();
    assert!(qp.throttle_wait());
}

#[test]
fn position_control_state_none_is_zero() {
    assert_eq!(PositionControlState::None as u8, 0);
    assert_eq!(PositionControlState::Approach as u8, 1);
    assert_eq!(PositionControlState::Airbrake as u8, 2);
}
