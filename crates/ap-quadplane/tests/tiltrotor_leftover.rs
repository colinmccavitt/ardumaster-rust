//! Leftover tiltrotor.cpp/h stubs — `update` / compensate / bicopter /
//! `write_log` / `get_forward_throttle` / `Tiltrotor_Transition`.

use ap_quadplane::tiltrotor::{
    BicopterIn, TiltType, TiltUpdateIn, Tiltrotor, TiltrotorConfig, TiltrotorLog, TiltrotorTransition,
    TILT_FAST_TILT_DPS, TILT_SERVO_MAX,
};
use ap_quadplane::transition_fsm::TransitionState;

fn enabled_mask() -> Tiltrotor {
    Tiltrotor::setup(TiltrotorConfig::with_tilt_mask(0b0011))
}

fn vectored() -> Tiltrotor {
    let cfg = TiltrotorConfig {
        enable: Some(1),
        tilt_mask: 0b0011,
        tilt_type: TiltType::VectoredYaw as i8,
    };
    Tiltrotor::setup(cfg)
}

fn binary() -> Tiltrotor {
    let cfg = TiltrotorConfig {
        enable: Some(1),
        tilt_mask: 0b0011,
        tilt_type: TiltType::Binary as i8,
    };
    Tiltrotor::setup(cfg)
}

fn bicopter() -> Tiltrotor {
    let cfg = TiltrotorConfig {
        enable: Some(1),
        tilt_mask: 0b0011,
        tilt_type: TiltType::Bicopter as i8,
    };
    Tiltrotor::setup(cfg)
}

#[test]
fn is_motor_tilting_reads_mask_bits() {
    let tr = enabled_mask();
    assert!(tr.is_motor_tilting(0));
    assert!(tr.is_motor_tilting(1));
    assert!(!tr.is_motor_tilting(2));
    assert!(!tr.is_motor_tilting(15));
    assert!(!tr.has_fw_motor());
    assert!(!tr.has_vtol_motor());
    assert!(!tr.motors_active());
}

#[test]
fn has_motor_flags_and_motors_active_gate() {
    let mut tr = enabled_mask();
    tr.set_have_fw_motor(true);
    tr.set_have_vtol_motor(true);
    assert!(tr.has_fw_motor());
    assert!(tr.has_vtol_motor());
    let disabled = Tiltrotor::setup(TiltrotorConfig::new());
    assert!(!disabled.motors_active());
}

#[test]
fn tilt_max_change_fast_tilt_is_at_least_90_dps() {
    let tr = enabled_mask();
    let dt = 0.1;
    let slow = tr.tilt_max_change(false, dt);
    let fast = tr.tilt_max_change_ex(false, false, true, dt);
    assert!((slow - (40.0 * dt / 90.0)).abs() < 1e-6);
    assert!((fast - (TILT_FAST_TILT_DPS * dt / 90.0)).abs() < 1e-6);
    assert!(fast > slow);
    // Hover-ward or flap-range does not take the 90 DPS floor.
    assert!(
        (tr.tilt_max_change_ex(true, false, true, dt) - slow).abs() < 1e-6
    );
    assert!(
        (tr.tilt_max_change_ex(false, true, true, dt) - slow).abs() < 1e-6
    );
}

#[test]
fn binary_slew_writes_servo_and_rate_limits_tilt() {
    let mut tr = binary();
    let dt = 0.1;
    let scaled = tr.binary_slew(true, dt);
    assert!((scaled - 1000.0).abs() < f32::EPSILON);
    assert!((tr.current_tilt() - (40.0 * dt / 90.0)).abs() < 1e-6);
    let back = tr.binary_slew(false, dt);
    assert!((back - 0.0).abs() < f32::EPSILON);
    assert!(tr.current_tilt() < 1e-6);
}

#[test]
fn binary_update_fw_masks_when_fully_forward() {
    let mut tr = binary();
    let mut inp = TiltUpdateIn::default();
    inp.in_vtol_mode = false;
    inp.fw_throttle = 50.0;
    inp.dt_s = 10.0;
    let out = tr.binary_update(inp);
    assert!(tr.motors_active());
    assert!(out.ran_motor_mask);
    assert!((out.motor_mask_throttle - 0.5).abs() < 1e-6);
    assert_eq!(out.motor_mask, 0b0011);
    assert_eq!(out.motor_tilt_scaled, Some(1000.0));
    assert!((tr.current_tilt() - 1.0).abs() < 1e-6);
}

#[test]
fn binary_update_vtol_slews_up() {
    let mut tr = binary();
    tr.binary_slew(true, 10.0);
    let mut inp = TiltUpdateIn::default();
    inp.in_vtol_mode = true;
    inp.dt_s = 10.0;
    let out = tr.binary_update(inp);
    assert!(!out.ran_motor_mask);
    assert_eq!(out.motor_tilt_scaled, Some(0.0));
    assert!(tr.current_tilt() < 1e-6);
}

#[test]
fn continuous_update_fw_disarmed_tilt_up() {
    let mut tr = enabled_mask();
    let mut inp = TiltUpdateIn::default();
    inp.disarmed_tilt_up = true;
    inp.manual_mode = false;
    inp.armed = false;
    inp.dt_s = 0.1;
    let out = tr.continuous_update(inp);
    assert!(!out.motors_active);
    assert!((out.current_throttle - 0.0).abs() < f32::EPSILON);
    assert!(tr.fully_up());
}

#[test]
fn continuous_update_fw_armed_runs_forward_mask() {
    let mut tr = enabled_mask();
    let mut inp = TiltUpdateIn::default();
    inp.armed = true;
    inp.in_vtol_mode = false;
    inp.assisted_flight = false;
    inp.fw_throttle = 80.0;
    inp.dt_s = 10.0;
    let out = tr.continuous_update(inp);
    assert!(out.motors_active);
    assert!(tr.motors_active());
    assert!(out.ran_motor_mask);
    assert!((out.motor_mask_throttle - 0.8).abs() < 1e-6);
    assert_eq!(out.motor_mask, 0b0011);
    assert!((tr.current_tilt() - 1.0).abs() < 1e-6);
}

#[test]
fn continuous_update_qautotune_slews_up() {
    let mut tr = enabled_mask();
    tr.slew(1.0, 10.0);
    let mut inp = TiltUpdateIn::default();
    inp.in_vtol_mode = true;
    inp.qautotune = true;
    inp.motors_throttle = 0.4;
    inp.dt_s = 10.0;
    let out = tr.continuous_update(inp);
    assert!(!out.motors_active);
    assert!((tr.current_tilt() - 0.0).abs() < 1e-6);
    assert!((tr.current_throttle() - 0.4).abs() < 1e-6);
}

#[test]
fn continuous_update_new_vfwd_tilts_from_forward_pct() {
    let mut tr = enabled_mask();
    let mut inp = TiltUpdateIn::default();
    inp.in_vtol_mode = true;
    inp.using_new_vfwd = true;
    inp.flying_vtol = true;
    inp.forward_throttle_pct = 100.0;
    inp.dt_s = 10.0;
    let _ = tr.continuous_update(inp);
    // atan(1) deg ≈ 45, capped at Q_TILT_MAX 45 → 45/90 = 0.5
    assert!((tr.current_tilt() - 0.5).abs() < 1e-5);
}

#[test]
fn continuous_update_qhover_manual_fwd_thr() {
    let mut tr = enabled_mask();
    let mut inp = TiltUpdateIn::default();
    inp.in_vtol_mode = true;
    inp.qacro_qstabilize_qhover = true;
    inp.has_rc_fwd_thr = true;
    inp.forward_throttle_pct = 100.0;
    inp.dt_s = 10.0;
    let _ = tr.continuous_update(inp);
    assert!((tr.current_tilt() - 0.5).abs() < 1e-5);

    inp.has_rc_fwd_thr = false;
    let _ = tr.continuous_update(inp);
    assert!(tr.current_tilt() < 1e-6);
}

#[test]
fn continuous_update_assisted_timer_goes_fully_fwd() {
    let mut tr = enabled_mask();
    let mut inp = TiltUpdateIn::default();
    inp.in_vtol_mode = true;
    inp.assisted_flight = true;
    inp.transition_state = TransitionState::Timer;
    inp.dt_s = 10.0;
    let _ = tr.continuous_update(inp);
    assert!((tr.current_tilt() - 1.0).abs() < 1e-6);
}

#[test]
fn update_disabled_or_zero_mask_is_noop() {
    let mut tr = Tiltrotor::setup(TiltrotorConfig::new());
    let out = tr.update(TiltUpdateIn::default());
    assert!(!out.ran_motor_mask);
    assert!(!out.ran_vectoring);
    let mut masked_binary = binary();
    let mut inp = TiltUpdateIn::default();
    inp.dt_s = 10.0;
    inp.in_vtol_mode = false;
    inp.fw_throttle = 10.0;
    let out = masked_binary.update(inp);
    assert!(!out.ran_vectoring);
    let mut v = vectored();
    let out = v.update(inp);
    assert!(out.ran_vectoring);
}

#[test]
fn tilt_compensate_skips_when_untilted() {
    let tr = enabled_mask();
    let mut thrust = [0.5, 0.5, 0.5, 0.5];
    tr.tilt_compensate(&mut thrust, true, 0.0, &[0.5, -0.5, 0.0, 0.0]);
    assert!((thrust[0] - 0.5).abs() < 1e-6);
}

#[test]
fn tilt_compensate_vtol_scales_fixed_motors() {
    let mut tr = enabled_mask();
    tr.slew(0.5, 10.0);
    assert!((tr.current_tilt() - 0.5).abs() < 1e-6);
    let mut thrust = [1.0, 1.0, 1.0, 1.0];
    tr.tilt_compensate(&mut thrust, true, 0.0, &[0.5, -0.5, 0.0, 0.0]);
    let cos45 = 0.5_f32.sqrt();
    assert!((thrust[0] - 1.0).abs() < 1e-5);
    assert!((thrust[1] - 1.0).abs() < 1e-5);
    assert!((thrust[2] - cos45).abs() < 1e-5);
    assert!((thrust[3] - cos45).abs() < 1e-5);
}

#[test]
fn tilt_compensate_fw_scales_tilted_then_renormalizes() {
    let mut tr = enabled_mask();
    tr.slew(0.5, 10.0);
    let mut thrust = [1.0, 1.0, 1.0, 1.0];
    tr.tilt_compensate(&mut thrust, false, 0.0, &[0.5, -0.5, 0.0, 0.0]);
    let cos45 = 0.5_f32.sqrt();
    assert!((thrust[0] - 1.0).abs() < 1e-5);
    assert!((thrust[1] - 1.0).abs() < 1e-5);
    assert!((thrust[2] - cos45).abs() < 1e-5);
    assert!((thrust[3] - cos45).abs() < 1e-5);
}

#[test]
fn bicopter_output_skips_wrong_type_and_motor_test() {
    let tr = enabled_mask();
    let inp = BicopterIn {
        motor_test_running: false,
        in_vtol_mode: true,
        assisted_flight: false,
        tilt_left: 100.0,
        tilt_right: -100.0,
    };
    let out = tr.bicopter_output(inp);
    assert!(!out.applied);
    let bc = bicopter();
    let mut test = inp;
    test.motor_test_running = true;
    assert!(!bc.bicopter_output(test).applied);
}

#[test]
fn bicopter_output_fw_fully_fwd_is_minus_servo_max() {
    let mut tr = bicopter();
    tr.slew(1.0, 10.0);
    assert!(tr.fully_fwd());
    let inp = BicopterIn {
        motor_test_running: false,
        in_vtol_mode: false,
        assisted_flight: false,
        tilt_left: 0.0,
        tilt_right: 0.0,
    };
    let out = tr.bicopter_output(inp);
    assert!(out.applied);
    assert!((out.left + TILT_SERVO_MAX).abs() < 1e-4);
    assert!((out.right + TILT_SERVO_MAX).abs() < 1e-4);
}

#[test]
fn bicopter_output_hover_scales_negative_by_yaw_angle() {
    let mut tr = bicopter();
    tr.set_tilt_yaw_angle(15.0);
    let inp = BicopterIn {
        motor_test_running: false,
        in_vtol_mode: true,
        assisted_flight: true,
        tilt_left: -TILT_SERVO_MAX,
        tilt_right: TILT_SERVO_MAX,
    };
    let out = tr.bicopter_output(inp);
    assert!(out.applied);
    assert!(out.motors_output_assisted);
    let expected_left = -TILT_SERVO_MAX * (15.0 / 90.0);
    assert!((out.left - expected_left).abs() < 1e-3);
    assert!((out.right - TILT_SERVO_MAX).abs() < 1e-3);
}

#[test]
fn write_log_disabled_is_none() {
    let tr = Tiltrotor::setup(TiltrotorConfig::new());
    assert!(tr.write_log(0.0, 0.0).is_none());
}

#[test]
fn write_log_continuous_nans_sides() {
    let mut tr = enabled_mask();
    tr.slew(0.5, 10.0);
    let log = tr.write_log(200.0, 300.0).expect("enabled");
    assert!((log.current_tilt_deg - 45.0).abs() < 1e-4);
    assert!(log.front_left_tilt.is_nan());
    assert!(log.front_right_tilt.is_nan());
    assert!(!log.sides_valid);
}

#[test]
fn write_log_vectored_from_servos() {
    let mut tr = vectored();
    tr.set_tilt_yaw_angle(15.0);
    tr.set_fixed_angle(10.0);
    tr.slew(1.0, 10.0);
    let log = tr.write_log(500.0, 250.0).expect("vectored");
    assert!((log.current_tilt_deg - 90.0).abs() < 1e-4);
    let scale = (90.0 + 15.0 + 10.0) * 0.001;
    assert!((log.front_left_tilt - (500.0 * scale - 15.0)).abs() < 1e-4);
    assert!((log.front_right_tilt - (250.0 * scale - 15.0)).abs() < 1e-4);
    assert!(log.sides_valid);
    let _ = TiltrotorLog {
        current_tilt_deg: log.current_tilt_deg,
        front_left_tilt: log.front_left_tilt,
        front_right_tilt: log.front_right_tilt,
        sides_valid: true,
    };
}

#[test]
fn get_forward_throttle_gates_and_averages() {
    let tr = enabled_mask();
    assert_eq!(tr.get_forward_throttle(0.1, 1.0, &[(0, 0.5)]), None);
    let v = vectored();
    assert_eq!(v.get_forward_throttle(0.2, 0.2, &[(0, 0.5)]), None);
    let thr = v
        .get_forward_throttle(0.1, 1.0, &[(0, 0.55), (1, 1.0), (2, 0.9)])
        .expect("tilting pair");
    // motors 0,1 tilting: (0.55-0.1)/0.9 and (1.0-0.1)/0.9
    let a = (0.55 - 0.1) / 0.9;
    let b = (1.0 - 0.1) / 0.9;
    assert!((thr - (a + b) * 0.5).abs() < 1e-6);
}

#[test]
fn transition_use_multirotor_and_view() {
    let tr = vectored();
    let t = tr.transition_view(false, TransitionState::Timer);
    assert!(t.use_multirotor_control_in_fwd_transition());
    assert!(t.show_vtol_view());
    let mut yaw = 0.0;
    assert!(t.update_yaw_target(&mut yaw));
    let done = tr.transition_view(false, TransitionState::Done);
    assert!(!done.use_multirotor_control_in_fwd_transition());
    assert!(!done.show_vtol_view());
    let vtol = tr.transition_view(true, TransitionState::Done);
    assert!(vtol.show_vtol_view());
    let plain = enabled_mask();
    let p = plain.transition_view(false, TransitionState::Timer);
    assert!(!p.use_multirotor_control_in_fwd_transition());
    assert!(!p.show_vtol_view());
    assert!(p.allow_vfwd(true, true, 0.5));
}

#[test]
fn transition_allow_vfwd_blocks_tilting_lost_motor() {
    let tr = vectored();
    let t = TiltrotorTransition::from_tiltrotor(&tr, false, TransitionState::AirspeedWait);
    assert!(t.allow_vfwd(false, true, 0.5));
    assert!(t.allow_vfwd(true, false, 0.5));
    assert!(t.allow_vfwd(true, true, 0.0));
    assert!(!t.allow_vfwd(true, true, 0.5));
}

#[test]
fn update_yaw_target_locks_and_integrates() {
    let mut tr = vectored();
    tr.update_yaw_target(1_000, 0.0, 12_000.0, None, 0, 10.0);
    assert!((tr.transition_yaw_cd() - 12_000.0).abs() < 1e-3);
    // Within 100 ms and no pilot yaw: keep lock, no airspeed → no integrate.
    tr.update_yaw_target(1_050, 0.0, 9_000.0, None, 2_000, 10.0);
    assert!((tr.transition_yaw_cd() - 12_000.0).abs() < 1e-3);
    // Pilot yaw re-locks to the sensor.
    tr.update_yaw_target(1_060, 50.0, 8_000.0, None, 0, 10.0);
    assert!((tr.transition_yaw_cd() - 8_000.0).abs() < 1e-3);
    // Bank + airspeed integrates coordinated-turn yaw.
    let before = tr.transition_yaw_cd();
    tr.update_yaw_target(1_160, 0.0, 8_000.0, Some(10.0), 4_500, 10.0);
    assert!(tr.transition_yaw_cd() > before);
}
