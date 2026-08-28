//! `RC_SPEED` / PWM update-rate, upstream `QuadPlane::rc_speed`.
//!
//! Plane's `Q_RC_SPEED` is the fast RC/servo output Hertz. Analog
//! surfaces stay on `SERVO_RATE` (50 Hz); this is the motor/ESC rate.

use ap_rc::{
    apply_rc_speed, clamp_rc_speed, pwm_period_us, RcSpeed, RC_FAST_SPEED, RC_SPEED_DEFAULT,
    RC_SPEED_MAX, RC_SPEED_MIN,
};

#[test]
fn defaults_match_upstream_quadplane_rc_speed() {
    assert_eq!(RC_SPEED_DEFAULT, 490);
    assert_eq!(RC_SPEED_MIN, 50);
    assert_eq!(RC_SPEED_MAX, 500);
    assert_eq!(RC_FAST_SPEED, 490);
    assert_eq!(RcSpeed::default().hz, 490);
}

#[test]
fn default_490hz_is_2040us_frame() {
    assert_eq!(pwm_period_us(RC_SPEED_DEFAULT), 1_000_000 / 490);
    assert_eq!(apply_rc_speed(490).period_us(), 2040);
}

#[test]
fn analog_floor_is_20ms_and_max_is_2ms() {
    assert_eq!(pwm_period_us(RC_SPEED_MIN), 20_000);
    assert_eq!(pwm_period_us(RC_SPEED_MAX), 2_000);
}

#[test]
fn clamp_and_apply_reject_zero_and_overrange() {
    assert_eq!(clamp_rc_speed(0), RC_SPEED_MIN);
    assert_eq!(clamp_rc_speed(-1), RC_SPEED_MIN);
    assert_eq!(clamp_rc_speed(1000), RC_SPEED_MAX);
    assert_eq!(apply_rc_speed(0).hz, RC_SPEED_MIN);
    assert_eq!(apply_rc_speed(1000).hz, RC_SPEED_MAX);
    assert_eq!(apply_rc_speed(0).period_us(), 20_000);
}

#[test]
fn in_range_value_is_passed_to_set_freq() {
    let speed = apply_rc_speed(400);
    assert_eq!(speed.hz, 400);
    assert_eq!(speed.period_us(), 2_500);
}
