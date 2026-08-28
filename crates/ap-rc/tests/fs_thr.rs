//! RC failsafe PWM threshold, upstream `Plane::rc_throttle_value_ok`.
//!
//! Throttle PWM at or below `THR_FS_VALUE` / `FS_THR_VALUE` is failsafe.

use ap_rc::{
    throttle_below_fs_thr_value, throttle_pwm_in_failsafe, ThrFailsafe, FS_THR_VALUE_DEFAULT,
    THR_FS_VALUE_DEFAULT,
};

#[test]
fn throttle_below_threshold_trips_failsafe() {
    assert!(throttle_below_fs_thr_value(900));
    assert!(throttle_pwm_in_failsafe(
        900,
        THR_FS_VALUE_DEFAULT,
        ThrFailsafe::Enabled,
        false
    ));
    assert!(throttle_pwm_in_failsafe(
        910,
        FS_THR_VALUE_DEFAULT,
        ThrFailsafe::Enabled,
        false
    ));
}

#[test]
fn throttle_above_threshold_is_healthy() {
    assert!(!throttle_below_fs_thr_value(1100));
    assert!(!throttle_pwm_in_failsafe(
        1100,
        FS_THR_VALUE_DEFAULT,
        ThrFailsafe::Enabled,
        false
    ));
    assert!(!throttle_pwm_in_failsafe(
        1500,
        FS_THR_VALUE_DEFAULT,
        ThrFailsafe::Enabled,
        false
    ));
}

#[test]
fn threshold_itself_is_failsafe() {
    assert!(throttle_below_fs_thr_value(THR_FS_VALUE_DEFAULT));
    assert!(!throttle_below_fs_thr_value(THR_FS_VALUE_DEFAULT + 1));
}

#[test]
fn disabled_thr_failsafe_never_trips_on_pwm() {
    assert!(!throttle_pwm_in_failsafe(
        0,
        THR_FS_VALUE_DEFAULT,
        ThrFailsafe::Disabled,
        false
    ));
}

#[test]
fn reversed_throttle_trips_when_pwm_is_high() {
    assert!(throttle_pwm_in_failsafe(
        2000,
        THR_FS_VALUE_DEFAULT,
        ThrFailsafe::Enabled,
        true
    ));
    assert!(!throttle_pwm_in_failsafe(
        800,
        THR_FS_VALUE_DEFAULT,
        ThrFailsafe::Enabled,
        true
    ));
}
