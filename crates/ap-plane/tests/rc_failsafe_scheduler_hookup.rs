//! RC failsafe scheduler hookup wiring.

use ap_plane::rc_failsafe_scheduler_hookup::{
    detect_rc_failsafe, norm_input_dz, rc_failsafe_scheduler_tick, RcChannelConfig,
    RcFailsafeConfig, RcFailsafeSchedulerInputs,
};

#[test]
fn norm_input_dz_is_neutral_at_trim() {
    let cfg = RcChannelConfig::default();
    assert_eq!(norm_input_dz(1500, &cfg), 0.0);
}

#[test]
fn norm_input_dz_reaches_extremes_at_limits() {
    let cfg = RcChannelConfig::default();
    assert!((norm_input_dz(1100, &cfg) + 1.0).abs() < 0.01);
    assert!((norm_input_dz(1900, &cfg) - 1.0).abs() < 0.01);
}

#[test]
fn detect_failsafe_on_lost_input() {
    let inp = RcFailsafeSchedulerInputs {
        has_valid_input: false,
        ..RcFailsafeSchedulerInputs::default()
    };
    assert!(detect_rc_failsafe(&inp));
}

#[test]
fn detect_failsafe_on_low_throttle() {
    let inp = RcFailsafeSchedulerInputs {
        has_valid_input: true,
        throttle_pwm: Some(900),
        failsafe_cfg: RcFailsafeConfig {
            throttle_failsafe_enabled: true,
            throttle_failsafe_pwm: 975,
        },
        ..RcFailsafeSchedulerInputs::default()
    };
    assert!(detect_rc_failsafe(&inp));
}

#[test]
fn scheduler_tick_zeros_sticks_in_failsafe() {
    let out = rc_failsafe_scheduler_tick(&RcFailsafeSchedulerInputs {
        has_valid_input: false,
        roll_pwm: Some(1700),
        pitch_pwm: Some(1300),
        ..RcFailsafeSchedulerInputs::default()
    });
    assert!(out.in_rc_failsafe);
    assert_eq!(out.rc_sticks.roll_norm_dz, 0.0);
    assert_eq!(out.rc_sticks.pitch_norm_dz, 0.0);
}

#[test]
fn scheduler_tick_publishes_sticks_when_valid() {
    let cfg = RcChannelConfig::default();
    let out = rc_failsafe_scheduler_tick(&RcFailsafeSchedulerInputs {
        has_valid_input: true,
        roll_pwm: Some(1700),
        pitch_pwm: Some(1300),
        throttle_pwm: Some(1100),
        roll_cfg: cfg,
        pitch_cfg: cfg,
        flap_pwm: None,
        flap_cfg: cfg,
        failsafe_cfg: RcFailsafeConfig {
            throttle_failsafe_enabled: true,
            throttle_failsafe_pwm: 975,
        },
        ..RcFailsafeSchedulerInputs::default()
    });
    assert!(!out.in_rc_failsafe);
    assert!(out.rc_sticks.roll_norm_dz > 0.4);
    assert!(out.rc_sticks.pitch_norm_dz < -0.4);
}


#[test]
fn percent_input_maps_flap_stick_to_percent() {
    use ap_plane::rc_failsafe_scheduler_hookup::percent_input;
    let cfg = RcChannelConfig::default();
    assert_eq!(percent_input(1100, &cfg), 0);
    assert_eq!(percent_input(1900, &cfg), 100);
    assert_eq!(percent_input(1500, &cfg), 50);
}

#[test]
fn scheduler_tick_publishes_manual_flap_from_rc() {
    let cfg = RcChannelConfig::default();
    let out = rc_failsafe_scheduler_tick(&RcFailsafeSchedulerInputs {
        has_valid_input: true,
        roll_pwm: Some(1500),
        pitch_pwm: Some(1500),
        throttle_pwm: Some(1100),
        flap_pwm: Some(1660),
        flap_cfg: cfg,
        roll_cfg: cfg,
        pitch_cfg: cfg,
        failsafe_cfg: RcFailsafeConfig {
            throttle_failsafe_enabled: true,
            throttle_failsafe_pwm: 975,
        },
        ..RcFailsafeSchedulerInputs::default()
    });
    assert_eq!(out.manual_flap_percent, 70);
}
