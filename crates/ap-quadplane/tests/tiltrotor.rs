//! Tiltrotor enable / type, tilt-angle slew, and vectored-yaw / flap
//! mix stub — upstream `Tiltrotor::enabled`, `Q_TILT_TYPE`,
//! `current_tilt` / `slew`, `Q_TILT_RATE_UP` / `Q_TILT_RATE_DN`,
//! `Tiltrotor::vectoring`, `get_forward_flight_tilt`.

use ap_quadplane::tiltrotor::{
    TiltType, Tiltrotor, TiltrotorConfig, TILT_ENABLE_DEFAULT, TILT_FIXED_ANGLE_DEG_DEFAULT,
    TILT_FIXED_GAIN_DEFAULT, TILT_FLAP_ANGLE_DEG_DEFAULT, TILT_MASK_DEFAULT,
    TILT_MAX_ANGLE_DEG_DEFAULT, TILT_RATE_DN_DPS_DEFAULT, TILT_RATE_UP_DPS_DEFAULT,
    TILT_TYPE_DEFAULT, TILT_YAW_ANGLE_DEG_DEFAULT,
};

#[test]
fn unconfigured_continuous_stays_disabled() {
    let tr = Tiltrotor::setup(TiltrotorConfig::new());
    assert_eq!(tr.enable(), TILT_ENABLE_DEFAULT);
    assert_eq!(tr.tilt_mask(), TILT_MASK_DEFAULT);
    assert_eq!(tr.tilt_type_raw(), TILT_TYPE_DEFAULT);
    assert!(!tr.enabled());
    assert_eq!(tr.tilt_type(), None);
    assert!(!tr.is_vectored());
}

#[test]
fn nonzero_tilt_mask_auto_enables_continuous() {
    let tr = Tiltrotor::setup(TiltrotorConfig::with_tilt_mask(0b0011));
    assert_eq!(tr.enable(), 1);
    assert!(tr.enabled());
    assert_eq!(tr.tilt_mask(), 0b0011);
    assert_eq!(tr.tilt_type(), Some(TiltType::Continuous));
    assert!(!tr.is_vectored());
}

#[test]
fn bicopter_type_auto_enables_without_mask() {
    let tr = Tiltrotor::setup(TiltrotorConfig::bicopter());
    assert_eq!(tr.enable(), 1);
    assert!(tr.enabled());
    assert_eq!(tr.tilt_mask(), 0);
    assert_eq!(tr.tilt_type(), Some(TiltType::Bicopter));
}

#[test]
fn explicit_disable_wins_even_with_mask() {
    let mut cfg = TiltrotorConfig::with_tilt_mask(0b1111);
    cfg.enable = Some(0);
    let tr = Tiltrotor::setup(cfg);
    assert_eq!(tr.enable(), 0);
    assert!(!tr.enabled());
    assert_eq!(tr.tilt_type(), None);
}

#[test]
fn explicit_enable_binary() {
    let cfg = TiltrotorConfig {
        enable: Some(1),
        tilt_mask: 0,
        tilt_type: TiltType::Binary as i8,
    };
    let tr = Tiltrotor::setup(cfg);
    assert!(tr.enabled());
    assert_eq!(tr.tilt_type(), Some(TiltType::Binary));
    assert!(!tr.is_vectored());
}

#[test]
fn explicit_enable_continuous() {
    let cfg = TiltrotorConfig {
        enable: Some(1),
        tilt_mask: 0,
        tilt_type: TiltType::Continuous as i8,
    };
    let tr = Tiltrotor::setup(cfg);
    assert!(tr.enabled());
    assert_eq!(tr.tilt_type(), Some(TiltType::Continuous));
}

#[test]
fn vectored_yaw_with_mask_is_vectored() {
    let cfg = TiltrotorConfig {
        enable: Some(1),
        tilt_mask: 0b0011,
        tilt_type: TiltType::VectoredYaw as i8,
    };
    let tr = Tiltrotor::setup(cfg);
    assert!(tr.enabled());
    assert_eq!(tr.tilt_type(), Some(TiltType::VectoredYaw));
    assert!(tr.is_vectored());
}

#[test]
fn vectored_yaw_without_mask_is_not_vectored() {
    let cfg = TiltrotorConfig {
        enable: Some(1),
        tilt_mask: 0,
        tilt_type: TiltType::VectoredYaw as i8,
    };
    let tr = Tiltrotor::setup(cfg);
    assert!(tr.enabled());
    assert_eq!(tr.tilt_type(), Some(TiltType::VectoredYaw));
    assert!(!tr.is_vectored());
}

#[test]
fn unknown_type_is_enabled_but_undecoded() {
    let cfg = TiltrotorConfig {
        enable: Some(1),
        tilt_mask: 0b0001,
        tilt_type: 9,
    };
    let tr = Tiltrotor::setup(cfg);
    assert!(tr.enabled());
    assert_eq!(tr.tilt_type_raw(), 9);
    assert_eq!(tr.tilt_type(), None);
}

#[test]
fn tilt_type_discriminants_match_upstream() {
    assert_eq!(TiltType::Continuous.as_i8(), 0);
    assert_eq!(TiltType::Binary.as_i8(), 1);
    assert_eq!(TiltType::VectoredYaw.as_i8(), 2);
    assert_eq!(TiltType::Bicopter.as_i8(), 3);
    assert_eq!(TiltType::from_i8(0), Some(TiltType::Continuous));
    assert_eq!(TiltType::from_i8(1), Some(TiltType::Binary));
    assert_eq!(TiltType::from_i8(2), Some(TiltType::VectoredYaw));
    assert_eq!(TiltType::from_i8(3), Some(TiltType::Bicopter));
    assert_eq!(TiltType::from_i8(4), None);
}

#[test]
fn slew_defaults_match_upstream_rates() {
    let tr = Tiltrotor::setup(TiltrotorConfig::with_tilt_mask(0b0011));
    assert_eq!(tr.max_rate_up_dps(), TILT_RATE_UP_DPS_DEFAULT);
    assert_eq!(tr.max_rate_up_dps(), 40);
    assert_eq!(tr.max_rate_down_dps(), TILT_RATE_DN_DPS_DEFAULT);
    assert_eq!(tr.max_rate_down_dps(), 0);
    assert!((tr.current_tilt() - 0.0).abs() < f32::EPSILON);
    assert!((tr.tilt_angle() - 0.0).abs() < f32::EPSILON);
    assert!(!tr.angle_achieved());
    assert!(tr.fully_up());
    assert!(!tr.fully_fwd());
}

#[test]
fn tilt_max_change_uses_rate_up_when_rate_dn_is_zero() {
    let tr = Tiltrotor::setup(TiltrotorConfig::with_tilt_mask(0b0011));
    let dt = 0.1;
    let expected = 40.0 * dt / 90.0;
    assert!((tr.tilt_max_change(false, dt) - expected).abs() < 1e-6);
    assert!((tr.tilt_max_change(true, dt) - expected).abs() < 1e-6);
}

#[test]
fn tilt_max_change_uses_rate_dn_when_nonzero() {
    let mut tr = Tiltrotor::setup(TiltrotorConfig::with_tilt_mask(0b0011));
    tr.set_max_rate_down_dps(20);
    let dt = 0.1;
    assert!((tr.tilt_max_change(false, dt) - (20.0 * dt / 90.0)).abs() < 1e-6);
    assert!((tr.tilt_max_change(true, dt) - (40.0 * dt / 90.0)).abs() < 1e-6);
}

#[test]
fn slew_toward_forward_uses_rate_up_when_rate_dn_zero() {
    let mut tr = Tiltrotor::setup(TiltrotorConfig::with_tilt_mask(0b0011));
    let dt = 0.1;
    tr.slew(1.0, dt);
    let expected = 40.0 * dt / 90.0;
    assert!((tr.current_tilt() - expected).abs() < 1e-6);
    assert!((tr.tilt_angle() - expected * 90.0).abs() < 1e-5);
    assert!(!tr.angle_achieved());
    assert!(!tr.tilt_angle_achieved());
    assert!(!tr.fully_up());
    assert!(!tr.fully_fwd());
}

#[test]
fn slew_toward_forward_uses_rate_dn_when_set() {
    let mut tr = Tiltrotor::setup(TiltrotorConfig::with_tilt_mask(0b0011));
    tr.set_max_rate_down_dps(20);
    let dt = 0.1;
    tr.slew(1.0, dt);
    assert!((tr.current_tilt() - (20.0 * dt / 90.0)).abs() < 1e-6);
    assert!(!tr.angle_achieved());
}

#[test]
fn slew_toward_hover_uses_rate_up() {
    let mut tr = Tiltrotor::setup(TiltrotorConfig::with_tilt_mask(0b0011));
    tr.set_max_rate_down_dps(10);
    tr.slew(1.0, 10.0);
    assert!((tr.current_tilt() - 1.0).abs() < 1e-6);
    assert!(tr.angle_achieved());
    assert!(tr.fully_fwd());
    assert!(!tr.fully_up());

    let dt = 0.1;
    tr.slew(0.0, dt);
    let expected = 1.0 - (40.0 * dt / 90.0);
    assert!((tr.current_tilt() - expected).abs() < 1e-6);
    assert!((tr.tilt_angle() - expected * 90.0).abs() < 1e-5);
    assert!(!tr.angle_achieved());
}

#[test]
fn slew_reaches_target_sets_angle_achieved() {
    let mut tr = Tiltrotor::setup(TiltrotorConfig::with_tilt_mask(0b0011));
    tr.slew(0.0, 0.1);
    assert!((tr.current_tilt() - 0.0).abs() < f32::EPSILON);
    assert!(tr.angle_achieved());
    assert!(tr.tilt_angle_achieved());
    assert!(tr.fully_up());

    tr.slew(1.0, 10.0);
    assert!((tr.current_tilt() - 1.0).abs() < 1e-6);
    assert!((tr.tilt_angle() - 90.0).abs() < 1e-5);
    assert!(tr.angle_achieved());
    assert!(tr.tilt_angle_achieved());
    assert!(tr.fully_fwd());
}

#[test]
fn tilt_angle_achieved_true_when_not_continuous() {
    let cfg = TiltrotorConfig {
        enable: Some(1),
        tilt_mask: 0b0011,
        tilt_type: TiltType::Binary as i8,
    };
    let mut tr = Tiltrotor::setup(cfg);
    tr.slew(1.0, 0.01);
    assert!(!tr.angle_achieved());
    assert!(tr.tilt_angle_achieved());
}

#[test]
fn tilt_angle_achieved_true_when_disabled() {
    let mut tr = Tiltrotor::setup(TiltrotorConfig::new());
    tr.slew(1.0, 0.01);
    assert!(!tr.angle_achieved());
    assert!(tr.tilt_angle_achieved());
    assert!(!tr.fully_up());
    assert!(!tr.fully_fwd());
}

#[test]
fn vectored_yaw_flap_defaults_match_upstream() {
    let tr = Tiltrotor::setup(TiltrotorConfig::with_tilt_mask(0b0011));
    assert_eq!(tr.max_angle_deg(), TILT_MAX_ANGLE_DEG_DEFAULT);
    assert_eq!(tr.max_angle_deg(), 45);
    assert!((tr.tilt_yaw_angle() - TILT_YAW_ANGLE_DEG_DEFAULT).abs() < f32::EPSILON);
    assert!((tr.fixed_angle() - TILT_FIXED_ANGLE_DEG_DEFAULT).abs() < f32::EPSILON);
    assert!((tr.fixed_gain() - TILT_FIXED_GAIN_DEFAULT).abs() < f32::EPSILON);
    assert!((tr.flap_angle_deg() - TILT_FLAP_ANGLE_DEG_DEFAULT).abs() < f32::EPSILON);
    assert!((tr.get_fully_forward_tilt() - 1.0).abs() < f32::EPSILON);
    assert!((tr.get_forward_flight_tilt(0.0) - 1.0).abs() < f32::EPSILON);
    assert!((tr.get_forward_flight_tilt(100.0) - 1.0).abs() < f32::EPSILON);
    assert!(!tr.tilt_over_max_angle(0.0));
}

#[test]
fn flap_mix_scales_forward_flight_tilt() {
    let mut tr = Tiltrotor::setup(TiltrotorConfig::with_tilt_mask(0b0011));
    tr.set_flap_angle_deg(15.0);
    assert!((tr.get_fully_forward_tilt() - (1.0 - 15.0 / 90.0)).abs() < 1e-6);
    assert!((tr.get_forward_flight_tilt(0.0) - 1.0).abs() < 1e-6);
    assert!((tr.get_forward_flight_tilt(100.0) - (1.0 - 15.0 / 90.0)).abs() < 1e-6);
    assert!((tr.get_forward_flight_tilt(50.0) - (1.0 - 15.0 / 90.0 * 0.5)).abs() < 1e-6);
}

#[test]
fn tilt_over_max_angle_uses_q_tilt_max() {
    let mut tr = Tiltrotor::setup(TiltrotorConfig::with_tilt_mask(0b0011));
    tr.set_max_rate_down_dps(90);
    assert!(!tr.tilt_over_max_angle(0.0));
    tr.slew(0.5, 1.0);
    assert!((tr.current_tilt() - 0.5).abs() < 1e-6);
    assert!(!tr.tilt_over_max_angle(0.0));
    tr.slew(0.6, 1.0);
    assert!(tr.tilt_over_max_angle(0.0));
}

#[test]
fn vectored_hover_zero_yaw_angle_is_uniform_base() {
    let cfg = TiltrotorConfig {
        enable: Some(1),
        tilt_mask: 0b0011,
        tilt_type: TiltType::VectoredYaw as i8,
    };
    let tr = Tiltrotor::setup(cfg);
    assert!(tr.is_vectored());
    assert!((tr.total_angle_deg() - 90.0).abs() < f32::EPSILON);
    assert!((tr.zero_out() - 0.0).abs() < f32::EPSILON);
    assert!((tr.base_output() - 0.0).abs() < f32::EPSILON);
    let out = tr.vectoring_hover(1.0, 0.0, 0.5, 0.5);
    assert!((out.left - 0.0).abs() < 1e-4);
    assert!((out.right - 0.0).abs() < 1e-4);
    assert!((out.rear - 0.0).abs() < 1e-4);
    assert!(!out.yaw_limited);
}

#[test]
fn vectored_hover_yaw_splits_left_right() {
    let cfg = TiltrotorConfig {
        enable: Some(1),
        tilt_mask: 0b0011,
        tilt_type: TiltType::VectoredYaw as i8,
    };
    let mut tr = Tiltrotor::setup(cfg);
    tr.set_tilt_yaw_angle(15.0);
    let total = 90.0 + 15.0;
    let zero = 15.0 / total;
    assert!((tr.total_angle_deg() - total).abs() < 1e-6);
    assert!((tr.zero_out() - zero).abs() < 1e-6);
    assert!((tr.base_output() - zero).abs() < 1e-6);

    // Hover, throttle == hover → scaler 1. yaw_out = 1, roll = 0, tilt = 0
    // → tilt_scale = 1 * 1 * 1 + 0 = 1, offset = zero.
    let out = tr.vectoring_hover(1.0, 0.0, 0.5, 0.5);
    let expected_left = (zero + zero) * 1000.0;
    let expected_right = (zero - zero) * 1000.0;
    assert!((out.left - expected_left).abs() < 1e-3);
    assert!((out.right - expected_right).abs() < 1e-3);
    assert!((out.rear - zero * 1000.0).abs() < 1e-3);
    assert!((out.rear_left - out.left).abs() < 1e-6);
    assert!((out.rear_right - out.right).abs() < 1e-6);
    assert!(out.left > out.right);
    assert!(!out.yaw_limited);
}

#[test]
fn vectored_hover_saturates_yaw_limit() {
    let cfg = TiltrotorConfig {
        enable: Some(1),
        tilt_mask: 0b0011,
        tilt_type: TiltType::VectoredYaw as i8,
    };
    let mut tr = Tiltrotor::setup(cfg);
    tr.set_tilt_yaw_angle(15.0);
    // throttle → 0 uses scaler 2; yaw 1 → tilt_scale 2, clamped to 1.
    let out = tr.vectoring_hover(1.0, 0.0, 0.0, 0.5);
    assert!(out.yaw_limited);
    let zero = 15.0 / 105.0;
    assert!((out.left - (zero + zero) * 1000.0).abs() < 1e-3);
    assert!((out.right - 0.0).abs() < 1e-3);
}

#[test]
fn vectored_fw_mix_uses_fix_gain_and_angle() {
    let cfg = TiltrotorConfig {
        enable: Some(1),
        tilt_mask: 0b0011,
        tilt_type: TiltType::VectoredYaw as i8,
    };
    let mut tr = Tiltrotor::setup(cfg);
    tr.set_tilt_yaw_angle(15.0);
    tr.set_fixed_angle(10.0);
    tr.set_fixed_gain(0.5);
    tr.slew(1.0, 10.0);
    assert!((tr.current_tilt() - 1.0).abs() < 1e-6);
    let total = 90.0 + 15.0 + 10.0;
    let zero = 15.0 / total;
    let limit = 10.0 / total;
    let level = 1.0 - limit;
    let base = zero + (1.0 * (level - zero));
    assert!((tr.base_output() - base).abs() < 1e-6);

    let out = tr.vectoring_fw(4500.0, -4500.0, 0.0, 1.0);
    let gain = 0.5 * limit * 1.0;
    let right = gain * (-4500.0) / 4500.0;
    let left = gain * 4500.0 / 4500.0;
    assert!((out.left - (base - right) * 1000.0).abs() < 1e-3);
    assert!((out.right - (base - left) * 1000.0).abs() < 1e-3);
    assert!((out.rear - base * 1000.0).abs() < 1e-3);
    assert!(!out.yaw_limited);
}
