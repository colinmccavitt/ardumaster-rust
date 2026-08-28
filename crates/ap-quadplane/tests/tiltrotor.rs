//! Tiltrotor enable / type and tilt-angle slew stub — upstream
//! `Tiltrotor::enabled`, `Q_TILT_TYPE`, `current_tilt` / `slew`,
//! `Q_TILT_RATE_UP` / `Q_TILT_RATE_DN`.

use ap_quadplane::tiltrotor::{
    TiltType, Tiltrotor, TiltrotorConfig, TILT_ENABLE_DEFAULT, TILT_MASK_DEFAULT,
    TILT_RATE_DN_DPS_DEFAULT, TILT_RATE_UP_DPS_DEFAULT, TILT_TYPE_DEFAULT,
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
