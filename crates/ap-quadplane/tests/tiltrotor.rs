//! Tiltrotor enable / type stub — upstream `Tiltrotor::enabled`,
//! `Q_TILT_TYPE` (`CONTINUOUS` / `BINARY` / `VECTORED_YAW` / `BICOPTER`).

use ap_quadplane::tiltrotor::{
    TiltType, Tiltrotor, TiltrotorConfig, TILT_ENABLE_DEFAULT, TILT_MASK_DEFAULT,
    TILT_TYPE_DEFAULT,
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
