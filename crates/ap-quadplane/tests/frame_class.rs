//! QuadPlane tilt / tailsitter-frame setup hook — upstream
//! `QuadPlane::setup` `Q_FRAME_CLASS` switch and the
//! tailsitter + tiltrotor config error.
//!
//! Frame class selection only: tailsitter / tiltrotor / multicopter.
//! Does not rewrite ap-motors mixing or `tailsitter.rs`.

use ap_quadplane::tailsitter::{MOTOR_FRAME_TAILSITTER, TAILSIT_ENABLE_DEFAULT};
use ap_quadplane::{
    FrameSetup, MotorFrameClass, MotorsKind, QuadPlane, VtolAirframe, Q_FRAME_CLASS_DEFAULT,
    Q_FRAME_TYPE_DEFAULT, Q_TILT_ENABLE_DEFAULT,
};

#[test]
fn defaults_are_quad_x_multicopter() {
    let qp = QuadPlane::new();
    assert_eq!(qp.frame_class(), Q_FRAME_CLASS_DEFAULT);
    assert_eq!(qp.frame_class(), MotorFrameClass::Quad as u8);
    assert_eq!(qp.frame_type(), Q_FRAME_TYPE_DEFAULT);
    assert_eq!(qp.tailsit_enable(), TAILSIT_ENABLE_DEFAULT);
    assert_eq!(qp.tilt_enable(), Q_TILT_ENABLE_DEFAULT);
    assert!(qp.motors_kind().is_none());
    assert!(qp.vtol_airframe().is_none());
    assert_eq!(
        QuadPlane::classify_frame(qp.frame_class(), qp.tailsit_enable(), qp.tilt_enable()),
        Some(FrameSetup {
            airframe: VtolAirframe::Multicopter,
            motors_kind: MotorsKind::Matrix,
        })
    );
}

#[test]
fn motor_frame_class_matches_upstream_and_tailsitter_const() {
    assert_eq!(MotorFrameClass::Quad as u8, 1);
    assert_eq!(MotorFrameClass::Hexa as u8, 2);
    assert_eq!(MotorFrameClass::Tri as u8, 7);
    assert_eq!(MotorFrameClass::Tailsitter as u8, 10);
    assert_eq!(MotorFrameClass::Tailsitter as u8, MOTOR_FRAME_TAILSITTER);
    assert_eq!(MotorFrameClass::Deca as u8, 14);
    assert_eq!(
        MotorFrameClass::from_u8(10),
        Some(MotorFrameClass::Tailsitter)
    );
    assert!(MotorFrameClass::from_u8(99).is_none());
}

#[test]
fn tailsitter_frame_class_selects_tailsitter_motors() {
    let mut qp = QuadPlane::with_enable(1);
    qp.set_frame_class(MotorFrameClass::Tailsitter as u8);
    assert!(qp.setup());
    assert_eq!(qp.motors_kind(), Some(MotorsKind::Tailsitter));
    assert_eq!(qp.vtol_airframe(), Some(VtolAirframe::Tailsitter));
}

#[test]
fn tilt_enable_on_quad_is_tiltrotor_with_matrix_motors() {
    let mut qp = QuadPlane::with_enable(1);
    qp.set_tilt_enable(1);
    assert!(qp.setup());
    assert_eq!(qp.motors_kind(), Some(MotorsKind::Matrix));
    assert_eq!(qp.vtol_airframe(), Some(VtolAirframe::Tiltrotor));
}

#[test]
fn tailsit_enable_on_quad_keeps_matrix_motors() {
    // Upstream allocates motors from Q_FRAME_CLASS; Q_TAILSIT_ENABLE
    // marks the airframe without switching to AP_MotorsTailsitter.
    let mut qp = QuadPlane::with_enable(1);
    qp.set_tailsit_enable(1);
    assert!(qp.setup());
    assert_eq!(qp.motors_kind(), Some(MotorsKind::Matrix));
    assert_eq!(qp.vtol_airframe(), Some(VtolAirframe::Tailsitter));
}

#[test]
fn tri_frame_class_selects_tri_motors() {
    let mut qp = QuadPlane::with_enable(1);
    qp.set_frame_class(MotorFrameClass::Tri as u8);
    assert!(qp.setup());
    assert_eq!(qp.motors_kind(), Some(MotorsKind::Tri));
    assert_eq!(qp.vtol_airframe(), Some(VtolAirframe::Multicopter));
}

#[test]
fn hexa_is_multicopter_matrix() {
    let sel = QuadPlane::classify_frame(MotorFrameClass::Hexa as u8, 0, 0);
    assert_eq!(
        sel,
        Some(FrameSetup {
            airframe: VtolAirframe::Multicopter,
            motors_kind: MotorsKind::Matrix,
        })
    );
}

#[test]
fn unsupported_heli_frame_class_fails_setup() {
    let mut qp = QuadPlane::with_enable(1);
    qp.set_frame_class(MotorFrameClass::Heli as u8);
    assert!(!QuadPlane::frame_class_supported(qp.frame_class()));
    assert!(QuadPlane::classify_frame(qp.frame_class(), 0, 0).is_none());
    assert!(!qp.setup());
    assert!(!qp.motors_inited());
    assert!(!qp.available());
}

#[test]
fn tailsitter_plus_tiltrotor_is_config_error() {
    assert!(QuadPlane::classify_frame(MotorFrameClass::Tailsitter as u8, 0, 1).is_none());
    assert!(QuadPlane::classify_frame(MotorFrameClass::Quad as u8, 1, 1).is_none());

    let mut qp = QuadPlane::with_enable(1);
    qp.set_tailsit_enable(1);
    qp.set_tilt_enable(1);
    assert!(!qp.setup());
    assert!(!qp.motors_inited());
    assert!(!qp.available());
}

#[test]
fn classify_does_not_require_q_enable() {
    // Selection is a parameter table; setup still gates on Q_ENABLE.
    let sel = QuadPlane::classify_frame(MotorFrameClass::Tailsitter as u8, 0, 0);
    assert_eq!(sel.unwrap().airframe, VtolAirframe::Tailsitter);
    let mut qp = QuadPlane::new();
    qp.set_frame_class(MotorFrameClass::Tailsitter as u8);
    assert!(!qp.setup());
    assert!(qp.motors_kind().is_none());
}

#[test]
fn setup_default_quad_still_inits_matrix() {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    assert_eq!(qp.motors_kind(), Some(MotorsKind::Matrix));
    assert_eq!(qp.vtol_airframe(), Some(VtolAirframe::Multicopter));
    assert!(qp.available());
}
