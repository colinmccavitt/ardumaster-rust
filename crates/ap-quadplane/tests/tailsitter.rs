//! Tailsitter enable / input-type stub — upstream `Tailsitter::enabled`,
//! `_is_vectored`, `is_control_surface_tailsitter`.

use ap_quadplane::tailsitter::{
    InputType, Tailsitter, TailsitterConfig, MOTOR_FRAME_TAILSITTER, TAILSIT_ENABLE_DEFAULT,
    VECTORED_HOVER_GAIN_DEFAULT,
};

#[test]
fn tailsitter_frame_auto_enables_when_unconfigured() {
    let ts = Tailsitter::setup(TailsitterConfig::tailsitter_frame());
    assert_eq!(ts.enable(), 1);
    assert!(ts.enabled());
}

#[test]
fn non_tailsitter_frame_stays_disabled_when_unconfigured() {
    let ts = Tailsitter::setup(TailsitterConfig::new());
    assert_eq!(ts.enable(), TAILSIT_ENABLE_DEFAULT);
    assert!(!ts.enabled());
    assert_eq!(ts.input_type(), None);
}

#[test]
fn explicit_disable_wins_even_on_a_tailsitter_frame() {
    let mut cfg = TailsitterConfig::tailsitter_frame();
    cfg.enable = Some(0);
    let ts = Tailsitter::setup(cfg);
    assert_eq!(ts.enable(), 0);
    assert!(!ts.enabled());
    assert_eq!(ts.input_type(), None);
}

#[test]
fn copter_tailsitter_motor_mask_auto_enables() {
    let mut cfg = TailsitterConfig::new();
    cfg.frame_class = 1; // MOTOR_FRAME_QUAD
    cfg.motor_mask = 0b1111;
    let ts = Tailsitter::setup(cfg);
    assert!(ts.enabled());
    // Duo-motor input types require Q_FRAME_CLASS = TAILSITTER.
    assert_eq!(ts.input_type(), None);
}

#[test]
fn zero_hover_gain_is_control_surfaces() {
    let mut cfg = TailsitterConfig::tailsitter_frame();
    cfg.vectored_hover_gain = 0.0;
    let ts = Tailsitter::setup(cfg);
    assert!(ts.enabled());
    assert!(!ts.is_vectored());
    assert!(ts.is_control_surface_tailsitter());
    assert_eq!(ts.input_type(), Some(InputType::ControlSurfaces));
}

#[test]
fn default_gain_without_tilt_motors_is_control_surfaces() {
    // Default VHGAIN is 0.5, but without a tilt servo the left-motor check
    // makes this a control-surface tailsitter.
    let ts = Tailsitter::setup(TailsitterConfig::tailsitter_frame());
    assert_eq!(ts.input_type(), Some(InputType::ControlSurfaces));
    assert!(!ts.is_vectored());
}

#[test]
fn tilt_motors_and_gain_are_vectored_yaw() {
    let mut cfg = TailsitterConfig::tailsitter_frame();
    cfg.vectored_hover_gain = VECTORED_HOVER_GAIN_DEFAULT;
    cfg.tilt_motor_left = true;
    cfg.tilt_motor_right = true;
    let ts = Tailsitter::setup(cfg);
    assert!(ts.is_vectored());
    assert!(!ts.is_control_surface_tailsitter());
    assert_eq!(ts.input_type(), Some(InputType::VectoredYaw));
}

#[test]
fn right_tilt_only_is_vectored_and_also_control_surfaces() {
    // Upstream `_is_vectored` accepts left OR right; control-surface
    // checks only the left function. Both predicates are true.
    let mut cfg = TailsitterConfig::tailsitter_frame();
    cfg.tilt_motor_right = true;
    let ts = Tailsitter::setup(cfg);
    assert!(ts.is_vectored());
    assert!(ts.is_control_surface_tailsitter());
    assert_eq!(ts.input_type(), Some(InputType::VectoredYaw));
}

#[test]
fn motor_frame_tailsitter_constant_matches_upstream() {
    assert_eq!(MOTOR_FRAME_TAILSITTER, 10);
}
