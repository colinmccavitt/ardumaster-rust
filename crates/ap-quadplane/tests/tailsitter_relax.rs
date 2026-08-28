//! Tailsitter `relax_pitch` leftover — hover-gain `_is_vectored`
//! keeps pitch tight on vectored belly-sitters.

use ap_quadplane::tailsitter::{Tailsitter, TailsitterConfig, VECTORED_HOVER_GAIN_DEFAULT};

fn vectored() -> Tailsitter {
    let mut cfg = TailsitterConfig::tailsitter_frame();
    cfg.vectored_hover_gain = VECTORED_HOVER_GAIN_DEFAULT;
    cfg.tilt_motor_left = true;
    Tailsitter::setup(cfg)
}

fn control_surface() -> Tailsitter {
    Tailsitter::setup(TailsitterConfig::tailsitter_frame())
}

#[test]
fn disabled_always_relaxes() {
    let ts = Tailsitter::setup(TailsitterConfig::new());
    assert!(!ts.enabled());
    assert!(ts.relax_pitch(0));
}

#[test]
fn control_surface_relaxes() {
    // Default VHGAIN is 0.5 but no tilt servo → not vectored.
    let ts = control_surface();
    assert!(ts.enabled());
    assert!(!ts.is_vectored());
    assert!(ts.relax_pitch(0));
}

#[test]
fn vectored_holds_pitch_until_vtol_limit() {
    let ts = vectored();
    assert!(ts.is_vectored());
    assert!(!ts.relax_pitch(0));
    assert!(ts.relax_pitch(1));
}

#[test]
fn zero_hover_gain_is_not_vectored_so_relaxes() {
    let mut cfg = TailsitterConfig::tailsitter_frame();
    cfg.vectored_hover_gain = 0.0;
    cfg.tilt_motor_left = true;
    let ts = Tailsitter::setup(cfg);
    assert!(!ts.is_vectored());
    assert!(ts.relax_pitch(0));
}
