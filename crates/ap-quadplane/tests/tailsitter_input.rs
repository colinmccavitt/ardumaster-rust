//! Tailsitter `Q_TAILSIT_INPUT` stick remapping — upstream
//! `Tailsitter::check_input` PlaneMode vs BodyFrameRoll.

use ap_quadplane::tailsitter::{
    TailsitInput, Tailsitter, TailsitterConfig, TAILSITTER_INPUT_BF_ROLL, TAILSITTER_INPUT_PLANE,
    TAILSIT_INPUT_DEFAULT,
};

fn enabled_with_input(input: i8) -> Tailsitter {
    let mut cfg = TailsitterConfig::tailsitter_frame();
    cfg.input = input;
    Tailsitter::setup(cfg)
}

#[test]
fn groupinfo_default_is_zero() {
    assert_eq!(TAILSIT_INPUT_DEFAULT, 0);
    let ts = Tailsitter::setup(TailsitterConfig::tailsitter_frame());
    assert_eq!(ts.input(), TAILSIT_INPUT_DEFAULT);
    assert_eq!(ts.tailsit_input(), TailsitInput::Multicopters);
    assert!(!ts.plane_mode());
    assert!(!ts.body_frame_roll());
}

#[test]
fn bitmask_constants_match_upstream() {
    assert_eq!(TAILSITTER_INPUT_PLANE, 1 << 0);
    assert_eq!(TAILSITTER_INPUT_BF_ROLL, 1 << 1);
}

#[test]
fn plane_mode_bit_is_plane_mode() {
    let ts = enabled_with_input(TAILSITTER_INPUT_PLANE as i8);
    assert!(ts.plane_mode());
    assert!(!ts.body_frame_roll());
    assert_eq!(ts.tailsit_input(), TailsitInput::PlaneMode);
}

#[test]
fn body_frame_roll_bit_is_body_frame_roll() {
    let ts = enabled_with_input(TAILSITTER_INPUT_BF_ROLL as i8);
    assert!(!ts.plane_mode());
    assert!(ts.body_frame_roll());
    assert_eq!(ts.tailsit_input(), TailsitInput::BodyFrameRoll);
}

#[test]
fn both_bits_are_plane_mode_body_frame_roll() {
    let ts = enabled_with_input((TAILSITTER_INPUT_PLANE | TAILSITTER_INPUT_BF_ROLL) as i8);
    assert!(ts.plane_mode());
    assert!(ts.body_frame_roll());
    assert_eq!(ts.tailsit_input(), TailsitInput::PlaneModeBodyFrameRoll);
}

#[test]
fn default_input_does_not_remap_when_active() {
    let ts = enabled_with_input(TAILSIT_INPUT_DEFAULT);
    assert_eq!(ts.check_input(1000, 400, true, false), (1000, 400));
}

#[test]
fn body_frame_roll_alone_does_not_swap_sticks() {
    // BodyFrameRoll is an attitude-controller path, not a control_in swap.
    let ts = enabled_with_input(TAILSITTER_INPUT_BF_ROLL as i8);
    assert_eq!(ts.check_input(1000, 400, true, false), (1000, 400));
}

#[test]
fn plane_mode_swaps_roll_and_yaw_when_hovering() {
    let ts = enabled_with_input(TAILSITTER_INPUT_PLANE as i8);
    assert_eq!(ts.check_input(1000, 400, true, false), (400, -1000));
}

#[test]
fn plane_mode_swaps_during_fw_transition() {
    // active() is also true in ANGLE_WAIT_FW.
    let ts = enabled_with_input(TAILSITTER_INPUT_PLANE as i8);
    assert!(ts.active(false, true));
    assert_eq!(ts.check_input(800, -200, false, true), (-200, -800));
}

#[test]
fn plane_mode_does_not_remap_when_inactive() {
    let ts = enabled_with_input(TAILSITTER_INPUT_PLANE as i8);
    assert!(!ts.active(false, false));
    assert_eq!(ts.check_input(1000, 400, false, false), (1000, 400));
}

#[test]
fn disabled_tailsitter_is_never_active() {
    let mut cfg = TailsitterConfig::tailsitter_frame();
    cfg.enable = Some(0);
    cfg.input = TAILSITTER_INPUT_PLANE as i8;
    let ts = Tailsitter::setup(cfg);
    assert!(!ts.enabled());
    assert!(!ts.active(true, true));
    assert_eq!(ts.check_input(1000, 400, true, true), (1000, 400));
}

#[test]
fn both_bits_still_swap_in_check_input() {
    // check_input only looks at PlaneMode; BodyFrameRoll does not cancel it.
    let ts = enabled_with_input((TAILSITTER_INPUT_PLANE | TAILSITTER_INPUT_BF_ROLL) as i8);
    assert_eq!(ts.check_input(1200, -300, true, false), (-300, -1200));
}
