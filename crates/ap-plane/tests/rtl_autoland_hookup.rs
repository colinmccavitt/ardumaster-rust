//! RTL_AUTOLAND hookup: RTL home loiter -> AUTO landing / return-path.

use ap_plane::mode_table::{BuildFeatures, ModeNumber};
use ap_plane::rtl_autoland_hookup::{
    rtl_autoland_tick, RtlAutolandInputs, MODE_REASON_RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND,
    RTL_AUTOLAND_ALT_ERROR_CM, RTL_AUTOLAND_DISABLE, RTL_AUTOLAND_DO_RETURN_PATH_START,
    RTL_AUTOLAND_IMMEDIATE_DO_LAND_START, RTL_AUTOLAND_NO_RTL_GO_AROUND,
    RTL_AUTOLAND_THEN_DO_LAND_START,
};

fn rtl_inp() -> RtlAutolandInputs {
    RtlAutolandInputs {
        control_mode: ModeNumber::Rtl.as_number(),
        features: BuildFeatures::default(),
        rtl_autoland: RTL_AUTOLAND_IMMEDIATE_DO_LAND_START,
        checked_for_autoland: false,
        have_position: true,
        have_landing_sequence: true,
        have_return_path: false,
        reached_loiter_target: false,
        alt_error_cm: 0,
    }
}

#[test]
fn rtl_autoland_immediate_switches_to_auto() {
    let out = rtl_autoland_tick(&rtl_inp());
    assert!(out.applied);
    assert!(out.switch_to_auto);
    assert!(out.force_resume);
    assert!(out.jump_landing_sequence);
    assert!(!out.jump_return_path);
    assert!(out.checked_for_autoland);
    assert_eq!(
        out.mode_reason,
        MODE_REASON_RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND
    );
}

#[test]
fn rtl_autoland_immediate_needs_position() {
    let mut inp = rtl_inp();
    inp.have_position = false;
    let out = rtl_autoland_tick(&inp);
    assert!(out.applied);
    assert!(!out.switch_to_auto);
    assert!(!out.force_resume);
    assert!(out.checked_for_autoland);
}

#[test]
fn rtl_autoland_then_waits_until_loiter_and_alt() {
    let mut inp = rtl_inp();
    inp.rtl_autoland = RTL_AUTOLAND_THEN_DO_LAND_START;
    inp.reached_loiter_target = true;
    inp.alt_error_cm = RTL_AUTOLAND_ALT_ERROR_CM;
    let out = rtl_autoland_tick(&inp);
    assert!(out.applied);
    assert!(!out.switch_to_auto);
    assert!(!out.checked_for_autoland);

    inp.alt_error_cm = RTL_AUTOLAND_ALT_ERROR_CM - 1;
    let out = rtl_autoland_tick(&inp);
    assert!(out.switch_to_auto);
    assert!(out.jump_landing_sequence);
}

#[test]
fn rtl_autoland_return_path_jumps_closest_leg() {
    let mut inp = rtl_inp();
    inp.rtl_autoland = RTL_AUTOLAND_DO_RETURN_PATH_START;
    inp.have_landing_sequence = false;
    inp.have_return_path = true;
    let out = rtl_autoland_tick(&inp);
    assert!(out.switch_to_auto);
    assert!(out.jump_return_path);
    assert!(!out.jump_landing_sequence);
}

#[test]
fn rtl_autoland_disable_is_idle() {
    let mut inp = rtl_inp();
    inp.rtl_autoland = RTL_AUTOLAND_DISABLE;
    let out = rtl_autoland_tick(&inp);
    assert!(out.applied);
    assert!(!out.switch_to_auto);
    assert!(!out.checked_for_autoland);
    assert_eq!(out.mode_reason, 0);
}

#[test]
fn rtl_autoland_go_around_only_does_not_change_rtl() {
    let mut inp = rtl_inp();
    inp.rtl_autoland = RTL_AUTOLAND_NO_RTL_GO_AROUND;
    let out = rtl_autoland_tick(&inp);
    assert!(out.applied);
    assert!(!out.switch_to_auto);
    assert!(!out.checked_for_autoland);
}

#[test]
fn rtl_autoland_skips_loiter() {
    let mut inp = rtl_inp();
    inp.control_mode = ModeNumber::Loiter.as_number();
    let out = rtl_autoland_tick(&inp);
    assert!(!out.applied);
    assert!(!out.switch_to_auto);
}
