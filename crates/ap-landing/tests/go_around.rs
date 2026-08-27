//! Go-around and override dispatch.

use ap_landing::deepstall_stage::DeepstallStage;
use ap_landing::go_around::{
    abort_landing_throttle_suppressed, apply_slope_abort_go_around, deepstall_request_go_around,
    override_servos, request_go_around, LandingFlags, LandingType, SlopeLandingFlags,
};

#[test]
fn override_servos_is_false_for_slope_landings() {
    let flags = LandingFlags {
        in_progress: true,
        commanded_go_around: false,
    };
    assert!(!override_servos(
        &flags,
        LandingType::StandardGlideSlope,
        None,
    ));
}

#[test]
fn override_servos_is_false_when_not_landing() {
    let flags = LandingFlags::default();
    assert!(!override_servos(
        &flags,
        LandingType::StandardGlideSlope,
        None,
    ));
}

#[test]
fn override_servos_is_true_in_deepstall_land() {
    let flags = LandingFlags {
        in_progress: true,
        commanded_go_around: false,
    };
    assert!(override_servos(
        &flags,
        LandingType::Deepstall,
        Some(DeepstallStage::Land),
    ));
    assert!(!override_servos(
        &flags,
        LandingType::Deepstall,
        Some(DeepstallStage::Approach),
    ));
}

#[test]
fn deepstall_go_around_respects_min_abort_alt() {
    let mut flags = LandingFlags {
        in_progress: true,
        commanded_go_around: false,
    };
    assert!(!deepstall_request_go_around(&mut flags, 10.0, 5.0));
    assert!(!flags.commanded_go_around);
    assert!(deepstall_request_go_around(&mut flags, 10.0, 15.0));
    assert!(flags.commanded_go_around);
}

#[test]
fn request_go_around_sets_the_flag() {
    let mut flags = LandingFlags {
        in_progress: true,
        commanded_go_around: false,
    };
    assert!(request_go_around(&mut flags));
    assert!(flags.commanded_go_around);
}

#[test]
fn slope_abort_latches_once_and_records_offset() {
    let mut landing = LandingFlags::default();
    let mut slope = SlopeLandingFlags::default();
    assert!(apply_slope_abort_go_around(&mut landing, &mut slope, -2.5));
    assert!(landing.commanded_go_around);
    assert!(slope.has_aborted_due_to_slope_recalc);
    assert!((slope.alt_offset + 2.5).abs() < 1e-6);
}

#[test]
fn abort_landing_unsuppresses_throttle() {
    assert!(!abort_landing_throttle_suppressed());
}
