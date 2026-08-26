//! Go-around and override dispatch.

use ap_landing::go_around::{
    abort_landing_throttle_suppressed, apply_slope_abort_go_around, override_servos,
    request_go_around, LandingFlags, LandingType, SlopeLandingFlags,
};

#[test]
fn override_servos_is_false_for_slope_landings() {
    let flags = LandingFlags {
        in_progress: true,
        commanded_go_around: false,
    };
    assert!(!override_servos(&flags, LandingType::StandardGlideSlope));
}

#[test]
fn override_servos_is_false_when_not_landing() {
    let flags = LandingFlags::default();
    assert!(!override_servos(&flags, LandingType::StandardGlideSlope));
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
