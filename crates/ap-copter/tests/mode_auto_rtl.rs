//! `ModeAuto::rtl_start` leftover, upstream `ArduCopter/mode_auto.cpp`.

use ap_copter::mode_auto::{auto_rtl_from_init, auto_rtl_start, AutoRtlStartView, AutoSubMode};
use ap_copter::mode_rtl::{rtl_init, RtlInit, RtlInitView, RtlSubMode};

fn bits(v: f32) -> u32 {
    v.to_bits()
}

#[test]
fn ready_reuses_rtl_init_and_parks_in_rtl() {
    let out = auto_rtl_start(&AutoRtlStartView::ready());
    assert!(out.ok);
    assert!(!out.flow_of_control_error);
    assert_eq!(out.submode, Some(AutoSubMode::Rtl));
    assert!(out.rtl.ok);
    assert_eq!(out.rtl.state, RtlSubMode::Starting);
    assert!(out.rtl.state_complete);
    assert!(out.rtl.terrain_following_allowed);
    assert!(!out.rtl.land_repo_active);
    assert!(!out.rtl.prec_land_active);
    assert_eq!(bits(out.rtl.wp_speed_ms), bits(0.0));
}

#[test]
fn no_home_still_parks_because_checks_are_ignored() {
    let out = auto_rtl_start(&AutoRtlStartView::no_home());
    assert!(out.ok);
    assert!(!out.flow_of_control_error);
    assert_eq!(out.submode, Some(AutoSubMode::Rtl));
    assert!(out.rtl.ok);
    assert_eq!(out.rtl.state, RtlSubMode::Starting);
}

#[test]
fn no_home_would_refuse_rtl_init_without_ignore_checks() {
    let view = RtlInitView {
        home_is_set: false,
        terrain_failsafe: false,
        speed_ms: 0.0,
    };
    let refused = rtl_init(&view, false);
    assert!(!refused.ok);
    let accepted = rtl_init(&view, true);
    assert!(accepted.ok);
    let out = auto_rtl_start(&AutoRtlStartView::no_home());
    assert_eq!(out.rtl, accepted);
}

#[test]
fn terrain_failsafe_forbids_terrain_following() {
    let mut view = AutoRtlStartView::ready();
    view.terrain_failsafe = true;
    let out = auto_rtl_start(&view);
    assert!(out.ok);
    assert_eq!(out.submode, Some(AutoSubMode::Rtl));
    assert!(!out.rtl.terrain_following_allowed);
}

#[test]
fn speed_override_is_forwarded_to_rtl_init() {
    let view = AutoRtlStartView {
        home_is_set: true,
        terrain_failsafe: false,
        speed_ms: 7.5,
    };
    let out = auto_rtl_start(&view);
    assert!(out.ok);
    assert_eq!(bits(out.rtl.wp_speed_ms), bits(7.5));
}

#[test]
fn repo_and_prec_land_are_cleared() {
    let out = auto_rtl_start(&AutoRtlStartView::ready());
    assert!(!out.rtl.land_repo_active);
    assert!(!out.rtl.prec_land_active);
}

#[test]
fn rtl_init_failure_is_a_flow_of_control_error() {
    let rtl = RtlInit {
        ok: false,
        state: RtlSubMode::Starting,
        state_complete: false,
        terrain_following_allowed: false,
        land_repo_active: false,
        prec_land_active: false,
        wp_speed_ms: 0.0,
    };
    let out = auto_rtl_from_init(rtl);
    assert!(!out.ok);
    assert!(out.flow_of_control_error);
    assert_eq!(out.submode, None);
    assert_eq!(out.rtl, rtl);
}

#[test]
fn from_init_success_parks_in_rtl() {
    let rtl = rtl_init(&RtlInitView::ready(), true);
    let out = auto_rtl_from_init(rtl);
    assert!(out.ok);
    assert!(!out.flow_of_control_error);
    assert_eq!(out.submode, Some(AutoSubMode::Rtl));
    assert_eq!(out.rtl, rtl);
}
