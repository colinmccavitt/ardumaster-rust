//! `ModeAuto::init` leftover, upstream `ArduCopter/mode_auto.cpp`.

use ap_copter::mode_auto::{
    auto_init, auto_mode_flags, auto_mode_number, AutoInitView, AutoSubMode, MODE_NUMBER_AUTO,
    MODE_NUMBER_AUTO_RTL,
};

#[test]
fn numbers_match_mode_h() {
    assert_eq!(MODE_NUMBER_AUTO, 3);
    assert_eq!(MODE_NUMBER_AUTO_RTL, 27);
    assert_eq!(AutoSubMode::Loiter as u8, 7);
    assert_eq!(auto_mode_number(false), MODE_NUMBER_AUTO);
    assert_eq!(auto_mode_number(true), MODE_NUMBER_AUTO_RTL);
}

#[test]
fn flags_match_mode_h_after_init() {
    let flags = auto_mode_flags();
    assert_eq!(flags.mode_number, MODE_NUMBER_AUTO);
    assert!(flags.requires_position);
    assert!(!flags.has_manual_throttle);
    assert!(flags.is_autopilot);
    assert!(flags.allows_gcs_or_scr_arming_with_throttle_high);
    assert!(flags.requires_terrain_failsafe);
}

#[test]
fn no_mission_refuses_unless_ignore_checks() {
    let refused = auto_init(&AutoInitView::no_mission());
    assert!(!refused.ok);
    assert!(!refused.auto_rtl);
    assert!(!refused.missing_takeoff_cmd);
    assert_eq!(refused.submode, None);
    assert!(!refused.waiting_to_start);
    assert!(!refused.wp_and_spline_init);

    let mut bench = AutoInitView::no_mission();
    bench.ignore_checks = true;
    let entered = auto_init(&bench);
    assert!(entered.ok);
    assert_eq!(entered.submode, Some(AutoSubMode::Loiter));
    assert!(entered.waiting_to_start);
}

#[test]
fn landed_armed_without_takeoff_refuses() {
    let out = auto_init(&AutoInitView::landed_armed_without_takeoff());
    assert!(!out.ok);
    assert!(out.missing_takeoff_cmd);
    assert!(!out.auto_rtl);
    assert_eq!(out.submode, None);
    assert!(!out.waiting_to_start);
    assert!(!out.wp_and_spline_init);
    assert!(!out.guided_limit_clear);
}

#[test]
fn ignore_checks_does_not_skip_the_takeoff_gate() {
    let mut view = AutoInitView::landed_armed_without_takeoff();
    view.ignore_checks = true;
    let out = auto_init(&view);
    assert!(!out.ok);
    assert!(out.missing_takeoff_cmd);
}

#[test]
fn disarmed_or_airborne_or_takeoff_passes_the_second_gate() {
    let mut landed_disarmed = AutoInitView::landed_armed_without_takeoff();
    landed_disarmed.armed = false;
    assert!(auto_init(&landed_disarmed).ok);

    let mut airborne = AutoInitView::landed_armed_without_takeoff();
    airborne.land_complete = false;
    assert!(auto_init(&airborne).ok);

    let mut takeoff = AutoInitView::landed_armed_without_takeoff();
    takeoff.starts_with_takeoff_cmd = true;
    assert!(auto_init(&takeoff).ok);
}

#[test]
fn success_parks_in_loiter_and_waits() {
    let out = auto_init(&AutoInitView::airborne_with_mission());
    assert!(out.ok);
    assert!(!out.auto_rtl);
    assert!(!out.missing_takeoff_cmd);
    assert_eq!(out.submode, Some(AutoSubMode::Loiter));
    assert!(!out.hold_yaw_from_roi);
    assert!(out.wp_and_spline_init);
    assert_eq!(out.desired_speed_override_xy_ms.to_bits(), 0.0f32.to_bits());
    assert_eq!(out.desired_speed_override_up_ms.to_bits(), 0.0f32.to_bits());
    assert_eq!(
        out.desired_speed_override_down_ms.to_bits(),
        0.0f32.to_bits()
    );
    assert!(out.waiting_to_start);
    assert!(out.check_mission_change);
    assert!(out.guided_limit_clear);
    assert!(!out.land_repo_active);
}

#[test]
fn leftover_roi_yaw_is_forced_to_hold() {
    let mut view = AutoInitView::airborne_with_mission();
    view.auto_yaw_is_roi = true;
    let out = auto_init(&view);
    assert!(out.ok);
    assert!(out.hold_yaw_from_roi);
}

#[test]
fn refuse_clears_auto_rtl_and_leaves_the_rest_untouched() {
    // auto_RTL = false is the first statement, even when there is no mission.
    let out = auto_init(&AutoInitView::no_mission());
    assert!(!out.auto_rtl);
    assert_eq!(out.submode, None);
    assert!(!out.hold_yaw_from_roi);
    assert!(!out.check_mission_change);
    assert!(!out.land_repo_active);
}
