//! `ModeAuto::wp_start` leftover, upstream `ArduCopter/mode_auto.cpp`.

use ap_copter::mode_auto::{auto_wp_start, AutoSubMode, AutoWpStartView};

fn bits(v: f32) -> u32 {
    v.to_bits()
}

#[test]
fn idle_loiter_inits_wpnav_and_parks_in_wp() {
    let out = auto_wp_start(&AutoWpStartView::idle_loiter());
    assert!(out.ok);
    assert!(out.wp_and_spline_init);
    assert_eq!(bits(out.init_speed_xy_ms), bits(0.0));
    assert!(!out.stopping_point_from_takeoff);
    assert!(!out.set_speed_up);
    assert!(!out.set_speed_down);
    assert!(out.set_wp_destination);
    assert!(out.yaw_set_default);
    assert_eq!(out.submode, Some(AutoSubMode::Wp));
}

#[test]
fn takeoff_completion_pos_is_the_stopping_point() {
    let out = auto_wp_start(&AutoWpStartView::from_takeoff());
    assert!(out.ok);
    assert!(out.wp_and_spline_init);
    assert!(out.stopping_point_from_takeoff);
    assert_eq!(out.submode, Some(AutoSubMode::Wp));
}

#[test]
fn takeoff_without_a_completion_pos_still_inits() {
    let mut view = AutoWpStartView::from_takeoff();
    view.takeoff_completion_pos = false;
    let out = auto_wp_start(&view);
    assert!(out.ok);
    assert!(out.wp_and_spline_init);
    assert!(!out.stopping_point_from_takeoff);
}

#[test]
fn idle_non_takeoff_ignores_a_stale_completion_pos() {
    let mut view = AutoWpStartView::idle_loiter();
    view.takeoff_completion_pos = true;
    let out = auto_wp_start(&view);
    assert!(out.wp_and_spline_init);
    assert!(!out.stopping_point_from_takeoff);
}

#[test]
fn active_wpnav_skips_init() {
    let mut view = AutoWpStartView::idle_loiter();
    view.wp_nav_active = true;
    view.desired_speed_override_xy_ms = 5.0;
    view.desired_speed_override_up_ms = 2.0;
    view.desired_speed_override_down_ms = 1.5;
    view.takeoff_completion_pos = true;
    view.submode = AutoSubMode::Takeoff;
    let out = auto_wp_start(&view);
    assert!(out.ok);
    assert!(!out.wp_and_spline_init);
    assert_eq!(bits(out.init_speed_xy_ms), bits(0.0));
    assert!(!out.stopping_point_from_takeoff);
    assert!(!out.set_speed_up);
    assert!(!out.set_speed_down);
    assert!(out.set_wp_destination);
    assert_eq!(out.submode, Some(AutoSubMode::Wp));
}

#[test]
fn dest_refuse_keeps_init_side_effects_and_skips_yaw() {
    let mut view = AutoWpStartView::dest_refused();
    view.desired_speed_override_xy_ms = 4.0;
    view.desired_speed_override_up_ms = 2.0;
    view.desired_speed_override_down_ms = 1.0;
    let out = auto_wp_start(&view);
    assert!(!out.ok);
    assert!(out.wp_and_spline_init);
    assert_eq!(bits(out.init_speed_xy_ms), bits(4.0));
    assert!(out.set_speed_up);
    assert!(out.set_speed_down);
    assert!(!out.set_wp_destination);
    assert!(!out.yaw_set_default);
    assert_eq!(out.submode, None);
}

#[test]
fn dest_refuse_on_active_wpnav_touches_nothing() {
    let mut view = AutoWpStartView::dest_refused();
    view.wp_nav_active = true;
    view.desired_speed_override_up_ms = 2.0;
    let out = auto_wp_start(&view);
    assert!(!out.ok);
    assert!(!out.wp_and_spline_init);
    assert!(!out.set_speed_up);
    assert!(!out.set_wp_destination);
    assert!(!out.yaw_set_default);
    assert_eq!(out.submode, None);
}

#[test]
fn roi_yaw_is_left_alone() {
    let mut view = AutoWpStartView::idle_loiter();
    view.auto_yaw_is_roi = true;
    let out = auto_wp_start(&view);
    assert!(out.ok);
    assert!(!out.yaw_set_default);
    assert_eq!(out.submode, Some(AutoSubMode::Wp));
}

#[test]
fn fixed_yaw_with_none_behavior_is_left_alone() {
    let mut view = AutoWpStartView::idle_loiter();
    view.auto_yaw_is_fixed = true;
    view.wp_yaw_behavior_none = true;
    let out = auto_wp_start(&view);
    assert!(out.ok);
    assert!(!out.yaw_set_default);
}

#[test]
fn fixed_yaw_with_any_other_behavior_resets_to_default() {
    let mut view = AutoWpStartView::idle_loiter();
    view.auto_yaw_is_fixed = true;
    view.wp_yaw_behavior_none = false;
    let out = auto_wp_start(&view);
    assert!(out.yaw_set_default);
}

#[test]
fn none_behavior_without_fixed_yaw_still_resets() {
    let mut view = AutoWpStartView::idle_loiter();
    view.wp_yaw_behavior_none = true;
    let out = auto_wp_start(&view);
    assert!(out.yaw_set_default);
}

#[test]
fn positive_speed_overrides_apply_only_on_init() {
    let mut view = AutoWpStartView::idle_loiter();
    view.desired_speed_override_xy_ms = 5.0;
    view.desired_speed_override_up_ms = 2.5;
    view.desired_speed_override_down_ms = 1.25;
    let out = auto_wp_start(&view);
    assert!(out.wp_and_spline_init);
    assert_eq!(bits(out.init_speed_xy_ms), bits(5.0));
    assert!(out.set_speed_up);
    assert!(out.set_speed_down);
}

#[test]
fn zero_and_negative_overrides_are_unset() {
    let mut view = AutoWpStartView::idle_loiter();
    view.desired_speed_override_xy_ms = 0.0;
    view.desired_speed_override_up_ms = 0.0;
    view.desired_speed_override_down_ms = -1.0;
    let out = auto_wp_start(&view);
    assert_eq!(bits(out.init_speed_xy_ms), bits(0.0));
    assert!(!out.set_speed_up);
    assert!(!out.set_speed_down);
}
