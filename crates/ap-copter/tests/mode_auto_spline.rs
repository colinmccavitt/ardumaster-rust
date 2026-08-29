//! `ModeAuto::do_spline_wp` leftover, upstream `ArduCopter/mode_auto.cpp`.
//!
//! C++ has no `spline_start`. This is the spline destination leftover.

use ap_copter::mode_auto::{
    auto_spline_from_cmd, auto_spline_start, AutoSplineFromCmdView, AutoSplineStartView,
    AutoSubMode,
};

#[test]
fn ready_sets_spline_dest_and_parks_in_wp() {
    let out = auto_spline_start(&AutoSplineStartView::ready());
    assert!(out.ok);
    assert!(!out.default_from_wp_dest);
    assert!(!out.dest_loc_flow_of_control);
    assert!(!out.terrain_failsafe);
    assert!(out.spline.ok);
    assert!(!out.spline.next_dest_is_dest);
    assert!(out.spline.next_dest_is_spline);
    assert!(out.set_spline_destination);
    assert!(out.loiter_time_cleared);
    assert_eq!(out.loiter_time_max_s, 0);
    assert!(out.set_next_wp);
    assert!(out.yaw_set_default);
    assert_eq!(out.submode, Some(AutoSubMode::Wp));
}

#[test]
fn last_segment_copies_dest_onto_next() {
    let mut view = AutoSplineStartView::ready();
    view.spline = AutoSplineFromCmdView::last_segment();
    let out = auto_spline_start(&view);
    assert!(out.ok);
    assert!(out.spline.next_dest_is_dest);
    assert!(!out.spline.next_dest_is_spline);
    assert_eq!(out.submode, Some(AutoSubMode::Wp));
}

#[test]
fn delay_stops_the_curve_and_latches_loiter_max() {
    let mut view = AutoSplineStartView::ready();
    view.spline = AutoSplineFromCmdView::with_delay(8);
    let out = auto_spline_start(&view);
    assert!(out.ok);
    assert!(out.spline.next_dest_is_dest);
    assert!(!out.spline.next_dest_is_spline);
    assert!(out.loiter_time_cleared);
    assert_eq!(out.loiter_time_max_s, 8);
    assert_eq!(out.submode, Some(AutoSubMode::Wp));
}

#[test]
fn delay_ignores_a_following_spline_flag() {
    let view = AutoSplineFromCmdView::with_delay(3);
    assert!(view.next_nav_cmd);
    assert!(view.next_is_spline_waypoint);
    let out = auto_spline_from_cmd(&view);
    assert!(out.ok);
    assert!(out.next_dest_is_dest);
    assert!(!out.next_dest_is_spline);
}

#[test]
fn next_straight_wp_is_not_a_spline_control() {
    let mut view = AutoSplineStartView::ready();
    view.spline = AutoSplineFromCmdView::through_to_wp();
    let out = auto_spline_start(&view);
    assert!(out.ok);
    assert!(!out.spline.next_dest_is_dest);
    assert!(!out.spline.next_dest_is_spline);
}

#[test]
fn dest_refuse_is_terrain_failsafe_before_yaw() {
    let out = auto_spline_start(&AutoSplineStartView::dest_refused());
    assert!(!out.ok);
    assert!(out.terrain_failsafe);
    assert!(!out.spline.ok);
    assert!(!out.set_spline_destination);
    assert!(!out.loiter_time_cleared);
    assert_eq!(out.loiter_time_max_s, 0);
    assert!(!out.set_next_wp);
    assert!(!out.yaw_set_default);
    assert_eq!(out.submode, None);
}

#[test]
fn next_loc_refuse_is_terrain_failsafe() {
    let mut view = AutoSplineStartView::ready();
    view.spline.next_loc_ok = false;
    let out = auto_spline_start(&view);
    assert!(!out.ok);
    assert!(out.terrain_failsafe);
    assert!(!out.spline.ok);
    assert!(!out.set_spline_destination);
    assert_eq!(out.submode, None);
}

#[test]
fn spline_dest_refuse_keeps_from_cmd_and_skips_loiter() {
    let out = auto_spline_start(&AutoSplineStartView::dest_set_refused());
    assert!(!out.ok);
    assert!(out.terrain_failsafe);
    assert!(out.spline.ok);
    assert!(out.spline.next_dest_is_spline);
    assert!(!out.set_spline_destination);
    assert!(!out.loiter_time_cleared);
    assert!(!out.set_next_wp);
    assert!(!out.yaw_set_default);
    assert_eq!(out.submode, None);
}

#[test]
fn next_wp_refuse_keeps_dest_and_loiter_and_skips_yaw() {
    let mut view = AutoSplineStartView::next_wp_refused();
    view.spline.delay_s = 5;
    let out = auto_spline_start(&view);
    assert!(!out.ok);
    assert!(out.terrain_failsafe);
    assert!(out.set_spline_destination);
    assert!(out.loiter_time_cleared);
    assert_eq!(out.loiter_time_max_s, 5);
    assert!(!out.set_next_wp);
    assert!(!out.yaw_set_default);
    assert_eq!(out.submode, None);
}

#[test]
fn reached_wp_uses_wpnav_dest_as_default() {
    let out = auto_spline_start(&AutoSplineStartView::from_reached_wp());
    assert!(out.ok);
    assert!(out.default_from_wp_dest);
    assert!(!out.dest_loc_flow_of_control);
    assert_eq!(out.submode, Some(AutoSubMode::Wp));
}

#[test]
fn reached_wp_without_a_dest_loc_is_flow_of_control_and_continues() {
    let mut view = AutoSplineStartView::from_reached_wp();
    view.wp_dest_loc_ok = false;
    let out = auto_spline_start(&view);
    assert!(out.ok);
    assert!(!out.default_from_wp_dest);
    assert!(out.dest_loc_flow_of_control);
    assert!(!out.terrain_failsafe);
    assert_eq!(out.submode, Some(AutoSubMode::Wp));
}

#[test]
fn active_but_not_reached_keeps_current_loc() {
    let mut view = AutoSplineStartView::ready();
    view.wp_nav_active = true;
    view.reached_wp_destination = false;
    view.wp_dest_loc_ok = true;
    let out = auto_spline_start(&view);
    assert!(out.ok);
    assert!(!out.default_from_wp_dest);
    assert!(!out.dest_loc_flow_of_control);
}

#[test]
fn not_active_ignores_a_stale_reached_flag() {
    let mut view = AutoSplineStartView::ready();
    view.wp_nav_active = false;
    view.reached_wp_destination = true;
    view.wp_dest_loc_ok = true;
    let out = auto_spline_start(&view);
    assert!(!out.default_from_wp_dest);
    assert!(!out.dest_loc_flow_of_control);
}

#[test]
fn roi_yaw_is_left_alone() {
    let mut view = AutoSplineStartView::ready();
    view.auto_yaw_is_roi = true;
    let out = auto_spline_start(&view);
    assert!(out.ok);
    assert!(!out.yaw_set_default);
    assert_eq!(out.submode, Some(AutoSubMode::Wp));
}

#[test]
fn fixed_yaw_with_none_behavior_is_left_alone() {
    let mut view = AutoSplineStartView::ready();
    view.auto_yaw_is_fixed = true;
    view.wp_yaw_behavior_none = true;
    let out = auto_spline_start(&view);
    assert!(out.ok);
    assert!(!out.yaw_set_default);
}

#[test]
fn fixed_yaw_with_any_other_behavior_resets_to_default() {
    let mut view = AutoSplineStartView::ready();
    view.auto_yaw_is_fixed = true;
    view.wp_yaw_behavior_none = false;
    let out = auto_spline_start(&view);
    assert!(out.yaw_set_default);
}

#[test]
fn none_behavior_without_fixed_yaw_still_resets() {
    let mut view = AutoSplineStartView::ready();
    view.wp_yaw_behavior_none = true;
    let out = auto_spline_start(&view);
    assert!(out.yaw_set_default);
}

#[test]
fn from_cmd_dest_refuse_does_not_look_at_next() {
    let out = auto_spline_from_cmd(&AutoSplineFromCmdView::dest_refused());
    assert!(!out.ok);
    assert!(!out.next_dest_is_dest);
    assert!(!out.next_dest_is_spline);
}

#[test]
fn from_cmd_last_segment_stops_the_curve() {
    let out = auto_spline_from_cmd(&AutoSplineFromCmdView::last_segment());
    assert!(out.ok);
    assert!(out.next_dest_is_dest);
    assert!(!out.next_dest_is_spline);
}
