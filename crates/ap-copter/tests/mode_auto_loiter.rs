//! `ModeAuto` do_loiter_* leftovers, upstream `ArduCopter/mode_auto.cpp`.
//!
//! `do_loiter_unlimited` is dest then `wp_start`. `do_loiter_time` and
//! `do_loiter_to_alt` reuse that leftover.

use ap_copter::mode_auto::{
    auto_loiter_time, auto_loiter_to_alt, auto_loiter_unlimited, AutoLoiterTimeView,
    AutoLoiterToAltView, AutoLoiterUnlimitedView, AutoSubMode,
};

fn bits(v: f32) -> u32 {
    v.to_bits()
}

#[test]
fn unlimited_ready_calls_wp_start_and_parks_in_wp() {
    let out = auto_loiter_unlimited(&AutoLoiterUnlimitedView::ready());
    assert!(out.ok);
    assert!(!out.default_from_wp_dest);
    assert!(!out.dest_loc_flow_of_control);
    assert!(!out.terrain_failsafe);
    assert!(out.wp.ok);
    assert!(out.wp.wp_and_spline_init);
    assert!(out.wp.set_wp_destination);
    assert!(out.wp.yaw_set_default);
    assert_eq!(out.submode, Some(AutoSubMode::Wp));
}

#[test]
fn unlimited_reached_wp_uses_wpnav_dest_as_default() {
    let out = auto_loiter_unlimited(&AutoLoiterUnlimitedView::from_reached_wp());
    assert!(out.ok);
    assert!(out.default_from_wp_dest);
    assert!(!out.dest_loc_flow_of_control);
    assert!(!out.wp.wp_and_spline_init);
    assert_eq!(out.submode, Some(AutoSubMode::Wp));
}

#[test]
fn unlimited_reached_wp_without_a_dest_loc_is_flow_of_control_and_continues() {
    let mut view = AutoLoiterUnlimitedView::from_reached_wp();
    view.wp_dest_loc_ok = false;
    let out = auto_loiter_unlimited(&view);
    assert!(out.ok);
    assert!(!out.default_from_wp_dest);
    assert!(out.dest_loc_flow_of_control);
    assert!(!out.terrain_failsafe);
    assert_eq!(out.submode, Some(AutoSubMode::Wp));
}

#[test]
fn unlimited_active_but_not_reached_keeps_current_loc() {
    let mut view = AutoLoiterUnlimitedView::ready();
    view.wp_nav_active = true;
    view.reached_wp_destination = false;
    view.wp_dest_loc_ok = true;
    view.wp.wp_nav_active = true;
    let out = auto_loiter_unlimited(&view);
    assert!(out.ok);
    assert!(!out.default_from_wp_dest);
    assert!(!out.dest_loc_flow_of_control);
}

#[test]
fn unlimited_not_active_ignores_a_stale_reached_flag() {
    let mut view = AutoLoiterUnlimitedView::ready();
    view.wp_nav_active = false;
    view.reached_wp_destination = true;
    view.wp_dest_loc_ok = true;
    let out = auto_loiter_unlimited(&view);
    assert!(!out.default_from_wp_dest);
    assert!(!out.dest_loc_flow_of_control);
}

#[test]
fn unlimited_dest_refuse_is_terrain_failsafe_before_wp_start() {
    let out = auto_loiter_unlimited(&AutoLoiterUnlimitedView::dest_refused());
    assert!(!out.ok);
    assert!(out.terrain_failsafe);
    assert!(!out.wp.ok);
    assert!(!out.wp.wp_and_spline_init);
    assert!(!out.wp.set_wp_destination);
    assert!(!out.wp.yaw_set_default);
    assert_eq!(out.submode, None);
}

#[test]
fn unlimited_wp_refuse_keeps_init_side_effects_and_is_terrain() {
    let mut view = AutoLoiterUnlimitedView::wp_refused();
    view.wp.desired_speed_override_xy_ms = 4.0;
    view.wp.desired_speed_override_up_ms = 2.0;
    view.wp.desired_speed_override_down_ms = 1.0;
    let out = auto_loiter_unlimited(&view);
    assert!(!out.ok);
    assert!(out.terrain_failsafe);
    assert!(!out.wp.ok);
    assert!(out.wp.wp_and_spline_init);
    assert_eq!(bits(out.wp.init_speed_xy_ms), bits(4.0));
    assert!(out.wp.set_speed_up);
    assert!(out.wp.set_speed_down);
    assert!(!out.wp.set_wp_destination);
    assert!(!out.wp.yaw_set_default);
    assert_eq!(out.submode, None);
}

#[test]
fn unlimited_roi_yaw_is_left_alone() {
    let mut view = AutoLoiterUnlimitedView::ready();
    view.wp.auto_yaw_is_roi = true;
    let out = auto_loiter_unlimited(&view);
    assert!(out.ok);
    assert!(!out.wp.yaw_set_default);
    assert_eq!(out.submode, Some(AutoSubMode::Wp));
}

#[test]
fn time_ready_latches_the_delay_after_unlimited() {
    let out = auto_loiter_time(&AutoLoiterTimeView::ready());
    assert!(out.unlimited.ok);
    assert!(out.loiter_time_cleared);
    assert_eq!(out.loiter_time_max_s, 10);
    assert_eq!(out.unlimited.submode, Some(AutoSubMode::Wp));
}

#[test]
fn time_zero_delay_still_clears_the_timer() {
    let mut view = AutoLoiterTimeView::ready();
    view.delay_s = 0;
    let out = auto_loiter_time(&view);
    assert!(out.unlimited.ok);
    assert!(out.loiter_time_cleared);
    assert_eq!(out.loiter_time_max_s, 0);
}

#[test]
fn time_dest_refuse_skips_the_timer() {
    let mut view = AutoLoiterTimeView::ready();
    view.unlimited = AutoLoiterUnlimitedView::dest_refused();
    let out = auto_loiter_time(&view);
    assert!(!out.unlimited.ok);
    assert!(out.unlimited.terrain_failsafe);
    assert!(!out.loiter_time_cleared);
    assert_eq!(out.loiter_time_max_s, 0);
}

#[test]
fn time_wp_refuse_skips_the_timer() {
    let mut view = AutoLoiterTimeView::ready();
    view.unlimited = AutoLoiterUnlimitedView::wp_refused();
    view.delay_s = 8;
    let out = auto_loiter_time(&view);
    assert!(!out.unlimited.ok);
    assert!(out.unlimited.wp.wp_and_spline_init);
    assert!(!out.loiter_time_cleared);
    assert_eq!(out.loiter_time_max_s, 0);
}

#[test]
fn to_alt_ready_sets_d_limits_and_parks_in_loiter_to_alt() {
    let out = auto_loiter_to_alt(&AutoLoiterToAltView::ready());
    assert!(out.unlimited.ok);
    assert!(!out.used_current_lat_lng);
    assert!(!out.bad_alt);
    assert!(!out.reached_destination_xy);
    assert!(!out.loiter_start_done);
    assert!(!out.reached_alt);
    assert_eq!(bits(out.alt_error_m), bits(0.0));
    assert_eq!(bits(out.d_speed_down_ms), bits(1.5));
    assert_eq!(bits(out.d_speed_up_ms), bits(2.5));
    assert_eq!(bits(out.d_accel_mss), bits(2.5));
    assert!(out.d_limits_set);
    assert_eq!(out.submode, Some(AutoSubMode::LoiterToAlt));
}

#[test]
fn to_alt_zero_lat_lng_copies_current_before_the_alt_read() {
    let out = auto_loiter_to_alt(&AutoLoiterToAltView::current_lat_lng());
    assert!(out.unlimited.ok);
    assert!(out.used_current_lat_lng);
    assert!(!out.bad_alt);
    assert_eq!(out.submode, Some(AutoSubMode::LoiterToAlt));
}

#[test]
fn to_alt_bad_alt_marks_both_reached_and_leaves_wp() {
    let out = auto_loiter_to_alt(&AutoLoiterToAltView::bad_alt());
    assert!(out.unlimited.ok);
    assert!(out.bad_alt);
    assert!(out.reached_destination_xy);
    assert!(out.reached_alt);
    assert!(!out.loiter_start_done);
    assert!(!out.d_limits_set);
    assert_eq!(out.submode, Some(AutoSubMode::Wp));
}

#[test]
fn to_alt_bad_alt_with_zero_lat_lng_still_copies_current() {
    let mut view = AutoLoiterToAltView::bad_alt();
    view.lat_lng_zero = true;
    let out = auto_loiter_to_alt(&view);
    assert!(out.used_current_lat_lng);
    assert!(out.bad_alt);
    assert_eq!(out.submode, Some(AutoSubMode::Wp));
}

#[test]
fn to_alt_unlimited_refuse_touches_nothing() {
    let mut view = AutoLoiterToAltView::ready();
    view.unlimited = AutoLoiterUnlimitedView::dest_refused();
    view.lat_lng_zero = true;
    let out = auto_loiter_to_alt(&view);
    assert!(!out.unlimited.ok);
    assert!(out.unlimited.terrain_failsafe);
    assert!(!out.used_current_lat_lng);
    assert!(!out.bad_alt);
    assert!(!out.reached_destination_xy);
    assert!(!out.reached_alt);
    assert!(!out.d_limits_set);
    assert_eq!(out.submode, None);
}

#[test]
fn to_alt_wpnav_limits_are_forwarded_unchanged() {
    let view = AutoLoiterToAltView {
        unlimited: AutoLoiterUnlimitedView::ready(),
        lat_lng_zero: false,
        alt_ok: true,
        speed_down_ms: 0.75,
        speed_up_ms: 4.0,
        accel_d_mss: 1.25,
    };
    let out = auto_loiter_to_alt(&view);
    assert!(out.d_limits_set);
    assert_eq!(bits(out.d_speed_down_ms), bits(0.75));
    assert_eq!(bits(out.d_speed_up_ms), bits(4.0));
    assert_eq!(bits(out.d_accel_mss), bits(1.25));
    assert_eq!(out.submode, Some(AutoSubMode::LoiterToAlt));
}
