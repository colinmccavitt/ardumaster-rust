//! `ModeRTL` init / run leftovers, upstream `ArduCopter/mode_rtl.cpp`.

use ap_copter::auto_yaw::YawMode;
use ap_copter::mode_rtl::{
    rtl_alt_type, rtl_descent_complete, rtl_descent_run, rtl_descent_start, rtl_init,
    rtl_is_landing, rtl_loiter_complete, rtl_loiter_yaw_aligned, rtl_mode_flags,
    rtl_restart_without_terrain, rtl_run, RtlAltType, RtlDescentView, RtlInitView, RtlRunView,
    RtlRunner, RtlSubMode, MODE_NUMBER_LAND, MODE_NUMBER_RTL, MODE_REASON_TERRAIN_FAILSAFE,
    RTL_ALT_FINAL_M_DEFAULT, RTL_ALT_MIN_M, RTL_ALT_M_DEFAULT, RTL_CLIMB_MIN_M_DEFAULT,
    RTL_DESCENT_COMPLETE_M, RTL_LOITER_TIME_MS, RTL_LOITER_YAW_ALIGN_DEG,
};
use ap_math::scalar::radians;
use ap_motors::spool::DesiredSpoolState;

#[test]
fn constants_match_upstream_defines() {
    assert_eq!(RTL_ALT_M_DEFAULT.to_bits(), 15.0f32.to_bits());
    assert_eq!(RTL_ALT_FINAL_M_DEFAULT.to_bits(), 0.0f32.to_bits());
    assert_eq!(RTL_CLIMB_MIN_M_DEFAULT.to_bits(), 0.0f32.to_bits());
    assert_eq!(RTL_ALT_MIN_M.to_bits(), 0.30f32.to_bits());
    assert_eq!(RTL_LOITER_TIME_MS, 5_000);
    assert_eq!(RTL_LOITER_YAW_ALIGN_DEG.to_bits(), 2.0f32.to_bits());
    assert_eq!(MODE_NUMBER_RTL, 6);
    assert_eq!(MODE_NUMBER_LAND, 9);
    assert_eq!(MODE_REASON_TERRAIN_FAILSAFE, 11);
    assert_eq!(RTL_DESCENT_COMPLETE_M.to_bits(), 0.2f32.to_bits());
}

#[test]
fn flags_match_mode_h() {
    let flags = rtl_mode_flags();
    assert_eq!(flags.mode_number, MODE_NUMBER_RTL);
    assert!(flags.requires_position);
    assert!(!flags.has_manual_throttle);
    assert!(!flags.allows_arming);
    assert!(flags.is_autopilot);
    assert!(flags.requires_terrain_failsafe);
}

#[test]
fn alt_type_falls_back_to_relative() {
    assert_eq!(rtl_alt_type(0), RtlAltType::Relative);
    assert_eq!(rtl_alt_type(1), RtlAltType::Terrain);
    assert_eq!(rtl_alt_type(2), RtlAltType::Relative);
    assert_eq!(rtl_alt_type(-1), RtlAltType::Relative);
}

#[test]
fn init_refuses_without_home_unless_ignore_checks() {
    let view = RtlInitView {
        home_is_set: false,
        ..RtlInitView::ready()
    };
    let refused = rtl_init(&view, false);
    assert!(!refused.ok);

    let ignored = rtl_init(&view, true);
    assert!(ignored.ok);
    assert_eq!(ignored.state, RtlSubMode::Starting);
    assert!(ignored.state_complete);
}

#[test]
fn init_starts_starting_and_clears_land_flags() {
    let mut view = RtlInitView::ready();
    view.speed_ms = 8.0;
    view.terrain_failsafe = true;
    let out = rtl_init(&view, false);
    assert!(out.ok);
    assert_eq!(out.state, RtlSubMode::Starting);
    assert!(out.state_complete);
    assert!(!out.terrain_following_allowed);
    assert!(!out.land_repo_active);
    assert!(!out.prec_land_active);
    assert_eq!(out.wp_speed_ms.to_bits(), 8.0f32.to_bits());
}

#[test]
fn disarmed_run_changes_nothing() {
    let mut view = RtlRunView::climbing();
    view.armed = false;
    view.state = RtlSubMode::Starting;
    view.state_complete = true;
    let out = rtl_run(&view);
    assert!(out.early_return_disarmed);
    assert!(!out.advanced);
    assert!(!out.built_path);
    assert_eq!(out.state, RtlSubMode::Starting);
    assert!(out.state_complete);
    assert_eq!(out.runner, None);
    assert_eq!(out.wp, None);
}

#[test]
fn starting_complete_builds_path_and_climbs() {
    let mut view = RtlRunView::climbing();
    view.state = RtlSubMode::Starting;
    view.state_complete = true;
    let out = rtl_run(&view);
    assert!(out.advanced);
    assert!(out.built_path);
    assert_eq!(out.state, RtlSubMode::InitialClimb);
    assert!(!out.state_complete);
    assert_eq!(out.yaw, Some(YawMode::Hold));
    assert_eq!(out.runner, Some(RtlRunner::ClimbReturn));
    let wp = out.wp.expect("climb runner");
    assert!(!wp.safe_ground);
    assert_eq!(wp.desired_spool, Some(DesiredSpoolState::ThrottleUnlimited));
    assert!(wp.update_wpnav);
    assert!(wp.update_d);
}

#[test]
fn climb_complete_returns_home() {
    let mut view = RtlRunView::climbing();
    view.state = RtlSubMode::InitialClimb;
    view.state_complete = true;
    view.default_yaw = YawMode::LookAtNextWp;
    let out = rtl_run(&view);
    assert!(out.advanced);
    assert!(!out.built_path);
    assert_eq!(out.state, RtlSubMode::ReturnHome);
    assert_eq!(out.yaw, Some(YawMode::LookAtNextWp));
    assert_eq!(out.runner, Some(RtlRunner::ClimbReturn));
}

#[test]
fn return_complete_loiters() {
    let mut view = RtlRunView::climbing();
    view.state = RtlSubMode::ReturnHome;
    view.state_complete = true;
    view.now_ms = 12_000;
    view.default_yaw = YawMode::LookAtNextWp;
    let out = rtl_run(&view);
    assert_eq!(out.state, RtlSubMode::LoiterAtHome);
    assert_eq!(out.yaw, Some(YawMode::ResetToArmedYaw));
    assert_eq!(out.loiter_start_ms, Some(12_000));
    assert_eq!(out.runner, Some(RtlRunner::LoiterAtHome));
    assert!(!out.state_complete);
}

#[test]
fn loiter_complete_lands_when_path_says_land() {
    let mut view = RtlRunView::climbing();
    view.state = RtlSubMode::LoiterAtHome;
    view.state_complete = true;
    view.path_land = true;
    let out = rtl_run(&view);
    assert_eq!(out.state, RtlSubMode::Land);
    assert_eq!(out.yaw, Some(YawMode::Hold));
    assert_eq!(
        out.runner,
        Some(RtlRunner::Land {
            disarm_on_land: true
        })
    );
    assert!(rtl_is_landing(out.state));
}

#[test]
fn loiter_complete_descends_when_final_alt_is_above_zero() {
    let mut view = RtlRunView::climbing();
    view.state = RtlSubMode::LoiterAtHome;
    view.state_complete = true;
    view.path_land = false;
    view.radio_failsafe = false;
    let out = rtl_run(&view);
    assert_eq!(out.state, RtlSubMode::FinalDescent);
    assert_eq!(out.runner, Some(RtlRunner::FinalDescent));
    assert!(!rtl_is_landing(out.state));
}

#[test]
fn radio_failsafe_lands_even_when_final_alt_is_set() {
    let mut view = RtlRunView::climbing();
    view.state = RtlSubMode::LoiterAtHome;
    view.state_complete = true;
    view.path_land = false;
    view.radio_failsafe = true;
    let out = rtl_run(&view);
    assert_eq!(out.state, RtlSubMode::Land);
}

#[test]
fn climb_dest_fail_asks_land_terrain_failsafe() {
    let mut view = RtlRunView::climbing();
    view.state = RtlSubMode::Starting;
    view.state_complete = true;
    view.climb_dest_ok = false;
    let out = rtl_run(&view);
    assert!(out.dest_failed);
    assert!(out.switch_to_land);
    assert_eq!(out.state, RtlSubMode::InitialClimb);
    assert_eq!(MODE_NUMBER_LAND, 9);
    assert_eq!(MODE_REASON_TERRAIN_FAILSAFE, 11);
}

#[test]
fn return_dest_fail_restarts_and_fallthrough_climbs() {
    let mut view = RtlRunView::climbing();
    view.state = RtlSubMode::InitialClimb;
    view.state_complete = true;
    view.return_dest_ok = false;
    let out = rtl_run(&view);
    assert!(out.dest_failed);
    assert!(out.restart_without_terrain);
    assert_eq!(out.terrain_following_allowed, Some(false));
    // restart leaves STARTING; the same tick's FALLTHROUGH climbs.
    assert_eq!(out.state, RtlSubMode::InitialClimb);
    assert_eq!(out.runner, Some(RtlRunner::ClimbReturn));
}

#[test]
fn climb_return_sets_complete_from_reached_wp() {
    let mut view = RtlRunView::climbing();
    view.reached_wp = true;
    let out = rtl_run(&view);
    assert!(out.state_complete);

    view.reached_wp = false;
    let early = rtl_run(&view);
    assert!(!early.state_complete);
}

#[test]
fn climb_return_ground_path_skips_wp_and_keeps_incomplete() {
    let mut view = RtlRunView::climbing();
    view.auto_armed = false;
    view.reached_wp = true;
    let out = rtl_run(&view);
    let wp = out.wp.expect("ground runner");
    assert!(wp.safe_ground);
    assert_eq!(wp.desired_spool, None);
    assert!(!wp.update_wpnav);
    assert!(!wp.update_d);
    assert!(!out.state_complete);
}

#[test]
fn loiter_timer_and_armed_yaw_gate() {
    assert!(!rtl_loiter_complete(
        4_999,
        0,
        RTL_LOITER_TIME_MS,
        YawMode::Hold,
        0.0,
        0.0,
    ));
    assert!(rtl_loiter_complete(
        5_000,
        0,
        RTL_LOITER_TIME_MS,
        YawMode::Hold,
        0.0,
        0.0,
    ));

    let aligned = 0.0;
    let off = radians(2.1);
    assert!(rtl_loiter_yaw_aligned(aligned, 0.0));
    assert!(!rtl_loiter_yaw_aligned(off, 0.0));
    assert!(!rtl_loiter_complete(
        5_000,
        0,
        RTL_LOITER_TIME_MS,
        YawMode::ResetToArmedYaw,
        off,
        0.0,
    ));
    assert!(rtl_loiter_complete(
        5_000,
        0,
        RTL_LOITER_TIME_MS,
        YawMode::ResetToArmedYaw,
        aligned,
        0.0,
    ));
}

#[test]
fn loiter_timer_wraps_like_uint32() {
    assert!(rtl_loiter_complete(
        89,
        u32::MAX - 10,
        100,
        YawMode::Hold,
        0.0,
        0.0,
    ));
    assert!(!rtl_loiter_complete(
        88,
        u32::MAX - 10,
        100,
        YawMode::Hold,
        0.0,
        0.0,
    ));
}

#[test]
fn submode_numbers_are_declaration_order() {
    assert_eq!(RtlSubMode::Starting.as_number(), 0);
    assert_eq!(RtlSubMode::InitialClimb.as_number(), 1);
    assert_eq!(RtlSubMode::ReturnHome.as_number(), 2);
    assert_eq!(RtlSubMode::LoiterAtHome.as_number(), 3);
    assert_eq!(RtlSubMode::FinalDescent.as_number(), 4);
    assert_eq!(RtlSubMode::Land.as_number(), 5);
}

#[test]
fn restart_without_terrain_parks_starting_complete() {
    let out = rtl_restart_without_terrain();
    assert!(!out.terrain_following_allowed);
    assert_eq!(out.state, RtlSubMode::Starting);
    assert!(out.state_complete);
}

#[test]
fn descent_start_seeds_d_at_the_stopping_point() {
    let out = rtl_descent_start();
    assert_eq!(out.state, RtlSubMode::FinalDescent);
    assert!(!out.state_complete);
    assert!(out.d_init_stopping_point);
    assert_eq!(out.yaw, YawMode::Hold);
}

#[test]
fn descent_complete_is_a_twenty_centimetre_window() {
    assert!(rtl_descent_complete(10.0, 10.0));
    assert!(rtl_descent_complete(10.0, 10.19));
    assert!(rtl_descent_complete(10.0, 9.81));
    // The gate is strict `<`, not `<=`. Compare against the constant
    // itself so the boundary is a representable difference.
    assert!(!rtl_descent_complete(0.0, RTL_DESCENT_COMPLETE_M));
    assert!(!rtl_descent_complete(10.0, 10.3));
    assert!(!rtl_descent_complete(10.0, 9.7));
}

#[test]
fn descent_run_flies_slew_and_is_not_done_far_from_target() {
    let out = rtl_descent_run(&RtlDescentView::descending());
    assert!(!out.safe_ground);
    assert!(!out.cancel_escape);
    assert!(!out.land_repo_active);
    assert_eq!(
        out.desired_spool,
        Some(DesiredSpoolState::ThrottleUnlimited)
    );
    assert!(out.input_vel_ne);
    assert!(out.d_slew);
    assert!(!out.state_complete);
}

#[test]
fn descent_run_completes_inside_the_window() {
    let mut view = RtlDescentView::descending();
    view.pos_u_m = 10.05;
    let out = rtl_descent_run(&view);
    assert!(out.state_complete);
}

#[test]
fn descent_run_grounds_before_pilot_or_complete_check() {
    let mut view = RtlDescentView::descending();
    view.land_complete = true;
    view.pos_u_m = 10.0;
    view.throttle_behavior = 2;
    view.filtered_throttle_control_in = 800.0;
    let out = rtl_descent_run(&view);
    assert!(out.safe_ground);
    assert!(!out.cancel_escape);
    assert!(!out.state_complete);
    assert_eq!(out.desired_spool, None);
}

#[test]
fn descent_run_throttle_cancel_is_loiter_then_althold() {
    let mut view = RtlDescentView::descending();
    view.throttle_behavior = 2;
    view.filtered_throttle_control_in = 701.0;
    let out = rtl_descent_run(&view);
    assert!(out.cancel_escape);
    assert!(out.input_vel_ne);
}

#[test]
fn descent_run_repositioning_latches_repo_and_does_not_clear_it() {
    let mut view = RtlDescentView::descending();
    view.land_repositioning = true;
    view.pilot_velocity_is_zero = false;
    let out = rtl_descent_run(&view);
    assert!(out.land_repo_active);

    view.pilot_velocity_is_zero = true;
    view.land_repo_active = true;
    let held = rtl_descent_run(&view);
    assert!(held.land_repo_active);
}
