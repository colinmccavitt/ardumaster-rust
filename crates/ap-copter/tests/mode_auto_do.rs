//! Remaining `ModeAuto` `do_*` / `verify_*` leftovers, upstream
//! `ArduCopter/mode_auto.cpp`.

use ap_copter::auto_yaw::{FixedYawDirection, RoiAction};
use ap_copter::mode_auto::{
    auto_change_speed, auto_circle, auto_do_roi, auto_do_yaw, auto_nav_delay, auto_payload_place,
    auto_set_home, auto_verify_circle, auto_verify_command, auto_verify_land,
    auto_verify_loiter_time, auto_verify_loiter_to_alt, auto_verify_loiter_unlimited,
    auto_verify_nav_delay, auto_verify_nav_guided_enable, auto_verify_nav_script_time,
    auto_verify_nav_wp, auto_verify_payload_place, auto_verify_rtl, auto_verify_spline_wp,
    auto_verify_takeoff,
    auto_verify_wait_delay, auto_verify_within_distance, auto_verify_yaw, auto_wait_delay,
    auto_within_distance, circle_radius_m, loiter_turns, AutoChangeSpeedView, AutoCircleView,
    AutoCircleYaw, AutoDoRoiView, AutoDoYawView, AutoLandState, AutoNavDelayView,
    AutoPayloadPlaceView, AutoSetHomeKind, AutoSetHomeView, AutoSpeedAxis, AutoStartFeatures,
    AutoSubMode, AutoVerifyCircleView, AutoVerifyCommand, AutoVerifyHandler, AutoVerifyLandView,
    AutoVerifyNavDelayView, AutoVerifyNavGuidedView, AutoVerifyNavScriptTimeView,
    AutoVerifyRtlView, AutoVerifyWaitDelayView, AutoVerifyWithinDistanceView, AutoVerifyWpTimerView,
    AutoVerifyYawView, AutoWaitDelayView, AutoWithinDistanceView, AutoWpStartView, PayloadPlaceState,
    MAV_CMD_CONDITION_DELAY, MAV_CMD_CONDITION_DISTANCE, MAV_CMD_CONDITION_YAW,
    MAV_CMD_DO_CHANGE_SPEED, MAV_CMD_DO_GUIDED_LIMITS, MAV_CMD_DO_LAND_START,
    MAV_CMD_DO_MOUNT_CONTROL, MAV_CMD_DO_SET_HOME, MAV_CMD_DO_SET_ROI, MAV_CMD_DO_WINCH,
    MAV_CMD_NAV_DELAY, MAV_CMD_NAV_GUIDED_ENABLE, MAV_CMD_NAV_LAND, MAV_CMD_NAV_LOITER_TIME,
    MAV_CMD_NAV_LOITER_TO_ALT, MAV_CMD_NAV_LOITER_TURNS, MAV_CMD_NAV_LOITER_UNLIM,
    MAV_CMD_NAV_PAYLOAD_PLACE, MAV_CMD_NAV_RETURN_TO_LAUNCH, MAV_CMD_NAV_SPLINE_WAYPOINT,
    MAV_CMD_NAV_TAKEOFF, MAV_CMD_NAV_WAYPOINT, SPEED_TYPE_AIRSPEED, SPEED_TYPE_CLIMB_SPEED,
    SPEED_TYPE_DESCENT_SPEED,
};
use ap_copter::mode_rtl::RtlSubMode;

fn bits(v: f32) -> u32 {
    v.to_bits()
}

fn verify(cmd_id: u16) -> AutoVerifyCommand {
    auto_verify_command(true, cmd_id, AutoStartFeatures::none())
}

fn verify_all(cmd_id: u16) -> AutoVerifyCommand {
    auto_verify_command(true, cmd_id, AutoStartFeatures::all())
}

#[test]
fn circle_ready_flies_to_the_edge_and_resets_last_complete() {
    let out = auto_circle(&AutoCircleView::ready_to_edge());
    assert!(out.ok);
    assert!(!out.terrain_failsafe);
    assert_eq!(out.radius_m, 20);
    assert_eq!(bits(out.rate_degs), bits(20.0));
    assert!(out.move_to_edge);
    assert!(out.set_wp_destination);
    assert_eq!(out.yaw, AutoCircleYaw::Default);
    assert_eq!(out.submode, Some(AutoSubMode::CircleMoveToEdge));
    assert_eq!(out.last_num_complete.map(bits), Some(bits(-1.0)));
}

#[test]
fn circle_inside_the_radius_holds_yaw_while_flying_to_the_edge() {
    let mut view = AutoCircleView::ready_to_edge();
    view.dist_to_center_m = 4.0;
    let out = auto_circle(&view);
    assert!(out.move_to_edge);
    assert_eq!(out.yaw, AutoCircleYaw::Hold);
}

#[test]
fn circle_roi_leaves_yaw_alone_on_the_edge_fly() {
    let mut view = AutoCircleView::ready_to_edge();
    view.auto_yaw_is_roi = true;
    let out = auto_circle(&view);
    assert_eq!(out.yaw, AutoCircleYaw::Unchanged);
    assert_eq!(out.submode, Some(AutoSubMode::CircleMoveToEdge));
}

#[test]
fn circle_edge_dest_refuse_is_terrain_failsafe_but_still_parks() {
    let mut view = AutoCircleView::ready_to_edge();
    view.dest_accepted = false;
    let out = auto_circle(&view);
    assert!(out.ok);
    assert!(out.terrain_failsafe);
    assert!(out.move_to_edge);
    assert_eq!(out.submode, Some(AutoSubMode::CircleMoveToEdge));
    assert_eq!(out.last_num_complete.map(bits), Some(bits(-1.0)));
}

#[test]
fn circle_already_on_edge_calls_circle_start() {
    let out = auto_circle(&AutoCircleView::already_on_edge());
    assert!(out.ok);
    assert!(!out.move_to_edge);
    assert!(!out.set_wp_destination);
    assert_eq!(out.yaw, AutoCircleYaw::Circle);
    assert_eq!(out.submode, Some(AutoSubMode::Circle));
    assert_eq!(out.last_num_complete.map(bits), Some(bits(-1.0)));
}

#[test]
fn circle_already_on_edge_with_roi_keeps_yaw() {
    let mut view = AutoCircleView::already_on_edge();
    view.auto_yaw_is_roi = true;
    let out = auto_circle(&view);
    assert_eq!(out.yaw, AutoCircleYaw::Unchanged);
    assert_eq!(out.submode, Some(AutoSubMode::Circle));
}

#[test]
fn circle_exactly_3m_is_already_on_the_edge() {
    let mut view = AutoCircleView::ready_to_edge();
    view.dist_to_edge_m = 3.0;
    let out = auto_circle(&view);
    assert!(!out.move_to_edge);
    assert_eq!(out.submode, Some(AutoSubMode::Circle));
}

#[test]
fn circle_dest_refuse_is_terrain_failsafe_before_radius() {
    let out = auto_circle(&AutoCircleView::dest_refused());
    assert!(!out.ok);
    assert!(out.terrain_failsafe);
    assert_eq!(out.radius_m, 0);
    assert_eq!(out.submode, None);
    assert_eq!(out.last_num_complete, None);
}

#[test]
fn circle_ccw_negates_the_rate() {
    let mut view = AutoCircleView::ready_to_edge();
    view.loiter_ccw = true;
    view.current_rate_degs = -15.0;
    let out = auto_circle(&view);
    assert_eq!(bits(out.rate_degs), bits(-15.0));
}

#[test]
fn circle_large_radius_bit_multiplies_highbyte_by_ten() {
    assert_eq!(circle_radius_m(0x1400, true, true), 200);
    assert_eq!(circle_radius_m(0x1400, true, false), 20);
    assert_eq!(circle_radius_m(0x1400, false, true), 20);
}

#[test]
fn circle_outside_but_within_5m_of_centre_holds() {
    let mut view = AutoCircleView::ready_to_edge();
    view.p1 = 0x0300;
    view.dist_to_center_m = 4.5;
    let out = auto_circle(&view);
    assert_eq!(out.radius_m, 3);
    assert_eq!(out.yaw, AutoCircleYaw::Hold);
}

#[test]
fn do_yaw_converts_degrees_and_treats_zero_relative_as_absolute() {
    let out = auto_do_yaw(&AutoDoYawView::absolute_90());
    assert_eq!(bits(out.angle_rad), bits(90.0_f32.to_radians()));
    assert_eq!(bits(out.turn_rate_rads), bits(10.0_f32.to_radians()));
    assert_eq!(out.direction, FixedYawDirection::Shortest);
    assert!(!out.relative);
}

#[test]
fn do_yaw_relative_is_only_when_relative_angle_is_positive() {
    let out = auto_do_yaw(&AutoDoYawView::relative_45());
    assert!(out.relative);
    assert_eq!(out.direction, FixedYawDirection::Clockwise);

    let mut view = AutoDoYawView::relative_45();
    view.relative_angle = 0;
    assert!(!auto_do_yaw(&view).relative);
    view.relative_angle = -1;
    assert!(!auto_do_yaw(&view).relative);
}

#[test]
fn do_yaw_negative_direction_is_counter_clockwise() {
    let mut view = AutoDoYawView::absolute_90();
    view.direction = -1;
    assert_eq!(
        auto_do_yaw(&view).direction,
        FixedYawDirection::CounterClockwise
    );
}

#[test]
fn do_roi_forwards_to_roi_action() {
    assert_eq!(
        auto_do_roi(&AutoDoRoiView::point_airframe()).action,
        RoiAction::PointAirframe
    );
    assert_eq!(auto_do_roi(&AutoDoRoiView::cancel()).action, RoiAction::Cancel);
    let mount = AutoDoRoiView {
        location_initialised: true,
        mount_has_pan_control: true,
    };
    assert_eq!(auto_do_roi(&mount).action, RoiAction::MountOnly);
}

#[test]
fn nav_delay_relative_seconds_become_milliseconds() {
    let out = auto_nav_delay(&AutoNavDelayView::relative_5s());
    assert!(out.relative);
    assert_eq!(out.max_ms, 5000);
}

#[test]
fn nav_delay_utc_uses_rtc_when_seconds_are_not_positive() {
    let out = auto_nav_delay(&AutoNavDelayView::utc());
    assert!(!out.relative);
    assert_eq!(out.max_ms, 12_000);

    let mut view = AutoNavDelayView::utc();
    view.rtc_enabled = false;
    assert_eq!(auto_nav_delay(&view).max_ms, 0);

    view.seconds = -1;
    view.rtc_enabled = true;
    assert_eq!(auto_nav_delay(&view).max_ms, 12_000);
}

#[test]
fn wait_delay_stores_seconds_as_milliseconds() {
    let out = auto_wait_delay(&AutoWaitDelayView::three_seconds());
    assert_eq!(bits(out.condition_value), bits(3000.0));
}

#[test]
fn within_distance_stores_metres() {
    let out = auto_within_distance(&AutoWithinDistanceView::ten_metres());
    assert_eq!(bits(out.condition_value), bits(10.0));
}

#[test]
fn change_speed_non_positive_is_a_noop() {
    let view = AutoChangeSpeedView {
        target_ms: 0.0,
        speed_type: SPEED_TYPE_CLIMB_SPEED,
    };
    let out = auto_change_speed(&view);
    assert_eq!(out.axis, AutoSpeedAxis::None);
    assert_eq!(bits(out.target_ms), bits(0.0));
}

#[test]
fn change_speed_axes_match_upstream() {
    assert_eq!(
        auto_change_speed(&AutoChangeSpeedView::groundspeed()).axis,
        AutoSpeedAxis::Horizontal
    );
    let climb = AutoChangeSpeedView {
        target_ms: 2.5,
        speed_type: SPEED_TYPE_CLIMB_SPEED,
    };
    assert_eq!(auto_change_speed(&climb).axis, AutoSpeedAxis::Climb);
    let down = AutoChangeSpeedView {
        target_ms: 1.5,
        speed_type: SPEED_TYPE_DESCENT_SPEED,
    };
    assert_eq!(auto_change_speed(&down).axis, AutoSpeedAxis::Descent);
    let air = AutoChangeSpeedView {
        target_ms: 8.0,
        speed_type: SPEED_TYPE_AIRSPEED,
    };
    assert_eq!(auto_change_speed(&air).axis, AutoSpeedAxis::Horizontal);
}

#[test]
fn set_home_p1_or_uninitialised_uses_current() {
    assert_eq!(
        auto_set_home(&AutoSetHomeView::current()).kind,
        AutoSetHomeKind::Current
    );
    let uninit = AutoSetHomeView {
        p1: 0,
        location_initialised: false,
    };
    assert_eq!(auto_set_home(&uninit).kind, AutoSetHomeKind::Current);
    assert_eq!(
        auto_set_home(&AutoSetHomeView::command()).kind,
        AutoSetHomeKind::Command
    );
}

#[test]
fn payload_place_without_a_location_starts_descent() {
    let out = auto_payload_place(&AutoPayloadPlaceView::descent_here());
    assert!(out.ok);
    assert!(!out.terrain_failsafe);
    assert_eq!(out.state, Some(PayloadPlaceState::DescentStart));
    assert!(!out.wp.ok);
    assert_eq!(bits(out.descent_max_m), bits(5.0));
    assert_eq!(out.submode, Some(AutoSubMode::NavPayloadPlace));
}

#[test]
fn payload_place_fly_to_reuses_wp_start() {
    let out = auto_payload_place(&AutoPayloadPlaceView::fly_to());
    assert!(out.ok);
    assert!(out.wp.ok);
    assert_eq!(out.state, Some(PayloadPlaceState::FlyToLocation));
    assert_eq!(out.submode, Some(AutoSubMode::NavPayloadPlace));
}

#[test]
fn payload_place_fly_to_dest_refuse_is_terrain_before_descent_max() {
    let mut view = AutoPayloadPlaceView::fly_to();
    view.dest_ok = false;
    let out = auto_payload_place(&view);
    assert!(!out.ok);
    assert!(out.terrain_failsafe);
    assert_eq!(out.state, None);
    assert_eq!(bits(out.descent_max_m), bits(0.0));
    assert_eq!(out.submode, None);
}

#[test]
fn payload_place_wp_refuse_is_terrain_before_submode() {
    let mut view = AutoPayloadPlaceView::fly_to();
    view.wp = AutoWpStartView::dest_refused();
    let out = auto_payload_place(&view);
    assert!(!out.ok);
    assert!(out.terrain_failsafe);
    assert!(out.wp.wp_and_spline_init);
    assert_eq!(out.submode, None);
}

#[test]
fn verify_command_not_in_auto_does_not_run() {
    let out = auto_verify_command(false, MAV_CMD_NAV_WAYPOINT, AutoStartFeatures::all());
    assert!(!out.ran);
    assert_eq!(out.handler, AutoVerifyHandler::NotInAuto);
    assert!(!out.immediate_complete);
}

#[test]
fn verify_command_dispatch_matches_start_command() {
    assert_eq!(
        verify(MAV_CMD_NAV_TAKEOFF).handler,
        AutoVerifyHandler::VerifyTakeoff
    );
    assert_eq!(
        verify(MAV_CMD_NAV_WAYPOINT).handler,
        AutoVerifyHandler::VerifyNavWp
    );
    assert_eq!(verify(MAV_CMD_NAV_LAND).handler, AutoVerifyHandler::VerifyLand);
    assert_eq!(
        verify(MAV_CMD_NAV_LOITER_UNLIM).handler,
        AutoVerifyHandler::VerifyLoiterUnlimited
    );
    assert_eq!(
        verify(MAV_CMD_NAV_LOITER_TURNS).handler,
        AutoVerifyHandler::VerifyCircle
    );
    assert_eq!(
        verify(MAV_CMD_NAV_LOITER_TIME).handler,
        AutoVerifyHandler::VerifyLoiterTime
    );
    assert_eq!(
        verify(MAV_CMD_NAV_RETURN_TO_LAUNCH).handler,
        AutoVerifyHandler::VerifyRtl
    );
    assert_eq!(
        verify(MAV_CMD_NAV_SPLINE_WAYPOINT).handler,
        AutoVerifyHandler::VerifySplineWp
    );
    assert_eq!(
        verify(MAV_CMD_NAV_DELAY).handler,
        AutoVerifyHandler::VerifyNavDelay
    );
    assert_eq!(
        verify(MAV_CMD_CONDITION_DELAY).handler,
        AutoVerifyHandler::VerifyWaitDelay
    );
    assert_eq!(
        verify(MAV_CMD_CONDITION_DISTANCE).handler,
        AutoVerifyHandler::VerifyWithinDistance
    );
    assert_eq!(
        verify(MAV_CMD_CONDITION_YAW).handler,
        AutoVerifyHandler::VerifyYaw
    );
}

#[test]
fn verify_command_loiter_to_alt_is_the_early_return_arm() {
    let out = verify(MAV_CMD_NAV_LOITER_TO_ALT);
    assert_eq!(out.handler, AutoVerifyHandler::VerifyLoiterToAlt);
    assert!(out.early_return);
    assert!(!out.immediate_complete);
}

#[test]
fn verify_command_do_ids_complete_immediately() {
    for id in [
        MAV_CMD_DO_CHANGE_SPEED,
        MAV_CMD_DO_SET_HOME,
        MAV_CMD_DO_SET_ROI,
        MAV_CMD_DO_LAND_START,
    ] {
        let out = verify(id);
        assert_eq!(out.handler, AutoVerifyHandler::DoAlwaysComplete);
        assert!(out.immediate_complete);
        assert!(!out.skip_invalid_text);
    }
}

#[test]
fn verify_command_gated_ids_skip_when_compiled_out() {
    let out = verify(MAV_CMD_NAV_PAYLOAD_PLACE);
    assert_eq!(out.handler, AutoVerifyHandler::SkipInvalid);
    assert!(out.immediate_complete);
    assert!(out.skip_invalid_text);

    let on = verify_all(MAV_CMD_NAV_PAYLOAD_PLACE);
    assert_eq!(on.handler, AutoVerifyHandler::VerifyPayloadPlace);
    assert!(!on.immediate_complete);

    let mount = verify_all(MAV_CMD_DO_MOUNT_CONTROL);
    assert_eq!(mount.handler, AutoVerifyHandler::DoAlwaysComplete);
    assert!(mount.immediate_complete);

    let guided = verify_all(MAV_CMD_NAV_GUIDED_ENABLE);
    assert_eq!(guided.handler, AutoVerifyHandler::VerifyNavGuidedEnable);

    let winch = verify(MAV_CMD_DO_WINCH);
    assert_eq!(winch.handler, AutoVerifyHandler::SkipInvalid);
    let winch_on = verify_all(MAV_CMD_DO_WINCH);
    assert!(winch_on.immediate_complete);
    let limits = verify_all(MAV_CMD_DO_GUIDED_LIMITS);
    assert_eq!(limits.handler, AutoVerifyHandler::DoAlwaysComplete);
}

#[test]
fn verify_command_unknown_id_skips() {
    let out = verify(1);
    assert_eq!(out.handler, AutoVerifyHandler::SkipInvalid);
    assert!(out.immediate_complete);
    assert!(out.skip_invalid_text);
}

#[test]
fn verify_takeoff_is_the_complete_flag() {
    assert!(!auto_verify_takeoff(false));
    assert!(auto_verify_takeoff(true));
}

#[test]
fn verify_land_fly_to_waits_then_starts_land() {
    let waiting = auto_verify_land(&AutoVerifyLandView::flying_to());
    assert!(!waiting.complete);
    assert!(!waiting.land_start);
    assert_eq!(waiting.state, AutoLandState::FlyToLocation);

    let arrived = auto_verify_land(&AutoVerifyLandView::arrived());
    assert!(!arrived.complete);
    assert!(arrived.land_start);
    assert_eq!(arrived.state, AutoLandState::Descending);
}

#[test]
fn verify_land_descending_disarms_and_stays_on_nav_land() {
    let out = auto_verify_land(&AutoVerifyLandView::landed());
    assert!(!out.complete);
    assert!(out.disarm);

    let mut view = AutoVerifyLandView::landed();
    view.continue_after_land = true;
    let cont = auto_verify_land(&view);
    assert!(cont.complete);
    assert!(!cont.disarm);

    view.continue_after_land = false;
    view.armed = false;
    let disarmed = auto_verify_land(&view);
    assert!(disarmed.complete);
    assert!(!disarmed.disarm);
}

#[test]
fn verify_loiter_unlimited_never_completes() {
    assert!(!auto_verify_loiter_unlimited());
}

#[test]
fn verify_loiter_to_alt_needs_both_flags() {
    assert!(!auto_verify_loiter_to_alt(true, false));
    assert!(!auto_verify_loiter_to_alt(false, true));
    assert!(auto_verify_loiter_to_alt(true, true));
}

#[test]
fn verify_wp_timer_waits_for_the_dest() {
    let out = auto_verify_loiter_time(&AutoVerifyWpTimerView::en_route());
    assert!(!out.complete);
    assert!(!out.timer_started);
}

#[test]
fn verify_nav_wp_zero_delay_completes_and_notifies() {
    let out = auto_verify_nav_wp(&AutoVerifyWpTimerView::arrived_no_delay());
    assert!(out.complete);
    assert!(out.timer_started);
    assert!(out.waypoint_complete_notify);
    assert!(out.reached_text);
}

#[test]
fn verify_nav_wp_delay_notifies_on_start_not_on_complete() {
    let start = AutoVerifyWpTimerView {
        reached_wp: true,
        timer_unset: true,
        elapsed_s: 0,
        loiter_time_max: 5,
    };
    let out = auto_verify_nav_wp(&start);
    assert!(!out.complete);
    assert!(out.timer_started);
    assert!(out.waypoint_complete_notify);

    let done = auto_verify_nav_wp(&AutoVerifyWpTimerView::delay_done(5));
    assert!(done.complete);
    assert!(!done.timer_started);
    assert!(!done.waypoint_complete_notify);
    assert!(done.reached_text);
}

#[test]
fn verify_spline_wp_does_not_notify() {
    let out = auto_verify_spline_wp(&AutoVerifyWpTimerView::arrived_no_delay());
    assert!(out.complete);
    assert!(!out.waypoint_complete_notify);
}

#[test]
fn verify_rtl_needs_complete_descent_or_land_and_ground_idle() {
    assert!(auto_verify_rtl(&AutoVerifyRtlView::landed()));
    let mut view = AutoVerifyRtlView::landed();
    view.state = RtlSubMode::Land;
    assert!(auto_verify_rtl(&view));
    view.ground_idle = false;
    assert!(!auto_verify_rtl(&view));
}

#[test]
fn verify_wait_delay_is_greater_than_and_clears() {
    let waiting = auto_verify_wait_delay(&AutoVerifyWaitDelayView::waiting());
    assert!(!waiting.complete);
    assert!(!waiting.cleared);
    let done = auto_verify_wait_delay(&AutoVerifyWaitDelayView::done());
    assert!(done.complete);
    assert!(done.cleared);

    let negative = AutoVerifyWaitDelayView {
        elapsed_ms: 0,
        condition_value: -5,
    };
    // MAX(neg, 0) == 0, and 0 > 0 is false.
    assert!(!auto_verify_wait_delay(&negative).complete);
    let just = AutoVerifyWaitDelayView {
        elapsed_ms: 1,
        condition_value: -5,
    };
    assert!(auto_verify_wait_delay(&just).complete);
}

#[test]
fn verify_within_distance_is_less_than_and_clears() {
    let outside = auto_verify_within_distance(&AutoVerifyWithinDistanceView::outside());
    assert!(!outside.complete);
    let inside = auto_verify_within_distance(&AutoVerifyWithinDistanceView::inside());
    assert!(inside.complete);
    assert!(inside.cleared);

    let negative = AutoVerifyWithinDistanceView {
        wp_distance_m: 0.0,
        condition_value: -1.0,
    };
    // MAX(neg, 0) == 0, and 0 < 0 is false.
    assert!(!auto_verify_within_distance(&negative).complete);
}

#[test]
fn verify_yaw_forces_fixed_then_asks_arrival() {
    let out = auto_verify_yaw(&AutoVerifyYawView::arrived());
    assert!(out.set_fixed);
    assert!(out.complete);

    let mut view = AutoVerifyYawView::arrived();
    view.fixed_yaw_offset_rad = 0.5;
    assert!(!auto_verify_yaw(&view).complete);
}

#[test]
fn verify_circle_edge_waits_then_starts() {
    let waiting = auto_verify_circle(&AutoVerifyCircleView::moving_to_edge());
    assert!(!waiting.complete);
    assert!(!waiting.circle_start);
    assert_eq!(waiting.submode, AutoSubMode::CircleMoveToEdge);

    let mut view = AutoVerifyCircleView::moving_to_edge();
    view.reached_wp = true;
    let start = auto_verify_circle(&view);
    assert!(!start.complete);
    assert!(start.circle_start);
    assert_eq!(start.yaw, AutoCircleYaw::Circle);
    assert_eq!(start.submode, AutoSubMode::Circle);
}

#[test]
fn verify_circle_completes_when_turns_are_done() {
    let mid = auto_verify_circle(&AutoVerifyCircleView::circling());
    assert!(!mid.complete);
    assert!(mid.starting_circle_text);
    assert_eq!(bits(mid.last_num_complete), bits(1.0));

    let mut view = AutoVerifyCircleView::circling();
    view.num_circles = 2.0;
    view.last_num_complete = 1.0;
    let done = auto_verify_circle(&view);
    assert!(done.complete);
    assert!(done.starting_circle_text);
}

#[test]
fn loiter_turns_fractional_bit_divides_by_256() {
    assert_eq!(bits(loiter_turns(2, false)), bits(2.0));
    assert_eq!(bits(loiter_turns(128, true)), bits(128.0 / 256.0));
}

#[test]
fn verify_nav_delay_is_greater_than_and_clears() {
    let waiting = auto_verify_nav_delay(&AutoVerifyNavDelayView::waiting());
    assert!(!waiting.complete);
    let done = auto_verify_nav_delay(&AutoVerifyNavDelayView::done());
    assert!(done.complete);
    assert!(done.cleared);
}

#[test]
fn verify_nav_guided_p1_zero_completes() {
    assert!(auto_verify_nav_guided_enable(&AutoVerifyNavGuidedView {
        p1: 0,
        limit_check: false,
    }));
    assert!(!auto_verify_nav_guided_enable(&AutoVerifyNavGuidedView {
        p1: 1,
        limit_check: false,
    }));
    assert!(auto_verify_nav_guided_enable(&AutoVerifyNavGuidedView {
        p1: 1,
        limit_check: true,
    }));
}

#[test]
fn verify_nav_script_time_done_or_timeout() {
    assert!(auto_verify_nav_script_time(&AutoVerifyNavScriptTimeView {
        done: true,
        timeout_s: 0,
        elapsed_ms: 0,
    }));
    assert!(!auto_verify_nav_script_time(&AutoVerifyNavScriptTimeView {
        done: false,
        timeout_s: 0,
        elapsed_ms: 10_000,
    }));
    assert!(auto_verify_nav_script_time(&AutoVerifyNavScriptTimeView {
        done: false,
        timeout_s: 2,
        elapsed_ms: 2001,
    }));
}

#[test]
fn verify_payload_place_is_done_only() {
    assert!(!auto_verify_payload_place(PayloadPlaceState::FlyToLocation));
    assert!(!auto_verify_payload_place(PayloadPlaceState::Descent));
    assert!(auto_verify_payload_place(PayloadPlaceState::Done));
}
