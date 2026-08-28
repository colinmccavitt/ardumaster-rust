//! `ModeAuto::start_command` and `ModeAuto::run` leftovers.

use ap_copter::mode_auto::{
    auto_run, auto_start_command, AutoMissionChangeText, AutoRunBody, AutoRunView,
    AutoStartFeatures, AutoStartHandler, AutoSubMode, MAV_CMD_CONDITION_DELAY,
    MAV_CMD_CONDITION_DISTANCE, MAV_CMD_CONDITION_YAW, MAV_CMD_DO_CHANGE_SPEED,
    MAV_CMD_DO_GUIDED_LIMITS, MAV_CMD_DO_LAND_START, MAV_CMD_DO_MOUNT_CONTROL,
    MAV_CMD_DO_RETURN_PATH_START, MAV_CMD_DO_SET_HOME, MAV_CMD_DO_SET_ROI,
    MAV_CMD_DO_SET_ROI_LOCATION, MAV_CMD_DO_SET_ROI_NONE, MAV_CMD_DO_WINCH,
    MAV_CMD_NAV_ARC_WAYPOINT, MAV_CMD_NAV_ATTITUDE_TIME, MAV_CMD_NAV_DELAY,
    MAV_CMD_NAV_GUIDED_ENABLE, MAV_CMD_NAV_LAND, MAV_CMD_NAV_LOITER_TIME,
    MAV_CMD_NAV_LOITER_TO_ALT, MAV_CMD_NAV_LOITER_TURNS, MAV_CMD_NAV_LOITER_UNLIM,
    MAV_CMD_NAV_PAYLOAD_PLACE, MAV_CMD_NAV_RETURN_TO_LAUNCH, MAV_CMD_NAV_SCRIPT_TIME,
    MAV_CMD_NAV_SPLINE_WAYPOINT, MAV_CMD_NAV_TAKEOFF, MAV_CMD_NAV_VTOL_LAND,
    MAV_CMD_NAV_VTOL_TAKEOFF, MAV_CMD_NAV_WAYPOINT,
};

fn start(cmd_id: u16) -> (bool, AutoStartHandler) {
    let out = auto_start_command(cmd_id, AutoStartFeatures::none());
    (out.accepted, out.handler)
}

fn start_all(cmd_id: u16) -> (bool, AutoStartHandler) {
    let out = auto_start_command(cmd_id, AutoStartFeatures::all());
    (out.accepted, out.handler)
}

#[test]
fn takeoff_and_vtol_takeoff_share_do_takeoff() {
    assert_eq!(
        start(MAV_CMD_NAV_TAKEOFF),
        (true, AutoStartHandler::DoTakeoff)
    );
    assert_eq!(
        start(MAV_CMD_NAV_VTOL_TAKEOFF),
        (true, AutoStartHandler::DoTakeoff)
    );
}

#[test]
fn waypoint_and_arc_share_do_nav_wp() {
    assert_eq!(
        start(MAV_CMD_NAV_WAYPOINT),
        (true, AutoStartHandler::DoNavWp)
    );
    assert_eq!(
        start(MAV_CMD_NAV_ARC_WAYPOINT),
        (true, AutoStartHandler::DoNavWp)
    );
}

#[test]
fn land_and_vtol_land_share_do_land() {
    assert_eq!(start(MAV_CMD_NAV_LAND), (true, AutoStartHandler::DoLand));
    assert_eq!(
        start(MAV_CMD_NAV_VTOL_LAND),
        (true, AutoStartHandler::DoLand)
    );
}

#[test]
fn always_on_nav_commands_are_accepted() {
    let cases = [
        (
            MAV_CMD_NAV_LOITER_UNLIM,
            AutoStartHandler::DoLoiterUnlimited,
        ),
        (MAV_CMD_NAV_LOITER_TURNS, AutoStartHandler::DoCircle),
        (MAV_CMD_NAV_LOITER_TIME, AutoStartHandler::DoLoiterTime),
        (MAV_CMD_NAV_LOITER_TO_ALT, AutoStartHandler::DoLoiterToAlt),
        (MAV_CMD_NAV_RETURN_TO_LAUNCH, AutoStartHandler::DoRtl),
        (MAV_CMD_NAV_SPLINE_WAYPOINT, AutoStartHandler::DoSplineWp),
        (MAV_CMD_NAV_DELAY, AutoStartHandler::DoNavDelay),
        (
            MAV_CMD_NAV_ATTITUDE_TIME,
            AutoStartHandler::DoNavAttitudeTime,
        ),
    ];
    for (id, handler) in cases {
        assert_eq!(start(id), (true, handler), "cmd {id}");
    }
}

#[test]
fn condition_and_do_commands_are_accepted() {
    let cases = [
        (MAV_CMD_CONDITION_DELAY, AutoStartHandler::DoWaitDelay),
        (
            MAV_CMD_CONDITION_DISTANCE,
            AutoStartHandler::DoWithinDistance,
        ),
        (MAV_CMD_CONDITION_YAW, AutoStartHandler::DoYaw),
        (MAV_CMD_DO_CHANGE_SPEED, AutoStartHandler::DoChangeSpeed),
        (MAV_CMD_DO_SET_HOME, AutoStartHandler::DoSetHome),
    ];
    for (id, handler) in cases {
        assert_eq!(start(id), (true, handler), "cmd {id}");
    }
}

#[test]
fn three_roi_ids_share_do_roi() {
    for id in [
        MAV_CMD_DO_SET_ROI_LOCATION,
        MAV_CMD_DO_SET_ROI_NONE,
        MAV_CMD_DO_SET_ROI,
    ] {
        assert_eq!(start(id), (true, AutoStartHandler::DoRoi), "cmd {id}");
    }
}

#[test]
fn land_start_and_return_path_are_recognised_noops() {
    assert_eq!(start(MAV_CMD_DO_LAND_START), (true, AutoStartHandler::NoOp));
    assert_eq!(
        start(MAV_CMD_DO_RETURN_PATH_START),
        (true, AutoStartHandler::NoOp)
    );
}

#[test]
fn unknown_command_returns_false() {
    let out = auto_start_command(0, AutoStartFeatures::all());
    assert!(!out.accepted);
    assert_eq!(out.handler, AutoStartHandler::Unknown);

    let out = auto_start_command(999, AutoStartFeatures::none());
    assert!(!out.accepted);
    assert_eq!(out.handler, AutoStartHandler::Unknown);
}

#[test]
fn gated_commands_refuse_when_compiled_out() {
    let gated = [
        MAV_CMD_NAV_GUIDED_ENABLE,
        MAV_CMD_NAV_PAYLOAD_PLACE,
        MAV_CMD_NAV_SCRIPT_TIME,
        MAV_CMD_DO_MOUNT_CONTROL,
        MAV_CMD_DO_GUIDED_LIMITS,
        MAV_CMD_DO_WINCH,
    ];
    for id in gated {
        let out = auto_start_command(id, AutoStartFeatures::none());
        assert!(!out.accepted, "cmd {id} should refuse when gated off");
        assert_eq!(out.handler, AutoStartHandler::Unknown);
    }
}

#[test]
fn gated_commands_dispatch_when_compiled_in() {
    let cases = [
        (
            MAV_CMD_NAV_GUIDED_ENABLE,
            AutoStartHandler::DoNavGuidedEnable,
        ),
        (MAV_CMD_NAV_PAYLOAD_PLACE, AutoStartHandler::DoPayloadPlace),
        (MAV_CMD_NAV_SCRIPT_TIME, AutoStartHandler::DoNavScriptTime),
        (MAV_CMD_DO_MOUNT_CONTROL, AutoStartHandler::DoMountControl),
        (MAV_CMD_DO_GUIDED_LIMITS, AutoStartHandler::DoGuidedLimits),
        (MAV_CMD_DO_WINCH, AutoStartHandler::DoWinch),
    ];
    for (id, handler) in cases {
        assert_eq!(start_all(id), (true, handler), "cmd {id}");
    }
}

#[test]
fn command_ids_match_mavlink() {
    assert_eq!(MAV_CMD_NAV_WAYPOINT, 16);
    assert_eq!(MAV_CMD_NAV_TAKEOFF, 22);
    assert_eq!(MAV_CMD_NAV_ARC_WAYPOINT, 36);
    assert_eq!(MAV_CMD_NAV_SPLINE_WAYPOINT, 82);
    assert_eq!(MAV_CMD_DO_RETURN_PATH_START, 188);
    assert_eq!(MAV_CMD_DO_LAND_START, 189);
    assert_eq!(MAV_CMD_DO_GUIDED_LIMITS, 222);
    assert_eq!(MAV_CMD_DO_WINCH, 42600);
    assert_eq!(MAV_CMD_NAV_SCRIPT_TIME, 42702);
    assert_eq!(MAV_CMD_NAV_ATTITUDE_TIME, 42703);
}

#[test]
fn waiting_without_origin_holds_loiter_and_does_not_start() {
    let out = auto_run(&AutoRunView::waiting_no_origin());
    assert!(!out.start_or_resume);
    assert!(out.waiting_to_start);
    assert!(!out.check_mission_change);
    assert!(!out.restart_current_nav_cmd);
    assert_eq!(out.mission_change_text, AutoMissionChangeText::None);
    assert!(!out.mission_update);
    assert_eq!(out.body, AutoRunBody::Loiter);
    assert!(!out.auto_rtl);
    assert!(!out.log_auto_rtl_exit);
}

#[test]
fn waiting_with_origin_starts_the_mission_and_still_runs_loiter() {
    let out = auto_run(&AutoRunView::waiting_with_origin());
    assert!(out.start_or_resume);
    assert!(!out.waiting_to_start);
    assert!(out.check_mission_change);
    assert!(!out.restart_current_nav_cmd);
    assert!(!out.mission_update);
    assert_eq!(out.body, AutoRunBody::Loiter);
}

#[test]
fn running_wp_updates_the_mission() {
    let out = auto_run(&AutoRunView::running_wp());
    assert!(!out.start_or_resume);
    assert!(!out.waiting_to_start);
    assert!(out.check_mission_change);
    assert!(!out.restart_current_nav_cmd);
    assert_eq!(out.mission_change_text, AutoMissionChangeText::None);
    assert!(out.mission_update);
    assert_eq!(out.body, AutoRunBody::Wp);
}

#[test]
fn mission_change_restarts_only_a_running_waypoint() {
    let mut view = AutoRunView::running_wp();
    view.mission_changed = true;
    let out = auto_run(&view);
    assert!(out.restart_current_nav_cmd);
    assert_eq!(out.mission_change_text, AutoMissionChangeText::Restarted);
    assert!(out.mission_update);

    view.restart_current_nav_cmd = false;
    let out = auto_run(&view);
    assert!(out.restart_current_nav_cmd);
    assert_eq!(
        out.mission_change_text,
        AutoMissionChangeText::RestartFailed
    );

    view.submode = AutoSubMode::Loiter;
    view.restart_current_nav_cmd = true;
    let out = auto_run(&view);
    assert!(!out.restart_current_nav_cmd);
    assert_eq!(out.mission_change_text, AutoMissionChangeText::None);
    assert!(out.mission_update);
    assert_eq!(out.body, AutoRunBody::Loiter);
}

#[test]
fn mission_change_ignored_when_mission_is_not_running() {
    let mut view = AutoRunView::running_wp();
    view.mission_changed = true;
    view.mission_running = false;
    let out = auto_run(&view);
    assert!(!out.restart_current_nav_cmd);
    assert_eq!(out.mission_change_text, AutoMissionChangeText::None);
    assert!(out.mission_update);
}

#[test]
fn submode_switch_picks_the_right_body() {
    let cases = [
        (AutoSubMode::Takeoff, AutoRunBody::Takeoff),
        (AutoSubMode::Wp, AutoRunBody::Wp),
        (AutoSubMode::CircleMoveToEdge, AutoRunBody::Wp),
        (AutoSubMode::Land, AutoRunBody::Land),
        (AutoSubMode::Rtl, AutoRunBody::Rtl),
        (AutoSubMode::Circle, AutoRunBody::Circle),
        (AutoSubMode::NavGuided, AutoRunBody::NavGuided),
        (AutoSubMode::Loiter, AutoRunBody::Loiter),
        (AutoSubMode::LoiterToAlt, AutoRunBody::LoiterToAlt),
        (AutoSubMode::NavScriptTime, AutoRunBody::NavGuided),
        (AutoSubMode::NavAttitudeTime, AutoRunBody::NavAttitudeTime),
    ];
    for (submode, body) in cases {
        let mut view = AutoRunView::running_wp();
        view.submode = submode;
        let out = auto_run(&view);
        assert_eq!(out.body, body, "submode {submode:?}");
    }
}

#[test]
fn nav_guided_body_is_skipped_when_compiled_out() {
    let mut view = AutoRunView::running_wp();
    view.submode = AutoSubMode::NavGuided;
    view.nav_guided_or_scripting = false;
    assert_eq!(auto_run(&view).body, AutoRunBody::None);

    view.submode = AutoSubMode::NavScriptTime;
    assert_eq!(auto_run(&view).body, AutoRunBody::None);
}

#[test]
fn auto_rtl_expires_unless_landing_return_or_complete() {
    let mut view = AutoRunView::running_wp();
    view.auto_rtl = true;
    let out = auto_run(&view);
    assert!(!out.auto_rtl);
    assert!(out.log_auto_rtl_exit);

    view.in_landing_sequence = true;
    let out = auto_run(&view);
    assert!(out.auto_rtl);
    assert!(!out.log_auto_rtl_exit);

    view.in_landing_sequence = false;
    view.in_return_path = true;
    let out = auto_run(&view);
    assert!(out.auto_rtl);
    assert!(!out.log_auto_rtl_exit);

    view.in_return_path = false;
    view.mission_complete = true;
    let out = auto_run(&view);
    assert!(out.auto_rtl);
    assert!(!out.log_auto_rtl_exit);
}

#[test]
fn auto_rtl_off_does_not_log_an_exit() {
    let out = auto_run(&AutoRunView::running_wp());
    assert!(!out.auto_rtl);
    assert!(!out.log_auto_rtl_exit);
}
