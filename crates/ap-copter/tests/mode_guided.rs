//! `ModeGuided` init / set_destination / run / set_velocity /
//! pos_control_run / set_angle leftovers, upstream `ArduCopter/mode_guided.cpp`.

use ap_copter::auto_yaw::YawMode;
use ap_copter::mode_guided::{
    guided_angle_control_start, guided_init, guided_mode_flags, guided_pos_control_run, guided_run,
    guided_set_angle, guided_set_destination, guided_set_vel_accel, guided_set_velocity,
    guided_timeout_ms, option_is_enabled, set_yaw_state_rad, use_wpnav_for_position_control,
    GuidedAngleStartView, GuidedInitView, GuidedOption, GuidedPosControlExit, GuidedPosControlView,
    GuidedRunBody, GuidedRunView, GuidedSetAngleView, GuidedSetDestFail, GuidedSetDestView,
    GuidedSetVelView, GuidedSubMode, GuidedYawAction, GUIDED_TIMEOUT_DEFAULT_S,
    GUIDED_TIMEOUT_MIN_S, MODE_NUMBER_FOLLOW, MODE_NUMBER_GUIDED, MODE_NUMBER_GUIDED_NOGPS,
    WPNAV_ACCELERATION_MSS, WP_ACC_Z_DEFAULT_MSS, WP_SPD_DEFAULT_MS, WP_SPD_DOWN_DEFAULT_MS,
    WP_SPD_UP_DEFAULT_MS,
};

#[test]
fn numbers_match_mode_h() {
    assert_eq!(MODE_NUMBER_GUIDED, 4);
    assert_eq!(MODE_NUMBER_GUIDED_NOGPS, 20);
    assert_eq!(MODE_NUMBER_FOLLOW, 23);
    assert_eq!(GuidedSubMode::TakeOff as u8, 0);
    assert_eq!(GuidedSubMode::Wp as u8, 1);
    assert_eq!(GuidedSubMode::Pos as u8, 2);
    assert_eq!(GuidedSubMode::PosVelAccel as u8, 3);
    assert_eq!(GuidedSubMode::VelAccel as u8, 4);
    assert_eq!(GuidedSubMode::Accel as u8, 5);
    assert_eq!(GuidedSubMode::Angle as u8, 6);
}

#[test]
fn flags_match_mode_h() {
    let flags = guided_mode_flags();
    assert_eq!(flags.mode_number, MODE_NUMBER_GUIDED);
    assert!(flags.requires_position);
    assert!(!flags.has_manual_throttle);
    assert!(flags.is_autopilot);
    assert!(flags.has_user_takeoff);
    assert!(flags.in_guided_mode);
    assert!(flags.requires_terrain_failsafe);
    assert!(flags.allows_gcs_or_scr_arming_with_throttle_high);
}

#[test]
fn guid_options_bits_match_mode_h() {
    assert_eq!(GuidedOption::AllowArmingFromTx as u32, 1);
    assert_eq!(GuidedOption::IgnorePilotYaw as u32, 4);
    assert_eq!(GuidedOption::SetAttitudeTargetThrustAsThrust as u32, 8);
    assert_eq!(GuidedOption::DoNotStabilizePositionXy as u32, 16);
    assert_eq!(GuidedOption::DoNotStabilizeVelocityXy as u32, 32);
    assert_eq!(GuidedOption::WpNavUsedForPosControl as u32, 64);
    assert_eq!(GuidedOption::AllowWeatherVaning as u32, 128);
    assert!(!option_is_enabled(0, GuidedOption::WpNavUsedForPosControl));
    assert!(option_is_enabled(64, GuidedOption::WpNavUsedForPosControl));
    assert!(!use_wpnav_for_position_control(0));
    assert!(use_wpnav_for_position_control(64));
}

#[test]
fn init_always_succeeds_and_parks_in_velaccel() {
    let view = GuidedInitView::typical();
    let ignore = guided_init(&view, true);
    let checks = guided_init(&view, false);
    assert!(ignore.ok);
    assert!(checks.ok);
    assert_eq!(ignore.submode, GuidedSubMode::VelAccel);
    assert_eq!(checks.submode, GuidedSubMode::VelAccel);
    assert!(!ignore.send_notification);
    assert!(!ignore.paused);
    assert!(!ignore.terrain_alt);
    assert!(ignore.vel_zero);
    assert!(ignore.accel_zero);
    assert!(ignore.init_ne);
    assert!(ignore.init_d);
    assert_eq!(ignore.yaw, GuidedYawAction::SetModeToDefault);
}

#[test]
fn init_sizes_pva_to_wpnav_defaults() {
    let view = GuidedInitView::typical();
    let out = guided_init(&view, false);
    assert_eq!(out.ne_speed_ms.to_bits(), WP_SPD_DEFAULT_MS.to_bits());
    assert_eq!(out.ne_accel_mss.to_bits(), WPNAV_ACCELERATION_MSS.to_bits());
    assert_eq!(
        out.d_speed_down_ms.to_bits(),
        WP_SPD_DOWN_DEFAULT_MS.to_bits()
    );
    assert_eq!(out.d_speed_up_ms.to_bits(), WP_SPD_UP_DEFAULT_MS.to_bits());
    assert_eq!(out.d_accel_mss.to_bits(), WP_ACC_Z_DEFAULT_MSS.to_bits());
}

#[test]
fn init_uses_the_callers_wpnav_limits_not_the_constants() {
    let view = GuidedInitView {
        default_speed_ne_ms: 7.0,
        wp_acceleration_mss: 3.5,
        default_speed_down_ms: 0.8,
        default_speed_up_ms: 1.2,
        accel_d_mss: 2.0,
    };
    let out = guided_init(&view, false);
    assert_eq!(out.ne_speed_ms.to_bits(), 7.0f32.to_bits());
    assert_eq!(out.ne_accel_mss.to_bits(), 3.5f32.to_bits());
    assert_eq!(out.d_speed_down_ms.to_bits(), 0.8f32.to_bits());
    assert_eq!(out.d_speed_up_ms.to_bits(), 1.2f32.to_bits());
    assert_eq!(out.d_accel_mss.to_bits(), 2.0f32.to_bits());
}

#[test]
fn fence_reject_leaves_submode_and_skips_yaw() {
    let mut view = GuidedSetDestView::after_init();
    view.within_fence = false;
    let out = guided_set_destination(&view);
    assert!(!out.ok);
    assert_eq!(out.fail, GuidedSetDestFail::OutsideFence);
    assert_eq!(out.submode, GuidedSubMode::VelAccel);
    assert!(!out.started_wp);
    assert!(!out.started_pos);
    assert!(!out.held_position);
    assert_eq!(out.yaw, GuidedYawAction::NotCalled);
    assert!(!out.send_notification);
    assert!(out.log_dest_outside_fence);
    assert!(!out.log_failed_to_set_destination);
}

#[test]
fn fence_compiled_out_does_not_consult_within_fence() {
    let mut view = GuidedSetDestView::after_init();
    view.fence_enabled = false;
    view.within_fence = false;
    let out = guided_set_destination(&view);
    assert!(out.ok);
    assert_eq!(out.fail, GuidedSetDestFail::None);
    assert_eq!(out.submode, GuidedSubMode::Pos);
    assert!(!out.log_dest_outside_fence);
}

#[test]
fn pos_path_after_init_starts_pos_and_notifies() {
    let view = GuidedSetDestView::after_init();
    let out = guided_set_destination(&view);
    assert!(out.ok);
    assert_eq!(out.submode, GuidedSubMode::Pos);
    assert!(out.started_pos);
    assert!(!out.started_wp);
    assert!(!out.held_position);
    assert_eq!(out.yaw, GuidedYawAction::SetModeToDefault);
    assert_eq!(out.init_pos_terrain_d_m, Some(0.0));
    assert_eq!(out.pos_target_ned_m, Some([20.0, 10.0, -15.0]));
    assert!(!out.terrain_alt);
    assert!(out.vel_zero);
    assert!(out.accel_zero);
    assert_eq!(out.update_time_ms, Some(1_000));
    assert!(out.send_notification);
}

#[test]
fn already_in_pos_does_not_restart_pva() {
    let mut view = GuidedSetDestView::after_init();
    view.submode = GuidedSubMode::Pos;
    let out = guided_set_destination(&view);
    assert!(out.ok);
    assert_eq!(out.submode, GuidedSubMode::Pos);
    assert!(!out.started_pos);
    assert!(out.send_notification);
}

#[test]
fn missing_vector_leaves_velaccel_and_skips_yaw() {
    let mut view = GuidedSetDestView::after_init();
    view.vector_ned_ok = false;
    let out = guided_set_destination(&view);
    assert!(!out.ok);
    assert_eq!(out.fail, GuidedSetDestFail::MissingVectorNed);
    assert_eq!(out.submode, GuidedSubMode::VelAccel);
    assert!(!out.started_pos);
    assert_eq!(out.yaw, GuidedYawAction::NotCalled);
    assert!(!out.send_notification);
    assert!(!out.log_failed_to_set_destination);
}

#[test]
fn terrain_dest_without_terrain_holds_in_velaccel() {
    let mut view = GuidedSetDestView::after_init();
    view.is_terrain_alt = true;
    view.terrain_d_ok = false;
    let out = guided_set_destination(&view);
    assert!(!out.ok);
    assert_eq!(out.fail, GuidedSetDestFail::MissingTerrainAlt);
    assert_eq!(out.submode, GuidedSubMode::VelAccel);
    assert!(out.started_pos);
    assert!(out.held_position);
    assert_eq!(out.yaw, GuidedYawAction::SetModeToDefault);
    assert_eq!(out.init_pos_terrain_d_m, None);
    assert!(out.vel_zero);
    assert!(out.accel_zero);
    assert!(!out.send_notification);
}

#[test]
fn terrain_dest_inits_offset_only_when_previous_was_not_terrain() {
    let mut first = GuidedSetDestView::after_init();
    first.is_terrain_alt = true;
    first.terrain_d_ok = true;
    first.terrain_d_m = 12.0;
    let out = guided_set_destination(&first);
    assert!(out.ok);
    assert_eq!(out.init_pos_terrain_d_m, Some(12.0));
    assert!(out.terrain_alt);
    assert_eq!(out.submode, GuidedSubMode::Pos);

    let mut again = first;
    again.submode = GuidedSubMode::Pos;
    again.guided_is_terrain_alt = true;
    let out = guided_set_destination(&again);
    assert!(out.ok);
    assert_eq!(out.init_pos_terrain_d_m, None);
    assert!(!out.started_pos);
    assert!(out.terrain_alt);
}

#[test]
fn wpnav_path_starts_wp_then_notifies() {
    let mut view = GuidedSetDestView::after_init();
    view.use_wpnav = true;
    let out = guided_set_destination(&view);
    assert!(out.ok);
    assert_eq!(out.submode, GuidedSubMode::Wp);
    assert!(out.started_wp);
    assert!(out.wp_and_spline_init);
    assert!(!out.started_pos);
    assert_eq!(out.yaw, GuidedYawAction::SetModeToDefault);
    assert!(out.send_notification);
    assert_eq!(out.pos_target_ned_m, None);
}

#[test]
fn already_in_wp_does_not_reinit_wpnav() {
    let mut view = GuidedSetDestView::after_init();
    view.use_wpnav = true;
    view.submode = GuidedSubMode::Wp;
    let out = guided_set_destination(&view);
    assert!(out.ok);
    assert!(!out.started_wp);
    assert!(!out.wp_and_spline_init);
    assert_eq!(out.submode, GuidedSubMode::Wp);
}

#[test]
fn wp_dest_fail_stays_in_wp_after_a_start() {
    let mut view = GuidedSetDestView::after_init();
    view.use_wpnav = true;
    view.wp_dest_ok = false;
    let out = guided_set_destination(&view);
    assert!(!out.ok);
    assert_eq!(out.fail, GuidedSetDestFail::FailedToSetWpDestination);
    assert_eq!(out.submode, GuidedSubMode::Wp);
    assert!(out.started_wp);
    assert!(out.wp_and_spline_init);
    assert_eq!(out.yaw, GuidedYawAction::NotCalled);
    assert!(!out.send_notification);
    assert!(out.log_failed_to_set_destination);
}

#[test]
fn relative_yaw_wins_over_a_simultaneous_rate() {
    let action = set_yaw_state_rad(true, 1.2, true, 0.4, true);
    assert_eq!(
        action,
        GuidedYawAction::SetFixedYaw {
            yaw_rad: 1.2,
            relative: true,
        }
    );

    let mut view = GuidedSetDestView::after_init();
    view.use_yaw = true;
    view.yaw_rad = 1.2;
    view.use_yaw_rate = true;
    view.yaw_rate_rads = 0.4;
    view.relative_yaw = true;
    let out = guided_set_destination(&view);
    assert!(out.ok);
    assert_eq!(out.yaw, action);
}

#[test]
fn yaw_angle_without_rate_writes_a_zero_rate() {
    let action = set_yaw_state_rad(true, 0.5, false, 9.0, false);
    match action {
        GuidedYawAction::SetAngleAndRate {
            yaw_rad,
            yaw_rate_rads,
        } => {
            assert_eq!(yaw_rad.to_bits(), 0.5f32.to_bits());
            assert_eq!(yaw_rate_rads.to_bits(), 0.0f32.to_bits());
        }
        other => panic!("expected SetAngleAndRate, got {other:?}"),
    }
}

#[test]
fn yaw_rate_only_does_not_touch_the_angle() {
    assert_eq!(
        set_yaw_state_rad(false, 1.0, true, 0.3, false),
        GuidedYawAction::SetRate { yaw_rate_rads: 0.3 }
    );
}

#[test]
fn run_after_init_dispatches_velaccel() {
    let out = guided_run(&GuidedRunView::after_init());
    assert_eq!(out.body, GuidedRunBody::VelAccel);
    assert!(!out.send_notification);
    assert!(!out.mission_item_reached);
}

#[test]
fn paused_skips_the_submode_switch() {
    let mut view = GuidedRunView::after_wp_dest();
    view.paused = true;
    view.wp_reached = true;
    let out = guided_run(&view);
    assert_eq!(out.body, GuidedRunBody::Pause);
    assert!(out.send_notification);
    assert!(!out.mission_item_reached);
}

#[test]
fn run_dispatches_each_unpaused_submode() {
    let cases = [
        (GuidedSubMode::TakeOff, GuidedRunBody::TakeOff),
        (GuidedSubMode::Wp, GuidedRunBody::Wp),
        (GuidedSubMode::Pos, GuidedRunBody::Pos),
        (GuidedSubMode::Accel, GuidedRunBody::Accel),
        (GuidedSubMode::VelAccel, GuidedRunBody::VelAccel),
        (GuidedSubMode::PosVelAccel, GuidedRunBody::PosVelAccel),
        (GuidedSubMode::Angle, GuidedRunBody::Angle),
    ];
    for (submode, body) in cases {
        let mut view = GuidedRunView::after_init();
        view.submode = submode;
        let out = guided_run(&view);
        assert_eq!(out.body, body);
        assert!(!out.send_notification);
        assert!(!out.mission_item_reached);
    }
}

#[test]
fn wp_reached_clears_notification_and_sends_gcs() {
    let mut view = GuidedRunView::after_wp_dest();
    view.wp_reached = true;
    let out = guided_run(&view);
    assert_eq!(out.body, GuidedRunBody::Wp);
    assert!(!out.send_notification);
    assert!(out.mission_item_reached);
}

#[test]
fn wp_not_yet_reached_keeps_notification() {
    let view = GuidedRunView::after_wp_dest();
    let out = guided_run(&view);
    assert_eq!(out.body, GuidedRunBody::Wp);
    assert!(out.send_notification);
    assert!(!out.mission_item_reached);
}

#[test]
fn wp_reached_without_notification_does_not_send() {
    let mut view = GuidedRunView::after_wp_dest();
    view.send_notification = false;
    view.wp_reached = true;
    let out = guided_run(&view);
    assert_eq!(out.body, GuidedRunBody::Wp);
    assert!(!out.send_notification);
    assert!(!out.mission_item_reached);
}

#[test]
fn pos_path_does_not_consult_wp_reached() {
    let mut view = GuidedRunView::after_wp_dest();
    view.submode = GuidedSubMode::Pos;
    view.wp_reached = true;
    let out = guided_run(&view);
    assert_eq!(out.body, GuidedRunBody::Pos);
    assert!(out.send_notification);
    assert!(!out.mission_item_reached);
}

#[test]
fn set_velocity_zeroes_accel_and_stays_in_velaccel() {
    let view = GuidedSetVelView::after_init();
    let out = guided_set_velocity(&view);
    assert_eq!(out.submode, GuidedSubMode::VelAccel);
    assert!(!out.started_velaccel);
    assert_eq!(out.yaw, GuidedYawAction::SetModeToDefault);
    assert_eq!(out.pos_target_ned_m, [0.0, 0.0, 0.0]);
    assert!(!out.terrain_alt);
    assert_eq!(out.vel_ned_ms, [1.5, -0.5, 0.0]);
    assert_eq!(out.accel_ned_mss, [0.0, 0.0, 0.0]);
    assert_eq!(out.update_time_ms, 2_000);
    assert!(out.logged);
}

#[test]
fn set_vel_accel_keeps_the_callers_accel() {
    let view = GuidedSetVelView::after_init();
    let out = guided_set_vel_accel(&view);
    assert_eq!(out.accel_ned_mss, [0.2, 0.1, 0.0]);
    assert_eq!(out.vel_ned_ms, view.vel_ned_ms);
    assert!(!out.started_velaccel);
}

#[test]
fn set_velocity_from_pos_starts_velaccel() {
    let mut view = GuidedSetVelView::after_init();
    view.submode = GuidedSubMode::Pos;
    let out = guided_set_velocity(&view);
    assert_eq!(out.submode, GuidedSubMode::VelAccel);
    assert!(out.started_velaccel);
}

#[test]
fn set_velocity_relative_yaw_wins_over_a_rate() {
    let mut view = GuidedSetVelView::after_init();
    view.use_yaw = true;
    view.yaw_rad = 0.8;
    view.use_yaw_rate = true;
    view.yaw_rate_rads = 0.2;
    view.relative_yaw = true;
    let out = guided_set_velocity(&view);
    assert_eq!(
        out.yaw,
        GuidedYawAction::SetFixedYaw {
            yaw_rad: 0.8,
            relative: true,
        }
    );
}

#[test]
fn set_velocity_skips_log_when_request_or_compile_is_off() {
    let mut no_request = GuidedSetVelView::after_init();
    no_request.log_request = false;
    assert!(!guided_set_velocity(&no_request).logged);

    let mut compiled_out = GuidedSetVelView::after_init();
    compiled_out.logging_enabled = false;
    assert!(!guided_set_velocity(&compiled_out).logged);
}

#[test]
fn timeout_ms_floors_at_one_tenth_second() {
    assert_eq!(guided_timeout_ms(GUIDED_TIMEOUT_DEFAULT_S), 3_000);
    assert_eq!(guided_timeout_ms(GUIDED_TIMEOUT_MIN_S), 100);
    assert_eq!(guided_timeout_ms(0.0), 100);
    assert_eq!(guided_timeout_ms(-1.0), 100);
    assert_eq!(guided_timeout_ms(0.05), 100);
    assert_eq!(guided_timeout_ms(1.5), 1_500);
}

#[test]
fn pos_run_after_dest_flies_without_terrain_or_hold() {
    let out = guided_pos_control_run(&GuidedPosControlView::after_pos_dest());
    assert_eq!(
        out.exit,
        GuidedPosControlExit::Flew {
            yaw_hold: false,
            terrain_d_m: 0.0,
            terrain_margin_m: 0.0,
        }
    );
}

#[test]
fn pos_run_disarmed_skips_terrain_failsafe() {
    let mut view = GuidedPosControlView::after_pos_dest();
    view.disarmed_or_landed = true;
    view.terrain_alt = true;
    view.terrain_d_ok = false;
    let out = guided_pos_control_run(&view);
    assert_eq!(
        out.exit,
        GuidedPosControlExit::Disarmed {
            keep_interlock: false,
        }
    );
}

#[test]
fn pos_run_tradheli_interlock_is_the_only_keep() {
    let mut heli = GuidedPosControlView::after_pos_dest();
    heli.disarmed_or_landed = true;
    heli.is_tradheli = true;
    heli.motor_interlock = true;
    assert_eq!(
        guided_pos_control_run(&heli).exit,
        GuidedPosControlExit::Disarmed {
            keep_interlock: true,
        }
    );

    let mut no_lock = heli;
    no_lock.motor_interlock = false;
    assert_eq!(
        guided_pos_control_run(&no_lock).exit,
        GuidedPosControlExit::Disarmed {
            keep_interlock: false,
        }
    );

    let mut not_heli = heli;
    not_heli.is_tradheli = false;
    assert_eq!(
        guided_pos_control_run(&not_heli).exit,
        GuidedPosControlExit::Disarmed {
            keep_interlock: false,
        }
    );
}

#[test]
fn pos_run_terrain_dest_without_terrain_fails_closed() {
    let mut view = GuidedPosControlView::after_pos_dest();
    view.terrain_alt = true;
    view.terrain_d_ok = false;
    assert_eq!(
        guided_pos_control_run(&view).exit,
        GuidedPosControlExit::TerrainFailsafe
    );
}

#[test]
fn pos_run_non_terrain_never_asks_for_terrain_d() {
    let mut view = GuidedPosControlView::after_pos_dest();
    view.terrain_d_ok = false;
    view.terrain_d_m = 12.0;
    view.wp_terrain_margin_m = 5.0;
    let out = guided_pos_control_run(&view);
    assert_eq!(
        out.exit,
        GuidedPosControlExit::Flew {
            yaw_hold: false,
            terrain_d_m: 0.0,
            terrain_margin_m: 0.0,
        }
    );
}

#[test]
fn pos_run_terrain_margin_is_min_of_wpnav_and_half_abs_z() {
    let mut view = GuidedPosControlView::after_pos_dest();
    view.terrain_alt = true;
    view.terrain_d_ok = true;
    view.terrain_d_m = 12.0;
    view.pos_target_ned_m = [0.0, 0.0, -15.0];
    view.wp_terrain_margin_m = 2.0;
    let out = guided_pos_control_run(&view);
    assert_eq!(
        out.exit,
        GuidedPosControlExit::Flew {
            yaw_hold: false,
            terrain_d_m: 12.0,
            terrain_margin_m: 2.0,
        }
    );

    view.wp_terrain_margin_m = 20.0;
    let out = guided_pos_control_run(&view);
    match out.exit {
        GuidedPosControlExit::Flew {
            terrain_margin_m, ..
        } => assert_eq!(terrain_margin_m.to_bits(), 7.5f32.to_bits()),
        other => panic!("expected Flew, got {other:?}"),
    }

    view.pos_target_ned_m = [0.0, 0.0, 4.0];
    view.wp_terrain_margin_m = 20.0;
    let out = guided_pos_control_run(&view);
    match out.exit {
        GuidedPosControlExit::Flew {
            terrain_margin_m, ..
        } => assert_eq!(terrain_margin_m.to_bits(), 2.0f32.to_bits()),
        other => panic!("expected Flew, got {other:?}"),
    }
}

#[test]
fn pos_run_timeout_holds_only_rate_and_angle_rate() {
    let mut view = GuidedPosControlView::after_pos_dest();
    view.now_ms = view.update_time_ms + 3_001;
    view.auto_yaw = YawMode::Rate;
    match guided_pos_control_run(&view).exit {
        GuidedPosControlExit::Flew { yaw_hold, .. } => assert!(yaw_hold),
        other => panic!("expected Flew, got {other:?}"),
    }

    view.auto_yaw = YawMode::AngleRate;
    match guided_pos_control_run(&view).exit {
        GuidedPosControlExit::Flew { yaw_hold, .. } => assert!(yaw_hold),
        other => panic!("expected Flew, got {other:?}"),
    }

    for mode in [
        YawMode::Hold,
        YawMode::LookAtNextWp,
        YawMode::Fixed,
        YawMode::PilotRate,
    ] {
        view.auto_yaw = mode;
        match guided_pos_control_run(&view).exit {
            GuidedPosControlExit::Flew { yaw_hold, .. } => {
                assert!(!yaw_hold, "{mode:?} must not HOLD on timeout")
            }
            other => panic!("expected Flew, got {other:?}"),
        }
    }
}

#[test]
fn pos_run_timeout_is_strictly_greater() {
    let mut view = GuidedPosControlView::after_pos_dest();
    view.auto_yaw = YawMode::Rate;
    view.now_ms = view.update_time_ms + 3_000;
    match guided_pos_control_run(&view).exit {
        GuidedPosControlExit::Flew { yaw_hold, .. } => assert!(!yaw_hold),
        other => panic!("expected Flew, got {other:?}"),
    }

    view.now_ms = view.update_time_ms + 3_001;
    match guided_pos_control_run(&view).exit {
        GuidedPosControlExit::Flew { yaw_hold, .. } => assert!(yaw_hold),
        other => panic!("expected Flew, got {other:?}"),
    }
}

#[test]
fn pos_run_timeout_uses_unsigned_wrap() {
    let mut view = GuidedPosControlView::after_pos_dest();
    view.auto_yaw = YawMode::Rate;
    view.update_time_ms = 200;
    view.now_ms = 100;
    match guided_pos_control_run(&view).exit {
        GuidedPosControlExit::Flew { yaw_hold, .. } => assert!(yaw_hold),
        other => panic!("expected Flew, got {other:?}"),
    }
}

#[test]
fn pos_run_zero_guid_timeout_still_floors_to_100ms() {
    let mut view = GuidedPosControlView::after_pos_dest();
    view.guided_timeout_s = 0.0;
    view.auto_yaw = YawMode::Rate;
    view.now_ms = view.update_time_ms + 100;
    match guided_pos_control_run(&view).exit {
        GuidedPosControlExit::Flew { yaw_hold, .. } => assert!(!yaw_hold),
        other => panic!("expected Flew, got {other:?}"),
    }
    view.now_ms = view.update_time_ms + 101;
    match guided_pos_control_run(&view).exit {
        GuidedPosControlExit::Flew { yaw_hold, .. } => assert!(yaw_hold),
        other => panic!("expected Flew, got {other:?}"),
    }
}

#[test]
fn angle_start_after_init_does_not_reinit_active_d() {
    let view = GuidedAngleStartView::after_init();
    let out = guided_angle_control_start(&view);
    assert_eq!(out.submode, GuidedSubMode::Angle);
    assert!(!out.init_d);
    assert!(!out.init_ne);
    assert_eq!(
        out.d_speed_down_ms.to_bits(),
        WP_SPD_DOWN_DEFAULT_MS.to_bits()
    );
    assert_eq!(out.d_speed_up_ms.to_bits(), WP_SPD_UP_DEFAULT_MS.to_bits());
    assert_eq!(out.d_accel_mss.to_bits(), WP_ACC_Z_DEFAULT_MSS.to_bits());
    assert_eq!(out.ang_vel_body, [0.0, 0.0, 0.0]);
    assert_eq!(out.climb_rate_ms.to_bits(), 0.0f32.to_bits());
    assert_eq!(out.update_time_ms, 2_000);
    assert_eq!(out.attitude_quat, [1.0, 0.0, 0.0, 0.0]);
}

#[test]
fn angle_start_inits_d_only_when_inactive() {
    let mut view = GuidedAngleStartView::after_init();
    view.d_is_active = false;
    let out = guided_angle_control_start(&view);
    assert!(out.init_d);
    assert!(!out.init_ne);
}

#[test]
fn angle_start_uses_the_callers_d_limits_not_the_constants() {
    let view = GuidedAngleStartView {
        d_is_active: true,
        default_speed_down_ms: 0.8,
        default_speed_up_ms: 1.2,
        accel_d_mss: 2.0,
        att_target_yaw_rad: 0.0,
        now_ms: 500,
    };
    let out = guided_angle_control_start(&view);
    assert_eq!(out.d_speed_down_ms.to_bits(), 0.8f32.to_bits());
    assert_eq!(out.d_speed_up_ms.to_bits(), 1.2f32.to_bits());
    assert_eq!(out.d_accel_mss.to_bits(), 2.0f32.to_bits());
}

#[test]
fn angle_start_seeds_a_yaw_only_quat() {
    let mut view = GuidedAngleStartView::after_init();
    view.att_target_yaw_rad = core::f32::consts::FRAC_PI_2;
    let out = guided_angle_control_start(&view);
    let q = ap_math::quaternion::Quaternion::from_euler(0.0, 0.0, view.att_target_yaw_rad);
    assert_eq!(out.attitude_quat, [q.q1, q.q2, q.q3, q.q4]);
    assert!((out.attitude_quat[0] - core::f32::consts::FRAC_1_SQRT_2).abs() < 1.0e-5);
}

#[test]
fn set_angle_after_init_starts_angle_and_stores_climb() {
    let view = GuidedSetAngleView::after_init();
    let out = guided_set_angle(&view);
    assert_eq!(out.submode, GuidedSubMode::Angle);
    assert!(out.started_angle);
    assert!(!out.init_d);
    assert_eq!(out.d_speed_down_ms, Some(WP_SPD_DOWN_DEFAULT_MS));
    assert_eq!(out.d_speed_up_ms, Some(WP_SPD_UP_DEFAULT_MS));
    assert_eq!(out.d_accel_mss, Some(WP_ACC_Z_DEFAULT_MSS));
    assert_eq!(out.attitude_quat, [1.0, 0.0, 0.0, 0.0]);
    assert_eq!(out.ang_vel_body, [0.1, -0.05, 0.2]);
    assert!(!out.use_thrust);
    assert_eq!(out.thrust_norm.to_bits(), 0.0f32.to_bits());
    assert_eq!(out.climb_rate_ms.to_bits(), 1.5f32.to_bits());
    assert_eq!(out.update_time_ms, 2_000);
    assert!(out.logged);
}

#[test]
fn set_angle_start_inits_d_when_inactive() {
    let mut view = GuidedSetAngleView::after_init();
    view.d_is_active = false;
    let out = guided_set_angle(&view);
    assert!(out.started_angle);
    assert!(out.init_d);
}

#[test]
fn already_in_angle_does_not_restart() {
    let mut view = GuidedSetAngleView::after_init();
    view.submode = GuidedSubMode::Angle;
    let out = guided_set_angle(&view);
    assert!(!out.started_angle);
    assert!(!out.init_d);
    assert_eq!(out.d_speed_down_ms, None);
    assert_eq!(out.d_speed_up_ms, None);
    assert_eq!(out.d_accel_mss, None);
    assert_eq!(out.submode, GuidedSubMode::Angle);
}

#[test]
fn already_in_angle_thrust_to_climb_reinits_d() {
    let mut view = GuidedSetAngleView::after_init();
    view.submode = GuidedSubMode::Angle;
    view.already_use_thrust = true;
    view.use_thrust = false;
    view.climb_rate_ms_or_thrust = 0.8;
    let out = guided_set_angle(&view);
    assert!(!out.started_angle);
    assert!(out.init_d);
    assert!(!out.use_thrust);
    assert_eq!(out.climb_rate_ms.to_bits(), 0.8f32.to_bits());
    assert_eq!(out.thrust_norm.to_bits(), 0.0f32.to_bits());
}

#[test]
fn already_in_angle_climb_to_thrust_does_not_reinit_d() {
    let mut view = GuidedSetAngleView::after_init();
    view.submode = GuidedSubMode::Angle;
    view.already_use_thrust = false;
    view.use_thrust = true;
    view.climb_rate_ms_or_thrust = 0.4;
    let out = guided_set_angle(&view);
    assert!(!out.started_angle);
    assert!(!out.init_d);
    assert!(out.use_thrust);
    assert_eq!(out.thrust_norm.to_bits(), 0.4f32.to_bits());
    assert_eq!(out.climb_rate_ms.to_bits(), 0.0f32.to_bits());
}

#[test]
fn already_in_angle_thrust_to_thrust_does_not_reinit_d() {
    let mut view = GuidedSetAngleView::after_init();
    view.submode = GuidedSubMode::Angle;
    view.already_use_thrust = true;
    view.use_thrust = true;
    view.d_is_active = false;
    let out = guided_set_angle(&view);
    assert!(!out.started_angle);
    assert!(!out.init_d);
}

#[test]
fn start_plus_climb_does_not_take_the_thrust_switch() {
    let mut view = GuidedSetAngleView::after_init();
    view.already_use_thrust = true;
    view.use_thrust = false;
    view.d_is_active = true;
    let out = guided_set_angle(&view);
    assert!(out.started_angle);
    assert!(!out.init_d);
}

#[test]
fn set_angle_thrust_zeroes_climb() {
    let mut view = GuidedSetAngleView::after_init();
    view.use_thrust = true;
    view.climb_rate_ms_or_thrust = 0.55;
    let out = guided_set_angle(&view);
    assert!(out.use_thrust);
    assert_eq!(out.thrust_norm.to_bits(), 0.55f32.to_bits());
    assert_eq!(out.climb_rate_ms.to_bits(), 0.0f32.to_bits());
}

#[test]
fn set_angle_always_converts_euler_even_when_logging_is_off() {
    let mut view = GuidedSetAngleView::after_init();
    view.logging_enabled = false;
    let q = ap_math::quaternion::Quaternion::from_euler(0.3, -0.2, 1.1);
    view.attitude_quat = [q.q1, q.q2, q.q3, q.q4];
    let out = guided_set_angle(&view);
    assert!(!out.logged);
    let (roll, pitch, yaw) = q.to_euler();
    assert!((out.euler_rad[0] - roll).abs() < 1.0e-5);
    assert!((out.euler_rad[1] - pitch).abs() < 1.0e-5);
    assert!((out.euler_rad[2] - yaw).abs() < 1.0e-5);
}

#[test]
fn set_angle_has_no_log_request_gate() {
    let view = GuidedSetAngleView::after_init();
    assert!(guided_set_angle(&view).logged);
}
