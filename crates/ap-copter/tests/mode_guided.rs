//! `ModeGuided` init / set_destination leftovers, upstream `ArduCopter/mode_guided.cpp`.

use ap_copter::mode_guided::{
    guided_init, guided_mode_flags, guided_set_destination, option_is_enabled, set_yaw_state_rad,
    use_wpnav_for_position_control, GuidedInitView, GuidedOption, GuidedSetDestFail,
    GuidedSetDestView, GuidedSubMode, GuidedYawAction, MODE_NUMBER_FOLLOW, MODE_NUMBER_GUIDED,
    MODE_NUMBER_GUIDED_NOGPS, WPNAV_ACCELERATION_MSS, WP_ACC_Z_DEFAULT_MSS, WP_SPD_DEFAULT_MS,
    WP_SPD_DOWN_DEFAULT_MS, WP_SPD_UP_DEFAULT_MS,
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
