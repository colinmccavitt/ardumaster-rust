//! `ModeFollow` init / run leftover, upstream `ArduCopter/mode_follow.cpp`.

use ap_copter::mode_follow::{
    follow_init, follow_mode_flags, follow_option_is_enabled, follow_run, FollowInitView,
    FollowPosInput, FollowRunExit, FollowRunView, FollowYawBehave, FollowYawSource,
    FOLLOW_OPTION_MOUNT_FOLLOW_ON_ENTER, FOLLOW_YAW_LENGTH_SQ_MIN,
};
use ap_copter::mode_guided::{
    guided_mode_flags, GuidedYawAction, MODE_NUMBER_FOLLOW, MODE_NUMBER_GUIDED,
    WPNAV_ACCELERATION_MSS, WP_ACC_Z_DEFAULT_MSS, WP_SPD_DEFAULT_MS, WP_SPD_DOWN_DEFAULT_MS,
    WP_SPD_UP_DEFAULT_MS,
};
use ap_math::scalar::radians;
use ap_math::vector2::Vector2f;

#[test]
fn follow_number_is_twenty_three() {
    assert_eq!(MODE_NUMBER_FOLLOW, 23);
    assert_ne!(MODE_NUMBER_FOLLOW, MODE_NUMBER_GUIDED);
}

#[test]
fn follow_flags_inherit_guided_except_number_and_arming() {
    let guided = guided_mode_flags();
    let follow = follow_mode_flags();
    assert_eq!(follow.guided.mode_number, MODE_NUMBER_FOLLOW);
    assert_eq!(follow.guided.requires_position, guided.requires_position);
    assert_eq!(
        follow.guided.has_manual_throttle,
        guided.has_manual_throttle
    );
    assert_eq!(follow.guided.is_autopilot, guided.is_autopilot);
    assert_eq!(follow.guided.has_user_takeoff, guided.has_user_takeoff);
    assert_eq!(follow.guided.in_guided_mode, guided.in_guided_mode);
    assert_eq!(
        follow.guided.requires_terrain_failsafe,
        guided.requires_terrain_failsafe
    );
    assert_eq!(
        follow.guided.allows_gcs_or_scr_arming_with_throttle_high,
        guided.allows_gcs_or_scr_arming_with_throttle_high
    );
    assert!(!follow.allows_arming);
}

#[test]
fn mount_follow_on_enter_is_bit_zero() {
    assert_eq!(FOLLOW_OPTION_MOUNT_FOLLOW_ON_ENTER, 1);
    assert!(!follow_option_is_enabled(
        0,
        FOLLOW_OPTION_MOUNT_FOLLOW_ON_ENTER
    ));
    assert!(follow_option_is_enabled(
        1,
        FOLLOW_OPTION_MOUNT_FOLLOW_ON_ENTER
    ));
}

#[test]
fn init_disabled_warns_and_skips_controllers() {
    let view = FollowInitView::typical();
    let ignore = follow_init(&view, true);
    let checks = follow_init(&view, false);
    assert_eq!(ignore, checks);
    assert!(!ignore.ok);
    assert!(ignore.gcs_enable_warning);
    assert_eq!(ignore.mount_sysid, None);
    assert!(!ignore.init_ne);
    assert!(!ignore.init_d);
    assert_eq!(ignore.yaw, GuidedYawAction::NotCalled);
    assert_eq!(ignore.ne_speed_ms.to_bits(), 0.0f32.to_bits());
}

#[test]
fn init_enabled_always_succeeds_and_inits_both_axes() {
    let view = FollowInitView::enabled();
    let ignore = follow_init(&view, true);
    let checks = follow_init(&view, false);
    assert_eq!(ignore, checks);
    assert!(ignore.ok);
    assert!(!ignore.gcs_enable_warning);
    assert_eq!(ignore.mount_sysid, None);
    assert!(ignore.init_ne);
    assert!(ignore.init_d);
    assert_eq!(ignore.yaw, GuidedYawAction::SetModeToDefault);
}

#[test]
fn init_sizes_pva_to_wpnav_defaults() {
    let view = FollowInitView::enabled();
    let out = follow_init(&view, false);
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
    let mut view = FollowInitView::enabled();
    view.default_speed_ne_ms = 7.0;
    view.wp_acceleration_mss = 3.5;
    view.default_speed_down_ms = 0.8;
    view.default_speed_up_ms = 1.2;
    view.accel_d_mss = 2.0;
    let out = follow_init(&view, false);
    assert_eq!(out.ne_speed_ms.to_bits(), 7.0f32.to_bits());
    assert_eq!(out.ne_accel_mss.to_bits(), 3.5f32.to_bits());
    assert_eq!(out.d_speed_down_ms.to_bits(), 0.8f32.to_bits());
    assert_eq!(out.d_speed_up_ms.to_bits(), 1.2f32.to_bits());
    assert_eq!(out.d_accel_mss.to_bits(), 2.0f32.to_bits());
}

#[test]
fn init_mount_needs_compile_option_and_singleton() {
    let mut view = FollowInitView::enabled();
    view.mount_follow_on_enter = true;
    view.mount_enabled = true;
    view.mount_present = true;
    view.target_sysid = 17;
    assert_eq!(follow_init(&view, false).mount_sysid, Some(17));

    view.mount_enabled = false;
    assert_eq!(follow_init(&view, false).mount_sysid, None);

    view.mount_enabled = true;
    view.mount_present = false;
    assert_eq!(follow_init(&view, false).mount_sysid, None);

    view.mount_present = true;
    view.mount_follow_on_enter = false;
    assert_eq!(follow_init(&view, false).mount_sysid, None);
}

#[test]
fn init_disabled_never_hands_off_the_mount() {
    let mut view = FollowInitView::typical();
    view.mount_follow_on_enter = true;
    view.mount_enabled = true;
    view.mount_present = true;
    view.target_sysid = 17;
    let out = follow_init(&view, false);
    assert!(!out.ok);
    assert_eq!(out.mount_sysid, None);
}

#[test]
fn yaw_behave_matches_ap_follow() {
    assert_eq!(FollowYawBehave::None as u8, 0);
    assert_eq!(FollowYawBehave::FaceLeadVehicle as u8, 1);
    assert_eq!(FollowYawBehave::SameAsLeadVehicle as u8, 2);
    assert_eq!(FollowYawBehave::DirOfFlight as u8, 3);
    assert_eq!(FOLLOW_YAW_LENGTH_SQ_MIN.to_bits(), 1.0f32.to_bits());
}

#[test]
fn run_flying_none_keeps_att_yaw_and_feeds_pva() {
    let view = FollowRunView::flying();
    assert_eq!(
        follow_run(&view).exit,
        FollowRunExit::Flew {
            init_offsets: true,
            input: FollowPosInput::PosVelAccel,
            yaw_source: FollowYawSource::AttTarget,
            yaw_rad: 0.3,
            yaw_rate_rads: 0.0,
            pos_ned_m: [8.0, 4.0, -12.0],
            vel_ned_ms: [1.5, -0.5, 0.0],
            accel_ned_mss: [0.2, 0.1, 0.0],
        }
    );
}

#[test]
fn run_disarmed_skips_offsets_and_has_no_tradheli_keep() {
    let mut view = FollowRunView::flying();
    view.disarmed_or_landed = true;
    view.yaw_behave = FollowYawBehave::SameAsLeadVehicle;
    assert_eq!(follow_run(&view).exit, FollowRunExit::Disarmed);
}

#[test]
fn run_invalid_ofs_holds_zero_vel_accel_and_keeps_att_yaw() {
    let mut view = FollowRunView::flying();
    view.ofs_valid = false;
    view.yaw_behave = FollowYawBehave::SameAsLeadVehicle;
    match follow_run(&view).exit {
        FollowRunExit::Flew {
            init_offsets,
            input,
            yaw_source,
            yaw_rad,
            yaw_rate_rads,
            pos_ned_m,
            vel_ned_ms,
            accel_ned_mss,
        } => {
            assert!(init_offsets);
            assert_eq!(input, FollowPosInput::VelAccelHold);
            assert_eq!(yaw_source, FollowYawSource::AttTarget);
            assert_eq!(yaw_rad.to_bits(), 0.3f32.to_bits());
            assert_eq!(yaw_rate_rads.to_bits(), 0.0f32.to_bits());
            assert_eq!(pos_ned_m, [0.0, 0.0, 0.0]);
            assert_eq!(vel_ned_ms, [0.0, 0.0, 0.0]);
            assert_eq!(accel_ned_mss, [0.0, 0.0, 0.0]);
        }
        other => panic!("expected Flew, got {other:?}"),
    }
}

#[test]
fn run_same_as_lead_takes_radians_of_heading_and_rate() {
    let mut view = FollowRunView::flying();
    view.yaw_behave = FollowYawBehave::SameAsLeadVehicle;
    match follow_run(&view).exit {
        FollowRunExit::Flew {
            yaw_source,
            yaw_rad,
            yaw_rate_rads,
            input,
            ..
        } => {
            assert_eq!(yaw_source, FollowYawSource::SameAsLead);
            assert_eq!(input, FollowPosInput::PosVelAccel);
            assert_eq!(yaw_rad.to_bits(), radians(90.0f32).to_bits());
            assert_eq!(yaw_rate_rads.to_bits(), radians(10.0f32).to_bits());
        }
        other => panic!("expected Flew, got {other:?}"),
    }
}

#[test]
fn run_face_lead_uses_lead_minus_pos_target_when_beyond_one_metre() {
    let mut view = FollowRunView::flying();
    view.yaw_behave = FollowYawBehave::FaceLeadVehicle;
    view.lead_pos_ned_m = [10.0, 0.0, -12.0];
    view.pos_target_ned_m = [0.0, 0.0, -12.0];
    let expected = Vector2f::new(10.0, 0.0).angle();
    match follow_run(&view).exit {
        FollowRunExit::Flew {
            yaw_source,
            yaw_rad,
            yaw_rate_rads,
            ..
        } => {
            assert_eq!(yaw_source, FollowYawSource::FaceLead);
            assert_eq!(yaw_rad.to_bits(), expected.to_bits());
            assert_eq!(yaw_rate_rads.to_bits(), 0.0f32.to_bits());
        }
        other => panic!("expected Flew, got {other:?}"),
    }
}

#[test]
fn run_face_lead_east_is_half_pi() {
    let mut view = FollowRunView::flying();
    view.yaw_behave = FollowYawBehave::FaceLeadVehicle;
    view.lead_pos_ned_m = [0.0, 10.0, -12.0];
    view.pos_target_ned_m = [0.0, 0.0, -12.0];
    let expected = Vector2f::new(0.0, 10.0).angle();
    match follow_run(&view).exit {
        FollowRunExit::Flew {
            yaw_source,
            yaw_rad,
            ..
        } => {
            assert_eq!(yaw_source, FollowYawSource::FaceLead);
            assert_eq!(yaw_rad.to_bits(), expected.to_bits());
        }
        other => panic!("expected Flew, got {other:?}"),
    }
}

#[test]
fn run_face_lead_at_exactly_one_metre_keeps_att_yaw() {
    let mut view = FollowRunView::flying();
    view.yaw_behave = FollowYawBehave::FaceLeadVehicle;
    view.lead_pos_ned_m = [1.0, 0.0, -12.0];
    view.pos_target_ned_m = [0.0, 0.0, -12.0];
    match follow_run(&view).exit {
        FollowRunExit::Flew {
            yaw_source,
            yaw_rad,
            ..
        } => {
            assert_eq!(yaw_source, FollowYawSource::AttTarget);
            assert_eq!(yaw_rad.to_bits(), 0.3f32.to_bits());
        }
        other => panic!("expected Flew, got {other:?}"),
    }
}

#[test]
fn run_face_lead_missing_lead_pva_keeps_att_yaw() {
    let mut view = FollowRunView::flying();
    view.yaw_behave = FollowYawBehave::FaceLeadVehicle;
    view.target_pva_ok = false;
    view.lead_pos_ned_m = [10.0, 0.0, -12.0];
    view.pos_target_ned_m = [0.0, 0.0, -12.0];
    match follow_run(&view).exit {
        FollowRunExit::Flew {
            yaw_source,
            yaw_rad,
            ..
        } => {
            assert_eq!(yaw_source, FollowYawSource::AttTarget);
            assert_eq!(yaw_rad.to_bits(), 0.3f32.to_bits());
        }
        other => panic!("expected Flew, got {other:?}"),
    }
}

#[test]
fn run_dir_of_flight_uses_ofs_vel_when_faster_than_one() {
    let mut view = FollowRunView::flying();
    view.yaw_behave = FollowYawBehave::DirOfFlight;
    view.vel_ofs_ned_ms = [2.0, 0.0, 0.3];
    let expected = Vector2f::new(2.0, 0.0).angle();
    match follow_run(&view).exit {
        FollowRunExit::Flew {
            yaw_source,
            yaw_rad,
            yaw_rate_rads,
            ..
        } => {
            assert_eq!(yaw_source, FollowYawSource::DirOfFlight);
            assert_eq!(yaw_rad.to_bits(), expected.to_bits());
            assert_eq!(yaw_rate_rads.to_bits(), 0.0f32.to_bits());
        }
        other => panic!("expected Flew, got {other:?}"),
    }
}

#[test]
fn run_dir_of_flight_at_exactly_one_keeps_att_yaw() {
    let mut view = FollowRunView::flying();
    view.yaw_behave = FollowYawBehave::DirOfFlight;
    view.vel_ofs_ned_ms = [1.0, 0.0, 0.0];
    match follow_run(&view).exit {
        FollowRunExit::Flew {
            yaw_source,
            yaw_rad,
            ..
        } => {
            assert_eq!(yaw_source, FollowYawSource::AttTarget);
            assert_eq!(yaw_rad.to_bits(), 0.3f32.to_bits());
        }
        other => panic!("expected Flew, got {other:?}"),
    }
}

#[test]
fn run_unknown_yaw_behave_is_none() {
    let mut view = FollowRunView::flying();
    view.yaw_behave = FollowYawBehave::None;
    view.target_heading_deg = 180.0;
    match follow_run(&view).exit {
        FollowRunExit::Flew {
            yaw_source,
            yaw_rad,
            yaw_rate_rads,
            ..
        } => {
            assert_eq!(yaw_source, FollowYawSource::AttTarget);
            assert_eq!(yaw_rad.to_bits(), 0.3f32.to_bits());
            assert_eq!(yaw_rate_rads.to_bits(), 0.0f32.to_bits());
        }
        other => panic!("expected Flew, got {other:?}"),
    }
}
