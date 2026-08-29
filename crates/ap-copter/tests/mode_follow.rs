//! `ModeFollow` init leftover, upstream `ArduCopter/mode_follow.cpp`.

use ap_copter::mode_follow::{
    follow_init, follow_mode_flags, follow_option_is_enabled, FollowInitView,
    FOLLOW_OPTION_MOUNT_FOLLOW_ON_ENTER,
};
use ap_copter::mode_guided::{
    guided_mode_flags, GuidedYawAction, MODE_NUMBER_FOLLOW, MODE_NUMBER_GUIDED,
    WPNAV_ACCELERATION_MSS, WP_ACC_Z_DEFAULT_MSS, WP_SPD_DEFAULT_MS, WP_SPD_DOWN_DEFAULT_MS,
    WP_SPD_UP_DEFAULT_MS,
};

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
