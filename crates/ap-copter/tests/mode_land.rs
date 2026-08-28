//! `ModeLand` init / run leftovers, upstream `ArduCopter/mode_land.cpp`.

use ap_copter::auto_yaw::YawMode;
use ap_copter::land::LAND_WITH_DELAY_MS;
use ap_copter::land_horizontal::THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND;
use ap_copter::mode_land::{
    land_do_not_use_gps, land_init, land_mode_flags, land_run, land_use_pilot_yaw, land_with_pause,
    landing_with_gps, LandInitView, LandRunView, LandRunner, LAND_ALT_LOW_M_DEFAULT,
    LAND_SPD_HIGH_MS_DEFAULT, LAND_SPD_MS_DEFAULT, MODE_NUMBER_ALT_HOLD, MODE_NUMBER_LAND,
    MODE_REASON_THROTTLE_LAND_ESCAPE,
};
use ap_motors::spool::{DesiredSpoolState, SpoolState};

#[test]
fn constants_match_upstream_defines() {
    assert_eq!(LAND_SPD_MS_DEFAULT.to_bits(), 0.5f32.to_bits());
    assert_eq!(LAND_SPD_HIGH_MS_DEFAULT.to_bits(), 0.0f32.to_bits());
    assert_eq!(LAND_ALT_LOW_M_DEFAULT.to_bits(), 10.0f32.to_bits());
    assert_eq!(LAND_WITH_DELAY_MS, 4_000);
    assert_eq!(MODE_NUMBER_LAND, 9);
    assert_eq!(MODE_NUMBER_ALT_HOLD, 2);
    assert_eq!(MODE_REASON_THROTTLE_LAND_ESCAPE, 9);
}

#[test]
fn flags_match_mode_h() {
    let flags = land_mode_flags();
    assert_eq!(flags.mode_number, MODE_NUMBER_LAND);
    assert!(!flags.requires_position);
    assert!(!flags.has_manual_throttle);
    assert!(!flags.allows_arming);
    assert!(flags.is_autopilot);
    assert!(flags.is_landing);
}

#[test]
fn init_always_succeeds_and_clears_pause() {
    let view = LandInitView::ready();
    let ignore = land_init(&view, true);
    let checks = land_init(&view, false);
    assert!(ignore.ok);
    assert!(checks.ok);
    assert!(!ignore.land_pause);
    assert!(!checks.land_pause);
    assert!(!ignore.land_repo_active);
    assert!(!ignore.prec_land_active);
    assert_eq!(ignore.yaw, YawMode::Hold);
    assert_eq!(ignore.land_start_ms, 1_000);
}

#[test]
fn init_latches_control_position_from_position_ok() {
    let ok = land_init(&LandInitView::ready(), false);
    assert!(ok.control_position);

    let mut view = LandInitView::ready();
    view.position_ok = false;
    let no_gps = land_init(&view, false);
    assert!(!no_gps.control_position);
    assert!(!no_gps.init_ne);
}

#[test]
fn init_sizes_controllers_from_wpnav_and_skips_active_ones() {
    let view = LandInitView::ready();
    let out = land_init(&view, false);
    assert_eq!(out.ne_speed_ms.to_bits(), 5.0f32.to_bits());
    assert_eq!(out.ne_accel_mss.to_bits(), 1.0f32.to_bits());
    assert!(!out.init_ne);
    assert_eq!(out.d_speed_down_ms.to_bits(), 1.5f32.to_bits());
    assert_eq!(out.d_speed_up_ms.to_bits(), 2.5f32.to_bits());
    assert_eq!(out.d_accel_mss.to_bits(), 2.5f32.to_bits());
    assert!(!out.init_d);
}

#[test]
fn init_reinits_inactive_ne_only_when_position_ok() {
    let mut view = LandInitView::ready();
    view.ne_is_active = false;
    view.d_is_active = false;
    let gps = land_init(&view, false);
    assert!(gps.init_ne);
    assert!(gps.init_d);

    view.position_ok = false;
    let no_gps = land_init(&view, false);
    assert!(!no_gps.init_ne);
    assert!(no_gps.init_d);
}

#[test]
fn gps_run_flies_normal_or_precland() {
    let out = land_run(&LandRunView::flying());
    assert_eq!(out.runner, LandRunner::Gps);
    assert!(!out.disarm_landed);
    assert!(!out.safe_ground);
    assert_eq!(
        out.desired_spool,
        Some(DesiredSpoolState::ThrottleUnlimited)
    );
    assert!(out.land_normal_or_precland);
    assert!(!out.land_vertical);
    assert!(!out.attitude);
    assert!(!out.cancel_to_althold);
}

#[test]
fn nogps_run_flies_vertical_and_always_runs_attitude() {
    let mut view = LandRunView::flying();
    view.control_position = false;
    let out = land_run(&view);
    assert_eq!(out.runner, LandRunner::NoGps);
    assert!(out.land_vertical);
    assert!(!out.land_normal_or_precland);
    assert!(out.attitude);
    assert!(!out.use_pilot_lean);
}

#[test]
fn nogps_ground_path_still_runs_attitude() {
    let mut view = LandRunView::flying();
    view.control_position = false;
    view.land_complete = true;
    let out = land_run(&view);
    assert!(out.safe_ground);
    assert_eq!(out.desired_spool, None);
    assert!(!out.land_vertical);
    assert!(out.attitude);
}

#[test]
fn landed_and_ground_idle_asks_disarm() {
    let mut view = LandRunView::flying();
    view.land_complete = true;
    view.spool_state = SpoolState::GroundIdle;
    let out = land_run(&view);
    assert!(out.disarm_landed);
    assert!(out.safe_ground);
}

#[test]
fn landed_but_not_ground_idle_does_not_disarm() {
    let mut view = LandRunView::flying();
    view.land_complete = true;
    view.spool_state = SpoolState::ThrottleUnlimited;
    let out = land_run(&view);
    assert!(!out.disarm_landed);
    assert!(out.safe_ground);
}

#[test]
fn pause_expires_on_the_flying_path_at_equality() {
    let mut view = LandRunView::flying();
    view.land_pause = true;
    view.land_start_ms = 1_000;
    view.now_ms = 1_000 + LAND_WITH_DELAY_MS;
    let out = land_run(&view);
    assert!(out.pause_cleared);
    assert!(!out.land_pause);
    assert!(out.land_normal_or_precland);
}

#[test]
fn pause_does_not_expire_one_millisecond_early() {
    let mut view = LandRunView::flying();
    view.land_pause = true;
    view.land_start_ms = 1_000;
    view.now_ms = 1_000 + LAND_WITH_DELAY_MS - 1;
    let out = land_run(&view);
    assert!(!out.pause_cleared);
    assert!(out.land_pause);
}

#[test]
fn pause_does_not_clear_on_the_ground_path() {
    let mut view = LandRunView::flying();
    view.land_complete = true;
    view.land_pause = true;
    view.land_start_ms = 0;
    view.now_ms = LAND_WITH_DELAY_MS + 10_000;
    let out = land_run(&view);
    assert!(out.safe_ground);
    assert!(!out.pause_cleared);
    assert!(out.land_pause);
}

#[test]
fn pause_uses_unsigned_wrap() {
    let mut view = LandRunView::flying();
    view.land_pause = true;
    view.land_start_ms = u32::MAX - 10;
    view.now_ms = LAND_WITH_DELAY_MS - 11;
    let out = land_run(&view);
    assert!(out.pause_cleared);
    assert!(!out.land_pause);
}

#[test]
fn nogps_throttle_cancel_goes_to_althold_and_keeps_landing() {
    let mut view = LandRunView::flying();
    view.control_position = false;
    view.throttle_behavior = THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND;
    view.filtered_throttle_control_in = 701.0;
    let out = land_run(&view);
    assert!(out.cancel_to_althold);
    assert!(out.land_vertical);
    assert!(out.attitude);
}

#[test]
fn nogps_throttle_cancel_needs_the_behave_bit() {
    let mut view = LandRunView::flying();
    view.control_position = false;
    view.throttle_behavior = 0;
    view.filtered_throttle_control_in = 701.0;
    let out = land_run(&view);
    assert!(!out.cancel_to_althold);
}

#[test]
fn nogps_reads_pilot_lean_only_when_repositioning() {
    let mut view = LandRunView::flying();
    view.control_position = false;
    view.land_repositioning = true;
    let out = land_run(&view);
    assert!(out.use_pilot_lean);

    view.has_valid_input = false;
    let no_rc = land_run(&view);
    assert!(!no_rc.use_pilot_lean);
    assert!(!no_rc.cancel_to_althold);
}

#[test]
fn do_not_use_gps_clears_the_position_latch() {
    assert!(!land_do_not_use_gps());
}

#[test]
fn use_pilot_yaw_is_repositioning() {
    assert!(land_use_pilot_yaw(true));
    assert!(!land_use_pilot_yaw(false));
}

#[test]
fn land_with_pause_sets_pause_and_notify() {
    let out = land_with_pause();
    assert_eq!(out.mode_number, MODE_NUMBER_LAND);
    assert!(out.land_pause);
    assert!(out.failsafe_mode_change);
}

#[test]
fn landing_with_gps_needs_land_and_the_latch() {
    assert!(landing_with_gps(MODE_NUMBER_LAND, true));
    assert!(!landing_with_gps(MODE_NUMBER_LAND, false));
    assert!(!landing_with_gps(MODE_NUMBER_ALT_HOLD, true));
}
