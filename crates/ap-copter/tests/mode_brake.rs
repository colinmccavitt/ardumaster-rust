//! `ModeBrake` init / run leftovers, upstream `ArduCopter/mode_brake.cpp`.

use ap_copter::mode_brake::{
    brake_init, brake_mode_flags, brake_run, brake_timeout_elapsed, is_disarmed_or_landed,
    timeout_to_loiter_ms, BrakeInitView, BrakeRunView, BrakeTimeoutExit,
    BRAKE_MODE_DECEL_RATE_MSS, BRAKE_MODE_SPEED_Z_MS, MODE_NUMBER_ALT_HOLD, MODE_NUMBER_BRAKE,
    MODE_NUMBER_LOITER, MODE_REASON_BRAKE_TIMEOUT,
};
use ap_motors::spool::DesiredSpoolState;

#[test]
fn constants_match_upstream_defines() {
    assert_eq!(BRAKE_MODE_SPEED_Z_MS.to_bits(), 2.50f32.to_bits());
    assert_eq!(BRAKE_MODE_DECEL_RATE_MSS.to_bits(), 7.50f32.to_bits());
    assert_eq!(MODE_NUMBER_BRAKE, 17);
    assert_eq!(MODE_NUMBER_LOITER, 5);
    assert_eq!(MODE_NUMBER_ALT_HOLD, 2);
    assert_eq!(MODE_REASON_BRAKE_TIMEOUT, 12);
}

#[test]
fn flags_match_mode_h() {
    let flags = brake_mode_flags();
    assert_eq!(flags.mode_number, MODE_NUMBER_BRAKE);
    assert!(flags.requires_position);
    assert!(!flags.has_manual_throttle);
    assert!(!flags.allows_arming);
    assert!(flags.is_autopilot);
}

#[test]
fn init_always_succeeds_and_clears_timeout() {
    let view = BrakeInitView::hovering();
    let ignore = brake_init(&view, true);
    let checks = brake_init(&view, false);
    assert!(ignore.ok);
    assert!(checks.ok);
    assert_eq!(ignore.timeout_ms, 0);
    assert_eq!(checks.timeout_ms, 0);
    assert!(ignore.init_ne);
    assert!(!ignore.init_d);
}

#[test]
fn init_sizes_ne_to_current_ground_speed() {
    let view = BrakeInitView {
        vel_ne_ms: 12.0,
        d_is_active: false,
    };
    let out = brake_init(&view, false);
    assert_eq!(out.ne_speed_ms.to_bits(), 12.0f32.to_bits());
    assert_eq!(
        out.ne_accel_mss.to_bits(),
        BRAKE_MODE_DECEL_RATE_MSS.to_bits()
    );
    assert_eq!(out.d_speed_ms.to_bits(), BRAKE_MODE_SPEED_Z_MS.to_bits());
    assert_eq!(
        out.d_accel_mss.to_bits(),
        BRAKE_MODE_DECEL_RATE_MSS.to_bits()
    );
    assert!(out.init_d);
}

#[test]
fn init_does_not_reinit_an_active_d_controller() {
    let view = BrakeInitView {
        vel_ne_ms: 3.0,
        d_is_active: true,
    };
    let out = brake_init(&view, false);
    assert!(!out.init_d);
    assert!(out.init_ne);
}

#[test]
fn disarmed_or_landed_is_the_or_of_three_gates() {
    assert!(is_disarmed_or_landed(false, true, false));
    assert!(is_disarmed_or_landed(true, false, false));
    assert!(is_disarmed_or_landed(true, true, true));
    assert!(!is_disarmed_or_landed(true, true, false));
}

#[test]
fn ground_path_relaxes_d_and_skips_timeout() {
    for mut view in [
        BrakeRunView {
            armed: false,
            ..BrakeRunView::flying()
        },
        BrakeRunView {
            auto_armed: false,
            ..BrakeRunView::flying()
        },
        BrakeRunView {
            land_complete: true,
            ..BrakeRunView::flying()
        },
    ] {
        view.timeout_ms = 100;
        view.timeout_start_ms = 0;
        view.now_ms = 200;
        view.land_complete_maybe = true;
        let out = brake_run(&view);
        assert!(out.safe_ground);
        assert!(out.relax_d);
        assert_eq!(out.desired_spool, None);
        assert!(!out.soften_ne);
        assert!(!out.update_ne);
        assert!(!out.update_d);
        assert_eq!(out.timeout_exit, BrakeTimeoutExit::None);
    }
}

#[test]
fn flying_asks_unlimited_and_stops_in_place() {
    let out = brake_run(&BrakeRunView::flying());
    assert!(!out.safe_ground);
    assert!(!out.relax_d);
    assert_eq!(
        out.desired_spool,
        Some(DesiredSpoolState::ThrottleUnlimited)
    );
    assert!(!out.soften_ne);
    assert_eq!(out.vel_ne_ms.to_bits(), 0.0f32.to_bits());
    assert_eq!(out.accel_ne_mss.to_bits(), 0.0f32.to_bits());
    assert!(out.update_ne);
    assert_eq!(out.heading_rate_rads.to_bits(), 0.0f32.to_bits());
    assert_eq!(out.climb_rate_ms.to_bits(), 0.0f32.to_bits());
    assert!(out.update_d);
    assert_eq!(out.timeout_exit, BrakeTimeoutExit::None);
}

#[test]
fn maybe_landed_softens_ne() {
    let mut view = BrakeRunView::flying();
    view.land_complete_maybe = true;
    let out = brake_run(&view);
    assert!(out.soften_ne);
    assert_eq!(
        out.desired_spool,
        Some(DesiredSpoolState::ThrottleUnlimited)
    );
}

#[test]
fn timeout_zero_never_exits() {
    let mut view = BrakeRunView::flying();
    view.timeout_ms = 0;
    view.timeout_start_ms = 0;
    view.now_ms = u32::MAX;
    let out = brake_run(&view);
    assert_eq!(out.timeout_exit, BrakeTimeoutExit::None);
    assert!(!brake_timeout_elapsed(0, 0, u32::MAX));
}

#[test]
fn timeout_fires_on_equality() {
    let mut view = BrakeRunView::flying();
    view.timeout_ms = 5_000;
    view.timeout_start_ms = 1_000;
    view.now_ms = 6_000;
    let at = brake_run(&view);
    assert_eq!(at.timeout_exit, BrakeTimeoutExit::LoiterThenAltHold);

    view.now_ms = 5_999;
    let early = brake_run(&view);
    assert_eq!(early.timeout_exit, BrakeTimeoutExit::None);
}

#[test]
fn timeout_wraps_like_uint32() {
    // start near overflow; 100 ms later wraps past 0.
    assert!(brake_timeout_elapsed(100, u32::MAX - 10, 89));
    assert!(!brake_timeout_elapsed(100, u32::MAX - 10, 88));
}

#[test]
fn timeout_to_loiter_writes_start_and_duration() {
    assert_eq!(timeout_to_loiter_ms(12_345, 2_000), (12_345, 2_000));
    assert_eq!(timeout_to_loiter_ms(99, 0), (99, 0));
}
