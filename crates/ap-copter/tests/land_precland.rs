//! PrecLand mode leftovers, upstream `ArduCopter/mode.cpp`.
//!
//! Tracked as **COP-013**. Pins the Mode consumers of `AC_PrecLand`:
//! `land_run_normal_or_precland`, `precland_run`, `precland_retry_position`,
//! and the vertical-control override.

#![allow(
    clippy::float_cmp,
    reason = "these leftovers pin exact C++ constants and zero demands; an \
epsilon would hide a demand that is supposed to be literally nothing"
)]

use ap_copter::land::{land_descent, LandDescent, LandDescentConfig};
use ap_copter::land_horizontal::{LAND_CANCEL_TRIGGER_THR, THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND};
use ap_copter::land_precland::{
    doing_precision_landing, land_descent_precland_override, land_run_normal_or_precland,
    precland_retry_position, precland_run, LandOrPrecland, PrecLandRetryView, PrecLandRunAction,
    PrecLandVerticalView, PRECLAND_ACCEPTABLE_ERROR_M, PRECLAND_MIN_DESCENT_SPEED_MS,
    PRECLAND_SLOWDOWN_MEAS_Z_MAX_M, PRECLAND_SLOWDOWN_MEAS_Z_MIN_M, REMAINING, RETRY_POS_ACCEL_MSS,
    RETRY_POS_SPEED_MS,
};
use ap_math::scalar::is_equal;
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;
use ap_precland::{
    FailSafeAction, RetryAction, RetryStrictness, StateMachine, StateMachineFrontend,
    StateMachineWorld, Status, TargetState, FAILSAFE_INIT_TIMEOUT_MS, RETRY_OFFSET_ALT_M,
};

fn almost(a: f32, b: f32) {
    assert!(is_equal(a, b), "{a} != {b}");
}

fn descent_config() -> LandDescentConfig {
    LandDescentConfig {
        land_alt_low_m: 10.0,
        land_speed_high_ms: 0.0,
        land_speed_ms: 0.5,
        max_speed_down_ms: 1.5,
        pos_p_kp: 1.0,
        max_accel_mss: 2.5,
    }
}

fn base_descent() -> LandDescent {
    land_descent(false, 20.0, false, &descent_config(), 0.0025)
}

fn lost(strict: RetryStrictness) -> StateMachineFrontend {
    StateMachineFrontend {
        enabled: true,
        target_state: TargetState::RecentlyLost,
        retry_strictness: strict,
        last_valid_target_ms: 1_000,
        min_retry_time_sec: 4.0,
        max_retry_allowed: 4,
        retry_behaviour: RetryAction::GoToLastLoc,
        last_detected_landing_pos_ned_m: Vector3f::new(10.0, 4.0, 2.0),
        last_vehicle_pos_when_target_detected_ned_m: Vector3f::new(3.0, -1.0, 2.5),
    }
}

fn found() -> StateMachineFrontend {
    StateMachineFrontend {
        enabled: true,
        target_state: TargetState::Found,
        retry_strictness: RetryStrictness::Normal,
        last_valid_target_ms: 1_000,
        min_retry_time_sec: 4.0,
        max_retry_allowed: 4,
        retry_behaviour: RetryAction::GoToLastLoc,
        last_detected_landing_pos_ned_m: Vector3f::new(10.0, 4.0, 2.0),
        last_vehicle_pos_when_target_detected_ned_m: Vector3f::new(3.0, -1.0, 2.5),
    }
}

fn world_at(now_ms: u32, pos: Option<Vector3f>) -> StateMachineWorld {
    StateMachineWorld {
        now_ms,
        relative_pos_ned: pos,
    }
}

fn retry_view() -> PrecLandRetryView {
    PrecLandRetryView {
        has_valid_input: true,
        throttle_behavior: 0,
        filtered_throttle_control_in: 0.0,
        land_repositioning: false,
        target_roll_rad: 0.0,
        target_pitch_rad: 0.0,
        land_repo_active: false,
        retry_pos_ned_m: Vector3f::new(3.0, -1.0, 1.0),
    }
}

fn vertical_on_target() -> PrecLandVerticalView {
    PrecLandVerticalView {
        pause_descent: false,
        doing_precision_landing: true,
        target_pos_ne_m: Some(Vector2f::new(1.0, 2.0)),
        current_pos_ne_m: Vector2f::new(1.0, 2.0),
        max_horiz_pos_error_m: 2.5,
        target_pos_meas_ned_z_m: 5.0,
        do_fast_descend: false,
        land_speed_ms: 0.5,
    }
}

#[test]
fn remaining_catalog_is_empty() {
    assert!(REMAINING.is_empty());
}

#[test]
fn constants_match_upstream() {
    almost(PRECLAND_ACCEPTABLE_ERROR_M, 0.15);
    almost(PRECLAND_MIN_DESCENT_SPEED_MS, 0.1);
    almost(PRECLAND_SLOWDOWN_MEAS_Z_MIN_M, 0.35);
    almost(PRECLAND_SLOWDOWN_MEAS_Z_MAX_M, 2.0);
    almost(RETRY_POS_SPEED_MS, 0.0);
    almost(RETRY_POS_ACCEL_MSS, 10.0);
}

#[test]
fn pause_or_disabled_runs_normal_land() {
    assert_eq!(
        land_run_normal_or_precland(true, true),
        LandOrPrecland::Normal {
            pause_descent: true
        }
    );
    assert_eq!(
        land_run_normal_or_precland(false, false),
        LandOrPrecland::Normal {
            pause_descent: false
        }
    );
    assert_eq!(
        land_run_normal_or_precland(true, false),
        LandOrPrecland::Normal {
            pause_descent: true
        }
    );
}

#[test]
fn enabled_unpaused_hands_the_tick_to_precland_run() {
    assert_eq!(
        land_run_normal_or_precland(false, true),
        LandOrPrecland::PrecLand
    );
}

#[test]
fn a_repositioning_pilot_skips_the_state_machine() {
    let mut machine = StateMachine::new();
    let frontend = lost(RetryStrictness::VeryStrict);
    let world = world_at(0, None);
    let before = machine;

    let out = precland_run(true, &mut machine, &frontend, &world);

    assert_eq!(
        out.action,
        PrecLandRunAction::HorizAndVert {
            pause_descent: false
        }
    );
    assert!(!out.need_internal_error);
    assert!(!out.need_gcs_retrying);
    assert!(!out.need_gcs_failsafe);
    assert_eq!(machine, before, "the machine must not be stepped");
}

#[test]
fn found_target_descends_through_the_normal_pair() {
    let mut machine = StateMachine::new();
    machine.init(&found());
    let out = precland_run(false, &mut machine, &found(), &world_at(0, None));
    assert_eq!(
        out.action,
        PrecLandRunAction::HorizAndVert {
            pause_descent: false
        }
    );
    assert!(!out.need_internal_error);
    assert!(out.retry_pos_m.is_none());
}

#[test]
fn never_seen_failsafe_holds_while_strict_or_fresh() {
    let mut frontend = found();
    frontend.target_state = TargetState::NeverSeen;
    frontend.retry_strictness = RetryStrictness::Normal;

    let mut machine = StateMachine::new();
    machine.init(&frontend);
    let out = precland_run(false, &mut machine, &frontend, &world_at(0, None));
    assert_eq!(
        out.action,
        PrecLandRunAction::HorizAndVert {
            pause_descent: true
        }
    );
    assert!(out.need_gcs_failsafe);

    let later = precland_run(
        false,
        &mut machine,
        &frontend,
        &world_at(FAILSAFE_INIT_TIMEOUT_MS, None),
    );
    assert_eq!(
        later.action,
        PrecLandRunAction::HorizAndVert {
            pause_descent: false
        }
    );
    assert!(!later.need_gcs_failsafe);
}

#[test]
fn very_strict_failsafe_holds_forever() {
    let mut frontend = found();
    frontend.target_state = TargetState::NeverSeen;
    frontend.retry_strictness = RetryStrictness::VeryStrict;

    let mut machine = StateMachine::new();
    machine.init(&frontend);
    let out = precland_run(
        false,
        &mut machine,
        &frontend,
        &world_at(FAILSAFE_INIT_TIMEOUT_MS + 1, None),
    );
    assert_eq!(
        out.action,
        PrecLandRunAction::HorizAndVert {
            pause_descent: true
        }
    );
}

#[test]
fn not_strict_failsafe_descends() {
    let mut frontend = found();
    frontend.target_state = TargetState::NeverSeen;
    frontend.retry_strictness = RetryStrictness::NotStrict;

    let mut machine = StateMachine::new();
    machine.init(&frontend);
    let out = precland_run(false, &mut machine, &frontend, &world_at(0, None));
    assert_eq!(
        out.action,
        PrecLandRunAction::HorizAndVert {
            pause_descent: false
        }
    );
    assert_eq!(FailSafeAction::Descend as u8, 1);
}

#[test]
fn disabled_frontend_reports_error_then_descends() {
    let mut frontend = found();
    frontend.enabled = false;
    let mut machine = StateMachine::new();
    let out = precland_run(false, &mut machine, &frontend, &world_at(0, None));
    assert_eq!(
        out.action,
        PrecLandRunAction::HorizAndVert {
            pause_descent: false
        }
    );
    assert!(out.need_internal_error);
    assert_eq!(Status::Error as u8, 0);
}

#[test]
fn retrying_flies_the_state_machine_position() {
    let frontend = lost(RetryStrictness::Normal);
    let mut machine = StateMachine::new();
    machine.init(&frontend);

    // First lost tick is Init → Descend. Wait out PLND_TIMEOUT.
    let _ = precland_run(false, &mut machine, &frontend, &world_at(1_000, None));
    let _ = precland_run(
        false,
        &mut machine,
        &frontend,
        &world_at(1_000 + 4_000, None),
    );
    let out = precland_run(
        false,
        &mut machine,
        &frontend,
        &world_at(1_000 + 4_000, None),
    );

    assert_eq!(out.action, PrecLandRunAction::RetryPosition);
    assert!(out.need_gcs_retrying);
    let pos = out.retry_pos_m.expect("retry writes a location");
    almost(pos.x, 3.0);
    almost(pos.y, -1.0);
    almost(pos.z, 2.5 - RETRY_OFFSET_ALT_M);
}

#[test]
fn retry_position_always_runs_the_controllers() {
    let out = precland_retry_position(&retry_view());
    assert!(!out.cancel);
    assert!(!out.land_repo_active);
    assert!(!out.need_log_repo_active);
    assert!(!out.need_log_cancel);
    almost(out.retry_pos_ned_m.x, 3.0);
    almost(out.retry_speed_ms, 0.0);
    almost(out.retry_accel_mss, 10.0);
    assert!(out.update_ne);
    assert!(out.update_d);
    assert!(out.attitude);
}

#[test]
fn retry_lean_sets_repo_and_does_not_clear_on_release() {
    let mut view = retry_view();
    view.land_repositioning = true;
    view.target_roll_rad = 0.2;
    let grabbed = precland_retry_position(&view);
    assert!(grabbed.land_repo_active);
    assert!(grabbed.need_log_repo_active);

    view.land_repo_active = true;
    view.target_roll_rad = 0.0;
    view.target_pitch_rad = 0.0;
    let released = precland_retry_position(&view);
    assert!(released.land_repo_active);
    assert!(!released.need_log_repo_active);
}

#[test]
fn retry_lean_is_ignored_without_valid_input_or_repositioning() {
    let mut view = retry_view();
    view.target_roll_rad = 0.4;
    view.land_repositioning = false;
    assert!(!precland_retry_position(&view).land_repo_active);

    view.land_repositioning = true;
    view.has_valid_input = false;
    assert!(!precland_retry_position(&view).land_repo_active);
}

#[test]
fn retry_high_throttle_cancels() {
    let mut view = retry_view();
    view.throttle_behavior = THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND;
    view.filtered_throttle_control_in = LAND_CANCEL_TRIGGER_THR;
    assert!(
        !precland_retry_position(&view).cancel,
        "equality is not above the trigger"
    );

    view.filtered_throttle_control_in = LAND_CANCEL_TRIGGER_THR + 1.0;
    let out = precland_retry_position(&view);
    assert!(out.cancel);
    assert!(out.need_log_cancel);

    view.has_valid_input = false;
    assert!(!precland_retry_position(&view).cancel);
}

#[test]
fn doing_precision_landing_needs_all_three() {
    assert!(doing_precision_landing(false, true, true));
    assert!(!doing_precision_landing(true, true, true));
    assert!(!doing_precision_landing(false, false, true));
    assert!(!doing_precision_landing(false, true, false));
}

#[test]
fn paused_or_inactive_override_leaves_the_descent_alone() {
    let base = base_descent();
    let mut view = vertical_on_target();
    view.pause_descent = true;
    view.target_pos_ne_m = Some(Vector2f::new(40.0, 0.0));
    assert_eq!(land_descent_precland_override(base, &view), base);

    view.pause_descent = false;
    view.doing_precision_landing = false;
    assert_eq!(land_descent_precland_override(base, &view), base);
}

#[test]
fn too_far_from_the_target_holds_the_descent() {
    let base = base_descent();
    assert!(base.climb_rate_ms < 0.0);

    let mut view = vertical_on_target();
    view.target_pos_ne_m = Some(Vector2f::new(10.0, 0.0));
    view.current_pos_ne_m = Vector2f::new(0.0, 0.0);
    view.max_horiz_pos_error_m = 2.5;
    let held = land_descent_precland_override(base, &view);
    assert_eq!(held.climb_rate_ms, 0.0);
    assert_eq!(held.ignore_descent_limit, base.ignore_descent_limit);
}

#[test]
fn a_zero_xy_limit_never_holds() {
    let base = base_descent();
    let mut view = vertical_on_target();
    view.target_pos_ne_m = Some(Vector2f::new(40.0, 0.0));
    view.current_pos_ne_m = Vector2f::zero();
    view.max_horiz_pos_error_m = 0.0;
    view.target_pos_meas_ned_z_m = 5.0;
    let out = land_descent_precland_override(base, &view);
    assert_eq!(out.climb_rate_ms, base.climb_rate_ms);
}

#[test]
fn missing_target_position_counts_as_zero_error() {
    let base = base_descent();
    let mut view = vertical_on_target();
    view.target_pos_ne_m = None;
    view.max_horiz_pos_error_m = 2.5;
    view.target_pos_meas_ned_z_m = 5.0;
    let out = land_descent_precland_override(base, &view);
    assert_eq!(
        out.climb_rate_ms, base.climb_rate_ms,
        "zero error is inside the limit, so the hold must not fire"
    );
}

#[test]
fn near_the_ground_the_descent_crawls() {
    let base = base_descent();
    let mut view = vertical_on_target();
    view.target_pos_ne_m = Some(Vector2f::new(0.05, 0.0));
    view.current_pos_ne_m = Vector2f::zero();
    view.target_pos_meas_ned_z_m = 1.0;
    view.do_fast_descend = false;
    view.land_speed_ms = 0.5;

    let out = land_descent_precland_override(base, &view);
    let max_descent = 0.5 * 0.5;
    let error = 0.05;
    let slowdown = error * (max_descent / PRECLAND_ACCEPTABLE_ERROR_M);
    let want = (-PRECLAND_MIN_DESCENT_SPEED_MS).min(-max_descent + slowdown);
    almost(out.climb_rate_ms, want);
    assert!(out.climb_rate_ms <= -PRECLAND_MIN_DESCENT_SPEED_MS);
    assert_ne!(out.climb_rate_ms, base.climb_rate_ms);
}

#[test]
fn fast_descend_skips_the_near_ground_crawl() {
    let base = base_descent();
    let mut view = vertical_on_target();
    view.target_pos_meas_ned_z_m = 1.0;
    view.do_fast_descend = true;
    view.target_pos_ne_m = Some(Vector2f::new(0.05, 0.0));
    view.current_pos_ne_m = Vector2f::zero();
    assert_eq!(land_descent_precland_override(base, &view), base);
}

#[test]
fn slowdown_band_is_open_at_both_ends() {
    let base = base_descent();
    let mut view = vertical_on_target();
    view.target_pos_ne_m = Some(Vector2f::new(0.05, 0.0));
    view.current_pos_ne_m = Vector2f::zero();
    view.do_fast_descend = false;

    view.target_pos_meas_ned_z_m = PRECLAND_SLOWDOWN_MEAS_Z_MIN_M;
    assert_eq!(
        land_descent_precland_override(base, &view).climb_rate_ms,
        base.climb_rate_ms,
        "z == 0.35 is not inside the band"
    );

    view.target_pos_meas_ned_z_m = PRECLAND_SLOWDOWN_MEAS_Z_MAX_M;
    assert_eq!(
        land_descent_precland_override(base, &view).climb_rate_ms,
        base.climb_rate_ms,
        "z == 2.0 is not inside the band"
    );

    view.target_pos_meas_ned_z_m = 0.3501;
    assert_ne!(
        land_descent_precland_override(base, &view).climb_rate_ms,
        base.climb_rate_ms
    );
}

#[test]
fn a_large_horizontal_error_in_the_slowdown_band_still_descends() {
    let base = base_descent();
    let mut view = vertical_on_target();
    // Inside XY_DIST_MAX so the hold does not fire, but far enough that
    // the slowdown would otherwise command a climb.
    view.target_pos_ne_m = Some(Vector2f::new(0.4, 0.0));
    view.current_pos_ne_m = Vector2f::zero();
    view.max_horiz_pos_error_m = 2.5;
    view.target_pos_meas_ned_z_m = 1.0;
    view.land_speed_ms = 0.5;
    let out = land_descent_precland_override(base, &view);
    assert!(
        out.climb_rate_ms <= -PRECLAND_MIN_DESCENT_SPEED_MS,
        "the crawl must remain a descent, got {}",
        out.climb_rate_ms
    );
}
