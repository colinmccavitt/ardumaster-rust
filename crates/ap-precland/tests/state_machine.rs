//! `AC_PrecLand_StateMachine` leftover.
//!
//! Tracked as **COP-028**. `GCS_SEND_TEXT` stays a leftover flag.

use ap_math::scalar::is_equal;
use ap_math::vector3::Vector3f;
use ap_precland::{
    FailSafeAction, PrecLand, PrecLandParams, RetryAction, RetryStrictness, StateMachine,
    StateMachineFrontend, StateMachineWorld, Status, TargetState, FAILSAFE_INIT_TIMEOUT_MS,
    MAX_POS_ERROR_M, REMAINING, RETRY_BEHAVE_DEFAULT, RETRY_MAX_DEFAULT, RETRY_OFFSET_ALT_M,
    RETRY_TIMEOUT_S_DEFAULT, STRICT_DEFAULT,
};

fn almost(a: f32, b: f32) {
    assert!(is_equal(a, b), "{a} != {b}");
}

fn almost_vec(got: Vector3f, want: Vector3f) {
    assert!(
        is_equal(got.x, want.x) && is_equal(got.y, want.y) && is_equal(got.z, want.z),
        "({} {} {}) != ({} {} {})",
        got.x,
        got.y,
        got.z,
        want.x,
        want.y,
        want.z
    );
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

fn world_at(now_ms: u32, pos: Option<Vector3f>) -> StateMachineWorld {
    StateMachineWorld {
        now_ms,
        relative_pos_ned: pos,
    }
}

#[test]
fn discriminants_and_defaults_match_upstream() {
    assert_eq!(Status::Error as u8, 0);
    assert_eq!(Status::Descend as u8, 1);
    assert_eq!(Status::Retrying as u8, 2);
    assert_eq!(Status::Failsafe as u8, 3);
    assert_eq!(FailSafeAction::HoldPos as u8, 0);
    assert_eq!(FailSafeAction::Descend as u8, 1);
    assert_eq!(RetryStrictness::NotStrict as u8, 0);
    assert_eq!(RetryStrictness::Normal as u8, 1);
    assert_eq!(RetryStrictness::VeryStrict as u8, 2);
    assert_eq!(RetryAction::GoToLastLoc as u8, 0);
    assert_eq!(RetryAction::GoToTargetLoc as u8, 1);
    almost(MAX_POS_ERROR_M, 0.75);
    assert_eq!(FAILSAFE_INIT_TIMEOUT_MS, 7_000);
    almost(RETRY_OFFSET_ALT_M, 1.5);
    assert_eq!(STRICT_DEFAULT, RetryStrictness::Normal);
    assert_eq!(RETRY_MAX_DEFAULT, 4);
    almost(RETRY_TIMEOUT_S_DEFAULT, 4.0);
    assert_eq!(RETRY_BEHAVE_DEFAULT, RetryAction::GoToLastLoc);
}

#[test]
fn update_errors_when_disabled() {
    let mut sm = StateMachine::new();
    let mut frontend = lost(RetryStrictness::Normal);
    frontend.enabled = false;
    let out = sm.update(&frontend, &world_at(0, None));
    assert_eq!(out.status, Status::Error);
    assert!(out.retry_pos_m.is_none());
}

#[test]
fn found_and_out_of_range_descend_and_reset() {
    let mut sm = StateMachine::new();
    let mut frontend = lost(RetryStrictness::Normal);
    frontend.target_state = TargetState::Found;
    let out = sm.update(&frontend, &world_at(0, None));
    assert_eq!(out.status, Status::Descend);

    frontend.target_state = TargetState::OutOfRange;
    let out = sm.update(&frontend, &world_at(0, None));
    assert_eq!(out.status, Status::Descend);
}

#[test]
fn never_seen_is_failsafe() {
    let mut sm = StateMachine::new();
    let mut frontend = lost(RetryStrictness::Normal);
    frontend.target_state = TargetState::NeverSeen;
    let out = sm.update(&frontend, &world_at(0, None));
    assert_eq!(out.status, Status::Failsafe);
}

#[test]
fn not_strict_lands_vertically() {
    let mut sm = StateMachine::new();
    let frontend = lost(RetryStrictness::NotStrict);
    let first = sm.update(&frontend, &world_at(1_000, None));
    assert_eq!(first.status, Status::Descend);
    let later = sm.update(&frontend, &world_at(20_000, None));
    assert_eq!(later.status, Status::Descend);
    assert!(later.retry_pos_m.is_none());
    assert_eq!(sm.retry_count(), 0);
}

#[test]
fn recently_lost_descends_until_retry_timeout() {
    let mut sm = StateMachine::new();
    let frontend = lost(RetryStrictness::Normal);
    let init = sm.update(&frontend, &world_at(1_000, None));
    assert_eq!(init.status, Status::Descend);

    // 3999 ms after last valid target: still descending.
    let early = sm.update(&frontend, &world_at(4_999, None));
    assert_eq!(early.status, Status::Descend);
    assert_eq!(sm.retry_count(), 0);

    // 4000 ms: still the DESCEND tick that arms retry; next tick retries.
    let arm = sm.update(&frontend, &world_at(5_000, None));
    assert_eq!(arm.status, Status::Descend);

    let retry = sm.update(&frontend, &world_at(5_001, None));
    assert_eq!(retry.status, Status::Retrying);
    assert!(retry.need_gcs_retrying);
    assert_eq!(sm.retry_count(), 1);
    let pos = retry.retry_pos_m.expect("retry pos");
    // GO_TO_LAST_LOC = last vehicle pos, z -= 1.5
    almost_vec(pos, Vector3f::new(3.0, -1.0, 1.0));
}

#[test]
fn very_strict_also_retries_after_descend() {
    let mut sm = StateMachine::new();
    let frontend = lost(RetryStrictness::VeryStrict);
    let _ = sm.update(&frontend, &world_at(1_000, None));
    let _ = sm.update(&frontend, &world_at(5_000, None));
    let retry = sm.update(&frontend, &world_at(5_001, None));
    assert_eq!(retry.status, Status::Retrying);
}

#[test]
fn retry_uses_detected_target_loc() {
    let mut sm = StateMachine::new();
    let mut frontend = lost(RetryStrictness::Normal);
    frontend.retry_behaviour = RetryAction::GoToTargetLoc;
    let _ = sm.update(&frontend, &world_at(1_000, None));
    let _ = sm.update(&frontend, &world_at(5_000, None));
    let retry = sm.update(&frontend, &world_at(5_001, None));
    let pos = retry.retry_pos_m.expect("retry pos");
    almost_vec(pos, Vector3f::new(10.0, 4.0, 0.5));
}

#[test]
fn max_retry_zero_fails_immediately() {
    let mut sm = StateMachine::new();
    let mut frontend = lost(RetryStrictness::Normal);
    frontend.max_retry_allowed = 0;
    let _ = sm.update(&frontend, &world_at(1_000, None));
    let _ = sm.update(&frontend, &world_at(5_000, None));
    let out = sm.update(&frontend, &world_at(5_001, None));
    assert_eq!(out.status, Status::Failsafe);
    assert_eq!(sm.retry_count(), 0);
}

#[test]
fn retry_converges_descends_then_completes() {
    let mut sm = StateMachine::new();
    let frontend = lost(RetryStrictness::Normal);
    let _ = sm.update(&frontend, &world_at(1_000, None));
    let _ = sm.update(&frontend, &world_at(5_000, None));
    let start = sm.update(&frontend, &world_at(5_001, None));
    assert_eq!(start.status, Status::Retrying);
    assert!(start.need_gcs_retrying);

    // Far from the climb target (3, -1, 1).
    let far = sm.update(
        &frontend,
        &world_at(5_100, Some(Vector3f::new(20.0, 20.0, 1.0))),
    );
    assert_eq!(far.status, Status::Retrying);
    assert!(!far.need_gcs_retry_completed);

    // Close enough: switch to DESCEND, still RETRYING this tick.
    let close = sm.update(
        &frontend,
        &world_at(5_200, Some(Vector3f::new(3.0, -1.0, 1.0))),
    );
    assert_eq!(close.status, Status::Retrying);

    // Still high: retry pos is current xy, original detected z (2.5).
    let high = sm.update(
        &frontend,
        &world_at(5_300, Some(Vector3f::new(3.1, -1.1, 0.0))),
    );
    assert_eq!(high.status, Status::Retrying);
    let pos = high.retry_pos_m.expect("descend pos");
    almost_vec(pos, Vector3f::new(3.1, -1.1, 2.5));
    assert!(!high.need_gcs_retry_completed);

    // At original height: complete this tick, still RETRYING.
    let done = sm.update(
        &frontend,
        &world_at(5_400, Some(Vector3f::new(3.1, -1.1, 2.5))),
    );
    assert_eq!(done.status, Status::Retrying);
    assert!(done.need_gcs_retry_completed);

    let fail = sm.update(
        &frontend,
        &world_at(5_500, Some(Vector3f::new(3.1, -1.1, 2.5))),
    );
    assert_eq!(fail.status, Status::Failsafe);
}

#[test]
fn in_progress_errors_without_ahrs() {
    let mut sm = StateMachine::new();
    let frontend = lost(RetryStrictness::Normal);
    let _ = sm.update(&frontend, &world_at(1_000, None));
    let _ = sm.update(&frontend, &world_at(5_000, None));
    let _ = sm.update(&frontend, &world_at(5_001, None));
    let err = sm.update(&frontend, &world_at(5_100, None));
    assert_eq!(err.status, Status::Error);
    assert!(err.retry_pos_m.is_some());
}

#[test]
fn found_during_retry_resets_lost_machine_keeps_count() {
    let mut sm = StateMachine::new();
    let mut frontend = lost(RetryStrictness::Normal);
    let _ = sm.update(&frontend, &world_at(1_000, None));
    let _ = sm.update(&frontend, &world_at(5_000, None));
    let _ = sm.update(&frontend, &world_at(5_001, None));
    assert_eq!(sm.retry_count(), 1);

    frontend.target_state = TargetState::Found;
    let found = sm.update(&frontend, &world_at(5_200, None));
    assert_eq!(found.status, Status::Descend);
    assert_eq!(sm.retry_count(), 1);

    frontend.target_state = TargetState::RecentlyLost;
    frontend.last_valid_target_ms = 5_200;
    let again = sm.update(&frontend, &world_at(5_200, None));
    assert_eq!(again.status, Status::Descend);
    assert_eq!(sm.retry_count(), 1);
}

#[test]
fn init_resets_retry_count_only_when_enabled() {
    let mut sm = StateMachine::new();
    let mut frontend = lost(RetryStrictness::Normal);
    let _ = sm.update(&frontend, &world_at(1_000, None));
    let _ = sm.update(&frontend, &world_at(5_000, None));
    let _ = sm.update(&frontend, &world_at(5_001, None));
    assert_eq!(sm.retry_count(), 1);

    frontend.enabled = false;
    sm.init(&frontend);
    assert_eq!(sm.retry_count(), 1);

    frontend.enabled = true;
    sm.init(&frontend);
    assert_eq!(sm.retry_count(), 0);
}

#[test]
fn exhausts_retries_then_failsafe() {
    let mut sm = StateMachine::new();
    let mut frontend = lost(RetryStrictness::Normal);
    frontend.max_retry_allowed = 1;
    let _ = sm.update(&frontend, &world_at(1_000, None));
    let _ = sm.update(&frontend, &world_at(5_000, None));
    let first = sm.update(&frontend, &world_at(5_001, None));
    assert!(first.need_gcs_retrying);
    assert_eq!(sm.retry_count(), 1);

    // Finish the first retry.
    let _ = sm.update(
        &frontend,
        &world_at(5_200, Some(Vector3f::new(3.0, -1.0, 1.0))),
    );
    let _ = sm.update(
        &frontend,
        &world_at(5_300, Some(Vector3f::new(3.0, -1.0, 2.5))),
    );
    let done = sm.update(
        &frontend,
        &world_at(5_400, Some(Vector3f::new(3.0, -1.0, 2.5))),
    );
    assert_eq!(done.status, Status::Failsafe);

    // A later INIT increment would be count=2 > 1, but COMPLETE already
    // failed. Re-arm via a Found reset then lose the target again.
    frontend.target_state = TargetState::Found;
    let _ = sm.update(&frontend, &world_at(6_000, None));
    frontend.target_state = TargetState::RecentlyLost;
    frontend.last_valid_target_ms = 6_000;
    let _ = sm.update(&frontend, &world_at(6_000, None));
    let _ = sm.update(&frontend, &world_at(10_000, None));
    let second = sm.update(&frontend, &world_at(10_001, None));
    assert_eq!(second.status, Status::Retrying);
    assert!(!second.need_gcs_retrying);
    assert_eq!(sm.retry_count(), 2);

    let fail = sm.update(
        &frontend,
        &world_at(10_100, Some(Vector3f::new(3.0, -1.0, 1.0))),
    );
    assert_eq!(fail.status, Status::Failsafe);
}

#[test]
fn failsafe_actions_match_strictness() {
    let mut sm = StateMachine::new();
    let mut frontend = lost(RetryStrictness::VeryStrict);
    let first = sm.get_failsafe_actions(&frontend, &world_at(100, None));
    assert_eq!(first.action, FailSafeAction::HoldPos);
    assert!(first.need_gcs_failsafe);
    let again = sm.get_failsafe_actions(&frontend, &world_at(20_000, None));
    assert_eq!(again.action, FailSafeAction::HoldPos);
    assert!(!again.need_gcs_failsafe);

    sm.init(&frontend);
    frontend.retry_strictness = RetryStrictness::Normal;
    let hold = sm.get_failsafe_actions(&frontend, &world_at(0, None));
    assert_eq!(hold.action, FailSafeAction::HoldPos);
    assert!(hold.need_gcs_failsafe);
    let still = sm.get_failsafe_actions(&frontend, &world_at(6_999, None));
    assert_eq!(still.action, FailSafeAction::HoldPos);
    let descend = sm.get_failsafe_actions(&frontend, &world_at(7_000, None));
    assert_eq!(descend.action, FailSafeAction::Descend);

    sm.init(&frontend);
    frontend.retry_strictness = RetryStrictness::NotStrict;
    let land = sm.get_failsafe_actions(&frontend, &world_at(0, None));
    assert_eq!(land.action, FailSafeAction::Descend);
    assert!(land.need_gcs_failsafe);

    frontend.enabled = false;
    let disabled = sm.get_failsafe_actions(&frontend, &world_at(0, None));
    assert_eq!(disabled.action, FailSafeAction::Descend);
    assert!(!disabled.need_gcs_failsafe);
}

#[test]
fn precland_frontend_carries_retry_params() {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled: true,
        retry_strictness: RetryStrictness::VeryStrict,
        retry_max: 2,
        retry_timeout_s: 3.5,
        retry_behave: RetryAction::GoToTargetLoc,
        ..PrecLandParams::default()
    });
    let _ = plnd.init(400);
    assert_eq!(plnd.retry_strictness(), RetryStrictness::VeryStrict);
    assert_eq!(plnd.max_retry_allowed(), 2);
    almost(plnd.min_retry_time_sec(), 3.5);
    assert_eq!(plnd.retry_behaviour(), RetryAction::GoToTargetLoc);

    let frontend = plnd.state_machine_frontend();
    assert!(frontend.enabled);
    assert_eq!(frontend.target_state, TargetState::NeverSeen);
    assert_eq!(frontend.retry_strictness, RetryStrictness::VeryStrict);
    assert_eq!(frontend.max_retry_allowed, 2);
    almost(frontend.min_retry_time_sec, 3.5);
    assert_eq!(frontend.retry_behaviour, RetryAction::GoToTargetLoc);
}

#[test]
fn leftover_catalog_drops_statemachine() {
    assert!(REMAINING.is_empty());
    assert!(!REMAINING.contains(&"AC_PrecLand_StateMachine::init"));
    assert!(!REMAINING.contains(&"AC_PrecLand_StateMachine::update"));
    assert!(!REMAINING.contains(&"AC_PrecLand_StateMachine::get_target_lost_actions"));
    assert!(!REMAINING.contains(&"AC_PrecLand_StateMachine::retry_landing"));
    assert!(!REMAINING.contains(&"AC_PrecLand_StateMachine::get_failsafe_actions"));
    assert!(!REMAINING.contains(&"AC_PrecLand_IRLock::init(irlock)"));
    assert!(!REMAINING.contains(&"AC_PrecLand_SITL::init(AP::sitl)"));
    assert!(!REMAINING.contains(&"AC_PrecLand_SITL_Gazebo::init(irlock)"));
}
