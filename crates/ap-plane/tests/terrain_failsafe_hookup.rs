//! Terrain failsafe when following data is missing — RTL / disable-follow.
//!
//! Upstream `Copter::failsafe_terrain_check` / `failsafe_terrain_on_event`:
//! AUTO / GUIDED / RTL trip after `FS_TERRAIN_TIMEOUT_MS` of failed lookups.
//! Already-RTL disables follow (`restart_without_terrain`); otherwise RTL.

use ap_plane::mode_table::ModeNumber;
use ap_plane::terrain_failsafe_hookup::{
    apply_terrain_status, check_terrain_failsafe, failsafe_terrain_set_status,
    requires_terrain_failsafe, terrain_missing_past_timeout, TerrainFailsafeDecision,
    TerrainFailsafeInputs, TerrainFailsafeState, FS_TERRAIN_CLEAR_MS, FS_TERRAIN_TIMEOUT_MS,
};

fn following_auto_after(first_ms: u32, last_ms: u32) -> TerrainFailsafeInputs {
    TerrainFailsafeInputs {
        state: TerrainFailsafeState {
            first_failure_ms: first_ms,
            last_failure_ms: last_ms,
            active: false,
        },
        requires_terrain: true,
        in_rtl: false,
        should_disarm: false,
    }
}

#[test]
fn timeout_and_mode_table_match_upstream() {
    assert_eq!(FS_TERRAIN_TIMEOUT_MS, 5_000);
    assert_eq!(FS_TERRAIN_CLEAR_MS, 100);
    assert!(requires_terrain_failsafe(ModeNumber::Auto));
    assert!(requires_terrain_failsafe(ModeNumber::Guided));
    assert!(requires_terrain_failsafe(ModeNumber::Rtl));
    for mode in [
        ModeNumber::Manual,
        ModeNumber::FlyByWireA,
        ModeNumber::Cruise,
        ModeNumber::Loiter,
        ModeNumber::Circle,
        ModeNumber::QLand,
    ] {
        assert!(
            !requires_terrain_failsafe(mode),
            "{mode:?} must not require the terrain event"
        );
    }
}

#[test]
fn first_miss_stamps_both_times_later_miss_keeps_first() {
    let (first, last) = failsafe_terrain_set_status(1_000, false, 0, 0);
    assert_eq!((first, last), (1_000, 1_000));
    let (first, last) = failsafe_terrain_set_status(2_500, false, first, last);
    assert_eq!((first, last), (1_000, 2_500));
}

#[test]
fn success_clears_only_after_one_hundred_ms() {
    let (first, last) = failsafe_terrain_set_status(1_000, false, 0, 0);
    let at_deadline = failsafe_terrain_set_status(1_000 + 100, true, first, last);
    assert_eq!(at_deadline, (1_000, 1_000));
    let past = failsafe_terrain_set_status(1_000 + 101, true, first, last);
    assert_eq!(past, (0, 0));
}

#[test]
fn timeout_is_last_minus_first_exclusive_at_five_seconds() {
    let first = 2_000;
    assert!(!terrain_missing_past_timeout(first, first));
    assert!(!terrain_missing_past_timeout(
        first,
        first + FS_TERRAIN_TIMEOUT_MS
    ));
    assert!(terrain_missing_past_timeout(
        first,
        first + FS_TERRAIN_TIMEOUT_MS + 1
    ));
}

#[test]
fn healthy_or_fresh_miss_never_enters() {
    let healthy = TerrainFailsafeInputs::default();
    assert_eq!(
        check_terrain_failsafe(&healthy),
        TerrainFailsafeDecision::Hold
    );

    let mut state = TerrainFailsafeState::default();
    apply_terrain_status(&mut state, 100, false);
    let inp = TerrainFailsafeInputs {
        state,
        requires_terrain: true,
        in_rtl: false,
        should_disarm: false,
    };
    assert_eq!(check_terrain_failsafe(&inp), TerrainFailsafeDecision::Hold);
}

#[test]
fn requires_terrain_holds_until_deadline_then_rtl() {
    let first = 1_000;
    let at_deadline = following_auto_after(first, first + 5_000);
    assert_eq!(
        check_terrain_failsafe(&at_deadline),
        TerrainFailsafeDecision::Hold
    );
    let past = following_auto_after(first, first + 5_001);
    assert_eq!(check_terrain_failsafe(&past), TerrainFailsafeDecision::Rtl);
}

#[test]
fn mode_without_terrain_requirement_never_trips() {
    let mut inp = following_auto_after(0, 5_001);
    inp.requires_terrain = false;
    assert_eq!(check_terrain_failsafe(&inp), TerrainFailsafeDecision::Hold);
}

#[test]
fn already_rtl_disables_follow_instead_of_reentering_rtl() {
    let mut inp = following_auto_after(0, 5_001);
    inp.in_rtl = true;
    assert_eq!(
        check_terrain_failsafe(&inp),
        TerrainFailsafeDecision::DisableFollow
    );
}

#[test]
fn grounded_vehicle_disarms_before_rtl_or_disable_follow() {
    let mut inp = following_auto_after(0, 5_001);
    inp.should_disarm = true;
    inp.in_rtl = true;
    assert_eq!(
        check_terrain_failsafe(&inp),
        TerrainFailsafeDecision::Disarm
    );
}

#[test]
fn already_active_holds_until_stamps_clear() {
    let mut inp = following_auto_after(0, 5_001);
    inp.state.active = true;
    assert_eq!(check_terrain_failsafe(&inp), TerrainFailsafeDecision::Hold);

    inp.state.first_failure_ms = 0;
    inp.state.last_failure_ms = 0;
    assert_eq!(check_terrain_failsafe(&inp), TerrainFailsafeDecision::Clear);
}

#[test]
fn apply_status_then_check_walks_the_upstream_path() {
    let mut state = TerrainFailsafeState::default();
    apply_terrain_status(&mut state, 10_000, false);
    apply_terrain_status(&mut state, 15_000, false);
    let at_five = TerrainFailsafeInputs {
        state,
        requires_terrain: true,
        in_rtl: false,
        should_disarm: false,
    };
    assert_eq!(
        check_terrain_failsafe(&at_five),
        TerrainFailsafeDecision::Hold
    );

    apply_terrain_status(&mut state, 15_001, false);
    let past = TerrainFailsafeInputs {
        state,
        requires_terrain: true,
        in_rtl: false,
        should_disarm: false,
    };
    assert_eq!(check_terrain_failsafe(&past), TerrainFailsafeDecision::Rtl);
}
