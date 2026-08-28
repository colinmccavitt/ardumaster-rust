//! Terrain-following failsafe when terrain data is missing.
//!
//! Upstream `Copter::failsafe_terrain_check` / `failsafe_terrain_set_status`
//! / `failsafe_terrain_on_event` in `ArduCopter/events.cpp`, the 5 s window
//! `FS_TERRAIN_TIMEOUT_MS` in `ArduCopter/config.h`, and
//! `ModeRTL::restart_without_terrain` in `ArduCopter/mode_rtl.cpp`.
//!
//! A mode that [`requires_terrain_failsafe`] (AUTO / GUIDED / RTL) trips after
//! terrain lookups have been failing for strictly more than 5 s. The action
//! is RTL (`set_mode_RTL_or_land_with_pause`) unless the vehicle is already
//! in RTL, in which case follow is disabled (`restart_without_terrain`). A
//! grounded vehicle disarms instead. Recovery after 100 ms of persistent
//! success clears the failure stamps and the `failsafe.terrain` latch.
//!
//! Plane 4.7 has no matching vehicle-level event — missing data there only
//! drops `target_altitude.terrain_following`. This stub keeps the Copter
//! RTL / disable-follow table so FW-027 can share one missing-data path.
//! Radio / GCS / battery / short-long timers are left to their own modules.

use crate::mode_table::ModeNumber;

/// Upstream `FS_TERRAIN_TIMEOUT_MS` — missing data must persist this long.
pub const FS_TERRAIN_TIMEOUT_MS: u32 = 5_000;
/// Persistent-success window before `failsafe_terrain_set_status` clears.
///
/// Upstream: `now - failsafe.terrain_last_failure_ms > 100`.
pub const FS_TERRAIN_CLEAR_MS: u32 = 100;

/// Failure stamps plus the `failsafe.terrain` latch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TerrainFailsafeState {
    /// `failsafe.terrain_first_failure_ms` — first miss in the current run.
    pub first_failure_ms: u32,
    /// `failsafe.terrain_last_failure_ms` — most recent miss.
    pub last_failure_ms: u32,
    /// `failsafe.terrain` — the event has already fired.
    pub active: bool,
}

impl Default for TerrainFailsafeState {
    fn default() -> Self {
        Self {
            first_failure_ms: 0,
            last_failure_ms: 0,
            active: false,
        }
    }
}

/// Inputs for `failsafe_terrain_check` / `failsafe_terrain_on_event`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TerrainFailsafeInputs {
    /// Current failure stamps and latch.
    pub state: TerrainFailsafeState,
    /// `flightmode->requires_terrain_failsafe()`.
    pub requires_terrain: bool,
    /// `flightmode->mode_number() == Mode::Number::RTL`.
    pub in_rtl: bool,
    /// `should_disarm_on_failsafe()` — typically landed / not flying.
    pub should_disarm: bool,
}

impl Default for TerrainFailsafeInputs {
    fn default() -> Self {
        Self {
            state: TerrainFailsafeState::default(),
            requires_terrain: false,
            in_rtl: false,
            should_disarm: false,
        }
    }
}

/// What `failsafe_terrain_check` asks the vehicle to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TerrainFailsafeDecision {
    /// Stay put — no edge on the `failsafe.terrain` latch.
    Hold,
    /// `set_mode_RTL_or_land_with_pause(ModeReason::TERRAIN_FAILSAFE)`.
    Rtl,
    /// Already in RTL: `ModeRTL::restart_without_terrain`.
    DisableFollow,
    /// `arming.disarm(AP_Arming::Method::TERRAINFAILSAFE)`.
    Disarm,
    /// `failsafe.terrain` falling edge — data recovered, log resolved.
    Clear,
}

/// Upstream `Mode::requires_terrain_failsafe`.
///
/// AUTO, GUIDED, and RTL override the default `false` in `mode.h`. Other
/// modes do not raise the terrain event even if lookups are failing.
#[must_use]
pub const fn requires_terrain_failsafe(mode: ModeNumber) -> bool {
    matches!(
        mode,
        ModeNumber::Auto | ModeNumber::Guided | ModeNumber::Rtl
    )
}

/// True when `last - first` is strictly older than [`FS_TERRAIN_TIMEOUT_MS`].
///
/// Matches
/// `(failsafe.terrain_last_failure_ms - failsafe.terrain_first_failure_ms) > FS_TERRAIN_TIMEOUT_MS`.
/// The window is last-minus-first, not `now` minus first: a single miss
/// never trips until a later miss is stamped more than 5 s later.
#[must_use]
pub fn terrain_missing_past_timeout(first_failure_ms: u32, last_failure_ms: u32) -> bool {
    last_failure_ms.wrapping_sub(first_failure_ms) > FS_TERRAIN_TIMEOUT_MS
}

/// Update the failure stamps, upstream `failsafe_terrain_set_status`.
///
/// A miss records `last = now` and, on the first miss of a run, `first = now`.
/// Persistent success for more than [`FS_TERRAIN_CLEAR_MS`] zeros both.
#[must_use]
pub fn failsafe_terrain_set_status(
    now_ms: u32,
    data_ok: bool,
    first_failure_ms: u32,
    last_failure_ms: u32,
) -> (u32, u32) {
    if !data_ok {
        let first = if first_failure_ms == 0 {
            now_ms
        } else {
            first_failure_ms
        };
        return (first, now_ms);
    }
    if now_ms.wrapping_sub(last_failure_ms) > FS_TERRAIN_CLEAR_MS {
        (0, 0)
    } else {
        (first_failure_ms, last_failure_ms)
    }
}

/// Apply [`failsafe_terrain_set_status`] to a live state.
pub fn apply_terrain_status(state: &mut TerrainFailsafeState, now_ms: u32, data_ok: bool) {
    let (first, last) = failsafe_terrain_set_status(
        now_ms,
        data_ok,
        state.first_failure_ms,
        state.last_failure_ms,
    );
    state.first_failure_ms = first;
    state.last_failure_ms = last;
}

/// Resolve `failsafe_terrain_check` plus the `failsafe_terrain_on_event` table.
///
/// The check only fires on a latch edge. Entry uses the RTL / disable-follow
/// / disarm table; exit is [`TerrainFailsafeDecision::Clear`].
#[must_use]
pub fn check_terrain_failsafe(inp: &TerrainFailsafeInputs) -> TerrainFailsafeDecision {
    let timeout =
        terrain_missing_past_timeout(inp.state.first_failure_ms, inp.state.last_failure_ms);
    let trigger = timeout && inp.requires_terrain;
    if trigger == inp.state.active {
        return TerrainFailsafeDecision::Hold;
    }
    if !trigger {
        return TerrainFailsafeDecision::Clear;
    }
    if inp.should_disarm {
        TerrainFailsafeDecision::Disarm
    } else if inp.in_rtl {
        TerrainFailsafeDecision::DisableFollow
    } else {
        TerrainFailsafeDecision::Rtl
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timeout_and_clear_match_upstream() {
        assert_eq!(FS_TERRAIN_TIMEOUT_MS, 5_000);
        assert_eq!(FS_TERRAIN_CLEAR_MS, 100);
        assert!(requires_terrain_failsafe(ModeNumber::Auto));
        assert!(requires_terrain_failsafe(ModeNumber::Guided));
        assert!(requires_terrain_failsafe(ModeNumber::Rtl));
        assert!(!requires_terrain_failsafe(ModeNumber::Manual));
    }

    #[test]
    fn last_minus_first_is_exclusive_at_five_seconds() {
        assert!(!terrain_missing_past_timeout(1_000, 1_000 + 5_000));
        assert!(terrain_missing_past_timeout(1_000, 1_000 + 5_001));
    }
}
