//! FW-027 failsafe event-dispatcher completeness: events.cpp / failsafe.cpp
//! hookups already on main vs remaining gates.
//!
//! Catalogs the Plane failsafe port. Items marked [`PortStatus::OnMain`]
//! landed in earlier slices and must not be redone. [`PortStatus::ThisSlice`]
//! is the FENCE_ACTION 8 AUTOLAND-or-RTL stub. [`PortStatus::Remaining`]
//! are still-open `events.cpp` / `failsafe.cpp` gaps (Q_OPTIONS RTL/QRTL).
//! This slice adds `FENCE_ACTION` 8 AUTOLAND-or-RTL.
//!
//! The emergency-landing gate wraps [`crate::failsafe_action_hookup`] the same
//! way [`crate::failsafe_in_landing_sequence_hookup`] wraps the landing
//! sequence. It does not rewrite the `FS_*` / check / off-event modules.

use crate::failsafe_action_hookup::{
    long_failsafe_action, short_failsafe_action, FailsafeActionLong, FailsafeActionResult,
    FailsafeActionShort,
};
use crate::mode_table::ModeNumber;

/// `Q_OPTIONS` bit 5, `QuadPlane::Option::FS_QRTL` — still remaining.
pub const Q_OPTIONS_FS_QRTL: u32 = 1 << 5;
/// `Q_OPTIONS` bit 20, `QuadPlane::Option::FS_RTL` — still remaining.
pub const Q_OPTIONS_FS_RTL: u32 = 1 << 20;
/// Plane `FENCE_ACTION` / `AC_Fence::Action::AUTOLAND_OR_RTL`.
pub const FENCE_ACTION_AUTOLAND_OR_RTL: u8 = 8;

/// Whether a catalog row is already hooked up or left for later work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortStatus {
    /// Present on `main` before this closing slice.
    OnMain,
    /// Added by this FW-027 slice (`FENCE_ACTION` 8 AUTOLAND-or-RTL).
    ThisSlice,
    /// Still deferred (`events.cpp` / `failsafe.cpp` leftover).
    Remaining,
}

/// One failsafe dispatcher surface in the completeness table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FailsafePortItem {
    /// Hookup or gate name.
    pub name: &'static str,
    /// Hooked up on main / this slice, or remaining.
    pub status: PortStatus,
    /// Short note (upstream symbol or why remaining).
    pub note: &'static str,
}

/// Completeness table: hooked-up failsafe dispatcher vs remaining events.cpp gaps.
pub const FAILSAFE_DISPATCHER_COMPLETENESS: &[FailsafePortItem] = &[
    FailsafePortItem {
        name: "FS_THR / THR_FS_VALUE",
        status: PortStatus::OnMain,
        note: "rc_failsafe_scheduler_hookup PWM-threshold failsafe",
    },
    FailsafePortItem {
        name: "rc_failsafe_scheduler_hookup",
        status: PortStatus::OnMain,
        note: "Plane::read_radio / rc().in_rc_failsafe()",
    },
    FailsafePortItem {
        name: "FS_SHORT_ACTN / FS_LONG_ACTN",
        status: PortStatus::OnMain,
        note: "failsafe_action_hookup / rc_failsafe_short_on_event / failsafe_long_on_event",
    },
    FailsafePortItem {
        name: "FS_GCS_ENABL",
        status: PortStatus::OnMain,
        note: "gcs_failsafe_hookup heartbeat timeout",
    },
    FailsafePortItem {
        name: "FS_BATT_ENABLE",
        status: PortStatus::OnMain,
        note: "battery_failsafe_hookup Land / RTL / Terminate",
    },
    FailsafePortItem {
        name: "FS_LONG_TIMEOUT",
        status: PortStatus::OnMain,
        note: "failsafe_long_timeout_hookup short-to-long promotion",
    },
    FailsafePortItem {
        name: "FS_SHORT_TIMEOUT",
        status: PortStatus::OnMain,
        note: "failsafe_short_timeout_hookup short-failsafe entry delay",
    },
    FailsafePortItem {
        name: "terrain failsafe",
        status: PortStatus::OnMain,
        note: "terrain_failsafe_hookup missing-data RTL / disable-follow",
    },
    FailsafePortItem {
        name: "geofence FENCE_ACTION",
        status: PortStatus::OnMain,
        note: "fence_failsafe_hookup Report / RTL / Guided / GuidedThrottlePass / Terminate",
    },
    FailsafePortItem {
        name: "failsafe_in_landing_sequence",
        status: PortStatus::OnMain,
        note: "failsafe_in_landing_sequence_hookup AUTO/AUTOLAND skip",
    },
    FailsafePortItem {
        name: "failsafe off-event recovery",
        status: PortStatus::OnMain,
        note: "failsafe_off_event_hookup short restore / long clear",
    },
    FailsafePortItem {
        name: "failsafe_check heartbeat",
        status: PortStatus::OnMain,
        note: "failsafe_check_hookup scheduler lockup / AFS calibration pulse",
    },
    FailsafePortItem {
        name: "ARSPD_FBW_MIN",
        status: PortStatus::OnMain,
        note: "airspeed_fbw_hookup already exists — do not redo",
    },
    FailsafePortItem {
        name: "CIRCLE/TAKEOFF/RTL no-short-action",
        status: PortStatus::OnMain,
        note: "failsafe_action_hookup ShortGroup::Never continues",
    },
    FailsafePortItem {
        name: "emergency-landing override",
        status: PortStatus::OnMain,
        note: "AUX_FUNC::EMERGENCY_LANDING_EN forces FBWA in stick / stick-or-hold",
    },
    FailsafePortItem {
        name: "completeness table",
        status: PortStatus::OnMain,
        note: "failsafe_event_dispatcher_completeness catalog",
    },
    FailsafePortItem {
        name: "Q_OPTIONS FS_RTL / FS_QRTL",
        status: PortStatus::Remaining,
        note: "QuadPlane::Option bits 20 / 5; Q modes still default QLAND",
    },
    FailsafePortItem {
        name: "FENCE_ACTION 8 AUTOLAND-or-RTL",
        status: PortStatus::ThisSlice,
        note: "AC_Fence::Action::AUTOLAND_OR_RTL; Autoland if available else RTL",
    },
];

/// Rows already hooked up on `main` (must not be redone).
#[must_use]
pub fn on_main_items() -> impl Iterator<Item = &'static FailsafePortItem> {
    FAILSAFE_DISPATCHER_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::OnMain)
}

/// Rows added by this closing slice.
#[must_use]
pub fn this_slice_items() -> impl Iterator<Item = &'static FailsafePortItem> {
    FAILSAFE_DISPATCHER_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::ThisSlice)
}

/// Rows left for Q_OPTIONS FS_RTL / FS_QRTL (not blocking this closer).
#[must_use]
pub fn remaining_items() -> impl Iterator<Item = &'static FailsafePortItem> {
    FAILSAFE_DISPATCHER_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::Remaining)
}

/// Count rows in each status bucket.
#[must_use]
pub fn completeness_counts() -> (usize, usize, usize) {
    let mut on_main = 0;
    let mut this_slice = 0;
    let mut remaining = 0;
    for item in FAILSAFE_DISPATCHER_COMPLETENESS {
        match item.status {
            PortStatus::OnMain => on_main += 1,
            PortStatus::ThisSlice => this_slice += 1,
            PortStatus::Remaining => remaining += 1,
        }
    }
    (on_main, this_slice, remaining)
}

/// True when `name` is listed with `status`.
#[must_use]
pub fn completeness_has(name: &str, status: PortStatus) -> bool {
    FAILSAFE_DISPATCHER_COMPLETENESS
        .iter()
        .any(|item| item.name == name && item.status == status)
}

/// True when every name in the table appears once.
#[must_use]
pub fn completeness_unique_names() -> bool {
    for (i, item) in FAILSAFE_DISPATCHER_COMPLETENESS.iter().enumerate() {
        for other in FAILSAFE_DISPATCHER_COMPLETENESS.iter().skip(i + 1) {
            if item.name == other.name {
                return false;
            }
        }
    }
    true
}

/// Stick modes where short failsafe consults `plane.emergency_landing`.
#[must_use]
pub fn short_emergency_landing_applies(mode: ModeNumber) -> bool {
    matches!(
        mode,
        ModeNumber::Manual
            | ModeNumber::Stabilize
            | ModeNumber::Acro
            | ModeNumber::FlyByWireA
            | ModeNumber::Autotune
            | ModeNumber::FlyByWireB
            | ModeNumber::Cruise
            | ModeNumber::Training
    )
}

/// Stick/hold modes where long failsafe consults `plane.emergency_landing`.
///
/// Upstream `failsafe_long_on_event` checks the switch after the TAKEOFF
/// climb-pending latch and before `FS_LONG_ACTN`. CIRCLE / LOITER / THERMAL
/// / TAKEOFF are in this group; RTL / AUTO-like / Q modes are not.
#[must_use]
pub fn long_emergency_landing_applies(mode: ModeNumber) -> bool {
    matches!(
        mode,
        ModeNumber::Manual
            | ModeNumber::Stabilize
            | ModeNumber::Acro
            | ModeNumber::FlyByWireA
            | ModeNumber::Autotune
            | ModeNumber::FlyByWireB
            | ModeNumber::Cruise
            | ModeNumber::Training
            | ModeNumber::Circle
            | ModeNumber::Loiter
            | ModeNumber::Thermal
            | ModeNumber::Takeoff
    )
}

/// `rc_failsafe_short_on_event` after the emergency-landing override.
///
/// Does not rewrite [`short_failsafe_action`]. Stick modes with the
/// `EMERGENCY_LANDING_EN` aux switch high force FBWA so an out-of-range
/// landing can keep throttle; CIRCLE / TAKEOFF / RTL stay on the existing
/// no-short-action table.
#[must_use]
pub fn emergency_landing_short_failsafe_action(
    mode: ModeNumber,
    action: FailsafeActionShort,
    emergency_landing: bool,
) -> FailsafeActionResult {
    if !action.is_enabled() {
        return FailsafeActionResult::Continue;
    }
    if emergency_landing && short_emergency_landing_applies(mode) {
        return FailsafeActionResult::Switch(ModeNumber::FlyByWireA);
    }
    short_failsafe_action(mode, action)
}

/// `failsafe_long_on_event` after the emergency-landing override.
///
/// Does not rewrite [`long_failsafe_action`]. The TAKEOFF climb-pending
/// latch (`long_failsafe_pending`) is still deferred.
#[must_use]
pub fn emergency_landing_long_failsafe_action(
    mode: ModeNumber,
    action: FailsafeActionLong,
    autoland_available: bool,
    emergency_landing: bool,
) -> FailsafeActionResult {
    if emergency_landing && long_emergency_landing_applies(mode) {
        return FailsafeActionResult::Switch(ModeNumber::FlyByWireA);
    }
    long_failsafe_action(mode, action, autoland_available)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_covers_main_hookups_and_remaining_gaps() {
        assert!(completeness_unique_names());
        let (on_main, this_slice, remaining) = completeness_counts();
        assert_eq!(on_main, 16);
        assert_eq!(this_slice, 1);
        assert_eq!(remaining, 1);
        assert!(completeness_has(
            "FS_SHORT_ACTN / FS_LONG_ACTN",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "CIRCLE/TAKEOFF/RTL no-short-action",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "emergency-landing override",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "completeness table",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "Q_OPTIONS FS_RTL / FS_QRTL",
            PortStatus::Remaining
        ));
        assert!(completeness_has(
            "FENCE_ACTION 8 AUTOLAND-or-RTL",
            PortStatus::ThisSlice
        ));
        assert_eq!(on_main_items().count(), 16);
        assert_eq!(this_slice_items().count(), 1);
        assert_eq!(remaining_items().count(), 1);
        assert_eq!(Q_OPTIONS_FS_QRTL, 1 << 5);
        assert_eq!(Q_OPTIONS_FS_RTL, 1 << 20);
        assert_eq!(FENCE_ACTION_AUTOLAND_OR_RTL, 8);
    }

    #[test]
    fn remaining_does_not_repeat_hooked_surfaces() {
        for item in remaining_items() {
            assert!(
                !completeness_has(item.name, PortStatus::OnMain),
                "{} listed remaining but already on main",
                item.name
            );
            assert!(
                !completeness_has(item.name, PortStatus::ThisSlice),
                "{} listed remaining but added this slice",
                item.name
            );
        }
    }

    #[test]
    fn emergency_landing_forces_fbwa_in_stick_modes() {
        assert_eq!(
            emergency_landing_short_failsafe_action(
                ModeNumber::Manual,
                FailsafeActionShort::Circle,
                true
            ),
            FailsafeActionResult::Switch(ModeNumber::FlyByWireA)
        );
        assert_eq!(
            emergency_landing_short_failsafe_action(
                ModeNumber::Manual,
                FailsafeActionShort::Circle,
                false
            ),
            FailsafeActionResult::Switch(ModeNumber::Circle)
        );
        assert_eq!(
            emergency_landing_long_failsafe_action(
                ModeNumber::Circle,
                FailsafeActionLong::Rtl,
                true,
                true
            ),
            FailsafeActionResult::Switch(ModeNumber::FlyByWireA)
        );
    }
}
