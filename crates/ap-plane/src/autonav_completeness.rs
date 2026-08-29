//! FW-023 autonav completeness: modes/hookups already on main vs remaining.
//!
//! Catalogs the fixed-wing autonomous navigation port. Items marked
//! [`PortStatus::OnMain`] or [`PortStatus::ThisSlice`] are hooked up;
//! [`PortStatus::Remaining`] are QLAND/VTOL, offboard-slew, or log-replay
//! work outside this ticket's stub surface.

/// Whether a catalog row is already hooked up or left for later work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortStatus {
    /// Present on `main` before this closing slice.
    OnMain,
    /// Added by the FW-023 closing slice (`RTL_AUTOLAND` + this table).
    ThisSlice,
    /// Out of scope for the fixed-wing autonav stub close (QLAND/VTOL, replay).
    Remaining,
}

/// One autonomous mode hookup in the completeness table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutonavPortItem {
    /// Mode or hookup name.
    pub name: &'static str,
    /// Hooked up on main / this slice, or remaining.
    pub status: PortStatus,
    /// Short note (upstream symbol or why remaining).
    pub note: &'static str,
}

/// Completeness table: hooked-up autonav modes vs remaining QLAND/VTOL / replay.
pub const AUTONAV_COMPLETENESS: &[AutonavPortItem] = &[
    AutonavPortItem {
        name: "AUTO enter/navigate",
        status: PortStatus::OnMain,
        note: "auto_mode_mission_tick / ModeAuto start+advance",
    },
    AutonavPortItem {
        name: "RTL enter/navigate",
        status: PortStatus::OnMain,
        note: "rtl_mode_nav_tick / ModeRTL home-loiter",
    },
    AutonavPortItem {
        name: "LOITER enter/navigate",
        status: PortStatus::OnMain,
        note: "loiter_mode_nav_tick / location-hold",
    },
    AutonavPortItem {
        name: "GUIDED enter/navigate",
        status: PortStatus::OnMain,
        note: "guided_mode_nav_tick / current-location loiter",
    },
    AutonavPortItem {
        name: "TAKEOFF enter/navigate",
        status: PortStatus::OnMain,
        note: "takeoff_mode_nav_tick / climb-then-loiter",
    },
    AutonavPortItem {
        name: "AUTOLAND enter-and-stage",
        status: PortStatus::OnMain,
        note: "autoland_mode_nav_tick / climb-loiter-land stages",
    },
    AutonavPortItem {
        name: "AVOID_ADSB enter/navigate",
        status: PortStatus::OnMain,
        note: "avoid_adsb_mode_nav_tick / guided-enter loiter",
    },
    AutonavPortItem {
        name: "AUTO mission-complete",
        status: PortStatus::OnMain,
        note: "auto_mode_complete_tick / MISSION_END -> RTL unless NAV_LAND",
    },
    AutonavPortItem {
        name: "RTL climb-then-home",
        status: PortStatus::OnMain,
        note: "rtl_mode_climb_tick / CLIMB_BEFORE_TURN / RTL_CLIMB_MIN",
    },
    AutonavPortItem {
        name: "AUTO NAV_LOITER_TO_ALT",
        status: PortStatus::OnMain,
        note: "auto_mode_loiter_to_alt_tick / complete then resume AUTO",
    },
    AutonavPortItem {
        name: "GUIDED remaining-leg",
        status: PortStatus::OnMain,
        note: "guided_mode_update_tick / handle_guided_request / handle_change_alt_request",
    },
    AutonavPortItem {
        name: "FW-022 assisted modes",
        status: PortStatus::OnMain,
        note: "MANUAL/CIRCLE/STABILIZE/TRAINING/ACRO/FBWA/FBWB/CRUISE/AUTOTUNE/THERMAL",
    },
    AutonavPortItem {
        name: "RTL_AUTOLAND",
        status: PortStatus::ThisSlice,
        note: "rtl_autoland_tick / RTL -> AUTO landing or return-path",
    },
    AutonavPortItem {
        name: "completeness table",
        status: PortStatus::ThisSlice,
        note: "this catalog",
    },
    AutonavPortItem {
        name: "QLAND / VTOL modes",
        status: PortStatus::Remaining,
        note: "QStabilize/QHover/QLoiter/QLand/QRtl/QAutotune/QAcro/LoiterAltQLand deferred",
    },
    AutonavPortItem {
        name: "RTL switch_QRTL",
        status: PortStatus::ThisSlice,
        note: "rtl_mode_switch_qrtl_tick / ModeRTL::switch_QRTL VTOL handoff (FW-041); VTOL_APPROACH_QRTL landing-approach state machine still remaining",
    },
    AutonavPortItem {
        name: "GUIDED offboard slew",
        status: PortStatus::Remaining,
        note: "GUIDED_TIMEOUT / forced RPY / target heading / change airspeed",
    },
    AutonavPortItem {
        name: "log-replay",
        status: PortStatus::Remaining,
        note: "ADR-0008 differential vs recorded outputs",
    },
];

/// Rows already hooked up on `main` (must not be redone).
#[must_use]
pub fn on_main_items() -> impl Iterator<Item = &'static AutonavPortItem> {
    AUTONAV_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::OnMain)
}

/// Rows added by this closing slice.
#[must_use]
pub fn this_slice_items() -> impl Iterator<Item = &'static AutonavPortItem> {
    AUTONAV_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::ThisSlice)
}

/// Rows left for QLAND/VTOL / offboard / replay (not blocking FW-023 close).
#[must_use]
pub fn remaining_items() -> impl Iterator<Item = &'static AutonavPortItem> {
    AUTONAV_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::Remaining)
}

/// Count rows in each status bucket.
#[must_use]
pub fn completeness_counts() -> (usize, usize, usize) {
    let mut on_main = 0;
    let mut this_slice = 0;
    let mut remaining = 0;
    for item in AUTONAV_COMPLETENESS {
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
    AUTONAV_COMPLETENESS
        .iter()
        .any(|item| item.name == name && item.status == status)
}

/// True when every name in the table appears once.
#[must_use]
pub fn completeness_unique_names() -> bool {
    for (i, item) in AUTONAV_COMPLETENESS.iter().enumerate() {
        for other in AUTONAV_COMPLETENESS.iter().skip(i + 1) {
            if item.name == other.name {
                return false;
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_covers_main_hookups_and_this_slice() {
        assert!(completeness_unique_names());
        let (on_main, this_slice, remaining) = completeness_counts();
        assert_eq!(on_main, 12);
        assert_eq!(this_slice, 3);
        assert_eq!(remaining, 3);
        assert!(completeness_has("AUTO enter/navigate", PortStatus::OnMain));
        assert!(completeness_has("RTL climb-then-home", PortStatus::OnMain));
        assert!(completeness_has("GUIDED remaining-leg", PortStatus::OnMain));
        assert!(completeness_has("FW-022 assisted modes", PortStatus::OnMain));
        assert!(completeness_has("RTL_AUTOLAND", PortStatus::ThisSlice));
        assert!(completeness_has("completeness table", PortStatus::ThisSlice));
        assert!(completeness_has("RTL switch_QRTL", PortStatus::ThisSlice));
        assert!(completeness_has("QLAND / VTOL modes", PortStatus::Remaining));
        assert_eq!(on_main_items().count(), 12);
        assert_eq!(this_slice_items().count(), 3);
        assert_eq!(remaining_items().count(), 3);
    }

    #[test]
    fn remaining_does_not_repeat_hooked_modes() {
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
}
