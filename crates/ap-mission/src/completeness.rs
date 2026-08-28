//! FW-024 AP_Mission completeness: command surfaces already on main vs remaining.
//!
//! Catalogs the SITL-first `AP_Mission` / `commands_logic.cpp` port. Items marked
//! [`PortStatus::OnMain`] or [`PortStatus::ThisSlice`] are hooked up;
//! [`PortStatus::Remaining`] are documented-deferred command families (camera,
//! mount, conditionals, leftover nav/do) or log-replay outside this ticket's
//! stub surface.

/// Whether a catalog row is already hooked up or left for later work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortStatus {
    /// Present on `main` before this closing slice.
    OnMain,
    /// Added by the FW-024 closing slice (this table).
    ThisSlice,
    /// Documented-deferred: not blocking the FW-024 SITL stub close.
    Remaining,
}

/// One AP_Mission command surface in the completeness table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MissionPortItem {
    /// Surface name.
    pub name: &'static str,
    /// Hooked up on main / this slice, or remaining.
    pub status: PortStatus,
    /// Short note (upstream symbol or why remaining).
    pub note: &'static str,
}

/// Completeness table: ported mission-command stubs vs documented-deferred gaps.
pub const MISSION_COMPLETENESS: &[MissionPortItem] = &[
    MissionPortItem {
        name: "Mission_Command storage",
        status: PortStatus::OnMain,
        note: "Mission / MissionCommand seq,command,frame,LLA / write_cmd / read_cmd",
    },
    MissionPortItem {
        name: "NAV_WAYPOINT verify-distance / reached-wp",
        status: PortStatus::OnMain,
        note: "verify_nav_wp / WP_RADIUS",
    },
    MissionPortItem {
        name: "NAV_LOITER_UNLIM do/verify",
        status: PortStatus::OnMain,
        note: "do_loiter_unlimited / verify_loiter_unlim",
    },
    MissionPortItem {
        name: "NAV_LOITER_TURNS do/verify",
        status: PortStatus::OnMain,
        note: "do_loiter_turns / verify_loiter_turns",
    },
    MissionPortItem {
        name: "NAV_LOITER_TIME do/verify",
        status: PortStatus::OnMain,
        note: "do_loiter_time / verify_loiter_time",
    },
    MissionPortItem {
        name: "NAV_LOITER_TO_ALT do/verify",
        status: PortStatus::OnMain,
        note: "do_loiter_to_alt / verify_loiter_to_alt",
    },
    MissionPortItem {
        name: "NAV_CONTINUE_AND_CHANGE_ALT do/verify",
        status: PortStatus::OnMain,
        note: "do_continue_and_change_alt / verify_continue_and_change_alt",
    },
    MissionPortItem {
        name: "NAV_LAND do/verify",
        status: PortStatus::OnMain,
        note: "do_land / verify_land",
    },
    MissionPortItem {
        name: "DO_JUMP / jump-to-seq",
        status: PortStatus::OnMain,
        note: "do_jump / jump_should_take / jump_target_valid",
    },
    MissionPortItem {
        name: "DO_CHANGE_SPEED / airspeed-groundspeed-throttle",
        status: PortStatus::OnMain,
        note: "do_change_speed / SPEED_TYPE_AIRSPEED/GROUNDSPEED",
    },
    MissionPortItem {
        name: "DO_SET_HOME / current-or-specified-LLA",
        status: PortStatus::OnMain,
        note: "do_set_home / set_home_use_current",
    },
    MissionPortItem {
        name: "DO_SET_ROI / point-camera-at-location",
        status: PortStatus::OnMain,
        note: "do_set_roi / roi_location_set",
    },
    MissionPortItem {
        name: "mission_scheduler_hookup",
        status: PortStatus::OnMain,
        note: "ap-plane mission waypoint advance + target altitude tick",
    },
    MissionPortItem {
        name: "mission_alt_offset_glue_hookup",
        status: PortStatus::OnMain,
        note: "ap-plane mission_alt_offset / target_altitude.offset_cm",
    },
    MissionPortItem {
        name: "completeness table",
        status: PortStatus::ThisSlice,
        note: "this catalog",
    },
    MissionPortItem {
        name: "DO_DIGICAM",
        status: PortStatus::Remaining,
        note: "DO_DIGICAM_CONFIGURE/CONTROL documented-deferred; AP_Camera owns trigger",
    },
    MissionPortItem {
        name: "DO_MOUNT",
        status: PortStatus::Remaining,
        note: "DO_MOUNT_CONTROL documented-deferred; AP_Mount owns gimbal",
    },
    MissionPortItem {
        name: "conditionals",
        status: PortStatus::Remaining,
        note: "CONDITION_DELAY / CONDITION_DISTANCE / CONDITION_YAW documented-deferred",
    },
    MissionPortItem {
        name: "remaining nav/do commands",
        status: PortStatus::Remaining,
        note: "NAV_TAKEOFF/RTL/DELAY/ALTITUDE_WAIT, servo/relay, VTOL; no new stubs",
    },
    MissionPortItem {
        name: "log-replay",
        status: PortStatus::Remaining,
        note: "ADR-0008 differential vs recorded outputs",
    },
];

/// Rows already hooked up on `main` (must not be redone).
#[must_use]
pub fn on_main_items() -> impl Iterator<Item = &'static MissionPortItem> {
    MISSION_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::OnMain)
}

/// Rows added by this closing slice.
#[must_use]
pub fn this_slice_items() -> impl Iterator<Item = &'static MissionPortItem> {
    MISSION_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::ThisSlice)
}

/// Rows left documented-deferred (not blocking FW-024 SITL close).
#[must_use]
pub fn remaining_items() -> impl Iterator<Item = &'static MissionPortItem> {
    MISSION_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::Remaining)
}

/// Count rows in each status bucket.
#[must_use]
pub fn completeness_counts() -> (usize, usize, usize) {
    let mut on_main = 0;
    let mut this_slice = 0;
    let mut remaining = 0;
    for item in MISSION_COMPLETENESS {
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
    MISSION_COMPLETENESS
        .iter()
        .any(|item| item.name == name && item.status == status)
}

/// True when every name in the table appears once.
#[must_use]
pub fn completeness_unique_names() -> bool {
    for (i, item) in MISSION_COMPLETENESS.iter().enumerate() {
        for other in MISSION_COMPLETENESS.iter().skip(i + 1) {
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
    fn table_covers_main_surfaces_and_this_slice() {
        assert!(completeness_unique_names());
        let (on_main, this_slice, remaining) = completeness_counts();
        assert_eq!(on_main, 14);
        assert_eq!(this_slice, 1);
        assert_eq!(remaining, 5);
        assert!(completeness_has(
            "Mission_Command storage",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "NAV_WAYPOINT verify-distance / reached-wp",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "DO_SET_ROI / point-camera-at-location",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "mission_scheduler_hookup",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "completeness table",
            PortStatus::ThisSlice
        ));
        assert!(completeness_has("DO_DIGICAM", PortStatus::Remaining));
        assert!(completeness_has("DO_MOUNT", PortStatus::Remaining));
        assert!(completeness_has("conditionals", PortStatus::Remaining));
        assert_eq!(on_main_items().count(), 14);
        assert_eq!(this_slice_items().count(), 1);
        assert_eq!(remaining_items().count(), 5);
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
}
