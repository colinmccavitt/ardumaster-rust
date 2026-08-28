//! FW-026 AP_Arming completeness: surfaces already on main vs remaining.
//!
//! Catalogs the SITL-first `AP_Arming` port. Items marked [`PortStatus::OnMain`]
//! or [`PortStatus::ThisSlice`] are hooked up; [`PortStatus::Remaining`] are
//! documented-deferred (mandatory/vehicle checks, leftover named-check bodies,
//! land/rally/RTL mission bits, rudder stick-hold FSM, log-replay) outside
//! this ticket's stub surface.

/// Whether a catalog row is already hooked up or left for later work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortStatus {
    /// Present on `main` before this closing slice.
    OnMain,
    /// Added by the FW-026 closing slice (this table).
    ThisSlice,
    /// Documented-deferred: not blocking the FW-026 SITL stub close.
    Remaining,
}

/// One AP_Arming surface in the completeness table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ArmingPortItem {
    /// Surface name.
    pub name: &'static str,
    /// Hooked up on main / this slice, or remaining.
    pub status: PortStatus,
    /// Short note (upstream symbol or why remaining).
    pub note: &'static str,
}

/// Completeness table: ported arming stubs vs documented-deferred gaps.
pub const ARMING_COMPLETENESS: &[ArmingPortItem] = &[
    ArmingPortItem {
        name: "check-registry (ARMING_REQUIRE / ARMING_SKIPCHK)",
        status: PortStatus::OnMain,
        note: "Arming / Required / Check / NamedCheck / pre_arm_checks",
    },
    ArmingPortItem {
        name: "ARMING_CHECK enable-bitmask",
        status: PortStatus::OnMain,
        note: "arming_check_enabled / skipchk_from_arming_check / ARMING_CHECK_ALL",
    },
    ArmingPortItem {
        name: "baro + AHRS named-checks",
        status: PortStatus::OnMain,
        note: "baro_ahrs_named_check_hookup / Check::Baro + AHRS",
    },
    ArmingPortItem {
        name: "GPS + INS named-checks",
        status: PortStatus::OnMain,
        note: "gps_ins_named_check_hookup / Check::Gps / Check::Ins",
    },
    ArmingPortItem {
        name: "compass + airspeed named-checks",
        status: PortStatus::OnMain,
        note: "compass_airspeed_named_check_hookup / Check::Compass / Check::Airspeed",
    },
    ArmingPortItem {
        name: "ARMING_RUDDER / rudder-arm-disarm gate",
        status: PortStatus::OnMain,
        note: "RudderArming / rudder_stick_allowed",
    },
    ArmingPortItem {
        name: "ARMING_ACCTHRESH / accel-threshold named-check",
        status: PortStatus::OnMain,
        note: "accel_threshold_named_check / ARMING_ACCTHRESH_DEFAULT",
    },
    ArmingPortItem {
        name: "vehicle arm/disarm gate",
        status: PortStatus::OnMain,
        note: "vehicle_arm.rs / Arming::arm / Arming::disarm",
    },
    ArmingPortItem {
        name: "ARMING_MIS_ITEMS / mission-item pre-arm check",
        status: PortStatus::OnMain,
        note: "mission_items_named_check / MAV_CMD_NAV_TAKEOFF first flown item",
    },
    ArmingPortItem {
        name: "ARMING_OPTIONS bitfield decode/apply",
        status: PortStatus::OnMain,
        note: "apply_arming_options / ArmingOption / ArmingOptionsApplied",
    },
    ArmingPortItem {
        name: "ARMING_NEED_LOC / require-position-before-arm",
        status: PortStatus::OnMain,
        note: "need_loc_named_check / RequireLocation / GPS_OK_FIX_3D",
    },
    ArmingPortItem {
        name: "ARMING_CRASH_IF_DISARMED / crash-check-while-disarmed",
        status: PortStatus::OnMain,
        note: "crash_if_disarmed_named_check / CrashIfDisarmed",
    },
    ArmingPortItem {
        name: "existing sensor pre-arm glue",
        status: PortStatus::OnMain,
        note: "ahrs/gps/baro/compass/airspeed_pre_arm_hookup; leave those hookups as-is",
    },
    ArmingPortItem {
        name: "completeness table",
        status: PortStatus::ThisSlice,
        note: "this catalog",
    },
    ArmingPortItem {
        name: "mandatory checks when skip-all",
        status: PortStatus::Remaining,
        note: "Plane still runs mandatory checks when get_enabled_checks==0; later slice",
    },
    ArmingPortItem {
        name: "vehicle arm_checks (RC / logging / estop)",
        status: PortStatus::Remaining,
        note: "vehicle-specific arm_checks, remaining Method, throttle-down disarm; no new stubs",
    },
    ArmingPortItem {
        name: "remaining named-check bodies",
        status: PortStatus::Remaining,
        note: "battery/voltage/logging/switch/gpsconfig/system/rangefinder/camera/auxauth/vision/fft/osd; no new stubs",
    },
    ArmingPortItem {
        name: "ARMING_MIS_ITEMS land / rally / RTL bits",
        status: PortStatus::Remaining,
        note: "MIS_ITEM_CHECK_LAND/RALLY/RTL documented-deferred; takeoff gate is on main",
    },
    ArmingPortItem {
        name: "rudder stick-hold FSM",
        status: PortStatus::Remaining,
        note: "throttle-at-zero / yaw-extreme hold; ARMING_RUDDER gate is on main",
    },
    ArmingPortItem {
        name: "log-replay",
        status: PortStatus::Remaining,
        note: "ADR-0008 differential vs recorded outputs",
    },
];

/// Rows already hooked up on `main` (must not be redone).
#[must_use]
pub fn on_main_items() -> impl Iterator<Item = &'static ArmingPortItem> {
    ARMING_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::OnMain)
}

/// Rows added by this closing slice.
#[must_use]
pub fn this_slice_items() -> impl Iterator<Item = &'static ArmingPortItem> {
    ARMING_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::ThisSlice)
}

/// Rows left documented-deferred (not blocking FW-026 SITL close).
#[must_use]
pub fn remaining_items() -> impl Iterator<Item = &'static ArmingPortItem> {
    ARMING_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::Remaining)
}

/// Count rows in each status bucket.
#[must_use]
pub fn completeness_counts() -> (usize, usize, usize) {
    let mut on_main = 0;
    let mut this_slice = 0;
    let mut remaining = 0;
    for item in ARMING_COMPLETENESS {
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
    ARMING_COMPLETENESS
        .iter()
        .any(|item| item.name == name && item.status == status)
}

/// True when every name in the table appears once.
#[must_use]
pub fn completeness_unique_names() -> bool {
    for (i, item) in ARMING_COMPLETENESS.iter().enumerate() {
        for other in ARMING_COMPLETENESS.iter().skip(i + 1) {
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
        assert_eq!(on_main, 13);
        assert_eq!(this_slice, 1);
        assert_eq!(remaining, 6);
        assert!(completeness_has(
            "check-registry (ARMING_REQUIRE / ARMING_SKIPCHK)",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "ARMING_CHECK enable-bitmask",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "baro + AHRS named-checks",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "GPS + INS named-checks",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "compass + airspeed named-checks",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "ARMING_RUDDER / rudder-arm-disarm gate",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "ARMING_ACCTHRESH / accel-threshold named-check",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "vehicle arm/disarm gate",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "ARMING_MIS_ITEMS / mission-item pre-arm check",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "ARMING_OPTIONS bitfield decode/apply",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "ARMING_NEED_LOC / require-position-before-arm",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "ARMING_CRASH_IF_DISARMED / crash-check-while-disarmed",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "existing sensor pre-arm glue",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "completeness table",
            PortStatus::ThisSlice
        ));
        assert!(completeness_has(
            "mandatory checks when skip-all",
            PortStatus::Remaining
        ));
        assert!(completeness_has(
            "vehicle arm_checks (RC / logging / estop)",
            PortStatus::Remaining
        ));
        assert!(completeness_has(
            "remaining named-check bodies",
            PortStatus::Remaining
        ));
        assert!(completeness_has(
            "ARMING_MIS_ITEMS land / rally / RTL bits",
            PortStatus::Remaining
        ));
        assert!(completeness_has(
            "rudder stick-hold FSM",
            PortStatus::Remaining
        ));
        assert!(completeness_has("log-replay", PortStatus::Remaining));
        assert_eq!(on_main_items().count(), 13);
        assert_eq!(this_slice_items().count(), 1);
        assert_eq!(remaining_items().count(), 6);
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
