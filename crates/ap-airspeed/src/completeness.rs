//! FW-010 airspeed completeness: params/hookups already on main vs remaining.
//!
//! Catalogs the SITL-first `AP_Airspeed` port. Items marked [`PortStatus::OnMain`]
//! or [`PortStatus::ThisSlice`] are hooked up; [`PortStatus::Remaining`] are
//! hardware / live-EKF / log-replay work outside this ticket's stub surface.

/// Whether a catalog row is already hooked up or left for later work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortStatus {
    /// Present on `main` before this closing slice.
    OnMain,
    /// Added by the FW-010 closing slice (`ARSPD_OFF_PCNT` + this table).
    ThisSlice,
    /// Out of scope for the SITL stub port (hardware, extra instances, replay).
    Remaining,
}

/// One airspeed param or vehicle hookup in the completeness table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AirspeedPortItem {
    /// Param name, hookup, or backend.
    pub name: &'static str,
    /// Hooked up on main / this slice, or remaining.
    pub status: PortStatus,
    /// Short note (upstream symbol or why remaining).
    pub note: &'static str,
}

/// Completeness table: hooked-up SITL stubs vs remaining hardware / replay.
pub const AIRSPEED_COMPLETENESS: &[AirspeedPortItem] = &[
    AirspeedPortItem {
        name: "SITL backend",
        status: PortStatus::OnMain,
        note: "SitlAirspeedBackend / sitl_airspeed_hookup",
    },
    AirspeedPortItem {
        name: "pre-arm",
        status: PortStatus::OnMain,
        note: "airspeed_pre_arm_hookup",
    },
    AirspeedPortItem {
        name: "health scheduler",
        status: PortStatus::OnMain,
        note: "airspeed_health_scheduler_hookup",
    },
    AirspeedPortItem {
        name: "ARSPD_OFFSET",
        status: PortStatus::OnMain,
        note: "pitot offset calibration stub",
    },
    AirspeedPortItem {
        name: "ARSPD_RATIO",
        status: PortStatus::OnMain,
        note: "pitot tube ratio",
    },
    AirspeedPortItem {
        name: "dual-instance",
        status: PortStatus::OnMain,
        note: "SitlAirspeedCluster / ARSPD2_*",
    },
    AirspeedPortItem {
        name: "ARSPD_USE",
        status: PortStatus::OnMain,
        note: "TAS unused for TECS/nav when disabled",
    },
    AirspeedPortItem {
        name: "temperature compensation",
        status: PortStatus::OnMain,
        note: "apply_temp_compensation",
    },
    AirspeedPortItem {
        name: "ARSPD_AUTOCAL",
        status: PortStatus::OnMain,
        note: "GPS GS vs TAS ratio learn",
    },
    AirspeedPortItem {
        name: "ARSPD_PIN",
        status: PortStatus::OnMain,
        note: "analog backend stub",
    },
    AirspeedPortItem {
        name: "ARSPD_SKIP_CAL",
        status: PortStatus::OnMain,
        note: "skip startup offset cal",
    },
    AirspeedPortItem {
        name: "ARSPD_TYPE",
        status: PortStatus::OnMain,
        note: "backend selection",
    },
    AirspeedPortItem {
        name: "ARSPD_TUBE_ORDER",
        status: PortStatus::OnMain,
        note: "pitot pressure sign",
    },
    AirspeedPortItem {
        name: "ARSPD_BUS",
        status: PortStatus::OnMain,
        note: "I2C bus stub",
    },
    AirspeedPortItem {
        name: "ARSPD_DEVID",
        status: PortStatus::OnMain,
        note: "device-id / bus_id",
    },
    AirspeedPortItem {
        name: "healthy-for-TECS",
        status: PortStatus::OnMain,
        note: "airspeed_tecs_health_hookup",
    },
    AirspeedPortItem {
        name: "ARSPD_OPTIONS",
        status: PortStatus::OnMain,
        note: "vehicle-level bitfield",
    },
    AirspeedPortItem {
        name: "ARSPD_WIND_MAX",
        status: PortStatus::OnMain,
        note: "airspeed vs groundspeed disable",
    },
    AirspeedPortItem {
        name: "ARSPD_WIND_WARN",
        status: PortStatus::OnMain,
        note: "airspeed vs wind warning",
    },
    AirspeedPortItem {
        name: "ARSPD_PRIMARY",
        status: PortStatus::OnMain,
        note: "primary-instance select",
    },
    AirspeedPortItem {
        name: "ARSPD_FBW_MIN",
        status: PortStatus::OnMain,
        note: "fly-by-wire min 9 m/s",
    },
    AirspeedPortItem {
        name: "ARSPD_FBW_MAX",
        status: PortStatus::OnMain,
        note: "fly-by-wire max 22 m/s",
    },
    AirspeedPortItem {
        name: "ARSPD_PSI_RANGE",
        status: PortStatus::OnMain,
        note: "sensor PSI clamp/validate",
    },
    AirspeedPortItem {
        name: "ARSPD_OFF_PCNT",
        status: PortStatus::ThisSlice,
        note: "Plane-only offset-cal speed-error warning",
    },
    AirspeedPortItem {
        name: "completeness table",
        status: PortStatus::ThisSlice,
        note: "this catalog",
    },
    AirspeedPortItem {
        name: "ARSPD_ENABLE",
        status: PortStatus::Remaining,
        note: "lib enable; SITL path is always on",
    },
    AirspeedPortItem {
        name: "ARSPD_WIND_GATE",
        status: PortStatus::Remaining,
        note: "EKF innovation gate needs live EKF3",
    },
    AirspeedPortItem {
        name: "instances 3-6",
        status: PortStatus::Remaining,
        note: "AIRSPEED_MAX_SENSORS > 2",
    },
    AirspeedPortItem {
        name: "hardware backends",
        status: PortStatus::Remaining,
        note: "MS4525/MS5525/SDP3X/DLVR/DroneCAN/NMEA/MSP/AUAV",
    },
    AirspeedPortItem {
        name: "log-replay",
        status: PortStatus::Remaining,
        note: "ADR-0008 differential vs recorded outputs",
    },
];

/// Rows already hooked up on `main` (must not be redone).
#[must_use]
pub fn on_main_items() -> impl Iterator<Item = &'static AirspeedPortItem> {
    AIRSPEED_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::OnMain)
}

/// Rows added by this closing slice.
#[must_use]
pub fn this_slice_items() -> impl Iterator<Item = &'static AirspeedPortItem> {
    AIRSPEED_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::ThisSlice)
}

/// Rows left for hardware / EKF / replay (not blocking FW-010 SITL close).
#[must_use]
pub fn remaining_items() -> impl Iterator<Item = &'static AirspeedPortItem> {
    AIRSPEED_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::Remaining)
}

/// Count rows in each status bucket.
#[must_use]
pub fn completeness_counts() -> (usize, usize, usize) {
    let mut on_main = 0;
    let mut this_slice = 0;
    let mut remaining = 0;
    for item in AIRSPEED_COMPLETENESS {
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
    AIRSPEED_COMPLETENESS
        .iter()
        .any(|item| item.name == name && item.status == status)
}

/// True when every name in the table appears once.
#[must_use]
pub fn completeness_unique_names() -> bool {
    for (i, item) in AIRSPEED_COMPLETENESS.iter().enumerate() {
        for other in AIRSPEED_COMPLETENESS.iter().skip(i + 1) {
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
        assert_eq!(on_main, 23);
        assert_eq!(this_slice, 2);
        assert_eq!(remaining, 5);
        assert!(completeness_has("ARSPD_PSI_RANGE", PortStatus::OnMain));
        assert!(completeness_has("ARSPD_FBW_MIN", PortStatus::OnMain));
        assert!(completeness_has("ARSPD_PRIMARY", PortStatus::OnMain));
        assert!(completeness_has("ARSPD_OFF_PCNT", PortStatus::ThisSlice));
        assert!(completeness_has("completeness table", PortStatus::ThisSlice));
        assert!(completeness_has("hardware backends", PortStatus::Remaining));
        assert_eq!(on_main_items().count(), 23);
        assert_eq!(this_slice_items().count(), 2);
        assert_eq!(remaining_items().count(), 5);
    }

    #[test]
    fn remaining_does_not_repeat_hooked_params() {
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
