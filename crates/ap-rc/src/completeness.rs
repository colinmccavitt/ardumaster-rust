//! FW-019 RC_Channel completeness: surfaces already on main vs remaining.
//!
//! Catalogs the SITL-first `RC_Channel` port. Items marked [`PortStatus::OnMain`]
//! or [`PortStatus::ThisSlice`] are hooked up; [`PortStatus::Remaining`] are
//! HAL I/O, hardware protocols, or log-replay work outside this ticket's stub
//! surface.

/// Whether a catalog row is already hooked up or left for later work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortStatus {
    /// Present on `main` before this closing slice.
    OnMain,
    /// Added by the FW-019 closing slice (this table).
    ThisSlice,
    /// Out of scope for the SITL stub port (HAL I/O, protocols, replay).
    Remaining,
}

/// One RC_Channel surface in the completeness table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcPortItem {
    /// Surface name.
    pub name: &'static str,
    /// Hooked up on main / this slice, or remaining.
    pub status: PortStatus,
    /// Short note (upstream symbol or why remaining).
    pub note: &'static str,
}

/// Completeness table: hooked-up RC stubs vs remaining HAL / protocol / replay.
pub const RC_COMPLETENESS: &[RcPortItem] = &[
    RcPortItem {
        name: "PWM scaling + deadzone",
        status: PortStatus::OnMain,
        note: "norm_input / norm_input_dz / RcChannel radio_min/trim/max",
    },
    RcPortItem {
        name: "aux-function switch latch",
        status: PortStatus::OnMain,
        note: "AuxSwitchLatch / read_3pos_switch / SWITCH_DEBOUNCE_TIME_MS",
    },
    RcPortItem {
        name: "FS_THR / failsafe throttle",
        status: PortStatus::OnMain,
        note: "throttle_pwm_in_failsafe / FS_THR_VALUE / THR_FS_VALUE",
    },
    RcPortItem {
        name: "RCMAP / RC trim persist",
        status: PortStatus::OnMain,
        note: "RcMap / set_and_save_trim / persist_stick_trims",
    },
    RcPortItem {
        name: "RC_OVERRIDE / GCS override timeout",
        status: PortStatus::OnMain,
        note: "OverrideTimeout / RC_OVERRIDE_TIME / apply_gcs_override_field",
    },
    RcPortItem {
        name: "option-switch 2-pos vs 3-pos PWM ranges",
        status: PortStatus::OnMain,
        note: "read_option_switch / read_2pos_switch / AUX_PWM_TRIGGER_*",
    },
    RcPortItem {
        name: "FLTMODE_CH six-position PWM decode",
        status: PortStatus::OnMain,
        note: "decode_fltmode_ch / read_6pos_switch / FLTMODE_POS*_MAX_PWM",
    },
    RcPortItem {
        name: "FLTMODE1-6 six-position mapping",
        status: PortStatus::OnMain,
        note: "fltmode_for_slot / FltModeTable / FLTMODE1-6",
    },
    RcPortItem {
        name: "INITIAL_MODE / boot-mode-from-switch",
        status: PortStatus::OnMain,
        note: "boot_mode_from_switch / BootMode / INITIAL_MODE",
    },
    RcPortItem {
        name: "RC_OPTIONS bitfield decode/apply",
        status: PortStatus::OnMain,
        note: "apply_rc_options / RcOption / RcOptionsApplied",
    },
    RcPortItem {
        name: "RC_SPEED / PWM update-rate",
        status: PortStatus::OnMain,
        note: "apply_rc_speed / pwm_period_us / RC_SPEED",
    },
    RcPortItem {
        name: "RC_REVERSED / per-channel reverse",
        status: PortStatus::OnMain,
        note: "get_reverse / apply_rc_reversed / reverse_range_pwm",
    },
    RcPortItem {
        name: "completeness table",
        status: PortStatus::ThisSlice,
        note: "this catalog",
    },
    RcPortItem {
        name: "full RCn_OPTION aux-function table",
        status: PortStatus::Remaining,
        note: "upstream AUX_FUNC is hundreds of codes; stub keeps Plane latch set",
    },
    RcPortItem {
        name: "HAL raw PWM I/O",
        status: PortStatus::Remaining,
        note: "AP_HAL RCInput owns pulse capture; this crate converts PWM",
    },
    RcPortItem {
        name: "hardware RC protocols",
        status: PortStatus::Remaining,
        note: "SBUS/CRSF/DSM/PPM backends live in AP_RCProtocol / HAL",
    },
    RcPortItem {
        name: "log-replay",
        status: PortStatus::Remaining,
        note: "ADR-0008 differential vs recorded outputs",
    },
];

/// Rows already hooked up on `main` (must not be redone).
#[must_use]
pub fn on_main_items() -> impl Iterator<Item = &'static RcPortItem> {
    RC_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::OnMain)
}

/// Rows added by this closing slice.
#[must_use]
pub fn this_slice_items() -> impl Iterator<Item = &'static RcPortItem> {
    RC_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::ThisSlice)
}

/// Rows left for HAL / protocol / replay (not blocking FW-019 SITL close).
#[must_use]
pub fn remaining_items() -> impl Iterator<Item = &'static RcPortItem> {
    RC_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::Remaining)
}

/// Count rows in each status bucket.
#[must_use]
pub fn completeness_counts() -> (usize, usize, usize) {
    let mut on_main = 0;
    let mut this_slice = 0;
    let mut remaining = 0;
    for item in RC_COMPLETENESS {
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
    RC_COMPLETENESS
        .iter()
        .any(|item| item.name == name && item.status == status)
}

/// True when every name in the table appears once.
#[must_use]
pub fn completeness_unique_names() -> bool {
    for (i, item) in RC_COMPLETENESS.iter().enumerate() {
        for other in RC_COMPLETENESS.iter().skip(i + 1) {
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
        assert_eq!(on_main, 12);
        assert_eq!(this_slice, 1);
        assert_eq!(remaining, 4);
        assert!(completeness_has("PWM scaling + deadzone", PortStatus::OnMain));
        assert!(completeness_has(
            "RC_REVERSED / per-channel reverse",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "FLTMODE1-6 six-position mapping",
            PortStatus::OnMain
        ));
        assert!(completeness_has("completeness table", PortStatus::ThisSlice));
        assert!(completeness_has("HAL raw PWM I/O", PortStatus::Remaining));
        assert_eq!(on_main_items().count(), 12);
        assert_eq!(this_slice_items().count(), 1);
        assert_eq!(remaining_items().count(), 4);
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
