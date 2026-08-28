//! VT-002 VTOL_Assist completeness: surfaces already on main vs leftover.
//!
//! Catalogs the `ArduPlane/VTOL_Assist` port. Items marked
//! [`PortStatus::OnMain`] landed in earlier slices and must not be
//! redone. [`PortStatus::ThisSlice`] is this table plus leftover-API
//! contract helpers. [`PortStatus::Remaining`] are the leftover
//! `VTOL_Assist.cpp` / `.h` surfaces not yet stubbed (state-update
//! tick, assist-active latch, recovery, logging / GCS bits, leftover
//! option paths).
//!
//! This module does not rewrite [`crate::assist`], [`crate::speed_alt`],
//! [`crate::force`], or [`crate::angle`].

use crate::assist::{AssistOption, ASSIST_DELAY_DEFAULT};

/// Whether a catalog row is already hooked up or left for later work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortStatus {
    /// Present on `main` before this closing slice.
    OnMain,
    /// Added by the VT-002 closing slice (this table).
    ThisSlice,
    /// Leftover `VTOL_Assist.cpp` / `.h` surface, not yet stubbed.
    Remaining,
}

/// One VTOL_Assist surface in the completeness table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AssistPortItem {
    /// Surface name.
    pub name: &'static str,
    /// Hooked up on main / this slice, or remaining.
    pub status: PortStatus,
    /// Short note (upstream symbol or why remaining).
    pub note: &'static str,
}

/// Completeness table: ported VTOL_Assist stubs vs leftover API.
///
/// Row names match the closer catalog: enable/check, speed/alt,
/// force, angle, this table, then leftover `VTOL_Assist.cpp` / `.h`
/// surfaces.
pub const ASSIST_COMPLETENESS: &[AssistPortItem] = &[
    AssistPortItem {
        name: "enable/check",
        status: PortStatus::OnMain,
        note: "assist.rs VtolAssist / should_check / is_enabled / STATE / OPTION",
    },
    AssistPortItem {
        name: "speed/alt trigger",
        status: PortStatus::OnMain,
        note: "speed_alt.rs evaluate_speed_alt (no Q_ASSIST_DELAY hysteresis)",
    },
    AssistPortItem {
        name: "force/option-bit",
        status: PortStatus::OnMain,
        note: "force.rs Q_ASSIST_FORCE_ENABLE / FORCE_ENABLED / spin-while-armed",
    },
    AssistPortItem {
        name: "angle-error",
        status: PortStatus::OnMain,
        note: "angle.rs evaluate_angle / Q_ASSIST_ANGLE (no delay hysteresis)",
    },
    AssistPortItem {
        name: "completeness table",
        status: PortStatus::ThisSlice,
        note: "this catalog + leftover API contract helpers",
    },
    AssistPortItem {
        name: "state update tick",
        status: PortStatus::Remaining,
        note: "Assist_Hysteresis::update Q_ASSIST_DELAY trigger/clear (not stubbed)",
    },
    AssistPortItem {
        name: "assist active latch",
        status: PortStatus::Remaining,
        note: "should_assist OR + reset() live flags (not stubbed)",
    },
    AssistPortItem {
        name: "recovery",
        status: PortStatus::Remaining,
        note: "check_VTOL_recovery / output_spin_recovery (not stubbed)",
    },
    AssistPortItem {
        name: "logging/GCS bits",
        status: PortStatus::Remaining,
        note: "in_* getters + Alt/Angle assist STATUSTEXT (not stubbed)",
    },
    AssistPortItem {
        name: "leftover option paths",
        status: PortStatus::Remaining,
        note: "OPTION FW_FORCE_DISABLED / SPIN_DISABLED recovery paths (not stubbed)",
    },
];

/// GCS warning prefix for altitude assist. Upstream
/// `gcs().send_text(..., "Alt assist %.1fm", ...)`.
pub const GCS_ALT_ASSIST_PREFIX: &str = "Alt assist";

/// GCS warning prefix for angle assist. Upstream
/// `gcs().send_text(..., "Angle assist r=%d p=%d", ...)`.
pub const GCS_ANGLE_ASSIST_PREFIX: &str = "Angle assist";

/// Logging getter names, upstream `in_force_assist` / `in_speed_assist`
/// / `in_alt_assist` / `in_angle_assist`.
pub const LOGGING_GETTERS: &[&str] = &[
    "in_force_assist",
    "in_speed_assist",
    "in_alt_assist",
    "in_angle_assist",
];

/// Attitude multiple of `Q_A_ANGLE_MAX` that starts FW recovery.
/// Upstream `abs_angle_cd > 2 * angle_max_cd`.
pub const RECOVERY_ANGLE_MULT: f32 = 2.0;

/// Yaw-rate threshold (deg/s) for spin recovery. Upstream `radians(10)`.
pub const SPIN_YAW_RATE_DEG: f32 = 10.0;

/// Roll-rate threshold (deg/s) for spin recovery. Upstream `radians(30)`.
pub const SPIN_ROLL_RATE_DEG: f32 = 30.0;

/// Pitch-rate threshold (deg/s) for spin recovery. Upstream `radians(30)`.
pub const SPIN_PITCH_RATE_DEG: f32 = 30.0;

/// Pitch (deg) that must be below this for spin recovery. Upstream `-45`.
pub const SPIN_PITCH_DEG: f32 = -45.0;

/// `Q_ASSIST_DELAY` seconds → trigger hysteresis, ms.
///
/// Upstream `tigger_delay_ms = delay * 1000` (sic).
#[must_use]
pub fn trigger_delay_ms(delay_s: f32) -> u32 {
    (delay_s * 1000.0) as u32
}

/// Clear hysteresis is twice the trigger delay.
///
/// Upstream `clear_delay_ms = tigger_delay_ms * 2`.
#[must_use]
pub fn clear_delay_ms(delay_s: f32) -> u32 {
    trigger_delay_ms(delay_s).saturating_mul(2)
}

/// Default trigger delay from `Q_ASSIST_DELAY` 0.5 s → 500 ms.
#[must_use]
pub fn default_trigger_delay_ms() -> u32 {
    trigger_delay_ms(ASSIST_DELAY_DEFAULT)
}

/// Default clear delay from `Q_ASSIST_DELAY` 0.5 s → 1000 ms.
#[must_use]
pub fn default_clear_delay_ms() -> u32 {
    clear_delay_ms(ASSIST_DELAY_DEFAULT)
}

/// Live assist latch. Upstream `should_assist` return:
/// `force_assist || speed_assist || alt_error.is_active() ||
/// angle_error.is_active()`.
#[must_use]
pub const fn assist_active(
    force_assist: bool,
    speed_assist: bool,
    alt_assist: bool,
    angle_assist: bool,
) -> bool {
    force_assist || speed_assist || alt_assist || angle_assist
}

/// `Q_ASSIST_OPTIONS` bit 0 blocks FW recovery.
///
/// Upstream `check_VTOL_recovery`: `!option_is_set(FW_FORCE_DISABLED)`.
#[must_use]
pub const fn fw_recovery_option_blocked(options: i16) -> bool {
    (options & AssistOption::FwForceDisabled.as_i16()) != 0
}

/// `Q_ASSIST_OPTIONS` bit 1 blocks spin recovery.
///
/// Upstream `check_VTOL_recovery`: `!option_is_set(SPIN_DISABLED)`.
#[must_use]
pub const fn spin_recovery_option_blocked(options: i16) -> bool {
    (options & AssistOption::SpinDisabled.as_i16()) != 0
}

/// Rows already hooked up on `main` (must not be redone).
#[must_use]
pub fn on_main_items() -> impl Iterator<Item = &'static AssistPortItem> {
    ASSIST_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::OnMain)
}

/// Rows added by this closing slice.
#[must_use]
pub fn this_slice_items() -> impl Iterator<Item = &'static AssistPortItem> {
    ASSIST_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::ThisSlice)
}

/// Leftover `VTOL_Assist.cpp` / `.h` surfaces not yet stubbed.
#[must_use]
pub fn remaining_items() -> impl Iterator<Item = &'static AssistPortItem> {
    ASSIST_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::Remaining)
}

/// Count rows in each status bucket.
#[must_use]
pub fn completeness_counts() -> (usize, usize, usize) {
    let mut on_main = 0;
    let mut this_slice = 0;
    let mut remaining = 0;
    for item in ASSIST_COMPLETENESS {
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
    ASSIST_COMPLETENESS
        .iter()
        .any(|item| item.name == name && item.status == status)
}

/// True when every name in the table appears once.
#[must_use]
pub fn completeness_unique_names() -> bool {
    for (i, item) in ASSIST_COMPLETENESS.iter().enumerate() {
        for other in ASSIST_COMPLETENESS.iter().skip(i + 1) {
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
    fn table_covers_main_surfaces_and_leftover_api() {
        assert!(completeness_unique_names());
        let (on_main, this_slice, remaining) = completeness_counts();
        assert_eq!(on_main, 4);
        assert_eq!(this_slice, 1);
        assert_eq!(remaining, 5);
        assert!(completeness_has("enable/check", PortStatus::OnMain));
        assert!(completeness_has("speed/alt trigger", PortStatus::OnMain));
        assert!(completeness_has("force/option-bit", PortStatus::OnMain));
        assert!(completeness_has("angle-error", PortStatus::OnMain));
        assert!(completeness_has(
            "completeness table",
            PortStatus::ThisSlice
        ));
        assert!(completeness_has("state update tick", PortStatus::Remaining));
        assert!(completeness_has(
            "assist active latch",
            PortStatus::Remaining
        ));
        assert!(completeness_has("recovery", PortStatus::Remaining));
        assert!(completeness_has("logging/GCS bits", PortStatus::Remaining));
        assert!(completeness_has(
            "leftover option paths",
            PortStatus::Remaining
        ));
        assert_eq!(on_main_items().count(), 4);
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

    #[test]
    fn leftover_delay_contract_matches_upstream() {
        assert_eq!(ASSIST_DELAY_DEFAULT, 0.5);
        assert_eq!(trigger_delay_ms(0.5), 500);
        assert_eq!(clear_delay_ms(0.5), 1000);
        assert_eq!(default_trigger_delay_ms(), 500);
        assert_eq!(default_clear_delay_ms(), 1000);
        assert_eq!(trigger_delay_ms(0.0), 0);
        assert_eq!(clear_delay_ms(1.0), 2000);
    }

    #[test]
    fn leftover_latch_or_matches_should_assist_return() {
        assert!(!assist_active(false, false, false, false));
        assert!(assist_active(true, false, false, false));
        assert!(assist_active(false, true, false, false));
        assert!(assist_active(false, false, true, false));
        assert!(assist_active(false, false, false, true));
        assert!(assist_active(true, true, true, true));
    }

    #[test]
    fn leftover_logging_and_gcs_names_match_upstream() {
        assert_eq!(GCS_ALT_ASSIST_PREFIX, "Alt assist");
        assert_eq!(GCS_ANGLE_ASSIST_PREFIX, "Angle assist");
        assert_eq!(
            LOGGING_GETTERS,
            &[
                "in_force_assist",
                "in_speed_assist",
                "in_alt_assist",
                "in_angle_assist",
            ]
        );
    }

    #[test]
    fn leftover_recovery_thresholds_and_option_paths() {
        assert_eq!(RECOVERY_ANGLE_MULT, 2.0);
        assert_eq!(SPIN_YAW_RATE_DEG, 10.0);
        assert_eq!(SPIN_ROLL_RATE_DEG, 30.0);
        assert_eq!(SPIN_PITCH_RATE_DEG, 30.0);
        assert_eq!(SPIN_PITCH_DEG, -45.0);
        assert!(!fw_recovery_option_blocked(0));
        assert!(!spin_recovery_option_blocked(0));
        assert!(fw_recovery_option_blocked(
            AssistOption::FwForceDisabled.as_i16()
        ));
        assert!(spin_recovery_option_blocked(
            AssistOption::SpinDisabled.as_i16()
        ));
        assert!(!fw_recovery_option_blocked(
            AssistOption::SpinDisabled.as_i16()
        ));
        assert!(!spin_recovery_option_blocked(
            AssistOption::FwForceDisabled.as_i16()
        ));
    }
}
