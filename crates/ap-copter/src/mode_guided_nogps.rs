//! `ModeGuidedNoGPS` init / run leftover, upstream
//! `ArduCopter/mode_guided_nogps.cpp`.
//!
//! Tracked as **COP-017**. Guided-NoGPS is Guided without a position
//! requirement: it only accepts attitude and a climb / thrust. The
//! leftover is that both `init` and `run` are Angle — there is no
//! pause gate and no submode switch. `ignore_checks` is unused. The
//! mode always starts (`angle_control_start`) and always runs
//! (`angle_control_run`).
//!
//! The flag leftover vs Guided is `requires_position = false`. Manual
//! throttle and autopilot stay the Guided values. User-takeoff,
//! in-guided, terrain-failsafe, and GCS/script high-throttle arming
//! are inherited from ModeGuided and stay true.

use crate::mode_guided::{
    guided_angle_control_run, guided_angle_control_start, GuidedAngleControl,
    GuidedAngleControlView, GuidedAngleStart, GuidedAngleStartView, GuidedModeFlags,
    MODE_NUMBER_GUIDED_NOGPS,
};

/// Upstream `ModeGuidedNoGPS` flags.
///
/// Only `mode_number` and `requires_position` differ from
/// [`crate::mode_guided::guided_mode_flags`]. The rest are inherited.
#[must_use]
pub const fn guided_nogps_mode_flags() -> GuidedModeFlags {
    GuidedModeFlags {
        mode_number: MODE_NUMBER_GUIDED_NOGPS,
        requires_position: false,
        has_manual_throttle: false,
        is_autopilot: true,
        has_user_takeoff: true,
        in_guided_mode: true,
        requires_terrain_failsafe: true,
        allows_gcs_or_scr_arming_with_throttle_high: true,
    }
}

/// Upstream `ModeGuidedNoGPS::init`.
///
/// Always [`guided_angle_control_start`]. Always succeeds.
/// `ignore_checks` is accepted and unused.
#[must_use]
pub fn guided_nogps_init(view: &GuidedAngleStartView, _ignore_checks: bool) -> GuidedAngleStart {
    guided_angle_control_start(view)
}

/// Upstream `ModeGuidedNoGPS::run`.
///
/// Always [`guided_angle_control_run`]. There is no `_paused` gate and
/// no `SubMode` switch — a leftover pause from Guided does not freeze
/// Guided-NoGPS.
#[must_use]
pub fn guided_nogps_run(view: &GuidedAngleControlView) -> GuidedAngleControl {
    guided_angle_control_run(view)
}
