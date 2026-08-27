//! Mode glue for the main vehicle loop.
//!
//! Upstream mode entry sets `throttle_suppressed` in auto-throttle modes and
//! `StickMixing::VtolYaw` only applies in VTOL modes. This module connects
//! those mode-dependent facts to the pilot-throttle and stabilize glue paths.

use crate::mode_run::StickMixing;
use crate::mode_table::{BuildFeatures, ModeNumber};

/// HAL inputs for one mode-glue tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModeGlueInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    pub stick_mixing: Option<StickMixing>,
    pub throttle_suppressed: bool,
    pub pilot_throttle: f32,
}

impl Default for ModeGlueInputs {
    fn default() -> Self {
        Self {
            control_mode: ModeNumber::FlyByWireB.as_number(),
            features: BuildFeatures::default(),
            stick_mixing: Some(StickMixing::Fbw),
            throttle_suppressed: false,
            pilot_throttle: 0.0,
        }
    }
}

/// Result of one mode-glue tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModeGlueOutput {
    pub effective_stick_mixing: Option<StickMixing>,
    pub pilot_throttle: f32,
    pub throttle_zeroed_by_mode_entry: bool,
}

fn is_vtol_mode(mode: ModeNumber) -> bool {
    matches!(
        mode,
        ModeNumber::QStabilize
            | ModeNumber::QHover
            | ModeNumber::QLoiter
            | ModeNumber::QLand
            | ModeNumber::QRtl
            | ModeNumber::QAutotune
            | ModeNumber::QAcro
            | ModeNumber::LoiterAltQLand
    )
}

fn does_auto_throttle(mode: ModeNumber) -> bool {
    !matches!(
        mode,
        ModeNumber::Manual
            | ModeNumber::Stabilize
            | ModeNumber::Training
            | ModeNumber::Acro
            | ModeNumber::FlyByWireA
            | ModeNumber::Autotune
            | ModeNumber::QAcro
    )
}

/// Resolve stick mixing for the active mode.
///
/// VTOL yaw mixing only applies in VTOL modes; FBW mixing is fixed-wing only.
#[must_use]
pub fn resolve_effective_stick_mixing(
    mode: ModeNumber,
    stick_mixing: Option<StickMixing>,
) -> Option<StickMixing> {
    match stick_mixing {
        Some(StickMixing::VtolYaw) if is_vtol_mode(mode) => Some(StickMixing::VtolYaw),
        Some(StickMixing::VtolYaw) => None,
        other if is_vtol_mode(mode) => None,
        other => other,
    }
}

/// Zero pilot throttle when mode entry suppresses an auto-throttle mode.
#[must_use]
pub fn apply_mode_entry_throttle_suppression(
    control_mode: u8,
    features: &BuildFeatures,
    throttle_suppressed: bool,
    pilot_throttle: f32,
) -> (f32, bool) {
    let Some(mode) = ModeNumber::from_number(control_mode, features) else {
        return (pilot_throttle, false);
    };
    if does_auto_throttle(mode) && throttle_suppressed {
        (0.0, true)
    } else {
        (pilot_throttle, false)
    }
}

/// Apply mode-entry throttle suppression and resolve effective stick mixing.
#[must_use]
pub fn mode_glue_tick(inp: &ModeGlueInputs) -> ModeGlueOutput {
    let effective_stick_mixing = ModeNumber::from_number(inp.control_mode, &inp.features)
        .map(|mode| resolve_effective_stick_mixing(mode, inp.stick_mixing))
        .unwrap_or(inp.stick_mixing);

    let (pilot_throttle, zeroed) = apply_mode_entry_throttle_suppression(
        inp.control_mode,
        &inp.features,
        inp.throttle_suppressed,
        inp.pilot_throttle,
    );

    ModeGlueOutput {
        effective_stick_mixing,
        pilot_throttle,
        throttle_zeroed_by_mode_entry: zeroed,
    }
}
