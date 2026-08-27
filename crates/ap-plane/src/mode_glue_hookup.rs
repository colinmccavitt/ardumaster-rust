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
/// Restore pilot throttle after mode transition clears entry suppression.
///
/// Upstream `Plane::suppress_throttle()` clears `throttle_suppressed` once
/// altitude or GPS movement conditions are met; the scaled throttle should
/// then reflect the pilot stick again.
#[must_use]
pub fn restore_pilot_throttle_on_transition_clear(
    transition_cleared: bool,
    throttle_suppressed: bool,
    current_throttle: f32,
    pilot_throttle: f32,
) -> (f32, bool) {
    if transition_cleared && !throttle_suppressed && current_throttle == 0.0 && pilot_throttle > 0.0
    {
        (pilot_throttle, true)
    } else {
        (current_throttle, false)
    }
}

/// Inputs for restoring pilot throttle once mode-transition clears suppression.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModeGlueRestoreInputs {
    pub transition_cleared: bool,
    pub throttle_suppressed: bool,
    pub current_throttle: f32,
    pub pilot_throttle: f32,
}

/// Result of the set_servos mode-glue restore tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModeGlueRestoreOutput {
    pub pilot_throttle: f32,
    pub restored: bool,
}

/// Re-apply pilot throttle after mode-transition clears entry suppression.
#[must_use]
pub fn mode_glue_restore_tick(inp: &ModeGlueRestoreInputs) -> ModeGlueRestoreOutput {
    let (pilot_throttle, restored) = restore_pilot_throttle_on_transition_clear(
        inp.transition_cleared,
        inp.throttle_suppressed,
        inp.current_throttle,
        inp.pilot_throttle,
    );
    ModeGlueRestoreOutput {
        pilot_throttle,
        restored,
    }
}

use crate::landing_hookup::ServoOutputState;
use crate::suppress_throttle_scheduler_hookup::{
    suppress_throttle_scheduler_tick, SuppressThrottleSchedulerInputs,
};

/// Inputs for the set_servos mode-glue throttle path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModeGlueSetServosInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    pub transition_cleared: bool,
    pub throttle_suppressed: bool,
    pub current_throttle: f32,
    pub pilot_throttle: PilotThrottleGlueInputs,
}

/// Result of the set_servos mode-glue throttle path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModeGlueSetServosOutput {
    pub servos: ServoOutputState,
    /// Stabilize demand throttle when restore syncs pilot stick.
    pub stabilize_throttle: Option<f32>,
    pub throttle_restored: bool,
    pub clear_throttle_zeroed: bool,
    pub mode_entry_applied: bool,
}

/// Restore pilot throttle on transition clear, then apply mode-entry suppression.
#[must_use]
pub fn mode_glue_set_servos_tick(
    servos: ServoOutputState,
    inp: &ModeGlueSetServosInputs,
) -> ModeGlueSetServosOutput {
    let pilot_throttle = pilot_throttle_glue_tick(&inp.pilot_throttle);
    let restore_out = mode_glue_restore_tick(&ModeGlueRestoreInputs {
        transition_cleared: inp.transition_cleared,
        throttle_suppressed: inp.throttle_suppressed,
        current_throttle: inp.current_throttle,
        pilot_throttle,
    });

    let mut servos = servos;
    let mut stabilize_throttle = None;
    if restore_out.restored {
        servos.throttle_scaled = restore_out.pilot_throttle;
        stabilize_throttle = Some(restore_out.pilot_throttle);
    }

    let sup_out = suppress_throttle_scheduler_tick(
        servos,
        &SuppressThrottleSchedulerInputs {
            control_mode: inp.control_mode,
            throttle_suppressed: inp.throttle_suppressed,
            features: inp.features,
        },
    );

    ModeGlueSetServosOutput {
        servos: sup_out.servos,
        stabilize_throttle,
        throttle_restored: restore_out.restored,
        clear_throttle_zeroed: restore_out.restored,
        mode_entry_applied: sup_out.applied,
    }
}

use crate::yaw_throttle_glue_hookup::{pilot_throttle_glue_tick, PilotThrottleGlueInputs};

/// Inputs for the update_control mode-glue path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModeGlueUpdateControlInputs {
    pub pilot_throttle: PilotThrottleGlueInputs,
    pub control_mode: u8,
    pub features: BuildFeatures,
    pub stick_mixing: Option<StickMixing>,
    pub throttle_suppressed: bool,
}

/// Result of the update_control mode-glue path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModeGlueUpdateControlOutput {
    pub effective_stick_mixing: Option<StickMixing>,
    pub pilot_throttle: f32,
    pub throttle_zeroed_by_mode_entry: bool,
}

/// Map RC throttle through pilot glue, then apply mode-entry suppression.
#[must_use]
pub fn mode_glue_update_control_tick(
    inp: &ModeGlueUpdateControlInputs,
) -> ModeGlueUpdateControlOutput {
    let pilot_throttle = pilot_throttle_glue_tick(&inp.pilot_throttle);
    let glue_out = mode_glue_tick(&ModeGlueInputs {
        control_mode: inp.control_mode,
        features: inp.features,
        stick_mixing: inp.stick_mixing,
        throttle_suppressed: inp.throttle_suppressed,
        pilot_throttle,
    });
    ModeGlueUpdateControlOutput {
        effective_stick_mixing: glue_out.effective_stick_mixing,
        pilot_throttle: glue_out.pilot_throttle,
        throttle_zeroed_by_mode_entry: glue_out.throttle_zeroed_by_mode_entry,
    }
}

