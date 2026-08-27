//! Mode entry state reset hookup for the scheduler tick.
//!
//! Upstream `Mode::enter` clears auto/steer/crash state before the mode's
//! `_enter()` runs, then sets `throttle_suppressed` from `does_auto_throttle`.

use crate::entry_state::ModeEntryState;
use crate::mode_table::{BuildFeatures, ModeNumber};

/// HAL inputs for one mode-entry scheduler tick.
#[derive(Debug, Clone, Copy)]
pub struct ModeEntrySchedulerInputs {
    pub control_mode: u8,
    pub previous_tracked_mode: u8,
    pub current_pitch_cd: i16,
    pub features: BuildFeatures,
}

/// Result of one mode-entry scheduler tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModeEntrySchedulerOutput {
    pub mode_changed: bool,
    pub tracked_mode: u8,
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

/// Reset mode-entry state when `control_mode` changes upstream `Mode::enter`.
#[must_use]
pub fn mode_entry_scheduler_tick(
    entry: &mut ModeEntryState,
    inp: &ModeEntrySchedulerInputs,
) -> ModeEntrySchedulerOutput {
    if inp.control_mode == inp.previous_tracked_mode {
        return ModeEntrySchedulerOutput {
            mode_changed: false,
            tracked_mode: inp.previous_tracked_mode,
        };
    }

    let Some(mode) = ModeNumber::from_number(inp.control_mode, &inp.features) else {
        return ModeEntrySchedulerOutput {
            mode_changed: false,
            tracked_mode: inp.control_mode,
        };
    };

    entry.reset(inp.current_pitch_cd, is_vtol_mode(mode));
    entry.after_enter(does_auto_throttle(mode));

    ModeEntrySchedulerOutput {
        mode_changed: true,
        tracked_mode: inp.control_mode,
    }
}
