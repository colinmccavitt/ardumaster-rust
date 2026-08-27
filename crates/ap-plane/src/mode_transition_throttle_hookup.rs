//! Mode-transition throttle unsuppress hookup for the scheduler tick.
//!
//! Upstream `Plane::suppress_throttle()` in `servos.cpp` clears
//! `throttle_suppressed` once flight conditions are met after mode entry.

use ap_gps::GpsStatus;

use crate::entry_state::ModeEntryState;
use crate::mode_table::{BuildFeatures, ModeNumber};

/// HAL inputs for one mode-transition throttle tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModeTransitionThrottleInputs {
    pub control_mode: u8,
    pub relative_altitude_m: f32,
    pub gps: Option<GpsStatus>,
    pub features: BuildFeatures,
}

/// Result of one mode-transition throttle tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModeTransitionThrottleOutput {
    pub cleared: bool,
    pub throttle_suppressed: bool,
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

fn gps_movement(gps: Option<GpsStatus>) -> bool {
    gps.is_some_and(|g| g.have_fix && g.ground_speed >= 5.0)
}

/// Clear mode-entry throttle suppression when upstream unsuppress conditions match.
#[must_use]
pub fn mode_transition_throttle_tick(
    entry: &mut ModeEntryState,
    inp: &ModeTransitionThrottleInputs,
) -> ModeTransitionThrottleOutput {
    let Some(mode) = ModeNumber::from_number(inp.control_mode, &inp.features) else {
        return ModeTransitionThrottleOutput {
            cleared: false,
            throttle_suppressed: entry.throttle_suppressed,
        };
    };

    if mode == ModeNumber::Manual || !does_auto_throttle(mode) {
        let cleared = entry.throttle_suppressed;
        entry.throttle_suppressed = false;
        return ModeTransitionThrottleOutput {
            cleared,
            throttle_suppressed: false,
        };
    }

    if !entry.throttle_suppressed {
        return ModeTransitionThrottleOutput {
            cleared: false,
            throttle_suppressed: false,
        };
    }

    if inp.relative_altitude_m.abs() >= 10.0 || gps_movement(inp.gps) {
        entry.throttle_suppressed = false;
        return ModeTransitionThrottleOutput {
            cleared: true,
            throttle_suppressed: false,
        };
    }

    ModeTransitionThrottleOutput {
        cleared: false,
        throttle_suppressed: true,
    }
}
