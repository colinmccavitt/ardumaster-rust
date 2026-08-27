//! Mode-entry throttle suppression hookup for the scheduler tick.
//!
//! Upstream `Plane::suppress_throttle()` in `servos.cpp` zeros throttle when
//! `throttle_suppressed` is set on mode entry in an auto-throttle mode.

use crate::landing_hookup::ServoOutputState;
use crate::mode_table::{BuildFeatures, ModeNumber};

/// HAL inputs for one suppress-throttle scheduler tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SuppressThrottleSchedulerInputs {
    pub control_mode: u8,
    pub throttle_suppressed: bool,
    pub features: BuildFeatures,
}

/// Result of one suppress-throttle scheduler tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SuppressThrottleSchedulerOutput {
    pub applied: bool,
    pub servos: ServoOutputState,
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

/// Zero throttle when mode-entry suppression is active in an auto-throttle mode.
#[must_use]
pub fn suppress_throttle_scheduler_tick(
    servos: ServoOutputState,
    inp: &SuppressThrottleSchedulerInputs,
) -> SuppressThrottleSchedulerOutput {
    if ModeNumber::from_number(inp.control_mode, &inp.features)
        == Some(ModeNumber::Manual)
    {
        return SuppressThrottleSchedulerOutput {
            applied: false,
            servos,
        };
    }

    let Some(mode) = ModeNumber::from_number(inp.control_mode, &inp.features) else {
        return SuppressThrottleSchedulerOutput {
            applied: false,
            servos,
        };
    };

    if does_auto_throttle(mode) && inp.throttle_suppressed {
        return SuppressThrottleSchedulerOutput {
            applied: true,
            servos: ServoOutputState {
                throttle_scaled: 0.0,
                ..servos
            },
        };
    }

    SuppressThrottleSchedulerOutput {
        applied: false,
        servos,
    }
}
