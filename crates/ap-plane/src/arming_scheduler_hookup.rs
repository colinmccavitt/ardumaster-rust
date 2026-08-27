//! Arming hookup for the scheduler tick.
//!
//! Upstream `Plane::set_servos` zeros throttle when disarmed
//! (`hal.util->get_soft_armed()` is false).

use crate::landing_hookup::ServoOutputState;

/// HAL inputs for one arming scheduler tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ArmingSchedulerInputs {
    /// `hal.util->get_soft_armed()`.
    pub soft_armed: bool,
}

/// Result of one arming scheduler tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArmingSchedulerOutput {
    pub applied: bool,
    pub servos: ServoOutputState,
}

/// Zero throttle when the vehicle is disarmed.
#[must_use]
pub fn arming_scheduler_tick(
    servos: ServoOutputState,
    inp: &ArmingSchedulerInputs,
) -> ArmingSchedulerOutput {
    if inp.soft_armed {
        return ArmingSchedulerOutput {
            applied: false,
            servos,
        };
    }
    ArmingSchedulerOutput {
        applied: true,
        servos: ServoOutputState {
            throttle_scaled: 0.0,
            ..servos
        },
    }
}
