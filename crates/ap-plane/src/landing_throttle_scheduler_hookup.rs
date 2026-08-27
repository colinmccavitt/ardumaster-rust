//! Landing throttle suppression hookup for the scheduler tick.
//!
//! Upstream `Plane::set_servos` zeros throttle when `AP_Landing` has
//! suppressed it during LAND (flare on slope landings, land stage on deepstall).

use crate::landing_hookup::ServoOutputState;

/// HAL inputs for one landing-throttle scheduler tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LandingThrottleSchedulerInputs {
    pub flight_stage_is_land: bool,
    pub throttle_suppressed: bool,
}

/// Result of one landing-throttle scheduler tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LandingThrottleSchedulerOutput {
    pub applied: bool,
    pub servos: ServoOutputState,
}

/// Zero throttle when landing has suppressed it during LAND.
#[must_use]
pub fn landing_throttle_scheduler_tick(
    servos: ServoOutputState,
    inp: &LandingThrottleSchedulerInputs,
) -> LandingThrottleSchedulerOutput {
    if inp.flight_stage_is_land && inp.throttle_suppressed {
        return LandingThrottleSchedulerOutput {
            applied: true,
            servos: ServoOutputState {
                throttle_scaled: 0.0,
                ..servos
            },
        };
    }
    LandingThrottleSchedulerOutput {
        applied: false,
        servos,
    }
}
