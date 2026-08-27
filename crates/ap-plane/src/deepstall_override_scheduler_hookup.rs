//! Vehicle loop deepstall servo override hookup for the scheduler tick.
//!
//! Upstream `Plane::set_servos` calls `landing.override_servos()` when
//! `flight_stage == LAND` and deepstall is in the land stage.

use ap_landing::deepstall_override::DeepstallOverrideInputs;

use crate::go_around_hookup::apply_landing_go_around_latch;
use crate::landing_hookup::{landing_servo_hookup, LandingServoHookupInputs, ServoOutputState};
use crate::landing_loop::LandingContext;

/// HAL inputs for one deepstall-override scheduler tick.
#[derive(Debug, Clone, Copy)]
pub struct DeepstallOverrideSchedulerInputs {
    pub flight_stage_is_land: bool,
    pub deepstall: DeepstallOverrideInputs,
}

/// Result of one deepstall-override scheduler tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DeepstallOverrideSchedulerOutput {
    pub applied_override: bool,
    pub request_go_around: bool,
    pub servos: ServoOutputState,
}

/// Apply deepstall landing servo overrides during the `set_servos` scheduler tick.
#[must_use]
pub fn deepstall_override_scheduler_tick(
    landing: &mut LandingContext,
    base_servos: ServoOutputState,
    inp: &DeepstallOverrideSchedulerInputs,
) -> DeepstallOverrideSchedulerOutput {
    let hookup_inp = LandingServoHookupInputs {
        flight_stage_is_land: inp.flight_stage_is_land,
        landing_flags: landing.flags,
        landing_type: landing.landing_type,
        deepstall_stage: landing.machine.deepstall.stage,
        deepstall: inp.deepstall,
    };
    let result = landing_servo_hookup(base_servos, &hookup_inp);
    apply_landing_go_around_latch(&mut landing.flags, result.request_go_around);
    DeepstallOverrideSchedulerOutput {
        applied_override: result.applied_override,
        request_go_around: result.request_go_around,
        servos: result.outputs,
    }
}
