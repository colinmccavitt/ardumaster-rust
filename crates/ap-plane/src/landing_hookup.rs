//! Landing servo override hookup, upstream `Plane::set_servos` in
//! `ArduPlane/servos.cpp`.
//!
//! When `flight_stage == LAND`, landing may override servos before throttle is
//! set. Deepstall is the only landing type that overrides today.

use ap_landing::deepstall_override::{
    deepstall_override_servos_step, DeepstallOverrideInputs, DeepstallOverrideOutputs,
};
use ap_landing::deepstall_stage::DeepstallStage;
use ap_landing::go_around::{override_servos, LandingFlags, LandingType};

/// Pending scaled and PWM servo outputs the vehicle is about to publish.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ServoOutputState {
    pub elevator_pwm: u16,
    pub aileron_scaled: f32,
    pub rudder_scaled: f32,
    pub throttle_scaled: f32,
}

impl Default for ServoOutputState {
    fn default() -> Self {
        Self {
            elevator_pwm: 1500,
            aileron_scaled: 0.0,
            rudder_scaled: 0.0,
            throttle_scaled: 0.0,
        }
    }
}

/// Everything the vehicle loop reads to decide whether landing overrides servos.
#[derive(Debug, Clone, Copy)]
pub struct LandingServoHookupInputs {
    pub flight_stage_is_land: bool,
    pub landing_flags: LandingFlags,
    pub landing_type: LandingType,
    pub deepstall_stage: DeepstallStage,
    pub deepstall: DeepstallOverrideInputs,
}

/// Result of one landing servo hookup tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LandingServoHookupResult {
    pub applied_override: bool,
    pub outputs: ServoOutputState,
    pub request_go_around: bool,
}

/// Apply landing servo overrides when in LAND and the library requests them,
/// upstream the `landing.override_servos()` call in `set_servos`.
#[must_use]
pub fn landing_servo_hookup(
    base: ServoOutputState,
    inp: &LandingServoHookupInputs,
) -> LandingServoHookupResult {
    if !inp.flight_stage_is_land {
        return LandingServoHookupResult {
            applied_override: false,
            outputs: base,
            request_go_around: false,
        };
    }

    if !override_servos(
        &inp.landing_flags,
        inp.landing_type,
        Some(inp.deepstall_stage),
    ) {
        return LandingServoHookupResult {
            applied_override: false,
            outputs: base,
            request_go_around: false,
        };
    }

    let ds = deepstall_override_servos_step(&inp.deepstall);
    if ds.missing_elevator {
        return LandingServoHookupResult {
            applied_override: false,
            outputs: base,
            request_go_around: true,
        };
    }

    if !ds.overrides {
        return LandingServoHookupResult {
            applied_override: false,
            outputs: base,
            request_go_around: false,
        };
    }

    LandingServoHookupResult {
        applied_override: true,
        outputs: merge_deepstall_override(base, ds),
        request_go_around: false,
    }
}

fn merge_deepstall_override(
    base: ServoOutputState,
    ds: DeepstallOverrideOutputs,
) -> ServoOutputState {
    ServoOutputState {
        elevator_pwm: ds.elevator_pwm,
        aileron_scaled: ds.aileron_scaled.unwrap_or(base.aileron_scaled),
        rudder_scaled: ds.rudder_scaled.unwrap_or(base.rudder_scaled),
        throttle_scaled: ds.throttle_scaled.unwrap_or(base.throttle_scaled),
    }
}
