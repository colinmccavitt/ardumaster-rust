//! Deepstall servo override HAL output, upstream
//! `AP_Landing_Deepstall::override_servos`.
//!
//! Returns what the vehicle should write to the elevator PWM channel and the
//! scaled aileron, rudder, and throttle outputs. The steering PID itself lives
//! in the vehicle; this module takes its output as an input per ADR-0004.

use crate::deepstall::{
    deepstall_elevator_output_pwm, deepstall_elevator_slew_progress, deepstall_steering_may_run,
    deepstall_steering_output, deepstall_travel_limit,
};
use crate::deepstall_stage::DeepstallStage;

/// HAL measurements for one deepstall override tick.
#[derive(Debug, Clone, Copy)]
pub struct DeepstallOverrideInputs {
    pub stage: DeepstallStage,
    pub stall_entry_ms: u32,
    pub now_ms: u32,
    pub slew_speed: f32,
    pub initial_elevator_pwm: u16,
    pub target_elevator_pwm: u16,
    /// Equivalent airspeed, m/s. `None` becomes zero upstream when airspeed
    /// is unavailable, forcing steering on.
    pub airspeed_ms: Option<f32>,
    pub handoff_airspeed_ms: f32,
    pub handoff_lower_limit_ms: f32,
    /// Output of `ds_PID.get_pid` from the vehicle's steering update.
    pub steering_pid: f32,
    pub aileron_scalar: f32,
    /// Whether an elevator output channel exists.
    pub elevator_present: bool,
}

/// Servo outputs landing imposes during deepstall land stage.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DeepstallOverrideOutputs {
    /// Whether landing overrides the normal servo path this tick.
    pub overrides: bool,
    pub elevator_pwm: u16,
    /// Set only when the steering controller runs this tick.
    pub aileron_scaled: Option<f32>,
    pub rudder_scaled: Option<f32>,
    pub throttle_scaled: Option<f32>,
    /// Elevator channel missing — vehicle should command a go-around.
    pub missing_elevator: bool,
}

impl Default for DeepstallOverrideOutputs {
    fn default() -> Self {
        Self {
            overrides: false,
            elevator_pwm: 0,
            aileron_scaled: None,
            rudder_scaled: None,
            throttle_scaled: None,
            missing_elevator: false,
        }
    }
}

/// Compute deepstall servo overrides for one tick, upstream
/// `AP_Landing_Deepstall::override_servos`.
#[must_use]
pub fn deepstall_override_servos_step(inp: &DeepstallOverrideInputs) -> DeepstallOverrideOutputs {
    if inp.stage != DeepstallStage::Land {
        return DeepstallOverrideOutputs::default();
    }

    if !inp.elevator_present {
        return DeepstallOverrideOutputs {
            missing_elevator: true,
            ..DeepstallOverrideOutputs::default()
        };
    }

    let slew_progress = deepstall_elevator_slew_progress(
        inp.stall_entry_ms,
        inp.now_ms,
        inp.slew_speed,
    );
    let elevator_pwm = deepstall_elevator_output_pwm(
        inp.initial_elevator_pwm,
        inp.target_elevator_pwm,
        slew_progress,
    );

    let airspeed_ms = inp.airspeed_ms.unwrap_or(0.0);

    let mut out = DeepstallOverrideOutputs {
        overrides: true,
        elevator_pwm,
        ..DeepstallOverrideOutputs::default()
    };

    if deepstall_steering_may_run(slew_progress, airspeed_ms, inp.handoff_airspeed_ms) {
        let travel_limit = deepstall_travel_limit(
            airspeed_ms,
            inp.handoff_airspeed_ms,
            inp.handoff_lower_limit_ms,
        );
        let pid = deepstall_steering_output(inp.steering_pid, travel_limit);
        let scaled = pid * 4500.0;
        out.aileron_scaled = Some(scaled * inp.aileron_scalar);
        out.rudder_scaled = Some(scaled);
        out.throttle_scaled = Some(0.0);
    }

    out
}
