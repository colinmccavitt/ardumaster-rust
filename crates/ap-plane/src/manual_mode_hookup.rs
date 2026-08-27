//! Manual mode glue for the main vehicle loop.
//!
//! Upstream ModeManual::update mirrors the current attitude into nav demands
//! and passes RC sticks directly to scaled surface outputs. Stabilization is
//! skipped via [dispatch_stabilize_from_mode](crate::mode_table_hookup::dispatch_stabilize_from_mode).

use crate::landing_hookup::ServoOutputState;
use crate::mode_table::{BuildFeatures, ModeNumber};
use crate::stabilize_hookup::{scaled_to_pwm_trim, RcStickInputs};

/// Upstream SERVO_MAX, scaled centidegrees.
pub const SERVO_MAX: f32 = 4500.0;

fn is_manual_mode(control_mode: u8, features: &BuildFeatures) -> bool {
    ModeNumber::from_number(control_mode, features) == Some(ModeNumber::Manual)
}

/// Inputs for manual-mode nav mirror tick (ModeManual::update nav half).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ManualModeNavInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    pub roll_sensor_cd: i32,
    pub pitch_sensor_cd: i32,
}

/// Result of the manual-mode nav mirror tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ManualModeNavOutput {
    pub nav_roll_cd: i32,
    pub nav_pitch_cd: i32,
    pub applied: bool,
}

/// Mirror attitude sensors into nav demands when MANUAL is active.
#[must_use]
pub fn manual_mode_nav_tick(inp: &ManualModeNavInputs) -> ManualModeNavOutput {
    if !is_manual_mode(inp.control_mode, &inp.features) {
        return ManualModeNavOutput {
            nav_roll_cd: 0,
            nav_pitch_cd: 0,
            applied: false,
        };
    }
    ManualModeNavOutput {
        nav_roll_cd: inp.roll_sensor_cd,
        nav_pitch_cd: inp.pitch_sensor_cd,
        applied: true,
    }
}

/// Inputs for manual-mode direct servo passthrough.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ManualModeServosInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    pub rc_sticks: RcStickInputs,
}

/// Result of the manual-mode servo passthrough tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ManualModeServosOutput {
    pub servos: ServoOutputState,
    pub applied: bool,
}

/// Map a normalized stick to scaled centidegrees (zero-expo manual path).
#[must_use]
pub fn stick_to_scaled(norm: f32) -> f32 {
    norm * SERVO_MAX
}

/// Pass RC sticks straight to surface outputs when MANUAL is active.
#[must_use]
pub fn manual_mode_servos_tick(
    servos: ServoOutputState,
    inp: &ManualModeServosInputs,
) -> ManualModeServosOutput {
    if !is_manual_mode(inp.control_mode, &inp.features) {
        return ManualModeServosOutput {
            servos,
            applied: false,
        };
    }
    let elevator_scaled = stick_to_scaled(inp.rc_sticks.pitch_norm_dz);
    ManualModeServosOutput {
        servos: ServoOutputState {
            aileron_scaled: stick_to_scaled(inp.rc_sticks.roll_norm_dz),
            rudder_scaled: stick_to_scaled(inp.rc_sticks.yaw_norm_dz),
            elevator_pwm: scaled_to_pwm_trim(elevator_scaled),
            ..servos
        },
        applied: true,
    }
}
