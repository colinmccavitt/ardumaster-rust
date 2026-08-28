//! Training mode glue for the main vehicle loop.
//!
//! Upstream `ModeTraining::update` holds nav roll/pitch at the attitude
//! limits when the aircraft is past them, and zeros nav (manual surfaces)
//! while inside the envelope. Stabilization is skipped via
//! [`dispatch_stabilize_from_mode`](crate::mode_table_hookup::dispatch_stabilize_from_mode)
//! so `ModeTraining::run` can mix manual vs hold itself.

use crate::mode_table::{BuildFeatures, ModeNumber};
use crate::stabilize_hookup::fly_inverted;

fn is_training_mode(control_mode: u8, features: &BuildFeatures) -> bool {
    ModeNumber::from_number(control_mode, features) == Some(ModeNumber::Training)
}

/// Inputs for Training nav demand tick (`ModeTraining::update`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TrainingModeNavInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    pub roll_sensor_cd: i32,
    pub pitch_sensor_cd: i32,
    pub roll_limit_cd: i32,
    pub pitch_limit_min_cd: i32,
    pub pitch_limit_max_cd: i32,
}

/// Result of the Training nav demand tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TrainingModeNavOutput {
    pub nav_roll_cd: i32,
    pub nav_pitch_cd: i32,
    pub training_manual_roll: bool,
    pub training_manual_pitch: bool,
    pub applied: bool,
}

/// Clamp nav to envelope limits when TRAINING is active.
#[must_use]
pub fn training_mode_nav_tick(inp: &TrainingModeNavInputs) -> TrainingModeNavOutput {
    if !is_training_mode(inp.control_mode, &inp.features) {
        return TrainingModeNavOutput {
            nav_roll_cd: 0,
            nav_pitch_cd: 0,
            training_manual_roll: false,
            training_manual_pitch: false,
            applied: false,
        };
    }

    let (nav_roll_cd, training_manual_roll) = if inp.roll_sensor_cd >= inp.roll_limit_cd {
        (inp.roll_limit_cd, false)
    } else if inp.roll_sensor_cd <= -inp.roll_limit_cd {
        (-inp.roll_limit_cd, false)
    } else {
        (0, true)
    };

    let (mut nav_pitch_cd, training_manual_pitch) = if inp.pitch_sensor_cd >= inp.pitch_limit_max_cd
    {
        (inp.pitch_limit_max_cd, false)
    } else if inp.pitch_sensor_cd <= inp.pitch_limit_min_cd {
        (inp.pitch_limit_min_cd, false)
    } else {
        (0, true)
    };
    if fly_inverted(inp.roll_sensor_cd) {
        nav_pitch_cd = -nav_pitch_cd;
    }

    TrainingModeNavOutput {
        nav_roll_cd,
        nav_pitch_cd,
        training_manual_roll,
        training_manual_pitch,
        applied: true,
    }
}
