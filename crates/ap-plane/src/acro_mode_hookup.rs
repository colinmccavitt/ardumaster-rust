//! Acro mode glue for the main vehicle loop.
//!
//! Upstream `ModeAcro::update` publishes nav roll/pitch from the acro lock
//! state: unlocked axes mirror the attitude sensors; locked roll uses
//! `locked_roll_err` and locked pitch uses `locked_pitch_cd`. Stabilization
//! (rate lock) is enabled via
//! [`dispatch_stabilize_from_mode`](crate::mode_table_hookup::dispatch_stabilize_from_mode).

use crate::mode_table::{BuildFeatures, ModeNumber};

fn is_acro_mode(control_mode: u8, features: &BuildFeatures) -> bool {
    ModeNumber::from_number(control_mode, features) == Some(ModeNumber::Acro)
}

/// Inputs for Acro nav demand tick (`ModeAcro::update`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AcroModeNavInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    pub locked_roll: bool,
    pub locked_pitch: bool,
    pub locked_roll_err: f32,
    pub locked_pitch_cd: i32,
    pub roll_sensor_cd: i32,
    pub pitch_sensor_cd: i32,
}

/// Result of the Acro nav demand tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AcroModeNavOutput {
    pub nav_roll_cd: i32,
    pub nav_pitch_cd: i32,
    pub applied: bool,
}

/// Publish locked or sensor-mirrored nav demands when ACRO is active.
#[must_use]
pub fn acro_mode_nav_tick(inp: &AcroModeNavInputs) -> AcroModeNavOutput {
    if !is_acro_mode(inp.control_mode, &inp.features) {
        return AcroModeNavOutput {
            nav_roll_cd: 0,
            nav_pitch_cd: 0,
            applied: false,
        };
    }

    let nav_roll_cd = if inp.locked_roll {
        inp.locked_roll_err as i32
    } else {
        inp.roll_sensor_cd
    };
    let nav_pitch_cd = if inp.locked_pitch {
        inp.locked_pitch_cd
    } else {
        inp.pitch_sensor_cd
    };

    AcroModeNavOutput {
        nav_roll_cd,
        nav_pitch_cd,
        applied: true,
    }
}
