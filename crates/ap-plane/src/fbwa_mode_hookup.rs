//! FBWA mode glue for the main vehicle loop.
//!
//! Upstream `ModeFBWA::update` maps RC roll/pitch sticks into nav demands
//! before stabilization runs. Stabilization is enabled via
//! [`dispatch_stabilize_from_mode`](crate::mode_table_hookup::dispatch_stabilize_from_mode).

use ap_math::scalar::constrain_int32;

use crate::mode_table::{BuildFeatures, ModeNumber};
use crate::stabilize_hookup::fly_inverted;

fn is_fbwa_mode(control_mode: u8, features: &BuildFeatures) -> bool {
    ModeNumber::from_number(control_mode, features) == Some(ModeNumber::FlyByWireA)
}

/// Inputs for FBWA nav demand tick (`ModeFBWA::update` nav half).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FbwaModeNavInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    pub roll_norm: f32,
    pub pitch_norm: f32,
    pub roll_limit_cd: i32,
    pub pitch_limit_min_cd: i32,
    pub pitch_limit_max_cd: i32,
    pub roll_sensor_cd: i32,
}

/// Result of the FBWA nav demand tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FbwaModeNavOutput {
    pub nav_roll_cd: i32,
    pub nav_pitch_cd: i32,
    pub applied: bool,
}

/// Map pitch stick to nav pitch, upstream `ModeFBWA::update` asymmetric path.
#[must_use]
pub fn fbwa_nav_pitch_from_stick(
    pitch_norm: f32,
    pitch_limit_min_cd: i32,
    pitch_limit_max_cd: i32,
) -> i32 {
    if pitch_norm > 0.0 {
        (pitch_norm * pitch_limit_max_cd as f32) as i32
    } else {
        (-(pitch_norm * pitch_limit_min_cd as f32)) as i32
    }
}

/// Map RC sticks into nav roll/pitch when FBWA is active.
#[must_use]
pub fn fbwa_mode_nav_tick(inp: &FbwaModeNavInputs) -> FbwaModeNavOutput {
    if !is_fbwa_mode(inp.control_mode, &inp.features) {
        return FbwaModeNavOutput {
            nav_roll_cd: 0,
            nav_pitch_cd: 0,
            applied: false,
        };
    }

    let nav_roll_cd = (inp.roll_norm * inp.roll_limit_cd as f32) as i32;
    let mut nav_pitch_cd = fbwa_nav_pitch_from_stick(
        inp.pitch_norm,
        inp.pitch_limit_min_cd,
        inp.pitch_limit_max_cd,
    );
    nav_pitch_cd = constrain_int32(
        nav_pitch_cd,
        inp.pitch_limit_min_cd,
        inp.pitch_limit_max_cd,
    );
    if fly_inverted(inp.roll_sensor_cd) {
        nav_pitch_cd = -nav_pitch_cd;
    }

    FbwaModeNavOutput {
        nav_roll_cd,
        nav_pitch_cd,
        applied: true,
    }
}
