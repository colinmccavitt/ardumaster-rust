//! AUTOTUNE mode glue for the main vehicle loop.
//!
//! Upstream ModeAutoTune::update delegates to ModeFBWA::update, mapping
//! RC roll/pitch sticks into nav demands while AP_AutoTune (FW-040) rewrites
//! gains. Stabilization is enabled via
//! [dispatch_stabilize_from_mode](crate::mode_table_hookup::dispatch_stabilize_from_mode).

use ap_math::scalar::constrain_int32;

use crate::fbwa_mode_hookup::fbwa_nav_pitch_from_stick;
use crate::mode_table::{BuildFeatures, ModeNumber};
use crate::stabilize_hookup::fly_inverted;

fn is_autotune_mode(control_mode: u8, features: &BuildFeatures) -> bool {
    ModeNumber::from_number(control_mode, features) == Some(ModeNumber::Autotune)
}

/// Inputs for AUTOTUNE nav demand tick (ModeAutoTune::update via FBWA).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutotuneModeNavInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    pub roll_norm: f32,
    pub pitch_norm: f32,
    pub roll_limit_cd: i32,
    pub pitch_limit_min_cd: i32,
    pub pitch_limit_max_cd: i32,
    pub roll_sensor_cd: i32,
}

/// Result of the AUTOTUNE nav demand tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutotuneModeNavOutput {
    pub nav_roll_cd: i32,
    pub nav_pitch_cd: i32,
    pub applied: bool,
}

/// Map RC sticks into nav roll/pitch when AUTOTUNE is active.
///
/// Same mapping as FBWA: Autotune's update is plane.mode_fbwa.update().
#[must_use]
pub fn autotune_mode_nav_tick(inp: &AutotuneModeNavInputs) -> AutotuneModeNavOutput {
    if !is_autotune_mode(inp.control_mode, &inp.features) {
        return AutotuneModeNavOutput {
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

    AutotuneModeNavOutput {
        nav_roll_cd,
        nav_pitch_cd,
        applied: true,
    }
}
