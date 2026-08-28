//! FBWB mode glue for the main vehicle loop.
//!
//! Upstream `ModeFBWB::update` maps the RC roll stick into nav roll, then
//! hands pitch/throttle to `update_fbwb_speed_height` (TECS altitude hold).
//! This tick covers the roll-stick half; commanded pitch stays on the TECS
//! feed already published by `update_control_mode`. Stabilization is enabled
//! via [`dispatch_stabilize_from_mode`](crate::mode_table_hookup::dispatch_stabilize_from_mode).

use crate::mode_table::{BuildFeatures, ModeNumber};

fn is_fbwb_mode(control_mode: u8, features: &BuildFeatures) -> bool {
    ModeNumber::from_number(control_mode, features) == Some(ModeNumber::FlyByWireB)
}

/// Inputs for FBWB nav demand tick (`ModeFBWB::update` roll half).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FbwbModeNavInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    pub roll_norm: f32,
    pub roll_limit_cd: i32,
}

/// Result of the FBWB nav demand tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FbwbModeNavOutput {
    pub nav_roll_cd: i32,
    pub applied: bool,
}

/// Map RC roll stick into nav roll when FBWB is active.
///
/// Pitch is not mapped: FBWB is cruise-assisted, so TECS owns nav pitch.
#[must_use]
pub fn fbwb_mode_nav_tick(inp: &FbwbModeNavInputs) -> FbwbModeNavOutput {
    if !is_fbwb_mode(inp.control_mode, &inp.features) {
        return FbwbModeNavOutput {
            nav_roll_cd: 0,
            applied: false,
        };
    }

    FbwbModeNavOutput {
        nav_roll_cd: (inp.roll_norm * inp.roll_limit_cd as f32) as i32,
        applied: true,
    }
}
