//! Stabilize mode glue for the main vehicle loop.
//!
//! Upstream `ModeStabilize::update` zeros nav roll/pitch so attitude control
//! holds wings-level. Stabilization is enabled via
//! [`dispatch_stabilize_from_mode`](crate::mode_table_hookup::dispatch_stabilize_from_mode).

use crate::mode_table::{BuildFeatures, ModeNumber};

fn is_stabilize_mode(control_mode: u8, features: &BuildFeatures) -> bool {
    ModeNumber::from_number(control_mode, features) == Some(ModeNumber::Stabilize)
}

/// Inputs for Stabilize nav demand tick (`ModeStabilize::update`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StabilizeModeNavInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
}

/// Result of the Stabilize nav demand tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StabilizeModeNavOutput {
    pub nav_roll_cd: i32,
    pub nav_pitch_cd: i32,
    pub applied: bool,
}

/// Zero nav roll/pitch when STABILIZE is active.
#[must_use]
pub fn stabilize_mode_nav_tick(inp: &StabilizeModeNavInputs) -> StabilizeModeNavOutput {
    if !is_stabilize_mode(inp.control_mode, &inp.features) {
        return StabilizeModeNavOutput {
            nav_roll_cd: 0,
            nav_pitch_cd: 0,
            applied: false,
        };
    }
    StabilizeModeNavOutput {
        nav_roll_cd: 0,
        nav_pitch_cd: 0,
        applied: true,
    }
}
