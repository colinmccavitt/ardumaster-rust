//! CIRCLE mode glue for the main vehicle loop.
//!
//! Upstream ModeCircle::update banks at one-third of the roll limit — a
//! gentle GPS-free circle — and leaves pitch/throttle to calc_nav_pitch /
//! calc_throttle (TECS). Stabilization is skipped via
//! [dispatch_stabilize_from_mode](crate::mode_table_hookup::dispatch_stabilize_from_mode)
//! (Circle => StabilizeDispatch::default()).

use crate::mode_table::{BuildFeatures, ModeNumber};

fn is_circle_mode(control_mode: u8, features: &BuildFeatures) -> bool {
    ModeNumber::from_number(control_mode, features) == Some(ModeNumber::Circle)
}

/// Inputs for CIRCLE nav demand tick (ModeCircle::update roll half).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CircleModeNavInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    pub roll_limit_cd: i32,
}

/// Result of the CIRCLE nav demand tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CircleModeNavOutput {
    pub nav_roll_cd: i32,
    pub applied: bool,
}

/// Bank at one-third of the roll limit when CIRCLE is active.
///
/// Pitch is not mapped: CIRCLE is loiter-assisted, so TECS owns nav pitch.
#[must_use]
pub fn circle_mode_nav_tick(inp: &CircleModeNavInputs) -> CircleModeNavOutput {
    if !is_circle_mode(inp.control_mode, &inp.features) {
        return CircleModeNavOutput {
            nav_roll_cd: 0,
            applied: false,
        };
    }

    CircleModeNavOutput {
        nav_roll_cd: inp.roll_limit_cd / 3,
        applied: true,
    }
}
