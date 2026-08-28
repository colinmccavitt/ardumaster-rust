//! CRUISE mode glue for the main vehicle loop.
//!
//! Upstream `ModeCruise::update` maps the RC roll stick into nav roll while
//! heading is unlocked, and leaves roll to `calc_nav_roll` once heading is
//! locked. Aileron, rudder, or an active nav script unlocks heading.
//! Pitch/throttle stay on the TECS feed already published by
//! `update_fbwb_speed_height`. Stabilization is enabled via
//! [`dispatch_stabilize_from_mode`](crate::mode_table_hookup::dispatch_stabilize_from_mode).

use crate::mode_table::{BuildFeatures, ModeNumber};

fn is_cruise_mode(control_mode: u8, features: &BuildFeatures) -> bool {
    ModeNumber::from_number(control_mode, features) == Some(ModeNumber::Cruise)
}

fn stick_deflected(norm: f32) -> bool {
    norm > 0.0 || norm < 0.0
}

/// Inputs for CRUISE nav demand tick (`ModeCruise::update` roll half).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CruiseModeNavInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    pub roll_norm: f32,
    pub rudder_norm: f32,
    pub locked_heading: bool,
    pub nav_scripting_active: bool,
    pub roll_limit_cd: i32,
    /// Existing nav roll from `calc_nav_roll` when heading is locked.
    pub commanded_roll_cd: i32,
}

/// Result of the CRUISE nav demand tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CruiseModeNavOutput {
    pub nav_roll_cd: i32,
    pub locked_heading: bool,
    pub applied: bool,
}

/// Map RC roll into nav roll, or hold the nav-controller demand, when CRUISE
/// is active.
///
/// Pitch is not mapped: CRUISE is cruise-assisted, so TECS owns nav pitch.
#[must_use]
pub fn cruise_mode_nav_tick(inp: &CruiseModeNavInputs) -> CruiseModeNavOutput {
    if !is_cruise_mode(inp.control_mode, &inp.features) {
        return CruiseModeNavOutput {
            nav_roll_cd: 0,
            locked_heading: inp.locked_heading,
            applied: false,
        };
    }

    let mut locked_heading = inp.locked_heading;
    if stick_deflected(inp.roll_norm)
        || stick_deflected(inp.rudder_norm)
        || inp.nav_scripting_active
    {
        locked_heading = false;
    }

    let nav_roll_cd = if locked_heading {
        inp.commanded_roll_cd
    } else {
        (inp.roll_norm * inp.roll_limit_cd as f32) as i32
    };

    CruiseModeNavOutput {
        nav_roll_cd,
        locked_heading,
        applied: true,
    }
}
