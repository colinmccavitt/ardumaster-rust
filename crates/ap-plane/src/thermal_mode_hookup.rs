//! THERMAL mode glue for the main vehicle loop.
//!
//! Upstream `ModeThermal::update` calls `calc_nav_roll` / `calc_nav_pitch` /
//! `calc_throttle` after `navigate()` loiters at the soaring thermalling
//! radius. Bank is commanded at `SOAR_THML_BANK` (default 30 deg). Pitch and
//! throttle stay on TECS. Stabilization is enabled via
//! [dispatch_stabilize_from_mode](crate::mode_table_hookup::dispatch_stabilize_from_mode)
//! (Thermal is in the default stabilize arm).

use ap_math::scalar::constrain_int32;

use crate::mode_table::{BuildFeatures, ModeNumber};

/// Upstream `SOAR_THML_BANK` default, degrees.
pub const SOAR_THML_BANK_DEFAULT_DEG: f32 = 30.0;

fn is_thermal_mode(control_mode: u8, features: &BuildFeatures) -> bool {
    ModeNumber::from_number(control_mode, features) == Some(ModeNumber::Thermal)
}

/// Inputs for THERMAL nav demand tick (`ModeThermal::update` roll half).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThermalModeNavInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    /// Upstream `SOAR_THML_BANK`, degrees.
    pub thermal_bank_deg: f32,
    pub roll_limit_cd: i32,
}

/// Result of the THERMAL nav demand tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ThermalModeNavOutput {
    pub nav_roll_cd: i32,
    pub applied: bool,
}

/// Bank at the soaring thermalling angle when THERMAL is active.
///
/// Pitch is not mapped: THERMAL is soaring-assisted, so TECS owns nav pitch.
#[must_use]
pub fn thermal_mode_nav_tick(inp: &ThermalModeNavInputs) -> ThermalModeNavOutput {
    if !is_thermal_mode(inp.control_mode, &inp.features) {
        return ThermalModeNavOutput {
            nav_roll_cd: 0,
            applied: false,
        };
    }

    let bank_cd = (inp.thermal_bank_deg * 100.0) as i32;
    ThermalModeNavOutput {
        nav_roll_cd: constrain_int32(bank_cd, -inp.roll_limit_cd, inp.roll_limit_cd),
        applied: true,
    }
}
