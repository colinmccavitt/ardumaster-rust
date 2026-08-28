//! LOITER mode glue for the main vehicle loop.
//!
//! Upstream ModeLoiter::_enter calls do_loiter_at_location() (next WP is
//! current loc; sign of WP_LOITER_RAD sets direction) and
//! loiter_angle_reset(). ModeLoiter::navigate calls update_loiter(0), which
//! uses WP_LOITER_RAD. ENABLE_LOITER_ALT_CONTROL plus stick mixing selects
//! FBWB-style altitude. Stabilization stays on the default arm via
//! [dispatch_stabilize_from_mode](crate::mode_table_hookup::dispatch_stabilize_from_mode).

use crate::mode_table::{BuildFeatures, ModeNumber};

fn is_loiter_mode(control_mode: u8, features: &BuildFeatures) -> bool {
    ModeNumber::from_number(control_mode, features) == Some(ModeNumber::Loiter)
}

/// Inputs for LOITER enter plus navigate (ModeLoiter::_enter and navigate).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoiterModeNavInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    /// True when Mode::enter just ran this tick.
    pub mode_just_entered: bool,
    /// Upstream WP_LOITER_RAD (aparm.loiter_radius), metres. Negative is CCW.
    pub wp_loiter_rad_m: i16,
    /// Upstream Plane::stick_mixing_enabled().
    pub stick_mixing_enabled: bool,
    /// Upstream FlightOptions::ENABLE_LOITER_ALT_CONTROL.
    pub loiter_alt_control: bool,
}

/// Result of the LOITER enter / navigate tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoiterModeNavOutput {
    /// do_loiter_at_location armed the hold this tick.
    pub started: bool,
    /// navigate will call update_loiter(0) this tick.
    pub allow_loiter: bool,
    /// abs(WP_LOITER_RAD); zero still means use WP_LOITER_RAD default.
    pub loiter_radius_m: u16,
    /// WP_LOITER_RAD < 0 selects counterclockwise loiter.
    pub loiter_ccw: bool,
    /// True when WP_LOITER_RAD is non-zero and direction should be applied.
    pub direction_set: bool,
    /// Stick mixing plus ENABLE_LOITER_ALT_CONTROL: FBWB-style altitude.
    pub alt_control: bool,
    pub applied: bool,
}

/// Start the location hold on LOITER entry and allow update_loiter(0),
/// matching ModeLoiter enter and navigate.
#[must_use]
pub fn loiter_mode_nav_tick(inp: &LoiterModeNavInputs) -> LoiterModeNavOutput {
    if !is_loiter_mode(inp.control_mode, &inp.features) {
        return LoiterModeNavOutput {
            started: false,
            allow_loiter: false,
            loiter_radius_m: 0,
            loiter_ccw: false,
            direction_set: false,
            alt_control: false,
            applied: false,
        };
    }

    let loiter_radius_m = inp.wp_loiter_rad_m.unsigned_abs();
    let direction_set = loiter_radius_m > 0;
    let loiter_ccw = direction_set && inp.wp_loiter_rad_m < 0;
    let alt_control = inp.stick_mixing_enabled && inp.loiter_alt_control;

    LoiterModeNavOutput {
        started: inp.mode_just_entered,
        allow_loiter: true,
        loiter_radius_m,
        loiter_ccw,
        direction_set,
        alt_control,
        applied: true,
    }
}
