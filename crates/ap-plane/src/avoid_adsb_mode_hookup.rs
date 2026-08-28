//! AVOID_ADSB mode glue for the main vehicle loop.
//!
//! Upstream ModeAvoidADSB is compiled only when `HAL_ADSB_ENABLED`.
//! `_enter` delegates to ModeGuided::_enter: it clears
//! `guided_throttle_passthru`, resets `active_radius_m` to 0, and calls
//! `set_guided_WP(current_loc)`. ModeAvoidADSB::navigate then calls
//! `update_loiter(0)` so the radius is always WP_LOITER_RAD. Without ADSB,
//! mode number 14 falls through to GUIDED in the mode table and this
//! hookup does not apply. Stabilization stays on the default arm via
//! [dispatch_stabilize_from_mode](crate::mode_table_hookup::dispatch_stabilize_from_mode).

use crate::mode_table::{BuildFeatures, ModeNumber};

fn is_avoid_adsb_mode(control_mode: u8, features: &BuildFeatures) -> bool {
    ModeNumber::from_number(control_mode, features) == Some(ModeNumber::AvoidAdsb)
}

/// Inputs for AVOID_ADSB enter plus navigate (ModeAvoidADSB::_enter and navigate).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AvoidAdsbModeNavInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    /// True when Mode::enter just ran this tick.
    pub mode_just_entered: bool,
    /// Upstream WP_LOITER_RAD (aparm.loiter_radius), metres. Negative is CCW.
    pub wp_loiter_rad_m: i16,
}

/// Result of the AVOID_ADSB enter / navigate tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AvoidAdsbModeNavOutput {
    /// ModeGuided::_enter armed set_guided_WP this tick.
    pub started: bool,
    /// navigate will call update_loiter(0) this tick.
    pub allow_loiter: bool,
    /// Always 0 when applied: navigate calls update_loiter(0).
    pub loiter_radius_m: u16,
    /// WP_LOITER_RAD < 0 selects counterclockwise loiter.
    pub loiter_ccw: bool,
    /// True when WP_LOITER_RAD is non-zero and direction should be applied.
    pub direction_set: bool,
    /// _enter cleared guided_throttle_passthru this tick.
    pub clear_throttle_passthru: bool,
    pub applied: bool,
}

/// Start the current-location hold via ModeGuided::_enter and allow
/// update_loiter(0), matching ModeAvoidADSB enter and navigate.
#[must_use]
pub fn avoid_adsb_mode_nav_tick(inp: &AvoidAdsbModeNavInputs) -> AvoidAdsbModeNavOutput {
    if !is_avoid_adsb_mode(inp.control_mode, &inp.features) {
        return AvoidAdsbModeNavOutput {
            started: false,
            allow_loiter: false,
            loiter_radius_m: 0,
            loiter_ccw: false,
            direction_set: false,
            clear_throttle_passthru: false,
            applied: false,
        };
    }

    let radius = inp.wp_loiter_rad_m.unsigned_abs();
    let direction_set = radius > 0;
    let loiter_ccw = direction_set && inp.wp_loiter_rad_m < 0;

    AvoidAdsbModeNavOutput {
        started: inp.mode_just_entered,
        allow_loiter: true,
        loiter_radius_m: 0,
        loiter_ccw,
        direction_set,
        clear_throttle_passthru: inp.mode_just_entered,
        applied: true,
    }
}
