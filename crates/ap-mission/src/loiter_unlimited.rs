//! NAV_LOITER_UNLIM / loiter-unlimited command.
//!
//! Upstream `Plane::do_loiter_unlimited` and `Plane::verify_loiter_unlim`
//! (`ArduPlane/commands_logic.cpp`). AUTO starts this item by sanitizing the
//! command location into `next_WP`, then stays on it forever: verify always
//! returns false and keeps calling `update_loiter(cmd.p1)`.
//!
//! EEPROM, CONDITION_DELAY breakout, and the L1 loiter controller come later.

use ap_math::location::{check_latlng_1e7, Location};

use crate::{MavFrame, MissionCommand};

/// `MAV_CMD_NAV_LOITER_UNLIM` — loiter indefinitely at a location.
pub const MAV_CMD_NAV_LOITER_UNLIM: u16 = 17;

/// Inputs for starting a NAV_LOITER_UNLIM item, upstream `do_loiter_unlimited`.
#[derive(Debug, Clone, Copy)]
pub struct DoLoiterUnlimitedInputs {
    /// Vehicle location this tick, used to fill a zero/invalid command LLA.
    pub current_loc: Location,
    /// Command location, upstream `cmd.content.location`.
    pub cmd_loc: Location,
}

impl Default for DoLoiterUnlimitedInputs {
    fn default() -> Self {
        Self {
            current_loc: Location::new(0, 0),
            cmd_loc: Location::new(0, 0),
        }
    }
}

/// Result of starting a NAV_LOITER_UNLIM item.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DoLoiterUnlimitedOutput {
    /// Sanitized loiter centre, upstream `set_next_WP(cmdloc)`.
    pub next_wp: Location,
    /// Loiter direction: `-1` CCW, `+1` CW. Upstream `loiter.direction`.
    pub loiter_direction: i8,
}

impl Default for DoLoiterUnlimitedOutput {
    fn default() -> Self {
        Self {
            next_wp: Location::new(0, 0),
            loiter_direction: 1,
        }
    }
}

/// Inputs for one NAV_LOITER_UNLIM verify tick, upstream `verify_loiter_unlim`.
#[derive(Debug, Clone, Copy)]
pub struct VerifyLoiterUnlimInputs {
    /// Command `p1`, the radius in metres passed to `update_loiter`. Zero
    /// means the navigation layer should use `WP_LOITER_RAD`.
    pub cmd_p1: u16,
}

impl Default for VerifyLoiterUnlimInputs {
    fn default() -> Self {
        Self { cmd_p1: 0 }
    }
}

/// Result of one NAV_LOITER_UNLIM verify tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VerifyLoiterUnlimOutput {
    /// Always false: unlimited loiter never completes the mission item.
    pub complete: bool,
    /// Radius to feed `update_loiter`, upstream `cmd.p1`.
    pub loiter_radius_m: u16,
}

/// A `MAV_CMD_NAV_LOITER_UNLIM` item at `seq` with the given frame and LLA.
#[must_use]
pub const fn loiter_unlimited_cmd(
    seq: u16,
    frame: MavFrame,
    lat: i32,
    lng: i32,
    alt_cm: i32,
) -> MissionCommand {
    MissionCommand {
        seq,
        command: MAV_CMD_NAV_LOITER_UNLIM,
        frame,
        location: Location::new_with_alt(lat, lng, alt_cm, frame.to_alt_frame()),
    }
}

/// Whether `command` is `MAV_CMD_NAV_LOITER_UNLIM`.
#[must_use]
pub const fn is_nav_loiter_unlim(command: u16) -> bool {
    command == MAV_CMD_NAV_LOITER_UNLIM
}

/// The lat/lng half of upstream `Location::sanitize` used when starting a
/// loiter: zero or out-of-range coordinates mean "here".
fn sanitize_loiter_loc(cmd_loc: Location, current_loc: Location) -> Location {
    let mut next_wp = cmd_loc;
    let missing = next_wp.lat == 0 && next_wp.lng == 0;
    let invalid = !check_latlng_1e7(next_wp.lat, next_wp.lng);
    if missing || invalid {
        next_wp.lat = current_loc.lat;
        next_wp.lng = current_loc.lng;
    }
    next_wp
}

/// Start a NAV_LOITER_UNLIM item, upstream `do_loiter_unlimited`.
///
/// Sanitizes the command location against `current_loc`, then reports the
/// next waypoint and the loiter direction from `cmd_loc.loiter_ccw`.
#[must_use]
pub fn do_loiter_unlimited(inp: &DoLoiterUnlimitedInputs) -> DoLoiterUnlimitedOutput {
    let next_wp = sanitize_loiter_loc(inp.cmd_loc, inp.current_loc);
    let loiter_direction = if next_wp.loiter_ccw { -1 } else { 1 };
    DoLoiterUnlimitedOutput {
        next_wp,
        loiter_direction,
    }
}

/// True never: NAV_LOITER_UNLIM does not advance the mission.
///
/// Upstream `verify_loiter_unlim` calls `update_loiter(cmd.p1)` and returns
/// false. This stub reports that radius so the navigation hookup can call
/// `update_loiter` with the same argument.
#[must_use]
pub fn verify_loiter_unlim(inp: &VerifyLoiterUnlimInputs) -> VerifyLoiterUnlimOutput {
    VerifyLoiterUnlimOutput {
        complete: false,
        loiter_radius_m: inp.cmd_p1,
    }
}
