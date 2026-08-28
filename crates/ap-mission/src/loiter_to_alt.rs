//! NAV_LOITER_TO_ALT / climb-then-loiter-to-alt command.
//!
//! Upstream `Plane::do_loiter_to_alt` and `Plane::verify_loiter_to_alt`
//! (`ArduPlane/commands_logic.cpp`). AUTO sanitizes the command location into
//! `next_WP`, starts circling at `cmd.p1` metres, and waits until the aircraft
//! has reached the item altitude (or is stuck and unable to). Heading-exit
//! (`verify_loiter_heading`) comes later; this stub treats the primary
//! altitude goal as completion.
//!
//! `condition_value` starts at 0 (altitude never reached) and becomes 1 once
//! the altitude goal is met — the inverse of TIME/TURNS, matching upstream.

use ap_math::location::{check_latlng_1e7, Location};

use crate::{MavFrame, MissionCommand};

/// `MAV_CMD_NAV_LOITER_TO_ALT` — loiter at a location until altitude is reached.
pub const MAV_CMD_NAV_LOITER_TO_ALT: u16 = 31;

/// Altitude band (cm) for "reached target alt".
///
/// Upstream `navigation.cpp`: `labs(current_loc.alt - target_altitude.amsl_cm) < 500`.
pub const LOITER_TO_ALT_BAND_CM: i32 = 500;

/// Inputs for starting a NAV_LOITER_TO_ALT item, upstream `do_loiter_to_alt`.
#[derive(Debug, Clone, Copy)]
pub struct DoLoiterToAltInputs {
    /// Vehicle location this tick, used to fill a zero/invalid command LLA.
    pub current_loc: Location,
    /// Command location, upstream `cmd.content.location`.
    pub cmd_loc: Location,
    /// Loiter radius in metres, upstream `cmd.p1` (`update_loiter(cmd.p1)`).
    pub cmd_p1: u16,
}

impl Default for DoLoiterToAltInputs {
    fn default() -> Self {
        Self {
            current_loc: Location::new(0, 0),
            cmd_loc: Location::new(0, 0),
            cmd_p1: 0,
        }
    }
}

/// Result of starting a NAV_LOITER_TO_ALT item.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DoLoiterToAltOutput {
    /// Sanitized loiter centre, upstream `set_next_WP(loc)`.
    pub next_wp: Location,
    /// Loiter direction: `-1` CCW, `+1` CW. Upstream `loiter.direction`.
    pub loiter_direction: i8,
    /// Radius to feed `update_loiter`, upstream `cmd.p1`.
    pub loiter_radius_m: u16,
    /// `0` until the primary altitude goal is met, then `1`.
    pub condition_value: i16,
}

impl Default for DoLoiterToAltOutput {
    fn default() -> Self {
        Self {
            next_wp: Location::new(0, 0),
            loiter_direction: 1,
            loiter_radius_m: 0,
            condition_value: 0,
        }
    }
}

/// Inputs for one NAV_LOITER_TO_ALT verify tick, upstream `verify_loiter_to_alt`.
#[derive(Debug, Clone, Copy)]
pub struct VerifyLoiterToAltInputs {
    /// Command `p1`, the radius in metres passed to `update_loiter`.
    pub cmd_p1: u16,
    /// Vehicle altitude this tick, centimetres in the same frame as `target_alt_cm`.
    pub current_alt_cm: i32,
    /// Loiter target altitude, upstream `next_WP_loc.alt` / `target_altitude.amsl_cm`.
    pub target_alt_cm: i32,
    /// Upstream `reached_loiter_target()`.
    pub reached_loiter_target: bool,
    /// Accumulated orbit, upstream `loiter.sum_cd`.
    pub sum_cd: u32,
    /// Upstream `loiter.unable_to_achieve_target_alt` (stuck after several laps).
    pub unable_to_achieve_target_alt: bool,
    /// `0` until the primary altitude goal is met, then `1`.
    pub condition_value: i16,
}

impl Default for VerifyLoiterToAltInputs {
    fn default() -> Self {
        Self {
            cmd_p1: 0,
            current_alt_cm: 0,
            target_alt_cm: 0,
            reached_loiter_target: false,
            sum_cd: 0,
            unable_to_achieve_target_alt: false,
            condition_value: 0,
        }
    }
}

/// Result of one NAV_LOITER_TO_ALT verify tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VerifyLoiterToAltOutput {
    /// True once the altitude goal is met. Heading-exit is later.
    pub complete: bool,
    /// Radius to feed `update_loiter`, upstream `cmd.p1`.
    pub loiter_radius_m: u16,
    /// Updated `condition_value` (`1` after the primary altitude goal).
    pub condition_value: i16,
}

/// True when the vehicle is at the loiter and inside the 5 m alt band.
#[must_use]
pub fn loiter_to_alt_reached(
    current_alt_cm: i32,
    target_alt_cm: i32,
    reached_loiter_target: bool,
) -> bool {
    if !reached_loiter_target {
        return false;
    }
    (current_alt_cm - target_alt_cm).abs() < LOITER_TO_ALT_BAND_CM
}

/// A `MAV_CMD_NAV_LOITER_TO_ALT` item at `seq` with the given frame and LLA.
#[must_use]
pub const fn loiter_to_alt_cmd(
    seq: u16,
    frame: MavFrame,
    lat: i32,
    lng: i32,
    alt_cm: i32,
) -> MissionCommand {
    MissionCommand {
        seq,
        command: MAV_CMD_NAV_LOITER_TO_ALT,
        frame,
        location: Location::new_with_alt(lat, lng, alt_cm, frame.to_alt_frame()),
    }
}

/// Whether `command` is `MAV_CMD_NAV_LOITER_TO_ALT`.
#[must_use]
pub const fn is_nav_loiter_to_alt(command: u16) -> bool {
    command == MAV_CMD_NAV_LOITER_TO_ALT
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

/// Start a NAV_LOITER_TO_ALT item, upstream `do_loiter_to_alt`.
///
/// Sanitizes the command location, reports direction from `loiter_ccw`, and
/// leaves `condition_value` at 0 (primary altitude goal not yet met).
#[must_use]
pub fn do_loiter_to_alt(inp: &DoLoiterToAltInputs) -> DoLoiterToAltOutput {
    let next_wp = sanitize_loiter_loc(inp.cmd_loc, inp.current_loc);
    let loiter_direction = if next_wp.loiter_ccw { -1 } else { 1 };
    DoLoiterToAltOutput {
        next_wp,
        loiter_direction,
        loiter_radius_m: inp.cmd_p1,
        condition_value: 0,
    }
}

/// True once the aircraft is circling and at (or stuck below) the target alt.
///
/// Upstream `verify_loiter_to_alt` then hands off to `verify_loiter_heading`.
/// This stub reports the commanded radius and treats the primary altitude
/// goal as the completion check.
#[must_use]
pub fn verify_loiter_to_alt(inp: &VerifyLoiterToAltInputs) -> VerifyLoiterToAltOutput {
    let loiter_radius_m = inp.cmd_p1;
    if inp.condition_value != 0 {
        return VerifyLoiterToAltOutput {
            complete: true,
            loiter_radius_m,
            condition_value: inp.condition_value,
        };
    }
    let reached_target_alt = loiter_to_alt_reached(
        inp.current_alt_cm,
        inp.target_alt_cm,
        inp.reached_loiter_target,
    );
    let primary_done = inp.sum_cd > 1 && (reached_target_alt || inp.unable_to_achieve_target_alt);
    VerifyLoiterToAltOutput {
        complete: primary_done,
        loiter_radius_m,
        condition_value: if primary_done { 1 } else { 0 },
    }
}
