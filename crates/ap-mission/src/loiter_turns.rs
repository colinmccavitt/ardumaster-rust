//! NAV_LOITER_TURNS / loiter-turns command.
//!
//! Upstream `Plane::do_loiter_turns` and `Plane::verify_loiter_turns`
//! (`ArduPlane/commands_logic.cpp`). AUTO sanitizes the command location into
//! `next_WP`, loads `loiter.total_cd` from the packed turn count, then stays
//! on the item until the aircraft has orbited that many times.
//!
//! `cmd.p1` packing (upstream `mavlink_to_mission_cmd` / `get_loiter_turns`):
//! - low byte: integer turns, or 256ths of a turn when `type_specific_bits`
//!   bit 1 is set
//! - high byte: radius in metres, or decametres when bit 0 is set
//!
//! Heading-exit (`verify_loiter_heading`) and the L1 loiter controller come
//! later. This stub treats the primary turns goal as completion.

use ap_math::location::{check_latlng_1e7, Location};

use crate::{MavFrame, MissionCommand};

/// `MAV_CMD_NAV_LOITER_TURNS` — loiter N times at a location.
pub const MAV_CMD_NAV_LOITER_TURNS: u16 = 18;

/// `type_specific_bits` bit 0: stored radius is decametres (`* 10` on read).
pub const LOITER_TURNS_RADIUS_X10_BIT: u8 = 1 << 0;

/// `type_specific_bits` bit 1: stored turns are 256ths (`/ 256` on read).
pub const LOITER_TURNS_FRACTIONAL_BIT: u8 = 1 << 1;

/// Centidegrees in one full orbit, upstream `turns * 36000UL`.
pub const LOITER_TURNS_CD_PER_ORBIT: u32 = 36_000;

/// Inputs for starting a NAV_LOITER_TURNS item, upstream `do_loiter_turns`.
#[derive(Debug, Clone, Copy)]
pub struct DoLoiterTurnsInputs {
    /// Vehicle location this tick, used to fill a zero/invalid command LLA.
    pub current_loc: Location,
    /// Command location, upstream `cmd.content.location`.
    pub cmd_loc: Location,
    /// Packed `cmd.p1`: low byte turns, high byte radius.
    pub cmd_p1: u16,
    /// Upstream `Mission_Command::type_specific_bits` (radius-x10 / fractional).
    pub type_specific_bits: u8,
}

impl Default for DoLoiterTurnsInputs {
    fn default() -> Self {
        Self {
            current_loc: Location::new(0, 0),
            cmd_loc: Location::new(0, 0),
            cmd_p1: 0,
            type_specific_bits: 0,
        }
    }
}

/// Result of starting a NAV_LOITER_TURNS item.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DoLoiterTurnsOutput {
    /// Sanitized loiter centre, upstream `set_next_WP(cmdloc)`.
    pub next_wp: Location,
    /// Loiter direction: `-1` CCW, `+1` CW. Upstream `loiter.direction`.
    pub loiter_direction: i8,
    /// Orbit budget in centidegrees, upstream `loiter.total_cd`.
    pub total_cd: u32,
    /// `1` while the primary turns goal is unmet, upstream `condition_value`.
    pub condition_value: i16,
}

impl Default for DoLoiterTurnsOutput {
    fn default() -> Self {
        Self {
            next_wp: Location::new(0, 0),
            loiter_direction: 1,
            total_cd: 0,
            condition_value: 1,
        }
    }
}

/// Inputs for one NAV_LOITER_TURNS verify tick, upstream `verify_loiter_turns`.
#[derive(Debug, Clone, Copy)]
pub struct VerifyLoiterTurnsInputs {
    /// Packed `cmd.p1`: high byte is the radius passed to `update_loiter`.
    pub cmd_p1: u16,
    /// Upstream `Mission_Command::type_specific_bits`.
    pub type_specific_bits: u8,
    /// Upstream `reached_loiter_target()`.
    pub reached_loiter_target: bool,
    /// Accumulated orbit, upstream `loiter.sum_cd`.
    pub sum_cd: u32,
    /// Orbit budget from [`do_loiter_turns`], upstream `loiter.total_cd`.
    pub total_cd: u32,
    /// `1` until the primary turns goal is met, then `0`.
    pub condition_value: i16,
}

impl Default for VerifyLoiterTurnsInputs {
    fn default() -> Self {
        Self {
            cmd_p1: 0,
            type_specific_bits: 0,
            reached_loiter_target: false,
            sum_cd: 0,
            total_cd: 0,
            condition_value: 1,
        }
    }
}

/// Result of one NAV_LOITER_TURNS verify tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VerifyLoiterTurnsOutput {
    /// True once the commanded orbits are done. Heading-exit is later.
    pub complete: bool,
    /// Radius to feed `update_loiter`, high byte of `p1` (x10 when bit 0).
    pub loiter_radius_m: u16,
    /// Updated `condition_value` (`0` after the primary turns goal).
    pub condition_value: i16,
}

/// Pack turns (low) and radius metres (high) into `cmd.p1`.
#[must_use]
pub const fn pack_loiter_turns_p1(turns: u8, radius_m: u8) -> u16 {
    (turns as u16) | ((radius_m as u16) << 8)
}

/// Orbit budget in centidegrees from packed `p1` / `type_specific_bits`.
///
/// Upstream `get_loiter_turns()` then `loiter.total_cd = turns * 36000UL`.
/// Fractional turns (bit 1) store 256ths in the low byte.
#[must_use]
pub const fn loiter_turns_total_cd(cmd_p1: u16, type_specific_bits: u8) -> u32 {
    let stored = (cmd_p1 & 0x00ff) as u32;
    if type_specific_bits & LOITER_TURNS_FRACTIONAL_BIT != 0 {
        (stored * LOITER_TURNS_CD_PER_ORBIT) / 256
    } else {
        stored * LOITER_TURNS_CD_PER_ORBIT
    }
}

/// Radius in metres from packed `p1` / `type_specific_bits`.
///
/// Upstream `HIGHBYTE(cmd.p1)`, times 10 when bit 0 is set.
#[must_use]
pub const fn loiter_turns_radius_m(cmd_p1: u16, type_specific_bits: u8) -> u16 {
    let mut radius = cmd_p1 >> 8;
    if type_specific_bits & LOITER_TURNS_RADIUS_X10_BIT != 0 {
        radius = radius.saturating_mul(10);
    }
    radius
}

/// A `MAV_CMD_NAV_LOITER_TURNS` item at `seq` with the given frame and LLA.
#[must_use]
pub const fn loiter_turns_cmd(
    seq: u16,
    frame: MavFrame,
    lat: i32,
    lng: i32,
    alt_cm: i32,
) -> MissionCommand {
    MissionCommand {
        seq,
        command: MAV_CMD_NAV_LOITER_TURNS,
        frame,
        location: Location::new_with_alt(lat, lng, alt_cm, frame.to_alt_frame()),
    }
}

/// Whether `command` is `MAV_CMD_NAV_LOITER_TURNS`.
#[must_use]
pub const fn is_nav_loiter_turns(command: u16) -> bool {
    command == MAV_CMD_NAV_LOITER_TURNS
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

/// Start a NAV_LOITER_TURNS item, upstream `do_loiter_turns`.
///
/// Sanitizes the command location, reports direction from `loiter_ccw`, and
/// loads `total_cd` from the packed turn count. `condition_value` starts at 1
/// (primary turns goal not yet met).
#[must_use]
pub fn do_loiter_turns(inp: &DoLoiterTurnsInputs) -> DoLoiterTurnsOutput {
    let next_wp = sanitize_loiter_loc(inp.cmd_loc, inp.current_loc);
    let loiter_direction = if next_wp.loiter_ccw { -1 } else { 1 };
    DoLoiterTurnsOutput {
        next_wp,
        loiter_direction,
        total_cd: loiter_turns_total_cd(inp.cmd_p1, inp.type_specific_bits),
        condition_value: 1,
    }
}

/// True once the aircraft has reached the loiter and flown past `total_cd`.
///
/// Upstream `verify_loiter_turns` then hands off to `verify_loiter_heading`.
/// This stub reports the decoded radius and treats the primary turns goal as
/// the completion check.
#[must_use]
pub fn verify_loiter_turns(inp: &VerifyLoiterTurnsInputs) -> VerifyLoiterTurnsOutput {
    let loiter_radius_m = loiter_turns_radius_m(inp.cmd_p1, inp.type_specific_bits);
    if !inp.reached_loiter_target {
        return VerifyLoiterTurnsOutput {
            complete: false,
            loiter_radius_m,
            condition_value: inp.condition_value,
        };
    }
    if inp.condition_value != 0 {
        let turns_done = inp.sum_cd > inp.total_cd && inp.sum_cd > 1;
        return VerifyLoiterTurnsOutput {
            complete: turns_done,
            loiter_radius_m,
            condition_value: if turns_done { 0 } else { inp.condition_value },
        };
    }
    VerifyLoiterTurnsOutput {
        complete: true,
        loiter_radius_m,
        condition_value: 0,
    }
}
