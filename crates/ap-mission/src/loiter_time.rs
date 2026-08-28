//! NAV_LOITER_TIME / timed-loiter command.
//!
//! Upstream `Plane::do_loiter_time` and `Plane::verify_loiter_time`
//! (`ArduPlane/commands_logic.cpp`). AUTO sanitizes the command location into
//! `next_WP`, loads `loiter.time_max_ms` from `cmd.p1` seconds, then starts the
//! hold timer only after the aircraft has reached the loiter.
//!
//! `cmd.p1` is hold time in whole seconds (`time_max_ms = p1 * 1000`). Verify
//! always calls `update_loiter(0)` so the navigation layer uses
//! `WP_LOITER_RAD`. Heading-exit (`verify_loiter_heading`) comes later; this
//! stub treats the primary time goal as completion.

use ap_math::location::{check_latlng_1e7, Location};

use crate::{MavFrame, MissionCommand};

/// `MAV_CMD_NAV_LOITER_TIME` — loiter at a location for a time.
pub const MAV_CMD_NAV_LOITER_TIME: u16 = 19;

/// Inputs for starting a NAV_LOITER_TIME item, upstream `do_loiter_time`.
#[derive(Debug, Clone, Copy)]
pub struct DoLoiterTimeInputs {
    /// Vehicle location this tick, used to fill a zero/invalid command LLA.
    pub current_loc: Location,
    /// Command location, upstream `cmd.content.location`.
    pub cmd_loc: Location,
    /// Hold time in seconds, upstream `cmd.p1`.
    pub cmd_p1: u16,
}

impl Default for DoLoiterTimeInputs {
    fn default() -> Self {
        Self {
            current_loc: Location::new(0, 0),
            cmd_loc: Location::new(0, 0),
            cmd_p1: 0,
        }
    }
}

/// Result of starting a NAV_LOITER_TIME item.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DoLoiterTimeOutput {
    /// Sanitized loiter centre, upstream `set_next_WP(cmdloc)`.
    pub next_wp: Location,
    /// Loiter direction: `-1` CCW, `+1` CW. Upstream `loiter.direction`.
    pub loiter_direction: i8,
    /// Hold budget in milliseconds, upstream `loiter.time_max_ms`.
    pub time_max_ms: u32,
    /// `1` while the primary time goal is unmet, upstream `condition_value`.
    pub condition_value: i16,
}

impl Default for DoLoiterTimeOutput {
    fn default() -> Self {
        Self {
            next_wp: Location::new(0, 0),
            loiter_direction: 1,
            time_max_ms: 0,
            condition_value: 1,
        }
    }
}

/// Inputs for one NAV_LOITER_TIME verify tick, upstream `verify_loiter_time`.
#[derive(Debug, Clone, Copy)]
pub struct VerifyLoiterTimeInputs {
    /// Current time, upstream `millis()`.
    pub now_ms: u32,
    /// Zero until the loiter is reached, then the start tick.
    pub start_time_ms: u32,
    /// Hold budget from [`do_loiter_time`], upstream `loiter.time_max_ms`.
    pub time_max_ms: u32,
    /// Upstream `reached_loiter_target()`.
    pub reached_loiter_target: bool,
    /// Accumulated orbit, upstream `loiter.sum_cd`.
    pub sum_cd: u32,
    /// `1` until the primary time goal is met, then `0`.
    pub condition_value: i16,
}

impl Default for VerifyLoiterTimeInputs {
    fn default() -> Self {
        Self {
            now_ms: 0,
            start_time_ms: 0,
            time_max_ms: 0,
            reached_loiter_target: false,
            sum_cd: 0,
            condition_value: 1,
        }
    }
}

/// Result of one NAV_LOITER_TIME verify tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VerifyLoiterTimeOutput {
    /// True once the commanded hold time has elapsed. Heading-exit is later.
    pub complete: bool,
    /// Always `0`: verify feeds `update_loiter(0)` / `WP_LOITER_RAD`.
    pub loiter_radius_m: u16,
    /// Updated `condition_value` (`0` after the primary time goal).
    pub condition_value: i16,
    /// Updated start tick; still `0` until the loiter is reached.
    pub start_time_ms: u32,
}

/// Hold budget in milliseconds from `cmd.p1` seconds.
///
/// Upstream `loiter.time_max_ms = cmd.p1 * (uint32_t)1000`.
#[must_use]
pub const fn loiter_time_max_ms(cmd_p1: u16) -> u32 {
    (cmd_p1 as u32).saturating_mul(1000)
}

/// A `MAV_CMD_NAV_LOITER_TIME` item at `seq` with the given frame and LLA.
#[must_use]
pub const fn loiter_time_cmd(
    seq: u16,
    frame: MavFrame,
    lat: i32,
    lng: i32,
    alt_cm: i32,
) -> MissionCommand {
    MissionCommand {
        seq,
        command: MAV_CMD_NAV_LOITER_TIME,
        frame,
        location: Location::new_with_alt(lat, lng, alt_cm, frame.to_alt_frame()),
    }
}

/// Whether `command` is `MAV_CMD_NAV_LOITER_TIME`.
#[must_use]
pub const fn is_nav_loiter_time(command: u16) -> bool {
    command == MAV_CMD_NAV_LOITER_TIME
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

/// Start a NAV_LOITER_TIME item, upstream `do_loiter_time`.
///
/// Sanitizes the command location, reports direction from `loiter_ccw`, and
/// loads `time_max_ms` from `p1` seconds. `condition_value` starts at 1
/// (primary time goal not yet met). The hold timer is not started here.
#[must_use]
pub fn do_loiter_time(inp: &DoLoiterTimeInputs) -> DoLoiterTimeOutput {
    let next_wp = sanitize_loiter_loc(inp.cmd_loc, inp.current_loc);
    let loiter_direction = if next_wp.loiter_ccw { -1 } else { 1 };
    DoLoiterTimeOutput {
        next_wp,
        loiter_direction,
        time_max_ms: loiter_time_max_ms(inp.cmd_p1),
        condition_value: 1,
    }
}

/// True once the aircraft has reached the loiter and held for `time_max_ms`.
///
/// Upstream `verify_loiter_time` then hands off to `verify_loiter_heading`.
/// This stub always reports radius `0` (`update_loiter(0)`) and treats the
/// primary time goal as the completion check.
#[must_use]
pub fn verify_loiter_time(inp: &VerifyLoiterTimeInputs) -> VerifyLoiterTimeOutput {
    let mut start_time_ms = inp.start_time_ms;
    if start_time_ms == 0 {
        if inp.reached_loiter_target && inp.sum_cd > 1 {
            start_time_ms = inp.now_ms;
        }
        return VerifyLoiterTimeOutput {
            complete: false,
            loiter_radius_m: 0,
            condition_value: inp.condition_value,
            start_time_ms,
        };
    }
    if inp.condition_value != 0 {
        let elapsed = inp.now_ms.wrapping_sub(start_time_ms);
        let time_done = elapsed > inp.time_max_ms;
        return VerifyLoiterTimeOutput {
            complete: time_done,
            loiter_radius_m: 0,
            condition_value: if time_done { 0 } else { inp.condition_value },
            start_time_ms,
        };
    }
    VerifyLoiterTimeOutput {
        complete: true,
        loiter_radius_m: 0,
        condition_value: 0,
        start_time_ms,
    }
}
