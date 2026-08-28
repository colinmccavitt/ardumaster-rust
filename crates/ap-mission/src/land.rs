//! NAV_LAND / land-to-waypoint mission command.
//!
//! Upstream `Plane::do_land` and the `MAV_CMD_NAV_LAND` arm of
//! `Plane::verify_command` (`ArduPlane/commands_logic.cpp`). AUTO copies the
//! command location into `next_WP`, configures abort altitude and pitch, and
//! then either verifies an abort climb or a landing approach.
//!
//! The AP_Landing slope / deepstall stage machines are FW-023 / FW-029; this
//! stub reports abort altitude, default pitch, leaving `ABORT_LANDING`, and
//! the height passed after terrain correction. Heading, rangefinder, and
//! `landing.do_land` come later.

use ap_math::location::Location;

use crate::{MavFrame, MissionCommand};

/// `MAV_CMD_NAV_LAND` — land at a waypoint.
pub const MAV_CMD_NAV_LAND: u16 = 21;

/// Default abort / takeoff altitude when `cmd.p1` and the last takeoff are unset.
///
/// Upstream `do_land`: `auto_state.takeoff_altitude_rel_cm = 3000` (30 m).
pub const LAND_ABORT_ALT_DEFAULT_CM: i32 = 3000;

/// Default abort pitch when no takeoff command has set one.
///
/// Upstream `do_land`: `auto_state.takeoff_pitch_cd = 1000` (10 deg).
pub const LAND_ABORT_PITCH_DEFAULT_CD: i32 = 1000;

/// Inputs for starting a NAV_LAND item, upstream `do_land`.
#[derive(Debug, Clone, Copy)]
pub struct DoLandInputs {
    /// Command location, upstream `cmd.content.location`.
    pub cmd_loc: Location,
    /// Abort altitude in metres, upstream `cmd.p1`. Zero means "use last
    /// takeoff, else 30 m".
    pub cmd_p1: u16,
    /// Last takeoff / abort altitude, centimetres above home.
    pub takeoff_altitude_rel_cm: i32,
    /// Last takeoff pitch, centidegrees. `<= 0` defaults to 10 deg.
    pub takeoff_pitch_cd: i32,
    /// True when `flight_stage == ABORT_LANDING` as the item starts.
    pub abort_landing: bool,
}

impl Default for DoLandInputs {
    fn default() -> Self {
        Self {
            cmd_loc: Location::new(0, 0),
            cmd_p1: 0,
            takeoff_altitude_rel_cm: 0,
            takeoff_pitch_cd: 0,
            abort_landing: false,
        }
    }
}

/// Result of starting a NAV_LAND item.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DoLandOutput {
    /// Landing waypoint, upstream `set_next_WP(cmd.content.location)`.
    pub next_wp: Location,
    /// Abort climb altitude, centimetres above home.
    pub takeoff_altitude_rel_cm: i32,
    /// Abort pitch, centidegrees.
    pub takeoff_pitch_cd: i32,
    /// True when do_land should `set_flight_stage(LAND)` to leave a sticky abort.
    pub leave_abort: bool,
}

impl Default for DoLandOutput {
    fn default() -> Self {
        Self {
            next_wp: Location::new(0, 0),
            takeoff_altitude_rel_cm: LAND_ABORT_ALT_DEFAULT_CM,
            takeoff_pitch_cd: LAND_ABORT_PITCH_DEFAULT_CD,
            leave_abort: false,
        }
    }
}

/// Inputs for one NAV_LAND verify tick, upstream `verify_command` `NAV_LAND`.
#[derive(Debug, Clone, Copy)]
pub struct VerifyLandInputs {
    /// True when `flight_stage == ABORT_LANDING`.
    pub abort_landing: bool,
    /// Height above the landing point after rangefinder correction, metres.
    ///
    /// Upstream `plane.get_landing_height(rangefinder_active)`.
    pub height_above_target_m: f32,
    /// Terrain correction subtracted before `landing.verify_land`, metres.
    pub terrain_correction_m: f32,
    /// Result of the later `landing.verify_land` hook (land path only).
    pub landing_complete: bool,
}

impl Default for VerifyLandInputs {
    fn default() -> Self {
        Self {
            abort_landing: false,
            height_above_target_m: 0.0,
            terrain_correction_m: 0.0,
            landing_complete: false,
        }
    }
}

/// Result of one NAV_LAND verify tick.
#[derive(Debug, Clone, Copy)]
pub struct VerifyLandOutput {
    /// True once the landing library says the item is done. Abort never completes.
    pub complete: bool,
    /// True when verify would call `landing.verify_abort_landing`.
    pub abort_path: bool,
    /// Height passed to `landing.verify_land` after terrain correction, metres.
    pub height_m: f32,
}

/// Height passed to `landing.verify_land` after terrain correction is removed.
///
/// Upstream `height -= auto_state.terrain_correction` in `verify_command`.
#[must_use]
pub fn land_verify_height_m(height_above_target_m: f32, terrain_correction_m: f32) -> f32 {
    height_above_target_m - terrain_correction_m
}

/// A `MAV_CMD_NAV_LAND` item at `seq` with the given frame and LLA.
#[must_use]
pub const fn land_cmd(
    seq: u16,
    frame: MavFrame,
    lat: i32,
    lng: i32,
    alt_cm: i32,
) -> MissionCommand {
    MissionCommand {
        seq,
        command: MAV_CMD_NAV_LAND,
        frame,
        location: Location::new_with_alt(lat, lng, alt_cm, frame.to_alt_frame()),
    }
}

/// Whether `command` is `MAV_CMD_NAV_LAND`.
#[must_use]
pub const fn is_nav_land(command: u16) -> bool {
    command == MAV_CMD_NAV_LAND
}

/// Abort / takeoff altitude stored by `do_land`.
///
/// `cmd.p1 > 0` is metres → centimetres; otherwise keep the last takeoff
/// altitude, or 30 m when that has never been set.
#[must_use]
pub const fn land_abort_altitude_cm(cmd_p1: u16, takeoff_altitude_rel_cm: i32) -> i32 {
    if cmd_p1 > 0 {
        return (cmd_p1 as i32).saturating_mul(100);
    }
    if takeoff_altitude_rel_cm <= 0 {
        LAND_ABORT_ALT_DEFAULT_CM
    } else {
        takeoff_altitude_rel_cm
    }
}

/// Start a NAV_LAND item, upstream `do_land`.
///
/// Copies the command location into `next_WP`, sets abort altitude / pitch,
/// and reports whether a sticky `ABORT_LANDING` stage should become `LAND`.
/// `landing.do_land` and rangefinder reset are later hooks.
#[must_use]
pub fn do_land(inp: &DoLandInputs) -> DoLandOutput {
    let takeoff_pitch_cd = if inp.takeoff_pitch_cd <= 0 {
        LAND_ABORT_PITCH_DEFAULT_CD
    } else {
        inp.takeoff_pitch_cd
    };
    DoLandOutput {
        next_wp: inp.cmd_loc,
        takeoff_altitude_rel_cm: land_abort_altitude_cm(inp.cmd_p1, inp.takeoff_altitude_rel_cm),
        takeoff_pitch_cd,
        leave_abort: inp.abort_landing,
    }
}

/// True once the landing library completes the item.
///
/// Upstream abort verify always returns false so the mission index is left
/// alone (`AP_Landing::verify_abort_landing`). The land path returns the
/// later `landing.verify_land` result after subtracting terrain correction.
#[must_use]
pub fn verify_land(inp: &VerifyLandInputs) -> VerifyLandOutput {
    let height_m = land_verify_height_m(inp.height_above_target_m, inp.terrain_correction_m);
    VerifyLandOutput {
        complete: !inp.abort_landing && inp.landing_complete,
        abort_path: inp.abort_landing,
        height_m,
    }
}
