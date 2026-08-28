//! NAV_CONTINUE_AND_CHANGE_ALT / continue-on-heading while changing altitude.
//!
//! Upstream `Plane::do_continue_and_change_alt` and
//! `Plane::verify_continue_and_change_alt` (`ArduPlane/commands_logic.cpp`).
//! AUTO keeps flying the current course (mission bearing, GPS ground course,
//! or yaw hold) and waits until the command altitude is reached.
//!
//! `cmd.p1` is the climb/descend hint stored in `condition_value`: `1` climb,
//! `2` descend, anything else "don't care". Completion uses that hint, then
//! a 5 m band (`labs(adjusted_altitude_cm() - next_WP.alt) <= 500`).
//!
//! Heading-hold steering and the L1 waypoint controller come later; this stub
//! reports which path verify would take and how far it would push `next_WP`.

use ap_math::location::{AltContext, AltFrame, Location};
use ap_math::scalar::wrap_360_cd;

use crate::{MavFrame, MissionCommand};

/// `MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT` — continue on heading, change alt.
pub const MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT: u16 = 30;

/// `steer_state.hold_course_cd` sentinel meaning "use waypoint / GPS bearing".
pub const HOLD_COURSE_NONE: i32 = -1;

/// Neutral / don't-care climb hint, `cmd.p1 == 0`.
pub const CHANGE_ALT_NEUTRAL: i16 = 0;

/// Climb until at or above the target, `cmd.p1 == 1`.
pub const CHANGE_ALT_CLIMB: i16 = 1;

/// Descend until at or below the target, `cmd.p1 == 2`.
pub const CHANGE_ALT_DESCEND: i16 = 2;

/// Altitude band (cm) for "don't care" completion.
///
/// Upstream `verify_continue_and_change_alt`:
/// `labs(adjusted_altitude_cm() - next_WP_loc.alt) <= 500`.
pub const CONTINUE_AND_CHANGE_ALT_BAND_CM: i32 = 500;

/// How far GPS / yaw projection pushes `next_WP` at start, metres.
///
/// Upstream `next_WP_loc.offset_bearing(bearing, 1000)`.
pub const CONTINUE_AND_CHANGE_ALT_OFFSET_M: f32 = 1000.0;

/// Verify extends `next_WP` when closer than this, metres.
///
/// Upstream `current_loc.get_distance(next_WP_loc) < 200.0f`.
pub const CONTINUE_AND_CHANGE_ALT_EXTEND_THRESHOLD_M: f32 = 200.0;

/// How far verify pushes `next_WP` down the line, metres.
///
/// Upstream `next_WP_loc.offset_bearing(..., 300.0f)`.
pub const CONTINUE_AND_CHANGE_ALT_EXTEND_M: f32 = 300.0;

/// Inputs for starting a NAV_CONTINUE_AND_CHANGE_ALT item,
/// upstream `do_continue_and_change_alt`.
#[derive(Debug, Clone, Copy)]
pub struct DoContinueAndChangeAltInputs {
    /// Previous waypoint, upstream `prev_WP_loc`.
    pub prev_wp: Location,
    /// Current next waypoint, upstream `next_WP_loc`.
    pub next_wp: Location,
    /// Command location (altitude / frame), upstream `cmd.content.location`.
    pub cmd_loc: Location,
    /// Climb/descend hint, upstream `cmd.p1`.
    pub cmd_p1: u16,
    /// Whether GPS has a 2-D fix, upstream `AP::gps().status() >= GPS_OK_FIX_2D`.
    pub gps_ok: bool,
    /// GPS ground course in degrees, upstream `AP::gps().ground_course()`.
    pub gps_ground_course_deg: f32,
    /// Aircraft yaw in centidegrees, upstream `ahrs.yaw_sensor`.
    pub yaw_cd: i32,
    /// Datums for converting the command altitude to absolute.
    pub alt_ctx: AltContext,
}

impl Default for DoContinueAndChangeAltInputs {
    fn default() -> Self {
        Self {
            prev_wp: Location::new(0, 0),
            next_wp: Location::new(0, 0),
            cmd_loc: Location::new(0, 0),
            cmd_p1: 0,
            gps_ok: false,
            gps_ground_course_deg: 0.0,
            yaw_cd: 0,
            alt_ctx: AltContext::default(),
        }
    }
}

/// Result of starting a NAV_CONTINUE_AND_CHANGE_ALT item.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DoContinueAndChangeAltOutput {
    /// Updated next waypoint (possibly projected 1 km ahead).
    pub next_wp: Location,
    /// Heading-hold course, upstream `steer_state.hold_course_cd`.
    ///
    /// `-1` means fly the waypoint / GPS line; a non-negative value is a
    /// fixed yaw hold used when prev/next coincide and GPS is unavailable.
    pub hold_course_cd: i32,
    /// Climb/descend hint copied from `cmd.p1`, upstream `condition_value`.
    pub condition_value: i16,
}

impl Default for DoContinueAndChangeAltOutput {
    fn default() -> Self {
        Self {
            next_wp: Location::new(0, 0),
            hold_course_cd: HOLD_COURSE_NONE,
            condition_value: CHANGE_ALT_NEUTRAL,
        }
    }
}

/// Inputs for one NAV_CONTINUE_AND_CHANGE_ALT verify tick,
/// upstream `verify_continue_and_change_alt`.
#[derive(Debug, Clone, Copy)]
pub struct VerifyContinueAndChangeAltInputs {
    /// Previous waypoint, upstream `prev_WP_loc`.
    pub prev_wp: Location,
    /// Current next waypoint, upstream `next_WP_loc`.
    pub next_wp: Location,
    /// Vehicle location this tick, used to decide whether to extend `next_WP`.
    pub current_loc: Location,
    /// Heading-hold course from [`do_continue_and_change_alt`].
    pub hold_course_cd: i32,
    /// Climb/descend hint from [`do_continue_and_change_alt`].
    pub condition_value: i16,
    /// Baro / TECS altitude this tick, upstream `adjusted_altitude_cm()`.
    pub current_alt_cm: i32,
}

impl Default for VerifyContinueAndChangeAltInputs {
    fn default() -> Self {
        Self {
            prev_wp: Location::new(0, 0),
            next_wp: Location::new(0, 0),
            current_loc: Location::new(0, 0),
            hold_course_cd: HOLD_COURSE_NONE,
            condition_value: CHANGE_ALT_NEUTRAL,
            current_alt_cm: 0,
        }
    }
}

/// Result of one NAV_CONTINUE_AND_CHANGE_ALT verify tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VerifyContinueAndChangeAltOutput {
    /// True once the altitude goal (hint or 5 m band) is met.
    pub complete: bool,
    /// True when verify would call `update_heading_hold` rather than
    /// `update_waypoint`, upstream `hold_course_cd != -1` with coincident WPs.
    pub heading_hold: bool,
    /// Next waypoint after a possible 300 m extension.
    pub next_wp: Location,
}

/// A `MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT` item at `seq`.
#[must_use]
pub const fn continue_and_change_alt_cmd(
    seq: u16,
    frame: MavFrame,
    lat: i32,
    lng: i32,
    alt_cm: i32,
) -> MissionCommand {
    MissionCommand {
        seq,
        command: MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT,
        frame,
        location: Location::new_with_alt(lat, lng, alt_cm, frame.to_alt_frame()),
    }
}

/// Whether `command` is `MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT`.
#[must_use]
pub const fn is_nav_continue_and_change_alt(command: u16) -> bool {
    command == MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT
}

/// True once climb / descend / 5 m band says the altitude goal is met.
///
/// Mirrors the `if / else if / else if` chain in
/// `Plane::verify_continue_and_change_alt`.
#[must_use]
pub const fn continue_and_change_alt_reached(
    condition_value: i16,
    current_alt_cm: i32,
    target_alt_cm: i32,
) -> bool {
    if condition_value == CHANGE_ALT_CLIMB && current_alt_cm >= target_alt_cm {
        return true;
    }
    if condition_value == CHANGE_ALT_DESCEND && current_alt_cm <= target_alt_cm {
        return true;
    }
    current_alt_cm.abs_diff(target_alt_cm) <= CONTINUE_AND_CHANGE_ALT_BAND_CM as u32
}

fn same_latlon(a: Location, b: Location) -> bool {
    a.lat == b.lat && a.lng == b.lng
}

/// Copy command altitude onto `next_wp`, upstream's terrain vs absolute split.
fn apply_cmd_alt(next_wp: &mut Location, cmd_loc: Location, ctx: &AltContext) {
    if cmd_loc.alt_frame() == AltFrame::AboveTerrain {
        next_wp.set_alt_cm(cmd_loc.alt, AltFrame::AboveTerrain);
        return;
    }
    if let Some(alt_abs_cm) = cmd_loc.get_alt_cm(AltFrame::Absolute, ctx) {
        next_wp.set_alt_cm(alt_abs_cm, AltFrame::Absolute);
    }
}

/// Start a NAV_CONTINUE_AND_CHANGE_ALT item, upstream `do_continue_and_change_alt`.
///
/// Picks a heading method from whether `prev_WP` and `next_WP` differ, then
/// writes the command altitude onto `next_WP`. `reset_offset_altitude` is a
/// later TECS hook.
#[must_use]
pub fn do_continue_and_change_alt(
    inp: &DoContinueAndChangeAltInputs,
) -> DoContinueAndChangeAltOutput {
    let mut next_wp = inp.next_wp;
    let hold_course_cd = if !same_latlon(inp.prev_wp, inp.next_wp) {
        HOLD_COURSE_NONE
    } else if inp.gps_ok {
        next_wp.offset_bearing(
            inp.gps_ground_course_deg.into(),
            CONTINUE_AND_CHANGE_ALT_OFFSET_M.into(),
        );
        HOLD_COURSE_NONE
    } else {
        #[allow(
            clippy::cast_precision_loss,
            reason = "upstream offset_bearing takes ahrs.get_yaw_deg(); yaw_cd/100 is that conversion"
        )]
        let yaw_deg = inp.yaw_cd as f32 * 0.01;
        next_wp.offset_bearing(yaw_deg.into(), CONTINUE_AND_CHANGE_ALT_OFFSET_M.into());
        wrap_360_cd(inp.yaw_cd)
    };
    apply_cmd_alt(&mut next_wp, inp.cmd_loc, &inp.alt_ctx);
    DoContinueAndChangeAltOutput {
        next_wp,
        hold_course_cd,
        condition_value: i16::try_from(inp.cmd_p1).unwrap_or(i16::MAX),
    }
}

/// True once the aircraft has reached the commanded altitude.
///
/// Upstream `verify_continue_and_change_alt` also steers (heading-hold vs
/// waypoint, extending `next_WP` by 300 m when closer than 200 m). This stub
/// reports that choice and the possibly-extended waypoint.
#[must_use]
pub fn verify_continue_and_change_alt(
    inp: &VerifyContinueAndChangeAltInputs,
) -> VerifyContinueAndChangeAltOutput {
    let heading_hold =
        same_latlon(inp.prev_wp, inp.next_wp) && inp.hold_course_cd != HOLD_COURSE_NONE;
    let mut next_wp = inp.next_wp;
    if !heading_hold
        && inp.current_loc.get_distance(next_wp) < CONTINUE_AND_CHANGE_ALT_EXTEND_THRESHOLD_M.into()
    {
        let bearing_cd = inp.prev_wp.get_bearing_to(next_wp);
        #[allow(
            clippy::cast_precision_loss,
            reason = "upstream multiplies get_bearing_to by 0.01f before offset_bearing"
        )]
        let bearing_deg = bearing_cd as f32 * 0.01;
        next_wp.offset_bearing(bearing_deg.into(), CONTINUE_AND_CHANGE_ALT_EXTEND_M.into());
    }
    let complete =
        continue_and_change_alt_reached(inp.condition_value, inp.current_alt_cm, inp.next_wp.alt);
    VerifyContinueAndChangeAltOutput {
        complete,
        heading_hold,
        next_wp,
    }
}
