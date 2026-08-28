//! DO_CHANGE_SPEED / airspeed-groundspeed-throttle mission command.
//!
//! Upstream `Plane::do_change_speed` (`ArduPlane/commands_logic.cpp`).
//! AUTO / GUIDED consume `MAV_CMD_DO_CHANGE_SPEED` in `start_command`;
//! there is no verify — a do-command completes immediately.
//!
//! Wire: `param1` is `SPEED_TYPE` (0 airspeed, 1 groundspeed), `param2` is
//! the target in m/s (`-2` restores default airspeed), `param3` is throttle
//! percent. Out-of-range airspeed falls through to the throttle arm.
//! Climb/descent speed types are accepted on the wire but ignored; they
//! also fall through to throttle. GCS text and param persistence come later.

use ap_math::location::Location;
use ap_math::scalar::is_equal;

use crate::{MavFrame, MissionCommand};

/// `MAV_CMD_DO_CHANGE_SPEED` — change airspeed, groundspeed, or throttle.
pub const MAV_CMD_DO_CHANGE_SPEED: u16 = 178;

/// `SPEED_TYPE_AIRSPEED` — set `new_airspeed_cm` for AUTO / GUIDED.
pub const SPEED_TYPE_AIRSPEED: u8 = 0;

/// `SPEED_TYPE_GROUNDSPEED` — set `aparm.min_groundspeed`.
pub const SPEED_TYPE_GROUNDSPEED: u8 = 1;

/// `SPEED_TYPE_CLIMB_SPEED` — ignored on Plane; fall through to throttle.
pub const SPEED_TYPE_CLIMB_SPEED: u8 = 2;

/// `SPEED_TYPE_DESCENT_SPEED` — ignored on Plane; fall through to throttle.
pub const SPEED_TYPE_DESCENT_SPEED: u8 = 3;

/// Target of `-2` m/s restores the cruise airspeed parameter.
///
/// Upstream `is_equal(speed_target_ms, -2.0f)` then `new_airspeed_cm = -1`.
pub const CHANGE_SPEED_RESET_MS: f32 = -2.0;

/// Scratch sentinel meaning "no DO_CHANGE_SPEED airspeed is active".
///
/// Upstream `new_airspeed_cm = -1` on mode entry and on the `-2` reset.
pub const NEW_AIRSPEED_CM_NONE: i32 = -1;

/// Default `AIRSPEED_MIN` / `aparm.airspeed_min`, metres per second.
pub const AIRSPEED_MIN_DEFAULT_MS: f32 = 9.0;

/// Default `AIRSPEED_MAX` / `aparm.airspeed_max`, metres per second.
pub const AIRSPEED_MAX_DEFAULT_MS: f32 = 22.0;

/// Packed speed payload, upstream `AP_Mission::Speed_Command`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpeedCommand {
    /// `SPEED_TYPE`, upstream `speed.speed_type`.
    pub speed_type: u8,
    /// Target speed, metres per second. Upstream `speed.target_ms`.
    pub target_ms: f32,
    /// Throttle percent. Upstream `speed.throttle_pct`.
    pub throttle_pct: f32,
}

impl SpeedCommand {
    /// Pack a speed payload from mavlink `param1` / `param2` / `param3`.
    #[must_use]
    pub const fn new(speed_type: u8, target_ms: f32, throttle_pct: f32) -> Self {
        Self {
            speed_type,
            target_ms,
            throttle_pct,
        }
    }
}

/// Inputs for one DO_CHANGE_SPEED apply, upstream `do_change_speed`.
#[derive(Debug, Clone, Copy)]
pub struct DoChangeSpeedInputs {
    /// Speed type, upstream `cmd.content.speed.speed_type`.
    pub speed_type: u8,
    /// Target speed m/s, upstream `cmd.content.speed.target_ms`.
    pub target_ms: f32,
    /// Throttle percent, upstream `cmd.content.speed.throttle_pct`.
    pub throttle_pct: f32,
    /// `aparm.airspeed_min`, metres per second.
    pub airspeed_min_ms: f32,
    /// `aparm.airspeed_max`, metres per second.
    pub airspeed_max_ms: f32,
    /// Current AUTO / GUIDED airspeed scratch, centimetres per second.
    pub new_airspeed_cm: i32,
    /// Current `aparm.min_groundspeed`, metres per second.
    pub min_groundspeed_ms: f32,
    /// Current `aparm.throttle_cruise`, percent.
    pub throttle_cruise: f32,
}

impl Default for DoChangeSpeedInputs {
    fn default() -> Self {
        Self {
            speed_type: SPEED_TYPE_AIRSPEED,
            target_ms: 0.0,
            throttle_pct: 0.0,
            airspeed_min_ms: AIRSPEED_MIN_DEFAULT_MS,
            airspeed_max_ms: AIRSPEED_MAX_DEFAULT_MS,
            new_airspeed_cm: NEW_AIRSPEED_CM_NONE,
            min_groundspeed_ms: 0.0,
            throttle_cruise: 45.0,
        }
    }
}

/// Result of one DO_CHANGE_SPEED apply.
#[derive(Debug, Clone, Copy)]
pub struct DoChangeSpeedOutput {
    /// True when at least one target was accepted.
    pub applied: bool,
    /// True when `new_airspeed_cm` was written (set or reset).
    pub set_airspeed: bool,
    /// AUTO / GUIDED airspeed scratch after this command, cm/s.
    pub new_airspeed_cm: i32,
    /// True when `min_groundspeed` was written.
    pub set_groundspeed: bool,
    /// `aparm.min_groundspeed` after this command, m/s.
    pub min_groundspeed_ms: f32,
    /// True when `throttle_cruise` was written.
    pub set_throttle: bool,
    /// `aparm.throttle_cruise` after this command, percent.
    pub throttle_cruise: f32,
}

impl Default for DoChangeSpeedOutput {
    fn default() -> Self {
        Self {
            applied: false,
            set_airspeed: false,
            new_airspeed_cm: NEW_AIRSPEED_CM_NONE,
            set_groundspeed: false,
            min_groundspeed_ms: 0.0,
            set_throttle: false,
            throttle_cruise: 45.0,
        }
    }
}

/// A `MAV_CMD_DO_CHANGE_SPEED` item at `seq`.
#[must_use]
pub const fn do_change_speed_cmd(seq: u16) -> MissionCommand {
    MissionCommand {
        seq,
        command: MAV_CMD_DO_CHANGE_SPEED,
        frame: MavFrame::Global,
        location: Location::new(0, 0),
    }
}

/// Pack a speed payload, upstream `Speed_Command` from mavlink params.
#[must_use]
pub const fn speed_content(speed_type: u8, target_ms: f32, throttle_pct: f32) -> SpeedCommand {
    SpeedCommand::new(speed_type, target_ms, throttle_pct)
}

/// Whether `command` is `MAV_CMD_DO_CHANGE_SPEED`.
#[must_use]
pub const fn is_do_change_speed(command: u16) -> bool {
    command == MAV_CMD_DO_CHANGE_SPEED
}

/// Apply one DO_CHANGE_SPEED item, upstream `Plane::do_change_speed`.
///
/// Airspeed in `[min, max]` writes `new_airspeed_cm = target * 100`.
/// Target `-2` restores the default (`new_airspeed_cm = -1`). Out-of-range
/// airspeed and the unused climb/descent types fall through to throttle:
/// `throttle_pct` in `(0, 100]` writes `throttle_cruise`. Groundspeed always
/// writes `min_groundspeed` and does not touch throttle.
#[must_use]
pub fn do_change_speed(inp: &DoChangeSpeedInputs) -> DoChangeSpeedOutput {
    let mut out = DoChangeSpeedOutput {
        applied: false,
        set_airspeed: false,
        new_airspeed_cm: inp.new_airspeed_cm,
        set_groundspeed: false,
        min_groundspeed_ms: inp.min_groundspeed_ms,
        set_throttle: false,
        throttle_cruise: inp.throttle_cruise,
    };

    match inp.speed_type {
        SPEED_TYPE_AIRSPEED => {
            if is_equal(inp.target_ms, CHANGE_SPEED_RESET_MS) {
                out.applied = true;
                out.set_airspeed = true;
                out.new_airspeed_cm = NEW_AIRSPEED_CM_NONE;
                return out;
            }
            if inp.target_ms >= inp.airspeed_min_ms && inp.target_ms <= inp.airspeed_max_ms {
                out.applied = true;
                out.set_airspeed = true;
                out.new_airspeed_cm = (inp.target_ms * 100.0) as i32;
                return out;
            }
        }
        SPEED_TYPE_GROUNDSPEED => {
            out.applied = true;
            out.set_groundspeed = true;
            out.min_groundspeed_ms = inp.target_ms;
            return out;
        }
        SPEED_TYPE_CLIMB_SPEED | SPEED_TYPE_DESCENT_SPEED => {}
        _ => {}
    }

    if inp.throttle_pct > 0.0 && inp.throttle_pct <= 100.0 {
        out.applied = true;
        out.set_throttle = true;
        out.throttle_cruise = inp.throttle_pct;
        return out;
    }

    out
}
