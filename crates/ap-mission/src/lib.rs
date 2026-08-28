//! Port of `libraries/AP_Mission` command/item storage. FW-024.
//!
//! This slice is the in-memory `Mission_Command` record — the MAV_CMD waypoint
//! item the rest of the mission library indexes — and the list that holds it.
//! EEPROM and the start/verify state machine come later.
//!
//! # What is stored
//!
//! Upstream `AP_Mission::Mission_Command` carries `index`, `id`, `p1`, a
//! `Content` union and a few extra bits. A NAV_WAYPOINT uses `Content.location`
//! for lat/lon/alt and the location's altitude-frame bits for the datum.
//! On the wire those same fields are mission-protocol `seq`, `command`,
//! `frame`, `x`/`y`/`z`.
//!
//! The first real command is index 1; index 0 is reserved for home
//! (`AP_MISSION_FIRST_REAL_COMMAND`).

#![no_std]

use ap_math::location::{AltFrame, Location};

mod verify_nav_wp;
pub use verify_nav_wp::{verify_nav_wp, VerifyNavWpInputs, WP_RADIUS_DEFAULT_M};

mod loiter_unlimited;
pub use loiter_unlimited::{
    do_loiter_unlimited, is_nav_loiter_unlim, loiter_unlimited_cmd, verify_loiter_unlim,
    DoLoiterUnlimitedInputs, DoLoiterUnlimitedOutput, VerifyLoiterUnlimInputs,
    VerifyLoiterUnlimOutput, MAV_CMD_NAV_LOITER_UNLIM,
};

mod loiter_turns;
pub use loiter_turns::{
    do_loiter_turns, is_nav_loiter_turns, loiter_turns_cmd, loiter_turns_radius_m,
    loiter_turns_total_cd, pack_loiter_turns_p1, verify_loiter_turns, DoLoiterTurnsInputs,
    DoLoiterTurnsOutput, VerifyLoiterTurnsInputs, VerifyLoiterTurnsOutput,
    LOITER_TURNS_CD_PER_ORBIT, LOITER_TURNS_FRACTIONAL_BIT, LOITER_TURNS_RADIUS_X10_BIT,
    MAV_CMD_NAV_LOITER_TURNS,
};

mod loiter_time;
pub use loiter_time::{
    do_loiter_time, is_nav_loiter_time, loiter_time_cmd, loiter_time_max_ms, verify_loiter_time,
    DoLoiterTimeInputs, DoLoiterTimeOutput, VerifyLoiterTimeInputs, VerifyLoiterTimeOutput,
    MAV_CMD_NAV_LOITER_TIME,
};

mod loiter_to_alt;
pub use loiter_to_alt::{
    do_loiter_to_alt, is_nav_loiter_to_alt, loiter_to_alt_cmd, loiter_to_alt_reached,
    verify_loiter_to_alt, DoLoiterToAltInputs, DoLoiterToAltOutput, VerifyLoiterToAltInputs,
    VerifyLoiterToAltOutput, LOITER_TO_ALT_BAND_CM, MAV_CMD_NAV_LOITER_TO_ALT,
};

mod continue_and_change_alt;
pub use continue_and_change_alt::{
    continue_and_change_alt_cmd, continue_and_change_alt_reached, do_continue_and_change_alt,
    is_nav_continue_and_change_alt, verify_continue_and_change_alt, DoContinueAndChangeAltInputs,
    DoContinueAndChangeAltOutput, VerifyContinueAndChangeAltInputs,
    VerifyContinueAndChangeAltOutput, CHANGE_ALT_CLIMB, CHANGE_ALT_DESCEND, CHANGE_ALT_NEUTRAL,
    CONTINUE_AND_CHANGE_ALT_BAND_CM, CONTINUE_AND_CHANGE_ALT_EXTEND_M,
    CONTINUE_AND_CHANGE_ALT_EXTEND_THRESHOLD_M, CONTINUE_AND_CHANGE_ALT_OFFSET_M, HOLD_COURSE_NONE,
    MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT,
};

mod land;
pub use land::{
    do_land, is_nav_land, land_abort_altitude_cm, land_cmd, land_verify_height_m, verify_land,
    DoLandInputs, DoLandOutput, VerifyLandInputs, VerifyLandOutput, LAND_ABORT_ALT_DEFAULT_CM,
    LAND_ABORT_PITCH_DEFAULT_CD, MAV_CMD_NAV_LAND,
};

mod do_jump;
pub use do_jump::{
    do_jump, do_jump_cmd, is_do_jump, jump_content, jump_should_take, jump_target_valid,
    DoJumpInputs, DoJumpOutput, JumpCommand, JUMP_MAX_LOOPS, JUMP_REPEAT_FOREVER, JUMP_TIMES_MAX,
    MAV_CMD_DO_JUMP,
};

mod do_change_speed;
pub use do_change_speed::{
    do_change_speed, do_change_speed_cmd, is_do_change_speed, speed_content, DoChangeSpeedInputs,
    DoChangeSpeedOutput, SpeedCommand, AIRSPEED_MAX_DEFAULT_MS, AIRSPEED_MIN_DEFAULT_MS,
    CHANGE_SPEED_RESET_MS, MAV_CMD_DO_CHANGE_SPEED, NEW_AIRSPEED_CM_NONE, SPEED_TYPE_AIRSPEED,
    SPEED_TYPE_CLIMB_SPEED, SPEED_TYPE_DESCENT_SPEED, SPEED_TYPE_GROUNDSPEED,
};

mod do_set_home;
pub use do_set_home::{
    do_set_home, do_set_home_cmd, home_content, is_do_set_home, set_home_location_valid,
    set_home_use_current, DoSetHomeInputs, DoSetHomeOutput, HomeCommand, MAV_CMD_DO_SET_HOME,
    SET_HOME_USE_CURRENT,
};

mod do_set_roi;
pub use do_set_roi::{
    do_set_roi, do_set_roi_cmd, is_do_set_roi, roi_content, roi_location_set, DoSetRoiInputs,
    DoSetRoiOutput, RoiCommand, MAV_CMD_DO_SET_ROI, MAV_MOUNT_MODE_GPS_POINT,
    MAV_MOUNT_MODE_NEUTRAL,
};

/// Mavlink cmd id of zero means invalid or missing command.
/// Upstream `AP_MISSION_CMD_ID_NONE`.
pub const CMD_ID_NONE: u16 = 0;

/// Command index of 65535 means invalid or missing command.
/// Upstream `AP_MISSION_CMD_INDEX_NONE`.
pub const CMD_INDEX_NONE: u16 = 65535;

/// Command #0 reserved to hold home position.
/// Upstream `AP_MISSION_FIRST_REAL_COMMAND`.
pub const FIRST_REAL_COMMAND: u16 = 1;

/// `MAV_CMD_NAV_WAYPOINT` — navigate to a waypoint.
pub const MAV_CMD_NAV_WAYPOINT: u16 = 16;

/// In-memory command capacity for this stub. EEPROM-backed size comes later.
pub const MAX_COMMANDS: usize = 16;

/// MAVLink coordinate frame stored on a mission item, upstream `MAV_FRAME`.
///
/// Only the global frames a waypoint item actually uses. The `*_INT` wire
/// aliases collapse to the same variant — they differ only in how GCS
/// encodes the integers, not in the stored datum.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MavFrame {
    /// `MAV_FRAME_GLOBAL` / `MAV_FRAME_GLOBAL_INT`. Altitude AMSL.
    Global = 0,
    /// `MAV_FRAME_GLOBAL_RELATIVE_ALT` / `*_INT`. Altitude above home.
    GlobalRelativeAlt = 3,
    /// `MAV_FRAME_GLOBAL_TERRAIN_ALT` / `*_INT`. Altitude above terrain.
    GlobalTerrainAlt = 10,
}

impl MavFrame {
    /// Decode a raw `MAV_FRAME` byte, including the INT aliases.
    #[must_use]
    pub const fn from_u8(raw: u8) -> Option<Self> {
        match raw {
            0 | 5 => Some(Self::Global),
            3 | 6 => Some(Self::GlobalRelativeAlt),
            10 | 11 => Some(Self::GlobalTerrainAlt),
            _ => None,
        }
    }

    /// The raw `MAV_FRAME` value this variant stores.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Map to `Location::AltFrame`, upstream
    /// `mavlink_coordinate_frame_to_location_alt_frame`.
    #[must_use]
    pub const fn to_alt_frame(self) -> AltFrame {
        match self {
            Self::Global => AltFrame::Absolute,
            Self::GlobalRelativeAlt => AltFrame::AboveHome,
            Self::GlobalTerrainAlt => AltFrame::AboveTerrain,
        }
    }
}

/// One stored mission item, upstream `AP_Mission::Mission_Command`.
///
/// The waypoint fields the vehicle reads first: `seq` (index), `command`
/// (MAV_CMD id), `frame` (MAV_FRAME), and lat/lon/alt via [`Location`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MissionCommand {
    /// Position in the command list, upstream `index`. Mission protocol `seq`.
    pub seq: u16,
    /// MAVLink command id, upstream `id`.
    pub command: u16,
    /// Coordinate frame for lat/lon/alt, upstream `MAV_FRAME` on the item.
    pub frame: MavFrame,
    /// Waypoint location: lat/lon in 1e-7 deg, alt in centimetres.
    pub location: Location,
}

impl MissionCommand {
    /// An empty / invalid command, matching upstream's "none" sentinels.
    #[must_use]
    pub const fn none() -> Self {
        Self {
            seq: CMD_INDEX_NONE,
            command: CMD_ID_NONE,
            frame: MavFrame::Global,
            location: Location::new(0, 0),
        }
    }

    /// A `MAV_CMD_NAV_WAYPOINT` item at `seq` with the given frame and LLA.
    #[must_use]
    pub const fn waypoint(seq: u16, frame: MavFrame, lat: i32, lng: i32, alt_cm: i32) -> Self {
        Self {
            seq,
            command: MAV_CMD_NAV_WAYPOINT,
            frame,
            location: Location::new_with_alt(lat, lng, alt_cm, frame.to_alt_frame()),
        }
    }

    /// Whether this item is `MAV_CMD_NAV_WAYPOINT`.
    #[must_use]
    pub const fn is_nav_waypoint(&self) -> bool {
        self.command == MAV_CMD_NAV_WAYPOINT
    }
}

impl Default for MissionCommand {
    fn default() -> Self {
        Self::none()
    }
}

/// In-memory mission command list, upstream `AP_Mission` storage.
///
/// `write_cmd` / `read_cmd` are the EEPROM-shaped API
/// (`write_cmd_to_storage` / `read_cmd_from_storage`) without the storage
/// manager. `add_cmd` appends, assigning the next `seq`.
#[derive(Debug, Clone)]
pub struct Mission {
    items: [MissionCommand; MAX_COMMANDS],
    /// Number of commands including home at index 0, upstream `num_commands`.
    count: u16,
}

impl Mission {
    /// An empty mission. Home has not been written.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            items: [MissionCommand::none(); MAX_COMMANDS],
            count: 0,
        }
    }

    /// Total commands including offset 0 (home), upstream `num_commands`.
    #[must_use]
    pub const fn num_commands(&self) -> u16 {
        self.count
    }

    /// Write `cmd` at `cmd.seq`, growing the list when writing at `count`.
    ///
    /// Upstream `write_cmd_to_storage`: the index must be in `0..=count` and
    /// inside the stub capacity. Writing at `count` appends.
    pub fn write_cmd(&mut self, cmd: MissionCommand) -> bool {
        let idx = usize::from(cmd.seq);
        if idx >= MAX_COMMANDS || cmd.seq > self.count {
            return false;
        }
        let Some(slot) = self.items.get_mut(idx) else {
            return false;
        };
        *slot = cmd;
        if cmd.seq == self.count {
            self.count = self.count.saturating_add(1);
        }
        true
    }

    /// Read the command at `seq`, upstream `read_cmd_from_storage`.
    #[must_use]
    pub fn read_cmd(&self, seq: u16) -> Option<MissionCommand> {
        if seq >= self.count {
            return None;
        }
        self.items.get(usize::from(seq)).copied()
    }

    /// Append a command, assigning `seq = num_commands()`. Upstream `add_cmd`.
    pub fn add_cmd(&mut self, mut cmd: MissionCommand) -> bool {
        if usize::from(self.count) >= MAX_COMMANDS {
            return false;
        }
        cmd.seq = self.count;
        self.write_cmd(cmd)
    }

    /// Drop every command. Home must be written again.
    pub fn clear(&mut self) {
        self.items = [MissionCommand::none(); MAX_COMMANDS];
        self.count = 0;
    }
}

impl Default for Mission {
    fn default() -> Self {
        Self::new()
    }
}
