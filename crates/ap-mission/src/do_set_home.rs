//! DO_SET_HOME / current-or-specified-LLA mission command.
//!
//! Upstream `Plane::do_set_home` (`ArduPlane/commands_logic.cpp`).
//! AUTO consumes `MAV_CMD_DO_SET_HOME` in `start_command`; there is no
//! verify — a do-command completes immediately.
//!
//! Wire: `param1` (`cmd.p1`) is 1 to use the current GPS location, or any
//! other value to use the item's lat/lon/alt. The current-location path
//! also requires a 3D GPS fix (`gps.status() >= GPS_OK_FIX_3D`); otherwise
//! the specified LLA is used. Persistent EEPROM write, watchdog refuse,
//! and AHRS altitude-frame conversion come later.

use ap_math::location::{check_latlng_1e7, Location};

use crate::{MavFrame, MissionCommand};

/// `MAV_CMD_DO_SET_HOME` — set home to current location or specified LLA.
pub const MAV_CMD_DO_SET_HOME: u16 = 179;

/// `cmd.p1 == 1` selects the current GPS location when a 3D fix is available.
pub const SET_HOME_USE_CURRENT: u16 = 1;

/// Packed home payload, upstream `cmd.p1` plus `content.location`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HomeCommand {
    /// 1 = current location, 0 = specified LLA. Upstream `cmd.p1`.
    pub p1: u16,
    /// Specified home, upstream `cmd.content.location`.
    pub location: Location,
}

impl HomeCommand {
    /// Pack a home payload from mavlink `param1` and the item LLA.
    #[must_use]
    pub const fn new(p1: u16, location: Location) -> Self {
        Self { p1, location }
    }
}

/// Inputs for one DO_SET_HOME apply, upstream `do_set_home`.
#[derive(Debug, Clone, Copy)]
pub struct DoSetHomeInputs {
    /// `cmd.p1` — 1 means current location.
    pub p1: u16,
    /// Specified LLA, upstream `cmd.content.location`.
    pub specified: Location,
    /// Current GPS location, upstream `gps.location()`.
    pub current: Location,
    /// True when `gps.status() >= GPS_OK_FIX_3D`.
    pub gps_ok_3d: bool,
    /// Current AHRS home, left unchanged when the apply is rejected.
    pub home: Location,
}

impl Default for DoSetHomeInputs {
    fn default() -> Self {
        Self {
            p1: 0,
            specified: Location::new(0, 0),
            current: Location::new(0, 0),
            gps_ok_3d: false,
            home: Location::new(0, 0),
        }
    }
}

/// Result of one DO_SET_HOME apply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DoSetHomeOutput {
    /// True when AHRS `set_home` would accept the chosen location.
    pub applied: bool,
    /// True when the current-GPS path was taken (`p1 == 1` and 3D fix).
    pub used_current: bool,
    /// Home location after this command (unchanged when not applied).
    pub home: Location,
}

impl Default for DoSetHomeOutput {
    fn default() -> Self {
        Self {
            applied: false,
            used_current: false,
            home: Location::new(0, 0),
        }
    }
}

/// A `MAV_CMD_DO_SET_HOME` item at `seq`. LLA lives on [`HomeCommand`].
#[must_use]
pub const fn do_set_home_cmd(seq: u16) -> MissionCommand {
    MissionCommand {
        seq,
        command: MAV_CMD_DO_SET_HOME,
        frame: MavFrame::Global,
        location: Location::new(0, 0),
    }
}

/// Pack a home payload, upstream `p1` plus `content.location`.
#[must_use]
pub const fn home_content(p1: u16, location: Location) -> HomeCommand {
    HomeCommand::new(p1, location)
}

/// Whether `command` is `MAV_CMD_DO_SET_HOME`.
#[must_use]
pub const fn is_do_set_home(command: u16) -> bool {
    command == MAV_CMD_DO_SET_HOME
}

/// True when `p1` selects current location and GPS has a 3D fix.
///
/// Upstream `do_set_home`: `cmd.p1 == 1 && gps.status() >= GPS_OK_FIX_3D`.
#[must_use]
pub const fn set_home_use_current(p1: u16, gps_ok_3d: bool) -> bool {
    p1 == SET_HOME_USE_CURRENT && gps_ok_3d
}

/// True when a location is acceptable to `AP_AHRS::set_home`.
///
/// Upstream rejects `!loc.initialised()` and `!loc.check_latlng()`.
#[must_use]
pub fn set_home_location_valid(loc: &Location) -> bool {
    loc.initialised() && check_latlng_1e7(loc.lat, loc.lng)
}

/// Apply one DO_SET_HOME item, upstream `Plane::do_set_home`.
///
/// `p1 == 1` with a 3D fix writes home from `gps.location()` (the persistent
/// path). Anything else writes the specified LLA. Invalid coordinates are a
/// silent no-op, matching the upstream ignore of `set_home` failure.
#[must_use]
pub fn do_set_home(inp: &DoSetHomeInputs) -> DoSetHomeOutput {
    let used_current = set_home_use_current(inp.p1, inp.gps_ok_3d);
    let candidate = if used_current {
        inp.current
    } else {
        inp.specified
    };
    if !set_home_location_valid(&candidate) {
        return DoSetHomeOutput {
            applied: false,
            used_current,
            home: inp.home,
        };
    }
    DoSetHomeOutput {
        applied: true,
        used_current,
        home: candidate,
    }
}
