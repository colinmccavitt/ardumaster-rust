//! DO_SET_ROI / point-camera-at-location mission command.
//!
//! Upstream `Plane::start_command` ROI arm (`ArduPlane/commands_logic.cpp`),
//! the Plane equivalent of Copter `do_roi`. AUTO consumes
//! `MAV_CMD_DO_SET_ROI` in `start_command`; there is no verify — a
//! do-command completes immediately.
//!
//! Wire: the item LLA is `cmd.content.location`. An initialised location
//! becomes the camera-mount ROI target (`set_roi_target`, mount mode
//! `MAV_MOUNT_MODE_GPS_POINT`). An uninitialised location (0,0) clears
//! GPS-point tracking back to the mount default. Mount hardware, RC lock,
//! and `MAV_CMD_DO_SET_ROI_LOCATION` / `_NONE` aliases come later.

use ap_math::location::Location;

use crate::{MavFrame, MissionCommand};

/// `MAV_CMD_DO_SET_ROI` — point the camera / ROI at a location.
pub const MAV_CMD_DO_SET_ROI: u16 = 201;

/// Mount GPS-point tracking, upstream `MAV_MOUNT_MODE_GPS_POINT`.
pub const MAV_MOUNT_MODE_GPS_POINT: u8 = 4;

/// Mount neutral / stow, a typical `MNTx_DEFLT_MODE` restore target.
pub const MAV_MOUNT_MODE_NEUTRAL: u8 = 1;

/// Packed ROI payload, upstream `cmd.content.location`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RoiCommand {
    /// ROI location, upstream `cmd.content.location`.
    pub location: Location,
}

impl RoiCommand {
    /// Pack an ROI payload from the item LLA.
    #[must_use]
    pub const fn new(location: Location) -> Self {
        Self { location }
    }
}

/// Inputs for one DO_SET_ROI apply, upstream ROI arm of `start_command`.
#[derive(Debug, Clone, Copy)]
pub struct DoSetRoiInputs {
    /// Item LLA, upstream `cmd.content.location`.
    pub location: Location,
    /// Current mount mode, upstream `camera_mount.get_mode()`.
    pub mount_mode: u8,
    /// Mode restored when GPS-point tracking is cleared.
    ///
    /// Upstream `camera_mount.set_mode_to_default()`.
    pub default_mode: u8,
    /// Current ROI target, left unchanged when the item LLA is unset.
    pub roi: Location,
}

impl Default for DoSetRoiInputs {
    fn default() -> Self {
        Self {
            location: Location::new(0, 0),
            mount_mode: MAV_MOUNT_MODE_NEUTRAL,
            default_mode: MAV_MOUNT_MODE_NEUTRAL,
            roi: Location::new(0, 0),
        }
    }
}

/// Result of one DO_SET_ROI apply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DoSetRoiOutput {
    /// True when `set_roi_target` ran (item LLA is initialised).
    pub applied: bool,
    /// True when GPS-point tracking was switched off.
    pub cleared: bool,
    /// ROI target after this command (unchanged when the item LLA is unset).
    pub roi: Location,
    /// Mount mode after this command.
    pub mount_mode: u8,
}

impl Default for DoSetRoiOutput {
    fn default() -> Self {
        Self {
            applied: false,
            cleared: false,
            roi: Location::new(0, 0),
            mount_mode: MAV_MOUNT_MODE_NEUTRAL,
        }
    }
}

/// A `MAV_CMD_DO_SET_ROI` item at `seq`. LLA lives on [`RoiCommand`].
#[must_use]
pub const fn do_set_roi_cmd(seq: u16) -> MissionCommand {
    MissionCommand {
        seq,
        command: MAV_CMD_DO_SET_ROI,
        frame: MavFrame::Global,
        location: Location::new(0, 0),
    }
}

/// Pack an ROI payload, upstream `content.location`.
#[must_use]
pub const fn roi_content(location: Location) -> RoiCommand {
    RoiCommand::new(location)
}

/// Whether `command` is `MAV_CMD_DO_SET_ROI`.
#[must_use]
pub const fn is_do_set_roi(command: u16) -> bool {
    command == MAV_CMD_DO_SET_ROI
}

/// True when the item LLA is a real ROI target, upstream `loc.initialised()`.
#[must_use]
pub const fn roi_location_set(loc: &Location) -> bool {
    loc.initialised()
}

/// Apply one DO_SET_ROI item, upstream ROI arm of `Plane::start_command`.
///
/// An initialised LLA becomes the mount ROI and switches the mount to
/// `MAV_MOUNT_MODE_GPS_POINT`. An uninitialised LLA only clears tracking
/// when the mount is already in GPS-point mode; otherwise it is a no-op.
#[must_use]
pub fn do_set_roi(inp: &DoSetRoiInputs) -> DoSetRoiOutput {
    if roi_location_set(&inp.location) {
        return DoSetRoiOutput {
            applied: true,
            cleared: false,
            roi: inp.location,
            mount_mode: MAV_MOUNT_MODE_GPS_POINT,
        };
    }
    if inp.mount_mode == MAV_MOUNT_MODE_GPS_POINT {
        return DoSetRoiOutput {
            applied: false,
            cleared: true,
            roi: inp.roi,
            mount_mode: inp.default_mode,
        };
    }
    DoSetRoiOutput {
        applied: false,
        cleared: false,
        roi: inp.roi,
        mount_mode: inp.mount_mode,
    }
}
