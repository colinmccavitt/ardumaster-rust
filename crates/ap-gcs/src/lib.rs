//! GCS MAVLink surface, upstream `libraries/GCS_MAVLink`. FW-028.
//!
//! This slice is the wire seam, not the dialect. It frames MAVLink 2 packets,
//! packs and unpacks [`heartbeat::Heartbeat`] (msgid 0), routes that msgid
//! through the `handle_message` stub that upstream uses to notice a GCS,
//! sends [`statustext::StatusText`] (msgid 253) via `send_text`, and
//! dispatches [`command::CommandLong`] / [`command::CommandInt`]
//! (msgid 76 / 75) through the Plane command table (ARM/DISARM,
//! DO_SET_MODE, NAV_TAKEOFF). The rest of `modules/mavlink` stays
//! ungenerated until later slices.
//!
//! # What this slice does not include
//!
//! The full common/ardupilotmega XML dialect, PARAM_* / stream rates,
//! STATUSTEXT receive / chunked queueing, signing, routing, COMMAND_ACK,
//! and the `GCS_MAVLINK_Plane` vehicle-side handlers. Those land in later
//! FW-028 slices. GCS failsafe (`FS_GCS_ENABL`) already lives in
//! `ap-plane` and is not rewritten here.

#![no_std]

pub mod command;
pub mod dispatch;
pub mod framing;
pub mod heartbeat;
pub mod statustext;

pub use command::{
    classify, CommandInt, CommandLong, CommandVia, PlaneCommand, ARM_DISARM_FORCE, COMMAND_INT_LEN,
    COMMAND_LONG_LEN, MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_DO_SET_MODE, MAV_CMD_NAV_TAKEOFF,
    MAV_FRAME_GLOBAL_RELATIVE_ALT, MSG_ID_COMMAND_INT, MSG_ID_COMMAND_LONG,
};
pub use dispatch::{Dispatch, GcsMavlink};
pub use framing::{decode_v2, encode_v2, DecodeError, Frame};
pub use heartbeat::{
    Heartbeat, MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_TYPE_FIXED_WING, MSG_ID_HEARTBEAT,
};
pub use statustext::{
    StatusText, MAV_SEVERITY_DEBUG, MAV_SEVERITY_EMERGENCY, MAV_SEVERITY_ERROR, MAV_SEVERITY_INFO,
    MAV_SEVERITY_WARNING, MSG_ID_STATUSTEXT, STATUSTEXT_LEN, TEXT_LEN,
};
