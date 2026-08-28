//! GCS MAVLink surface, upstream `libraries/GCS_MAVLink`. FW-028.
//!
//! This slice is the wire seam, not the dialect. It frames MAVLink 2 packets,
//! packs and unpacks [`heartbeat::Heartbeat`] (msgid 0), routes that msgid
//! through the `handle_message` stub that upstream uses to notice a GCS,
//! sends [`statustext::StatusText`] (msgid 253) via `send_text`,
//! dispatches [`command::CommandLong`] / [`command::CommandInt`]
//! (msgid 76 / 75) through the Plane command table (ARM/DISARM,
//! DO_SET_MODE, NAV_TAKEOFF), and walks a small in-memory table for
//! [`param::ParamRequestList`] / [`param::ParamSet`] (msgid 21 / 23),
//! emitting [`param::ParamValue`] (msgid 22). The rest of `modules/mavlink`
//! stays ungenerated until later slices.
//!
//! # What this slice does not include
//!
//! The full common/ardupilotmega XML dialect, stream rates, STATUSTEXT
//! receive / chunked queueing, signing, routing, COMMAND_ACK, PARAM_REQUEST_READ,
//! and the `GCS_MAVLINK_Plane` vehicle-side handlers. Those land in later
//! FW-028 slices. GCS failsafe (`FS_GCS_ENABL`) already lives in
//! `ap-plane` and is not rewritten here.

#![no_std]

pub mod command;
pub mod dispatch;
pub mod framing;
pub mod heartbeat;
pub mod param;
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
pub use param::{
    encode_param_id, param_id_name, ParamEntry, ParamRequestList, ParamSet, ParamTable, ParamValue,
    MAV_PARAM_TYPE_REAL32, MAX_PARAMS, MSG_ID_PARAM_REQUEST_LIST, MSG_ID_PARAM_SET,
    MSG_ID_PARAM_VALUE, PARAM_ID_LEN, PARAM_REQUEST_LIST_CRC, PARAM_REQUEST_LIST_LEN,
    PARAM_SET_CRC, PARAM_SET_LEN, PARAM_VALUE_CRC, PARAM_VALUE_LEN,
};
pub use statustext::{
    StatusText, MAV_SEVERITY_DEBUG, MAV_SEVERITY_EMERGENCY, MAV_SEVERITY_ERROR, MAV_SEVERITY_INFO,
    MAV_SEVERITY_WARNING, MSG_ID_STATUSTEXT, STATUSTEXT_LEN, TEXT_LEN,
};
