//! GCS MAVLink surface, upstream `libraries/GCS_MAVLink`. FW-028.
//!
//! This slice is the wire seam, not the dialect. It frames MAVLink 2 packets,
//! packs and unpacks [`heartbeat::Heartbeat`] (msgid 0), routes that msgid
//! through the `handle_message` stub that upstream uses to notice a GCS,
//! and sends [`statustext::StatusText`] (msgid 253) via `send_text`. The
//! rest of `modules/mavlink` stays ungenerated until later slices.
//!
//! # What this slice does not include
//!
//! The full common/ardupilotmega XML dialect, COMMAND_LONG / COMMAND_INT,
//! STATUSTEXT receive / chunked queueing, signing, routing, and the
//! `GCS_MAVLINK_Plane` overrides. Those land in later FW-028 slices. GCS
//! failsafe (`FS_GCS_ENABL`) already lives in `ap-plane` and is not
//! rewritten here.

#![no_std]

pub mod dispatch;
pub mod framing;
pub mod heartbeat;
pub mod statustext;

pub use dispatch::{Dispatch, GcsMavlink};
pub use framing::{decode_v2, encode_v2, DecodeError, Frame};
pub use heartbeat::{
    Heartbeat, MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_TYPE_FIXED_WING, MSG_ID_HEARTBEAT,
};
pub use statustext::{
    StatusText, MAV_SEVERITY_DEBUG, MAV_SEVERITY_EMERGENCY, MAV_SEVERITY_ERROR, MAV_SEVERITY_INFO,
    MAV_SEVERITY_WARNING, MSG_ID_STATUSTEXT, STATUSTEXT_LEN, TEXT_LEN,
};
