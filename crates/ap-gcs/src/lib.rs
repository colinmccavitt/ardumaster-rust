//! GCS MAVLink surface, upstream `libraries/GCS_MAVLink`. FW-028.
//!
//! This slice is the wire seam, not the dialect. It frames MAVLink 2 packets,
//! packs and unpacks [`heartbeat::Heartbeat`] (msgid 0), and routes that
//! msgid through the `handle_message` stub that upstream uses to notice a
//! GCS. The rest of `modules/mavlink` stays ungenerated until later slices.
//!
//! # What this slice does not include
//!
//! The full common/ardupilotmega XML dialect, STATUSTEXT, COMMAND_LONG /
//! COMMAND_INT, signing, routing, and the `GCS_MAVLINK_Plane` overrides.
//! Those land in later FW-028 slices. GCS failsafe (`FS_GCS_ENABL`) already
//! lives in `ap-plane` and is not rewritten here.

#![no_std]

pub mod dispatch;
pub mod framing;
pub mod heartbeat;

pub use dispatch::{Dispatch, GcsMavlink};
pub use framing::{decode_v2, encode_v2, DecodeError, Frame};
pub use heartbeat::{
    Heartbeat, MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_TYPE_FIXED_WING, MSG_ID_HEARTBEAT,
};
