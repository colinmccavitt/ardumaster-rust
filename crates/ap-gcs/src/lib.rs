//! GCS MAVLink surface, upstream `libraries/GCS_MAVLink`. FW-028.
//!
//! This slice is the wire seam, not the dialect. It frames MAVLink 2 packets,
//! packs and unpacks [`heartbeat::Heartbeat`] (msgid 0), routes that msgid
//! through the `handle_message` stub that upstream uses to notice a GCS,
//! sends [`statustext::StatusText`] (msgid 253) via `send_text`,
//! dispatches [`command::CommandLong`] / [`command::CommandInt`]
//! (msgid 76 / 75) through the Plane command table (ARM/DISARM,
//! DO_SET_MODE, NAV_TAKEOFF), walks a small in-memory table for
//! [`param::ParamRequestList`] / [`param::ParamSet`] (msgid 21 / 23),
//! emitting [`param::ParamValue`] (msgid 22), sends
//! [`pose::Attitude`] / [`pose::GlobalPositionInt`] (msgid 30 / 33) from a
//! [`pose::PoseSnapshot`], uploads / downloads one waypoint through
//! [`mission::MissionItemInt`] / [`mission::MissionRequestInt`]
//! (msgid 73 / 51) against a small in-memory mission table, and sends
//! [`health::SysStatus`] / [`health::BatteryStatus`] (msgid 1 / 147) from a
//! [`health::HealthSnapshot`], sends [`channels::RcChannels`] /
//! [`channels::ServoOutputRaw`] (msgid 65 / 36) from a
//! [`channels::ChannelSnapshot`], and sends [`hud::VfrHud`] /
//! [`hud::NavControllerOutput`] (msgid 74 / 62) from a
//! [`hud::HudSnapshot`], and stores msgid → interval in
//! [`rates::RateTable`] from [`rates::RequestDataStream`] (msgid 66) /
//! `MAV_CMD_SET_MESSAGE_INTERVAL` (511), skipping a stream send when the
//! period has not elapsed, and stores channel overrides from
//! [`rc_override::RcChannelsOverride`] / [`rc_override::ManualControl`]
//! (msgid 70 / 69) in [`rc_override::OverrideStore`]. The rest of
//! `modules/mavlink` stays ungenerated until later slices.
//!
//! # What this slice does not include
//!
//! The full common/ardupilotmega XML dialect, STATUSTEXT receive /
//! chunked queueing, signing, routing, COMMAND_ACK, PARAM_REQUEST_READ,
//! MISSION_COUNT / MISSION_ACK, and the `GCS_MAVLINK_Plane` vehicle-side
//! handlers. Those land in later FW-028 slices. GCS failsafe
//! (`FS_GCS_ENABL`) already lives in `ap-plane` and is not rewritten here.

#![no_std]

pub mod channels;
pub mod command;
pub mod dispatch;
pub mod framing;
pub mod health;
pub mod heartbeat;
pub mod hud;
pub mod mission;
pub mod param;
pub mod pose;
pub mod rates;
pub mod rc_override;
pub mod statustext;

pub use channels::{
    ChannelSnapshot, RcChannels, ServoOutputRaw, MSG_ID_RC_CHANNELS, MSG_ID_SERVO_OUTPUT_RAW,
    RC_CHANNELS_COUNT, RC_CHANNELS_CRC, RC_CHANNELS_LEN, RC_CHANNELS_MIN_LEN, RC_CHANNEL_UNUSED,
    RSSI_UNKNOWN, SERVO_OUTPUT_COUNT, SERVO_OUTPUT_RAW_CRC, SERVO_OUTPUT_RAW_LEN,
    SERVO_OUTPUT_RAW_MIN_LEN,
};
pub use command::{
    classify, CommandInt, CommandLong, CommandVia, PlaneCommand, ARM_DISARM_FORCE, COMMAND_INT_LEN,
    COMMAND_LONG_LEN, MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_DO_SET_MODE, MAV_CMD_NAV_TAKEOFF,
    MAV_FRAME_GLOBAL_RELATIVE_ALT, MSG_ID_COMMAND_INT, MSG_ID_COMMAND_LONG,
};
pub use dispatch::{Dispatch, GcsMavlink};
pub use framing::{decode_v2, encode_v2, DecodeError, Frame};
pub use health::{
    BatteryStatus, HealthSnapshot, SysStatus, BATTERY_STATUS_CRC, BATTERY_STATUS_LEN,
    BATTERY_STATUS_MIN_LEN, BATTERY_TEMPERATURE_UNKNOWN, BATTERY_VOLTAGES_EXT_LEN,
    BATTERY_VOLTAGES_LEN, MAV_BATTERY_FUNCTION_UNKNOWN, MAV_BATTERY_TYPE_UNKNOWN,
    MSG_ID_BATTERY_STATUS, MSG_ID_SYS_STATUS, SYS_STATUS_CRC, SYS_STATUS_LEN, SYS_STATUS_MIN_LEN,
};
pub use heartbeat::{
    Heartbeat, MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_TYPE_FIXED_WING, MSG_ID_HEARTBEAT,
};
pub use hud::{
    HudSnapshot, NavControllerOutput, VfrHud, MSG_ID_NAV_CONTROLLER_OUTPUT, MSG_ID_VFR_HUD,
    NAV_CONTROLLER_OUTPUT_CRC, NAV_CONTROLLER_OUTPUT_LEN, VFR_HUD_CRC, VFR_HUD_LEN,
};
pub use mission::{
    MissionItemInt, MissionRequestInt, MissionTable, MAV_CMD_NAV_WAYPOINT,
    MAV_MISSION_TYPE_MISSION, MAX_MISSION_ITEMS, MISSION_ITEM_INT_CRC, MISSION_ITEM_INT_LEN,
    MISSION_REQUEST_INT_CRC, MISSION_REQUEST_INT_LEN, MSG_ID_MISSION_ITEM_INT,
    MSG_ID_MISSION_REQUEST_INT,
};
pub use param::{
    encode_param_id, param_id_name, ParamEntry, ParamRequestList, ParamSet, ParamTable, ParamValue,
    MAV_PARAM_TYPE_REAL32, MAX_PARAMS, MSG_ID_PARAM_REQUEST_LIST, MSG_ID_PARAM_SET,
    MSG_ID_PARAM_VALUE, PARAM_ID_LEN, PARAM_REQUEST_LIST_CRC, PARAM_REQUEST_LIST_LEN,
    PARAM_SET_CRC, PARAM_SET_LEN, PARAM_VALUE_CRC, PARAM_VALUE_LEN,
};
pub use pose::{
    Attitude, GlobalPositionInt, PoseSnapshot, ATTITUDE_CRC, ATTITUDE_LEN, GLOBAL_POSITION_INT_CRC,
    GLOBAL_POSITION_INT_LEN, MSG_ID_ATTITUDE, MSG_ID_GLOBAL_POSITION_INT,
};
pub use rates::{
    hz_to_interval_ms, interval_us_to_ms, RateTable, RequestDataStream,
    MAV_CMD_SET_MESSAGE_INTERVAL, MAV_DATA_STREAM_ALL, MAV_DATA_STREAM_EXTENDED_STATUS,
    MAV_DATA_STREAM_EXTRA1, MAV_DATA_STREAM_EXTRA2, MAV_DATA_STREAM_EXTRA3,
    MAV_DATA_STREAM_POSITION, MAV_DATA_STREAM_RC_CHANNELS, MAX_INTERVAL_MS, MAX_RATES,
    MSG_ID_REQUEST_DATA_STREAM, REQUEST_DATA_STREAM_CRC, REQUEST_DATA_STREAM_LEN,
};
pub use rc_override::{
    map_manual_axis, ManualControl, OverrideStore, RcChannelsOverride, MANUAL_AXIS_INVALID,
    MANUAL_CONTROL_CRC, MANUAL_CONTROL_LEN, MANUAL_CONTROL_MIN_LEN, MANUAL_RADIO_MAX,
    MANUAL_RADIO_MIN, MSG_ID_MANUAL_CONTROL, MSG_ID_RC_CHANNELS_OVERRIDE, OVERRIDE_CHANNEL_COUNT,
    OVERRIDE_IGNORE, OVERRIDE_RELEASE_EXT, RC_CHANNELS_OVERRIDE_CRC, RC_CHANNELS_OVERRIDE_LEN,
    RC_CHANNELS_OVERRIDE_MIN_LEN,
};
pub use statustext::{
    StatusText, MAV_SEVERITY_DEBUG, MAV_SEVERITY_EMERGENCY, MAV_SEVERITY_ERROR, MAV_SEVERITY_INFO,
    MAV_SEVERITY_WARNING, MSG_ID_STATUSTEXT, STATUSTEXT_LEN, TEXT_LEN,
};
