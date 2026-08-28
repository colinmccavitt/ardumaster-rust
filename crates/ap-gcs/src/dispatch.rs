//! Msgid dispatch stub, upstream `GCS_MAVLINK::handle_message`.
//!
//! HEARTBEAT is wired on receive. `handle_heartbeat` records
//! `sysid_mygcs_seen` when the sender is `MAV_GCS_SYSID`. COMMAND_LONG
//! and COMMAND_INT are classified against the Plane table
//! (ARM/DISARM, DO_SET_MODE, NAV_TAKEOFF). PARAM_REQUEST_LIST starts a
//! queued `PARAM_VALUE` walk; PARAM_SET writes a named scalar in the
//! in-memory table. MISSION_ITEM_INT writes one waypoint; MISSION_REQUEST_INT
//! looks it up for download. Send path is `GCS_MAVLINK::send_heartbeat`,
//! `send_text`, `queued_param_send`, `send_parameter_value`,
//! `send_attitude`, `send_global_position_int`, `send_mission_item_int`,
//! `send_sys_status`, `send_battery_status`, `send_rc_channels`,
//! `send_servo_output_raw`, `send_vfr_hud`, and
//! `send_nav_controller_output`. REQUEST_DATA_STREAM and
//! `MAV_CMD_SET_MESSAGE_INTERVAL` write the rate table.
//! RC_CHANNELS_OVERRIDE / MANUAL_CONTROL store channel overrides.

use crate::channels::{
    ChannelSnapshot, MSG_ID_RC_CHANNELS, MSG_ID_SERVO_OUTPUT_RAW, RC_CHANNELS_LEN,
    SERVO_OUTPUT_RAW_LEN,
};
use crate::command::{
    classify, CommandInt, CommandLong, CommandVia, PlaneCommand, MSG_ID_COMMAND_INT,
    MSG_ID_COMMAND_LONG,
};
use crate::framing::{decode_v2, encode_v2, DecodeError, Frame};
use crate::health::{
    HealthSnapshot, BATTERY_STATUS_LEN, MSG_ID_BATTERY_STATUS, MSG_ID_SYS_STATUS, SYS_STATUS_LEN,
};
use crate::heartbeat::{Heartbeat, MSG_ID_HEARTBEAT};
use crate::hud::{
    HudSnapshot, MSG_ID_NAV_CONTROLLER_OUTPUT, MSG_ID_VFR_HUD, NAV_CONTROLLER_OUTPUT_LEN,
    VFR_HUD_LEN,
};
use crate::mission::{
    MissionItemInt, MissionRequestInt, MissionTable, MISSION_ITEM_INT_LEN, MSG_ID_MISSION_ITEM_INT,
    MSG_ID_MISSION_REQUEST_INT,
};
use crate::param::{
    ParamRequestList, ParamSet, ParamTable, ParamValue, MSG_ID_PARAM_REQUEST_LIST,
    MSG_ID_PARAM_SET, PARAM_VALUE_LEN,
};
use crate::pose::{
    PoseSnapshot, ATTITUDE_LEN, GLOBAL_POSITION_INT_LEN, MSG_ID_ATTITUDE,
    MSG_ID_GLOBAL_POSITION_INT,
};
use crate::rates::{
    RateTable, RequestDataStream, MAV_CMD_SET_MESSAGE_INTERVAL, MSG_ID_REQUEST_DATA_STREAM,
};
use crate::rc_override::{
    ManualControl, OverrideStore, RcChannelsOverride, MSG_ID_MANUAL_CONTROL,
    MSG_ID_RC_CHANNELS_OVERRIDE,
};
use crate::statustext::{StatusText, MSG_ID_STATUSTEXT, STATUSTEXT_LEN};

/// Default vehicle sysid, upstream `MAV_SYSID` / `g.sysid_this_mav` default.
pub const DEFAULT_SYSID: u8 = 1;

/// Autopilot component id, upstream `MAV_COMP_ID_AUTOPILOT1`.
pub const MAV_COMP_ID_AUTOPILOT1: u8 = 1;

/// Default GCS sysid, upstream `MAV_GCS_SYSID`.
pub const DEFAULT_GCS_SYSID: u8 = 255;

/// Result of [`GcsMavlink::handle_message`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Dispatch {
    /// Msgid 0 — decoded HEARTBEAT, and whether it refreshed the GCS timer.
    Heartbeat {
        /// Unpacked payload.
        heartbeat: Heartbeat,
        /// `true` when `msg.sysid` matched `MAV_GCS_SYSID` (`sysid_is_gcs`).
        from_gcs: bool,
    },
    /// Msgid 76 / 75 — COMMAND_LONG / COMMAND_INT against the Plane table.
    Command {
        /// Which command message carried the id.
        via: CommandVia,
        /// Raw `MAV_CMD`.
        command: u16,
        /// `Some` when the id is ARM/DISARM, DO_SET_MODE, or NAV_TAKEOFF.
        kind: Option<PlaneCommand>,
    },
    /// Msgid 21 — PARAM_REQUEST_LIST started a queued `PARAM_VALUE` walk.
    ParamRequestList {
        /// `AP_Param::count_parameters` for this table.
        count: u16,
    },
    /// Msgid 23 — PARAM_SET against a named scalar.
    ParamSet {
        /// `true` when `AP_Param::find` succeeded and the value was finite.
        applied: bool,
    },
    /// Msgid 73 — MISSION_ITEM_INT written into the in-memory table.
    MissionItemInt {
        /// Waypoint sequence number from the payload.
        seq: u16,
        /// `true` when the table accepted the item.
        stored: bool,
    },
    /// Msgid 51 — MISSION_REQUEST_INT looked up a stored waypoint.
    MissionRequestInt {
        /// Requested sequence number.
        seq: u16,
        /// `true` when that seq is in the table.
        found: bool,
    },
    /// Msgid 66 — REQUEST_DATA_STREAM wrote stream msgid intervals.
    RequestDataStream {
        /// `MAV_DATA_STREAM` id.
        stream_id: u8,
        /// Requested rate, Hz (`0` when `start_stop` is stop).
        rate_hz: u16,
        /// How many known stream msgids were written into the rate table.
        written: usize,
    },
    /// COMMAND_LONG / COMMAND_INT `MAV_CMD_SET_MESSAGE_INTERVAL` (511).
    SetMessageInterval {
        /// Target MAVLink msgid (`param1`).
        msgid: u32,
        /// Stored period, milliseconds (`0` = stop).
        interval_ms: u16,
        /// `true` when the table accepted the interval.
        applied: bool,
    },
    /// Msgid 70 — RC_CHANNELS_OVERRIDE stored into the override table.
    RcChannelsOverride {
        /// How many channel slots were written (not ignored).
        applied: usize,
    },
    /// Msgid 69 — MANUAL_CONTROL axes mapped onto roll/pitch/throttle/rudder.
    ManualControl {
        /// How many of the four Plane axes were written.
        applied: usize,
    },
    /// Any other msgid, or a command/param/mission not addressed to this vehicle.
    Unknown {
        /// Unrecognised message id.
        msgid: u32,
    },
}

/// One GCS channel: HEARTBEAT send, msgid-0 receive, command-table stub,
/// PARAM_REQUEST_LIST / PARAM_SET against an in-memory table,
/// ATTITUDE / GLOBAL_POSITION_INT stream send from a pose snapshot,
/// MISSION_ITEM_INT / MISSION_REQUEST_INT against an in-memory mission table,
/// SYS_STATUS / BATTERY_STATUS stream send from a health snapshot,
/// RC_CHANNELS / SERVO_OUTPUT_RAW stream send from a channel snapshot,
/// and MANUAL_CONTROL / RC_CHANNELS_OVERRIDE ingest into an override table.
///
/// Mirrors the `GCS_MAVLINK` methods this slice covers, not the full class.
#[derive(Debug, Clone)]
pub struct GcsMavlink {
    sysid: u8,
    compid: u8,
    seq: u8,
    gcs_sysid: u8,
    last_gcs_heartbeat_ms: u32,
    params: ParamTable,
    mission: MissionTable,
    rates: RateTable,
    overrides: OverrideStore,
}

impl Default for GcsMavlink {
    fn default() -> Self {
        Self {
            sysid: DEFAULT_SYSID,
            compid: MAV_COMP_ID_AUTOPILOT1,
            seq: 0,
            gcs_sysid: DEFAULT_GCS_SYSID,
            last_gcs_heartbeat_ms: 0,
            params: ParamTable::plane_stub(),
            mission: MissionTable::new(),
            rates: RateTable::new(),
            overrides: OverrideStore::new(),
        }
    }
}

impl GcsMavlink {
    /// A channel with default sysids (vehicle 1, GCS 255).
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Configure `MAV_GCS_SYSID` used by `sysid_is_gcs`.
    #[must_use]
    pub fn with_gcs_sysid(mut self, gcs_sysid: u8) -> Self {
        self.gcs_sysid = gcs_sysid;
        self
    }

    /// Last `sysid_mygcs_seen` timestamp, 0 until a GCS HEARTBEAT arrives.
    #[must_use]
    pub const fn last_gcs_heartbeat_ms(&self) -> u32 {
        self.last_gcs_heartbeat_ms
    }

    /// Scalar count in the in-memory table.
    #[must_use]
    pub const fn param_count(&self) -> u16 {
        self.params.count()
    }

    /// Current value of a named scalar, if present.
    #[must_use]
    pub fn param_value(&self, name: &str) -> Option<f32> {
        let id = crate::param::encode_param_id(name);
        self.params.find(&id).map(|(_, entry)| entry.value)
    }

    /// Stored mission item count.
    #[must_use]
    pub const fn mission_count(&self) -> u16 {
        self.mission.count()
    }

    /// Stored stream interval for `msgid`, if the rate table has a slot.
    #[must_use]
    pub fn stream_interval_ms(&self, msgid: u32) -> Option<u16> {
        self.rates.interval_ms(msgid)
    }

    /// Stored PWM override for channel `i` (0-based), if that slot is active.
    #[must_use]
    pub fn override_channel(&self, i: usize) -> Option<u16> {
        self.overrides.get(i)
    }

    /// Timestamp of the last accepted override ingest.
    #[must_use]
    pub const fn last_override_ms(&self) -> u32 {
        self.overrides.last_ms()
    }

    /// Look up a stored mission item by sequence number.
    #[must_use]
    pub fn mission_item(&self, seq: u16) -> Option<MissionItemInt> {
        self.mission.get(seq).copied()
    }

    /// Encode one outgoing HEARTBEAT, upstream `GCS_MAVLINK::send_heartbeat`.
    ///
    /// Autopilot is `MAV_AUTOPILOT_ARDUPILOTMEGA`. Returns the framed length.
    #[must_use]
    pub fn send_heartbeat(
        &mut self,
        out: &mut [u8],
        frame_type: u8,
        base_mode: u8,
        custom_mode: u32,
        system_status: u8,
    ) -> Option<usize> {
        let hb = Heartbeat::plane(frame_type, base_mode, custom_mode, system_status);
        let mut payload = [0u8; 9];
        hb.encode(&mut payload)?;
        let frame = Frame::new(
            self.seq,
            self.sysid,
            self.compid,
            MSG_ID_HEARTBEAT,
            &payload,
        )?;
        self.seq = self.seq.wrapping_add(1);
        encode_v2(&frame, out)
    }

    /// Encode one outgoing STATUSTEXT, upstream `GCS_MAVLINK::send_text`.
    ///
    /// Packs severity + the 50-byte text field (truncated, zero-filled)
    /// and frames it for Write. Queueing / printf / multi-chunk is later.
    #[must_use]
    pub fn send_text(&mut self, out: &mut [u8], severity: u8, text: &str) -> Option<usize> {
        let st = StatusText::new(severity, text);
        let mut payload = [0u8; STATUSTEXT_LEN];
        st.encode(&mut payload)?;
        let frame = Frame::new(
            self.seq,
            self.sysid,
            self.compid,
            MSG_ID_STATUSTEXT,
            &payload,
        )?;
        self.seq = self.seq.wrapping_add(1);
        encode_v2(&frame, out)
    }

    /// Encode the next queued `PARAM_VALUE`, upstream `queued_param_send`.
    ///
    /// `None` when the list walk is finished or `out` is too small.
    #[must_use]
    pub fn queued_param_send(&mut self, out: &mut [u8]) -> Option<usize> {
        let value = self.params.next_queued()?;
        self.encode_param_value(out, &value)
    }

    /// Encode one `PARAM_VALUE` by name, upstream `send_parameter_value`.
    #[must_use]
    pub fn send_parameter_value(&mut self, out: &mut [u8], name: &str) -> Option<usize> {
        let id = crate::param::encode_param_id(name);
        let (index, _) = self.params.find(&id)?;
        let value = self.params.value_at(index)?;
        self.encode_param_value(out, &value)
    }

    /// Encode one outgoing ATTITUDE, upstream `GCS_MAVLINK::send_attitude`.
    ///
    /// Packs roll / pitch / yaw and gyro rates from `pose` and frames them
    /// for Write. Stream-rate gating (`MSG_ATTITUDE`) is later.
    #[must_use]
    pub fn send_attitude(&mut self, out: &mut [u8], pose: &PoseSnapshot) -> Option<usize> {
        let att = pose.attitude();
        let mut payload = [0u8; ATTITUDE_LEN];
        att.encode(&mut payload)?;
        let frame = Frame::new(self.seq, self.sysid, self.compid, MSG_ID_ATTITUDE, &payload)?;
        self.seq = self.seq.wrapping_add(1);
        encode_v2(&frame, out)
    }

    /// Encode one outgoing GLOBAL_POSITION_INT, upstream
    /// `GCS_MAVLINK::send_global_position_int`.
    ///
    /// Packs lat / lon / alt / NED velocity / heading from `pose` and frames
    /// them for Write. Stream-rate gating (`MSG_LOCATION`) is later.
    #[must_use]
    pub fn send_global_position_int(
        &mut self,
        out: &mut [u8],
        pose: &PoseSnapshot,
    ) -> Option<usize> {
        let gpi = pose.global_position_int();
        let mut payload = [0u8; GLOBAL_POSITION_INT_LEN];
        gpi.encode(&mut payload)?;
        let frame = Frame::new(
            self.seq,
            self.sysid,
            self.compid,
            MSG_ID_GLOBAL_POSITION_INT,
            &payload,
        )?;
        self.seq = self.seq.wrapping_add(1);
        encode_v2(&frame, out)
    }

    /// Encode one outgoing SYS_STATUS, upstream `GCS_MAVLINK::send_sys_status`.
    ///
    /// Packs sensor bitmaps, load, battery voltage / current / remaining, and
    /// error counters from `health` and frames them for Write. Stream-rate
    /// gating (`MSG_SYS_STATUS`) is later.
    #[must_use]
    pub fn send_sys_status(&mut self, out: &mut [u8], health: &HealthSnapshot) -> Option<usize> {
        let sys = health.sys_status();
        let mut payload = [0u8; SYS_STATUS_LEN];
        sys.encode(&mut payload)?;
        let frame = Frame::new(
            self.seq,
            self.sysid,
            self.compid,
            MSG_ID_SYS_STATUS,
            &payload,
        )?;
        self.seq = self.seq.wrapping_add(1);
        encode_v2(&frame, out)
    }

    /// Encode one outgoing BATTERY_STATUS, upstream
    /// `GCS_MAVLINK::send_battery_status`.
    ///
    /// Packs instance id, cell voltages, current, consumed charge / energy,
    /// and remaining time from `health` and frames them for Write.
    /// Stream-rate gating (`MSG_BATTERY_STATUS`) is later.
    #[must_use]
    pub fn send_battery_status(
        &mut self,
        out: &mut [u8],
        health: &HealthSnapshot,
    ) -> Option<usize> {
        let batt = health.battery_status();
        let mut payload = [0u8; BATTERY_STATUS_LEN];
        batt.encode(&mut payload)?;
        let frame = Frame::new(
            self.seq,
            self.sysid,
            self.compid,
            MSG_ID_BATTERY_STATUS,
            &payload,
        )?;
        self.seq = self.seq.wrapping_add(1);
        encode_v2(&frame, out)
    }

    /// Encode one outgoing RC_CHANNELS, upstream `GCS_MAVLINK::send_rc_channels`.
    ///
    /// Packs radio-in PWM, channel count, and RSSI from `channels` and frames
    /// them for Write. Stream-rate gating (`MSG_RC_CHANNELS`) is later.
    #[must_use]
    pub fn send_rc_channels(
        &mut self,
        out: &mut [u8],
        channels: &ChannelSnapshot,
    ) -> Option<usize> {
        let rc = channels.rc_channels();
        let mut payload = [0u8; RC_CHANNELS_LEN];
        rc.encode(&mut payload)?;
        let frame = Frame::new(
            self.seq,
            self.sysid,
            self.compid,
            MSG_ID_RC_CHANNELS,
            &payload,
        )?;
        self.seq = self.seq.wrapping_add(1);
        encode_v2(&frame, out)
    }

    /// Encode one outgoing SERVO_OUTPUT_RAW, upstream
    /// `GCS_MAVLINK::send_servo_output_raw`.
    ///
    /// Packs servo PWM and port from `channels` and frames them for Write.
    /// Stream-rate gating (`MSG_SERVO_OUTPUT_RAW`) is later.
    #[must_use]
    pub fn send_servo_output_raw(
        &mut self,
        out: &mut [u8],
        channels: &ChannelSnapshot,
    ) -> Option<usize> {
        let servo = channels.servo_output_raw();
        let mut payload = [0u8; SERVO_OUTPUT_RAW_LEN];
        servo.encode(&mut payload)?;
        let frame = Frame::new(
            self.seq,
            self.sysid,
            self.compid,
            MSG_ID_SERVO_OUTPUT_RAW,
            &payload,
        )?;
        self.seq = self.seq.wrapping_add(1);
        encode_v2(&frame, out)
    }

    /// Encode one outgoing VFR_HUD, upstream `GCS_MAVLINK::send_vfr_hud`.
    ///
    /// Packs airspeed / groundspeed / heading / throttle / altitude / climb
    /// from `hud` and frames them for Write. Stream-rate gating
    /// (`MSG_VFR_HUD`) is later.
    #[must_use]
    pub fn send_vfr_hud(&mut self, out: &mut [u8], hud: &HudSnapshot) -> Option<usize> {
        let vfr = hud.vfr_hud();
        let mut payload = [0u8; VFR_HUD_LEN];
        vfr.encode(&mut payload)?;
        let frame = Frame::new(self.seq, self.sysid, self.compid, MSG_ID_VFR_HUD, &payload)?;
        self.seq = self.seq.wrapping_add(1);
        encode_v2(&frame, out)
    }

    /// Encode VFR_HUD only when the stored period has elapsed.
    ///
    /// Upstream deferred send skips the msgid while
    /// `now - last_sent < interval_ms`. `None` when the table has no
    /// non-zero interval, the period has not elapsed, or `out` is too small.
    #[must_use]
    pub fn send_vfr_hud_if_due(
        &mut self,
        out: &mut [u8],
        hud: &HudSnapshot,
        now_ms: u32,
    ) -> Option<usize> {
        if !self.rates.should_send(MSG_ID_VFR_HUD, now_ms) {
            return None;
        }
        let n = self.send_vfr_hud(out, hud)?;
        self.rates.mark_sent(MSG_ID_VFR_HUD, now_ms);
        Some(n)
    }

    /// Encode one outgoing NAV_CONTROLLER_OUTPUT, upstream
    /// `GCS_MAVLINK_Plane::send_nav_controller_output`.
    ///
    /// Packs desired attitude / bearings, waypoint distance, and nav errors
    /// from `hud` and frames them for Write. Stream-rate gating
    /// (`MSG_NAV_CONTROLLER_OUTPUT`) is later.
    #[must_use]
    pub fn send_nav_controller_output(
        &mut self,
        out: &mut [u8],
        hud: &HudSnapshot,
    ) -> Option<usize> {
        let nav = hud.nav_controller_output();
        let mut payload = [0u8; NAV_CONTROLLER_OUTPUT_LEN];
        nav.encode(&mut payload)?;
        let frame = Frame::new(
            self.seq,
            self.sysid,
            self.compid,
            MSG_ID_NAV_CONTROLLER_OUTPUT,
            &payload,
        )?;
        self.seq = self.seq.wrapping_add(1);
        encode_v2(&frame, out)
    }

    /// Encode one stored `MISSION_ITEM_INT`, upstream mission-download reply.
    ///
    /// `None` when `seq` is missing or `out` is too small.
    #[must_use]
    pub fn send_mission_item_int(&mut self, out: &mut [u8], seq: u16) -> Option<usize> {
        let item = *self.mission.get(seq)?;
        let mut payload = [0u8; MISSION_ITEM_INT_LEN];
        item.encode(&mut payload)?;
        let frame = Frame::new(
            self.seq,
            self.sysid,
            self.compid,
            MSG_ID_MISSION_ITEM_INT,
            &payload,
        )?;
        self.seq = self.seq.wrapping_add(1);
        encode_v2(&frame, out)
    }

    /// Dispatch one already-framed message, upstream `handle_message`.
    pub fn handle_message(&mut self, frame: &Frame, now_ms: u32) -> Dispatch {
        match frame.msgid {
            MSG_ID_HEARTBEAT => self.handle_heartbeat_frame(frame, now_ms),
            MSG_ID_COMMAND_LONG => match CommandLong::from_frame(frame) {
                Some(cmd) if self.addressed_to_us(cmd.target_system) => {
                    if cmd.command == MAV_CMD_SET_MESSAGE_INTERVAL {
                        self.handle_set_message_interval(cmd.param1, cmd.param2, cmd.param3)
                    } else {
                        Dispatch::Command {
                            via: CommandVia::Long,
                            command: cmd.command,
                            kind: classify(cmd.command),
                        }
                    }
                }
                _ => Dispatch::Unknown {
                    msgid: MSG_ID_COMMAND_LONG,
                },
            },
            MSG_ID_COMMAND_INT => match CommandInt::from_frame(frame) {
                Some(cmd) if self.addressed_to_us(cmd.target_system) => {
                    if cmd.command == MAV_CMD_SET_MESSAGE_INTERVAL {
                        self.handle_set_message_interval(cmd.param1, cmd.param2, cmd.param3)
                    } else {
                        Dispatch::Command {
                            via: CommandVia::Int,
                            command: cmd.command,
                            kind: classify(cmd.command),
                        }
                    }
                }
                _ => Dispatch::Unknown {
                    msgid: MSG_ID_COMMAND_INT,
                },
            },
            MSG_ID_PARAM_REQUEST_LIST => match ParamRequestList::from_frame(frame) {
                Some(req) if self.addressed_to_us(req.target_system) => {
                    self.params.start_list();
                    Dispatch::ParamRequestList {
                        count: self.params.count(),
                    }
                }
                _ => Dispatch::Unknown {
                    msgid: MSG_ID_PARAM_REQUEST_LIST,
                },
            },
            MSG_ID_PARAM_SET => match ParamSet::from_frame(frame) {
                Some(set) if self.addressed_to_us(set.target_system) => Dispatch::ParamSet {
                    applied: self.params.set(&set.param_id, set.param_value).is_some(),
                },
                _ => Dispatch::Unknown {
                    msgid: MSG_ID_PARAM_SET,
                },
            },
            MSG_ID_MISSION_ITEM_INT => match MissionItemInt::from_frame(frame) {
                Some(item) if self.addressed_to_us(item.target_system) => {
                    let seq = item.seq;
                    Dispatch::MissionItemInt {
                        seq,
                        stored: self.mission.set(item).is_some(),
                    }
                }
                _ => Dispatch::Unknown {
                    msgid: MSG_ID_MISSION_ITEM_INT,
                },
            },
            MSG_ID_MISSION_REQUEST_INT => match MissionRequestInt::from_frame(frame) {
                Some(req) if self.addressed_to_us(req.target_system) => {
                    Dispatch::MissionRequestInt {
                        seq: req.seq,
                        found: self.mission.get(req.seq).is_some(),
                    }
                }
                _ => Dispatch::Unknown {
                    msgid: MSG_ID_MISSION_REQUEST_INT,
                },
            },
            MSG_ID_RC_CHANNELS_OVERRIDE => match RcChannelsOverride::from_frame(frame) {
                Some(pkt)
                    if frame.sysid == self.gcs_sysid && self.addressed_to_us(pkt.target_system) =>
                {
                    let applied = self.overrides.apply_rc_channels_override(&pkt, now_ms);
                    // Upstream `handle_rc_channels_override` → `sysid_mygcs_seen`.
                    self.last_gcs_heartbeat_ms = now_ms;
                    Dispatch::RcChannelsOverride { applied }
                }
                _ => Dispatch::Unknown {
                    msgid: MSG_ID_RC_CHANNELS_OVERRIDE,
                },
            },
            MSG_ID_MANUAL_CONTROL => match ManualControl::from_frame(frame) {
                Some(pkt) if frame.sysid == self.gcs_sysid && pkt.target == self.sysid => {
                    let applied = self.overrides.apply_manual_control(&pkt, now_ms);
                    // Upstream `handle_manual_control` → `sysid_mygcs_seen`.
                    self.last_gcs_heartbeat_ms = now_ms;
                    Dispatch::ManualControl { applied }
                }
                _ => Dispatch::Unknown {
                    msgid: MSG_ID_MANUAL_CONTROL,
                },
            },
            MSG_ID_REQUEST_DATA_STREAM => match RequestDataStream::from_frame(frame) {
                Some(req) if self.addressed_to_us(req.target_system) => {
                    let written = self.rates.apply_request_data_stream(&req);
                    let rate_hz = if req.start_stop == 1 {
                        req.req_message_rate
                    } else {
                        0
                    };
                    Dispatch::RequestDataStream {
                        stream_id: req.req_stream_id,
                        rate_hz,
                        written,
                    }
                }
                _ => Dispatch::Unknown {
                    msgid: MSG_ID_REQUEST_DATA_STREAM,
                },
            },
            msgid => Dispatch::Unknown { msgid },
        }
    }

    /// Decode a raw buffer then [`Self::handle_message`].
    pub fn handle_bytes(&mut self, buf: &[u8], now_ms: u32) -> Result<Dispatch, DecodeError> {
        let frame = decode_v2(buf)?;
        Ok(self.handle_message(&frame, now_ms))
    }

    fn encode_param_value(&mut self, out: &mut [u8], value: &ParamValue) -> Option<usize> {
        let mut payload = [0u8; PARAM_VALUE_LEN];
        value.encode(&mut payload)?;
        let frame = Frame::new(
            self.seq,
            self.sysid,
            self.compid,
            crate::param::MSG_ID_PARAM_VALUE,
            &payload,
        )?;
        self.seq = self.seq.wrapping_add(1);
        encode_v2(&frame, out)
    }

    fn handle_heartbeat_frame(&mut self, frame: &Frame, now_ms: u32) -> Dispatch {
        let heartbeat = match Heartbeat::from_frame(frame) {
            Some(hb) => hb,
            None => {
                return Dispatch::Unknown {
                    msgid: MSG_ID_HEARTBEAT,
                };
            }
        };
        let from_gcs = frame.sysid == self.gcs_sysid;
        if from_gcs {
            // Upstream `handle_heartbeat` → `sysid_mygcs_seen(millis())`.
            self.last_gcs_heartbeat_ms = now_ms;
        }
        Dispatch::Heartbeat {
            heartbeat,
            from_gcs,
        }
    }

    /// Broadcast (`target_system` 0) or our `MAV_SYSID`.
    #[must_use]
    const fn addressed_to_us(&self, target_system: u8) -> bool {
        target_system == 0 || target_system == self.sysid
    }

    fn handle_set_message_interval(&mut self, param1: f32, param2: f32, param3: f32) -> Dispatch {
        let msgid = param1 as u32;
        // Upstream denies when param3 is non-zero. Compare bits so clippy
        // `float_cmp` stays quiet and -0.0 counts as zero.
        if param3.to_bits() != 0 && param3.to_bits() != (-0.0_f32).to_bits() {
            return Dispatch::SetMessageInterval {
                msgid,
                interval_ms: 0,
                applied: false,
            };
        }
        let interval_us = param2 as i32;
        match self.rates.set_message_interval(msgid, interval_us) {
            Some(interval_ms) => Dispatch::SetMessageInterval {
                msgid,
                interval_ms,
                applied: true,
            },
            None => Dispatch::SetMessageInterval {
                msgid,
                interval_ms: 0,
                applied: false,
            },
        }
    }
}
