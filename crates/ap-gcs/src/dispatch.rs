//! Msgid dispatch stub, upstream `GCS_MAVLINK::handle_message`.
//!
//! HEARTBEAT is wired on receive. `handle_heartbeat` records
//! `sysid_mygcs_seen` when the sender is `MAV_GCS_SYSID`. COMMAND_LONG
//! and COMMAND_INT are classified against the Plane table
//! (ARM/DISARM, DO_SET_MODE, NAV_TAKEOFF). Send path is
//! `GCS_MAVLINK::send_heartbeat` and `send_text`.

use crate::command::{
    classify, CommandInt, CommandLong, CommandVia, PlaneCommand, MSG_ID_COMMAND_INT,
    MSG_ID_COMMAND_LONG,
};
use crate::framing::{decode_v2, encode_v2, DecodeError, Frame};
use crate::heartbeat::{Heartbeat, MSG_ID_HEARTBEAT};
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
    /// Any other msgid, or a command not addressed to this vehicle.
    Unknown {
        /// Unrecognised message id.
        msgid: u32,
    },
}

/// One GCS channel: HEARTBEAT send, msgid-0 receive, command-table stub.
///
/// Mirrors the `GCS_MAVLINK` methods this slice covers, not the full class.
#[derive(Debug, Clone)]
pub struct GcsMavlink {
    sysid: u8,
    compid: u8,
    seq: u8,
    gcs_sysid: u8,
    last_gcs_heartbeat_ms: u32,
}

impl Default for GcsMavlink {
    fn default() -> Self {
        Self {
            sysid: DEFAULT_SYSID,
            compid: MAV_COMP_ID_AUTOPILOT1,
            seq: 0,
            gcs_sysid: DEFAULT_GCS_SYSID,
            last_gcs_heartbeat_ms: 0,
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

    /// Dispatch one already-framed message, upstream `handle_message`.
    pub fn handle_message(&mut self, frame: &Frame, now_ms: u32) -> Dispatch {
        match frame.msgid {
            MSG_ID_HEARTBEAT => self.handle_heartbeat_frame(frame, now_ms),
            MSG_ID_COMMAND_LONG => match CommandLong::from_frame(frame) {
                Some(cmd) if self.addressed_to_us(cmd.target_system) => Dispatch::Command {
                    via: CommandVia::Long,
                    command: cmd.command,
                    kind: classify(cmd.command),
                },
                _ => Dispatch::Unknown {
                    msgid: MSG_ID_COMMAND_LONG,
                },
            },
            MSG_ID_COMMAND_INT => match CommandInt::from_frame(frame) {
                Some(cmd) if self.addressed_to_us(cmd.target_system) => Dispatch::Command {
                    via: CommandVia::Int,
                    command: cmd.command,
                    kind: classify(cmd.command),
                },
                _ => Dispatch::Unknown {
                    msgid: MSG_ID_COMMAND_INT,
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
}
