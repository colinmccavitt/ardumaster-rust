//! Msgid dispatch stub, upstream `GCS_MAVLINK::handle_message`.
//!
//! Only HEARTBEAT is wired on receive. `handle_heartbeat` records
//! `sysid_mygcs_seen` when the sender is `MAV_GCS_SYSID`. Send path is
//! `GCS_MAVLINK::send_heartbeat` and `send_text`.

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
    /// Any other msgid. Later slices fill this table.
    Unknown {
        /// Unrecognised message id.
        msgid: u32,
    },
}

/// One GCS channel: HEARTBEAT send and msgid-0 receive.
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
        if frame.msgid != MSG_ID_HEARTBEAT {
            return Dispatch::Unknown { msgid: frame.msgid };
        }
        let heartbeat = match Heartbeat::from_frame(frame) {
            Some(hb) => hb,
            None => {
                return Dispatch::Unknown { msgid: frame.msgid };
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

    /// Decode a raw buffer then [`Self::handle_message`].
    pub fn handle_bytes(&mut self, buf: &[u8], now_ms: u32) -> Result<Dispatch, DecodeError> {
        let frame = decode_v2(buf)?;
        Ok(self.handle_message(&frame, now_ms))
    }
}
