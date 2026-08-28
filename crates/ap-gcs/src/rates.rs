//! REQUEST_DATA_STREAM / SET_MESSAGE_INTERVAL rate table, extracted from
//! the pinned Plane-4.7.0 `modules/mavlink/message_definitions/v1.0`
//! defs (`common.xml` msgid 66, `MAV_CMD_SET_MESSAGE_INTERVAL` 511).
//!
//! Upstream `GCS_MAVLINK::handle_request_data_stream` writes a stream
//! rate then `initialise_message_intervals_for_stream` stores per-message
//! intervals. `handle_command_set_message_interval` does the same for one
//! msgid (`set_message_interval`). Deferred send then skips a msgid when
//! `now - last_sent < interval_ms`. This slice is that table plus the two
//! wire setters — not the full dialect or the bucket scheduler.

use crate::framing::Frame;

/// REQUEST_DATA_STREAM message id, upstream `MAVLINK_MSG_ID_REQUEST_DATA_STREAM`.
pub const MSG_ID_REQUEST_DATA_STREAM: u32 = 66;

/// Packed payload length, upstream `MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN`.
pub const REQUEST_DATA_STREAM_LEN: usize = 6;

/// CRC extra, upstream `MAVLINK_MSG_ID_REQUEST_DATA_STREAM_CRC`.
pub const REQUEST_DATA_STREAM_CRC: u8 = 148;

/// `MAV_CMD_SET_MESSAGE_INTERVAL` — per-msgid interval (microseconds).
pub const MAV_CMD_SET_MESSAGE_INTERVAL: u16 = 511;

/// `MAV_DATA_STREAM_ALL`.
pub const MAV_DATA_STREAM_ALL: u8 = 0;

/// `MAV_DATA_STREAM_EXTENDED_STATUS`.
pub const MAV_DATA_STREAM_EXTENDED_STATUS: u8 = 2;

/// `MAV_DATA_STREAM_RC_CHANNELS`.
pub const MAV_DATA_STREAM_RC_CHANNELS: u8 = 3;

/// `MAV_DATA_STREAM_POSITION`.
pub const MAV_DATA_STREAM_POSITION: u8 = 6;

/// `MAV_DATA_STREAM_EXTRA1` — Plane ATTITUDE stream.
pub const MAV_DATA_STREAM_EXTRA1: u8 = 10;

/// `MAV_DATA_STREAM_EXTRA2` — Plane VFR_HUD stream.
pub const MAV_DATA_STREAM_EXTRA2: u8 = 11;

/// `MAV_DATA_STREAM_EXTRA3` — Plane BATTERY_STATUS stream.
pub const MAV_DATA_STREAM_EXTRA3: u8 = 12;

/// Capacity of the in-memory msgid → interval table.
pub const MAX_RATES: usize = 16;

/// Fastest interval this stub will store, upstream `set_message_interval` cap.
pub const MAX_INTERVAL_MS: u16 = 60_000;

/// Packed REQUEST_DATA_STREAM fields, upstream `mavlink_request_data_stream_t`.
///
/// Wire order is size-sorted (`mavlink_msg_request_data_stream_pack`):
/// `req_message_rate`, then `target_system`, `target_component`,
/// `req_stream_id`, `start_stop`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RequestDataStream {
    /// Requested stream rate, Hz.
    pub req_message_rate: u16,
    /// System ID.
    pub target_system: u8,
    /// Component ID.
    pub target_component: u8,
    /// `MAV_DATA_STREAM` id.
    pub req_stream_id: u8,
    /// 1 to start sending, 0 to stop.
    pub start_stop: u8,
}

impl RequestDataStream {
    /// Build a REQUEST_DATA_STREAM from the XML field order.
    #[must_use]
    pub const fn new(
        target_system: u8,
        target_component: u8,
        req_stream_id: u8,
        req_message_rate: u16,
        start_stop: u8,
    ) -> Self {
        Self {
            req_message_rate,
            target_system,
            target_component,
            req_stream_id,
            start_stop,
        }
    }

    /// Pack into 6 little-endian bytes. `None` if `buf` is too short.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..REQUEST_DATA_STREAM_LEN)?;
        dest.get_mut(..2)?
            .copy_from_slice(&self.req_message_rate.to_le_bytes());
        *dest.get_mut(2)? = self.target_system;
        *dest.get_mut(3)? = self.target_component;
        *dest.get_mut(4)? = self.req_stream_id;
        *dest.get_mut(5)? = self.start_stop;
        Some(REQUEST_DATA_STREAM_LEN)
    }

    /// Unpack 6 bytes. `None` if `buf` is shorter than the min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..REQUEST_DATA_STREAM_LEN)?;
        Some(Self {
            req_message_rate: u16::from_le_bytes(src.get(..2)?.try_into().ok()?),
            target_system: *src.get(2)?,
            target_component: *src.get(3)?,
            req_stream_id: *src.get(4)?,
            start_stop: *src.get(5)?,
        })
    }

    /// Decode a framed REQUEST_DATA_STREAM. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_REQUEST_DATA_STREAM {
            return None;
        }
        Self::decode(frame.payload())
    }
}

/// One scheduled msgid, upstream deferred-message / bucket interval slot.
#[derive(Debug, Clone, Copy)]
struct RateSlot {
    msgid: u32,
    interval_ms: u16,
    last_sent_ms: u32,
    sent: bool,
}

/// Msgid → interval table, upstream `set_ap_message_interval` + last-sent gate.
///
/// `interval_ms == 0` means "do not send" (`SET_MESSAGE_INTERVAL` −1 / stream
/// stop). A missing slot is also "do not send" until a setter installs one.
#[derive(Debug, Clone)]
pub struct RateTable {
    slots: [RateSlot; MAX_RATES],
    used: usize,
}

impl Default for RateTable {
    fn default() -> Self {
        Self::new()
    }
}

impl RateTable {
    /// Empty table — nothing is due until a setter writes an interval.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            slots: [RateSlot {
                msgid: 0,
                interval_ms: 0,
                last_sent_ms: 0,
                sent: false,
            }; MAX_RATES],
            used: 0,
        }
    }

    /// Stored interval for `msgid`, or `None` if the msgid is not scheduled.
    #[must_use]
    pub fn interval_ms(&self, msgid: u32) -> Option<u16> {
        self.slot(msgid).map(|slot| slot.interval_ms)
    }

    /// `true` when `msgid` has a non-zero interval and the period has elapsed.
    ///
    /// Mirrors `deferred_message_to_send_index`: skip while
    /// `now - last_sent < interval_ms`. The first send after an interval is
    /// installed is always due.
    #[must_use]
    pub fn should_send(&self, msgid: u32, now_ms: u32) -> bool {
        let Some(slot) = self.slot(msgid) else {
            return false;
        };
        if slot.interval_ms == 0 {
            return false;
        }
        if !slot.sent {
            return true;
        }
        now_ms.wrapping_sub(slot.last_sent_ms) >= u32::from(slot.interval_ms)
    }

    /// Record that `msgid` was emitted at `now_ms`.
    pub fn mark_sent(&mut self, msgid: u32, now_ms: u32) {
        if let Some(slot) = self.slot_mut(msgid) {
            slot.last_sent_ms = now_ms;
            slot.sent = true;
        }
    }

    /// Install or replace the interval for one msgid.
    ///
    /// `interval_ms == 0` stops the msgid. `None` if the table is full.
    pub fn set_interval(&mut self, msgid: u32, interval_ms: u16) -> Option<u16> {
        if let Some(slot) = self.slot_mut(msgid) {
            slot.interval_ms = interval_ms;
            slot.last_sent_ms = 0;
            slot.sent = false;
            return Some(interval_ms);
        }
        if self.used >= MAX_RATES {
            return None;
        }
        let slot = self.slots.get_mut(self.used)?;
        *slot = RateSlot {
            msgid,
            interval_ms,
            last_sent_ms: 0,
            sent: false,
        };
        self.used = self.used.saturating_add(1);
        Some(interval_ms)
    }

    /// Apply `MAV_CMD_SET_MESSAGE_INTERVAL` (`param1` msgid, `param2` µs).
    ///
    /// Upstream `GCS_MAVLINK::set_message_interval`: 0 resets to default
    /// (this stub: stop), −1 stops, otherwise µs / 1000 (clamped 1..60000).
    /// `None` when the command is denied (`interval_us < -1`).
    pub fn set_message_interval(&mut self, msgid: u32, interval_us: i32) -> Option<u16> {
        let interval_ms = interval_us_to_ms(interval_us)?;
        self.set_interval(msgid, interval_ms)
    }

    /// Apply a decoded REQUEST_DATA_STREAM to the known Plane stream map.
    ///
    /// `start_stop == 0` stops the stream's msgids. `start_stop == 1` sets
    /// `1000 / req_message_rate` ms (or 0 Hz → stop). Other `start_stop`
    /// values are ignored. Unknown stream ids are ignored. Returns how
    /// many msgids were written.
    pub fn apply_request_data_stream(&mut self, req: &RequestDataStream) -> usize {
        let freq = match req.start_stop {
            0 => 0,
            1 => req.req_message_rate,
            _ => return 0,
        };
        let interval_ms = hz_to_interval_ms(freq);
        let mut written = 0usize;
        for &msgid in stream_msgids(req.req_stream_id) {
            if self.set_interval(msgid, interval_ms).is_some() {
                written = written.saturating_add(1);
            }
        }
        written
    }

    fn slot(&self, msgid: u32) -> Option<&RateSlot> {
        self.slots
            .get(..self.used)?
            .iter()
            .find(|slot| slot.msgid == msgid)
    }

    fn slot_mut(&mut self, msgid: u32) -> Option<&mut RateSlot> {
        self.slots
            .get_mut(..self.used)?
            .iter_mut()
            .find(|slot| slot.msgid == msgid)
    }
}

/// Convert `SET_MESSAGE_INTERVAL` `param2` (µs) to the stored millisecond period.
#[must_use]
pub const fn interval_us_to_ms(interval_us: i32) -> Option<u16> {
    if interval_us == 0 || interval_us == -1 {
        return Some(0);
    }
    if interval_us < 0 {
        return None;
    }
    if interval_us < 1000 {
        return Some(1);
    }
    if interval_us > 60_000_000 {
        return Some(MAX_INTERVAL_MS);
    }
    Some((interval_us / 1000) as u16)
}

/// Convert a REQUEST_DATA_STREAM Hz rate to milliseconds, upstream
/// `get_interval_for_stream` (`1000 / frate`, cap 60000).
#[must_use]
pub const fn hz_to_interval_ms(freq_hz: u16) -> u16 {
    if freq_hz == 0 {
        return 0;
    }
    let ret = 1000 / (freq_hz as u32);
    if ret > (MAX_INTERVAL_MS as u32) {
        MAX_INTERVAL_MS
    } else {
        ret as u16
    }
}

/// Msgids this crate already streams, grouped like Plane `all_stream_entries`.
fn stream_msgids(stream_id: u8) -> &'static [u32] {
    match stream_id {
        MAV_DATA_STREAM_ALL => &[
            crate::health::MSG_ID_SYS_STATUS,
            crate::hud::MSG_ID_NAV_CONTROLLER_OUTPUT,
            crate::channels::MSG_ID_SERVO_OUTPUT_RAW,
            crate::channels::MSG_ID_RC_CHANNELS,
            crate::pose::MSG_ID_GLOBAL_POSITION_INT,
            crate::pose::MSG_ID_ATTITUDE,
            crate::hud::MSG_ID_VFR_HUD,
            crate::health::MSG_ID_BATTERY_STATUS,
        ],
        MAV_DATA_STREAM_EXTENDED_STATUS => &[
            crate::health::MSG_ID_SYS_STATUS,
            crate::hud::MSG_ID_NAV_CONTROLLER_OUTPUT,
        ],
        MAV_DATA_STREAM_RC_CHANNELS => &[
            crate::channels::MSG_ID_SERVO_OUTPUT_RAW,
            crate::channels::MSG_ID_RC_CHANNELS,
        ],
        MAV_DATA_STREAM_POSITION => &[crate::pose::MSG_ID_GLOBAL_POSITION_INT],
        MAV_DATA_STREAM_EXTRA1 => &[crate::pose::MSG_ID_ATTITUDE],
        MAV_DATA_STREAM_EXTRA2 => &[crate::hud::MSG_ID_VFR_HUD],
        MAV_DATA_STREAM_EXTRA3 => &[crate::health::MSG_ID_BATTERY_STATUS],
        _ => &[],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_writes_rate_first() {
        let req = RequestDataStream::new(1, 1, MAV_DATA_STREAM_EXTRA2, 0x0201, 1);
        let mut buf = [0u8; REQUEST_DATA_STREAM_LEN];
        assert_eq!(req.encode(&mut buf), Some(REQUEST_DATA_STREAM_LEN));
        assert_eq!(buf.get(..2), Some([0x01, 0x02].as_slice()));
        assert_eq!(buf.get(4).copied(), Some(MAV_DATA_STREAM_EXTRA2));
        assert_eq!(buf.get(5).copied(), Some(1));
    }

    #[test]
    fn decode_rejects_short_payload() {
        assert!(RequestDataStream::decode(&[0, 1, 2]).is_none());
    }

    #[test]
    fn interval_us_matches_upstream_clamp() {
        assert_eq!(interval_us_to_ms(0), Some(0));
        assert_eq!(interval_us_to_ms(-1), Some(0));
        assert_eq!(interval_us_to_ms(-2), None);
        assert_eq!(interval_us_to_ms(500), Some(1));
        assert_eq!(interval_us_to_ms(100_000), Some(100));
        assert_eq!(interval_us_to_ms(70_000_000), Some(MAX_INTERVAL_MS));
    }
}
