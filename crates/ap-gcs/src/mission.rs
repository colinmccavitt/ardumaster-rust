//! MISSION_ITEM_INT / MISSION_REQUEST_INT stub, extracted from the pinned
//! Plane-4.7.0 `modules/mavlink/message_definitions/v1.0/common.xml`
//! (msgid 73 / 51).
//!
//! This slice is the GCS_MAVLink mission microservice stub: upload one
//! waypoint (`handle_mission_item_int`) into a small in-memory table, then
//! download it (`handle_mission_request_int` + `send_mission_item_int`).
//! `MISSION_COUNT` / `MISSION_ACK` and the full dialect generator are later
//! FW-028 slices.

use crate::command::MAV_FRAME_GLOBAL_RELATIVE_ALT;
use crate::framing::Frame;

/// MISSION_REQUEST_INT message id, upstream `MAVLINK_MSG_ID_MISSION_REQUEST_INT`.
pub const MSG_ID_MISSION_REQUEST_INT: u32 = 51;

/// MISSION_ITEM_INT message id, upstream `MAVLINK_MSG_ID_MISSION_ITEM_INT`.
pub const MSG_ID_MISSION_ITEM_INT: u32 = 73;

/// Packed payload length, upstream `MAVLINK_MSG_ID_MISSION_REQUEST_INT_LEN`.
pub const MISSION_REQUEST_INT_LEN: usize = 5;

/// Packed payload length, upstream `MAVLINK_MSG_ID_MISSION_ITEM_INT_LEN`.
pub const MISSION_ITEM_INT_LEN: usize = 38;

/// CRC extra, upstream `MAVLINK_MSG_ID_MISSION_REQUEST_INT_CRC`.
pub const MISSION_REQUEST_INT_CRC: u8 = 196;

/// CRC extra, upstream `MAVLINK_MSG_ID_MISSION_ITEM_INT_CRC`.
pub const MISSION_ITEM_INT_CRC: u8 = 38;

/// `MAV_CMD_NAV_WAYPOINT` — typical Plane waypoint command.
pub const MAV_CMD_NAV_WAYPOINT: u16 = 16;

/// `MAV_MISSION_TYPE_MISSION` — main mission items (not fence / rally).
pub const MAV_MISSION_TYPE_MISSION: u8 = 0;

/// Capacity of the in-memory table (not the full `AP_Mission` storage).
pub const MAX_MISSION_ITEMS: usize = 8;

/// Packed MISSION_REQUEST_INT fields, upstream `mavlink_mission_request_int_t`.
///
/// Wire order is size-sorted (`mavlink_msg_mission_request_int_pack`): `seq`,
/// then `target_system`, `target_component`, `mission_type`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MissionRequestInt {
    /// Sequence number of the requested item.
    pub seq: u16,
    /// System ID.
    pub target_system: u8,
    /// Component ID.
    pub target_component: u8,
    /// Mission type (`MAV_MISSION_TYPE`).
    pub mission_type: u8,
}

impl MissionRequestInt {
    /// Build a MISSION_REQUEST_INT from the XML field order (not wire order).
    #[must_use]
    pub const fn new(target_system: u8, target_component: u8, seq: u16, mission_type: u8) -> Self {
        Self {
            seq,
            target_system,
            target_component,
            mission_type,
        }
    }

    /// Pack into 5 little-endian bytes. `None` if `buf` is too short.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..MISSION_REQUEST_INT_LEN)?;
        dest.get_mut(..2)?.copy_from_slice(&self.seq.to_le_bytes());
        *dest.get_mut(2)? = self.target_system;
        *dest.get_mut(3)? = self.target_component;
        *dest.get_mut(4)? = self.mission_type;
        Some(MISSION_REQUEST_INT_LEN)
    }

    /// Unpack 5 bytes. `None` if `buf` is shorter than the min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..MISSION_REQUEST_INT_LEN)?;
        Some(Self {
            seq: u16::from_le_bytes(src.get(..2)?.try_into().ok()?),
            target_system: *src.get(2)?,
            target_component: *src.get(3)?,
            mission_type: *src.get(4)?,
        })
    }

    /// Decode a framed MISSION_REQUEST_INT. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_MISSION_REQUEST_INT {
            return None;
        }
        Self::decode(frame.payload())
    }
}

/// Packed MISSION_ITEM_INT fields, upstream `mavlink_mission_item_int_t`.
///
/// Wire order is size-sorted (`mavlink_msg_mission_item_int_pack`): four
/// floats, `x` / `y`, `z`, `seq`, `command`, then the six u8 fields.
#[derive(Debug, Clone, Copy)]
pub struct MissionItemInt {
    /// PARAM1, see `MAV_CMD`.
    pub param1: f32,
    /// PARAM2, see `MAV_CMD`.
    pub param2: f32,
    /// PARAM3, see `MAV_CMD`.
    pub param3: f32,
    /// PARAM4, see `MAV_CMD`.
    pub param4: f32,
    /// PARAM5 / local: x * 1e4, global: latitude degE7.
    pub x: i32,
    /// PARAM6 / local: y * 1e4, global: longitude degE7.
    pub y: i32,
    /// PARAM7 / altitude in metres (frame-dependent).
    pub z: f32,
    /// Waypoint sequence number (starts at zero).
    pub seq: u16,
    /// Scheduled action (`MAV_CMD`).
    pub command: u16,
    /// System ID.
    pub target_system: u8,
    /// Component ID.
    pub target_component: u8,
    /// Coordinate system (`MAV_FRAME`).
    pub frame: u8,
    /// `false`: 0, `true`: 1.
    pub current: u8,
    /// Autocontinue to the next waypoint.
    pub autocontinue: u8,
    /// Mission type (`MAV_MISSION_TYPE`).
    pub mission_type: u8,
}

impl MissionItemInt {
    /// A Plane-shaped NAV_WAYPOINT at `lat`/`lon`/`alt_m` (global relative).
    #[must_use]
    pub const fn waypoint(
        target_system: u8,
        target_component: u8,
        seq: u16,
        lat: i32,
        lon: i32,
        alt_m: f32,
    ) -> Self {
        Self {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: lat,
            y: lon,
            z: alt_m,
            seq,
            command: MAV_CMD_NAV_WAYPOINT,
            target_system,
            target_component,
            frame: MAV_FRAME_GLOBAL_RELATIVE_ALT,
            current: 0,
            autocontinue: 1,
            mission_type: MAV_MISSION_TYPE_MISSION,
        }
    }

    /// Pack into 38 little-endian bytes. `None` if `buf` is too short.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..MISSION_ITEM_INT_LEN)?;
        dest.get_mut(..4)?
            .copy_from_slice(&self.param1.to_le_bytes());
        dest.get_mut(4..8)?
            .copy_from_slice(&self.param2.to_le_bytes());
        dest.get_mut(8..12)?
            .copy_from_slice(&self.param3.to_le_bytes());
        dest.get_mut(12..16)?
            .copy_from_slice(&self.param4.to_le_bytes());
        dest.get_mut(16..20)?.copy_from_slice(&self.x.to_le_bytes());
        dest.get_mut(20..24)?.copy_from_slice(&self.y.to_le_bytes());
        dest.get_mut(24..28)?.copy_from_slice(&self.z.to_le_bytes());
        dest.get_mut(28..30)?
            .copy_from_slice(&self.seq.to_le_bytes());
        dest.get_mut(30..32)?
            .copy_from_slice(&self.command.to_le_bytes());
        *dest.get_mut(32)? = self.target_system;
        *dest.get_mut(33)? = self.target_component;
        *dest.get_mut(34)? = self.frame;
        *dest.get_mut(35)? = self.current;
        *dest.get_mut(36)? = self.autocontinue;
        *dest.get_mut(37)? = self.mission_type;
        Some(MISSION_ITEM_INT_LEN)
    }

    /// Unpack 38 bytes. `None` if `buf` is shorter than the min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..MISSION_ITEM_INT_LEN)?;
        Some(Self {
            param1: f32::from_le_bytes(src.get(..4)?.try_into().ok()?),
            param2: f32::from_le_bytes(src.get(4..8)?.try_into().ok()?),
            param3: f32::from_le_bytes(src.get(8..12)?.try_into().ok()?),
            param4: f32::from_le_bytes(src.get(12..16)?.try_into().ok()?),
            x: i32::from_le_bytes(src.get(16..20)?.try_into().ok()?),
            y: i32::from_le_bytes(src.get(20..24)?.try_into().ok()?),
            z: f32::from_le_bytes(src.get(24..28)?.try_into().ok()?),
            seq: u16::from_le_bytes(src.get(28..30)?.try_into().ok()?),
            command: u16::from_le_bytes(src.get(30..32)?.try_into().ok()?),
            target_system: *src.get(32)?,
            target_component: *src.get(33)?,
            frame: *src.get(34)?,
            current: *src.get(35)?,
            autocontinue: *src.get(36)?,
            mission_type: *src.get(37)?,
        })
    }

    /// Decode a framed MISSION_ITEM_INT. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_MISSION_ITEM_INT {
            return None;
        }
        Self::decode(frame.payload())
    }
}

/// Small in-memory mission table, upstream `AP_Mission` item store.
///
/// `MISSION_ITEM_INT` writes by `seq`. `MISSION_REQUEST_INT` reads by `seq`
/// and the channel frames the stored item as `MISSION_ITEM_INT`.
#[derive(Debug, Clone)]
pub struct MissionTable {
    items: [Option<MissionItemInt>; MAX_MISSION_ITEMS],
    len: u16,
}

impl Default for MissionTable {
    fn default() -> Self {
        Self::new()
    }
}

impl MissionTable {
    /// Empty table.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            items: [None; MAX_MISSION_ITEMS],
            len: 0,
        }
    }

    /// Number of stored items.
    #[must_use]
    pub const fn count(&self) -> u16 {
        self.len
    }

    /// Look up by sequence number.
    #[must_use]
    pub fn get(&self, seq: u16) -> Option<&MissionItemInt> {
        let mut i = 0usize;
        while i < usize::from(self.len) {
            if let Some(item) = self.items.get(i).and_then(|slot| slot.as_ref()) {
                if item.seq == seq {
                    return Some(item);
                }
            }
            i = i.saturating_add(1);
        }
        None
    }

    /// Insert or replace by `seq`. `None` if the table is full and `seq` is new.
    pub fn set(&mut self, item: MissionItemInt) -> Option<u16> {
        let mut i = 0usize;
        while i < usize::from(self.len) {
            if let Some(slot) = self.items.get_mut(i) {
                if slot.as_ref().is_some_and(|stored| stored.seq == item.seq) {
                    *slot = Some(item);
                    return Some(item.seq);
                }
            }
            i = i.saturating_add(1);
        }
        let idx = self.len;
        let slot = self.items.get_mut(usize::from(idx))?;
        *slot = Some(item);
        self.len = idx.checked_add(1)?;
        Some(item.seq)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_rejects_short_buffer() {
        let req = MissionRequestInt::new(1, 1, 0, MAV_MISSION_TYPE_MISSION);
        assert!(req.encode(&mut [0u8; 2]).is_none());
        let item = MissionItemInt::waypoint(1, 1, 1, 0, 0, 100.0);
        assert!(item.encode(&mut [0u8; 8]).is_none());
    }

    #[test]
    fn decode_rejects_short_payload() {
        assert!(MissionRequestInt::decode(&[0, 1, 2]).is_none());
        assert!(MissionItemInt::decode(&[0, 1, 2]).is_none());
    }

    #[test]
    fn set_rejects_when_full() {
        let mut table = MissionTable::new();
        let mut seq = 0u16;
        while seq < MAX_MISSION_ITEMS as u16 {
            assert!(table
                .set(MissionItemInt::waypoint(1, 1, seq, 0, 0, 10.0))
                .is_some());
            seq = seq.saturating_add(1);
        }
        assert!(table
            .set(MissionItemInt::waypoint(1, 1, 99, 0, 0, 10.0))
            .is_none());
        assert_eq!(table.count(), MAX_MISSION_ITEMS as u16);
    }
}
