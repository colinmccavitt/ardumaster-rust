//! PARAM_REQUEST_LIST / PARAM_SET / PARAM_VALUE, extracted from the pinned
//! Plane-4.7.0 `modules/mavlink/message_definitions/v1.0/common.xml`.
//!
//! This slice is the GCS_MAVLink parameter microservice stub: list the
//! onboard table (`handle_param_request_list` + `queued_param_send`) then
//! write a named value (`handle_param_set`) and emit `PARAM_VALUE`. A full
//! dialect generator and `AP_Param` persistence are later FW-028 slices.

use crate::framing::Frame;

/// PARAM_REQUEST_LIST message id, upstream `MAVLINK_MSG_ID_PARAM_REQUEST_LIST`.
pub const MSG_ID_PARAM_REQUEST_LIST: u32 = 21;

/// PARAM_VALUE message id, upstream `MAVLINK_MSG_ID_PARAM_VALUE`.
pub const MSG_ID_PARAM_VALUE: u32 = 22;

/// PARAM_SET message id, upstream `MAVLINK_MSG_ID_PARAM_SET`.
pub const MSG_ID_PARAM_SET: u32 = 23;

/// Packed payload length, upstream `MAVLINK_MSG_ID_PARAM_REQUEST_LIST_LEN`.
pub const PARAM_REQUEST_LIST_LEN: usize = 2;

/// Packed payload length, upstream `MAVLINK_MSG_ID_PARAM_VALUE_LEN`.
pub const PARAM_VALUE_LEN: usize = 25;

/// Packed payload length, upstream `MAVLINK_MSG_ID_PARAM_SET_LEN`.
pub const PARAM_SET_LEN: usize = 23;

/// CRC extra, upstream `MAVLINK_MSG_ID_PARAM_REQUEST_LIST_CRC`.
pub const PARAM_REQUEST_LIST_CRC: u8 = 159;

/// CRC extra, upstream `MAVLINK_MSG_ID_PARAM_VALUE_CRC`.
pub const PARAM_VALUE_CRC: u8 = 220;

/// CRC extra, upstream `MAVLINK_MSG_ID_PARAM_SET_CRC`.
pub const PARAM_SET_CRC: u8 = 168;

/// `param_id` field width, upstream `MAVLINK_MSG_PARAM_*_FIELD_PARAM_ID_LEN`.
pub const PARAM_ID_LEN: usize = 16;

/// `MAV_PARAM_TYPE_REAL32` — the type this stub stores and emits.
pub const MAV_PARAM_TYPE_REAL32: u8 = 9;

/// Capacity of the in-memory table (not the full AP_Param tree).
pub const MAX_PARAMS: usize = 8;

/// Packed PARAM_REQUEST_LIST fields, upstream `mavlink_param_request_list_t`.
///
/// Wire order matches `mavlink_msg_param_request_list_pack`: `target_system`,
/// then `target_component`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParamRequestList {
    /// System ID.
    pub target_system: u8,
    /// Component ID.
    pub target_component: u8,
}

impl ParamRequestList {
    /// Build a PARAM_REQUEST_LIST from the XML field order.
    #[must_use]
    pub const fn new(target_system: u8, target_component: u8) -> Self {
        Self {
            target_system,
            target_component,
        }
    }

    /// Pack into 2 bytes. `None` if `buf` is too short.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..PARAM_REQUEST_LIST_LEN)?;
        *dest.get_mut(0)? = self.target_system;
        *dest.get_mut(1)? = self.target_component;
        Some(PARAM_REQUEST_LIST_LEN)
    }

    /// Unpack 2 bytes. `None` if `buf` is shorter than the min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..PARAM_REQUEST_LIST_LEN)?;
        Some(Self {
            target_system: *src.get(0)?,
            target_component: *src.get(1)?,
        })
    }

    /// Decode a framed PARAM_REQUEST_LIST. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_PARAM_REQUEST_LIST {
            return None;
        }
        Self::decode(frame.payload())
    }
}

/// Packed PARAM_SET fields, upstream `mavlink_param_set_t`.
///
/// Wire order is size-sorted (`mavlink_msg_param_set_pack`): `param_value`,
/// `target_system`, `target_component`, `param_id[16]`, `param_type`.
#[derive(Debug, Clone, Copy)]
pub struct ParamSet {
    /// Onboard parameter value.
    pub param_value: f32,
    /// System ID.
    pub target_system: u8,
    /// Component ID.
    pub target_component: u8,
    /// Onboard parameter id (16-byte MAVLink field).
    pub param_id: [u8; PARAM_ID_LEN],
    /// Onboard parameter type (`MAV_PARAM_TYPE`).
    pub param_type: u8,
}

impl ParamSet {
    /// Build a PARAM_SET from the XML field order (not wire order).
    #[must_use]
    pub fn new(
        target_system: u8,
        target_component: u8,
        param_id: &str,
        param_value: f32,
        param_type: u8,
    ) -> Self {
        Self {
            param_value,
            target_system,
            target_component,
            param_id: encode_param_id(param_id),
            param_type,
        }
    }

    /// Pack into 23 little-endian bytes. `None` if `buf` is too short.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..PARAM_SET_LEN)?;
        write_f32_le(dest, 0, self.param_value)?;
        *dest.get_mut(4)? = self.target_system;
        *dest.get_mut(5)? = self.target_component;
        dest.get_mut(6..22)?.copy_from_slice(&self.param_id);
        *dest.get_mut(22)? = self.param_type;
        Some(PARAM_SET_LEN)
    }

    /// Unpack 23 bytes. `None` if `buf` is shorter than the min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..PARAM_SET_LEN)?;
        let mut param_id = [0u8; PARAM_ID_LEN];
        param_id.copy_from_slice(src.get(6..22)?);
        Some(Self {
            param_value: read_f32_le(src, 0)?,
            target_system: *src.get(4)?,
            target_component: *src.get(5)?,
            param_id,
            param_type: *src.get(22)?,
        })
    }

    /// Decode a framed PARAM_SET. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_PARAM_SET {
            return None;
        }
        Self::decode(frame.payload())
    }

    /// `param_id` as a UTF-8 name, stopping at the first NUL (or 16 chars).
    #[must_use]
    pub fn name(&self) -> Option<&str> {
        param_id_name(&self.param_id)
    }
}

/// Packed PARAM_VALUE fields, upstream `mavlink_param_value_t`.
///
/// Wire order is size-sorted (`mavlink_msg_param_value_pack`): `param_value`,
/// `param_count`, `param_index`, `param_id[16]`, `param_type`.
#[derive(Debug, Clone, Copy)]
pub struct ParamValue {
    /// Onboard parameter value.
    pub param_value: f32,
    /// Total number of onboard parameters.
    pub param_count: u16,
    /// Index of this onboard parameter.
    pub param_index: u16,
    /// Onboard parameter id (16-byte MAVLink field).
    pub param_id: [u8; PARAM_ID_LEN],
    /// Onboard parameter type (`MAV_PARAM_TYPE`).
    pub param_type: u8,
}

impl ParamValue {
    /// Build a PARAM_VALUE from the XML field order (not wire order).
    #[must_use]
    pub fn new(
        param_id: &str,
        param_value: f32,
        param_type: u8,
        param_count: u16,
        param_index: u16,
    ) -> Self {
        Self {
            param_value,
            param_count,
            param_index,
            param_id: encode_param_id(param_id),
            param_type,
        }
    }

    /// Pack into 25 little-endian bytes. `None` if `buf` is too short.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..PARAM_VALUE_LEN)?;
        write_f32_le(dest, 0, self.param_value)?;
        write_u16_le(dest, 4, self.param_count)?;
        write_u16_le(dest, 6, self.param_index)?;
        dest.get_mut(8..24)?.copy_from_slice(&self.param_id);
        *dest.get_mut(24)? = self.param_type;
        Some(PARAM_VALUE_LEN)
    }

    /// Unpack 25 bytes. `None` if `buf` is shorter than the min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..PARAM_VALUE_LEN)?;
        let mut param_id = [0u8; PARAM_ID_LEN];
        param_id.copy_from_slice(src.get(8..24)?);
        Some(Self {
            param_value: read_f32_le(src, 0)?,
            param_count: read_u16_le(src, 4)?,
            param_index: read_u16_le(src, 6)?,
            param_id,
            param_type: *src.get(24)?,
        })
    }

    /// Decode a framed PARAM_VALUE. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_PARAM_VALUE {
            return None;
        }
        Self::decode(frame.payload())
    }

    /// `param_id` as a UTF-8 name, stopping at the first NUL (or 16 chars).
    #[must_use]
    pub fn name(&self) -> Option<&str> {
        param_id_name(&self.param_id)
    }
}

/// One scalar in the in-memory table, standing in for `AP_Param`.
#[derive(Debug, Clone, Copy)]
pub struct ParamEntry {
    /// Onboard name (16-byte MAVLink `param_id`).
    pub id: [u8; PARAM_ID_LEN],
    /// Current value.
    pub value: f32,
    /// `MAV_PARAM_TYPE` emitted on the wire.
    pub param_type: u8,
}

impl ParamEntry {
    /// `id` as a UTF-8 name, stopping at the first NUL (or 16 chars).
    #[must_use]
    pub fn name(&self) -> Option<&str> {
        param_id_name(&self.id)
    }
}

/// Small in-memory parameter table, upstream `AP_Param` scalar walk.
///
/// `PARAM_REQUEST_LIST` starts at index 0; `queued_param_send` walks
/// [`Self::next_queued`]. `PARAM_SET` writes by name (`AP_Param::find`).
#[derive(Debug, Clone)]
pub struct ParamTable {
    entries: [ParamEntry; MAX_PARAMS],
    len: u16,
    queued_index: u16,
}

impl Default for ParamTable {
    fn default() -> Self {
        Self::plane_stub()
    }
}

impl ParamTable {
    /// Three Plane-shaped scalars used by the list-then-set stub.
    ///
    /// Names match common.xml / Plane defaults: `SYSID_THISMAV`,
    /// `ARSPD_ENABLE`, `TRIM_PITCH`. Persistence is later.
    #[must_use]
    pub fn plane_stub() -> Self {
        let mut table = Self {
            entries: [ParamEntry {
                id: [0; PARAM_ID_LEN],
                value: 0.0,
                param_type: MAV_PARAM_TYPE_REAL32,
            }; MAX_PARAMS],
            len: 0,
            queued_index: 0,
        };
        let _ = table.insert("SYSID_THISMAV", 1.0, MAV_PARAM_TYPE_REAL32);
        let _ = table.insert("ARSPD_ENABLE", 0.0, MAV_PARAM_TYPE_REAL32);
        let _ = table.insert("TRIM_PITCH", 0.0, MAV_PARAM_TYPE_REAL32);
        table
    }

    /// Number of scalars, upstream `AP_Param::count_parameters`.
    #[must_use]
    pub const fn count(&self) -> u16 {
        self.len
    }

    /// Append a named scalar. `None` if the table is full or the name is empty.
    pub fn insert(&mut self, name: &str, value: f32, param_type: u8) -> Option<u16> {
        if name.is_empty() {
            return None;
        }
        let idx = self.len;
        let slot = self.entries.get_mut(usize::from(idx))?;
        *slot = ParamEntry {
            id: encode_param_id(name),
            value,
            param_type,
        };
        self.len = idx.checked_add(1)?;
        Some(idx)
    }

    /// Look up by index.
    #[must_use]
    pub fn get(&self, index: u16) -> Option<&ParamEntry> {
        if index >= self.len {
            return None;
        }
        self.entries.get(usize::from(index))
    }

    /// Look up by MAVLink `param_id`, matching `AP_Param::find`.
    #[must_use]
    pub fn find(&self, param_id: &[u8; PARAM_ID_LEN]) -> Option<(u16, &ParamEntry)> {
        let mut i = 0u16;
        while i < self.len {
            if let Some(entry) = self.entries.get(usize::from(i)) {
                if param_id_eq(&entry.id, param_id) {
                    return Some((i, entry));
                }
            }
            i = i.saturating_add(1);
        }
        None
    }

    /// Write a named value. `None` if missing, NaN, or Inf (upstream reject).
    pub fn set(&mut self, param_id: &[u8; PARAM_ID_LEN], value: f32) -> Option<(u16, ParamEntry)> {
        if !value.is_finite() {
            return None;
        }
        let (index, _) = self.find(param_id)?;
        let slot = self.entries.get_mut(usize::from(index))?;
        slot.value = value;
        Some((index, *slot))
    }

    /// Start a list walk, upstream `handle_param_request_list`.
    pub fn start_list(&mut self) {
        self.queued_index = 0;
    }

    /// Next queued `PARAM_VALUE`, upstream `queued_param_send` one step.
    pub fn next_queued(&mut self) -> Option<ParamValue> {
        let index = self.queued_index;
        let entry = *self.get(index)?;
        self.queued_index = index.saturating_add(1);
        Some(ParamValue {
            param_value: entry.value,
            param_count: self.len,
            param_index: index,
            param_id: entry.id,
            param_type: entry.param_type,
        })
    }

    /// Build a `PARAM_VALUE` for an already-found entry (SET ack).
    #[must_use]
    pub fn value_at(&self, index: u16) -> Option<ParamValue> {
        let entry = *self.get(index)?;
        Some(ParamValue {
            param_value: entry.value,
            param_count: self.len,
            param_index: index,
            param_id: entry.id,
            param_type: entry.param_type,
        })
    }
}

/// Pack a name into the 16-byte MAVLink `param_id` field.
#[must_use]
pub fn encode_param_id(name: &str) -> [u8; PARAM_ID_LEN] {
    let mut id = [0u8; PARAM_ID_LEN];
    let bytes = name.as_bytes();
    let n = core::cmp::min(bytes.len(), PARAM_ID_LEN);
    if let (Some(src), Some(dest)) = (bytes.get(..n), id.get_mut(..n)) {
        dest.copy_from_slice(src);
    }
    id
}

/// UTF-8 view of a `param_id`, stopping at the first NUL (or 16 chars).
#[must_use]
pub fn param_id_name(id: &[u8; PARAM_ID_LEN]) -> Option<&str> {
    let end = id.iter().position(|&b| b == 0).unwrap_or(PARAM_ID_LEN);
    let bytes = id.get(..end)?;
    core::str::from_utf8(bytes).ok()
}

fn param_id_eq(a: &[u8; PARAM_ID_LEN], b: &[u8; PARAM_ID_LEN]) -> bool {
    match (param_id_name(a), param_id_name(b)) {
        (Some(left), Some(right)) => left == right,
        _ => a == b,
    }
}

fn write_f32_le(buf: &mut [u8], off: usize, value: f32) -> Option<()> {
    let end = off.checked_add(4)?;
    buf.get_mut(off..end)?.copy_from_slice(&value.to_le_bytes());
    Some(())
}

fn read_f32_le(buf: &[u8], off: usize) -> Option<f32> {
    let end = off.checked_add(4)?;
    let bytes = buf.get(off..end)?;
    let mut le = [0u8; 4];
    le.copy_from_slice(bytes);
    Some(f32::from_le_bytes(le))
}

fn write_u16_le(buf: &mut [u8], off: usize, value: u16) -> Option<()> {
    let end = off.checked_add(2)?;
    buf.get_mut(off..end)?.copy_from_slice(&value.to_le_bytes());
    Some(())
}

fn read_u16_le(buf: &[u8], off: usize) -> Option<u16> {
    let end = off.checked_add(2)?;
    let bytes = buf.get(off..end)?;
    let mut le = [0u8; 2];
    le.copy_from_slice(bytes);
    Some(u16::from_le_bytes(le))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_rejects_short_buffer() {
        let list = ParamRequestList::new(1, 1);
        assert!(list.encode(&mut [0u8; 1]).is_none());
        let set = ParamSet::new(1, 1, "ARSPD_ENABLE", 1.0, MAV_PARAM_TYPE_REAL32);
        assert!(set.encode(&mut [0u8; 8]).is_none());
        let value = ParamValue::new("ARSPD_ENABLE", 1.0, MAV_PARAM_TYPE_REAL32, 3, 1);
        assert!(value.encode(&mut [0u8; 8]).is_none());
    }

    #[test]
    fn decode_rejects_short_payload() {
        assert!(ParamRequestList::decode(&[0]).is_none());
        assert!(ParamSet::decode(&[0, 1, 2]).is_none());
        assert!(ParamValue::decode(&[0, 1, 2]).is_none());
    }

    #[test]
    fn set_rejects_nan_and_unknown_name() {
        let mut table = ParamTable::plane_stub();
        let missing = encode_param_id("NO_SUCH_PARAM");
        assert!(table.set(&missing, 1.0).is_none());
        let known = encode_param_id("ARSPD_ENABLE");
        assert!(table.set(&known, f32::NAN).is_none());
        assert!(table.set(&known, f32::INFINITY).is_none());
    }
}
