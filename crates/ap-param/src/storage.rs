//! Reading ArduPilot's parameter storage (FW-004 slice 3, ADR-0010).
//!
//! Storage is an append-only log, not a table. Each entry is a four-byte
//! header followed by the value's native bytes, and the list ends at a
//! sentinel. Saving a parameter that is already present overwrites its value in
//! place; saving a new one appends at the sentinel and moves it along.
//!
//! Only parameters that differ from their default are stored at all, so a
//! vehicle's storage is far smaller than its parameter list.
//!
//! # Reading needs no object graph
//!
//! Upstream reaches values through type-erased pointers into the vehicle's
//! objects. Decoding storage does not need any of that: an entry carries its
//! own key, group element and type, so this layer produces plain values and
//! leaves matching them to variables for the caller.
//!
//! # Order of writes
//!
//! Upstream appends by writing the *new sentinel first*, then the value, then
//! the header. Losing power partway leaves either the old list intact or a
//! trailing entry whose header was never written — and an unwritten header
//! reads as the erase pattern, which [`ParamHeader::is_sentinel`] already
//! treats as a terminator. The order is not incidental and is preserved when
//! the write path lands.

use crate::{ParamHeader, VarType, EEPROM_HEADER_SIZE, PARAM_HEADER_SIZE};

/// Backing store for parameters, upstream `StorageAccess`.
///
/// Offsets are relative to the start of the parameter area, so an
/// implementation that shares a device with other data adds its own base.
pub trait Storage {
    /// Bytes available to parameters.
    fn size(&self) -> u16;
    /// Read exactly `buf.len()` bytes, or return false and leave `buf` alone.
    fn read(&self, offset: u16, buf: &mut [u8]) -> bool;
    /// Write exactly `data.len()` bytes, or return false having written none.
    fn write(&mut self, offset: u16, data: &[u8]) -> bool;
}

/// A decoded parameter value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ParamValue {
    /// Upstream `AP_PARAM_INT8`.
    Int8(i8),
    /// Upstream `AP_PARAM_INT16`.
    Int16(i16),
    /// Upstream `AP_PARAM_INT32`.
    Int32(i32),
    /// Upstream `AP_PARAM_FLOAT`.
    Float(f32),
    /// Upstream `AP_PARAM_VECTOR3F`, stored as three consecutive floats.
    Vector3f([f32; 3]),
}

impl ParamValue {
    /// The type tag this value stores under.
    #[must_use]
    pub const fn var_type(self) -> VarType {
        match self {
            Self::Int8(_) => VarType::Int8,
            Self::Int16(_) => VarType::Int16,
            Self::Int32(_) => VarType::Int32,
            Self::Float(_) => VarType::Float,
            Self::Vector3f(_) => VarType::Vector3f,
        }
    }

    /// The value as a float, upstream `cast_to_float`.
    ///
    /// A `Vector3f` has no single float; upstream's equivalent is only ever
    /// called on scalars, so this reports the X component and callers that
    /// care about the distinction should match on the variant instead.
    #[must_use]
    pub fn as_f32(self) -> f32 {
        match self {
            Self::Int8(v) => f32::from(v),
            Self::Int16(v) => f32::from(v),
            #[allow(
                clippy::cast_precision_loss,
                reason = "upstream's cast_to_float does the same; an int32 \
parameter beyond 2^24 cannot round-trip through a float either way"
            )]
            Self::Int32(v) => v as f32,
            Self::Float(v) => v,
            Self::Vector3f(v) => v[0],
        }
    }

    /// One component of a value, by the token index the enumeration uses.
    ///
    /// Index 0 is the value itself; 1, 2 and 3 are a vector's components.
    #[must_use]
    pub fn component(self, token_idx: u8) -> Option<f32> {
        match (self, token_idx) {
            (Self::Vector3f(v), 1..=3) => v.get(token_idx as usize - 1).copied(),
            (_, 0) => Some(self.as_f32()),
            _ => None,
        }
    }
}

/// Why storage could not be read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StorageError {
    /// The first two bytes are not `AP`. Upstream erases and reformats.
    BadMagic,
    /// The format revision is not the one this build writes.
    BadRevision {
        /// What was found.
        found: u8,
        /// What was expected.
        expected: u8,
    },
    /// The backing store refused a read.
    ReadFailed {
        /// Where.
        offset: u16,
    },
    /// The backing store refused a write. Storage may be left with a record
    /// half appended -- which is invisible to a reader, by design; see the
    /// ordering in `save`.
    WriteFailed {
        /// Where.
        offset: u16,
    },
}

/// One entry as it appears in storage.
#[derive(Debug, Clone, Copy)]
pub struct StoredParam {
    /// Where the header starts.
    pub offset: u16,
    /// The header, decoded.
    pub header: ParamHeader,
    /// The value, or `None` when the type tag is not one this build knows.
    ///
    /// Upstream keeps walking past such an entry rather than stopping, because
    /// `type_size` reports zero for an unknown tag — so the walk advances by
    /// the header alone. Reproduced, since stopping would discard every
    /// parameter written after one an older build does not recognise.
    pub value: Option<ParamValue>,
}

/// Walks the entries in storage, in the order they were written.
pub struct StorageIter<'s, S: Storage + ?Sized> {
    storage: &'s S,
    offset: u16,
    done: bool,
}

/// Start reading a store, checking the header first.
///
/// # Errors
///
/// Returns [`StorageError`] if the magic or revision is not this format's, or
/// if the store refuses the read.
pub fn read<S: Storage + ?Sized>(storage: &S) -> Result<StorageIter<'_, S>, StorageError> {
    let mut buf = [0u8; EEPROM_HEADER_SIZE];
    if !storage.read(0, &mut buf) {
        return Err(StorageError::ReadFailed { offset: 0 });
    }
    let header = crate::EepromHeader::from_bytes(buf);
    if header.magic != crate::EEPROM_MAGIC {
        return Err(StorageError::BadMagic);
    }
    if header.revision != crate::EEPROM_REVISION {
        return Err(StorageError::BadRevision {
            found: header.revision,
            expected: crate::EEPROM_REVISION,
        });
    }
    Ok(StorageIter {
        storage,
        #[allow(
            clippy::cast_possible_truncation,
            reason = "the header is four bytes, far inside u16"
        )]
        offset: EEPROM_HEADER_SIZE as u16,
        done: false,
    })
}

fn decode_value<S: Storage + ?Sized>(
    storage: &S,
    offset: u16,
    ptype: u8,
) -> Option<(ParamValue, u8)> {
    let ty = VarType::from_u8(ptype)?;
    let size = ty.size();
    let mut buf = [0u8; 12];
    let slice = buf.get_mut(..size as usize)?;
    if !storage.read(offset, slice) {
        return None;
    }
    let v = match ty {
        VarType::Int8 => ParamValue::Int8(*buf.first()? as i8),
        VarType::Int16 => ParamValue::Int16(i16::from_le_bytes([*buf.first()?, *buf.get(1)?])),
        VarType::Int32 => ParamValue::Int32(i32::from_le_bytes([
            *buf.first()?,
            *buf.get(1)?,
            *buf.get(2)?,
            *buf.get(3)?,
        ])),
        VarType::Float => ParamValue::Float(f32::from_le_bytes([
            *buf.first()?,
            *buf.get(1)?,
            *buf.get(2)?,
            *buf.get(3)?,
        ])),
        VarType::Vector3f => {
            let mut v = [0f32; 3];
            for (i, out) in v.iter_mut().enumerate() {
                let b = buf.get(i * 4..i * 4 + 4)?;
                *out = f32::from_le_bytes([*b.first()?, *b.get(1)?, *b.get(2)?, *b.get(3)?]);
            }
            ParamValue::Vector3f(v)
        }
        // Neither carries a value; both occupy no bytes.
        VarType::None | VarType::Group => return None,
    };
    Some((v, size))
}

impl<S: Storage + ?Sized> Iterator for StorageIter<'_, S> {
    type Item = StoredParam;

    fn next(&mut self) -> Option<Self::Item> {
        if self.done {
            return None;
        }
        #[allow(
            clippy::cast_possible_truncation,
            reason = "the header is four bytes, far inside u16"
        )]
        let header_size = PARAM_HEADER_SIZE as u16;

        if self.offset.checked_add(header_size)? > self.storage.size() {
            self.done = true;
            return None;
        }
        let mut buf = [0u8; PARAM_HEADER_SIZE];
        if !self.storage.read(self.offset, &mut buf) {
            self.done = true;
            return None;
        }
        let header = ParamHeader::from_bytes(buf);
        if header.is_sentinel() {
            self.done = true;
            return None;
        }

        let offset = self.offset;
        let decoded = decode_value(self.storage, offset + header_size, header.var_type);
        // An unrecognised tag has no size, so the walk steps over the header
        // alone -- exactly what upstream's `type_size` returning zero makes it
        // do. The entry is still yielded, with no value, rather than skipped:
        // a reader that stopped there would lose every parameter written after
        // one an older build does not recognise.
        let size = decoded.map_or(0, |(_, s)| s);
        self.offset = offset + header_size + u16::from(size);

        Some(StoredParam {
            offset,
            header,
            value: decoded.map(|(v, _)| v),
        })
    }
}
