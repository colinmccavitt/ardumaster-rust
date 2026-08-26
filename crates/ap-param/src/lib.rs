//! Port of `AP_Param`'s on-storage format and conversion. Tracked as FW-004.
//!
//! ADR-0010 requires byte compatibility with ArduPilot's parameter storage, so
//! this layer is not free to be tidy: it reproduces upstream's encoding
//! exactly, including the parts nobody would choose fresh.
//!
//! # Storage layout
//!
//! ```text
//! offset 0   EEPROM header    magic[2] = "AP", revision, spare
//! offset 4   parameter        header (4 bytes) then the value's native bytes
//!            ...
//!            sentinel         a header with key 0x1FF and type 0x1F
//! ```
//!
//! # The header, and why the key is split
//!
//! ```text
//!   bit  0..7   key_low
//!   bit  8..12  type
//!   bit 13      key_high
//!   bit 14..31  group_element
//! ```
//!
//! The key is nine bits stored as eight plus one, with the type wedged between
//! the halves. Upstream's comment says why: "to get 9 bits for key we needed to
//! split it into two parts to keep binary compatibility". The key grew past 256
//! after the format was already in the field, and the only free bit was on the
//! far side of the type.
//!
//! Bitfield allocation order is implementation-defined, so this layout is not
//! something the C++ can be read off. It is measured: `tools/parity/gen_param_fixture.py`
//! builds headers with upstream's own compiled `set_key`, dumps the raw words,
//! and `param_format_parity` requires the port to reproduce all 640 of them.
//!
//! # No unsafe reinterpretation
//!
//! Upstream's `is_sentinel` reads the header through `*(uint32_t *)&phdr`,
//! which is a strict-aliasing violation that happens to work. Here the word
//! *is* the representation and the fields are derived from it, so the same
//! behaviour needs no cast.

#![no_std]

pub mod conversion;
pub mod info;
pub mod save;
pub mod storage;

pub use conversion::{
    configured_in_storage, convert_class, convert_class_entry, convert_parameter_width,
    convert_scalar, eeprom_header_valid, find_old_parameter, format_storage, migrate_scalar,
    old_group_element_for_member, scalar_from_f32, ConversionInfo, ConvertClassStats,
    ConvertFlags, ConvertOutcome, GroupMemberDescriptor,
};
pub use save::{save, scan, write_sentinel, SaveOutcome, ScanResult};
pub use storage::{read, ParamValue, Storage, StorageError, StorageIter, StoredParam};

pub use info::{
    check_frame_type, enumerate, group_id, EnumFilter, GroupInfo, ParamInfo, ParamName, ParamRef,
    FLAG_DEFAULT_POINTER, FLAG_ENABLE, FLAG_HIDDEN, FLAG_INFO_POINTER, FLAG_INTERNAL_USE_ONLY,
    FLAG_NESTED_OFFSET, FLAG_NO_SHIFT, FLAG_POINTER, MAX_NAME_SIZE,
};

/// Parameter type tags, upstream `enum ap_var_type`.
///
/// The discriminants are part of the storage format and appear in the header's
/// five type bits, so they are pinned rather than incidental.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum VarType {
    /// Upstream `AP_PARAM_NONE`.
    None = 0,
    /// Upstream `AP_PARAM_INT8`.
    Int8 = 1,
    /// Upstream `AP_PARAM_INT16`.
    Int16 = 2,
    /// Upstream `AP_PARAM_INT32`.
    Int32 = 3,
    /// Upstream `AP_PARAM_FLOAT`.
    Float = 4,
    /// Upstream `AP_PARAM_VECTOR3F`.
    Vector3f = 5,
    /// A nested group rather than a value, upstream `AP_PARAM_GROUP`.
    Group = 6,
}

impl VarType {
    /// The tag as stored in the header's type field.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Recover a type from its stored tag, or `None` for a tag the format does
    /// not define — including the sentinel's `0x1F`.
    #[must_use]
    pub const fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::None,
            1 => Self::Int8,
            2 => Self::Int16,
            3 => Self::Int32,
            4 => Self::Float,
            5 => Self::Vector3f,
            6 => Self::Group,
            _ => return None,
        })
    }

    /// Bytes the value occupies in storage, upstream `type_size`.
    ///
    /// `None` and `Group` occupy nothing: neither has a value of its own, a
    /// group being a container for the parameters below it.
    #[must_use]
    pub const fn size(self) -> u8 {
        match self {
            Self::None | Self::Group => 0,
            Self::Int8 => 1,
            Self::Int16 => 2,
            Self::Int32 | Self::Float => 4,
            Self::Vector3f => 3 * 4,
        }
    }
}

/// Storage magic, upstream `k_EEPROM_magic0`/`k_EEPROM_magic1` — "AP".
pub const EEPROM_MAGIC: [u8; 2] = [0x50, 0x41];

/// Format revision, upstream `k_EEPROM_revision`.
pub const EEPROM_REVISION: u8 = 6;

/// Bytes at the start of storage before the first parameter.
pub const EEPROM_HEADER_SIZE: usize = 4;

/// Bytes in a parameter header.
pub const PARAM_HEADER_SIZE: usize = 4;

/// Key marking the end of the parameter list, upstream `_sentinel_key`.
pub const SENTINEL_KEY: u16 = 0x1FF;

/// Type marking the end of the parameter list, upstream `_sentinel_type`.
pub const SENTINEL_TYPE: u8 = 0x1F;

/// Group element written into a sentinel, upstream `_sentinel_group`.
///
/// Note it is `0xFF`, not all eighteen bits set — the sentinel is recognised by
/// its key and type, so the group element is only conventional.
pub const SENTINEL_GROUP: u32 = 0xFF;

/// Bits per level of group nesting, upstream `_group_level_shift`.
pub const GROUP_LEVEL_SHIFT: u8 = 6;

/// Bits available to the whole group element, upstream `_group_bits`.
pub const GROUP_BITS: u8 = 18;

/// The four bytes at the start of storage, upstream `EEPROM_header`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EepromHeader {
    /// Should equal [`EEPROM_MAGIC`].
    pub magic: [u8; 2],
    /// Should equal [`EEPROM_REVISION`].
    pub revision: u8,
    /// Unused, written as zero.
    pub spare: u8,
}

impl Default for EepromHeader {
    /// The header written to freshly formatted storage.
    fn default() -> Self {
        Self {
            magic: EEPROM_MAGIC,
            revision: EEPROM_REVISION,
            spare: 0,
        }
    }
}

impl EepromHeader {
    /// The header's bytes, in storage order.
    #[must_use]
    pub const fn to_bytes(self) -> [u8; EEPROM_HEADER_SIZE] {
        [self.magic[0], self.magic[1], self.revision, self.spare]
    }

    /// Read a header from storage.
    #[must_use]
    pub const fn from_bytes(b: [u8; EEPROM_HEADER_SIZE]) -> Self {
        Self {
            magic: [b[0], b[1]],
            revision: b[2],
            spare: b[3],
        }
    }

    /// Whether this is storage the port can read.
    ///
    /// Upstream erases and reformats when this fails, on the reasoning that
    /// storage which is not ours is not worth trying to interpret.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.magic == EEPROM_MAGIC && self.revision == EEPROM_REVISION
    }
}

/// A parameter's header, upstream `Param_header`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParamHeader {
    /// Nine-bit key identifying the top-level variable.
    pub key: u16,
    /// Five-bit type tag. Kept as a raw tag rather than a [`VarType`] because
    /// the sentinel's `0x1F` is not a type, and storage written by a newer
    /// firmware could carry a tag this build does not know.
    pub var_type: u8,
    /// Eighteen-bit path through the nested groups.
    pub group_element: u32,
}

/// Bit positions, from the measured layout. Changing any of these breaks
/// compatibility with every vehicle in the field.
const KEY_LOW_BITS: u32 = 8;
const TYPE_SHIFT: u32 = 8;
const TYPE_BITS: u32 = 5;
const KEY_HIGH_SHIFT: u32 = 13;
const GROUP_SHIFT: u32 = 14;

impl ParamHeader {
    /// Build a header, discarding bits that do not fit their fields.
    ///
    /// Truncation rather than rejection, because that is what assigning to a
    /// C bitfield does and the storage format has to match.
    #[must_use]
    pub const fn new(key: u16, var_type: u8, group_element: u32) -> Self {
        Self {
            key: key & 0x1FF,
            var_type: var_type & 0x1F,
            group_element: group_element & ((1 << GROUP_BITS) - 1),
        }
    }

    /// The header as the 32-bit word stored on disk.
    #[must_use]
    pub const fn to_word(self) -> u32 {
        let key = self.key as u32;
        let key_low = key & ((1 << KEY_LOW_BITS) - 1);
        let key_high = (key >> KEY_LOW_BITS) & 1;
        let var_type = (self.var_type as u32) & ((1 << TYPE_BITS) - 1);
        let group = self.group_element & ((1 << GROUP_BITS) - 1);

        key_low | (var_type << TYPE_SHIFT) | (key_high << KEY_HIGH_SHIFT) | (group << GROUP_SHIFT)
    }

    /// Decode a header from its stored word.
    #[must_use]
    pub const fn from_word(w: u32) -> Self {
        let key_low = w & ((1 << KEY_LOW_BITS) - 1);
        let key_high = (w >> KEY_HIGH_SHIFT) & 1;
        Self {
            key: ((key_high << KEY_LOW_BITS) | key_low) as u16,
            var_type: ((w >> TYPE_SHIFT) & ((1 << TYPE_BITS) - 1)) as u8,
            group_element: (w >> GROUP_SHIFT) & ((1 << GROUP_BITS) - 1),
        }
    }

    /// The header's bytes, in storage order.
    #[must_use]
    pub const fn to_bytes(self) -> [u8; PARAM_HEADER_SIZE] {
        self.to_word().to_le_bytes()
    }

    /// Read a header from storage.
    #[must_use]
    pub const fn from_bytes(b: [u8; PARAM_HEADER_SIZE]) -> Self {
        Self::from_word(u32::from_le_bytes(b))
    }

    /// The sentinel that terminates the parameter list.
    #[must_use]
    pub const fn sentinel() -> Self {
        Self {
            key: SENTINEL_KEY,
            var_type: SENTINEL_TYPE,
            group_element: SENTINEL_GROUP,
        }
    }

    /// Whether this header ends the list, upstream `is_sentinel`.
    ///
    /// The key and type are tested with `||` rather than `&&`, and upstream
    /// says why: it makes the reader robust against losing power partway
    /// through appending a variable, when only one of the two has been
    /// overwritten.
    ///
    /// The all-zeroes and all-ones words are also treated as terminators.
    /// Those are what erased and uninitialised storage read as, so a
    /// half-finished write leaves something the reader stops at rather than
    /// something it misinterprets as a parameter.
    #[must_use]
    pub const fn is_sentinel(self) -> bool {
        if self.var_type == SENTINEL_TYPE || self.key == SENTINEL_KEY {
            return true;
        }
        let w = self.to_word();
        w == 0 || w == 0xFFFF_FFFF
    }
}
