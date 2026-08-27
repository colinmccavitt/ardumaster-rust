//! Parameter conversion and first-boot storage setup, upstream
//! `AP_Param::convert_*` and `setup`. FW-004 slice 4.
//!
//! The vehicle object graph and name lookup arrive in a later slice; here are
//! the storage-facing primitives those callers will use.

use ap_math::scalar::{constrain_value, is_equal};

use crate::save::{save, scan, write_sentinel, SaveOutcome, ScanResult};
use crate::storage::{ParamValue, Storage, StorageError};
use crate::{
    EepromHeader, ParamHeader, VarType, EEPROM_HEADER_SIZE, EEPROM_MAGIC, EEPROM_REVISION,
    PARAM_HEADER_SIZE,
};

/// Old parameter location for a rename or regroup, upstream `ConversionInfo`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConversionInfo {
    /// Top-level key the value was stored under.
    pub old_key: u16,
    /// Group path in the old layout.
    pub old_group_element: u32,
    /// Stored type of the old value.
    pub old_type: VarType,
}

/// Flags for [`convert_scalar`], upstream `CONVERT_FLAG_*`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConvertFlags(u8);

impl ConvertFlags {
    /// Convert `_REV` (-1/0) to `_REVERSED` (1/0).
    pub const REVERSE: Self = Self(1);
    /// Write even when the destination is already configured.
    pub const FORCE: Self = Self(2);
    /// No conversion flags set.
    pub const NONE: Self = Self(0);

    /// Whether `flag` is set.
    #[must_use]
    pub const fn contains(self, flag: Self) -> bool {
        self.0 & flag.0 != 0
    }
}

/// Whether storage has a readable header, upstream `AP_Param::setup`.
#[must_use]
pub fn eeprom_header_valid<S: Storage + ?Sized>(storage: &S) -> bool {
    let mut buf = [0u8; EEPROM_HEADER_SIZE];
    if !storage.read(0, &mut buf) {
        return false;
    }
    EepromHeader::from_bytes(buf).is_valid()
}

/// Wipe storage to a fresh header and sentinel, upstream `erase_all`.
///
/// Upstream erases the whole backing store first; callers with a typed
/// [`Storage`] implementation should zero-fill before calling if they need
/// bit-identical behaviour.
pub fn format_storage<S: Storage + ?Sized>(storage: &mut S) -> Result<(), StorageError> {
    let hdr = EepromHeader::default().to_bytes();
    if !storage.write(0, &hdr) {
        return Err(StorageError::WriteFailed { offset: 0 });
    }
    write_sentinel(storage, EEPROM_HEADER_SIZE as u16)
}

/// Load an old parameter from storage, upstream `find_old_parameter`.
pub fn find_old_parameter<S: Storage + ?Sized>(
    storage: &S,
    info: ConversionInfo,
) -> Option<(ParamValue, u16)> {
    let header = ParamHeader::new(info.old_key, info.old_type.as_u8(), info.old_group_element);
    let ScanResult::Found(ofs) = scan(storage, header) else {
        return None;
    };
    let mut buf = [0u8; 12];
    let size = info.old_type.size() as usize;
    let at = ofs + PARAM_HEADER_SIZE as u16;
    if !storage.read(at, &mut buf[..size]) {
        return None;
    }
    decode_stored(info.old_type, &buf[..size]).map(|v| (v, ofs))
}

fn decode_stored(ty: VarType, bytes: &[u8]) -> Option<ParamValue> {
    Some(match ty {
        VarType::Int8 => ParamValue::Int8(*bytes.first()? as i8),
        VarType::Int16 => ParamValue::Int16(i16::from_le_bytes([*bytes.first()?, *bytes.get(1)?])),
        VarType::Int32 => ParamValue::Int32(i32::from_le_bytes([
            *bytes.first()?,
            *bytes.get(1)?,
            *bytes.get(2)?,
            *bytes.get(3)?,
        ])),
        VarType::Float => ParamValue::Float(f32::from_le_bytes([
            *bytes.first()?,
            *bytes.get(1)?,
            *bytes.get(2)?,
            *bytes.get(3)?,
        ])),
        VarType::Vector3f => {
            let mut v = [0f32; 3];
            for (i, out) in v.iter_mut().enumerate() {
                let b = bytes.get(i * 4..i * 4 + 4)?;
                *out = f32::from_le_bytes([*b.first()?, *b.get(1)?, *b.get(2)?, *b.get(3)?]);
            }
            ParamValue::Vector3f(v)
        }
        VarType::None | VarType::Group => return None,
    })
}

/// Scalar types that participate in conversion, upstream `<= AP_PARAM_FLOAT`.
#[must_use]
pub const fn is_scalar_type(ty: VarType) -> bool {
    matches!(
        ty,
        VarType::Int8 | VarType::Int16 | VarType::Int32 | VarType::Float
    )
}

/// Build a scalar value from a float, upstream `AP_Param::set_float`.
#[must_use]
pub fn scalar_from_f32(ty: VarType, value: f32) -> Option<ParamValue> {
    if !value.is_finite() {
        return None;
    }
    let mut rounding = 0.01_f32;
    Some(match ty {
        VarType::Float => ParamValue::Float(value),
        VarType::Int32 => {
            if value < 0.0 {
                rounding = -rounding;
            }
            let v = constrain_value(value + rounding, i32::MIN as f32, i32::MAX as f32);
            ParamValue::Int32(v as i32)
        }
        VarType::Int16 => {
            if value < 0.0 {
                rounding = -rounding;
            }
            let v = constrain_value(value + rounding, i16::MIN as f32, i16::MAX as f32);
            ParamValue::Int16(v as i16)
        }
        VarType::Int8 => {
            if value < 0.0 {
                rounding = -rounding;
            }
            let v = constrain_value(value + rounding, i8::MIN as f32, i8::MAX as f32);
            ParamValue::Int8(v as i8)
        }
        _ => return None,
    })
}

/// Convert one stored scalar to another, upstream `convert_old_parameter`'s
/// scalar branch.
#[must_use]
pub fn convert_scalar(
    old: ParamValue,
    old_type: VarType,
    new_type: VarType,
    scaler: f32,
    flags: ConvertFlags,
) -> Option<ParamValue> {
    if !is_scalar_type(old_type) || !is_scalar_type(new_type) {
        return None;
    }
    if old_type == new_type && is_equal(scaler, 1.0) && flags.0 == 0 {
        return Some(old);
    }
    let mut v = old.as_f32();
    if flags.contains(ConvertFlags::REVERSE) {
        v = if is_equal(v, -1.0) { 1.0 } else { 0.0 };
    }
    scalar_from_f32(new_type, v * scaler)
}

/// Widen a stored scalar in place, upstream `_convert_parameter_width`.
///
/// Returns the converted value when storage holds `old_type` at `header` and
/// the live type is `new_type`.
pub fn convert_parameter_width<S: Storage + ?Sized>(
    storage: &S,
    header: ParamHeader,
    old_type: VarType,
    new_type: VarType,
    scale_factor: f32,
    bitmask: bool,
) -> Option<ParamValue> {
    if !is_scalar_type(old_type) || !is_scalar_type(new_type) {
        return None;
    }
    let probe = ParamHeader::new(header.key, old_type.as_u8(), header.group_element);
    let ScanResult::Found(ofs) = scan(storage, probe) else {
        return None;
    };
    let mut buf = [0u8; 4];
    let size = old_type.size() as usize;
    let at = ofs + PARAM_HEADER_SIZE as u16;
    if !storage.read(at, &mut buf[..size]) {
        return None;
    }
    let old = decode_stored(old_type, &buf[..size])?;

    if bitmask {
        let mask = match old {
            ParamValue::Int8(v) => u32::from(v as u8),
            ParamValue::Int16(v) => u32::from(v as u16),
            ParamValue::Int32(v) => v as u32,
            _ => return None,
        };
        scalar_from_f32(new_type, mask as f32)
    } else {
        convert_scalar(old, old_type, new_type, scale_factor, ConvertFlags(0))
    }
}

/// Migrate a parameter whose width or units changed in place, upstream
/// `_convert_parameter_width` plus save.
pub fn migrate_parameter_width<S: Storage + ?Sized>(
    storage: &mut S,
    header: ParamHeader,
    old_type: VarType,
    new_type: VarType,
    dest_configured: bool,
    scale_factor: f32,
    bitmask: bool,
) -> Result<ConvertOutcome, StorageError> {
    if dest_configured {
        return Ok(ConvertOutcome::SkippedConfigured);
    }
    let Some(new_value) =
        convert_parameter_width(storage, header, old_type, new_type, scale_factor, bitmask)
    else {
        return Ok(ConvertOutcome::NotFound);
    };
    match save(storage, header, new_value, None, true)? {
        SaveOutcome::Updated | SaveOutcome::Appended => Ok(ConvertOutcome::Saved),
        _ => Ok(ConvertOutcome::Unchanged),
    }
}

/// Centi-unit migration (`_CM` / `_CD` to metres or degrees), upstream
/// `convert_centi_parameter`.
pub fn migrate_centi_parameter<S: Storage + ?Sized>(
    storage: &mut S,
    header: ParamHeader,
    old_type: VarType,
    new_type: VarType,
    dest_configured: bool,
) -> Result<ConvertOutcome, StorageError> {
    migrate_parameter_width(
        storage,
        header,
        old_type,
        new_type,
        dest_configured,
        0.01,
        false,
    )
}

/// Outcome of migrating one old parameter to a new header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConvertOutcome {
    /// Nothing in storage under the old location.
    NotFound,
    /// Destination already configured and `force` was not set.
    SkippedConfigured,
    /// Value unchanged after conversion.
    Unchanged,
    /// Record written or updated.
    Saved,
}

/// Read an old value, convert, and save under a new header when appropriate.
pub fn migrate_scalar<S: Storage + ?Sized>(
    storage: &mut S,
    info: ConversionInfo,
    new_header: ParamHeader,
    new_type: VarType,
    dest_configured: bool,
    scaler: f32,
    flags: ConvertFlags,
) -> Result<ConvertOutcome, StorageError> {
    let Some((old, _)) = find_old_parameter(storage, info) else {
        return Ok(ConvertOutcome::NotFound);
    };
    if dest_configured && !flags.contains(ConvertFlags::FORCE) {
        return Ok(ConvertOutcome::SkippedConfigured);
    }
    let Some(new_value) = convert_scalar(old, info.old_type, new_type, scaler, flags) else {
        return Ok(ConvertOutcome::NotFound);
    };
    if new_type == info.old_type && is_equal(scaler, 1.0) && flags.0 == 0 {
        // Same type: only save when the bytes differ from what is already live.
        // The caller passes `dest_configured`; when false we still append/update.
        if dest_configured {
            return Ok(ConvertOutcome::Unchanged);
        }
    }
    match save(storage, new_header, new_value, None, true)? {
        SaveOutcome::Updated | SaveOutcome::Appended => {
            Ok(ConvertOutcome::Saved)
        }
        _ => Ok(ConvertOutcome::Unchanged),
    }
}

/// One rename/rehome row, upstream `ConversionInfo` plus its destination.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParameterMigration {
    pub old: ConversionInfo,
    pub new_header: ParamHeader,
    pub new_type: VarType,
    pub scaler: f32,
    pub flags: ConvertFlags,
}

/// Run a migration table, upstream `convert_old_parameters`.
pub fn migrate_parameters<S: Storage + ?Sized>(
    storage: &mut S,
    migrations: &[ParameterMigration],
) -> Result<ConvertClassStats, StorageError> {
    let mut stats = ConvertClassStats::default();
    for m in migrations {
        let dest = configured_in_storage(storage, m.new_header);
        match migrate_scalar(
            storage,
            m.old,
            m.new_header,
            m.new_type,
            dest,
            m.scaler,
            m.flags,
        )? {
            ConvertOutcome::Saved => stats.saved += 1,
            ConvertOutcome::SkippedConfigured | ConvertOutcome::Unchanged => stats.skipped += 1,
            ConvertOutcome::NotFound => stats.not_found += 1,
        }
    }
    Ok(stats)
}

/// One member of a converted group, upstream `GroupInfo` fields used by
/// `AP_Param::convert_class`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GroupMemberDescriptor {
    /// Byte offset of the live value within the object image.
    pub offset: usize,
    /// Stored type of the member.
    pub var_type: VarType,
    /// Index within the old group; six bits per nesting level.
    pub idx: u8,
    /// Destination [`ParamHeader::group_element`] in the new layout.
    pub dest_group_element: u32,
}

/// Summary of a [`convert_class`] pass.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ConvertClassStats {
    /// Members written to storage.
    pub saved: u16,
    /// Members skipped because the destination was already configured.
    pub skipped: u16,
    /// Members with no old value in storage.
    pub not_found: u16,
}

/// Whether a parameter header has a record in storage, upstream
/// `AP_Param::configured_in_storage`.
#[must_use]
pub fn configured_in_storage<S: Storage + ?Sized>(storage: &S, header: ParamHeader) -> bool {
    matches!(scan(storage, header), ScanResult::Found(_))
}


/// One rename row keyed by destination name, upstream `ConversionInfo`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NamedParameterMigration {
    pub old_key: u16,
    pub old_group_element: u32,
    pub old_type: VarType,
    pub new_name: &'static str,
    pub scaler: f32,
    pub flags: ConvertFlags,
}

/// Resolve a named migration row against a descriptor table.
#[must_use]
pub fn resolve_named_migration(
    table: &[crate::ParamInfo<'_>],
    filter: crate::EnumFilter,
    entry: NamedParameterMigration,
) -> Option<ParameterMigration> {
    let dest = crate::find_by_name(table, filter, entry.new_name)?;
    if dest.token_idx != 0 {
        return None;
    }
    let new_type = VarType::from_u8(dest.ptype)?;
    Some(ParameterMigration {
        old: ConversionInfo {
            old_key: entry.old_key,
            old_group_element: entry.old_group_element,
            old_type: entry.old_type,
        },
        new_header: ParamHeader::new(dest.key, dest.ptype, dest.group_element),
        new_type,
        scaler: entry.scaler,
        flags: entry.flags,
    })
}

/// Run a named migration table, upstream `convert_old_parameters`.
pub fn migrate_named_parameters<S: Storage + ?Sized>(
    storage: &mut S,
    table: &[crate::ParamInfo<'_>],
    filter: crate::EnumFilter,
    migrations: &[NamedParameterMigration],
) -> Result<ConvertClassStats, StorageError> {
    let mut stats = ConvertClassStats::default();
    for entry in migrations {
        let Some(m) = resolve_named_migration(table, filter, *entry) else {
            continue;
        };
        let dest = configured_in_storage(storage, m.new_header);
        match migrate_scalar(
            storage,
            m.old,
            m.new_header,
            m.new_type,
            dest,
            m.scaler,
            m.flags,
        )? {
            ConvertOutcome::Saved => stats.saved += 1,
            ConvertOutcome::SkippedConfigured | ConvertOutcome::Unchanged => stats.skipped += 1,
            ConvertOutcome::NotFound => stats.not_found += 1,
        }
    }
    Ok(stats)
}

/// Group-element shift for one nesting level, upstream `group_shift` in
/// `convert_class`.
#[must_use]
pub const fn convert_class_group_shift(is_top_level: bool) -> u8 {
    if is_top_level {
        0
    } else {
        crate::GROUP_LEVEL_SHIFT
    }
}

/// Old storage path for one group member, upstream
/// `(idx << group_shift) + old_index` with the index-zero workaround.
#[must_use]
pub const fn old_group_element_for_member(idx: u8, old_index: u32, is_top_level: bool) -> u32 {
    let shift = convert_class_group_shift(is_top_level);
    let mut effective = idx as u32;
    if shift != 0 && idx == 0 {
        effective = 63;
    }
    (effective << shift) + old_index
}

fn encode_value_to_object(value: ParamValue, dest: &mut [u8]) -> bool {
    match value {
        ParamValue::Int8(v) => {
            if dest.is_empty() {
                return false;
            }
            dest[0] = v as u8;
            true
        }
        ParamValue::Int16(v) => {
            if dest.len() < 2 {
                return false;
            }
            dest[..2].copy_from_slice(&v.to_le_bytes());
            true
        }
        ParamValue::Int32(v) => {
            if dest.len() < 4 {
                return false;
            }
            dest[..4].copy_from_slice(&v.to_le_bytes());
            true
        }
        ParamValue::Float(v) => {
            if dest.len() < 4 {
                return false;
            }
            dest[..4].copy_from_slice(&v.to_le_bytes());
            true
        }
        ParamValue::Vector3f(v) => {
            if dest.len() < 12 {
                return false;
            }
            for (i, c) in v.iter().enumerate() {
                dest[i * 4..i * 4 + 4].copy_from_slice(&c.to_le_bytes());
            }
            true
        }
    }
}

/// Migrate one group member from an old layout, upstream one loop body of
/// `AP_Param::convert_class`.
pub fn convert_class_entry<S: Storage + ?Sized>(
    storage: &mut S,
    old_key: u16,
    new_key: u16,
    old_index: u32,
    is_top_level: bool,
    member: GroupMemberDescriptor,
    object_bytes: &mut [u8],
) -> Result<ConvertOutcome, StorageError> {
    if matches!(member.var_type, VarType::None | VarType::Group) {
        return Ok(ConvertOutcome::NotFound);
    }

    let old_group_element = old_group_element_for_member(member.idx, old_index, is_top_level);
    let info = ConversionInfo {
        old_key,
        old_group_element,
        old_type: member.var_type,
    };

    let Some((value, _)) = find_old_parameter(storage, info) else {
        return Ok(ConvertOutcome::NotFound);
    };

    let new_header = ParamHeader::new(
        new_key,
        member.var_type.as_u8(),
        member.dest_group_element,
    );
    if configured_in_storage(storage, new_header) {
        return Ok(ConvertOutcome::SkippedConfigured);
    }

    let size = member.var_type.size() as usize;
    let obj = object_bytes
        .get_mut(member.offset..member.offset.saturating_add(size))
        .ok_or(StorageError::WriteFailed {
            offset: member.offset as u16,
        })?;
    if !encode_value_to_object(value, obj) {
        return Ok(ConvertOutcome::NotFound);
    }

    match save(storage, new_header, value, None, true)? {
        SaveOutcome::Updated | SaveOutcome::Appended => Ok(ConvertOutcome::Saved),
        _ => Ok(ConvertOutcome::Unchanged),
    }
}

/// Descriptor-driven `AP_Param::convert_class` without the vehicle object graph.
pub fn convert_class<S: Storage + ?Sized>(
    storage: &mut S,
    old_key: u16,
    new_key: u16,
    old_index: u32,
    is_top_level: bool,
    members: &[GroupMemberDescriptor],
    object_bytes: &mut [u8],
) -> Result<ConvertClassStats, StorageError> {
    let mut stats = ConvertClassStats::default();
    for member in members {
        match convert_class_entry(
            storage,
            old_key,
            new_key,
            old_index,
            is_top_level,
            *member,
            object_bytes,
        )? {
            ConvertOutcome::Saved => stats.saved += 1,
            ConvertOutcome::SkippedConfigured => stats.skipped += 1,
            ConvertOutcome::NotFound | ConvertOutcome::Unchanged => stats.not_found += 1,
        }
    }
    Ok(stats)
}


#[cfg(test)]
mod tests {
    use super::*;

    struct Ram {
        bytes: [u8; 128],
    }

    impl Ram {
        fn formatted() -> Self {
            let mut s = Self { bytes: [0xFF; 128] };
            s.bytes[0] = EEPROM_MAGIC[0];
            s.bytes[1] = EEPROM_MAGIC[1];
            s.bytes[2] = EEPROM_REVISION;
            s.bytes[3] = 0;
            let sentinel = ParamHeader::sentinel().to_bytes();
            s.bytes[4..8].copy_from_slice(&sentinel);
            s
        }
    }

    impl Storage for Ram {
        fn read(&self, offset: u16, buf: &mut [u8]) -> bool {
            buf.copy_from_slice(&self.bytes[offset as usize..offset as usize + buf.len()]);
            true
        }

        fn write(&mut self, offset: u16, buf: &[u8]) -> bool {
            self.bytes[offset as usize..offset as usize + buf.len()].copy_from_slice(buf);
            true
        }

        fn size(&self) -> u16 {
            self.bytes.len() as u16
        }
    }

    #[test]
    fn format_storage_writes_header_and_sentinel() {
        let mut s = Ram {
            bytes: [0; 128],
        };
        format_storage(&mut s).expect("format");
        assert!(eeprom_header_valid(&s));
        let missing = ParamHeader::new(1, VarType::Float.as_u8(), 0);
        assert!(matches!(scan(&s, missing), ScanResult::Sentinel(4)));
    }

    #[test]
    fn find_old_parameter_loads_a_stored_scalar() {
        let mut s = Ram::formatted();
        let h = ParamHeader::new(42, VarType::Float.as_u8(), 0);
        save(&mut s, h, ParamValue::Float(3.5), None, false).expect("save");
        let info = ConversionInfo {
            old_key: 42,
            old_group_element: 0,
            old_type: VarType::Float,
        };
        let (v, _) = find_old_parameter(&s, info).expect("found");
        assert_eq!(v, ParamValue::Float(3.5));
    }

    #[test]
    fn reverse_flag_maps_rev_to_reversed() {
        let v = convert_scalar(
            ParamValue::Float(-1.0),
            VarType::Float,
            VarType::Float,
            1.0,
            ConvertFlags::REVERSE,
        )
        .expect("convert");
        assert_eq!(v, ParamValue::Float(1.0));
        let v = convert_scalar(
            ParamValue::Float(0.0),
            VarType::Float,
            VarType::Float,
            1.0,
            ConvertFlags::REVERSE,
        )
        .expect("convert");
        assert_eq!(v, ParamValue::Float(0.0));
    }

    #[test]
    fn int8_minus_one_widens_to_int16_255_via_bitmask() {
        let mut s = Ram::formatted();
        let h = ParamHeader::new(7, VarType::Int8.as_u8(), 0);
        save(&mut s, h, ParamValue::Int8(-1), None, false).expect("save");
        let out = convert_parameter_width(
            &s,
            h,
            VarType::Int8,
            VarType::Int16,
            1.0,
            true,
        )
        .expect("convert");
        assert_eq!(out, ParamValue::Int16(255));
    }

    #[test]
    fn migrate_centi_scales_stored_centi_value() {
        let mut s = Ram::formatted();
        let old_h = ParamHeader::new(10, VarType::Int16.as_u8(), 0);
        save(&mut s, old_h, ParamValue::Int16(5000), None, false).expect("old");
        let new_h = ParamHeader::new(10, VarType::Float.as_u8(), 0);
        let r = migrate_centi_parameter(&mut s, new_h, VarType::Int16, VarType::Float, false)
            .expect("migrate");
        assert_eq!(r, ConvertOutcome::Saved);
        let (v, _) = find_old_parameter(
            &s,
            ConversionInfo {
                old_key: 10,
                old_group_element: 0,
                old_type: VarType::Float,
            },
        )
        .expect("read back");
        assert_eq!(v, ParamValue::Float(50.0));
    }

    #[test]
    fn migrate_parameters_runs_table() {
        let mut s = Ram::formatted();
        let old_a = ParamHeader::new(1, VarType::Int8.as_u8(), 101);
        save(&mut s, old_a, ParamValue::Int8(1), None, false).expect("old a");
        let old_b = ParamHeader::new(1, VarType::Float.as_u8(), 293);
        save(&mut s, old_b, ParamValue::Float(10.0), None, false).expect("old b");
        let table = [
            ParameterMigration {
                old: ConversionInfo {
                    old_key: 1,
                    old_group_element: 101,
                    old_type: VarType::Int8,
                },
                new_header: ParamHeader::new(2, VarType::Int8.as_u8(), 0),
                new_type: VarType::Int8,
                scaler: 1.0,
                flags: ConvertFlags(0),
            },
            ParameterMigration {
                old: ConversionInfo {
                    old_key: 1,
                    old_group_element: 293,
                    old_type: VarType::Float,
                },
                new_header: ParamHeader::new(2, VarType::Float.as_u8(), 1),
                new_type: VarType::Float,
                scaler: 1.0,
                flags: ConvertFlags(0),
            },
        ];
        let stats = migrate_parameters(&mut s, &table).expect("migrate");
        assert_eq!(stats.saved, 2);
        assert_eq!(stats.not_found, 0);
    }

    #[test]
    fn migrate_centi_skips_when_destination_configured() {
        let mut s = Ram::formatted();
        let h = ParamHeader::new(10, VarType::Float.as_u8(), 0);
        save(&mut s, h, ParamValue::Float(1.0), None, false).expect("dest");
        let r = migrate_centi_parameter(&mut s, h, VarType::Int16, VarType::Float, true)
            .expect("migrate");
        assert_eq!(r, ConvertOutcome::SkippedConfigured);
    }

    #[test]
    fn migrate_scalar_appends_converted_value() {
        let mut s = Ram::formatted();
        let old = ConversionInfo {
            old_key: 10,
            old_group_element: 0,
            old_type: VarType::Int16,
        };
        let old_h = ParamHeader::new(10, VarType::Int16.as_u8(), 0);
        save(&mut s, old_h, ParamValue::Int16(100), None, false).expect("old");
        let new_h = ParamHeader::new(10, VarType::Float.as_u8(), 0);
        let r = migrate_scalar(
            &mut s,
            old,
            new_h,
            VarType::Float,
            false,
            0.1,
            ConvertFlags(0),
        )
        .expect("migrate");
        assert_eq!(r, ConvertOutcome::Saved);
        let (v, _) = find_old_parameter(
            &s,
            ConversionInfo {
                old_key: 10,
                old_group_element: 0,
                old_type: VarType::Float,
            },
        )
        .expect("new");
        assert_eq!(v, ParamValue::Float(10.0));
    }
    #[test]
    fn convert_class_entry_copies_old_value_and_saves() {
        let mut s = Ram::formatted();
        let old_key = 50u16;
        let new_key = 55u16;
        let old_h = ParamHeader::new(old_key, VarType::Float.as_u8(), 67);
        save(&mut s, old_h, ParamValue::Float(7.25), None, false).expect("save old");
        let mut obj = [0u8; 16];
        let members = [GroupMemberDescriptor {
            offset: 4,
            var_type: VarType::Float,
            idx: 1,
            dest_group_element: 1,
        }];
        let stats = convert_class(
            &mut s,
            old_key,
            new_key,
            3,
            false,
            &members,
            &mut obj,
        )
        .expect("convert");
        assert_eq!(stats.saved, 1);
        assert_eq!(stats.skipped, 0);
        assert_eq!(stats.not_found, 0);
        let got = f32::from_le_bytes([obj[4], obj[5], obj[6], obj[7]]);
        assert!((got - 7.25).abs() < 1e-6);
        let new_h = ParamHeader::new(new_key, VarType::Float.as_u8(), 1);
        assert!(configured_in_storage(&s, new_h));
    }

    #[test]
    fn old_group_element_applies_index_zero_workaround() {
        assert_eq!(old_group_element_for_member(0, 0, true), 0);
        assert_eq!(old_group_element_for_member(1, 0, true), 1);
        assert_eq!(old_group_element_for_member(0, 0, false), 63 << 6);
        assert_eq!(old_group_element_for_member(1, 0, false), 64);
    }


    #[test]
    fn convert_class_skips_when_destination_configured() {
        let mut s = Ram::formatted();
        let old_key = 51u16;
        let new_key = 56u16;
        let old_h = ParamHeader::new(old_key, VarType::Int16.as_u8(), 65);
        save(&mut s, old_h, ParamValue::Int16(500), None, false).expect("save old");
        let dest_h = ParamHeader::new(new_key, VarType::Int16.as_u8(), 1);
        save(&mut s, dest_h, ParamValue::Int16(999), None, false).expect("save dest");
        let mut obj = [0u8; 8];
        let members = [GroupMemberDescriptor {
            offset: 0,
            var_type: VarType::Int16,
            idx: 1,
            dest_group_element: 1,
        }];
        let stats = convert_class(
            &mut s,
            old_key,
            new_key,
            1,
            false,
            &members,
            &mut obj,
        )
        .expect("convert");
        assert_eq!(stats.saved, 0);
        assert_eq!(stats.skipped, 1);
        let (v, _) = find_old_parameter(
            &s,
            ConversionInfo {
                old_key: new_key,
                old_group_element: 1,
                old_type: VarType::Int16,
            },
        )
        .expect("still dest");
        assert_eq!(v, ParamValue::Int16(999));
    }

    #[test]
    fn migrate_named_parameters_resolves_plane_fence_table() {
        use crate::info::{find_by_name, EnumFilter, GroupInfo, ParamInfo, ParamRef, FRAME_PLANE};
        use crate::plane::PLANE_FENCE_CONVERSIONS;

        static FENCE_ALT_MIN: [GroupInfo<'static>; 1] = [GroupInfo {
            name: "ALT_MIN",
            idx: 7,
            ptype: VarType::Float.as_u8(),
            flags: 0,
            group: None,
        }];
        static FENCE: [ParamInfo<'static>; 1] = [ParamInfo {
            name: "FENCE_",
            key: 132,
            ptype: VarType::Group.as_u8(),
            flags: 0,
            group: Some(&FENCE_ALT_MIN),
        }];

        let filter = EnumFilter::for_frame(FRAME_PLANE);
        let dest = find_by_name(&FENCE, filter, "FENCE_ALT_MIN").expect("descriptor");
        let resolved = resolve_named_migration(
            &FENCE,
            filter,
            PLANE_FENCE_CONVERSIONS[0],
        )
        .expect("resolved");
        assert_eq!(
            resolved.new_header,
            ParamHeader::new(dest.key, dest.ptype, dest.group_element),
        );

        let mut s = Ram::formatted();
        let old_h = ParamHeader::new(228, VarType::Int16.as_u8(), 0);
        save(&mut s, old_h, ParamValue::Int16(-10), None, false).expect("old");
        let stats = migrate_named_parameters(&mut s, &FENCE, filter, PLANE_FENCE_CONVERSIONS)
            .expect("migrate");
        assert_eq!(stats.saved, 1);
        assert_eq!(stats.skipped, 0);
        assert_eq!(stats.not_found, 0);
        let (v, _) = find_old_parameter(
            &s,
            ConversionInfo {
                old_key: dest.key,
                old_group_element: dest.group_element,
                old_type: VarType::Float,
            },
        )
        .expect("migrated");
        assert_eq!(v, ParamValue::Float(-10.0));
    }


}
