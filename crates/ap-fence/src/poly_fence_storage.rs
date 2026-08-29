//! `AC_PolyFence_loader` EEPROM leftover: item types, storage magic,
//! `formatted()`, `format()`, write primitives, `fence_storage_space_required`,
//! `scan_eeprom`, the storage index, and `write_fence`.
//! `load_from_storage` / SD stay later. Upstream
//! `libraries/AC_Fence/AC_PolyFence_loader.cpp`. Tracked as **COP-025**.

use ap_math::location::check_latlng_1e7;
use ap_math::scalar::is_positive;

/// `new_fence_storage_magic`. Byte 0 of a formatted fence store.
pub const STORAGE_MAGIC: u8 = 235;

/// `AC_PolyFenceType`. Values match the C++ enum, including the
/// deprecated integer-radius circle types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PolyFenceType {
    /// Float-radius inclusion circle. Upstream `CIRCLE_INCLUSION`.
    CircleInclusion = 92,
    /// Float-radius exclusion circle. Upstream `CIRCLE_EXCLUSION`.
    CircleExclusion = 93,
    /// Deprecated integer-radius inclusion circle.
    CircleInclusionInt = 94,
    /// Rally / return point. Upstream `RETURN_POINT`.
    ReturnPoint = 95,
    /// Deprecated integer-radius exclusion circle.
    CircleExclusionInt = 96,
    /// Vertex exclusion polygon. Upstream `POLYGON_EXCLUSION`.
    PolygonExclusion = 97,
    /// Vertex inclusion polygon. Upstream `POLYGON_INCLUSION`.
    PolygonInclusion = 98,
    /// End-of-storage marker. Upstream `END_OF_STORAGE`.
    EndOfStorage = 99,
}

impl PolyFenceType {
    /// Decode a stored type byte. `None` is the C++ `default:` corrupt path.
    #[must_use]
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            92 => Some(Self::CircleInclusion),
            93 => Some(Self::CircleExclusion),
            94 => Some(Self::CircleInclusionInt),
            95 => Some(Self::ReturnPoint),
            96 => Some(Self::CircleExclusionInt),
            97 => Some(Self::PolygonExclusion),
            98 => Some(Self::PolygonInclusion),
            99 => Some(Self::EndOfStorage),
            _ => None,
        }
    }

    /// The stored type byte.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Types `scan_eeprom` accepts as non-corrupt.
    #[must_use]
    pub const fn is_storage_item(self) -> bool {
        !matches!(self, Self::EndOfStorage)
    }
}

/// One `AC_PolyFenceItem` — type, lat/lng, optional vertex count / radius.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PolyFenceItem {
    /// Item type. Integer-radius circle types should not appear here;
    /// `get_item` rewrites them to the float forms.
    pub kind: PolyFenceType,
    /// Latitude, 1e-7 degrees. Upstream `loc.x`.
    pub lat: i32,
    /// Longitude, 1e-7 degrees. Upstream `loc.y`.
    pub lng: i32,
    /// Vertices in this polygon. Ignored for circles and return points.
    pub vertex_count: u8,
    /// Circle radius, metres. Ignored for polygons and return points.
    pub radius_m: f32,
}

impl PolyFenceItem {
    /// A polygon vertex row. `vertex_count` is the same on every row.
    #[must_use]
    pub const fn polygon(kind: PolyFenceType, lat: i32, lng: i32, vertex_count: u8) -> Self {
        Self {
            kind,
            lat,
            lng,
            vertex_count,
            radius_m: 0.0,
        }
    }

    /// A float-radius circle.
    #[must_use]
    pub const fn circle(kind: PolyFenceType, lat: i32, lng: i32, radius_m: f32) -> Self {
        Self {
            kind,
            lat,
            lng,
            vertex_count: 0,
            radius_m,
        }
    }

    /// A return point.
    #[must_use]
    pub const fn return_point(lat: i32, lng: i32) -> Self {
        Self {
            kind: PolyFenceType::ReturnPoint,
            lat,
            lng,
            vertex_count: 0,
            radius_m: 0.0,
        }
    }
}

/// `FenceIndex` — type, item count, and type-byte offset of one stored fence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FenceIndex {
    /// Stored fence type. EOS never appears in a successful index.
    pub kind: PolyFenceType,
    /// Vertices for a polygon; `1` for circles and return points.
    pub count: u16,
    /// Offset of the type byte. Upstream `FenceIndex::storage_offset`.
    pub storage_offset: u16,
}

impl FenceIndex {
    /// An unused slot. `index_eeprom` overwrites the live prefix.
    pub const EMPTY: Self = Self {
        kind: PolyFenceType::EndOfStorage,
        count: 0,
        storage_offset: 0,
    };
}

/// `_eeprom_fence_count` / `_eeprom_item_count` from `scan_eeprom_count_fences`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EepromCounts {
    /// Number of fences (not including EOS).
    pub fence_count: u16,
    /// Sum of polygon vertex counts plus one per circle / return point.
    pub item_count: u16,
}

/// Result of `index_eeprom`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IndexResult {
    /// Entries written into the caller-supplied index. Equals `counts.fence_count`.
    pub num_fences: u16,
    /// Counts from the first scan pass.
    pub counts: EepromCounts,
    /// Offset of the EOS marker. Upstream `_eos_offset`.
    pub eos_offset: u16,
}

/// Result of `write_fence`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WriteFenceResult {
    /// Offset of the EOS marker written after the items.
    pub eos_offset: u16,
    /// `FENCE_TOTAL` leftover: `0` unless `total_vertex_count >= 3`, then `+ 2`.
    pub new_total: u16,
}

/// `formatted()` — bytes `[0..4]` are magic, then three zeros.
#[must_use]
pub fn storage_formatted(buf: &[u8]) -> bool {
    matches!(buf.get(..4), Some(&[STORAGE_MAGIC, 0, 0, 0]))
}

/// `max_items`. Upstream `fence_storage.size() / sizeof(Vector2l)`.
#[must_use]
pub fn max_items(buf: &[u8]) -> u16 {
    u16::try_from(buf.len() / 8).unwrap_or(u16::MAX)
}

/// `write_type_to_storage`. Advances `offset` by 1.
pub fn write_type_to_storage(buf: &mut [u8], offset: &mut u16, kind: PolyFenceType) -> bool {
    write_uint8_to_storage(buf, offset, kind.as_u8())
}

/// `fence_storage.write_uint8` leftover. Advances `offset` by 1.
pub fn write_uint8_to_storage(buf: &mut [u8], offset: &mut u16, value: u8) -> bool {
    let i = usize::from(*offset);
    let Some(slot) = buf.get_mut(i) else {
        return false;
    };
    *slot = value;
    *offset = offset.saturating_add(1);
    true
}

/// `write_latlon_to_storage`. Little-endian int32 lat then lng; +8.
pub fn write_latlon_to_storage(buf: &mut [u8], offset: &mut u16, lat: i32, lng: i32) -> bool {
    if !write_i32_le(buf, offset, lat) {
        return false;
    }
    write_i32_le(buf, offset, lng)
}

/// `fence_storage.write_float` leftover. Little-endian IEEE-754; +4.
pub fn write_f32_to_storage(buf: &mut [u8], offset: &mut u16, value: f32) -> bool {
    let at = usize::from(*offset);
    let bytes = value.to_le_bytes();
    for (k, byte) in bytes.iter().enumerate() {
        let Some(slot) = buf.get_mut(at.saturating_add(k)) else {
            return false;
        };
        *slot = *byte;
    }
    *offset = offset.saturating_add(4);
    true
}

/// `write_eos_to_storage`. Returns the offset of the EOS marker.
pub fn write_eos_to_storage(buf: &mut [u8], offset: &mut u16) -> Option<u16> {
    if !write_type_to_storage(buf, offset, PolyFenceType::EndOfStorage) {
        return None;
    }
    Some(offset.saturating_sub(1))
}

/// `read_uint8` leftover. Advances `offset` by 1.
pub fn read_uint8_from_storage(buf: &[u8], offset: &mut u16) -> Option<u8> {
    let value = *buf.get(usize::from(*offset))?;
    *offset = offset.saturating_add(1);
    Some(value)
}

/// `read_latlon_from_storage`. Little-endian int32 lat then lng; +8.
pub fn read_latlon_from_storage(buf: &[u8], offset: &mut u16) -> Option<(i32, i32)> {
    let lat = read_i32_le(buf, offset)?;
    let lng = read_i32_le(buf, offset)?;
    Some((lat, lng))
}

/// `fence_storage.read_float` leftover. Little-endian IEEE-754; +4.
pub fn read_f32_from_storage(buf: &[u8], offset: &mut u16) -> Option<f32> {
    let at = usize::from(*offset);
    let bytes = buf.get(at..at.saturating_add(4))?;
    let arr: [u8; 4] = bytes.try_into().ok()?;
    *offset = offset.saturating_add(4);
    Some(f32::from_le_bytes(arr))
}

/// `format()` — write a 4-byte header (`235, 0, 0, 0`) and an EOS marker.
///
/// Returns the EOS offset (`4` on success). The rest of the buffer is left
/// untouched.
pub fn format_storage(buf: &mut [u8]) -> Option<u16> {
    // `write_uint32(0, 0)` then `write_uint8(0, magic)` then `offset += 4`.
    if buf.len() < 5 {
        return None;
    }
    if !write_u32_le_at(buf, 0, 0) {
        return None;
    }
    if let Some(slot) = buf.get_mut(0) {
        *slot = STORAGE_MAGIC;
    }
    let mut offset = 4_u16;
    write_eos_to_storage(buf, &mut offset)
}

/// `fence_storage_space_required`. Header plus packed items; no EOS.
///
/// A polygon of `N` vertices is `N` items that share `vertex_count`.
/// Integer-radius circle types are not stored as items (C++
/// `INTERNAL_ERROR`); they contribute only the type byte so a later
/// leftover can reject them.
#[must_use]
pub fn fence_storage_space_required(items: &[PolyFenceItem]) -> u16 {
    let mut ret = 4_u16;
    let mut i = 0_usize;
    while i < items.len() {
        let Some(item) = items.get(i) else {
            break;
        };
        ret = ret.saturating_add(1);
        match item.kind {
            PolyFenceType::PolygonInclusion | PolyFenceType::PolygonExclusion => {
                let vc = u16::from(item.vertex_count);
                ret = ret.saturating_add(1).saturating_add(vc.saturating_mul(8));
                i = i.saturating_add(usize::from(item.vertex_count.saturating_sub(1)));
            }
            PolyFenceType::CircleInclusion | PolyFenceType::CircleExclusion => {
                ret = ret.saturating_add(12);
            }
            PolyFenceType::ReturnPoint => {
                ret = ret.saturating_add(8);
            }
            PolyFenceType::EndOfStorage
            | PolyFenceType::CircleInclusionInt
            | PolyFenceType::CircleExclusionInt => {}
        }
        i = i.saturating_add(1);
    }
    ret
}

/// `scan_eeprom`. Calls `visit(type, offset_of_type_byte)` for each record,
/// including EOS. Returns the EOS offset.
///
/// Unformatted or corrupt storage is `None`. The visitor leftover of
/// `scan_fn_t` is a `FnMut` so count and index can share this walk.
pub fn scan_eeprom<F>(buf: &[u8], mut visit: F) -> Option<u16>
where
    F: FnMut(PolyFenceType, u16),
{
    if !storage_formatted(buf) {
        return None;
    }
    let mut read_offset = 4_u16;
    loop {
        // C++: `read_offset > fence_storage.size()` then `read_uint8`.
        // Offset equal to the length is already past the last byte.
        if usize::from(read_offset) >= buf.len() {
            return None;
        }
        let raw = *buf.get(usize::from(read_offset))?;
        let Some(kind) = PolyFenceType::from_u8(raw) else {
            return None;
        };
        visit(kind, read_offset);
        read_offset = read_offset.saturating_add(1);
        match kind {
            PolyFenceType::EndOfStorage => return Some(read_offset.saturating_sub(1)),
            PolyFenceType::PolygonInclusion | PolyFenceType::PolygonExclusion => {
                let vertex_count = *buf.get(usize::from(read_offset))?;
                read_offset = read_offset
                    .saturating_add(1)
                    .saturating_add(u16::from(vertex_count).saturating_mul(8));
            }
            PolyFenceType::CircleInclusion
            | PolyFenceType::CircleExclusion
            | PolyFenceType::CircleInclusionInt
            | PolyFenceType::CircleExclusionInt => {
                read_offset = read_offset.saturating_add(12);
            }
            PolyFenceType::ReturnPoint => {
                read_offset = read_offset.saturating_add(8);
            }
        }
    }
}

/// `count_eeprom_fences` / `scan_eeprom_count_fences`.
#[must_use]
pub fn count_eeprom_fences(buf: &[u8]) -> Option<EepromCounts> {
    let mut counts = EepromCounts {
        fence_count: 0,
        item_count: 0,
    };
    scan_eeprom(buf, |kind, read_offset| {
        count_one_fence(kind, read_offset, buf, &mut counts);
    })?;
    Some(counts)
}

/// `index_eeprom` / `scan_eeprom_index_fences`.
///
/// Fills `index[..num_fences]`. The caller supplies the storage the C++
/// `NEW_NOTHROW FenceIndex[_eeprom_fence_count]` would have allocated.
/// Too-small `index` is the allocation-failure leftover (`None`).
/// Unformatted storage is `None` — call [`format_storage`] first; the C++
/// auto-format path stays with the owning loader.
#[must_use]
pub fn index_eeprom(buf: &[u8], index: &mut [FenceIndex]) -> Option<IndexResult> {
    let counts = count_eeprom_fences(buf)?;
    if counts.fence_count == 0 {
        let eos_offset = scan_eeprom(buf, |_, _| {})?;
        return Some(IndexResult {
            num_fences: 0,
            counts,
            eos_offset,
        });
    }
    if index.len() < usize::from(counts.fence_count) {
        return None;
    }
    let mut num_fences = 0_u16;
    let eos_offset = scan_eeprom(buf, |kind, read_offset| {
        if !kind.is_storage_item() {
            return;
        }
        let count = match kind {
            PolyFenceType::PolygonInclusion | PolyFenceType::PolygonExclusion => u16::from(
                buf.get(usize::from(read_offset.saturating_add(1)))
                    .copied()
                    .unwrap_or(0),
            ),
            _ => 1,
        };
        if let Some(slot) = index.get_mut(usize::from(num_fences)) {
            *slot = FenceIndex {
                kind,
                count,
                storage_offset: read_offset,
            };
        }
        num_fences = num_fences.saturating_add(1);
    })?;
    if num_fences != counts.fence_count {
        return None;
    }
    Some(IndexResult {
        num_fences,
        counts,
        eos_offset,
    })
}

/// `index_fence_count`.
#[must_use]
pub fn index_fence_count(index: &[FenceIndex], num_fences: u16, kind: PolyFenceType) -> u16 {
    let mut ret = 0_u16;
    let n = usize::from(num_fences).min(index.len());
    let Some(live) = index.get(..n) else {
        return 0;
    };
    for entry in live {
        if entry.kind == kind {
            ret = ret.saturating_add(1);
        }
    }
    ret
}

/// `sum_of_polygon_point_counts_and_returnpoint`.
#[must_use]
pub fn sum_of_polygon_point_counts_and_returnpoint(index: &[FenceIndex], num_fences: u16) -> u16 {
    let mut ret = 0_u16;
    let n = usize::from(num_fences).min(index.len());
    let Some(live) = index.get(..n) else {
        return 0;
    };
    for entry in live {
        match entry.kind {
            PolyFenceType::ReturnPoint
            | PolyFenceType::PolygonInclusion
            | PolyFenceType::PolygonExclusion => {
                ret = ret.saturating_add(entry.count);
            }
            _ => {}
        }
    }
    ret
}

/// `validate_fence`. Lat/lng use [`check_latlng_1e7`]; circle radius uses
/// [`is_positive`]. Integer-radius items and a mid-polygon type change fail.
#[must_use]
pub fn validate_fence(items: &[PolyFenceItem]) -> bool {
    let mut expecting_type = PolyFenceType::EndOfStorage;
    let mut expected_type_count = 0_u16;
    let mut orig_expected_type_count = 0_u16;
    let mut seen_return_point = false;

    for item in items {
        let validate_latlon = match item.kind {
            PolyFenceType::EndOfStorage => return false,
            PolyFenceType::PolygonInclusion | PolyFenceType::PolygonExclusion => {
                if item.vertex_count < 3 {
                    return false;
                }
                if expected_type_count == 0 {
                    expected_type_count = u16::from(item.vertex_count);
                    orig_expected_type_count = expected_type_count;
                    expecting_type = item.kind;
                } else if item.kind != expecting_type
                    || u16::from(item.vertex_count) != orig_expected_type_count
                {
                    return false;
                }
                expected_type_count = expected_type_count.saturating_sub(1);
                true
            }
            PolyFenceType::CircleInclusionInt | PolyFenceType::CircleExclusionInt => {
                return false;
            }
            PolyFenceType::CircleInclusion | PolyFenceType::CircleExclusion => {
                if expected_type_count != 0 {
                    return false;
                }
                if !is_positive(item.radius_m) {
                    return false;
                }
                true
            }
            PolyFenceType::ReturnPoint => {
                if expected_type_count != 0 {
                    return false;
                }
                if seen_return_point {
                    return false;
                }
                seen_return_point = true;
                true
            }
        };
        if validate_latlon && !check_latlng_1e7(item.lat, item.lng) {
            return false;
        }
    }
    expected_type_count == 0
}

/// `write_fence`. Validates, checks `fence_storage_space_required` against
/// the buffer, `format()`s, packs items, and writes EOS.
///
/// Logger / `void_index` / `FENCE_TOTAL` param-save stay later; [`WriteFenceResult::new_total`]
/// is the value those would have stored. `load_from_storage` stays later.
pub fn write_fence(buf: &mut [u8], items: &[PolyFenceItem]) -> Option<WriteFenceResult> {
    if !validate_fence(items) {
        return None;
    }
    if usize::from(fence_storage_space_required(items)) > buf.len() {
        return None;
    }
    format_storage(buf)?;
    let mut offset = 4_u16;
    let mut vertex_count = 0_u8;
    let mut total_vertex_count = 0_u16;
    for item in items {
        match item.kind {
            PolyFenceType::PolygonInclusion | PolyFenceType::PolygonExclusion => {
                if vertex_count == 0 {
                    vertex_count = item.vertex_count;
                    total_vertex_count = total_vertex_count.saturating_add(u16::from(vertex_count));
                    if !write_type_to_storage(buf, &mut offset, item.kind) {
                        return None;
                    }
                    if !write_uint8_to_storage(buf, &mut offset, vertex_count) {
                        return None;
                    }
                }
                vertex_count = vertex_count.saturating_sub(1);
                if !write_latlon_to_storage(buf, &mut offset, item.lat, item.lng) {
                    return None;
                }
            }
            PolyFenceType::EndOfStorage
            | PolyFenceType::CircleInclusionInt
            | PolyFenceType::CircleExclusionInt => {
                return None;
            }
            PolyFenceType::CircleInclusion | PolyFenceType::CircleExclusion => {
                total_vertex_count = total_vertex_count.saturating_add(1);
                if !write_type_to_storage(buf, &mut offset, item.kind) {
                    return None;
                }
                if !write_latlon_to_storage(buf, &mut offset, item.lat, item.lng) {
                    return None;
                }
                if !write_f32_to_storage(buf, &mut offset, item.radius_m) {
                    return None;
                }
            }
            PolyFenceType::ReturnPoint => {
                if !write_type_to_storage(buf, &mut offset, item.kind) {
                    return None;
                }
                if !write_latlon_to_storage(buf, &mut offset, item.lat, item.lng) {
                    return None;
                }
            }
        }
    }
    let eos_offset = write_eos_to_storage(buf, &mut offset)?;
    let new_total = if total_vertex_count >= 3 {
        total_vertex_count.saturating_add(2)
    } else {
        0
    };
    Some(WriteFenceResult {
        eos_offset,
        new_total,
    })
}

fn count_one_fence(kind: PolyFenceType, read_offset: u16, buf: &[u8], counts: &mut EepromCounts) {
    if !kind.is_storage_item() {
        return;
    }
    counts.fence_count = counts.fence_count.saturating_add(1);
    match kind {
        PolyFenceType::PolygonInclusion | PolyFenceType::PolygonExclusion => {
            if let Some(vertex_count) = buf.get(usize::from(read_offset.saturating_add(1))) {
                counts.item_count = counts.item_count.saturating_add(u16::from(*vertex_count));
            }
        }
        PolyFenceType::CircleInclusion
        | PolyFenceType::CircleExclusion
        | PolyFenceType::CircleInclusionInt
        | PolyFenceType::CircleExclusionInt
        | PolyFenceType::ReturnPoint => {
            counts.item_count = counts.item_count.saturating_add(1);
        }
        PolyFenceType::EndOfStorage => {}
    }
}

fn write_u32_le_at(buf: &mut [u8], at: usize, value: u32) -> bool {
    let bytes = value.to_le_bytes();
    for (k, byte) in bytes.iter().enumerate() {
        let Some(slot) = buf.get_mut(at.saturating_add(k)) else {
            return false;
        };
        *slot = *byte;
    }
    true
}

fn write_i32_le(buf: &mut [u8], offset: &mut u16, value: i32) -> bool {
    let at = usize::from(*offset);
    let bytes = value.to_le_bytes();
    for (k, byte) in bytes.iter().enumerate() {
        let Some(slot) = buf.get_mut(at.saturating_add(k)) else {
            return false;
        };
        *slot = *byte;
    }
    *offset = offset.saturating_add(4);
    true
}

fn read_i32_le(buf: &[u8], offset: &mut u16) -> Option<i32> {
    let at = usize::from(*offset);
    let bytes = buf.get(at..at.saturating_add(4))?;
    let arr: [u8; 4] = bytes.try_into().ok()?;
    *offset = offset.saturating_add(4);
    Some(i32::from_le_bytes(arr))
}
