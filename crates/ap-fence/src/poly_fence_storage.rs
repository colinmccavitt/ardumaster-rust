//! First `AC_PolyFence_loader` EEPROM format leftover: item types,
//! storage magic, `formatted()`, `format()`, write primitives, and
//! `fence_storage_space_required`. Scan / index / `write_fence` /
//! `load_from_storage` / SD stay later. Upstream
//! `libraries/AC_Fence/AC_PolyFence_loader.cpp`. Tracked as **COP-025**.

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

/// `formatted()` — bytes `[0..4]` are magic, then three zeros.
#[must_use]
pub fn storage_formatted(buf: &[u8]) -> bool {
    matches!(buf.get(..4), Some(&[STORAGE_MAGIC, 0, 0, 0]))
}

/// `write_type_to_storage`. Advances `offset` by 1.
pub fn write_type_to_storage(buf: &mut [u8], offset: &mut u16, kind: PolyFenceType) -> bool {
    let i = usize::from(*offset);
    let Some(slot) = buf.get_mut(i) else {
        return false;
    };
    *slot = kind.as_u8();
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

/// `write_eos_to_storage`. Returns the offset of the EOS marker.
pub fn write_eos_to_storage(buf: &mut [u8], offset: &mut u16) -> Option<u16> {
    if !write_type_to_storage(buf, offset, PolyFenceType::EndOfStorage) {
        return None;
    }
    Some(offset.saturating_sub(1))
}

/// `format()` — write a 4-byte header (`235, 0, 0, 0`) and an EOS marker.
///
/// Returns the EOS offset (`4` on success). The rest of the buffer is left
/// untouched. Scan / index stay later.
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
