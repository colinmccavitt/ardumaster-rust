//! Descriptor tables and the name/key/group_element mapping (FW-004 slice 2).
//!
//! Upstream's `Info` and `GroupInfo` carry a pointer to the variable in memory,
//! and the whole parameter system works by writing through it. This port keeps
//! only the *description* here — name, key, index, type, flags, nesting — and
//! leaves reaching the value to a later slice, because the description is what
//! decides where a parameter is stored and that is what ADR-0010 pins.
//!
//! # Tables borrow, and are not `'static`
//!
//! Upstream's tables are file-scope arrays reached through a global. Under
//! ADR-0004 the caller passes them in, and they carry a lifetime rather than
//! demanding `'static`, so a test can build a table at runtime and the eventual
//! vehicle can still hand over a `const` one.
//!
//! # Where a parameter lives
//!
//! A parameter is identified by its top-level key plus an eighteen-bit
//! `group_element`, built six bits per level of nesting by [`group_id`]. Three
//! levels fit; upstream stops recursing when a fourth would overflow.

use crate::VarType;

/// Longest parameter name, upstream `AP_MAX_NAME_SIZE`.
///
/// Names are built by concatenation and simply truncated at this length —
/// upstream uses `strncpy` into a fixed buffer, so a long path silently loses
/// its tail rather than being rejected.
pub const MAX_NAME_SIZE: usize = 16;

/// Bits of `group_element` consumed per level of nesting, upstream
/// `_group_level_shift`.
pub const GROUP_LEVEL_SHIFT: u8 = 6;

/// Total bits available to `group_element`, upstream `_group_bits`.
pub const GROUP_BITS: u8 = 18;

/// The group's offset is relative to the nested object, upstream
/// `AP_PARAM_FLAG_NESTED_OFFSET`.
pub const FLAG_NESTED_OFFSET: u16 = 1 << 0;
/// The entry reaches its object through a pointer, upstream
/// `AP_PARAM_FLAG_POINTER`. Such a group contributes no parameters when the
/// pointer is null, which is why the enumeration here can legitimately be a
/// superset of a running vehicle's.
pub const FLAG_POINTER: u16 = 1 << 1;
/// The parameter enables or disables its group, upstream
/// `AP_PARAM_FLAG_ENABLE`.
pub const FLAG_ENABLE: u16 = 1 << 2;
/// Exempt from the index-zero workaround in [`group_id`], upstream
/// `AP_PARAM_FLAG_NO_SHIFT`.
pub const FLAG_NO_SHIFT: u16 = 1 << 3;
/// The nested table is reached through a pointer, upstream
/// `AP_PARAM_FLAG_INFO_POINTER`.
pub const FLAG_INFO_POINTER: u16 = 1 << 4;
/// Not offered over MAVLink, upstream `AP_PARAM_FLAG_INTERNAL_USE_ONLY`.
pub const FLAG_INTERNAL_USE_ONLY: u16 = 1 << 5;
/// Hidden from parameter listings, upstream `AP_PARAM_FLAG_HIDDEN`.
pub const FLAG_HIDDEN: u16 = 1 << 6;
/// The default value is at an offset from the object, upstream
/// `AP_PARAM_FLAG_DEFAULT_POINTER`.
pub const FLAG_DEFAULT_POINTER: u16 = 1 << 7;

/// One entry in a nested group, upstream `GroupInfo` minus the memory offset.
#[derive(Debug, Clone, Copy)]
pub struct GroupInfo<'a> {
    /// Name fragment, concatenated onto the enclosing prefix.
    pub name: &'a str,
    /// Identifier within the group; six bits, shifted by nesting depth.
    pub idx: u8,
    /// Type tag, raw rather than a [`VarType`] so an unknown tag round-trips.
    pub ptype: u8,
    /// `AP_PARAM_FLAG_*` bits.
    pub flags: u16,
    /// Entries below this one, when `ptype` is a group.
    pub group: Option<&'a [GroupInfo<'a>]>,
}

/// One top-level variable, upstream `Info` minus the pointer.
#[derive(Debug, Clone, Copy)]
pub struct ParamInfo<'a> {
    /// Name, or the prefix for everything in the group below it.
    pub name: &'a str,
    /// Nine-bit storage key, upstream `k_param_*`.
    pub key: u16,
    /// Type tag.
    pub ptype: u8,
    /// `AP_PARAM_FLAG_*` bits.
    pub flags: u16,
    /// Entries below this one, when `ptype` is a group.
    pub group: Option<&'a [GroupInfo<'a>]>,
}

/// The identifier of one element of a group, upstream `group_id`.
///
/// # The index-zero workaround
///
/// An `idx` of 0 shifted by any number of bits is still 0, which makes a
/// nested element with index 0 indistinguishable from its own parent. Upstream
/// calls this "a bug in the original design" and works around it by
/// substituting 63 — so a group's first element is stored under an identifier
/// that has nothing to do with its index. The substitution applies only below
/// the top level, and `AP_PARAM_FLAG_NO_SHIFT` opts an entry out.
///
/// This is reproduced rather than fixed: the identifier is the storage address,
/// so changing it would move every affected parameter (ADR-0010).
#[must_use]
pub const fn group_id(idx: u8, base: u32, shift: u8, flags: u16) -> u32 {
    if idx == 0 && shift != 0 && (flags & FLAG_NO_SHIFT) == 0 {
        base + (63u32 << shift)
    } else {
        base + ((idx as u32) << shift)
    }
}

/// Bit position of the frame-type flags within a `flags` field, upstream
/// `AP_PARAM_FRAME_TYPE_SHIFT`. The low eight bits are the `FLAG_*` values and
/// everything above is a frame mask.
pub const FRAME_TYPE_SHIFT: u16 = 8;

/// Frame bits, upstream `AP_PARAM_FRAME_*`. A parameter carrying any of these
/// appears only on a vehicle whose frame matches.
pub const FRAME_COPTER: u16 = 1 << 0;
/// See [`FRAME_COPTER`].
pub const FRAME_ROVER: u16 = 1 << 1;
/// See [`FRAME_COPTER`].
pub const FRAME_PLANE: u16 = 1 << 2;
/// See [`FRAME_COPTER`].
pub const FRAME_SUB: u16 = 1 << 3;
/// See [`FRAME_COPTER`].
pub const FRAME_TRICOPTER: u16 = 1 << 4;
/// See [`FRAME_COPTER`].
pub const FRAME_HELI: u16 = 1 << 5;
/// See [`FRAME_COPTER`].
pub const FRAME_BLIMP: u16 = 1 << 6;

/// Whether an entry belongs on this vehicle, upstream `check_frame_type`.
///
/// An entry with no frame bits belongs on every frame; one with frame bits
/// belongs only where at least one of them is set. `FLAG_HIDDEN` excludes an
/// entry from every frame.
///
/// A group that fails this takes its whole subtree with it, which is how a
/// fixed-wing build ends up without the multirotor parameters even though the
/// tables describe them.
#[must_use]
pub const fn check_frame_type(flags: u16, frame_type_flags: u16) -> bool {
    if flags & FLAG_HIDDEN != 0 {
        return false;
    }
    let frame_flags = flags >> FRAME_TYPE_SHIFT;
    frame_flags == 0 || (frame_flags & frame_type_flags) != 0
}

/// What to include when walking a descriptor table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EnumFilter {
    /// The vehicle's frame mask, upstream `_frame_type_flags`.
    pub frame_type_flags: u16,
    /// Whether to include entries flagged [`FLAG_HIDDEN`].
    ///
    /// Upstream never does — `check_frame_type` rejects them, so they never
    /// reach `first()`/`next()`. But `save_sync` writes them to storage like
    /// any other parameter; the flag only suppresses the announcement to a
    /// GCS. So anything that has to account for what is *in* storage needs
    /// them, and anything reproducing upstream's parameter list does not.
    pub include_hidden: bool,
}

impl EnumFilter {
    /// Upstream's behaviour: this frame, no hidden entries.
    #[must_use]
    pub const fn for_frame(frame_type_flags: u16) -> Self {
        Self {
            frame_type_flags,
            include_hidden: false,
        }
    }

    /// Everything on this frame, hidden entries included.
    #[must_use]
    pub const fn including_hidden(frame_type_flags: u16) -> Self {
        Self {
            frame_type_flags,
            include_hidden: true,
        }
    }

    /// Whether an entry passes this filter.
    #[must_use]
    pub const fn admits(self, flags: u16) -> bool {
        if flags & FLAG_HIDDEN != 0 && !self.include_hidden {
            return false;
        }
        let frame_flags = flags >> FRAME_TYPE_SHIFT;
        frame_flags == 0 || (frame_flags & self.frame_type_flags) != 0
    }
}

/// A fixed-capacity parameter name, built by truncating concatenation.
///
/// Reproduces upstream's `strncpy` into a 16-byte buffer: appending past the
/// end silently drops the excess, and once full nothing more is added.
#[derive(Clone, Copy)]
pub struct ParamName {
    buf: [u8; MAX_NAME_SIZE],
    len: u8,
}

impl Default for ParamName {
    fn default() -> Self {
        Self::new()
    }
}

impl ParamName {
    /// An empty name.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            buf: [0; MAX_NAME_SIZE],
            len: 0,
        }
    }

    /// Append, dropping whatever does not fit.
    pub fn push(&mut self, s: &str) {
        for &b in s.as_bytes() {
            let len = self.len as usize;
            if len >= MAX_NAME_SIZE {
                return;
            }
            #[allow(
                clippy::indexing_slicing,
                reason = "len is checked against the array's length immediately above"
            )]
            {
                self.buf[len] = b;
            }
            self.len += 1;
        }
    }

    /// Append `_X`, `_Y` or `_Z`, upstream `add_vector3f_suffix`.
    ///
    /// Upstream requires room for both characters, so a name already 15 long
    /// gets no suffix at all rather than a truncated one — and then two
    /// components of the same vector share a name.
    pub fn push_vector_suffix(&mut self, idx: u8) {
        let len = self.len as usize;
        if len + 2 > MAX_NAME_SIZE {
            return;
        }
        #[allow(
            clippy::indexing_slicing,
            reason = "len + 2 is checked against the array's length immediately above"
        )]
        {
            self.buf[len] = b'_';
            self.buf[len + 1] = b'X' + idx;
        }
        self.len += 2;
    }

    /// The name as text.
    #[must_use]
    pub fn as_str(&self) -> &str {
        let len = self.len as usize;
        #[allow(
            clippy::indexing_slicing,
            reason = "len never exceeds the array's length, by construction"
        )]
        let bytes = &self.buf[..len];
        core::str::from_utf8(bytes).unwrap_or("")
    }
}

impl core::fmt::Debug for ParamName {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// One parameter as the enumeration yields it.
#[derive(Debug, Clone, Copy)]
pub struct ParamRef {
    /// Full name, prefixes concatenated and truncated.
    pub name: ParamName,
    /// Top-level key.
    pub key: u16,
    /// Position within an array type, upstream's `ParamToken::idx`.
    ///
    /// Zero for a scalar, and for the `Vector3f` itself; 1, 2 and 3 for that
    /// vector's three components.
    pub token_idx: u8,
    /// Eighteen-bit identifier within the key.
    pub group_element: u32,
    /// Type tag.
    pub ptype: u8,
    /// Whether this parameter is reached through a pointer somewhere above it.
    ///
    /// The tables cannot say whether that pointer is null, so a parameter
    /// marked here may not exist on a given vehicle at all: upstream's
    /// enumeration silently omits a whole group when its object was never
    /// allocated. Everything not marked here exists whenever the table does.
    pub behind_pointer: bool,
}

/// Emit a leaf and, if it is a vector, its components.
fn emit_leaf(
    name: ParamName,
    key: u16,
    group_element: u32,
    ptype: u8,
    behind_pointer: bool,
    visit: &mut dyn FnMut(&ParamRef),
) {
    if ptype == VarType::Vector3f.as_u8() {
        // Upstream yields the vector itself first, named as though it were the
        // X component -- `copy_name_token` is called with force_scalar, and the
        // suffix for index 0 is `_X`. Then the three floats.
        for token_idx in 0..4u8 {
            let mut n = name;
            n.push_vector_suffix(token_idx.saturating_sub(1));
            visit(&ParamRef {
                name: n,
                key,
                token_idx,
                group_element,
                ptype: if token_idx == 0 {
                    VarType::Vector3f.as_u8()
                } else {
                    VarType::Float.as_u8()
                },
                behind_pointer,
            });
        }
    } else {
        visit(&ParamRef {
            name,
            key,
            token_idx: 0,
            group_element,
            ptype,
            behind_pointer,
        });
    }
}

/// What stays fixed for the length of one top-level variable's walk.
struct Walk<'v> {
    key: u16,
    filter: EnumFilter,
    visit: &'v mut dyn FnMut(&ParamRef),
}

/// Where the walk currently is.
#[derive(Clone, Copy)]
struct Position {
    prefix: ParamName,
    base: u32,
    shift: u8,
    behind_pointer: bool,
}

fn walk_group(group: &[GroupInfo<'_>], at: Position, w: &mut Walk<'_>) {
    for entry in group {
        if !w.filter.admits(entry.flags) {
            continue;
        }
        let mut here = Position {
            prefix: at.prefix,
            base: group_id(entry.idx, at.base, at.shift, entry.flags),
            shift: at.shift + GROUP_LEVEL_SHIFT,
            behind_pointer: at.behind_pointer || entry.flags & FLAG_POINTER != 0,
        };
        here.prefix.push(entry.name);

        if entry.ptype == VarType::Group.as_u8() {
            // Upstream stops here rather than overflowing group_element, which
            // caps nesting at three levels.
            if at.shift + GROUP_LEVEL_SHIFT >= GROUP_BITS {
                continue;
            }
            if let Some(child) = entry.group {
                walk_group(child, here, w);
            }
        } else {
            emit_leaf(
                here.prefix,
                w.key,
                here.base,
                entry.ptype,
                here.behind_pointer,
                w.visit,
            );
        }
    }
}

/// Walk a descriptor table, yielding every parameter it describes.
///
/// The order matches upstream's `first()`/`next()`: table order, depth first,
/// with a vector immediately followed by its components.
///
/// The filter carries the vehicle's frame mask, upstream's
/// `_frame_type_flags`: entries whose frame bits do not intersect it are
/// skipped, along with everything below them. It also decides whether hidden
/// entries are included — see [`EnumFilter::include_hidden`], which is the one
/// place this deliberately offers something upstream does not.
///
/// A running vehicle can still enumerate fewer parameters than this, because a
/// group reached through a null pointer contributes nothing. Reaching the
/// objects is a later slice; what this settles is the naming and the storage
/// identifier.
pub fn enumerate(table: &[ParamInfo<'_>], filter: EnumFilter, visit: &mut dyn FnMut(&ParamRef)) {
    for info in table {
        if !filter.admits(info.flags) {
            continue;
        }
        let mut name = ParamName::new();
        name.push(info.name);

        let behind_pointer = info.flags & FLAG_POINTER != 0;
        if info.ptype == VarType::Group.as_u8() {
            if let Some(group) = info.group {
                walk_group(
                    group,
                    Position {
                        prefix: name,
                        base: 0,
                        shift: 0,
                        behind_pointer,
                    },
                    &mut Walk {
                        key: info.key,
                        filter,
                        visit,
                    },
                );
            }
        } else {
            emit_leaf(name, info.key, 0, info.ptype, behind_pointer, visit);
        }
    }
}


/// Locate a parameter by name, upstream `AP_Param::find`.
///
/// Returns the scalar or vector container (`token_idx == 0`). Vector
/// components (`_X`, `_Y`, `_Z`) are not matched.
#[must_use]
pub fn find_by_name(table: &[ParamInfo<'_>], filter: EnumFilter, name: &str) -> Option<ParamRef> {
    let mut found = None;
    enumerate(table, filter, &mut |r| {
        if r.token_idx == 0 && r.name.as_str() == name {
            found = Some(*r);
        }
    });
    found
}

#[cfg(test)]
mod find_tests {
    use super::*;
    use crate::VarType;

    static CHILD: [GroupInfo<'static>; 1] = [GroupInfo {
        name: "ALT_MIN",
        idx: 7,
        ptype: VarType::Float.as_u8(),
        flags: 0,
        group: None,
    }];

    static TABLE: [ParamInfo<'static>; 1] = [ParamInfo {
        name: "FENCE_",
        key: 132,
        ptype: VarType::Group.as_u8(),
        flags: 0,
        group: Some(&CHILD),
    }];

    #[test]
    fn find_by_name_resolves_nested_parameter() {
        let f = EnumFilter::for_frame(FRAME_PLANE);
        let r = find_by_name(&TABLE, f, "FENCE_ALT_MIN").expect("found");
        assert_eq!(r.key, 132);
        assert_eq!(r.ptype, VarType::Float.as_u8());
    }

    #[test]
    fn find_by_name_returns_none_for_missing() {
        let f = EnumFilter::for_frame(FRAME_PLANE);
        assert!(find_by_name(&TABLE, f, "NO_SUCH").is_none());
    }
}
