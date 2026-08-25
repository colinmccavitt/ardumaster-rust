//! The storage write path, upstream `AP_Param::save_sync`. FW-004 slice 3b.
//!
//! Storage is an append log, not an array. Every parameter that has ever been
//! saved sits somewhere in it as a header followed by a value, terminated by a
//! sentinel, and finding one means walking from the start. Saving an existing
//! parameter overwrites its value in place; saving a new one appends at the
//! sentinel and moves the sentinel along.
//!
//! # Why the header is written last
//!
//! An append is three writes, and upstream orders them deliberately:
//!
//! 1. a **new sentinel**, past where the record will end
//! 2. the **value**
//! 3. the **header**
//!
//! Until the header lands, the old sentinel is still the first thing a scan
//! meets at that offset, so the record is invisible. Lose power after step one
//! or two and storage reads exactly as it did before, with some unreferenced
//! bytes past the sentinel that the next append overwrites. Lose power after
//! step three and the record is complete. There is no ordering that leaves a
//! header pointing at a value that was never written — which is the failure
//! that would silently hand the vehicle a garbage parameter on the next boot.
//!
//! # Defaults are not stored
//!
//! A parameter equal to its default is not written at all, which is what keeps
//! storage from filling with values nobody set. Upstream extends that to a
//! *near*-default rule: for everything except `int32`, a value within 0.01% of
//! the default counts as the default. See [`save`] for what that does to large
//! integers.

use ap_math::scalar::is_equal;

use crate::storage::{ParamValue, Storage, StorageError};
use crate::{ParamHeader, VarType, EEPROM_HEADER_SIZE, PARAM_HEADER_SIZE};

/// Where a scan for a parameter ended, upstream `AP_Param::scan`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScanResult {
    /// An existing record starts here. Its value can be overwritten in place.
    Found(u16),
    /// No record; this is where the sentinel sits and where an append goes.
    Sentinel(u16),
    /// The walk ran off the end of storage without meeting a sentinel, which
    /// means storage is corrupt or was never formatted. Upstream reports this
    /// as an offset of `0xffff` and treats it as full.
    PastEnd,
}

/// Walk storage looking for a parameter, upstream `AP_Param::scan`.
///
/// Records are variable width — the header's type says how many value bytes
/// follow — so there is no way to seek. A parameter late in a full storage
/// costs a walk over everything before it, which is why upstream caches the
/// sentinel offset.
pub fn scan<S: Storage + ?Sized>(storage: &S, target: ParamHeader) -> ScanResult {
    let mut ofs = EEPROM_HEADER_SIZE as u16;
    let size = storage.size();

    while ofs < size {
        let mut bytes = [0_u8; PARAM_HEADER_SIZE];
        if !storage.read(ofs, &mut bytes) {
            return ScanResult::PastEnd;
        }
        let phdr = ParamHeader::from_bytes(bytes);

        if phdr.var_type == target.var_type
            && phdr.key == target.key
            && phdr.group_element == target.group_element
        {
            return ScanResult::Found(ofs);
        }

        if phdr.is_sentinel() {
            return ScanResult::Sentinel(ofs);
        }

        // A type the format does not define would leave the walk with no way
        // to know the record's width. Upstream's type_size returns 0 there and
        // the loop spins on the same offset forever; stopping is the only
        // honest thing to do.
        let Some(t) = VarType::from_u8(phdr.var_type) else {
            return ScanResult::PastEnd;
        };
        let step = u16::from(t.size()) + PARAM_HEADER_SIZE as u16;
        if step == PARAM_HEADER_SIZE as u16 {
            // A zero-width type (None or Group) never appears as a stored
            // record; treating it as progress would still be a walk that
            // cannot terminate on a corrupt byte.
            return ScanResult::PastEnd;
        }
        ofs = ofs.saturating_add(step);
    }

    ScanResult::PastEnd
}

/// What [`save`] did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SaveOutcome {
    /// An existing record's value was overwritten in place.
    Updated,
    /// A new record was appended and the sentinel moved past it.
    Appended,
    /// The value equals its default, so nothing was written.
    AtDefault,
    /// The value is within 0.01% of its default, which upstream treats as the
    /// same. See [`save`].
    NearDefault,
    /// No room for another record.
    Full,
}

/// Bytes a value occupies in storage.
fn value_size(value: ParamValue) -> u16 {
    u16::from(value.var_type().size())
}

/// Serialise a value little-endian, as upstream's memcpy from the live
/// variable does on every platform ArduPilot targets.
fn value_bytes(value: ParamValue) -> ([u8; 12], usize) {
    let mut out = [0_u8; 12];

    /// Copy `src` to `out` at `at`, doing nothing if it would not fit. The
    /// buffer is sized for the largest value, so it always fits.
    fn put(out: &mut [u8; 12], at: usize, src: &[u8]) {
        if let Some(dst) = out.get_mut(at..at + src.len()) {
            dst.copy_from_slice(src);
        }
    }

    match value {
        ParamValue::Int8(v) => {
            put(&mut out, 0, &v.to_le_bytes());
            (out, 1)
        }
        ParamValue::Int16(v) => {
            put(&mut out, 0, &v.to_le_bytes());
            (out, 2)
        }
        ParamValue::Int32(v) => {
            put(&mut out, 0, &v.to_le_bytes());
            (out, 4)
        }
        ParamValue::Float(v) => {
            put(&mut out, 0, &v.to_le_bytes());
            (out, 4)
        }
        ParamValue::Vector3f(v) => {
            for (i, c) in v.iter().enumerate() {
                put(&mut out, i * 4, &c.to_le_bytes());
            }
            (out, 12)
        }
    }
}

/// Write the sentinel at an offset, upstream `AP_Param::write_sentinel`.
pub fn write_sentinel<S: Storage + ?Sized>(storage: &mut S, ofs: u16) -> Result<(), StorageError> {
    let bytes = ParamHeader::sentinel().to_bytes();
    if storage.write(ofs, &bytes) {
        Ok(())
    } else {
        Err(StorageError::WriteFailed { offset: ofs })
    }
}

/// Save one parameter, upstream `AP_Param::save_sync`.
///
/// `default` is the value from the descriptor table; pass `None` for a
/// parameter with no default, which skips the not-worth-saving checks
/// entirely. `force` is upstream's `force_save` and bypasses them too.
///
/// # The near-default rule and large integers
///
/// Upstream skips the write when
///
/// ```text
/// !force_save && type != AP_PARAM_INT32 && fabsf(v1-v2) < 0.0001f*fabsf(v1)
/// ```
///
/// with the comment "for other than 32 bit integers, we accept values within
/// 0.01 percent of the current value as being the same". For floats that is
/// the intent. For `int8` it can never fire — a tolerance of 0.0001×127 is far
/// below 1. For `int16` it *can*: at a magnitude of 10,000 the tolerance
/// reaches 1, so a value differing from its default by one would not be
/// written and would read back as the default on the next boot.
///
/// Whether any fixed-wing `int16` parameter actually reaches that magnitude is
/// not established here, so this is reproduced rather than diverged from. It
/// is recorded because it is the kind of thing that looks like a rounding
/// nicety and is not.
///
/// Note also that the tolerance is relative to `v1`, the value being saved,
/// not to the default. Saving zero over a non-zero default therefore always
/// writes, because the tolerance collapses to nothing.
pub fn save<S: Storage + ?Sized>(
    storage: &mut S,
    header: ParamHeader,
    value: ParamValue,
    default: Option<f32>,
    force: bool,
) -> Result<SaveOutcome, StorageError> {
    let vsize = value_size(value);
    let (bytes, n) = value_bytes(value);

    match scan(storage, header) {
        ScanResult::Found(ofs) => {
            // In place: the header is already correct and the value is the
            // only thing that changes. No sentinel move, no default check —
            // upstream returns before reaching it.
            let at = ofs + PARAM_HEADER_SIZE as u16;
            if !storage.write(at, bytes.get(..n).unwrap_or(&[])) {
                return Err(StorageError::WriteFailed { offset: at });
            }
            Ok(SaveOutcome::Updated)
        }

        ScanResult::PastEnd => Ok(SaveOutcome::Full),

        ScanResult::Sentinel(ofs) => {
            // Only scalars are checked against their default; upstream guards
            // with `phdr.type <= AP_PARAM_FLOAT`, so a Vector3f is always
            // written.
            let is_scalar = matches!(
                value,
                ParamValue::Int8(_)
                    | ParamValue::Int16(_)
                    | ParamValue::Int32(_)
                    | ParamValue::Float(_)
            );

            if let (Some(v2), true, false) = (default, is_scalar, force) {
                let v1 = value.as_f32();
                if is_equal(v1, v2) {
                    return Ok(SaveOutcome::AtDefault);
                }
                if value.var_type() != VarType::Int32 && (v1 - v2).abs() < 0.0001 * v1.abs() {
                    return Ok(SaveOutcome::NearDefault);
                }
            }

            // Room for this record and the sentinel that follows it.
            let need = u32::from(ofs) + u32::from(vsize) + 2 * PARAM_HEADER_SIZE as u32;
            if need >= u32::from(storage.size()) {
                return Ok(SaveOutcome::Full);
            }

            // Sentinel, value, header — in that order. See the module docs.
            write_sentinel(storage, ofs + PARAM_HEADER_SIZE as u16 + vsize)?;
            let value_at = ofs + PARAM_HEADER_SIZE as u16;
            if !storage.write(value_at, bytes.get(..n).unwrap_or(&[])) {
                return Err(StorageError::WriteFailed { offset: value_at });
            }
            if !storage.write(ofs, &header.to_bytes()) {
                return Err(StorageError::WriteFailed { offset: ofs });
            }
            Ok(SaveOutcome::Appended)
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::indexing_slicing,
        reason = "these index a fixed-size test buffer at offsets the test itself \ncomputed; in a test an index fault is a test failure, which is the desired outcome"
    )]

    use super::*;
    use crate::{EEPROM_MAGIC, EEPROM_REVISION};

    /// A storage backed by a fixed array, formatted the way upstream's
    /// `erase_all` leaves it: an eeprom header then an immediate sentinel.
    struct Ram {
        bytes: [u8; 256],
    }

    impl Ram {
        fn formatted() -> Self {
            let mut s = Self { bytes: [0xFF; 256] };
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
        fn size(&self) -> u16 {
            self.bytes.len() as u16
        }
        fn read(&self, offset: u16, buf: &mut [u8]) -> bool {
            let start = offset as usize;
            let Some(src) = self.bytes.get(start..start + buf.len()) else {
                return false;
            };
            buf.copy_from_slice(src);
            true
        }
        fn write(&mut self, offset: u16, data: &[u8]) -> bool {
            let start = offset as usize;
            let Some(dst) = self.bytes.get_mut(start..start + data.len()) else {
                return false;
            };
            dst.copy_from_slice(data);
            true
        }
    }

    fn hdr(key: u16, t: VarType, group: u32) -> ParamHeader {
        ParamHeader::new(key, t.as_u8(), group)
    }

    #[test]
    fn a_fresh_storage_scans_straight_to_the_sentinel() {
        let s = Ram::formatted();
        assert_eq!(
            scan(&s, hdr(7, VarType::Float, 0)),
            ScanResult::Sentinel(EEPROM_HEADER_SIZE as u16)
        );
    }

    #[test]
    fn appending_writes_header_value_and_a_new_sentinel() {
        let mut s = Ram::formatted();
        let h = hdr(7, VarType::Float, 0);
        assert_eq!(
            save(&mut s, h, ParamValue::Float(1.5), None, false),
            Ok(SaveOutcome::Appended)
        );

        // header at 4, value at 8, sentinel at 12
        assert_eq!(&s.bytes[4..8], &h.to_bytes());
        assert_eq!(&s.bytes[8..12], &1.5_f32.to_le_bytes());
        assert_eq!(&s.bytes[12..16], &ParamHeader::sentinel().to_bytes());
    }

    #[test]
    fn a_saved_parameter_can_be_found_again() {
        let mut s = Ram::formatted();
        let h = hdr(7, VarType::Float, 0);
        save(&mut s, h, ParamValue::Float(1.5), None, false).expect("save");
        assert_eq!(scan(&s, h), ScanResult::Found(4));
    }

    /// Re-saving overwrites the value where it already is, and does not append
    /// a second copy — which is what stops a parameter tweaked in flight from
    /// eating storage.
    #[test]
    fn resaving_updates_in_place() {
        let mut s = Ram::formatted();
        let h = hdr(7, VarType::Float, 0);
        save(&mut s, h, ParamValue::Float(1.5), None, false).expect("save");
        let sentinel_before = [s.bytes[12], s.bytes[13], s.bytes[14], s.bytes[15]];

        assert_eq!(
            save(&mut s, h, ParamValue::Float(-2.25), None, false),
            Ok(SaveOutcome::Updated)
        );
        assert_eq!(&s.bytes[8..12], &(-2.25_f32).to_le_bytes());
        assert_eq!(&s.bytes[12..16], &sentinel_before, "no new record");
    }

    /// Several parameters chain, each starting where the last one's sentinel
    /// was.
    #[test]
    fn parameters_append_in_sequence() {
        let mut s = Ram::formatted();
        let a = hdr(1, VarType::Int8, 0);
        let b = hdr(2, VarType::Int16, 0);
        let c = hdr(3, VarType::Vector3f, 0);

        save(&mut s, a, ParamValue::Int8(-3), None, false).expect("a");
        save(&mut s, b, ParamValue::Int16(1234), None, false).expect("b");
        save(
            &mut s,
            c,
            ParamValue::Vector3f([1.0, 2.0, 3.0]),
            None,
            false,
        )
        .expect("c");

        assert_eq!(scan(&s, a), ScanResult::Found(4));
        assert_eq!(scan(&s, b), ScanResult::Found(4 + 4 + 1));
        assert_eq!(scan(&s, c), ScanResult::Found(4 + 4 + 1 + 4 + 2));
    }

    /// A value equal to its default is not stored at all.
    #[test]
    fn a_default_value_is_not_written() {
        let mut s = Ram::formatted();
        let h = hdr(7, VarType::Float, 0);
        assert_eq!(
            save(&mut s, h, ParamValue::Float(2.5), Some(2.5), false),
            Ok(SaveOutcome::AtDefault)
        );
        assert_eq!(
            scan(&s, h),
            ScanResult::Sentinel(4),
            "nothing should have been appended"
        );
    }

    /// Unless forced, which is how a parameter is pinned at its default
    /// against a later change of that default.
    #[test]
    fn forcing_writes_a_default_value_anyway() {
        let mut s = Ram::formatted();
        let h = hdr(7, VarType::Float, 0);
        assert_eq!(
            save(&mut s, h, ParamValue::Float(2.5), Some(2.5), true),
            Ok(SaveOutcome::Appended)
        );
    }

    /// The 0.01% rule, on a float, where it is intended.
    #[test]
    fn a_value_within_a_hundredth_of_a_percent_counts_as_default() {
        let mut s = Ram::formatted();
        let h = hdr(7, VarType::Float, 0);
        // 1000.0 against a default of 1000.05: 0.05 < 0.0001*1000 = 0.1
        assert_eq!(
            save(&mut s, h, ParamValue::Float(1000.0), Some(1000.05), false),
            Ok(SaveOutcome::NearDefault)
        );
        // 1000.0 against 1000.5: 0.5 > 0.1, so it is written
        assert_eq!(
            save(&mut s, h, ParamValue::Float(1000.0), Some(1000.5), false),
            Ok(SaveOutcome::Appended)
        );
    }

    /// int32 is exempt from the near-default rule, so a difference of one is
    /// always written.
    #[test]
    fn int32_is_exempt_from_the_near_default_rule() {
        let mut s = Ram::formatted();
        let h = hdr(7, VarType::Int32, 0);
        assert_eq!(
            save(
                &mut s,
                h,
                ParamValue::Int32(1_000_000),
                Some(1_000_001.0),
                false
            ),
            Ok(SaveOutcome::Appended),
            "int32 must be written even one apart"
        );
    }

    /// int16 is NOT exempt, and at a large magnitude the tolerance reaches a
    /// whole count. This is upstream's behaviour, reproduced deliberately —
    /// see the note on `save`.
    #[test]
    fn a_large_int16_one_from_its_default_is_treated_as_default() {
        let mut s = Ram::formatted();
        let h = hdr(7, VarType::Int16, 0);
        // 0.0001 * 20000 = 2, and |20000 - 20001| = 1 < 2
        assert_eq!(
            save(&mut s, h, ParamValue::Int16(20_000), Some(20_001.0), false),
            Ok(SaveOutcome::NearDefault),
            "upstream's 0.01 percent rule swallows this"
        );
        // Below the crossover it behaves as expected.
        assert_eq!(
            save(&mut s, h, ParamValue::Int16(5_000), Some(5_001.0), false),
            Ok(SaveOutcome::Appended)
        );
    }

    /// The 0.01 percent tolerance is relative to the value being *saved*, not
    /// to the default, so it collapses to nothing at zero and a zero is always
    /// written.
    ///
    /// The default here has to sit outside `is_equal`'s epsilon, or the first
    /// check catches it and the tolerance is never consulted — which is what
    /// the first version of this test actually measured.
    #[test]
    fn saving_zero_over_a_nonzero_default_always_writes() {
        let mut s = Ram::formatted();
        let h = hdr(7, VarType::Float, 0);
        assert_eq!(
            save(&mut s, h, ParamValue::Float(0.0), Some(0.5), false),
            Ok(SaveOutcome::Appended),
            "the tolerance collapses to nothing at zero"
        );
    }

    /// And a default within epsilon is caught by the equality check first,
    /// before the tolerance is reached at all.
    #[test]
    fn an_epsilon_close_default_is_caught_by_the_equality_check() {
        let mut s = Ram::formatted();
        let h = hdr(7, VarType::Float, 0);
        assert_eq!(
            save(&mut s, h, ParamValue::Float(0.0), Some(1e-9), false),
            Ok(SaveOutcome::AtDefault)
        );
    }

    /// A vector is never checked against a default; upstream's guard stops at
    /// float.
    #[test]
    fn a_vector_is_always_written() {
        let mut s = Ram::formatted();
        let h = hdr(7, VarType::Vector3f, 0);
        assert_eq!(
            save(&mut s, h, ParamValue::Vector3f([0.0; 3]), Some(0.0), false),
            Ok(SaveOutcome::Appended)
        );
    }

    /// Running out of room is reported rather than corrupting the tail.
    #[test]
    fn a_full_storage_refuses_to_append() {
        let mut s = Ram::formatted();
        let mut appended = 0;
        for key in 0..200_u16 {
            match save(
                &mut s,
                hdr(key, VarType::Int32, 0),
                ParamValue::Int32(i32::from(key)),
                None,
                false,
            ) {
                Ok(SaveOutcome::Appended) => appended += 1,
                Ok(SaveOutcome::Full) => break,
                other => panic!("unexpected {other:?}"),
            }
        }
        assert!(appended > 20, "should fit a good few first, got {appended}");
        assert_eq!(
            save(
                &mut s,
                hdr(999, VarType::Int32, 0),
                ParamValue::Int32(1),
                None,
                false
            ),
            Ok(SaveOutcome::Full)
        );

        // Everything written is still findable — the tail was not scribbled on.
        for key in 0..appended {
            assert!(matches!(
                scan(&s, hdr(key, VarType::Int32, 0)),
                ScanResult::Found(_)
            ));
        }
    }

    /// The crash-safety property, stated directly: after the sentinel and the
    /// value are written but before the header is, storage still reads exactly
    /// as it did before.
    #[test]
    fn a_half_written_append_is_invisible() {
        let mut s = Ram::formatted();
        let existing = hdr(1, VarType::Float, 0);
        save(&mut s, existing, ParamValue::Float(9.0), None, false).expect("setup");

        let before = s.bytes;
        let ofs = match scan(&s, hdr(2, VarType::Float, 0)) {
            ScanResult::Sentinel(o) => o,
            other => panic!("expected a sentinel, got {other:?}"),
        };

        // Steps one and two of the append, without step three.
        write_sentinel(&mut s, ofs + PARAM_HEADER_SIZE as u16 + 4).expect("sentinel");
        s.write(ofs + PARAM_HEADER_SIZE as u16, &7.5_f32.to_le_bytes());

        // The old sentinel is still at `ofs`, so the scan stops there and the
        // new record does not exist.
        assert_eq!(
            scan(&s, hdr(2, VarType::Float, 0)),
            ScanResult::Sentinel(ofs)
        );
        assert_eq!(
            scan(&s, existing),
            ScanResult::Found(4),
            "and the old one is intact"
        );

        // Only bytes at or past the old sentinel were touched.
        assert_eq!(&s.bytes[..ofs as usize], &before[..ofs as usize]);
    }
}
