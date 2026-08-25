//! Shared helpers, upstream `libraries/AP_Common`. FW-006.
//!
//! Small things used across the vehicle: bounds checks, hex parsing, a
//! `strncpy` that does not warn, a `mktime` replacement, and IEEE
//! half-precision floats.
//!
//! # What is here, and why the rest is not
//!
//! `Location` is the largest part of AP_Common and lives in [`ap_math`],
//! alongside the coordinate arithmetic it belongs with.
//!
//! The remainder was audited against the fixed-wing path rather than ported
//! wholesale:
//!
//! - `sorting.cpp` — the five `uint16` helpers have exactly one caller,
//!   `AP_CANManager/AP_MAVLinkCAN.cpp`, which is not on the fixed-wing SITL
//!   path. Not ported; revisit if CAN lands.
//! - `ExpandingString.cpp`, `AP_ExpandingArray.h` — both grow heap
//!   allocations, which ADR-0004 rules out. They serve `@SYS` file generation
//!   and parameter dumps, neither of which is flight code.
//! - `c++.cpp` — global `new`/`delete`. No allocator, nothing to port.
//! - `AP_FWVersion.cpp` — build metadata.
//! - `NMEA.cpp` — sentence formatting for the NMEA GPS and rangefinder
//!   backends. SITL uses its own GPS backend.
//! - `Bitmask.h` — no user anywhere in `ArduPlane`, `AP_Param`, `AP_GPS`,
//!   `AP_Baro` or `AP_InertialSensor`.
//! - `AP_Test.h`, `TSIndex.h` — test scaffolding and type-safe index wrappers
//!   with no runtime behaviour to reproduce.
//!
//! # A note on D-015
//!
//! `AP_Common.cpp` opens with
//!
//! ```text
//! static_assert(sizeof(1e6) == sizeof(float),
//!               "Compilation needs to use single-precision constants");
//! ```
//!
//! which is upstream asserting the `-fsingle-precision-constant` behaviour
//! that D-015 documents. The divergence is not an inference from build flags;
//! upstream states the requirement itself and fails the build without it.

#![no_std]

/// Whether `value` lies within an inclusive range, upstream
/// `is_bounded_int32`.
///
/// A reversed range is never satisfied — upstream checks `lower <= upper`
/// first, so `is_bounded_int32(5, 10, 1)` is false rather than an accident.
#[must_use]
pub const fn is_bounded_int32(value: i32, lower_bound: i32, upper_bound: i32) -> bool {
    lower_bound <= upper_bound && value >= lower_bound && value <= upper_bound
}

/// The value of an ASCII hex digit, upstream `hex_to_uint8`.
///
/// `None` for anything that is not `0-9`, `A-F` or `a-f`.
///
/// Upstream works on the nibbles rather than comparing ranges: the high
/// nibble selects the case (`0x30` digits, `0x40`/`0x60` letters) and the low
/// nibble carries the value. It reaches the same answer as a range check and
/// is reproduced in that shape because the boundaries are where a
/// transcription would go wrong — `0x40` is `@` and `0x47` is `G`, both of
/// which must be rejected.
#[must_use]
pub const fn hex_to_uint8(a: u8) -> Option<u8> {
    let nibble_low = a & 0xf;
    match a & 0xf0 {
        0x30 => {
            if nibble_low > 9 {
                None
            } else {
                Some(nibble_low)
            }
        }
        0x40 | 0x60 => {
            if nibble_low == 0 || nibble_low > 6 {
                None
            } else {
                Some(nibble_low + 9)
            }
        }
        _ => None,
    }
}

/// Returned by [`char_to_hex`] for a character that is not a hex digit,
/// upstream's sentinel.
pub const CHAR_TO_HEX_INVALID: u8 = 255;

/// The value of an ASCII hex digit, upstream `char_to_hex`.
///
/// Returns [`CHAR_TO_HEX_INVALID`] rather than `None` because that is
/// upstream's interface and its callers compare against 255 directly. Use
/// [`hex_to_uint8`] where a real option is wanted; the two differ in shape but
/// agree on every input.
#[must_use]
pub const fn char_to_hex(a: u8) -> u8 {
    if a >= b'A' && a <= b'F' {
        a - b'A' + 10
    } else if a >= b'a' && a <= b'f' {
        a - b'a' + 10
    } else if a.is_ascii_digit() {
        a - b'0'
    } else {
        CHAR_TO_HEX_INVALID
    }
}

/// Copy `src` into `dest`, terminating only if there is room. Upstream
/// `strncpy_noterm`.
///
/// Returns the length of `src`, which may exceed what was copied — that is
/// upstream's contract and it is how callers detect truncation.
///
/// The point of the function is that a plain `strncpy` into an exactly-sized
/// field draws a compiler warning for possibly not terminating, even where not
/// terminating is intended. Fixed-width name fields in MAVLink messages and
/// parameter records are exactly that case.
///
/// `dest.len()` plays the role of upstream's `n`.
pub fn strncpy_noterm(dest: &mut [u8], src: &[u8]) -> usize {
    let n = dest.len();
    // strnlen: bytes before the first nul, capped at n.
    let len = src
        .iter()
        .take(n)
        .position(|&b| b == 0)
        .unwrap_or(if src.len() < n { src.len() } else { n });

    let copy = if len < n { len + 1 } else { len };
    for i in 0..copy {
        // The nul is only written when it fits, which is the `len < n` case.
        let byte = if i < len {
            *src.get(i).unwrap_or(&0)
        } else {
            0
        };
        if let Some(d) = dest.get_mut(i) {
            *d = byte;
        }
    }
    len
}

/// A broken-down calendar time, upstream's `struct tm` fields that
/// [`ap_mktime`] reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Tm {
    /// Years since 1900. Values below 70 are rejected.
    pub year: i32,
    /// Month, 0–11.
    pub mon: i32,
    /// Day of month, 1–31.
    pub mday: i32,
    /// Hours since midnight, 0–23.
    pub hour: i32,
    /// Minutes, 0–59.
    pub min: i32,
    /// Seconds, 0–60.
    pub sec: i32,
}

const MINUTE: i64 = 60;
const HOUR: i64 = 60 * MINUTE;
const DAY: i64 = 24 * HOUR;
const YEAR: i64 = 365 * DAY;

/// Seconds since the Unix epoch for a UTC calendar time, upstream
/// `ap_mktime`.
///
/// `None` where upstream returns `(time_t)-1`: a year before 1970, which the
/// epoch cannot express.
///
/// Upstream carries a `mktime` replacement from Samba because the C library's
/// is locale- and timezone-aware and a flight controller wants neither — GPS
/// time is UTC and must convert the same way everywhere.
///
/// DIVERGENCE D-022: upstream computes the year term as `(tm_year - 70) *
/// YEAR` where `YEAR` is an `unsigned`, so the multiplication is done in 32
/// bits and wraps for years past about 2106. This computes in 64 bits. The two
/// agree for every date a vehicle will see.
#[must_use]
pub fn ap_mktime(t: &Tm) -> Option<i64> {
    if t.year < 70 {
        return None;
    }

    let n = i64::from(t.year) + 1900 - 1;
    // Leap days between 1970 and the target year, as a difference of two
    // Gregorian leap counts.
    let leaps = (n / 4 - n / 100 + n / 400) - (1969 / 4 - 1969 / 100 + 1969 / 400);
    let mut epoch = (i64::from(t.year) - 70) * YEAR + leaps * DAY;

    const MON: [i64; 12] = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
    let y = i64::from(t.year) + 1900;
    for m in 0..t.mon.clamp(0, 12) {
        let idx = usize::try_from(m).unwrap_or(0);
        epoch += MON.get(idx).copied().unwrap_or(0) * DAY;
        // February of a leap year gets its extra day here. Upstream also
        // advances a year counter when the month index wraps, but `tm_mon` is
        // at most 11 so it never does — the wrap is unreachable.
        if m == 1 && y % 4 == 0 && (y % 100 != 0 || y % 400 == 0) {
            epoch += DAY;
        }
    }

    epoch += (i64::from(t.mday) - 1) * DAY;
    epoch += i64::from(t.hour) * HOUR + i64::from(t.min) * MINUTE + i64::from(t.sec);

    Some(epoch)
}

/// IEEE half-precision float, upstream `Float16_t`.
///
/// Half of a `f32`'s width for a tenth of its precision, which is a good trade
/// for logged telemetry and mission item payloads where the value is a reading
/// rather than something to compute with. This is IEEE binary16, **not**
/// bfloat16 — the two have the same width and different exponent ranges, and
/// confusing them silently rescales everything.
///
/// The conversion algorithm is upstream's, from libcanard. Upstream punts
/// through a union; this uses `to_bits`/`from_bits`, which is the same
/// reinterpretation with defined behaviour.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Float16 {
    /// The raw 16-bit pattern, upstream `v16`.
    pub bits: u16,
}

impl Float16 {
    /// Wrap a raw bit pattern.
    #[must_use]
    pub const fn from_bits(bits: u16) -> Self {
        Self { bits }
    }

    /// The `f32` this represents, upstream `get`.
    #[must_use]
    pub fn get(self) -> f32 {
        let magic = f32::from_bits((254_u32 - 15) << 23);
        let was_inf_nan = f32::from_bits((127_u32 + 16) << 23);

        let mut out_u = u32::from(self.bits & 0x7FFF) << 13;
        let mut out_f = f32::from_bits(out_u) * magic;
        if out_f >= was_inf_nan {
            out_u = out_f.to_bits() | (255_u32 << 23);
            out_f = f32::from_bits(out_u);
        }
        out_u = out_f.to_bits() | (u32::from(self.bits & 0x8000) << 16);
        f32::from_bits(out_u)
    }

    /// Convert from an `f32`, upstream `set`.
    #[must_use]
    pub fn set(value: f32) -> Self {
        let f32inf = 255_u32 << 23;
        let f16inf = 31_u32 << 23;
        let magic = f32::from_bits(15_u32 << 23);
        let sign_mask = 0x8000_0000_u32;
        let round_mask = 0xFFFF_F000_u32;

        let mut in_u = value.to_bits();
        let sign = in_u & sign_mask;
        in_u ^= sign;

        let mut v16: u16;

        if in_u >= f32inf {
            // Infinity stays infinity; anything larger is a NaN and becomes
            // the canonical half-precision NaN.
            v16 = if in_u > f32inf { 0x7FFF } else { 0x7C00 };
        } else {
            in_u &= round_mask;
            let scaled = f32::from_bits(in_u) * magic;
            in_u = scaled.to_bits();
            in_u = in_u.wrapping_add(0x1000);

            if in_u > f16inf {
                in_u = f16inf;
            }
            #[allow(
                clippy::cast_possible_truncation,
                reason = "the shift leaves at most 16 significant bits, which is the \
representation being built"
            )]
            {
                v16 = (in_u >> 13) as u16;
            }
        }

        #[allow(
            clippy::cast_possible_truncation,
            reason = "the sign bit is moved into bit 15 of a u16 by construction"
        )]
        {
            v16 |= (sign >> 16) as u16;
        }
        Self { bits: v16 }
    }
}

impl From<f32> for Float16 {
    fn from(v: f32) -> Self {
        Self::set(v)
    }
}

impl From<Float16> for f32 {
    fn from(v: Float16) -> Self {
        v.get()
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "half precision either round-trips exactly or it is broken; an \nepsilon here would accept a conversion that had quietly lost a bit, which is the \nonly failure these tests are looking for"
    )]

    use super::*;

    #[test]
    fn bounds_are_inclusive_and_a_reversed_range_is_empty() {
        assert!(is_bounded_int32(5, 1, 10));
        assert!(is_bounded_int32(1, 1, 10), "inclusive at the bottom");
        assert!(is_bounded_int32(10, 1, 10), "inclusive at the top");
        assert!(!is_bounded_int32(0, 1, 10));
        assert!(!is_bounded_int32(11, 1, 10));
        assert!(
            !is_bounded_int32(5, 10, 1),
            "a reversed range should never be satisfied"
        );
    }

    /// The boundaries are where a hex parser goes wrong: `@` is `0x40` and
    /// `G` is `0x47`, either side of `A`–`F`.
    #[test]
    fn hex_parsing_rejects_the_neighbours_of_the_valid_ranges() {
        assert_eq!(hex_to_uint8(b'0'), Some(0));
        assert_eq!(hex_to_uint8(b'9'), Some(9));
        assert_eq!(hex_to_uint8(b'A'), Some(10));
        assert_eq!(hex_to_uint8(b'F'), Some(15));
        assert_eq!(hex_to_uint8(b'a'), Some(10));
        assert_eq!(hex_to_uint8(b'f'), Some(15));

        assert_eq!(hex_to_uint8(b'@'), None, "0x40, just below A");
        assert_eq!(hex_to_uint8(b'G'), None, "0x47, just above F");
        assert_eq!(hex_to_uint8(b'`'), None, "0x60, just below a");
        assert_eq!(hex_to_uint8(b'g'), None, "0x67, just above f");
        assert_eq!(hex_to_uint8(b'/'), None);
        assert_eq!(hex_to_uint8(b':'), None);
        assert_eq!(hex_to_uint8(b' '), None);
    }

    /// Two functions, two shapes, one answer. If they ever disagree one of
    /// them has been transcribed wrong.
    #[test]
    fn the_two_hex_parsers_agree_on_every_byte() {
        for a in 0..=255_u8 {
            let by_nibble = hex_to_uint8(a);
            let by_range = char_to_hex(a);
            match by_nibble {
                Some(v) => assert_eq!(by_range, v, "byte {a:#04x}"),
                None => assert_eq!(by_range, CHAR_TO_HEX_INVALID, "byte {a:#04x}"),
            }
        }
    }

    #[test]
    fn strncpy_noterm_terminates_only_when_there_is_room() {
        let mut dest = [0xAA_u8; 8];
        let len = strncpy_noterm(&mut dest, b"abc\0");
        assert_eq!(len, 3, "the return is the source length");
        assert_eq!(&dest[..4], b"abc\0", "it fits, so it terminates");
        assert_eq!(dest[4], 0xAA, "and nothing beyond is touched");
    }

    #[test]
    fn strncpy_noterm_fills_an_exact_field_without_terminating() {
        let mut dest = [0xAA_u8; 3];
        let len = strncpy_noterm(&mut dest, b"abc\0");
        assert_eq!(len, 3);
        assert_eq!(&dest, b"abc", "exactly full, so no nul");
    }

    #[test]
    fn strncpy_noterm_reports_the_untruncated_length() {
        let mut dest = [0_u8; 3];
        let len = strncpy_noterm(&mut dest, b"abcdef\0");
        assert_eq!(len, 3, "capped at n — how callers see truncation");
        assert_eq!(&dest, b"abc");
    }

    /// The epoch itself, and the two boundaries either side of it.
    #[test]
    fn the_epoch_converts_to_zero() {
        let t = Tm {
            year: 70,
            mon: 0,
            mday: 1,
            hour: 0,
            min: 0,
            sec: 0,
        };
        assert_eq!(ap_mktime(&t), Some(0));
    }

    #[test]
    fn a_year_before_the_epoch_has_no_answer() {
        let t = Tm {
            year: 69,
            mon: 11,
            mday: 31,
            ..Tm::default()
        };
        assert_eq!(ap_mktime(&t), None);
    }

    /// Known instants, checked against values that can be verified
    /// independently.
    #[test]
    fn known_instants_convert_correctly() {
        // 2000-01-01 00:00:00 UTC
        let y2k = Tm {
            year: 100,
            mon: 0,
            mday: 1,
            hour: 0,
            min: 0,
            sec: 0,
        };
        assert_eq!(ap_mktime(&y2k), Some(946_684_800));

        // 2024-02-29 12:34:56 UTC — a leap day, which is where a calendar
        // routine fails if it fails at all.
        let leap = Tm {
            year: 124,
            mon: 1,
            mday: 29,
            hour: 12,
            min: 34,
            sec: 56,
        };
        assert_eq!(ap_mktime(&leap), Some(1_709_210_096));
    }

    /// 1900 is not a leap year and 2000 is — the century rule, which a naive
    /// `year % 4` gets wrong once every hundred years.
    #[test]
    fn the_century_leap_rule_is_applied() {
        // 2000-03-01 minus 2000-02-28 should be two days (Feb 29 exists).
        let feb28 = Tm {
            year: 100,
            mon: 1,
            mday: 28,
            ..Tm::default()
        };
        let mar01 = Tm {
            year: 100,
            mon: 2,
            mday: 1,
            ..Tm::default()
        };
        let d = ap_mktime(&mar01).expect("valid") - ap_mktime(&feb28).expect("valid");
        assert_eq!(d, 2 * 86400, "2000 is a leap year");

        // 2100 is not, so the same span is one day.
        let feb28 = Tm {
            year: 200,
            mon: 1,
            mday: 28,
            ..Tm::default()
        };
        let mar01 = Tm {
            year: 200,
            mon: 2,
            mday: 1,
            ..Tm::default()
        };
        let d = ap_mktime(&mar01).expect("valid") - ap_mktime(&feb28).expect("valid");
        assert_eq!(d, 86400, "2100 is not a leap year");
    }

    /// Half-precision round trips exactly for values it can represent.
    #[test]
    fn float16_round_trips_representable_values() {
        for v in [0.0_f32, 1.0, -1.0, 0.5, -0.5, 2.0, 1024.0, -1024.0, 0.25] {
            let back = Float16::set(v).get();
            assert_eq!(back, v, "{v} did not survive the round trip");
        }
    }

    /// And loses precision predictably for values it cannot.
    #[test]
    fn float16_loses_precision_gracefully() {
        let v = 1.0_f32 / 3.0;
        let back = Float16::set(v).get();
        assert!(
            (back - v).abs() < 1e-3,
            "{v} came back as {back}, which is too far even for half precision"
        );
        assert_ne!(
            back, v,
            "it should not be exact — that would mean no conversion"
        );
    }

    /// Infinity and NaN have to survive, because a logged reading that has
    /// gone bad must stay visibly bad rather than becoming a large number.
    #[test]
    fn float16_preserves_infinity_and_nan() {
        assert!(Float16::set(f32::INFINITY).get().is_infinite());
        assert!(Float16::set(f32::INFINITY).get() > 0.0);
        assert!(Float16::set(f32::NEG_INFINITY).get().is_infinite());
        assert!(Float16::set(f32::NEG_INFINITY).get() < 0.0);
        assert!(Float16::set(f32::NAN).get().is_nan());
    }

    /// Values past half-precision's range saturate to infinity rather than
    /// wrapping.
    #[test]
    fn float16_saturates_beyond_its_range() {
        // binary16's largest finite value is 65504.
        let big = Float16::set(1.0e6).get();
        assert!(big.is_infinite(), "1e6 should saturate, got {big}");
        assert!(Float16::set(65504.0).get().is_finite());
    }

    /// Signed zero is a distinct pattern and must not be flattened.
    #[test]
    fn float16_keeps_the_sign_of_zero() {
        assert_eq!(Float16::set(-0.0_f32).bits & 0x8000, 0x8000);
        assert!(Float16::set(-0.0_f32).get().is_sign_negative());
    }
}
