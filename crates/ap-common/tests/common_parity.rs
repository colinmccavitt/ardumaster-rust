//! Parity test: `AP_Common` helpers against upstream's own implementations.
//!
//! The half-precision conversion is swept **exhaustively** — all 65,536 bit
//! patterns through `get()`. That is the only way to be sure about subnormals,
//! the infinity boundary and the NaN canonicalisation, which is exactly where
//! a hand-transcribed bit-twiddling routine goes wrong and where a sampled
//! test would sail past.
//!
//! `hex_to_uint8` and `char_to_hex` are swept over all 256 input bytes, and
//! `is_bounded_int32` over a grid that includes reversed ranges.
//!
//! `ap_mktime` covers the epoch, the boundary below it, leap years, the
//! century rule at 2000 and 2100, and the 32-bit `time_t` rollover.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_common::{
    ap_mktime, char_to_hex, hex_to_uint8, is_bounded_int32, Float16, Tm, CHAR_TO_HEX_INVALID,
};

fn fixture() -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/common_parity.csv"))
        .expect("workspace root");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_common_fixture.py",
            path.display()
        )
    })
}

fn same(a: f32, b: f32) -> bool {
    (a.is_nan() && b.is_nan()) || a.to_bits() == b.to_bits()
}

#[test]
fn the_common_helpers_match_upstream() {
    let text = fixture();

    let mut f16_get = 0_usize;
    let mut f16_set = 0_usize;
    let mut hex = 0_usize;
    let mut bounded = 0_usize;
    let mut mktimes = 0_usize;
    let mut wrapped = 0_usize;

    // ap_mktime rows come in pairs: the date row, then its time-of-day row.
    let mut pending_date: Option<(i32, i32, i32, i64)> = None;

    for line in text.lines() {
        if line.is_empty() || line.starts_with('#') || line.starts_with("kind,") {
            continue;
        }
        let row: Vec<&str> = line.split(',').collect();
        assert_eq!(row.len(), 5, "malformed row: {line}");

        match row[0] {
            "f16_get" => {
                let bits: u16 = row[1].parse().expect("bit pattern");
                let want = f32::from_bits(row[4].parse::<u32>().expect("float bits"));
                let got = Float16::from_bits(bits).get();
                assert!(
                    same(got, want),
                    "f16 get({bits:#06x}): {got:e} ({:#010x}) != upstream {want:e} ({:#010x})",
                    got.to_bits(),
                    want.to_bits()
                );
                f16_get += 1;
            }
            "f16_set" => {
                let input = f32::from_bits(row[1].parse::<u32>().expect("float bits"));
                let want: u16 = row[4].parse().expect("half bits");
                let got = Float16::set(input).bits;
                assert_eq!(
                    got, want,
                    "f16 set({input:e}): {got:#06x} != upstream {want:#06x}"
                );
                f16_set += 1;
            }
            "hex_to_uint8" => {
                let a: u8 = row[1].parse::<u32>().expect("byte") as u8;
                let want: i32 = row[4].parse().expect("result");
                let got = hex_to_uint8(a);
                match want {
                    -1 => assert_eq!(got, None, "hex_to_uint8({a:#04x})"),
                    v => assert_eq!(
                        got,
                        Some(u8::try_from(v).expect("in range")),
                        "hex_to_uint8({a:#04x})"
                    ),
                }
                hex += 1;
            }
            "char_to_hex" => {
                let a: u8 = row[1].parse::<u32>().expect("byte") as u8;
                let want: u8 = row[4].parse::<u32>().expect("result") as u8;
                assert_eq!(char_to_hex(a), want, "char_to_hex({a:#04x})");
                hex += 1;
            }
            "bounded" => {
                let v: i32 = row[1].parse().expect("value");
                let lo: i32 = row[2].parse().expect("lower");
                let hi: i32 = row[3].parse().expect("upper");
                let want = row[4] == "1";
                assert_eq!(
                    is_bounded_int32(v, lo, hi),
                    want,
                    "is_bounded_int32({v}, {lo}, {hi})"
                );
                bounded += 1;
            }
            "mktime" => {
                let year: i32 = row[1].parse().expect("year");
                let mon: i32 = row[2].parse().expect("mon");
                let mday: i32 = row[3].parse().expect("mday");
                let want: i64 = row[4].parse().expect("epoch");
                pending_date = Some((year, mon, mday, want));
            }
            "mktime_tod" => {
                let (year, mon, mday, want) = pending_date
                    .take()
                    .expect("a date row must precede its time");
                let t = Tm {
                    year,
                    mon,
                    mday,
                    hour: row[1].parse().expect("hour"),
                    min: row[2].parse().expect("min"),
                    sec: row[3].parse().expect("sec"),
                };
                let got = ap_mktime(&t);
                if want == -1 {
                    assert_eq!(got, None, "{t:?} should be rejected as pre-epoch");
                    mktimes += 1;
                } else {
                    let g = got.expect("a post-epoch date must convert");
                    if g == want {
                        mktimes += 1;
                    } else {
                        // D-022. Upstream computes `(tm_year - 70) * YEAR` with
                        // YEAR an `unsigned`, so the product is taken in 32
                        // bits. Past 2106 it wraps, and the difference is
                        // exactly a multiple of 2^32 — which is what makes this
                        // an overflow rather than a transcription error on
                        // either side.
                        const WRAP: i64 = 1_i64 << 32;
                        assert!(
                            t.year >= 207,
                            "{t:?}: differs from upstream at a year where the 32-bit intermediate still fits — got {g}, upstream {want}"
                        );
                        assert_eq!(
                            (g - want) % WRAP,
                            0,
                            "{t:?}: expected a clean 2^32 wrap, got {g} against upstream {want}"
                        );
                        assert!(g > want, "the port should give the larger, unwrapped value");
                        wrapped += 1;
                        mktimes += 1;
                    }
                }
            }
            other => panic!("unknown fixture kind {other}"),
        }
    }

    assert_eq!(
        f16_get, 65_536,
        "the half-precision sweep must be exhaustive"
    );
    assert!(f16_set >= 30);
    assert_eq!(hex, 512, "both hex helpers over all 256 bytes");
    assert_eq!(bounded, 9 * 9 * 9);
    assert!(mktimes >= 20);
    assert_eq!(
        wrapped, 2,
        "exactly the 2107 and 2150 dates should show upstream's 32-bit wrap; if this changes, D-022 needs revisiting"
    );

    println!(
        "bit-exact against upstream: {f16_get} half-precision patterns, {f16_set} conversions to half, {hex} hex lookups, {bounded} bounds checks, {mktimes} dates ({wrapped} showing upstream's D-022 overflow)"
    );
}

/// Every finite half-precision value must survive a round trip through `f32`
/// unchanged. This is a property of the pair rather than of either function,
/// so a fixture comparing them separately would not catch a matched error.
#[test]
fn every_half_precision_value_round_trips() {
    let mut checked = 0_usize;
    for bits in 0..=u16::MAX {
        let h = Float16::from_bits(bits);
        let as_f32 = h.get();
        if !as_f32.is_finite() {
            continue;
        }
        let back = Float16::set(as_f32).bits;
        assert_eq!(
            back, bits,
            "{bits:#06x} became {as_f32:e} and came back as {back:#06x}"
        );
        checked += 1;
    }
    println!("{checked} finite half-precision values round-tripped");
    assert!(checked > 60_000);
}

/// `hex_to_uint8` and `char_to_hex` are two shapes of the same lookup, and
/// upstream's agree on every byte. If the port's ever disagree, one has been
/// transcribed wrong — and the fixture would still pass, because it checks
/// each against upstream separately.
#[test]
fn the_two_hex_helpers_agree_with_each_other() {
    for a in 0..=u8::MAX {
        match hex_to_uint8(a) {
            Some(v) => assert_eq!(char_to_hex(a), v, "byte {a:#04x}"),
            None => assert_eq!(char_to_hex(a), CHAR_TO_HEX_INVALID, "byte {a:#04x}"),
        }
    }
}
