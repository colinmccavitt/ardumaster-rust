//! Parity test: every CRC against upstream `AP_Math/crc.cpp`'s own output.
//!
//! The expected values here are neither written by hand nor taken from
//! published check constants. They are produced by compiling upstream's
//! `crc.cpp` unmodified and running it over a fixed input set — see
//! `tools/parity/gen_crc_fixture.py`. Regenerate after an upstream
//! re-baseline; a diff in the fixture is an upstream behaviour change and
//! should be read as one.
//!
//! Why not published constants: a named CRC's "check" value is easy to attach
//! to the wrong variant, and three of these look standard but are not.
//! `crc_crc16_ibm` is the non-reflected 0x8005 form, not CRC-16/ARC.
//! `crc16_ccitt_GDL90` indexes its table with the high byte alone, unlike every
//! other CCITT implementation. `crc8_rds02uf` uses a vendor table matching no
//! polynomial. Running the C removes the opportunity to be confidently wrong.
//!
//! Parsed by hand rather than with a CSV or JSON crate, matching `ap-replay`:
//! a test-support dependency that can break the build is not worth the
//! convenience.

#![allow(
    clippy::indexing_slicing,
    reason = "slices a hex string whose length is asserted, and indexes fixture rows whose field count is asserted; in a test an index fault is a test failure, which is the desired outcome"
)]
use ap_math::crc::*;

/// One row of the `#cases` section.
struct Case {
    func: String,
    buf: String,
    seed: Option<u64>,
    seed2: Option<u64>,
    out: u64,
}

struct Fixture {
    buffers: Vec<(String, Vec<u8>)>,
    cases: Vec<Case>,
}

fn fixture_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/crc_parity.csv"))
        .expect("workspace root")
}

fn unhex(s: &str) -> Vec<u8> {
    assert!(s.len().is_multiple_of(2), "odd-length hex string");
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).expect("hex digit"))
        .collect()
}

fn parse(text: &str) -> Fixture {
    let mut buffers = Vec::new();
    let mut cases = Vec::new();
    let mut section = "";

    for line in text.lines() {
        let line = line.trim_end();
        if line.is_empty() {
            continue;
        }
        if let Some(name) = line.strip_prefix('#') {
            section = match name {
                "buffers" => "buffers",
                "cases" => "cases",
                other => panic!("unknown section {other}"),
            };
            continue;
        }
        if line.starts_with("name,") || line.starts_with("fn,") {
            continue; // column header
        }

        let f: Vec<&str> = line.split(',').collect();
        match section {
            "buffers" => {
                assert_eq!(f.len(), 2, "buffer row: {line}");
                buffers.push((f[0].to_string(), unhex(f[1])));
            }
            "cases" => {
                assert_eq!(f.len(), 5, "case row: {line}");
                let num = |s: &str| -> Option<u64> {
                    if s.is_empty() {
                        None
                    } else {
                        Some(s.parse().unwrap_or_else(|_| panic!("bad number {s:?}")))
                    }
                };
                cases.push(Case {
                    func: f[0].to_string(),
                    buf: f[1].to_string(),
                    seed: num(f[2]),
                    seed2: num(f[3]),
                    out: num(f[4]).expect("out is required"),
                });
            }
            _ => panic!("row outside any section: {line}"),
        }
    }
    Fixture { buffers, cases }
}

/// The word inputs the harness uses, repeated here because the fixture names
/// them rather than spelling them out. They must stay in step with
/// `gen_crc_fixture.py`; `crc_crc4` and `crc_crc64` would otherwise be compared
/// against a different input than upstream saw, which is why the values are
/// asserted below rather than merely used.
const CRC4_WORDS: [u16; 8] = [
    0x1234, 0x5678, 0x9abc, 0xdef0, 0x0f1e, 0x2d3c, 0x4b5a, 0x6978,
];
const CRC64_WORDS: [u32; 4] = [0x0000_0000, 0xFFFF_FFFF, 0x1234_5678, 0xdead_beef];

#[test]
fn every_crc_matches_upstream() {
    let path = fixture_path();
    if !path.exists() {
        eprintln!("skipping: {} not present", path.display());
        return;
    }
    let fx = parse(&std::fs::read_to_string(&path).expect("read fixture"));

    let get = |name: &str| -> &[u8] {
        fx.buffers
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, b)| b.as_slice())
            .unwrap_or_else(|| panic!("fixture has no buffer {name}"))
    };

    let mut checked = 0usize;
    let mut uncovered: Vec<String> = Vec::new();

    for c in &fx.cases {
        // buffer-driven cases name a buffer; the rest name their input shape
        let d: &[u8] = match c.buf.as_str() {
            "pair" | "byte" | "words" | "zeros" | "words4" | "words1" | "words0" => &[],
            name => get(name),
        };
        let seed = || c.seed.expect("case needs a seed");
        let pair = || {
            (
                c.seed.expect("case needs a pair") as u8,
                c.seed2.expect("case needs a pair") as u8,
            )
        };

        let got: u64 = match c.func.as_str() {
            "crc_crc8" => crc_crc8(d).into(),
            "crc8_maxim" => crc8_maxim(d).into(),
            "crc8_sae" => crc8_sae(d).into(),
            "crc8_rds02uf" => crc8_rds02uf(d).into(),
            "crc_xor_of_bytes" => crc_xor_of_bytes(d).into(),
            "crc_xmodem" => crc_xmodem(d).into(),
            "calc_crc_modbus" => calc_crc_modbus(d).into(),
            "crc_fletcher16" => crc_fletcher16(d).into(),
            "crc_crc24" => crc_crc24(d).into(),
            "crc_sum_of_bytes" => crc_sum_of_bytes(d).into(),
            "crc_sum_of_bytes_16" => crc_sum_of_bytes_16(d).into(),
            "crc_sum8_with_carry" => crc_sum8_with_carry(d).into(),
            "hash_fnv_1a" => hash_fnv_1a(d, FNV_1_OFFSET_BASIS_64),

            "crc_crc32" => crc_crc32(seed() as u32, d).into(),
            "crc32_small" => crc32_small(seed() as u32, d).into(),
            "crc16_ccitt" => crc16_ccitt(d, seed() as u16).into(),
            "crc16_ccitt_GDL90" => crc16_ccitt_gdl90(d, seed() as u16).into(),
            "crc16_ccitt_r" => crc16_ccitt_r(d, seed() as u16, 0xFFFF).into(),
            "crc_crc16_ibm" => crc_crc16_ibm(seed() as u16, d).into(),
            "crc8_dvb_s2_update" => crc8_dvb_s2_update(seed() as u8, d).into(),
            "crc8_dvb_update" => crc8_dvb_update(seed() as u8, d).into(),
            "crc8_generic_07" => crc8_generic(d, 0x07, seed() as u8).into(),
            "crc8_generic_d5" => crc8_generic(d, 0xD5, seed() as u8).into(),

            "crc8_dvb_s2" => {
                let (a, b) = pair();
                crc8_dvb_s2(a, b).into()
            }
            "crc8_dvb_07" => {
                let (a, b) = pair();
                crc8_dvb(a, b, 0x07).into()
            }
            "crc_xmodem_update" => {
                let (a, b) = pair();
                crc_xmodem_update(u16::from(a) * 257, b).into()
            }
            "parity" => parity(seed() as u8).into(),

            "crc_crc4" => match c.buf.as_str() {
                "words" => crc_crc4(&CRC4_WORDS).into(),
                "zeros" => crc_crc4(&[0; 8]).into(),
                other => panic!("unknown crc4 input {other}"),
            },
            "crc_crc64" => {
                let n = match c.buf.as_str() {
                    "words4" => 4,
                    "words1" => 1,
                    "words0" => 0,
                    other => panic!("unknown crc64 input {other}"),
                };
                crc_crc64(&CRC64_WORDS[..n])
            }

            other => {
                uncovered.push(other.to_string());
                continue;
            }
        };

        assert_eq!(
            got, c.out,
            "{}(buf {}, seed {:?}/{:?}): port {:#x}, upstream {:#x}",
            c.func, c.buf, c.seed, c.seed2, got, c.out
        );
        checked += 1;
    }

    // A parity test that quietly matched nothing would pass. Both guards below
    // must hold, and the first catches a function added upstream that the port
    // has not picked up.
    uncovered.sort();
    uncovered.dedup();
    assert!(
        uncovered.is_empty(),
        "fixture has functions this test does not cover: {uncovered:?}"
    );
    assert!(
        checked > 500,
        "expected the whole fixture to be checked, got {checked}"
    );
    println!("{checked} cases matched upstream crc.cpp exactly");
}

/// The buffers must be the ones upstream was actually run over.
///
/// The fixture carries them, so a mismatch is impossible for the byte-oriented
/// functions. It is possible for the word-oriented ones, whose inputs the
/// fixture only names — so those are pinned here.
#[test]
fn word_inputs_match_the_harness() {
    assert_eq!(
        CRC4_WORDS,
        [0x1234, 0x5678, 0x9abc, 0xdef0, 0x0f1e, 0x2d3c, 0x4b5a, 0x6978],
        "must match d4[] in gen_crc_fixture.py"
    );
    assert_eq!(
        CRC64_WORDS,
        [0x0000_0000, 0xFFFF_FFFF, 0x1234_5678, 0xdead_beef],
        "must match d64[] in gen_crc_fixture.py"
    );
}

/// The `allbytes` buffer exists so every table entry is hit at least once.
/// If it were ever truncated the parity test would still pass while covering
/// far less, so its shape is checked directly.
#[test]
fn fixture_buffers_are_what_they_claim() {
    let path = fixture_path();
    if !path.exists() {
        eprintln!("skipping: fixture not present");
        return;
    }
    let fx = parse(&std::fs::read_to_string(&path).expect("read fixture"));
    let get = |name: &str| -> Vec<u8> {
        fx.buffers
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, b)| b.clone())
            .unwrap_or_else(|| panic!("no buffer {name}"))
    };

    assert!(get("empty").is_empty());
    assert_eq!(get("check"), b"123456789");
    let all = get("allbytes");
    assert_eq!(all.len(), 256, "must cover every table index");
    assert!(
        all.iter().enumerate().all(|(i, &b)| b as usize == i),
        "allbytes must be 0x00..=0xFF in order"
    );
    assert_eq!(get("pseudo").len(), 64);
}
