//! CRCs and checksums, ported from upstream `AP_Math/crc.cpp`.
//!
//! Every function here is pure and `no_std`. The lookup tables live in
//! [`crate::crc_tables`], generated from upstream's rather than retyped.
//!
//! # Slices instead of pointer and length
//!
//! Upstream takes a pointer and a separate length, several of them as `uint8_t`
//! which silently caps the buffer at 255 bytes. The port takes `&[u8]`, so the
//! length cannot disagree with the buffer and the cap disappears. No input the
//! C could express behaves differently.
//!
//! # Naming
//!
//! Upstream's names are kept, minus the redundant `crc_` where it only repeats
//! the module. `crc16_ccitt_GDL90` becomes `crc16_ccitt_gdl90` to satisfy Rust
//! casing; it is the same non-standard FAA variant.

use crate::crc_tables::{
    CRC16TAB, CRC32_TAB, CRC8_TABLE, CRC8_TABLE_MAXIM, CRC8_TABLE_RDS02UF, CRC8_TABLE_SAE,
    CRC_TABLE_IBM,
};

/// Offset basis for the 64-bit FNV-1a hash, upstream `FNV_1_OFFSET_BASIS_64`.
pub const FNV_1_OFFSET_BASIS_64: u64 = 14_695_981_039_346_656_037;

/// Prime for the 64-bit FNV-1a hash, upstream `FNV_1_PRIME_64`.
const FNV_1_PRIME_64: u64 = 1_099_511_628_211;

/// Index a 256-entry lookup table by a byte.
///
/// Total by construction: a `u8` widens into `0..=255` and the table has
/// exactly 256 entries, so the index cannot be out of range. The workspace
/// denies `clippy::indexing_slicing` because an unchecked index in flight code
/// is a panic waiting to happen; the proof lives here once rather than at each
/// call site. Using `get().unwrap_or(..)` instead would add a branch and an
/// arbitrary fallback on a path that cannot occur.
#[inline]
fn byte_lookup<T: Copy>(table: &[T; 256], idx: u8) -> T {
    #[allow(clippy::indexing_slicing)]
    {
        table[idx as usize]
    }
}

/// CRC-4 over eight 16-bit words, the method given in the MS5611 datasheet.
///
/// Reads the words most significant byte first, which is the datasheet's order
/// and independent of the host's endianness.
pub fn crc_crc4(data: &[u16; 8]) -> u16 {
    let mut n_rem: u16 = 0;

    // Upstream steps a counter 0..16 and derives the word from `cnt >> 1` and
    // the half from `cnt & 1`, which is exactly each word high byte first.
    // Iterating says so directly and removes an index.
    for &word in data {
        for half in [word >> 8, word & 0x00FF] {
            n_rem ^= half;
            for _ in 0..8 {
                if n_rem & 0x8000 != 0 {
                    n_rem = (n_rem << 1) ^ 0x3000;
                } else {
                    n_rem <<= 1;
                }
            }
        }
    }

    (n_rem >> 12) & 0xF
}

/// CRC-8 with polynomial 0x07, from the TeraRanger driver.
pub fn crc_crc8(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &b in data {
        crc = byte_lookup(&CRC8_TABLE, crc ^ b);
    }
    crc
}

/// CRC-8 for an arbitrary polynomial, without a lookup table.
pub fn crc8_generic(data: &[u8], polynomial: u8, initial_value: u8) -> u8 {
    let mut crc = initial_value;
    for &b in data {
        // Upstream writes `crc8_dvb(buf[i], crc, polynomial)`, with the byte in
        // the `crc` parameter and the accumulator in `a`. The two are
        // interchangeable because the first thing `crc8_dvb` does is a
        // symmetric XOR of them; `crc8_dvb_is_symmetric_in_its_first_two_args`
        // pins that. Written the natural way round here so it reads correctly.
        crc = crc8_dvb(crc, b, polynomial);
    }
    crc
}

/// One byte of CRC-8/DVB-S2, from Betaflight.
pub fn crc8_dvb_s2(crc: u8, a: u8) -> u8 {
    crc8_dvb(crc, a, 0xD5)
}

/// One byte of CRC-8 with an arbitrary polynomial, from Betaflight.
pub fn crc8_dvb(crc: u8, a: u8, seed: u8) -> u8 {
    let mut crc = crc ^ a;
    for _ in 0..8 {
        if crc & 0x80 != 0 {
            crc = (crc << 1) ^ seed;
        } else {
            crc <<= 1;
        }
    }
    crc
}

/// CRC-8/DVB-S2 over a buffer, continuing from `crc`.
pub fn crc8_dvb_s2_update(crc: u8, data: &[u8]) -> u8 {
    let mut crc = crc;
    for &b in data {
        crc = crc8_dvb_s2(crc, b);
    }
    crc
}

/// CRC-8 with polynomial 0x07 over a buffer, continuing from `crc`.
///
/// From `AP_FETtecOneWire`.
pub fn crc8_dvb_update(crc: u8, data: &[u8]) -> u8 {
    let mut crc = crc;
    for &b in data {
        crc = crc8_dvb(crc, b, 0x07);
    }
    crc
}

/// CRC-8/MAXIM, the Dallas 1-Wire checksum.
pub fn crc8_maxim(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &b in data {
        crc = byte_lookup(&CRC8_TABLE_MAXIM, crc ^ b);
    }
    crc
}

/// CRC-8/SAE-J1850.
pub fn crc8_sae(data: &[u8]) -> u8 {
    let mut crc: u8 = 0xFF;
    for &b in data {
        crc = byte_lookup(&CRC8_TABLE_SAE, crc ^ b);
    }
    crc ^ 0xFF
}

/// CRC-8 for the RDS02UF rangefinder, using that device's vendor table.
pub fn crc8_rds02uf(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &b in data {
        crc = byte_lookup(&CRC8_TABLE_RDS02UF, crc ^ b);
    }
    crc
}

/// XOR of every byte.
pub fn crc_xor_of_bytes(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |acc, &b| acc ^ b)
}

/// One byte of the XMODEM CRC-16.
pub fn crc_xmodem_update(crc: u16, data: u8) -> u16 {
    let mut crc = crc ^ (u16::from(data) << 8);
    for _ in 0..8 {
        if crc & 0x8000 != 0 {
            crc = (crc << 1) ^ 0x1021;
        } else {
            crc <<= 1;
        }
    }
    crc
}

/// XMODEM CRC-16 over a buffer, starting from zero.
pub fn crc_xmodem(data: &[u8]) -> u16 {
    data.iter().fold(0u16, |crc, &b| crc_xmodem_update(crc, b))
}

/// Table-driven CRC-32, continuing from `crc`.
///
/// The conventional pre- and post-inversion is **not** applied here, matching
/// upstream: a caller wanting the standard CRC-32 of a buffer passes
/// `0xFFFF_FFFF` and inverts the result.
pub fn crc_crc32(crc: u32, data: &[u8]) -> u32 {
    let mut crc = crc;
    for &b in data {
        crc = byte_lookup(&CRC32_TAB, (crc as u8) ^ b) ^ (crc >> 8);
    }
    crc
}

/// Bitwise CRC-32, smaller and slower than [`crc_crc32`], for the bootloader.
///
/// Produces the same value as [`crc_crc32`] for the same inputs;
/// `crc32_small_agrees_with_table_driven` pins that.
pub fn crc32_small(crc: u32, data: &[u8]) -> u32 {
    let mut crc = crc;
    for &byte in data {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            // C negates an unsigned value here, which is defined as wrapping;
            // Rust requires it to be spelled out.
            let mask = (crc & 1).wrapping_neg();
            crc >>= 1;
            crc ^= 0xEDB8_8320 & mask;
        }
    }
    crc
}

/// CRC-24 with polynomial 0x1864CFB, computed bitwise to save table space.
pub fn crc_crc24(data: &[u8]) -> u32 {
    const POLYCRC24: u32 = 0x0186_4CFB;
    let mut crc: u32 = 0;
    for &b in data {
        let idx = ((crc >> 16) as u8) ^ b;
        let mut crct = u32::from(idx) << 16;
        for _ in 0..8 {
            crct <<= 1;
            if crct & 0x0100_0000 != 0 {
                crct ^= POLYCRC24;
            }
        }
        crc = ((crc << 8) & 0x00FF_FFFF) ^ crct;
    }
    crc
}

/// CRC-16/CCITT, MSB-first, continuing from `crc`.
pub fn crc16_ccitt(data: &[u8], crc: u16) -> u16 {
    let mut crc = crc;
    for &b in data {
        crc = (crc << 8) ^ byte_lookup(&CRC16TAB, ((crc >> 8) as u8) ^ b);
    }
    crc
}

/// CRC-16/CCITT computed with right shifts, with a final output XOR.
///
/// This is the reflected form (polynomial 0x8408), which is a different
/// checksum from [`crc16_ccitt`] rather than another way of computing it.
pub fn crc16_ccitt_r(data: &[u8], crc: u16, out: u16) -> u16 {
    let mut crc = crc;
    for &b in data {
        crc ^= u16::from(b);
        for _ in 0..8 {
            if crc & 0x0001 != 0 {
                crc = (crc >> 1) ^ 0x8408;
            } else {
                crc >>= 1;
            }
        }
    }
    crc ^ out
}

/// The GDL90 variant of CRC-16/CCITT, as specified by the FAA.
///
/// Non-standard: it indexes the table with the high byte alone rather than the
/// high byte XOR the input, so it is not interchangeable with
/// [`crc16_ccitt`].
pub fn crc16_ccitt_gdl90(data: &[u8], crc: u16) -> u16 {
    let mut crc = crc;
    for &b in data {
        crc = byte_lookup(&CRC16TAB, (crc >> 8) as u8) ^ (crc << 8) ^ u16::from(b);
    }
    crc
}

/// Modbus CRC-16.
pub fn calc_crc_modbus(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= u16::from(b);
        for _ in 0..8 {
            if crc & 0x0001 != 0 {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

/// Fletcher-16 checksum.
pub fn crc_fletcher16(data: &[u8]) -> u16 {
    let mut c0: u16 = 0;
    let mut c1: u16 = 0;
    for &b in data {
        c0 = (c0 + u16::from(b)) % 255;
        c1 = (c1 + c0) % 255;
    }
    (c1 << 8) | c0
}

/// 64-bit FNV-1a hash, continuing from `hash`.
///
/// Upstream takes the accumulator by pointer and updates it in place; this
/// returns it. Start from [`FNV_1_OFFSET_BASIS_64`].
pub fn hash_fnv_1a(data: &[u8], hash: u64) -> u64 {
    let mut hash = hash;
    for &b in data {
        hash ^= u64::from(b);
        hash = hash.wrapping_mul(FNV_1_PRIME_64);
    }
    hash
}

/// CRC-64-WE with polynomial 0x42F0E1EBA9EA3693, matching the PX4 bootloader.
///
/// # Endianness
///
/// Upstream reads each 32-bit word through a `uint8_t*`, so the byte order it
/// feeds the CRC is the host's. Every ArduPilot target is little-endian, so
/// that is what the format has always meant in practice; the port says so
/// explicitly with `to_le_bytes` rather than inheriting an implicit dependency
/// on the compilation target. Registered as D-010 -- the two agree on every
/// supported target and differ only on a big-endian one, where upstream would
/// produce a value no PX4 bootloader would accept.
pub fn crc_crc64(data: &[u32]) -> u64 {
    const POLY: u64 = 0x42F0_E1EB_A9EA_3693;
    let mut crc: u64 = u64::MAX;
    for &value in data {
        for byte in value.to_le_bytes() {
            crc ^= u64::from(byte) << 56;
            for _ in 0..8 {
                if crc & (1 << 63) != 0 {
                    crc = (crc << 1) ^ POLY;
                } else {
                    crc <<= 1;
                }
            }
        }
    }
    crc ^ u64::MAX
}

/// CRC-16 with polynomial 0x8005, MSB-first, continuing from `crc_accum`.
///
/// Despite the name this is not CRC-16/ARC, which is the reflected form of the
/// same polynomial and gives different values.
pub fn crc_crc16_ibm(crc_accum: u16, data: &[u8]) -> u16 {
    let mut crc_accum = crc_accum;
    for &b in data {
        let i = ((crc_accum >> 8) as u8) ^ b;
        crc_accum = (crc_accum << 8) ^ byte_lookup(&CRC_TABLE_IBM, i);
    }
    crc_accum
}

/// The 8-bit checksum used by SPORT and FPort.
///
/// Adds each byte into a 16-bit sum, folding the carry back in, and returns the
/// complement.
pub fn crc_sum8_with_carry(data: &[u8]) -> u8 {
    let mut sum: u16 = 0;
    for &b in data {
        sum += u16::from(b);
        sum += sum >> 8;
        sum &= 0xFF;
    }
    // The mask above leaves nothing above bit 7, so upstream's trailing
    // `(sum & 0xff) + (sum >> 8)` is just `sum`. Kept simple rather than
    // reproducing arithmetic that cannot do anything.
    0xFF - (sum as u8)
}

/// Parity of a byte: 1 for an odd number of set bits, 0 for even.
pub fn parity(byte: u8) -> u8 {
    // Upstream unrolls eight shifts by hand, because `__builtin_parity` was
    // slower for one byte and hard-faulted on Pixracer-periph. `count_ones` is
    // a Rust intrinsic with no such history and is exactly equivalent.
    (byte.count_ones() & 1) as u8
}

/// Sum of the bytes, modulo 0x10000.
///
/// Upstream's comment says "mod 0xFFFF"; the code is a `uint16_t` accumulator,
/// so it is mod 0x10000. The code is what the port follows.
pub fn crc_sum_of_bytes_16(data: &[u8]) -> u16 {
    data.iter()
        .fold(0u16, |acc, &b| acc.wrapping_add(u16::from(b)))
}

/// Sum of the bytes, modulo 256.
pub fn crc_sum_of_bytes(data: &[u8]) -> u8 {
    crc_sum_of_bytes_16(data) as u8
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::indexing_slicing,
        reason = "indexes 256-entry arrays by a byte, which cannot be out of range; in a test an index fault is a test failure, not a flight hazard"
    )]

    use super::*;

    /// Re-derive a byte-wide MSB-first table from its polynomial.
    fn derive_msb_u8(poly: u8) -> [u8; 256] {
        let mut t = [0u8; 256];
        for (i, e) in t.iter_mut().enumerate() {
            let mut crc = i as u8;
            for _ in 0..8 {
                crc = if crc & 0x80 != 0 {
                    (crc << 1) ^ poly
                } else {
                    crc << 1
                };
            }
            *e = crc;
        }
        t
    }

    /// Re-derive a byte-wide reflected table from its (already reflected)
    /// polynomial.
    fn derive_lsb_u8(poly: u8) -> [u8; 256] {
        let mut t = [0u8; 256];
        for (i, e) in t.iter_mut().enumerate() {
            let mut crc = i as u8;
            for _ in 0..8 {
                crc = if crc & 1 != 0 {
                    (crc >> 1) ^ poly
                } else {
                    crc >> 1
                };
            }
            *e = crc;
        }
        t
    }

    fn derive_msb_u16(poly: u16) -> [u16; 256] {
        let mut t = [0u16; 256];
        for (i, e) in t.iter_mut().enumerate() {
            let mut crc = (i as u16) << 8;
            for _ in 0..8 {
                crc = if crc & 0x8000 != 0 {
                    (crc << 1) ^ poly
                } else {
                    crc << 1
                };
            }
            *e = crc;
        }
        t
    }

    fn derive_lsb_u32(poly: u32) -> [u32; 256] {
        let mut t = [0u32; 256];
        for (i, e) in t.iter_mut().enumerate() {
            let mut crc = i as u32;
            for _ in 0..8 {
                crc = if crc & 1 != 0 {
                    (crc >> 1) ^ poly
                } else {
                    crc >> 1
                };
            }
            *e = crc;
        }
        t
    }

    /// The tables are generated from upstream's, so this cannot catch a
    /// generator bug by comparing them to upstream again. It checks something
    /// independent: that each table really is the polynomial it claims, which
    /// would fail on a corrupted extraction or a mislabelled table.
    #[test]
    fn tables_match_their_polynomials() {
        assert_eq!(
            CRC8_TABLE,
            derive_msb_u8(0x07),
            "crc8_table is poly 0x07 MSB"
        );
        assert_eq!(
            CRC8_TABLE_MAXIM,
            derive_lsb_u8(0x8C),
            "maxim is poly 0x31 reflected to 0x8C"
        );
        assert_eq!(
            CRC8_TABLE_SAE,
            derive_msb_u8(0x1D),
            "SAE-J1850 is poly 0x1D MSB"
        );
        assert_eq!(CRC16TAB, derive_msb_u16(0x1021), "CCITT is poly 0x1021 MSB");
        assert_eq!(
            CRC_TABLE_IBM,
            derive_msb_u16(0x8005),
            "the IBM table is poly 0x8005 MSB, NOT the reflected ARC form"
        );
        assert_eq!(
            CRC32_TAB,
            derive_lsb_u32(0xEDB8_8320),
            "crc32 is poly 0x04C11DB7 reflected"
        );
    }

    /// The RDS02UF table is a vendor S-box, and it has a defect upstream.
    ///
    /// It is bijective except for one collision: 0x06 appears at indices 128
    /// and 202, and 0xA6 appears nowhere. One duplicate and one omission in an
    /// otherwise-bijective table is what a single mistyped nibble looks like --
    /// 0xA6 entered as 0x06.
    ///
    /// The port reproduces upstream byte for byte and does not correct it.
    /// ADR-0007 says to fix inherited bugs, but that presumes knowing the
    /// correct behaviour, and here nothing does: either index 128 or index 202
    /// should hold 0xA6, and the table cannot say which. It is not GF(2)-linear
    /// (checked, both as published and with the candidate fix), so unlike a
    /// polynomial table its entries cannot be re-derived. Correcting the wrong
    /// one would break interoperability with hardware that works today.
    ///
    /// Upstream cannot see this: `AP_RangeFinder_RDS02UF` and the SITL model
    /// `SIM_RF_RDS02UF` both call `crc8_rds02uf`, so simulated frames validate
    /// against the same table that produced them whatever it contains.
    ///
    /// Resolving it needs the vendor protocol document. Tracked as an open
    /// question rather than a divergence, since the port currently differs from
    /// upstream in no way at all.
    #[test]
    fn rds02uf_table_has_upstreams_one_byte_collision() {
        let mut count = [0u8; 256];
        for &v in CRC8_TABLE_RDS02UF.iter() {
            count[usize::from(v)] += 1;
        }

        // no_std: count and locate without allocating
        let mut n_duplicated = 0;
        let mut n_missing = 0;
        let mut duplicated = 0usize;
        let mut missing = 0usize;
        for (v, &c) in count.iter().enumerate() {
            if c > 1 {
                n_duplicated += 1;
                duplicated = v;
            } else if c == 0 {
                n_missing += 1;
                missing = v;
            }
        }

        assert_eq!(
            (n_duplicated, n_missing),
            (1, 1),
            "expected exactly one duplicate and one omission; a change here              means upstream edited the table and the port must be regenerated"
        );
        assert_eq!(duplicated, 0x06, "0x06 is the duplicated value");
        assert_eq!(missing, 0xA6, "0xA6 is the value that appears nowhere");

        let mut first = None;
        let mut second = None;
        for (i, &v) in CRC8_TABLE_RDS02UF.iter().enumerate() {
            if v == 0x06 {
                if first.is_none() {
                    first = Some(i);
                } else {
                    second = Some(i);
                }
            }
        }
        assert_eq!(
            (first, second),
            (Some(128), Some(202)),
            "the collision is at indices 128 and 202"
        );
    }

    /// Upstream passes the byte and the accumulator to `crc8_dvb` in the
    /// opposite order in `crc8_generic` and `crc8_dvb_update`. That is harmless
    /// only because the first operation is a symmetric XOR, which this pins --
    /// if upstream ever adds anything before it, this fails and the port's
    /// argument order stops being equivalent.
    #[test]
    fn crc8_dvb_is_symmetric_in_its_first_two_args() {
        for c in (0..=255u8).step_by(17) {
            for a in (0..=255u8).step_by(13) {
                for &seed in &[0x07u8, 0xD5, 0x1D, 0x00] {
                    assert_eq!(crc8_dvb(c, a, seed), crc8_dvb(a, c, seed));
                }
            }
        }
    }

    /// The bitwise and table-driven CRC-32 must agree, or one of them is wrong.
    #[test]
    fn crc32_small_agrees_with_table_driven() {
        let data: [u8; 64] = core::array::from_fn(|i| (i as u8).wrapping_mul(37).wrapping_add(11));
        for &seed in &[0u32, 0xFFFF_FFFF, 0x1234_5678] {
            for len in [0usize, 1, 7, 63, 64] {
                assert_eq!(
                    crc_crc32(seed, &data[..len]),
                    crc32_small(seed, &data[..len]),
                    "seed {seed:#x} len {len}"
                );
            }
        }
    }

    /// Every function must accept an empty buffer and return its seed
    /// unchanged, or the identity a caller relies on when streaming is broken.
    #[test]
    fn empty_input_returns_the_seed() {
        assert_eq!(crc_crc8(&[]), 0);
        assert_eq!(crc8_maxim(&[]), 0);
        assert_eq!(crc8_rds02uf(&[]), 0);
        assert_eq!(crc_xor_of_bytes(&[]), 0);
        assert_eq!(crc_xmodem(&[]), 0);
        assert_eq!(crc_fletcher16(&[]), 0);
        assert_eq!(crc_crc24(&[]), 0);
        assert_eq!(crc_sum_of_bytes(&[]), 0);
        assert_eq!(crc_sum_of_bytes_16(&[]), 0);
        assert_eq!(crc_crc32(0x1234_5678, &[]), 0x1234_5678);
        assert_eq!(crc32_small(0x1234_5678, &[]), 0x1234_5678);
        assert_eq!(crc16_ccitt(&[], 0xABCD), 0xABCD);
        assert_eq!(crc16_ccitt_gdl90(&[], 0xABCD), 0xABCD);
        assert_eq!(crc_crc16_ibm(0xABCD, &[]), 0xABCD);
        assert_eq!(crc8_dvb_s2_update(0x5A, &[]), 0x5A);
        assert_eq!(crc8_dvb_update(0x5A, &[]), 0x5A);
        assert_eq!(crc8_generic(&[], 0x07, 0x5A), 0x5A);
        assert_eq!(
            hash_fnv_1a(&[], FNV_1_OFFSET_BASIS_64),
            FNV_1_OFFSET_BASIS_64
        );
        assert_eq!(crc_crc64(&[]), 0);

        // these two apply a final transform, so their empty value is not the seed
        assert_eq!(crc8_sae(&[]), 0x00, "0xFF init XOR 0xFF out");
        assert_eq!(crc_sum8_with_carry(&[]), 0xFF, "complement of a zero sum");
        assert_eq!(crc16_ccitt_r(&[], 0, 0xFFFF), 0xFFFF, "output XOR only");
    }

    /// Streaming a buffer in pieces must equal hashing it whole, for every
    /// function that takes a running value. A driver that reads a packet in two
    /// reads depends on this.
    #[test]
    fn seeded_forms_are_streamable() {
        let d: [u8; 32] = core::array::from_fn(|i| (i as u8).wrapping_mul(29).wrapping_add(7));
        for split in 0..=d.len() {
            let (a, b) = d.split_at(split);
            assert_eq!(
                crc_crc32(crc_crc32(0xFFFF_FFFF, a), b),
                crc_crc32(0xFFFF_FFFF, &d),
                "crc_crc32 split at {split}"
            );
            assert_eq!(
                crc16_ccitt(b, crc16_ccitt(a, 0xFFFF)),
                crc16_ccitt(&d, 0xFFFF),
                "crc16_ccitt split at {split}"
            );
            assert_eq!(
                crc_crc16_ibm(crc_crc16_ibm(0, a), b),
                crc_crc16_ibm(0, &d),
                "crc_crc16_ibm split at {split}"
            );
            assert_eq!(
                crc8_dvb_s2_update(crc8_dvb_s2_update(0, a), b),
                crc8_dvb_s2_update(0, &d),
                "crc8_dvb_s2_update split at {split}"
            );
            assert_eq!(
                hash_fnv_1a(b, hash_fnv_1a(a, FNV_1_OFFSET_BASIS_64)),
                hash_fnv_1a(&d, FNV_1_OFFSET_BASIS_64),
                "hash_fnv_1a split at {split}"
            );
        }
    }

    /// `parity` is expressed with `count_ones` rather than upstream's unrolled
    /// shifts, so the equivalence is checked over the whole domain.
    #[test]
    fn parity_matches_the_unrolled_form() {
        for byte in 0..=255u8 {
            let mut p = 0u8;
            let mut b = byte;
            for _ in 0..8 {
                p ^= b & 1;
                b >>= 1;
            }
            assert_eq!(parity(byte), p, "byte {byte:#04x}");
        }
    }

    /// A CRC's whole purpose is to change when the data does.
    #[test]
    fn single_bit_flips_change_every_crc() {
        let base: [u8; 16] = core::array::from_fn(|i| (i as u8).wrapping_mul(53));
        for bit in 0..(base.len() * 8) {
            let mut flipped = base;
            flipped[bit / 8] ^= 1 << (bit % 8);
            assert_ne!(crc_crc8(&base), crc_crc8(&flipped), "crc_crc8 bit {bit}");
            assert_ne!(crc8_maxim(&base), crc8_maxim(&flipped), "maxim bit {bit}");
            assert_ne!(crc_xmodem(&base), crc_xmodem(&flipped), "xmodem bit {bit}");
            assert_ne!(
                crc_crc32(0, &base),
                crc_crc32(0, &flipped),
                "crc32 bit {bit}"
            );
            assert_ne!(crc_crc24(&base), crc_crc24(&flipped), "crc24 bit {bit}");
        }
    }
}
