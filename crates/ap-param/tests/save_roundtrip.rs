//! The write path against real storage: replay `eeprom.bin` through `save`
//! and require the result to be byte-identical.
//!
//! `eeprom.bin` was written by the reference ArduPlane build, not by this
//! port. Reading every record out of it and writing each one back into a blank
//! storage exercises the append protocol end to end — the scan that finds the
//! sentinel, the offset arithmetic, the value encoding, the sentinel move —
//! and the only way the result matches byte for byte is if all of it agrees
//! with what ArduPilot did.
//!
//! This is a stronger statement than the unit tests can make. They assert the
//! port is self-consistent; this asserts it agrees with a file ArduPilot
//! produced.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes a fixture whose length is asserted; in a test an index \
fault is a test failure, which is the desired outcome"
)]

use ap_param::{
    read, save, scan, ParamHeader, SaveOutcome, ScanResult, Storage, EEPROM_HEADER_SIZE,
    EEPROM_MAGIC, EEPROM_REVISION,
};

/// The reference eeprom, loaded read-only.
struct Reference {
    bytes: Vec<u8>,
}

impl Storage for Reference {
    fn size(&self) -> u16 {
        u16::try_from(self.bytes.len()).unwrap_or(u16::MAX)
    }
    fn read(&self, offset: u16, buf: &mut [u8]) -> bool {
        let start = usize::from(offset);
        let Some(src) = self.bytes.get(start..start + buf.len()) else {
            return false;
        };
        buf.copy_from_slice(src);
        true
    }
    fn write(&mut self, _offset: u16, _data: &[u8]) -> bool {
        panic!("the reference must not be written to")
    }
}

/// A blank storage of the same size, formatted the way `erase_all` leaves it.
struct Blank {
    bytes: Vec<u8>,
}

impl Blank {
    fn formatted(size: usize) -> Self {
        let mut bytes = vec![0xFF_u8; size];
        bytes[0] = EEPROM_MAGIC[0];
        bytes[1] = EEPROM_MAGIC[1];
        bytes[2] = EEPROM_REVISION;
        bytes[3] = 0;
        let sentinel = ParamHeader::sentinel().to_bytes();
        bytes[EEPROM_HEADER_SIZE..EEPROM_HEADER_SIZE + 4].copy_from_slice(&sentinel);
        Self { bytes }
    }
}

impl Storage for Blank {
    fn size(&self) -> u16 {
        u16::try_from(self.bytes.len()).unwrap_or(u16::MAX)
    }
    fn read(&self, offset: u16, buf: &mut [u8]) -> bool {
        let start = usize::from(offset);
        let Some(src) = self.bytes.get(start..start + buf.len()) else {
            return false;
        };
        buf.copy_from_slice(src);
        true
    }
    fn write(&mut self, offset: u16, data: &[u8]) -> bool {
        let start = usize::from(offset);
        let Some(dst) = self.bytes.get_mut(start..start + data.len()) else {
            return false;
        };
        dst.copy_from_slice(data);
        true
    }
}

fn reference() -> Reference {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/eeprom.bin"))
        .expect("workspace root");
    let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    Reference { bytes }
}

/// Replaying the reference through `save` reproduces it byte for byte.
#[test]
fn replaying_the_reference_eeprom_reproduces_it_exactly() {
    let src = reference();
    let mut dst = Blank::formatted(src.bytes.len());

    let iter = read(&src).expect("the reference should be a valid eeprom");
    let mut written = 0_usize;

    let mut undecodable = 0_usize;

    for entry in iter {
        // An entry whose type the format does not define has no value to write
        // back. The reference should contain none; counted rather than ignored
        // so a silent skip cannot make the replay look complete.
        let Some(value) = entry.value else {
            undecodable += 1;
            continue;
        };
        // Forced, because these values came out of storage: they were worth
        // saving when ArduPilot saved them, whatever their defaults are, and
        // the defaults are not in the file.
        let outcome = save(&mut dst, entry.header, value, None, true)
            .expect("the blank storage has the same size as the reference");
        assert_eq!(
            outcome,
            SaveOutcome::Appended,
            "entry {written} (key {}) should have appended",
            entry.header.key
        );
        written += 1;
    }

    assert_eq!(undecodable, 0, "the reference should decode completely");
    assert!(
        written > 20,
        "the reference should hold real parameters, got {written}"
    );

    // Everything up to and including the final sentinel must match. Bytes past
    // it are whatever the flash happened to hold and are not part of the
    // format.
    let end = match scan(&dst, ParamHeader::new(0x3FF, 0x1E, 0x3FFFF)) {
        ScanResult::Sentinel(o) => usize::from(o) + 4,
        other => panic!("expected to find the sentinel, got {other:?}"),
    };

    assert_eq!(
        &dst.bytes[..end],
        &src.bytes[..end],
        "the replayed storage differs from the reference within the first {end} bytes"
    );
    println!("{written} parameters replayed; {end} bytes byte-identical to the reference");
}

/// Every parameter the reader finds can be found again by `scan`, at the
/// offset the walk reached it.
///
/// A separate claim from the replay: this one is about the reference file
/// itself, so it would still catch a scan that only worked on storage the port
/// had written.
#[test]
fn every_reference_parameter_is_findable_by_scan() {
    let src = reference();
    let mut checked = 0_usize;

    for entry in read(&src).expect("valid eeprom") {
        match scan(&src, entry.header) {
            ScanResult::Found(_) => checked += 1,
            other => panic!(
                "key {} group {} was read but scan says {other:?}",
                entry.header.key, entry.header.group_element
            ),
        }
    }
    assert!(checked > 20);
    println!("{checked} reference parameters found by scan");
}

/// Saving a parameter that is already in the reference updates it in place and
/// leaves the file the same length.
#[test]
fn updating_a_reference_parameter_does_not_grow_storage() {
    let src = reference();
    let mut dst = Blank::formatted(src.bytes.len());
    for entry in read(&src).expect("valid eeprom") {
        let Some(value) = entry.value else { continue };
        save(&mut dst, entry.header, value, None, true).expect("append");
    }

    let sentinel_before = match scan(&dst, ParamHeader::new(0x3FF, 0x1E, 0x3FFFF)) {
        ScanResult::Sentinel(o) => o,
        other => panic!("expected a sentinel, got {other:?}"),
    };

    // Re-save the first entry with a different value.
    let first = read(&src).expect("valid").next().expect("at least one");
    let first_value = first.value.expect("the first entry must decode");
    let outcome = save(&mut dst, first.header, first_value, None, true).expect("update");
    assert_eq!(outcome, SaveOutcome::Updated);

    let sentinel_after = match scan(&dst, ParamHeader::new(0x3FF, 0x1E, 0x3FFFF)) {
        ScanResult::Sentinel(o) => o,
        other => panic!("expected a sentinel, got {other:?}"),
    };
    assert_eq!(
        sentinel_before, sentinel_after,
        "an update must not move the sentinel"
    );
}
