//! Reads a real vehicle's parameter storage (FW-004 slice 3, ADR-0010).
//!
//! The fixture is `eeprom.bin` exactly as the reference SITL build wrote it
//! during the flight that produced the yaw and steering replays. Nothing about
//! it was generated for this test, which is what makes it worth running: if the
//! port's understanding of the format were wrong anywhere, this file would not
//! decode.
//!
//! Each stored entry is matched to a parameter name through the slice-2
//! enumeration — key and group element to name — and its value is checked
//! against the `PARM` records the same flight logged. Storage holds only
//! parameters that differ from their default, so it is a small subset of the
//! log, but every entry in it must agree.

#![allow(
    clippy::float_cmp,
    reason = "an exact match is the expected case; the tolerance below is the exception, and conflating the two would hide which is which"
)]
#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture fields whose count is checked first; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use std::collections::HashMap;

use ap_param::{enumerate, read, EnumFilter, ParamRef, ParamValue, Storage};

mod table;

fn fixtures_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures"))
        .expect("workspace root")
}

/// A whole storage image held in memory.
struct FileStorage(Vec<u8>);

impl Storage for FileStorage {
    fn size(&self) -> u16 {
        u16::try_from(self.0.len()).unwrap_or(u16::MAX)
    }

    fn read(&self, offset: u16, buf: &mut [u8]) -> bool {
        let start = offset as usize;
        match self.0.get(start..start + buf.len()) {
            Some(src) => {
                buf.copy_from_slice(src);
                true
            }
            None => false,
        }
    }

    fn write(&mut self, offset: u16, data: &[u8]) -> bool {
        let start = offset as usize;
        match self.0.get_mut(start..start + data.len()) {
            Some(dst) => {
                dst.copy_from_slice(data);
                true
            }
            None => false,
        }
    }
}

/// Every parameter's value as it stood in the boot that wrote this storage.
///
/// Taken from the enumeration fixture rather than from a flight log, because
/// the two must come from the same boot: a vehicle recalibrates its baro on
/// every start and writes the result straight back to storage, so comparing
/// against another run disagrees on exactly the parameters that are supposed
/// to change.
fn load_values(fixtures: &std::path::Path) -> Option<HashMap<String, f32>> {
    let text = std::fs::read_to_string(fixtures.join("param_enumeration.csv")).ok()?;
    let mut out = HashMap::new();
    for line in text.lines().skip(1) {
        let f: Vec<&str> = line.split(',').collect();
        if f.len() < 7 {
            continue;
        }
        if let Ok(v) = f[6].trim().parse::<f32>() {
            out.insert(f[0].to_owned(), v);
        }
    }
    Some(out)
}

#[test]
fn a_real_vehicles_storage_decodes() {
    let Ok(bytes) = std::fs::read(fixtures_dir().join("eeprom.bin")) else {
        eprintln!("skipping: eeprom.bin not present");
        return;
    };
    let Some(rows) = table::load_structure(&fixtures_dir()) else {
        eprintln!("skipping: param_structure.csv not present");
        return;
    };
    let Some(values) = load_values(&fixtures_dir()) else {
        eprintln!("skipping: parameter enumeration not present");
        return;
    };
    let frame_type_flags = table::load_frame_flags(&fixtures_dir());

    let storage = FileStorage(bytes);
    let table = table::build_table(&rows);

    // key, group_element, token_idx -> name, from slice 2
    let mut names: HashMap<(u16, u32, u8), String> = HashMap::new();
    // Hidden entries are included: storage can contain them, and three of
    // this vehicle's saved parameters are hidden ones.
    enumerate(
        &table,
        EnumFilter::including_hidden(frame_type_flags),
        &mut |p: &ParamRef| {
            names.insert(
                (p.key, p.group_element, p.token_idx),
                p.name.as_str().to_owned(),
            );
        },
    );

    let iter = read(&storage).expect("a real vehicle's storage should have a valid header");

    /// Relative tolerance for a value the vehicle re-derives while running.
    ///
    /// Far tighter than any decoding error could survive: a float read at the
    /// wrong offset or with the wrong byte order is not two parts in a hundred
    /// thousand away from the right answer.
    const DRIFT_TOLERANCE: f32 = 1e-4;

    let mut entries = 0usize;
    let mut checked = 0usize;
    let mut inexact = Vec::new();
    let mut unknown_type = 0usize;
    let mut unnamed = Vec::new();
    let mut mismatched = Vec::new();

    for e in iter {
        entries += 1;
        let Some(value) = e.value else {
            unknown_type += 1;
            continue;
        };

        // A Vector3f occupies one entry but three names.
        let components: &[u8] = if matches!(value, ParamValue::Vector3f(_)) {
            &[1, 2, 3]
        } else {
            &[0]
        };

        for &token_idx in components {
            let key = (e.header.key, e.header.group_element, token_idx);
            let Some(name) = names.get(&key) else {
                if unnamed.len() < 8 {
                    unnamed.push(format!(
                        "key={} group={} idx={token_idx} at offset {}",
                        e.header.key, e.header.group_element, e.offset
                    ));
                }
                continue;
            };
            let Some(stored) = value.component(token_idx) else {
                continue;
            };
            let Some(&want) = values.get(name.as_str()) else {
                // in storage but not enumerated: a hidden or unreachable entry
                continue;
            };
            if stored == want {
                checked += 1;
            } else if (stored - want).abs() <= want.abs() * DRIFT_TOLERANCE {
                checked += 1;
                if inexact.len() < 8 {
                    inexact.push(format!(
                        "{name}: storage {stored}, live {want} ({:.1e} relative)",
                        ((stored - want) / want).abs()
                    ));
                }
            } else if mismatched.len() < 8 {
                mismatched.push(format!("{name}: storage {stored}, live {want}"));
            }
        }
    }

    println!("{entries} entries in the vehicle's storage");
    println!("  {checked} matched the value the vehicle held");
    println!("  {unknown_type} with a type tag this build does not know");
    println!("  {} could not be named", unnamed.len());
    println!(
        "  {} matched only within tolerance (re-derived while running): {:?}",
        inexact.len(),
        inexact
    );

    assert!(
        entries > 20,
        "only {entries} entries decoded, which is too few for a flown vehicle -- \
         the walk is probably stopping early"
    );
    assert_eq!(
        unknown_type, 0,
        "every type tag in storage should be one this build knows"
    );
    assert!(
        unnamed.is_empty(),
        "{} stored parameter(s) could not be matched to a name, so the key or \
         group element the port computes does not agree with what the vehicle \
         wrote; first few:\n  {}",
        unnamed.len(),
        unnamed.join("\n  ")
    );
    assert!(
        mismatched.is_empty(),
        "{} stored value(s) disagree with what the vehicle held in the same \
         boot; first few:\n  {}",
        mismatched.len(),
        mismatched.join("\n  ")
    );
    assert!(
        inexact.len() <= 4,
        "{} value(s) matched only within tolerance, more than the two ground \
         pressures a vehicle re-derives; the tolerance may be hiding a real \
         decoding problem: {:?}",
        inexact.len(),
        inexact
    );
    assert!(checked > 20, "only {checked} values were actually compared");
}

#[test]
fn a_store_with_the_wrong_header_is_refused() {
    let mut bytes = vec![0u8; 64];
    bytes[0] = 0x50;
    bytes[1] = 0x41;
    bytes[2] = 5; // an older revision
    assert!(matches!(
        read(&FileStorage(bytes.clone())),
        Err(ap_param::StorageError::BadRevision { found: 5, .. })
    ));

    bytes[1] = 0x42;
    bytes[2] = 6;
    assert!(matches!(
        read(&FileStorage(bytes)),
        Err(ap_param::StorageError::BadMagic)
    ));
}
