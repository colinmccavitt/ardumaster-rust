//! Parity test for AP_Param's on-storage format (FW-004 slice 1, ADR-0010).
//!
//! ADR-0010 makes byte compatibility a requirement rather than a nicety, and
//! the header's layout depends on how the compiler packs a bitfield — which is
//! implementation-defined and so cannot be settled by reading the C++.
//!
//! The fixture is therefore produced by upstream's own compiled code:
//! `tools/parity/gen_param_fixture.py` builds headers through upstream's
//! `set_key`, dumps the raw words, and records what upstream's `get_key` and
//! `is_sentinel` say about each one. Every value here is measured, not
//! transcribed.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fields of a fixture row whose length is checked first; in a test an index fault is a test failure, which is the desired outcome"
)]

use ap_param::{
    EepromHeader, ParamHeader, VarType, EEPROM_HEADER_SIZE, EEPROM_MAGIC, EEPROM_REVISION,
    GROUP_BITS, GROUP_LEVEL_SHIFT, PARAM_HEADER_SIZE, SENTINEL_GROUP, SENTINEL_KEY, SENTINEL_TYPE,
};

fn fixtures_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures"))
        .expect("workspace root")
}

/// The format fixture is a small `kind,name,value` table rather than the
/// numeric row format the replay fixtures use, so it is parsed here.
fn load_format() -> Option<Vec<(String, String, String)>> {
    let path = fixtures_dir().join("param_format.csv");
    let text = std::fs::read_to_string(path).ok()?;
    let mut out = Vec::new();
    for line in text.lines().skip(1) {
        let f: Vec<&str> = line.splitn(3, ',').collect();
        if f.len() == 3 {
            out.push((f[0].to_owned(), f[1].to_owned(), f[2].to_owned()));
        }
    }
    Some(out)
}

#[test]
fn the_format_constants_match_upstream() {
    let Some(rows) = load_format() else {
        eprintln!("skipping: param_format.csv not present");
        return;
    };
    let get = |name: &str| -> u32 {
        rows.iter()
            .find(|(k, n, _)| k == "const" && n == name)
            .unwrap_or_else(|| panic!("{name} missing from the fixture"))
            .2
            .parse()
            .expect("integer")
    };

    assert_eq!(get("eeprom_header_size"), EEPROM_HEADER_SIZE as u32);
    assert_eq!(get("param_header_size"), PARAM_HEADER_SIZE as u32);
    assert_eq!(get("magic0"), u32::from(EEPROM_MAGIC[0]));
    assert_eq!(get("magic1"), u32::from(EEPROM_MAGIC[1]));
    assert_eq!(get("revision"), u32::from(EEPROM_REVISION));
    assert_eq!(get("sentinel_key"), u32::from(SENTINEL_KEY));
    assert_eq!(get("sentinel_type"), u32::from(SENTINEL_TYPE));
    assert_eq!(get("sentinel_group"), SENTINEL_GROUP);
    assert_eq!(get("group_level_shift"), u32::from(GROUP_LEVEL_SHIFT));
    assert_eq!(get("group_bits"), u32::from(GROUP_BITS));

    // the exact word upstream's write_sentinel puts on disk
    assert_eq!(
        get("sentinel_word"),
        ParamHeader::sentinel().to_word(),
        "the sentinel word must be byte-identical or an upstream reader will \
         not stop where it should"
    );

    // and the header the port writes to fresh storage
    let hdr = EepromHeader::default();
    assert_eq!(hdr.to_bytes(), [0x50, 0x41, 6, 0]);
    assert!(hdr.is_valid());
    assert!(!EepromHeader::from_bytes([0x50, 0x41, 5, 0]).is_valid());
    assert!(!EepromHeader::from_bytes([0x50, 0x42, 6, 0]).is_valid());
}

#[test]
fn type_sizes_match_upstream() {
    let Some(rows) = load_format() else {
        eprintln!("skipping: param_format.csv not present");
        return;
    };
    let mut checked = 0;
    for (kind, tag, value) in &rows {
        if kind != "type_size" {
            continue;
        }
        let tag: u8 = tag.parse().expect("integer");
        let want: u8 = value.parse().expect("integer");
        let ty = VarType::from_u8(tag).expect("upstream only emits defined tags here");
        assert_eq!(ty.size(), want, "type_size disagrees for tag {tag}");
        assert_eq!(ty.as_u8(), tag, "the discriminant is part of the format");
        checked += 1;
    }
    assert_eq!(checked, 7, "expected every ap_var_type to be covered");
}

#[test]
fn header_encoding_matches_upstream() {
    let path = fixtures_dir().join("param_header.csv");
    let Ok(text) = std::fs::read_to_string(&path) else {
        eprintln!("skipping: param_header.csv not present");
        return;
    };

    let mut checked = 0usize;
    let mut sentinels = 0usize;
    for line in text.lines().skip(1) {
        let f: Vec<&str> = line.split(',').collect();
        if f.len() != 6 {
            continue;
        }
        let key: u16 = f[0].parse().expect("integer");
        let var_type: u8 = f[1].parse().expect("integer");
        let group: u32 = f[2].parse().expect("integer");
        let want_word: u32 = f[3].parse().expect("integer");
        let want_key: u16 = f[4].parse().expect("integer");
        let want_sentinel = f[5] != "0";

        let hdr = ParamHeader::new(key, var_type, group);

        assert_eq!(
            hdr.to_word(),
            want_word,
            "encoding key={key} type={var_type} group={group}: port {:#010X}, \
             upstream {want_word:#010X}",
            hdr.to_word()
        );
        assert_eq!(hdr.key, want_key, "get_key disagrees for key={key}");
        assert_eq!(
            hdr.is_sentinel(),
            want_sentinel,
            "is_sentinel disagrees for key={key} type={var_type} group={group}"
        );

        // and the round trip, which upstream does implicitly every time it
        // reads storage back
        let decoded = ParamHeader::from_word(want_word);
        assert_eq!(decoded, hdr, "decode is not the inverse of encode");
        assert_eq!(ParamHeader::from_bytes(hdr.to_bytes()), hdr);

        if want_sentinel {
            sentinels += 1;
        }
        checked += 1;
    }

    println!("{checked} header encodings compared, {sentinels} of them sentinels");
    assert!(checked > 500, "too few encodings compared: {checked}");
    assert!(
        sentinels > 0,
        "the fixture covered no sentinel cases, so the terminator logic is \
         untested against upstream"
    );
}

#[test]
fn is_sentinel_matches_upstream_on_raw_storage_words() {
    let Some(rows) = load_format() else {
        eprintln!("skipping: param_format.csv not present");
        return;
    };
    let mut checked = 0;
    for (kind, word, value) in &rows {
        if kind != "is_sentinel_raw" {
            continue;
        }
        let word: u32 = word.parse().expect("integer");
        let (want_key, want_sentinel) = value.split_once(';').expect("key;sentinel");
        let want_key: u16 = want_key.parse().expect("integer");
        let want_sentinel = want_sentinel != "0";

        let hdr = ParamHeader::from_word(word);
        assert_eq!(hdr.key, want_key, "get_key disagrees for word {word:#010X}");
        assert_eq!(
            hdr.is_sentinel(),
            want_sentinel,
            "is_sentinel disagrees for word {word:#010X}"
        );
        checked += 1;
    }
    assert!(checked >= 5, "expected the fill-value cases: {checked}");
}
