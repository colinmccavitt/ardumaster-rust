//! Parity test for the descriptor traversal (FW-004 slice 2, ADR-0010).
//!
//! Two fixtures, and keeping them apart is what makes this test worth running:
//!
//! - `param_structure.csv` is the `var_info` tree as the reference build holds
//!   it — names, indices, types, flags, nesting. This is the *input*.
//! - `param_enumeration.csv` is upstream's own `first()`/`next()` walk of that
//!   tree — full name, key, `token.idx`, `group_element`, type. This is the
//!   *oracle*.
//!
//! Building the port's table from the enumeration would only prove that a
//! lookup finds what was put into it. Building it from the structure and
//! comparing against the enumeration tests the traversal order, the name
//! concatenation and truncation, and the `group_id` encoding — which is what
//! decides where each parameter is stored, and so what ADR-0010 is about.
//!
//! # Why the port enumerates more than the vehicle
//!
//! A group reached through a null pointer contributes no parameters to a
//! running vehicle, and 135 of the tree's top-level entries include several
//! such. The port has no object graph yet — that is a later slice — so it walks
//! everything the tables describe. The test therefore requires that every
//! parameter upstream produced is reproduced exactly, and reports the extras
//! rather than demanding there be none.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture fields whose count is checked first; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use std::collections::HashMap;

use ap_param::{enumerate, EnumFilter, ParamRef};

mod table;

fn fixtures_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures"))
        .expect("workspace root")
}

#[test]
fn the_traversal_reproduces_upstreams_enumeration() {
    let Some(rows) = table::load_structure(&fixtures_dir()) else {
        eprintln!("skipping: param_structure.csv not present");
        return;
    };
    let Ok(oracle_text) = std::fs::read_to_string(fixtures_dir().join("param_enumeration.csv"))
    else {
        eprintln!("skipping: param_enumeration.csv not present");
        return;
    };

    // The mask the oracle was produced under. It is zero, because
    // set_frame_type_flags() runs later in vehicle init than load_all() does,
    // so every entry carrying frame bits is excluded -- which is what the 231
    // "extras" turned out to be, rather than the null pointers I first assumed.
    let frame_type_flags = table::load_frame_flags(&fixtures_dir());

    let table = table::build_table(&rows);
    assert!(table.len() > 100, "table looks empty: {}", table.len());

    // `ParamToken::key` is, despite its name, an INDEX into var_info -- first()
    // and next() walk it as one and never put the storage key there. The oracle
    // therefore carries indices where the port carries keys, and the two
    // coincide only where an entry's index happens to equal its key.
    let vindex_to_key: Vec<u16> = table.iter().map(|i| i.key).collect();

    // key, token_idx, group_element -> (name, type), as the port computes them
    let mut produced: HashMap<(u16, u8, u32), (String, u8, bool)> = HashMap::new();
    let mut count = 0usize;
    enumerate(
        &table,
        EnumFilter::for_frame(frame_type_flags),
        &mut |p: &ParamRef| {
            produced.insert(
                (p.key, p.token_idx, p.group_element),
                (p.name.as_str().to_owned(), p.ptype, p.behind_pointer),
            );
            count += 1;
        },
    );

    let mut matched = 0usize;
    let mut missing = Vec::new();
    let mut wrong_name = Vec::new();
    let mut wrong_type = Vec::new();

    for line in oracle_text.lines().skip(1) {
        let f: Vec<&str> = line.split(',').collect();
        if f.len() < 6 {
            continue;
        }
        let want_name = f[0];
        let vindex: usize = f[1].parse().expect("var_info index");
        let Some(&key) = vindex_to_key.get(vindex) else {
            panic!("oracle names var_info index {vindex}, beyond the {} entries the structure fixture describes", vindex_to_key.len());
        };
        let token_idx: u8 = f[2].parse().expect("idx");
        let group_element: u32 = f[3].parse().expect("group_element");
        let want_type: u8 = f[4].parse().expect("type");

        match produced.get(&(key, token_idx, group_element)) {
            None => {
                if missing.len() < 10 {
                    missing.push(format!(
                        "{want_name} (key={key} idx={token_idx} group={group_element})"
                    ));
                }
            }
            Some((name, ptype, _)) => {
                if name != want_name && wrong_name.len() < 10 {
                    wrong_name.push(format!(
                        "key={key} idx={token_idx} group={group_element}: upstream {want_name:?}, port {name:?}"
                    ));
                }
                if *ptype != want_type && wrong_type.len() < 10 {
                    wrong_type.push(format!(
                        "{want_name}: upstream type {want_type}, port {ptype}"
                    ));
                }
                if name == want_name && *ptype == want_type {
                    matched += 1;
                }
            }
        }
    }

    let oracle_count = oracle_text.lines().count() - 1;

    // Everything the port produced that upstream did not. These should all be
    // groups the running vehicle never allocated; naming them precisely needs
    // the object graph, which is a later slice, so they are reported and
    // bounded rather than explained here.
    let mut oracle_keys = std::collections::HashSet::new();
    for line in oracle_text.lines().skip(1) {
        let f: Vec<&str> = line.split(',').collect();
        if f.len() < 6 {
            continue;
        }
        let vindex: usize = f[1].parse().expect("var_info index");
        if let Some(&key) = vindex_to_key.get(vindex) {
            oracle_keys.insert((
                key,
                f[2].parse::<u8>().expect("idx"),
                f[3].parse::<u32>().expect("group_element"),
            ));
        }
    }
    // Extras are only legitimate where a pointer is involved: the vehicle
    // allocated no object, so upstream's walk found nothing to enumerate.
    let mut extras: Vec<&str> = Vec::new();
    let mut unexplained: Vec<&str> = Vec::new();
    for (k, (name, _, behind_pointer)) in &produced {
        if oracle_keys.contains(k) {
            continue;
        }
        extras.push(name.as_str());
        if !behind_pointer {
            unexplained.push(name.as_str());
        }
    }
    extras.sort_unstable();
    unexplained.sort_unstable();
    println!(
        "  {} extra, all behind pointer groups; first few: {:?}",
        extras.len(),
        &extras[..extras.len().min(6)]
    );
    println!(
        "port enumerated {count} parameters from {} table entries",
        table.len()
    );
    println!("upstream enumerated {oracle_count}; {matched} matched exactly");
    println!(
        "  {} extra in the port, with frame mask {frame_type_flags:#x}",
        count.saturating_sub(oracle_count)
    );

    assert!(
        missing.is_empty(),
        "{} parameter(s) upstream produced were not produced by the port; \
         first few:\n  {}",
        missing.len(),
        missing.join("\n  ")
    );
    assert!(
        wrong_name.is_empty(),
        "name mismatch on {} parameter(s); first few:\n  {}",
        wrong_name.len(),
        wrong_name.join("\n  ")
    );
    assert!(
        wrong_type.is_empty(),
        "type mismatch on {} parameter(s); first few:\n  {}",
        wrong_type.len(),
        wrong_type.join("\n  ")
    );
    assert_eq!(
        matched, oracle_count,
        "every parameter upstream enumerated must be reproduced exactly"
    );
    assert!(
        unexplained.is_empty(),
        "{} parameter(s) the port enumerated and upstream did not sit behind \
         no pointer, so an unallocated object cannot explain them: either the \
         traversal visits something upstream skips or a filter is missing. \
         First few: {:?}",
        unexplained.len(),
        &unexplained[..unexplained.len().min(8)]
    );
}
