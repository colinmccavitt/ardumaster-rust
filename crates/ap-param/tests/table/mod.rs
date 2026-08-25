//! Rebuilds the vehicle's descriptor tables from the structure fixture.
//!
//! Shared by the enumeration and storage tests. The tables are vehicle data
//! rather than port code, which is why they are built here at runtime instead
//! of generated into `src/`: the eventual port assembles them from the modules
//! that own each parameter, and a frozen copy of ArduPilot's would be the wrong
//! thing to carry.

#![allow(
    dead_code,
    reason = "each test uses a different part of this helper module"
)]

use std::collections::HashMap;
use std::path::Path;

use ap_param::{GroupInfo, ParamInfo};

/// One row of `param_structure.csv`.
pub struct Row {
    pub parent_path: String,
    pub pos: usize,
    pub key: u16,
    pub idx: u8,
    pub ptype: u8,
    pub flags: u16,
    pub name: String,
}

/// Read the structure fixture, or `None` if it is not present.
pub fn load_structure(fixtures: &Path) -> Option<Vec<Row>> {
    let text = std::fs::read_to_string(fixtures.join("param_structure.csv")).ok()?;
    let mut rows = Vec::new();
    for line in text.lines().skip(1) {
        let f: Vec<&str> = line.splitn(7, ',').collect();
        if f.len() != 7 {
            continue;
        }
        rows.push(Row {
            parent_path: f[0].to_owned(),
            pos: f[1].parse().expect("pos"),
            key: f[2].parse().expect("key"),
            idx: f[3].parse().expect("idx"),
            ptype: f[4].parse().expect("type"),
            flags: f[5].parse().expect("flags"),
            name: f[6].to_owned(),
        });
    }
    Some(rows)
}

/// The frame mask the reference build had when it produced the fixtures.
///
/// Zero, because `set_frame_type_flags()` runs later in vehicle init than
/// `load_all()` does. Read rather than assumed, so a fixture taken at a
/// different point does not silently change what the port filters.
pub fn load_frame_flags(fixtures: &Path) -> u16 {
    std::fs::read_to_string(fixtures.join("param_frame.csv"))
        .ok()
        .and_then(|s| {
            s.lines()
                .nth(1)
                .and_then(|l| l.split(',').nth(1).map(str::to_owned))
        })
        .and_then(|v| v.trim().parse().ok())
        .expect("param_frame.csv should record frame_type_flags")
}

/// Rebuild the nested group tables below `path`.
///
/// The port's tables borrow rather than owning, so each level is leaked as it
/// is built. Children are built before their parent can reference them, so the
/// construction runs bottom up.
fn build_groups(
    by_parent: &HashMap<String, Vec<&Row>>,
    path: &str,
) -> Option<&'static [GroupInfo<'static>]> {
    let children = by_parent.get(path)?;
    let mut out: Vec<GroupInfo<'static>> = Vec::with_capacity(children.len());
    for (i, r) in children.iter().enumerate() {
        let child_path = format!("{path}.{i}");
        out.push(GroupInfo {
            name: Box::leak(r.name.clone().into_boxed_str()),
            idx: r.idx,
            ptype: r.ptype,
            flags: r.flags,
            group: build_groups(by_parent, &child_path),
        });
    }
    Some(Box::leak(out.into_boxed_slice()))
}

/// Rebuild the whole table.
pub fn build_table(rows: &[Row]) -> Vec<ParamInfo<'static>> {
    let mut by_parent: HashMap<String, Vec<&Row>> = HashMap::new();
    for r in rows {
        by_parent.entry(r.parent_path.clone()).or_default().push(r);
    }
    for v in by_parent.values_mut() {
        v.sort_by_key(|r| r.pos);
    }

    let top = by_parent.get("").cloned().unwrap_or_default();
    top.iter()
        .enumerate()
        .map(|(i, r)| ParamInfo {
            name: Box::leak(r.name.clone().into_boxed_str()),
            key: r.key,
            ptype: r.ptype,
            flags: r.flags,
            group: build_groups(&by_parent, &i.to_string()),
        })
        .collect()
}
