//! Nested `ParametersG2` TUNE leftover, upstream `ArduCopter/Parameters.cpp`.
//!
//! `TUNE_MIN` / `TUNE_MAX` live in `ParametersG2::var_info` at idx 31/32.
//! `TUNE2_MIN` / `TUNE2_MAX` / `TUNE2` live in `var_info2` (empty-prefix
//! `AP_SUBGROUPEXTENSION` idx 61, then idx 11/12/13). Compiled only when
//! `AP_RC_TRANSMITTER_TUNING_ENABLED`. Heli `H_` / `IM_` are not rows of
//! this leftover.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an index fault is a test failure, which is the desired outcome"
)]

use ap_copter::parameters::{
    find_g2_gobject_var, find_g2_tune_var, find_tune_var, find_vehicle_mav_var,
    for_each_g2_tune_param_info, g2_tune_parent_param_info, g2_tune_var_info_entry,
    G2_TUNE2_GROUP_INFO, G2_TUNE2_IDX, G2_TUNE2_MAX_IDX, G2_TUNE2_MIN_IDX, G2_TUNE2_VAR_INFO,
    G2_TUNE_MAX_IDX, G2_TUNE_MIN_IDX, G2_TUNE_NESTED_GROUP_INFO, G2_TUNE_VAR_INFO,
    G2_VAR_INFO2_EXTENSION_IDX, K_PARAM_G2, K_PARAM_RC_TUNING_PARAM, TUNE_NONE, TUNE_VAR_INFO,
};
use ap_param::info::{find_by_name, group_id, EnumFilter, GROUP_LEVEL_SHIFT};
use ap_param::VarType;

#[test]
fn table_starts_with_tune_min() {
    let first = g2_tune_var_info_entry().expect("TUNE_MIN");
    assert_eq!(first.name, "TUNE_MIN");
    assert_eq!(first.idx, G2_TUNE_MIN_IDX);
    assert_eq!(first.idx, 31);
    assert_eq!(first.ptype, VarType::Float);
    assert_eq!(first.default.to_bits(), 0.0f32.to_bits());
}

#[test]
fn slice_is_tune_min_and_tune_max() {
    assert_eq!(G2_TUNE_VAR_INFO.len(), 2);
    let names: Vec<_> = G2_TUNE_VAR_INFO.iter().map(|e| e.name).collect();
    assert_eq!(names, ["TUNE_MIN", "TUNE_MAX"]);
    let max = find_g2_tune_var("TUNE_MAX").expect("TUNE_MAX");
    assert_eq!(max.idx, G2_TUNE_MAX_IDX);
    assert_eq!(max.idx, 32);
    assert_eq!(max.ptype, VarType::Float);
    assert_eq!(max.default.to_bits(), 0.0f32.to_bits());
    for entry in G2_TUNE_VAR_INFO {
        assert_eq!(entry.default.to_bits(), 0.0f32.to_bits());
    }
}

#[test]
fn var_info2_is_tune2_min_max_and_tune2() {
    assert_eq!(G2_TUNE2_VAR_INFO.len(), 3);
    let names: Vec<_> = G2_TUNE2_VAR_INFO.iter().map(|e| e.name).collect();
    assert_eq!(names, ["TUNE2_MIN", "TUNE2_MAX", "TUNE2"]);
    let min = find_g2_tune_var("TUNE2_MIN").expect("TUNE2_MIN");
    assert_eq!(min.idx, G2_TUNE2_MIN_IDX);
    assert_eq!(min.idx, 11);
    assert_eq!(min.ptype, VarType::Float);
    let max = find_g2_tune_var("TUNE2_MAX").expect("TUNE2_MAX");
    assert_eq!(max.idx, G2_TUNE2_MAX_IDX);
    assert_eq!(max.idx, 12);
    assert_eq!(max.ptype, VarType::Float);
    let tune2 = find_g2_tune_var("TUNE2").expect("TUNE2");
    assert_eq!(tune2.idx, G2_TUNE2_IDX);
    assert_eq!(tune2.idx, 13);
    assert_eq!(tune2.ptype, VarType::Int8);
    assert_eq!(tune2.default.to_bits(), (TUNE_NONE as f32).to_bits());
}

#[test]
fn idxs_match_parameters_g2_groupinfo() {
    assert_eq!(G2_TUNE_MIN_IDX, 31);
    assert_eq!(G2_TUNE_MAX_IDX, 32);
    assert_eq!(G2_VAR_INFO2_EXTENSION_IDX, 61);
    assert_eq!(G2_TUNE2_MIN_IDX, 11);
    assert_eq!(G2_TUNE2_MAX_IDX, 12);
    assert_eq!(G2_TUNE2_IDX, 13);
    assert_eq!(K_PARAM_G2, 6);
}

#[test]
fn tune2_is_not_the_toplevel_tune() {
    let tune = find_tune_var("TUNE").expect("TUNE");
    let tune2 = find_g2_tune_var("TUNE2").expect("TUNE2");
    assert_eq!(tune.key, K_PARAM_RC_TUNING_PARAM);
    assert_eq!(tune.key, 187);
    assert_eq!(tune.ptype, VarType::Int8);
    assert_eq!(tune2.ptype, VarType::Int8);
    assert_eq!(tune2.param_info().key, K_PARAM_G2);
    assert_ne!(tune.key, tune2.param_info().key);
    assert!(find_tune_var("TUNE2").is_none());
    assert!(find_g2_tune_var("TUNE").is_none());
}

#[test]
fn names_and_idxs_are_unique() {
    let mut names = std::collections::BTreeSet::new();
    let mut var_info_idxs = std::collections::BTreeSet::new();
    for entry in G2_TUNE_VAR_INFO {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(
            var_info_idxs.insert(entry.idx),
            "duplicate idx {}",
            entry.idx
        );
    }
    let mut var_info2_idxs = std::collections::BTreeSet::new();
    for entry in G2_TUNE2_VAR_INFO {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(
            var_info2_idxs.insert(entry.idx),
            "duplicate idx {}",
            entry.idx
        );
    }
}

#[test]
fn earlier_leftovers_do_not_include_this_slice() {
    assert!(find_g2_gobject_var("TUNE_MIN").is_none());
    assert!(find_g2_gobject_var("TUNE_MAX").is_none());
    assert!(find_g2_gobject_var("TUNE2").is_none());
    assert!(find_g2_gobject_var("TUNE2_MIN").is_none());
    assert!(find_g2_gobject_var("TUNE2_MAX").is_none());
    assert!(find_tune_var("TUNE_MIN").is_none());
    assert!(find_tune_var("TUNE_MAX").is_none());
    assert!(find_vehicle_mav_var("TUNE_MIN").is_none());
    assert!(find_vehicle_mav_var("TUNE2").is_none());
}

#[test]
fn later_rows_are_not_in_this_slice() {
    assert!(find_g2_tune_var("H_").is_none());
    assert!(find_g2_tune_var("IM_").is_none());
    assert!(find_g2_tune_var("").is_none());
    assert!(find_g2_tune_var("MAV").is_none());
    assert!(find_g2_tune_var("CC").is_none());
    assert!(find_g2_tune_var("TUNE").is_none());
}

#[test]
fn nested_group_info_matches_the_catalog() {
    assert_eq!(G2_TUNE_NESTED_GROUP_INFO.len(), 3);
    assert_eq!(G2_TUNE_NESTED_GROUP_INFO[0].name, "TUNE_MIN");
    assert_eq!(G2_TUNE_NESTED_GROUP_INFO[0].idx, G2_TUNE_MIN_IDX);
    assert_eq!(G2_TUNE_NESTED_GROUP_INFO[1].name, "TUNE_MAX");
    assert_eq!(G2_TUNE_NESTED_GROUP_INFO[1].idx, G2_TUNE_MAX_IDX);
    assert_eq!(G2_TUNE_NESTED_GROUP_INFO[2].name, "");
    assert_eq!(G2_TUNE_NESTED_GROUP_INFO[2].idx, G2_VAR_INFO2_EXTENSION_IDX);
    assert_eq!(G2_TUNE_NESTED_GROUP_INFO[2].ptype, VarType::Group.as_u8());
    assert_eq!(G2_TUNE2_GROUP_INFO.len(), 3);
    assert_eq!(G2_TUNE2_GROUP_INFO[0].name, "TUNE2_MIN");
    assert_eq!(G2_TUNE2_GROUP_INFO[2].name, "TUNE2");
    assert_eq!(G2_TUNE2_GROUP_INFO[2].ptype, VarType::Int8.as_u8());
}

#[test]
fn group_elements_match_g2_nesting() {
    let tune_min = group_id(G2_TUNE_MIN_IDX, 0, 0, 0);
    assert_eq!(tune_min, 31);
    let extension = group_id(G2_VAR_INFO2_EXTENSION_IDX, 0, 0, 0);
    assert_eq!(extension, 61);
    let tune2_min = group_id(G2_TUNE2_MIN_IDX, extension, GROUP_LEVEL_SHIFT, 0);
    assert_eq!(tune2_min, 61 + (11u32 << 6));
}

#[test]
fn ap_param_finds_nested_tune_by_name() {
    let table = [g2_tune_parent_param_info()];
    let filter = EnumFilter::for_frame(0);
    let min = find_by_name(&table, filter, "TUNE_MIN").expect("TUNE_MIN");
    assert_eq!(min.key, K_PARAM_G2);
    assert_eq!(min.ptype, VarType::Float.as_u8());
    let max = find_by_name(&table, filter, "TUNE_MAX").expect("TUNE_MAX");
    assert_eq!(max.key, K_PARAM_G2);
    let tune2 = find_by_name(&table, filter, "TUNE2").expect("TUNE2");
    assert_eq!(tune2.key, K_PARAM_G2);
    assert_eq!(tune2.ptype, VarType::Int8.as_u8());
    assert!(find_by_name(&table, filter, "TUNE2_MIN").is_some());
    assert!(find_by_name(&table, filter, "TUNE2_MAX").is_some());
    assert!(find_by_name(&table, filter, "TUNE").is_none());
    assert!(find_by_name(&table, filter, "H_").is_none());
    assert!(find_by_name(&table, filter, "IM_").is_none());
}

#[test]
fn flat_walk_is_five_g2_rows() {
    let mut table = [ap_param::info::ParamInfo {
        name: "",
        key: 0,
        ptype: 0,
        flags: 0,
        group: None,
    }; 5];
    let mut n = 0_usize;
    for_each_g2_tune_param_info(&mut |info| {
        table[n] = info;
        n += 1;
    });
    assert_eq!(n, 5);
    assert_eq!(TUNE_VAR_INFO.len(), 1);

    let filter = EnumFilter::for_frame(0);
    let found = find_by_name(&table, filter, "TUNE_MIN").expect("TUNE_MIN");
    assert_eq!(found.key, K_PARAM_G2);
    assert_eq!(found.ptype, VarType::Float.as_u8());
    assert!(find_by_name(&table, filter, "TUNE2").is_some());
    assert!(find_by_name(&table, filter, "TUNE").is_none());
    assert!(find_by_name(&table, filter, "LOG_BITMASK").is_none());
}
