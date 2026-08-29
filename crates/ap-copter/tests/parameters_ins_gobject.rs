//! Stock `INS` leftover, upstream `ArduCopter/Parameters.cpp`.
//!
//! The next Multi `GOBJECT` after `COMPASS_`. Nested `AP_InertialSensor`
//! `var_info` is not this leftover. `WP_` / `LOIT_` / `CIRCLE_` stay later.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::parameters::{
    find_compass_gobject_var, find_disarm_gobject_var, find_ins_gobject_var, find_log_gobject_var,
    find_relay_gobject_var, find_tune_var, for_each_ins_gobject_param_info,
    ins_gobject_var_info_entry, COMPASS_GOBJECT_VAR_INFO, DISARM_GOBJECT_VAR_INFO, FIRST_VAR_INFO,
    INS_GOBJECT_VAR_INFO, K_PARAM_COMPASS, K_PARAM_INS, K_PARAM_INS_OLD, K_PARAM_WP_NAV,
    LOG_GOBJECT_VAR_INFO, RELAY_GOBJECT_VAR_INFO, TUNE_VAR_INFO,
};
use ap_param::info::{find_by_name, EnumFilter};
use ap_param::VarType;

#[test]
fn table_starts_with_ins() {
    let first = ins_gobject_var_info_entry().expect("INS");
    assert_eq!(first.name, "INS");
    assert_eq!(first.key, K_PARAM_INS);
    assert_eq!(first.key, 3);
    assert_eq!(first.ptype, VarType::Group);
    assert_eq!(first.default.to_bits(), 0.0f32.to_bits());
}

#[test]
fn slice_is_one_stock_gobject() {
    assert_eq!(INS_GOBJECT_VAR_INFO.len(), 1);
    let names: Vec<_> = INS_GOBJECT_VAR_INFO.iter().map(|e| e.name).collect();
    assert_eq!(names, ["INS"]);
    for entry in INS_GOBJECT_VAR_INFO {
        assert_eq!(entry.ptype, VarType::Group);
        assert_eq!(entry.default.to_bits(), 0.0f32.to_bits());
    }
}

#[test]
fn keys_match_the_parameters_enum() {
    let entry = find_ins_gobject_var("INS").expect("INS");
    assert_eq!(entry.key, K_PARAM_INS);
    assert_eq!(entry.key, 3);
}

#[test]
fn ins_is_not_the_deprecated_old_key() {
    assert_eq!(K_PARAM_INS_OLD, 2);
    assert_ne!(K_PARAM_INS, K_PARAM_INS_OLD);
    let entry = find_ins_gobject_var("INS").expect("INS");
    assert_ne!(entry.key, K_PARAM_INS_OLD);
}

#[test]
fn names_and_keys_are_unique_across_leftovers() {
    let mut names = std::collections::BTreeSet::new();
    let mut keys = std::collections::BTreeSet::new();
    for entry in FIRST_VAR_INFO
        .iter()
        .chain(LOG_GOBJECT_VAR_INFO)
        .chain(TUNE_VAR_INFO)
        .chain(DISARM_GOBJECT_VAR_INFO)
        .chain(RELAY_GOBJECT_VAR_INFO)
        .chain(COMPASS_GOBJECT_VAR_INFO)
        .chain(INS_GOBJECT_VAR_INFO)
    {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(keys.insert(entry.key), "duplicate key {}", entry.key);
    }
}

#[test]
fn sits_after_compass_in_table_order() {
    let compass = find_compass_gobject_var("COMPASS_").expect("COMPASS_");
    let ins = find_ins_gobject_var("INS").expect("INS");
    assert_eq!(compass.key, K_PARAM_COMPASS);
    assert_eq!(compass.ptype, VarType::Group);
    assert_eq!(ins.ptype, VarType::Group);
    // `COMPASS_` is 147; `INS` is an earlier enum slot that still sits
    // after `COMPASS_` on the compiled Multi table.
    assert!(ins.key < compass.key);
}

#[test]
fn earlier_leftovers_do_not_include_this_slice() {
    assert!(find_compass_gobject_var("INS").is_none());
    assert!(find_relay_gobject_var("INS").is_none());
    assert!(find_disarm_gobject_var("INS").is_none());
    assert!(find_log_gobject_var("INS").is_none());
    assert!(find_tune_var("INS").is_none());
}

#[test]
fn later_rows_are_not_in_this_slice() {
    assert!(find_ins_gobject_var("COMPASS_").is_none());
    assert!(find_ins_gobject_var("IM_").is_none());
    assert!(find_ins_gobject_var("WP_").is_none());
    assert!(find_ins_gobject_var("LOIT_").is_none());
    assert!(find_ins_gobject_var("CIRCLE_").is_none());
    assert!(find_ins_gobject_var("TUNE_MIN").is_none());
}

#[test]
fn wp_key_is_not_this_leftover() {
    assert_eq!(K_PARAM_WP_NAV, 101);
    let entry = find_ins_gobject_var("INS").expect("INS");
    assert_ne!(entry.key, K_PARAM_WP_NAV);
}

#[test]
fn ap_param_does_not_find_the_empty_group() {
    let mut table = [ap_param::info::ParamInfo {
        name: "",
        key: 0,
        ptype: 0,
        flags: 0,
        group: None,
    }; 1];
    let mut n = 0_usize;
    for_each_ins_gobject_param_info(&mut |info| {
        table[n] = info;
        n += 1;
    });
    assert_eq!(n, 1);

    let filter = EnumFilter::for_frame(0);
    // Nested `AP_InertialSensor` `var_info` is not this leftover, so the
    // group contributes no children.
    assert!(find_by_name(&table, filter, "INS").is_none());
    assert!(find_by_name(&table, filter, "WP_").is_none());
}
