//! Stock `ATC_` leftover, upstream `ArduCopter/Parameters.cpp`.
//!
//! The next Multi `GOBJECT` after `CIRCLE_`. Upstream is
//! `GOBJECTVARPTR`. Nested `AC_AttitudeControl` `var_info` is not this
//! leftover. `PSC` stays later.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::parameters::{
    atc_gobject_var_info_entry, find_atc_gobject_var, find_compass_gobject_var,
    find_disarm_gobject_var, find_ins_gobject_var, find_log_gobject_var, find_relay_gobject_var,
    find_tune_var, find_wp_loit_circle_gobject_var, for_each_atc_gobject_param_info,
    ATC_GOBJECT_VAR_INFO, COMPASS_GOBJECT_VAR_INFO, DISARM_GOBJECT_VAR_INFO, FIRST_VAR_INFO,
    INS_GOBJECT_VAR_INFO, K_PARAM_ATTITUDE_CONTROL, K_PARAM_CIRCLE_NAV, K_PARAM_POS_CONTROL,
    K_PARAM_WP_NAV, LOG_GOBJECT_VAR_INFO, RELAY_GOBJECT_VAR_INFO, TUNE_VAR_INFO,
    WP_LOIT_CIRCLE_GOBJECT_VAR_INFO,
};
use ap_param::info::{find_by_name, EnumFilter};
use ap_param::VarType;

#[test]
fn table_starts_with_atc() {
    let first = atc_gobject_var_info_entry().expect("ATC_");
    assert_eq!(first.name, "ATC_");
    assert_eq!(first.key, K_PARAM_ATTITUDE_CONTROL);
    assert_eq!(first.key, 102);
    assert_eq!(first.ptype, VarType::Group);
    assert_eq!(first.default.to_bits(), 0.0f32.to_bits());
}

#[test]
fn slice_is_one_stock_gobject() {
    assert_eq!(ATC_GOBJECT_VAR_INFO.len(), 1);
    let names: Vec<_> = ATC_GOBJECT_VAR_INFO.iter().map(|e| e.name).collect();
    assert_eq!(names, ["ATC_"]);
    for entry in ATC_GOBJECT_VAR_INFO {
        assert_eq!(entry.ptype, VarType::Group);
        assert_eq!(entry.default.to_bits(), 0.0f32.to_bits());
    }
}

#[test]
fn keys_match_the_parameters_enum() {
    let entry = find_atc_gobject_var("ATC_").expect("ATC_");
    assert_eq!(entry.key, K_PARAM_ATTITUDE_CONTROL);
    assert_eq!(entry.key, 102);
}

#[test]
fn atc_is_not_the_wp_or_psc_key() {
    assert_eq!(K_PARAM_WP_NAV, 101);
    assert_eq!(K_PARAM_POS_CONTROL, 103);
    assert_ne!(K_PARAM_ATTITUDE_CONTROL, K_PARAM_WP_NAV);
    assert_ne!(K_PARAM_ATTITUDE_CONTROL, K_PARAM_POS_CONTROL);
    let entry = find_atc_gobject_var("ATC_").expect("ATC_");
    assert_ne!(entry.key, K_PARAM_WP_NAV);
    assert_ne!(entry.key, K_PARAM_POS_CONTROL);
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
        .chain(WP_LOIT_CIRCLE_GOBJECT_VAR_INFO)
        .chain(ATC_GOBJECT_VAR_INFO)
    {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(keys.insert(entry.key), "duplicate key {}", entry.key);
    }
}

#[test]
fn sits_after_circle_in_table_order() {
    let circle = find_wp_loit_circle_gobject_var("CIRCLE_").expect("CIRCLE_");
    let atc = find_atc_gobject_var("ATC_").expect("ATC_");
    assert_eq!(circle.key, K_PARAM_CIRCLE_NAV);
    assert_eq!(circle.ptype, VarType::Group);
    assert_eq!(atc.ptype, VarType::Group);
    // `CIRCLE_` is 104; `ATC_` is an earlier enum slot that still sits
    // after `CIRCLE_` on the compiled Multi table.
    assert!(atc.key < circle.key);
}

#[test]
fn earlier_leftovers_do_not_include_this_slice() {
    assert!(find_wp_loit_circle_gobject_var("ATC_").is_none());
    assert!(find_ins_gobject_var("ATC_").is_none());
    assert!(find_compass_gobject_var("ATC_").is_none());
    assert!(find_relay_gobject_var("ATC_").is_none());
    assert!(find_disarm_gobject_var("ATC_").is_none());
    assert!(find_log_gobject_var("ATC_").is_none());
    assert!(find_tune_var("ATC_").is_none());
}

#[test]
fn later_rows_are_not_in_this_slice() {
    assert!(find_atc_gobject_var("CIRCLE_").is_none());
    assert!(find_atc_gobject_var("IM_").is_none());
    assert!(find_atc_gobject_var("PSC").is_none());
    assert!(find_atc_gobject_var("AHRS_").is_none());
    assert!(find_atc_gobject_var("TUNE_MIN").is_none());
}

#[test]
fn psc_key_is_not_this_leftover() {
    assert_eq!(K_PARAM_POS_CONTROL, 103);
    let entry = find_atc_gobject_var("ATC_").expect("ATC_");
    assert_ne!(entry.key, K_PARAM_POS_CONTROL);
    assert!(find_atc_gobject_var("PSC").is_none());
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
    for_each_atc_gobject_param_info(&mut |info| {
        table[n] = info;
        n += 1;
    });
    assert_eq!(n, 1);

    let filter = EnumFilter::for_frame(0);
    // Nested `AC_AttitudeControl` `var_info` is not this leftover, so
    // the group contributes no children.
    assert!(find_by_name(&table, filter, "ATC_").is_none());
    assert!(find_by_name(&table, filter, "PSC").is_none());
}
