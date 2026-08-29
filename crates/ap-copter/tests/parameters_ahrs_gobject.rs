//! Stock `AHRS_` leftover, upstream `ArduCopter/Parameters.cpp`.
//!
//! The next Multi `GOBJECT` after `PSC`. Nested `AP_AHRS` `var_info`
//! is not this leftover. Later groups, G2, and `load_parameters` stay
//! later. Heli `IM_` is not a row of this leftover.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::parameters::{
    ahrs_gobject_var_info_entry, find_ahrs_gobject_var, find_atc_gobject_var,
    find_compass_gobject_var, find_disarm_gobject_var, find_ins_gobject_var, find_log_gobject_var,
    find_psc_gobject_var, find_relay_gobject_var, find_tune_var, find_wp_loit_circle_gobject_var,
    for_each_ahrs_gobject_param_info, AHRS_GOBJECT_VAR_INFO, ATC_GOBJECT_VAR_INFO,
    COMPASS_GOBJECT_VAR_INFO, DISARM_GOBJECT_VAR_INFO, FIRST_VAR_INFO, INS_GOBJECT_VAR_INFO,
    K_PARAM_AHRS, K_PARAM_CAMERA_MOUNT, K_PARAM_POS_CONTROL, LOG_GOBJECT_VAR_INFO,
    PSC_GOBJECT_VAR_INFO, RELAY_GOBJECT_VAR_INFO, TUNE_VAR_INFO, WP_LOIT_CIRCLE_GOBJECT_VAR_INFO,
};
use ap_param::info::{find_by_name, EnumFilter};
use ap_param::VarType;

#[test]
fn table_starts_with_ahrs() {
    let first = ahrs_gobject_var_info_entry().expect("AHRS_");
    assert_eq!(first.name, "AHRS_");
    assert_eq!(first.key, K_PARAM_AHRS);
    assert_eq!(first.key, 159);
    assert_eq!(first.ptype, VarType::Group);
    assert_eq!(first.default.to_bits(), 0.0f32.to_bits());
}

#[test]
fn slice_is_one_stock_gobject() {
    assert_eq!(AHRS_GOBJECT_VAR_INFO.len(), 1);
    let names: Vec<_> = AHRS_GOBJECT_VAR_INFO.iter().map(|e| e.name).collect();
    assert_eq!(names, ["AHRS_"]);
    for entry in AHRS_GOBJECT_VAR_INFO {
        assert_eq!(entry.ptype, VarType::Group);
        assert_eq!(entry.default.to_bits(), 0.0f32.to_bits());
    }
}

#[test]
fn keys_match_the_parameters_enum() {
    let entry = find_ahrs_gobject_var("AHRS_").expect("AHRS_");
    assert_eq!(entry.key, K_PARAM_AHRS);
    assert_eq!(entry.key, 159);
}

#[test]
fn ahrs_is_not_the_psc_or_mnt_key() {
    assert_eq!(K_PARAM_POS_CONTROL, 103);
    assert_eq!(K_PARAM_CAMERA_MOUNT, 166);
    assert_ne!(K_PARAM_AHRS, K_PARAM_POS_CONTROL);
    assert_ne!(K_PARAM_AHRS, K_PARAM_CAMERA_MOUNT);
    let entry = find_ahrs_gobject_var("AHRS_").expect("AHRS_");
    assert_ne!(entry.key, K_PARAM_POS_CONTROL);
    assert_ne!(entry.key, K_PARAM_CAMERA_MOUNT);
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
        .chain(PSC_GOBJECT_VAR_INFO)
        .chain(AHRS_GOBJECT_VAR_INFO)
    {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(keys.insert(entry.key), "duplicate key {}", entry.key);
    }
}

#[test]
fn sits_after_psc_in_table_order() {
    let psc = find_psc_gobject_var("PSC").expect("PSC");
    let ahrs = find_ahrs_gobject_var("AHRS_").expect("AHRS_");
    assert_eq!(psc.key, K_PARAM_POS_CONTROL);
    assert_eq!(psc.ptype, VarType::Group);
    assert_eq!(ahrs.ptype, VarType::Group);
    // `PSC` is 103; `AHRS_` is 159. Table order and key order agree.
    assert!(ahrs.key > psc.key);
}

#[test]
fn earlier_leftovers_do_not_include_this_slice() {
    assert!(find_psc_gobject_var("AHRS_").is_none());
    assert!(find_atc_gobject_var("AHRS_").is_none());
    assert!(find_wp_loit_circle_gobject_var("AHRS_").is_none());
    assert!(find_ins_gobject_var("AHRS_").is_none());
    assert!(find_compass_gobject_var("AHRS_").is_none());
    assert!(find_relay_gobject_var("AHRS_").is_none());
    assert!(find_disarm_gobject_var("AHRS_").is_none());
    assert!(find_log_gobject_var("AHRS_").is_none());
    assert!(find_tune_var("AHRS_").is_none());
}

#[test]
fn later_rows_are_not_in_this_slice() {
    assert!(find_ahrs_gobject_var("PSC").is_none());
    assert!(find_ahrs_gobject_var("IM_").is_none());
    assert!(find_ahrs_gobject_var("MNT").is_none());
    assert!(find_ahrs_gobject_var("TUNE_MIN").is_none());
}

#[test]
fn mnt_key_is_not_this_leftover() {
    assert_eq!(K_PARAM_CAMERA_MOUNT, 166);
    let entry = find_ahrs_gobject_var("AHRS_").expect("AHRS_");
    assert_ne!(entry.key, K_PARAM_CAMERA_MOUNT);
    assert!(find_ahrs_gobject_var("MNT").is_none());
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
    for_each_ahrs_gobject_param_info(&mut |info| {
        table[n] = info;
        n += 1;
    });
    assert_eq!(n, 1);

    let filter = EnumFilter::for_frame(0);
    // Nested `AP_AHRS` `var_info` is not this leftover, so the
    // group contributes no children.
    assert!(find_by_name(&table, filter, "AHRS_").is_none());
    assert!(find_by_name(&table, filter, "MNT").is_none());
}
