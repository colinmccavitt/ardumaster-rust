//! Stock `CAN_` leftover, upstream `ArduCopter/Parameters.cpp`.
//!
//! The next Multi `GOBJECT` after `BRD_`. `CAN_` is
//! `HAL_MAX_CAN_PROTOCOL_DRIVERS`. Nested `AP_CANManager` `var_info`
//! is not this leftover. Later groups, G2, and `load_parameters` stay
//! later. Heli `IM_` is not a row of this leftover.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::parameters::{
    can_gobject_var_info_entry, find_ahrs_gobject_var, find_atc_gobject_var, find_batt_gobject_var,
    find_brd_gobject_var, find_can_gobject_var, find_compass_gobject_var, find_disarm_gobject_var,
    find_ins_gobject_var, find_log_gobject_var, find_mount_gobject_var, find_psc_gobject_var,
    find_relay_gobject_var, find_tune_var, find_wp_loit_circle_gobject_var,
    for_each_can_gobject_param_info, AHRS_GOBJECT_VAR_INFO, ATC_GOBJECT_VAR_INFO,
    BATT_GOBJECT_VAR_INFO, BRD_GOBJECT_VAR_INFO, CAN_GOBJECT_VAR_INFO, COMPASS_GOBJECT_VAR_INFO,
    DISARM_GOBJECT_VAR_INFO, FIRST_VAR_INFO, INS_GOBJECT_VAR_INFO, K_PARAM_BOARDCONFIG,
    K_PARAM_CAN_MGR, K_PARAM_SPRAYER, LOG_GOBJECT_VAR_INFO, MOUNT_GOBJECT_VAR_INFO,
    PSC_GOBJECT_VAR_INFO, RELAY_GOBJECT_VAR_INFO, TUNE_VAR_INFO, WP_LOIT_CIRCLE_GOBJECT_VAR_INFO,
};
use ap_param::info::{find_by_name, EnumFilter};
use ap_param::VarType;

#[test]
fn table_starts_with_can() {
    let first = can_gobject_var_info_entry().expect("CAN_");
    assert_eq!(first.name, "CAN_");
    assert_eq!(first.key, K_PARAM_CAN_MGR);
    assert_eq!(first.key, 8);
    assert_eq!(first.ptype, VarType::Group);
    assert_eq!(first.default.to_bits(), 0.0f32.to_bits());
}

#[test]
fn slice_is_one_stock_gobject() {
    assert_eq!(CAN_GOBJECT_VAR_INFO.len(), 1);
    let names: Vec<_> = CAN_GOBJECT_VAR_INFO.iter().map(|e| e.name).collect();
    assert_eq!(names, ["CAN_"]);
    for entry in CAN_GOBJECT_VAR_INFO {
        assert_eq!(entry.ptype, VarType::Group);
        assert_eq!(entry.default.to_bits(), 0.0f32.to_bits());
    }
}

#[test]
fn keys_match_the_parameters_enum() {
    let entry = find_can_gobject_var("CAN_").expect("CAN_");
    assert_eq!(entry.key, K_PARAM_CAN_MGR);
    assert_eq!(entry.key, 8);
}

#[test]
fn can_is_not_the_brd_or_spray_key() {
    assert_eq!(K_PARAM_BOARDCONFIG, 15);
    assert_eq!(K_PARAM_SPRAYER, 33);
    assert_ne!(K_PARAM_CAN_MGR, K_PARAM_BOARDCONFIG);
    assert_ne!(K_PARAM_CAN_MGR, K_PARAM_SPRAYER);
    let entry = find_can_gobject_var("CAN_").expect("CAN_");
    assert_ne!(entry.key, K_PARAM_BOARDCONFIG);
    assert_ne!(entry.key, K_PARAM_SPRAYER);
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
        .chain(MOUNT_GOBJECT_VAR_INFO)
        .chain(BATT_GOBJECT_VAR_INFO)
        .chain(BRD_GOBJECT_VAR_INFO)
        .chain(CAN_GOBJECT_VAR_INFO)
    {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(keys.insert(entry.key), "duplicate key {}", entry.key);
    }
}

#[test]
fn sits_after_brd_in_table_order() {
    let brd = find_brd_gobject_var("BRD_").expect("BRD_");
    let can = find_can_gobject_var("CAN_").expect("CAN_");
    assert_eq!(brd.key, K_PARAM_BOARDCONFIG);
    assert_eq!(brd.ptype, VarType::Group);
    assert_eq!(can.ptype, VarType::Group);
    // `BRD_` is 15; `CAN_` is an earlier enum slot that still sits
    // after `BRD_` on the compiled Multi table.
    assert!(can.key < brd.key);
}

#[test]
fn earlier_leftovers_do_not_include_this_slice() {
    assert!(find_brd_gobject_var("CAN_").is_none());
    assert!(find_batt_gobject_var("CAN_").is_none());
    assert!(find_mount_gobject_var("CAN_").is_none());
    assert!(find_ahrs_gobject_var("CAN_").is_none());
    assert!(find_psc_gobject_var("CAN_").is_none());
    assert!(find_atc_gobject_var("CAN_").is_none());
    assert!(find_wp_loit_circle_gobject_var("CAN_").is_none());
    assert!(find_ins_gobject_var("CAN_").is_none());
    assert!(find_compass_gobject_var("CAN_").is_none());
    assert!(find_relay_gobject_var("CAN_").is_none());
    assert!(find_disarm_gobject_var("CAN_").is_none());
    assert!(find_log_gobject_var("CAN_").is_none());
    assert!(find_tune_var("CAN_").is_none());
}

#[test]
fn later_rows_are_not_in_this_slice() {
    assert!(find_can_gobject_var("BRD_").is_none());
    assert!(find_can_gobject_var("IM_").is_none());
    assert!(find_can_gobject_var("SPRAY_").is_none());
    assert!(find_can_gobject_var("TUNE_MIN").is_none());
}

#[test]
fn spray_key_is_not_this_leftover() {
    assert_eq!(K_PARAM_SPRAYER, 33);
    let entry = find_can_gobject_var("CAN_").expect("CAN_");
    assert_ne!(entry.key, K_PARAM_SPRAYER);
    assert!(find_can_gobject_var("SPRAY_").is_none());
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
    for_each_can_gobject_param_info(&mut |info| {
        table[n] = info;
        n += 1;
    });
    assert_eq!(n, 1);

    let filter = EnumFilter::for_frame(0);
    // Nested `AP_CANManager` `var_info` is not this leftover, so the
    // group contributes no children.
    assert!(find_by_name(&table, filter, "CAN_").is_none());
    assert!(find_by_name(&table, filter, "SPRAY_").is_none());
}
