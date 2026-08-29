//! Stock `RCMAP_` leftover, upstream `ArduCopter/Parameters.cpp`.
//!
//! The next Multi `GOBJECT` after `MOT_`. Nested `RCMapper` `var_info`
//! is not this leftover. Later groups, G2, and `load_parameters` stay
//! later. Heli `H_` / `IM_` are not rows of this leftover.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::parameters::{
    find_ahrs_gobject_var, find_atc_gobject_var, find_avoid_gobject_var, find_baro_gobject_var,
    find_batt_gobject_var, find_brd_gobject_var, find_can_gobject_var, find_compass_gobject_var,
    find_disarm_gobject_var, find_gps_gobject_var, find_ins_gobject_var, find_log_gobject_var,
    find_mot_gobject_var, find_mount_gobject_var, find_psc_gobject_var, find_rally_gobject_var,
    find_rcmap_gobject_var, find_relay_gobject_var, find_sched_gobject_var, find_sim_gobject_var,
    find_spray_gobject_var, find_tune_var, find_wp_loit_circle_gobject_var,
    for_each_rcmap_gobject_param_info, rcmap_gobject_var_info_entry, AHRS_GOBJECT_VAR_INFO,
    ATC_GOBJECT_VAR_INFO, AVOID_GOBJECT_VAR_INFO, BARO_GOBJECT_VAR_INFO, BATT_GOBJECT_VAR_INFO,
    BRD_GOBJECT_VAR_INFO, CAN_GOBJECT_VAR_INFO, COMPASS_GOBJECT_VAR_INFO, DISARM_GOBJECT_VAR_INFO,
    FIRST_VAR_INFO, GPS_GOBJECT_VAR_INFO, INS_GOBJECT_VAR_INFO, K_PARAM_MOTORS, K_PARAM_NAVEKF2,
    K_PARAM_RCMAP, LOG_GOBJECT_VAR_INFO, MOT_GOBJECT_VAR_INFO, MOUNT_GOBJECT_VAR_INFO,
    PSC_GOBJECT_VAR_INFO, RALLY_GOBJECT_VAR_INFO, RCMAP_GOBJECT_VAR_INFO, RELAY_GOBJECT_VAR_INFO,
    SCHED_GOBJECT_VAR_INFO, SIM_GOBJECT_VAR_INFO, SPRAY_GOBJECT_VAR_INFO, TUNE_VAR_INFO,
    WP_LOIT_CIRCLE_GOBJECT_VAR_INFO,
};
use ap_param::info::{find_by_name, EnumFilter};
use ap_param::VarType;

#[test]
fn table_starts_with_rcmap() {
    let first = rcmap_gobject_var_info_entry().expect("RCMAP_");
    assert_eq!(first.name, "RCMAP_");
    assert_eq!(first.key, K_PARAM_RCMAP);
    assert_eq!(first.key, 199);
    assert_eq!(first.ptype, VarType::Group);
    assert_eq!(first.default.to_bits(), 0.0f32.to_bits());
}

#[test]
fn slice_is_one_stock_gobject() {
    assert_eq!(RCMAP_GOBJECT_VAR_INFO.len(), 1);
    let names: Vec<_> = RCMAP_GOBJECT_VAR_INFO.iter().map(|e| e.name).collect();
    assert_eq!(names, ["RCMAP_"]);
    for entry in RCMAP_GOBJECT_VAR_INFO {
        assert_eq!(entry.ptype, VarType::Group);
        assert_eq!(entry.default.to_bits(), 0.0f32.to_bits());
    }
}

#[test]
fn keys_match_the_parameters_enum() {
    let entry = find_rcmap_gobject_var("RCMAP_").expect("RCMAP_");
    assert_eq!(entry.key, K_PARAM_RCMAP);
    assert_eq!(entry.key, 199);
}

#[test]
fn rcmap_is_not_the_mot_or_navekf2_key() {
    assert_eq!(K_PARAM_MOTORS, 90);
    assert_eq!(K_PARAM_NAVEKF2, 5);
    assert_ne!(K_PARAM_RCMAP, K_PARAM_MOTORS);
    assert_ne!(K_PARAM_RCMAP, K_PARAM_NAVEKF2);
    let entry = find_rcmap_gobject_var("RCMAP_").expect("RCMAP_");
    assert_ne!(entry.key, K_PARAM_MOTORS);
    assert_ne!(entry.key, K_PARAM_NAVEKF2);
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
        .chain(SPRAY_GOBJECT_VAR_INFO)
        .chain(SIM_GOBJECT_VAR_INFO)
        .chain(BARO_GOBJECT_VAR_INFO)
        .chain(GPS_GOBJECT_VAR_INFO)
        .chain(SCHED_GOBJECT_VAR_INFO)
        .chain(AVOID_GOBJECT_VAR_INFO)
        .chain(RALLY_GOBJECT_VAR_INFO)
        .chain(MOT_GOBJECT_VAR_INFO)
        .chain(RCMAP_GOBJECT_VAR_INFO)
    {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(keys.insert(entry.key), "duplicate key {}", entry.key);
    }
}

#[test]
fn sits_after_mot_in_table_order() {
    let mot = find_mot_gobject_var("MOT_").expect("MOT_");
    let rcmap = find_rcmap_gobject_var("RCMAP_").expect("RCMAP_");
    assert_eq!(mot.key, K_PARAM_MOTORS);
    assert_eq!(rcmap.key, K_PARAM_RCMAP);
    assert_eq!(mot.ptype, VarType::Group);
    assert_eq!(rcmap.ptype, VarType::Group);
    // `MOT_` is 90; `RCMAP_` is a later enum slot and sits after `MOT_`
    // on the compiled Multi table.
    assert!(rcmap.key > mot.key);
}

#[test]
fn earlier_leftovers_do_not_include_this_slice() {
    assert!(find_mot_gobject_var("RCMAP_").is_none());
    assert!(find_rally_gobject_var("RCMAP_").is_none());
    assert!(find_avoid_gobject_var("RCMAP_").is_none());
    assert!(find_sched_gobject_var("RCMAP_").is_none());
    assert!(find_gps_gobject_var("RCMAP_").is_none());
    assert!(find_baro_gobject_var("RCMAP_").is_none());
    assert!(find_sim_gobject_var("RCMAP_").is_none());
    assert!(find_spray_gobject_var("RCMAP_").is_none());
    assert!(find_can_gobject_var("RCMAP_").is_none());
    assert!(find_brd_gobject_var("RCMAP_").is_none());
    assert!(find_batt_gobject_var("RCMAP_").is_none());
    assert!(find_mount_gobject_var("RCMAP_").is_none());
    assert!(find_ahrs_gobject_var("RCMAP_").is_none());
    assert!(find_psc_gobject_var("RCMAP_").is_none());
    assert!(find_atc_gobject_var("RCMAP_").is_none());
    assert!(find_wp_loit_circle_gobject_var("RCMAP_").is_none());
    assert!(find_ins_gobject_var("RCMAP_").is_none());
    assert!(find_compass_gobject_var("RCMAP_").is_none());
    assert!(find_relay_gobject_var("RCMAP_").is_none());
    assert!(find_disarm_gobject_var("RCMAP_").is_none());
    assert!(find_log_gobject_var("RCMAP_").is_none());
    assert!(find_tune_var("RCMAP_").is_none());
}

#[test]
fn later_rows_are_not_in_this_slice() {
    assert!(find_rcmap_gobject_var("MOT_").is_none());
    assert!(find_rcmap_gobject_var("H_").is_none());
    assert!(find_rcmap_gobject_var("IM_").is_none());
    assert!(find_rcmap_gobject_var("EK2_").is_none());
    assert!(find_rcmap_gobject_var("TUNE_MIN").is_none());
}

#[test]
fn navekf2_key_is_not_this_leftover() {
    assert_eq!(K_PARAM_NAVEKF2, 5);
    let entry = find_rcmap_gobject_var("RCMAP_").expect("RCMAP_");
    assert_ne!(entry.key, K_PARAM_NAVEKF2);
    assert!(find_rcmap_gobject_var("EK2_").is_none());
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
    for_each_rcmap_gobject_param_info(&mut |info| {
        table[n] = info;
        n += 1;
    });
    assert_eq!(n, 1);

    let filter = EnumFilter::for_frame(0);
    // Nested `RCMapper` `var_info` is not this leftover, so the
    // group contributes no children.
    assert!(find_by_name(&table, filter, "RCMAP_").is_none());
    assert!(find_by_name(&table, filter, "EK2_").is_none());
    assert!(find_by_name(&table, filter, "H_").is_none());
}
