//! Stock `MIS_` leftover, upstream `ArduCopter/Parameters.cpp`.
//!
//! The next Multi `GOBJECT` after `EK3_`. Upstream is `GOBJECTN`.
//! Nested `AP_Mission` `var_info` is not this leftover. Later groups, G2,
//! and `load_parameters` stay later. Heli `H_` / `IM_` are not rows of
//! this leftover.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::parameters::{
    find_ahrs_gobject_var, find_atc_gobject_var, find_avoid_gobject_var, find_baro_gobject_var,
    find_batt_gobject_var, find_brd_gobject_var, find_can_gobject_var, find_compass_gobject_var,
    find_disarm_gobject_var, find_ek2_gobject_var, find_ek3_gobject_var, find_gps_gobject_var,
    find_ins_gobject_var, find_log_gobject_var, find_mis_gobject_var, find_mot_gobject_var,
    find_mount_gobject_var, find_psc_gobject_var, find_rally_gobject_var, find_rcmap_gobject_var,
    find_relay_gobject_var, find_sched_gobject_var, find_sim_gobject_var, find_spray_gobject_var,
    find_tune_var, find_wp_loit_circle_gobject_var, for_each_mis_gobject_param_info,
    mis_gobject_var_info_entry, AHRS_GOBJECT_VAR_INFO, ATC_GOBJECT_VAR_INFO, AVOID_GOBJECT_VAR_INFO,
    BARO_GOBJECT_VAR_INFO, BATT_GOBJECT_VAR_INFO, BRD_GOBJECT_VAR_INFO, CAN_GOBJECT_VAR_INFO,
    COMPASS_GOBJECT_VAR_INFO, DISARM_GOBJECT_VAR_INFO, EK2_GOBJECT_VAR_INFO, EK3_GOBJECT_VAR_INFO,
    FIRST_VAR_INFO, GPS_GOBJECT_VAR_INFO, INS_GOBJECT_VAR_INFO, K_PARAM_MISSION, K_PARAM_NAVEKF3,
    K_PARAM_RSSI, LOG_GOBJECT_VAR_INFO, MIS_GOBJECT_VAR_INFO, MOT_GOBJECT_VAR_INFO,
    MOUNT_GOBJECT_VAR_INFO, PSC_GOBJECT_VAR_INFO, RALLY_GOBJECT_VAR_INFO, RCMAP_GOBJECT_VAR_INFO,
    RELAY_GOBJECT_VAR_INFO, SCHED_GOBJECT_VAR_INFO, SIM_GOBJECT_VAR_INFO, SPRAY_GOBJECT_VAR_INFO,
    TUNE_VAR_INFO, WP_LOIT_CIRCLE_GOBJECT_VAR_INFO,
};
use ap_param::info::{find_by_name, EnumFilter};
use ap_param::VarType;

#[test]
fn table_starts_with_mis() {
    let first = mis_gobject_var_info_entry().expect("MIS_");
    assert_eq!(first.name, "MIS_");
    assert_eq!(first.key, K_PARAM_MISSION);
    assert_eq!(first.key, 42);
    assert_eq!(first.ptype, VarType::Group);
    assert_eq!(first.default.to_bits(), 0.0f32.to_bits());
}

#[test]
fn slice_is_one_stock_gobject() {
    assert_eq!(MIS_GOBJECT_VAR_INFO.len(), 1);
    let names: Vec<_> = MIS_GOBJECT_VAR_INFO.iter().map(|e| e.name).collect();
    assert_eq!(names, ["MIS_"]);
    for entry in MIS_GOBJECT_VAR_INFO {
        assert_eq!(entry.ptype, VarType::Group);
        assert_eq!(entry.default.to_bits(), 0.0f32.to_bits());
    }
}

#[test]
fn keys_match_the_parameters_enum() {
    let entry = find_mis_gobject_var("MIS_").expect("MIS_");
    assert_eq!(entry.key, K_PARAM_MISSION);
    assert_eq!(entry.key, 42);
}

#[test]
fn mis_is_not_the_ek3_or_rssi_key() {
    assert_eq!(K_PARAM_NAVEKF3, 7);
    assert_eq!(K_PARAM_RSSI, 97);
    assert_ne!(K_PARAM_MISSION, K_PARAM_NAVEKF3);
    assert_ne!(K_PARAM_MISSION, K_PARAM_RSSI);
    let entry = find_mis_gobject_var("MIS_").expect("MIS_");
    assert_ne!(entry.key, K_PARAM_NAVEKF3);
    assert_ne!(entry.key, K_PARAM_RSSI);
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
        .chain(EK2_GOBJECT_VAR_INFO)
        .chain(EK3_GOBJECT_VAR_INFO)
        .chain(MIS_GOBJECT_VAR_INFO)
    {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(keys.insert(entry.key), "duplicate key {}", entry.key);
    }
}

#[test]
fn sits_after_ek3_in_table_order() {
    let ek3 = find_ek3_gobject_var("EK3_").expect("EK3_");
    let mis = find_mis_gobject_var("MIS_").expect("MIS_");
    assert_eq!(ek3.key, K_PARAM_NAVEKF3);
    assert_eq!(mis.key, K_PARAM_MISSION);
    assert_eq!(ek3.ptype, VarType::Group);
    assert_eq!(mis.ptype, VarType::Group);
    // `EK3_` is 7; `MIS_` is 42. Both sit after `EK2_` on the compiled
    // Multi table.
    assert!(ek3.key < mis.key);
}

#[test]
fn earlier_leftovers_do_not_include_this_slice() {
    assert!(find_ek3_gobject_var("MIS_").is_none());
    assert!(find_ek2_gobject_var("MIS_").is_none());
    assert!(find_rcmap_gobject_var("MIS_").is_none());
    assert!(find_mot_gobject_var("MIS_").is_none());
    assert!(find_rally_gobject_var("MIS_").is_none());
    assert!(find_avoid_gobject_var("MIS_").is_none());
    assert!(find_sched_gobject_var("MIS_").is_none());
    assert!(find_gps_gobject_var("MIS_").is_none());
    assert!(find_baro_gobject_var("MIS_").is_none());
    assert!(find_sim_gobject_var("MIS_").is_none());
    assert!(find_spray_gobject_var("MIS_").is_none());
    assert!(find_can_gobject_var("MIS_").is_none());
    assert!(find_brd_gobject_var("MIS_").is_none());
    assert!(find_batt_gobject_var("MIS_").is_none());
    assert!(find_mount_gobject_var("MIS_").is_none());
    assert!(find_ahrs_gobject_var("MIS_").is_none());
    assert!(find_psc_gobject_var("MIS_").is_none());
    assert!(find_atc_gobject_var("MIS_").is_none());
    assert!(find_wp_loit_circle_gobject_var("MIS_").is_none());
    assert!(find_ins_gobject_var("MIS_").is_none());
    assert!(find_compass_gobject_var("MIS_").is_none());
    assert!(find_relay_gobject_var("MIS_").is_none());
    assert!(find_disarm_gobject_var("MIS_").is_none());
    assert!(find_log_gobject_var("MIS_").is_none());
    assert!(find_tune_var("MIS_").is_none());
}

#[test]
fn later_rows_are_not_in_this_slice() {
    assert!(find_mis_gobject_var("EK3_").is_none());
    assert!(find_mis_gobject_var("H_").is_none());
    assert!(find_mis_gobject_var("IM_").is_none());
    assert!(find_mis_gobject_var("RSSI_").is_none());
    assert!(find_mis_gobject_var("TUNE_MIN").is_none());
}

#[test]
fn rssi_key_is_not_this_leftover() {
    assert_eq!(K_PARAM_RSSI, 97);
    let entry = find_mis_gobject_var("MIS_").expect("MIS_");
    assert_ne!(entry.key, K_PARAM_RSSI);
    assert!(find_mis_gobject_var("RSSI_").is_none());
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
    for_each_mis_gobject_param_info(&mut |info| {
        table[n] = info;
        n += 1;
    });
    assert_eq!(n, 1);

    let filter = EnumFilter::for_frame(0);
    // Nested `AP_Mission` `var_info` is not this leftover, so the
    // group contributes no children.
    assert!(find_by_name(&table, filter, "MIS_").is_none());
    assert!(find_by_name(&table, filter, "RSSI_").is_none());
    assert!(find_by_name(&table, filter, "H_").is_none());
}
