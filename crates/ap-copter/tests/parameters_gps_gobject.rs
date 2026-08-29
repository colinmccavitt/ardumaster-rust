//! Stock `GPS` leftover, upstream `ArduCopter/Parameters.cpp`.
//!
//! The next Multi `GOBJECT` after `BARO`. Nested `AP_GPS` `var_info`
//! is not this leftover. Later groups, G2, and `load_parameters` stay
//! later. Heli `IM_` is not a row of this leftover.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::parameters::{
    find_ahrs_gobject_var, find_atc_gobject_var, find_baro_gobject_var, find_batt_gobject_var,
    find_brd_gobject_var, find_can_gobject_var, find_compass_gobject_var, find_disarm_gobject_var,
    find_gps_gobject_var, find_ins_gobject_var, find_log_gobject_var, find_mount_gobject_var,
    find_psc_gobject_var, find_relay_gobject_var, find_sim_gobject_var, find_spray_gobject_var,
    find_tune_var, find_wp_loit_circle_gobject_var, for_each_gps_gobject_param_info,
    gps_gobject_var_info_entry, AHRS_GOBJECT_VAR_INFO, ATC_GOBJECT_VAR_INFO, BARO_GOBJECT_VAR_INFO,
    BATT_GOBJECT_VAR_INFO, BRD_GOBJECT_VAR_INFO, CAN_GOBJECT_VAR_INFO, COMPASS_GOBJECT_VAR_INFO,
    DISARM_GOBJECT_VAR_INFO, FIRST_VAR_INFO, GPS_GOBJECT_VAR_INFO, INS_GOBJECT_VAR_INFO,
    K_PARAM_BAROMETER, K_PARAM_GPS, K_PARAM_SCHEDULER, LOG_GOBJECT_VAR_INFO,
    MOUNT_GOBJECT_VAR_INFO, PSC_GOBJECT_VAR_INFO, RELAY_GOBJECT_VAR_INFO, SIM_GOBJECT_VAR_INFO,
    SPRAY_GOBJECT_VAR_INFO, TUNE_VAR_INFO, WP_LOIT_CIRCLE_GOBJECT_VAR_INFO,
};
use ap_param::info::{find_by_name, EnumFilter};
use ap_param::VarType;

#[test]
fn table_starts_with_gps() {
    let first = gps_gobject_var_info_entry().expect("GPS");
    assert_eq!(first.name, "GPS");
    assert_eq!(first.key, K_PARAM_GPS);
    assert_eq!(first.key, 16);
    assert_eq!(first.ptype, VarType::Group);
    assert_eq!(first.default.to_bits(), 0.0f32.to_bits());
}

#[test]
fn slice_is_one_stock_gobject() {
    assert_eq!(GPS_GOBJECT_VAR_INFO.len(), 1);
    let names: Vec<_> = GPS_GOBJECT_VAR_INFO.iter().map(|e| e.name).collect();
    assert_eq!(names, ["GPS"]);
    for entry in GPS_GOBJECT_VAR_INFO {
        assert_eq!(entry.ptype, VarType::Group);
        assert_eq!(entry.default.to_bits(), 0.0f32.to_bits());
    }
}

#[test]
fn keys_match_the_parameters_enum() {
    let entry = find_gps_gobject_var("GPS").expect("GPS");
    assert_eq!(entry.key, K_PARAM_GPS);
    assert_eq!(entry.key, 16);
}

#[test]
fn gps_is_not_the_baro_or_sched_key() {
    assert_eq!(K_PARAM_BAROMETER, 11);
    assert_eq!(K_PARAM_SCHEDULER, 12);
    assert_ne!(K_PARAM_GPS, K_PARAM_BAROMETER);
    assert_ne!(K_PARAM_GPS, K_PARAM_SCHEDULER);
    let entry = find_gps_gobject_var("GPS").expect("GPS");
    assert_ne!(entry.key, K_PARAM_BAROMETER);
    assert_ne!(entry.key, K_PARAM_SCHEDULER);
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
    {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(keys.insert(entry.key), "duplicate key {}", entry.key);
    }
}

#[test]
fn sits_after_baro_in_table_order() {
    let baro = find_baro_gobject_var("BARO").expect("BARO");
    let gps = find_gps_gobject_var("GPS").expect("GPS");
    assert_eq!(baro.key, K_PARAM_BAROMETER);
    assert_eq!(baro.ptype, VarType::Group);
    assert_eq!(gps.ptype, VarType::Group);
    // `BARO` is 11; `GPS` is a later enum slot and sits after `BARO`
    // on the compiled Multi table.
    assert!(gps.key > baro.key);
}

#[test]
fn earlier_leftovers_do_not_include_this_slice() {
    assert!(find_baro_gobject_var("GPS").is_none());
    assert!(find_sim_gobject_var("GPS").is_none());
    assert!(find_spray_gobject_var("GPS").is_none());
    assert!(find_can_gobject_var("GPS").is_none());
    assert!(find_brd_gobject_var("GPS").is_none());
    assert!(find_batt_gobject_var("GPS").is_none());
    assert!(find_mount_gobject_var("GPS").is_none());
    assert!(find_ahrs_gobject_var("GPS").is_none());
    assert!(find_psc_gobject_var("GPS").is_none());
    assert!(find_atc_gobject_var("GPS").is_none());
    assert!(find_wp_loit_circle_gobject_var("GPS").is_none());
    assert!(find_ins_gobject_var("GPS").is_none());
    assert!(find_compass_gobject_var("GPS").is_none());
    assert!(find_relay_gobject_var("GPS").is_none());
    assert!(find_disarm_gobject_var("GPS").is_none());
    assert!(find_log_gobject_var("GPS").is_none());
    assert!(find_tune_var("GPS").is_none());
}

#[test]
fn later_rows_are_not_in_this_slice() {
    assert!(find_gps_gobject_var("BARO").is_none());
    assert!(find_gps_gobject_var("IM_").is_none());
    assert!(find_gps_gobject_var("SCHED_").is_none());
    assert!(find_gps_gobject_var("TUNE_MIN").is_none());
}

#[test]
fn scheduler_key_is_not_this_leftover() {
    assert_eq!(K_PARAM_SCHEDULER, 12);
    let entry = find_gps_gobject_var("GPS").expect("GPS");
    assert_ne!(entry.key, K_PARAM_SCHEDULER);
    assert!(find_gps_gobject_var("SCHED_").is_none());
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
    for_each_gps_gobject_param_info(&mut |info| {
        table[n] = info;
        n += 1;
    });
    assert_eq!(n, 1);

    let filter = EnumFilter::for_frame(0);
    // Nested `AP_GPS` `var_info` is not this leftover, so the
    // group contributes no children.
    assert!(find_by_name(&table, filter, "GPS").is_none());
    assert!(find_by_name(&table, filter, "SCHED_").is_none());
}
