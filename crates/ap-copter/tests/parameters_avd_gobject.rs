//! Stock `AVD_` leftover, upstream `ArduCopter/Parameters.cpp`.
//!
//! The next Multi `GOBJECT` after `ADSB_`. Upstream is `GOBJECT`.
//! Nested `AP_Avoidance_Copter` `var_info` is not this leftover. Later groups, G2,
//! and `load_parameters` stay later. Heli `H_` / `IM_` are not rows of
//! this leftover.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::parameters::{
    avd_gobject_var_info_entry, find_adsb_gobject_var, find_ahrs_gobject_var, find_atc_gobject_var,
    find_avd_gobject_var, find_avoid_gobject_var, find_baro_gobject_var, find_batt_gobject_var,
    find_brd_gobject_var, find_can_gobject_var, find_compass_gobject_var, find_disarm_gobject_var,
    find_ek2_gobject_var, find_ek3_gobject_var, find_flow_gobject_var, find_gps_gobject_var,
    find_ins_gobject_var, find_log_gobject_var, find_mis_gobject_var, find_mot_gobject_var,
    find_mount_gobject_var, find_plnd_gobject_var, find_psc_gobject_var, find_rally_gobject_var,
    find_rcmap_gobject_var, find_relay_gobject_var, find_rngfnd_gobject_var, find_rssi_gobject_var,
    find_sched_gobject_var, find_sim_gobject_var, find_spray_gobject_var, find_terrain_gobject_var,
    find_tune_var, find_wp_loit_circle_gobject_var, for_each_avd_gobject_param_info,
    ADSB_GOBJECT_VAR_INFO, AHRS_GOBJECT_VAR_INFO, ATC_GOBJECT_VAR_INFO, AVD_GOBJECT_VAR_INFO,
    AVOID_GOBJECT_VAR_INFO, BARO_GOBJECT_VAR_INFO, BATT_GOBJECT_VAR_INFO, BRD_GOBJECT_VAR_INFO,
    CAN_GOBJECT_VAR_INFO, COMPASS_GOBJECT_VAR_INFO, DISARM_GOBJECT_VAR_INFO, EK2_GOBJECT_VAR_INFO,
    EK3_GOBJECT_VAR_INFO, FIRST_VAR_INFO, FLOW_GOBJECT_VAR_INFO, GPS_GOBJECT_VAR_INFO,
    INS_GOBJECT_VAR_INFO, K_PARAM_ADSB, K_PARAM_AVOIDANCE_ADSB, K_PARAM_NOTIFY,
    LOG_GOBJECT_VAR_INFO, MIS_GOBJECT_VAR_INFO, MOT_GOBJECT_VAR_INFO, MOUNT_GOBJECT_VAR_INFO,
    PLND_GOBJECT_VAR_INFO, PSC_GOBJECT_VAR_INFO, RALLY_GOBJECT_VAR_INFO, RCMAP_GOBJECT_VAR_INFO,
    RELAY_GOBJECT_VAR_INFO, RNGFND_GOBJECT_VAR_INFO, RSSI_GOBJECT_VAR_INFO, SCHED_GOBJECT_VAR_INFO,
    SIM_GOBJECT_VAR_INFO, SPRAY_GOBJECT_VAR_INFO, TERRAIN_GOBJECT_VAR_INFO, TUNE_VAR_INFO,
    WP_LOIT_CIRCLE_GOBJECT_VAR_INFO,
};
use ap_param::info::{find_by_name, EnumFilter};
use ap_param::VarType;

#[test]
fn table_starts_with_avd() {
    let first = avd_gobject_var_info_entry().expect("AVD_");
    assert_eq!(first.name, "AVD_");
    assert_eq!(first.key, K_PARAM_AVOIDANCE_ADSB);
    assert_eq!(first.key, 96);
    assert_eq!(first.ptype, VarType::Group);
    assert_eq!(first.default.to_bits(), 0.0f32.to_bits());
}

#[test]
fn slice_is_one_stock_gobject() {
    assert_eq!(AVD_GOBJECT_VAR_INFO.len(), 1);
    let names: Vec<_> = AVD_GOBJECT_VAR_INFO.iter().map(|e| e.name).collect();
    assert_eq!(names, ["AVD_"]);
    for entry in AVD_GOBJECT_VAR_INFO {
        assert_eq!(entry.ptype, VarType::Group);
        assert_eq!(entry.default.to_bits(), 0.0f32.to_bits());
    }
}

#[test]
fn keys_match_the_parameters_enum() {
    let entry = find_avd_gobject_var("AVD_").expect("AVD_");
    assert_eq!(entry.key, K_PARAM_AVOIDANCE_ADSB);
    assert_eq!(entry.key, 96);
}

#[test]
fn avd_is_not_the_adsb_or_ntf_key() {
    assert_eq!(K_PARAM_ADSB, 72);
    assert_eq!(K_PARAM_NOTIFY, 73);
    assert_ne!(K_PARAM_AVOIDANCE_ADSB, K_PARAM_ADSB);
    assert_ne!(K_PARAM_AVOIDANCE_ADSB, K_PARAM_NOTIFY);
    let entry = find_avd_gobject_var("AVD_").expect("AVD_");
    assert_ne!(entry.key, K_PARAM_ADSB);
    assert_ne!(entry.key, K_PARAM_NOTIFY);
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
        .chain(RSSI_GOBJECT_VAR_INFO)
        .chain(RNGFND_GOBJECT_VAR_INFO)
        .chain(TERRAIN_GOBJECT_VAR_INFO)
        .chain(FLOW_GOBJECT_VAR_INFO)
        .chain(PLND_GOBJECT_VAR_INFO)
        .chain(ADSB_GOBJECT_VAR_INFO)
        .chain(AVD_GOBJECT_VAR_INFO)
    {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(keys.insert(entry.key), "duplicate key {}", entry.key);
    }
}

#[test]
fn sits_after_adsb_in_table_order() {
    let adsb = find_adsb_gobject_var("ADSB_").expect("ADSB_");
    let avd = find_avd_gobject_var("AVD_").expect("AVD_");
    assert_eq!(adsb.key, K_PARAM_ADSB);
    assert_eq!(avd.key, K_PARAM_AVOIDANCE_ADSB);
    assert_eq!(adsb.ptype, VarType::Group);
    assert_eq!(avd.ptype, VarType::Group);
    // `ADSB_` is 72; `AVD_` is 96 and sits later than `ADSB_`
    // on the compiled Multi table. `NTF_` is 73 and stays later.
    assert_ne!(K_PARAM_ADSB, K_PARAM_AVOIDANCE_ADSB);
    assert_ne!(K_PARAM_AVOIDANCE_ADSB, K_PARAM_NOTIFY);
}

#[test]
fn earlier_leftovers_do_not_include_this_slice() {
    assert!(find_adsb_gobject_var("AVD_").is_none());
    assert!(find_plnd_gobject_var("AVD_").is_none());
    assert!(find_flow_gobject_var("AVD_").is_none());
    assert!(find_terrain_gobject_var("AVD_").is_none());
    assert!(find_rngfnd_gobject_var("AVD_").is_none());
    assert!(find_rssi_gobject_var("AVD_").is_none());
    assert!(find_mis_gobject_var("AVD_").is_none());
    assert!(find_ek3_gobject_var("AVD_").is_none());
    assert!(find_ek2_gobject_var("AVD_").is_none());
    assert!(find_rcmap_gobject_var("AVD_").is_none());
    assert!(find_mot_gobject_var("AVD_").is_none());
    assert!(find_rally_gobject_var("AVD_").is_none());
    assert!(find_avoid_gobject_var("AVD_").is_none());
    assert!(find_sched_gobject_var("AVD_").is_none());
    assert!(find_gps_gobject_var("AVD_").is_none());
    assert!(find_baro_gobject_var("AVD_").is_none());
    assert!(find_sim_gobject_var("AVD_").is_none());
    assert!(find_spray_gobject_var("AVD_").is_none());
    assert!(find_can_gobject_var("AVD_").is_none());
    assert!(find_brd_gobject_var("AVD_").is_none());
    assert!(find_batt_gobject_var("AVD_").is_none());
    assert!(find_mount_gobject_var("AVD_").is_none());
    assert!(find_ahrs_gobject_var("AVD_").is_none());
    assert!(find_psc_gobject_var("AVD_").is_none());
    assert!(find_atc_gobject_var("AVD_").is_none());
    assert!(find_wp_loit_circle_gobject_var("AVD_").is_none());
    assert!(find_ins_gobject_var("AVD_").is_none());
    assert!(find_compass_gobject_var("AVD_").is_none());
    assert!(find_relay_gobject_var("AVD_").is_none());
    assert!(find_disarm_gobject_var("AVD_").is_none());
    assert!(find_log_gobject_var("AVD_").is_none());
    assert!(find_tune_var("AVD_").is_none());
}

#[test]
fn later_rows_are_not_in_this_slice() {
    assert!(find_avd_gobject_var("ADSB_").is_none());
    assert!(find_avd_gobject_var("H_").is_none());
    assert!(find_avd_gobject_var("IM_").is_none());
    assert!(find_avd_gobject_var("NTF_").is_none());
    assert!(find_avd_gobject_var("TUNE_MIN").is_none());
}

#[test]
fn ntf_key_is_not_this_leftover() {
    assert_eq!(K_PARAM_NOTIFY, 73);
    let entry = find_avd_gobject_var("AVD_").expect("AVD_");
    assert_ne!(entry.key, K_PARAM_NOTIFY);
    assert!(find_avd_gobject_var("NTF_").is_none());
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
    for_each_avd_gobject_param_info(&mut |info| {
        table[n] = info;
        n += 1;
    });
    assert_eq!(n, 1);

    let filter = EnumFilter::for_frame(0);
    // Nested `AP_Avoidance_Copter` `var_info` is not this leftover, so the
    // group contributes no children.
    assert!(find_by_name(&table, filter, "AVD_").is_none());
    assert!(find_by_name(&table, filter, "NTF_").is_none());
    assert!(find_by_name(&table, filter, "H_").is_none());
}
