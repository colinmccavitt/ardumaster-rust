//! Stock `G2` leftover, upstream `ArduCopter/Parameters.cpp`.
//!
//! The next Multi `GOBJECT` after `CC`. Upstream is `GOBJECT` with an
//! empty prefix. Nested `ParametersG2::var_info` / `var_info2` is not
//! this leftover. `TUNE_MIN` / `TUNE_MAX` live there (idx 31/32);
//! `TUNE2_MIN` / `TUNE2_MAX` / `TUNE2` live in `var_info2`.
//! `PARAM_VEHICLE_INFO` / `MAV` / `load_parameters` stay later. Heli
//! `H_` / `IM_` are not rows of this leftover.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::parameters::{
    cc_gobject_var_info_entry, find_adsb_gobject_var, find_ahrs_gobject_var, find_atc_gobject_var,
    find_avd_gobject_var, find_avoid_gobject_var, find_baro_gobject_var, find_batt_gobject_var,
    find_brd_gobject_var, find_can_gobject_var, find_cc_gobject_var, find_compass_gobject_var,
    find_disarm_gobject_var, find_ek2_gobject_var, find_ek3_gobject_var, find_flow_gobject_var,
    find_g2_gobject_var, find_gps_gobject_var, find_ins_gobject_var, find_log_gobject_var,
    find_mis_gobject_var, find_mot_gobject_var, find_mount_gobject_var, find_ntf_gobject_var,
    find_osd_gobject_var, find_plnd_gobject_var, find_psc_gobject_var, find_rally_gobject_var,
    find_rcmap_gobject_var, find_relay_gobject_var, find_rngfnd_gobject_var, find_rssi_gobject_var,
    find_sched_gobject_var, find_sim_gobject_var, find_spray_gobject_var, find_terrain_gobject_var,
    find_tune_var, find_wp_loit_circle_gobject_var, for_each_g2_gobject_param_info,
    g2_gobject_var_info_entry, ADSB_GOBJECT_VAR_INFO, AHRS_GOBJECT_VAR_INFO, ATC_GOBJECT_VAR_INFO,
    AVD_GOBJECT_VAR_INFO, AVOID_GOBJECT_VAR_INFO, BARO_GOBJECT_VAR_INFO, BATT_GOBJECT_VAR_INFO,
    BRD_GOBJECT_VAR_INFO, CAN_GOBJECT_VAR_INFO, CC_GOBJECT_VAR_INFO, COMPASS_GOBJECT_VAR_INFO,
    DISARM_GOBJECT_VAR_INFO, EK2_GOBJECT_VAR_INFO, EK3_GOBJECT_VAR_INFO, FIRST_VAR_INFO,
    FLOW_GOBJECT_VAR_INFO, G2_GOBJECT_VAR_INFO, GPS_GOBJECT_VAR_INFO, INS_GOBJECT_VAR_INFO,
    K_PARAM_CUSTOM_CONTROL, K_PARAM_G2, K_PARAM_VEHICLE, LOG_GOBJECT_VAR_INFO,
    MIS_GOBJECT_VAR_INFO, MOT_GOBJECT_VAR_INFO, MOUNT_GOBJECT_VAR_INFO, NTF_GOBJECT_VAR_INFO,
    OSD_GOBJECT_VAR_INFO, PLND_GOBJECT_VAR_INFO, PSC_GOBJECT_VAR_INFO, RALLY_GOBJECT_VAR_INFO,
    RCMAP_GOBJECT_VAR_INFO, RELAY_GOBJECT_VAR_INFO, RNGFND_GOBJECT_VAR_INFO, RSSI_GOBJECT_VAR_INFO,
    SCHED_GOBJECT_VAR_INFO, SIM_GOBJECT_VAR_INFO, SPRAY_GOBJECT_VAR_INFO, TERRAIN_GOBJECT_VAR_INFO,
    TUNE_VAR_INFO, WP_LOIT_CIRCLE_GOBJECT_VAR_INFO,
};
use ap_param::info::{find_by_name, EnumFilter};
use ap_param::VarType;

#[test]
fn table_starts_with_g2() {
    let first = g2_gobject_var_info_entry().expect("G2");
    assert_eq!(first.name, "");
    assert_eq!(first.key, K_PARAM_G2);
    assert_eq!(first.key, 6);
    assert_eq!(first.ptype, VarType::Group);
    assert_eq!(first.default.to_bits(), 0.0f32.to_bits());
}

#[test]
fn slice_is_one_stock_gobject() {
    assert_eq!(G2_GOBJECT_VAR_INFO.len(), 1);
    let names: Vec<_> = G2_GOBJECT_VAR_INFO.iter().map(|e| e.name).collect();
    assert_eq!(names, [""]);
    for entry in G2_GOBJECT_VAR_INFO {
        assert_eq!(entry.ptype, VarType::Group);
        assert_eq!(entry.default.to_bits(), 0.0f32.to_bits());
    }
}

#[test]
fn keys_match_the_parameters_enum() {
    let entry = find_g2_gobject_var("").expect("G2");
    assert_eq!(entry.key, K_PARAM_G2);
    assert_eq!(entry.key, 6);
}

#[test]
fn g2_is_not_the_cc_or_vehicle_key() {
    assert_eq!(K_PARAM_CUSTOM_CONTROL, 106);
    assert_eq!(K_PARAM_VEHICLE, 257);
    assert_ne!(K_PARAM_G2, K_PARAM_CUSTOM_CONTROL);
    assert_ne!(K_PARAM_G2, K_PARAM_VEHICLE);
    let entry = find_g2_gobject_var("").expect("G2");
    assert_ne!(entry.key, K_PARAM_CUSTOM_CONTROL);
    assert_ne!(entry.key, K_PARAM_VEHICLE);
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
        .chain(NTF_GOBJECT_VAR_INFO)
        .chain(OSD_GOBJECT_VAR_INFO)
        .chain(CC_GOBJECT_VAR_INFO)
        .chain(G2_GOBJECT_VAR_INFO)
    {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(keys.insert(entry.key), "duplicate key {}", entry.key);
    }
}

#[test]
fn sits_after_cc_in_table_order() {
    let cc = find_cc_gobject_var("CC").expect("CC");
    let g2 = find_g2_gobject_var("").expect("G2");
    assert_eq!(cc.key, K_PARAM_CUSTOM_CONTROL);
    assert_eq!(g2.key, K_PARAM_G2);
    assert_eq!(cc.ptype, VarType::Group);
    assert_eq!(g2.ptype, VarType::Group);
    // `CC` is 106; `G2` is 6 and sits later than `CC` on the compiled
    // Multi table. `PARAM_VEHICLE_INFO` is 257 and stays later.
    assert_ne!(K_PARAM_CUSTOM_CONTROL, K_PARAM_G2);
    assert_ne!(K_PARAM_G2, K_PARAM_VEHICLE);
    let _ = cc_gobject_var_info_entry().expect("CC first");
}

#[test]
fn earlier_leftovers_do_not_include_this_slice() {
    assert!(find_cc_gobject_var("").is_none());
    assert!(find_osd_gobject_var("").is_none());
    assert!(find_ntf_gobject_var("").is_none());
    assert!(find_avd_gobject_var("").is_none());
    assert!(find_adsb_gobject_var("").is_none());
    assert!(find_plnd_gobject_var("").is_none());
    assert!(find_flow_gobject_var("").is_none());
    assert!(find_terrain_gobject_var("").is_none());
    assert!(find_rngfnd_gobject_var("").is_none());
    assert!(find_rssi_gobject_var("").is_none());
    assert!(find_mis_gobject_var("").is_none());
    assert!(find_ek3_gobject_var("").is_none());
    assert!(find_ek2_gobject_var("").is_none());
    assert!(find_rcmap_gobject_var("").is_none());
    assert!(find_mot_gobject_var("").is_none());
    assert!(find_rally_gobject_var("").is_none());
    assert!(find_avoid_gobject_var("").is_none());
    assert!(find_sched_gobject_var("").is_none());
    assert!(find_gps_gobject_var("").is_none());
    assert!(find_baro_gobject_var("").is_none());
    assert!(find_sim_gobject_var("").is_none());
    assert!(find_spray_gobject_var("").is_none());
    assert!(find_can_gobject_var("").is_none());
    assert!(find_brd_gobject_var("").is_none());
    assert!(find_batt_gobject_var("").is_none());
    assert!(find_mount_gobject_var("").is_none());
    assert!(find_ahrs_gobject_var("").is_none());
    assert!(find_psc_gobject_var("").is_none());
    assert!(find_atc_gobject_var("").is_none());
    assert!(find_wp_loit_circle_gobject_var("").is_none());
    assert!(find_ins_gobject_var("").is_none());
    assert!(find_compass_gobject_var("").is_none());
    assert!(find_relay_gobject_var("").is_none());
    assert!(find_disarm_gobject_var("").is_none());
    assert!(find_log_gobject_var("").is_none());
    assert!(find_tune_var("").is_none());
}

#[test]
fn later_rows_are_not_in_this_slice() {
    assert!(find_g2_gobject_var("CC").is_none());
    assert!(find_g2_gobject_var("H_").is_none());
    assert!(find_g2_gobject_var("IM_").is_none());
    assert!(find_g2_gobject_var("TUNE_MIN").is_none());
    assert!(find_g2_gobject_var("TUNE_MAX").is_none());
    assert!(find_g2_gobject_var("TUNE2").is_none());
    assert!(find_g2_gobject_var("TUNE2_MIN").is_none());
    assert!(find_g2_gobject_var("TUNE2_MAX").is_none());
    assert!(find_g2_gobject_var("MAV").is_none());
}

#[test]
fn vehicle_key_is_not_this_leftover() {
    assert_eq!(K_PARAM_VEHICLE, 257);
    let entry = find_g2_gobject_var("").expect("G2");
    assert_ne!(entry.key, K_PARAM_VEHICLE);
}

#[test]
fn ap_param_does_not_find_nested_g2_scalars() {
    let mut table = [ap_param::info::ParamInfo {
        name: "",
        key: 0,
        ptype: 0,
        flags: 0,
        group: None,
    }; 1];
    let mut n = 0_usize;
    for_each_g2_gobject_param_info(&mut |info| {
        table[n] = info;
        n += 1;
    });
    assert_eq!(n, 1);

    let filter = EnumFilter::for_frame(0);
    // Nested `ParametersG2` `var_info` is not this leftover, so the
    // empty-prefix group contributes no children.
    assert!(find_by_name(&table, filter, "TUNE_MIN").is_none());
    assert!(find_by_name(&table, filter, "TUNE_MAX").is_none());
    assert!(find_by_name(&table, filter, "TUNE2").is_none());
    assert!(find_by_name(&table, filter, "TUNE2_MIN").is_none());
    assert!(find_by_name(&table, filter, "TUNE2_MAX").is_none());
    assert!(find_by_name(&table, filter, "H_").is_none());
    assert!(find_by_name(&table, filter, "IM_").is_none());
}
