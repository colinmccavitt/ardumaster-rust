//! Stock `PARAM_VEHICLE_INFO` / `MAV` leftover plus `load_parameters`
//! conversions, upstream `ArduCopter/Parameters.cpp`.
//!
//! The remaining Multi `Copter::var_info` rows after `G2`. Nested
//! `AP_Vehicle::var_info` / `GCS` `var_info` is not this leftover.
//! Nested `ParametersG2::var_info` (`TUNE_MIN` / `TUNE_MAX`) and
//! `var_info2` (`TUNE2_MIN` / `TUNE2_MAX` / `TUNE2`) are a separate leftover.
//! Heli `H_` / `IM_` are not rows of this leftover.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::parameters::{
    find_adsb_gobject_var, find_ahrs_gobject_var, find_atc_gobject_var, find_avd_gobject_var,
    find_avoid_gobject_var, find_baro_gobject_var, find_batt_gobject_var, find_brd_gobject_var,
    find_can_gobject_var, find_cc_gobject_var, find_compass_gobject_var, find_disarm_gobject_var,
    find_ek2_gobject_var, find_ek3_gobject_var, find_flow_gobject_var, find_g2_gobject_var,
    find_gps_gobject_var, find_ins_gobject_var, find_log_gobject_var, find_mis_gobject_var,
    find_mot_gobject_var, find_mount_gobject_var, find_ntf_gobject_var, find_osd_gobject_var,
    find_plnd_gobject_var, find_psc_gobject_var, find_rally_gobject_var, find_rcmap_gobject_var,
    find_relay_gobject_var, find_rngfnd_gobject_var, find_rssi_gobject_var, find_sched_gobject_var,
    find_sim_gobject_var, find_spray_gobject_var, find_terrain_gobject_var, find_tune_var,
    find_vehicle_mav_var, find_wp_loit_circle_gobject_var, for_each_vehicle_mav_param_info,
    g2_gobject_var_info_entry, vehicle_mav_var_info_entry, ADSB_GOBJECT_VAR_INFO,
    AHRS_GOBJECT_VAR_INFO, ATC_GOBJECT_VAR_INFO, AVD_GOBJECT_VAR_INFO, AVOID_GOBJECT_VAR_INFO,
    BARO_GOBJECT_VAR_INFO, BATT_GOBJECT_VAR_INFO, BRD_GOBJECT_VAR_INFO, CAN_GOBJECT_VAR_INFO,
    CC_GOBJECT_VAR_INFO, COMPASS_GOBJECT_VAR_INFO, COPTER_CLASS_CONVERSIONS,
    COPTER_FENCE_CLASS_CONVERSION, COPTER_G2_CONVERSIONS, COPTER_GCS_CONVERSIONS,
    COPTER_LOGGER_CLASS_CONVERSION, COPTER_PILOT_CONVERSIONS, COPTER_RPM_CLASS_CONVERSION,
    COPTER_SERIAL_CLASS_CONVERSION, DISARM_GOBJECT_VAR_INFO, EK2_GOBJECT_VAR_INFO,
    EK3_GOBJECT_VAR_INFO, FIRST_VAR_INFO, FLOW_GOBJECT_VAR_INFO, G2_GOBJECT_VAR_INFO,
    GPS_GOBJECT_VAR_INFO, INS_GOBJECT_VAR_INFO, K_PARAM_FENCE_OLD, K_PARAM_G2, K_PARAM_GCS,
    K_PARAM_LOGGER, K_PARAM_PILOT_ACCEL_D_CMSS, K_PARAM_PILOT_SPEED_UP_CMS,
    K_PARAM_PILOT_TAKEOFF_ALT_CM, K_PARAM_RPM_SENSOR_OLD, K_PARAM_SERIAL_MANAGER_OLD,
    K_PARAM_SYSID_MY_GCS_OLD, K_PARAM_SYSID_THIS_MAV_OLD, K_PARAM_TELEM_DELAY_OLD, K_PARAM_VEHICLE,
    LOG_GOBJECT_VAR_INFO, MIS_GOBJECT_VAR_INFO, MOT_GOBJECT_VAR_INFO, MOUNT_GOBJECT_VAR_INFO,
    NTF_GOBJECT_VAR_INFO, OSD_GOBJECT_VAR_INFO, PLND_GOBJECT_VAR_INFO, PSC_GOBJECT_VAR_INFO,
    RALLY_GOBJECT_VAR_INFO, RCMAP_GOBJECT_VAR_INFO, RELAY_GOBJECT_VAR_INFO,
    RNGFND_GOBJECT_VAR_INFO, RSSI_GOBJECT_VAR_INFO, SCHED_GOBJECT_VAR_INFO, SIM_GOBJECT_VAR_INFO,
    SPRAY_GOBJECT_VAR_INFO, TERRAIN_GOBJECT_VAR_INFO, TUNE_VAR_INFO, VEHICLE_MAV_VAR_INFO,
    WP_LOIT_CIRCLE_GOBJECT_VAR_INFO,
};
use ap_param::info::{find_by_name, EnumFilter};
use ap_param::VarType;

#[test]
fn table_starts_with_param_vehicle_info() {
    let first = vehicle_mav_var_info_entry().expect("PARAM_VEHICLE_INFO");
    assert_eq!(first.name, "");
    assert_eq!(first.key, K_PARAM_VEHICLE);
    assert_eq!(first.key, 257);
    assert_eq!(first.ptype, VarType::Group);
    assert_eq!(first.default.to_bits(), 0.0f32.to_bits());
}

#[test]
fn slice_is_vehicle_and_mav() {
    assert_eq!(VEHICLE_MAV_VAR_INFO.len(), 2);
    let names: Vec<_> = VEHICLE_MAV_VAR_INFO.iter().map(|e| e.name).collect();
    assert_eq!(names, ["", "MAV"]);
    for entry in VEHICLE_MAV_VAR_INFO {
        assert_eq!(entry.ptype, VarType::Group);
        assert_eq!(entry.default.to_bits(), 0.0f32.to_bits());
    }
    let mav = find_vehicle_mav_var("MAV").expect("MAV");
    assert_eq!(mav.key, K_PARAM_GCS);
    assert_eq!(mav.key, 260);
}

#[test]
fn keys_match_the_parameters_enum() {
    assert_eq!(K_PARAM_VEHICLE, 257);
    assert_eq!(K_PARAM_GCS, 260);
    assert_eq!(K_PARAM_FENCE_OLD, 69);
    assert_eq!(K_PARAM_RPM_SENSOR_OLD, 250);
    assert_eq!(K_PARAM_LOGGER, 253);
    assert_eq!(K_PARAM_SERIAL_MANAGER_OLD, 119);
    assert_eq!(K_PARAM_SYSID_THIS_MAV_OLD, 112);
    assert_eq!(K_PARAM_SYSID_MY_GCS_OLD, 113);
    assert_eq!(K_PARAM_TELEM_DELAY_OLD, 115);
    assert_eq!(K_PARAM_PILOT_SPEED_UP_CMS, 28);
    assert_eq!(K_PARAM_PILOT_ACCEL_D_CMSS, 48);
    assert_eq!(K_PARAM_PILOT_TAKEOFF_ALT_CM, 64);
}

#[test]
fn empty_prefix_is_not_the_g2_key() {
    let vehicle = find_vehicle_mav_var("").expect("PARAM_VEHICLE_INFO");
    let g2 = find_g2_gobject_var("").expect("G2");
    assert_eq!(vehicle.name, "");
    assert_eq!(g2.name, "");
    assert_eq!(vehicle.key, K_PARAM_VEHICLE);
    assert_eq!(g2.key, K_PARAM_G2);
    assert_ne!(vehicle.key, g2.key);
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
        .chain(VEHICLE_MAV_VAR_INFO)
    {
        if entry.name.is_empty() {
            // G2 and PARAM_VEHICLE_INFO both use an empty prefix; keys differ.
            assert!(keys.insert(entry.key), "duplicate key {}", entry.key);
            continue;
        }
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(keys.insert(entry.key), "duplicate key {}", entry.key);
    }
    assert!(keys.contains(&K_PARAM_G2));
    assert!(keys.contains(&K_PARAM_VEHICLE));
}

#[test]
fn sits_after_g2_in_table_order() {
    let g2 = find_g2_gobject_var("").expect("G2");
    let vehicle = find_vehicle_mav_var("").expect("PARAM_VEHICLE_INFO");
    let mav = find_vehicle_mav_var("MAV").expect("MAV");
    assert_eq!(g2.key, K_PARAM_G2);
    assert_eq!(vehicle.key, K_PARAM_VEHICLE);
    assert_eq!(mav.key, K_PARAM_GCS);
    // `G2` is 6; `PARAM_VEHICLE_INFO` is 257 and sits later on the
    // compiled Multi table. `MAV` is 260.
    assert_ne!(K_PARAM_G2, K_PARAM_VEHICLE);
    assert_ne!(K_PARAM_VEHICLE, K_PARAM_GCS);
    let _ = g2_gobject_var_info_entry().expect("G2 first");
}

#[test]
fn earlier_leftovers_do_not_include_mav() {
    assert!(find_g2_gobject_var("MAV").is_none());
    assert!(find_cc_gobject_var("MAV").is_none());
    assert!(find_osd_gobject_var("MAV").is_none());
    assert!(find_ntf_gobject_var("MAV").is_none());
    assert!(find_avd_gobject_var("MAV").is_none());
    assert!(find_adsb_gobject_var("MAV").is_none());
    assert!(find_plnd_gobject_var("MAV").is_none());
    assert!(find_flow_gobject_var("MAV").is_none());
    assert!(find_terrain_gobject_var("MAV").is_none());
    assert!(find_rngfnd_gobject_var("MAV").is_none());
    assert!(find_rssi_gobject_var("MAV").is_none());
    assert!(find_mis_gobject_var("MAV").is_none());
    assert!(find_ek3_gobject_var("MAV").is_none());
    assert!(find_ek2_gobject_var("MAV").is_none());
    assert!(find_rcmap_gobject_var("MAV").is_none());
    assert!(find_mot_gobject_var("MAV").is_none());
    assert!(find_rally_gobject_var("MAV").is_none());
    assert!(find_avoid_gobject_var("MAV").is_none());
    assert!(find_sched_gobject_var("MAV").is_none());
    assert!(find_gps_gobject_var("MAV").is_none());
    assert!(find_baro_gobject_var("MAV").is_none());
    assert!(find_sim_gobject_var("MAV").is_none());
    assert!(find_spray_gobject_var("MAV").is_none());
    assert!(find_can_gobject_var("MAV").is_none());
    assert!(find_brd_gobject_var("MAV").is_none());
    assert!(find_batt_gobject_var("MAV").is_none());
    assert!(find_mount_gobject_var("MAV").is_none());
    assert!(find_ahrs_gobject_var("MAV").is_none());
    assert!(find_psc_gobject_var("MAV").is_none());
    assert!(find_atc_gobject_var("MAV").is_none());
    assert!(find_wp_loit_circle_gobject_var("MAV").is_none());
    assert!(find_ins_gobject_var("MAV").is_none());
    assert!(find_compass_gobject_var("MAV").is_none());
    assert!(find_relay_gobject_var("MAV").is_none());
    assert!(find_disarm_gobject_var("MAV").is_none());
    assert!(find_log_gobject_var("MAV").is_none());
    assert!(find_tune_var("MAV").is_none());
}

#[test]
fn later_rows_are_not_in_this_slice() {
    assert!(find_vehicle_mav_var("CC").is_none());
    assert!(find_vehicle_mav_var("H_").is_none());
    assert!(find_vehicle_mav_var("IM_").is_none());
    assert!(find_vehicle_mav_var("TUNE_MIN").is_none());
    assert!(find_vehicle_mav_var("TUNE_MAX").is_none());
    assert!(find_vehicle_mav_var("TUNE2").is_none());
}

#[test]
fn ap_param_does_not_find_nested_vehicle_or_g2_scalars() {
    let mut table = [ap_param::info::ParamInfo {
        name: "",
        key: 0,
        ptype: 0,
        flags: 0,
        group: None,
    }; 2];
    let mut n = 0_usize;
    for_each_vehicle_mav_param_info(&mut |info| {
        table[n] = info;
        n += 1;
    });
    assert_eq!(n, 2);

    let filter = EnumFilter::for_frame(0);
    assert!(find_by_name(&table, filter, "TUNE_MIN").is_none());
    assert!(find_by_name(&table, filter, "TUNE_MAX").is_none());
    assert!(find_by_name(&table, filter, "H_").is_none());
    assert!(find_by_name(&table, filter, "IM_").is_none());
    assert!(find_by_name(&table, filter, "MAV").is_none());
}

#[test]
fn class_conversions_match_load_parameters() {
    assert_eq!(COPTER_CLASS_CONVERSIONS.len(), 4);
    assert_eq!(COPTER_FENCE_CLASS_CONVERSION.old_key, K_PARAM_FENCE_OLD);
    assert_eq!(COPTER_FENCE_CLASS_CONVERSION.object_name, "FENCE");
    assert!(!COPTER_FENCE_CLASS_CONVERSION.force);
    assert!(COPTER_FENCE_CLASS_CONVERSION.is_top_level);
    assert_eq!(COPTER_RPM_CLASS_CONVERSION.old_key, K_PARAM_RPM_SENSOR_OLD);
    assert_eq!(COPTER_RPM_CLASS_CONVERSION.object_name, "RPM");
    assert!(COPTER_RPM_CLASS_CONVERSION.force);
    assert_eq!(COPTER_LOGGER_CLASS_CONVERSION.old_key, K_PARAM_LOGGER);
    assert_eq!(COPTER_LOGGER_CLASS_CONVERSION.object_name, "LOG");
    assert!(!COPTER_LOGGER_CLASS_CONVERSION.force);
    assert_eq!(
        COPTER_SERIAL_CLASS_CONVERSION.old_key,
        K_PARAM_SERIAL_MANAGER_OLD
    );
    assert_eq!(COPTER_SERIAL_CLASS_CONVERSION.object_name, "SERIAL");
    assert!(COPTER_SERIAL_CLASS_CONVERSION.is_top_level);
}

#[test]
fn g2_conversions_match_load_parameters() {
    assert_eq!(COPTER_G2_CONVERSIONS.len(), 3);
    assert_eq!(COPTER_G2_CONVERSIONS[0].old_index, 12);
    assert_eq!(COPTER_G2_CONVERSIONS[0].object_name, "STAT");
    assert_eq!(COPTER_G2_CONVERSIONS[1].old_index, 30);
    assert_eq!(COPTER_G2_CONVERSIONS[1].object_name, "SCR");
    assert_eq!(COPTER_G2_CONVERSIONS[2].old_index, 13);
    assert_eq!(COPTER_G2_CONVERSIONS[2].object_name, "GRIP");
}

#[test]
fn gcs_conversions_match_load_parameters() {
    assert_eq!(COPTER_GCS_CONVERSIONS.len(), 4);
    assert_eq!(
        COPTER_GCS_CONVERSIONS[0].old_key,
        K_PARAM_SYSID_THIS_MAV_OLD
    );
    assert_eq!(COPTER_GCS_CONVERSIONS[0].new_name, "MAV_SYSID");
    assert_eq!(COPTER_GCS_CONVERSIONS[0].old_type, VarType::Int16);
    assert_eq!(COPTER_GCS_CONVERSIONS[1].old_key, K_PARAM_SYSID_MY_GCS_OLD);
    assert_eq!(COPTER_GCS_CONVERSIONS[1].new_name, "MAV_GCS_SYSID");
    assert_eq!(COPTER_GCS_CONVERSIONS[2].old_key, K_PARAM_G2);
    assert_eq!(COPTER_GCS_CONVERSIONS[2].old_group_element, 11);
    assert_eq!(COPTER_GCS_CONVERSIONS[2].new_name, "MAV_OPTIONS");
    assert_eq!(COPTER_GCS_CONVERSIONS[2].old_type, VarType::Int8);
    assert_eq!(COPTER_GCS_CONVERSIONS[3].old_key, K_PARAM_TELEM_DELAY_OLD);
    assert_eq!(COPTER_GCS_CONVERSIONS[3].new_name, "MAV_TELEM_DELAY");
}

#[test]
fn pilot_conversions_are_scaled_centi() {
    assert_eq!(COPTER_PILOT_CONVERSIONS.len(), 4);
    for entry in COPTER_PILOT_CONVERSIONS {
        assert_eq!(entry.scaler.to_bits(), 0.01f32.to_bits());
    }
    assert_eq!(
        COPTER_PILOT_CONVERSIONS[0].old_key,
        K_PARAM_PILOT_SPEED_UP_CMS
    );
    assert_eq!(COPTER_PILOT_CONVERSIONS[0].new_name, "PILOT_SPD_UP");
    assert_eq!(
        COPTER_PILOT_CONVERSIONS[1].old_key,
        K_PARAM_PILOT_ACCEL_D_CMSS
    );
    assert_eq!(COPTER_PILOT_CONVERSIONS[1].new_name, "PILOT_ACC_Z");
    assert_eq!(
        COPTER_PILOT_CONVERSIONS[2].old_key,
        K_PARAM_PILOT_TAKEOFF_ALT_CM
    );
    assert_eq!(COPTER_PILOT_CONVERSIONS[2].new_name, "PILOT_TKO_ALT_M");
    assert_eq!(COPTER_PILOT_CONVERSIONS[2].old_type, VarType::Float);
    assert_eq!(COPTER_PILOT_CONVERSIONS[3].old_key, K_PARAM_G2);
    assert_eq!(COPTER_PILOT_CONVERSIONS[3].old_group_element, 24);
    assert_eq!(COPTER_PILOT_CONVERSIONS[3].new_name, "PILOT_SPD_DN");
}
