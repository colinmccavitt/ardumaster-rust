//! `LOG_BITMASK` + first `GOBJECT` leftover, upstream `ArduCopter/Parameters.cpp`.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::parameters::{
    find_first_var, find_log_gobject_var, for_each_log_gobject_param_info,
    log_gobject_var_info_entry, DEFAULT_LOG_BITMASK, FIRST_VAR_INFO, HAL_FRAME_TYPE_DEFAULT,
    K_PARAM_ARMING, K_PARAM_ESC_CALIBRATE, K_PARAM_FRAME_TYPE, K_PARAM_LOG_BITMASK,
    K_PARAM_LOG_BITMASK_OLD, LOG_GOBJECT_VAR_INFO, MASK_LOG_ANY, MASK_LOG_ATTITUDE_FAST,
    MASK_LOG_ATTITUDE_MED, MASK_LOG_CAMERA, MASK_LOG_CMD, MASK_LOG_COMPASS, MASK_LOG_CTUN,
    MASK_LOG_CURRENT, MASK_LOG_GPS, MASK_LOG_IMU, MASK_LOG_INAV, MASK_LOG_MOTBATT, MASK_LOG_NTUN,
    MASK_LOG_OPTFLOW, MASK_LOG_PID, MASK_LOG_PM, MASK_LOG_RCIN, MASK_LOG_RCOUT,
    MOTOR_FRAME_TYPE_PLUS, MOTOR_FRAME_TYPE_X,
};
use ap_param::info::{find_by_name, EnumFilter};
use ap_param::VarType;

#[test]
fn table_starts_with_log_bitmask() {
    let first = log_gobject_var_info_entry().expect("LOG_BITMASK");
    assert_eq!(first.name, "LOG_BITMASK");
    assert_eq!(first.key, K_PARAM_LOG_BITMASK);
    assert_eq!(first.key, 60);
    assert_eq!(first.ptype, VarType::Int32);
    assert_eq!(
        first.default.to_bits(),
        (DEFAULT_LOG_BITMASK as f32).to_bits()
    );
}

#[test]
fn log_bitmask_is_not_the_deprecated_old_key() {
    assert_eq!(K_PARAM_LOG_BITMASK_OLD, 20);
    assert_ne!(K_PARAM_LOG_BITMASK, K_PARAM_LOG_BITMASK_OLD);
    let entry = find_log_gobject_var("LOG_BITMASK").expect("LOG_BITMASK");
    assert_ne!(entry.key, K_PARAM_LOG_BITMASK_OLD);
}

#[test]
fn slice_is_three_gscalars_and_the_first_gobject() {
    assert_eq!(LOG_GOBJECT_VAR_INFO.len(), 4);
    assert_eq!(LOG_GOBJECT_VAR_INFO[0].name, "LOG_BITMASK");
    assert_eq!(LOG_GOBJECT_VAR_INFO[1].name, "ESC_CALIBRATION");
    assert_eq!(LOG_GOBJECT_VAR_INFO[2].name, "FRAME_TYPE");
    assert_eq!(LOG_GOBJECT_VAR_INFO[3].name, "ARMING_");
    assert_eq!(LOG_GOBJECT_VAR_INFO[3].ptype, VarType::Group);
}

#[test]
fn keys_match_the_parameters_enum() {
    let want = [
        ("LOG_BITMASK", K_PARAM_LOG_BITMASK),
        ("ESC_CALIBRATION", K_PARAM_ESC_CALIBRATE),
        ("FRAME_TYPE", K_PARAM_FRAME_TYPE),
        ("ARMING_", K_PARAM_ARMING),
    ];
    for (name, key) in want {
        let entry = find_log_gobject_var(name).unwrap_or_else(|| panic!("missing {name}"));
        assert_eq!(entry.key, key, "{name}");
    }
    assert_eq!(K_PARAM_ESC_CALIBRATE, 186);
    assert_eq!(K_PARAM_FRAME_TYPE, 149);
    assert_eq!(K_PARAM_ARMING, 252);
}

#[test]
fn names_and_keys_are_unique_across_both_leftovers() {
    let mut names = std::collections::BTreeSet::new();
    let mut keys = std::collections::BTreeSet::new();
    for entry in FIRST_VAR_INFO.iter().chain(LOG_GOBJECT_VAR_INFO) {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(keys.insert(entry.key), "duplicate key {}", entry.key);
    }
}

#[test]
fn frame_type_follows_esc_but_has_the_earlier_key() {
    let names: Vec<_> = LOG_GOBJECT_VAR_INFO.iter().map(|e| e.name).collect();
    let esc = names
        .iter()
        .position(|&n| n == "ESC_CALIBRATION")
        .expect("ESC_CALIBRATION");
    assert_eq!(names[esc + 1], "FRAME_TYPE");
    let frame = find_log_gobject_var("FRAME_TYPE").expect("FRAME_TYPE");
    let esc_cal = find_log_gobject_var("ESC_CALIBRATION").expect("ESC_CALIBRATION");
    assert!(frame.key < esc_cal.key);
}

#[test]
fn first_gobject_is_arming_after_frame_type() {
    let names: Vec<_> = LOG_GOBJECT_VAR_INFO.iter().map(|e| e.name).collect();
    let frame = names
        .iter()
        .position(|&n| n == "FRAME_TYPE")
        .expect("FRAME_TYPE");
    assert_eq!(names[frame + 1], "ARMING_");
    let arming = find_log_gobject_var("ARMING_").expect("ARMING_");
    assert_eq!(arming.ptype, VarType::Group);
    assert_eq!(arming.default.to_bits(), 0.0f32.to_bits());
}

#[test]
fn copter_log_default_is_not_plane_all_bits() {
    assert_eq!(MASK_LOG_ANY, 0xFFFF);
    assert_ne!(DEFAULT_LOG_BITMASK, MASK_LOG_ANY);
    assert_eq!(DEFAULT_LOG_BITMASK & MASK_LOG_ATTITUDE_FAST, 0);
    assert_eq!(DEFAULT_LOG_BITMASK & MASK_LOG_INAV, 0);
    assert_ne!(DEFAULT_LOG_BITMASK & MASK_LOG_MOTBATT, 0);
    assert_eq!(
        DEFAULT_LOG_BITMASK,
        MASK_LOG_ATTITUDE_MED
            | MASK_LOG_GPS
            | MASK_LOG_PM
            | MASK_LOG_CTUN
            | MASK_LOG_NTUN
            | MASK_LOG_RCIN
            | MASK_LOG_IMU
            | MASK_LOG_CMD
            | MASK_LOG_CURRENT
            | MASK_LOG_RCOUT
            | MASK_LOG_OPTFLOW
            | MASK_LOG_PID
            | MASK_LOG_COMPASS
            | MASK_LOG_CAMERA
            | MASK_LOG_MOTBATT
    );
    assert_eq!(DEFAULT_LOG_BITMASK, 180_222);
}

#[test]
fn stock_defaults_are_the_gscalar_values() {
    let bits = |v: f32| v.to_bits();
    let entry = |name| find_log_gobject_var(name).unwrap_or_else(|| panic!("missing {name}"));
    assert_eq!(
        entry("LOG_BITMASK").default.to_bits(),
        bits(DEFAULT_LOG_BITMASK as f32)
    );
    assert_eq!(entry("ESC_CALIBRATION").default.to_bits(), bits(0.0));
    assert_eq!(
        entry("FRAME_TYPE").default.to_bits(),
        bits(HAL_FRAME_TYPE_DEFAULT as f32)
    );
    assert_eq!(HAL_FRAME_TYPE_DEFAULT, MOTOR_FRAME_TYPE_X);
    assert_ne!(HAL_FRAME_TYPE_DEFAULT, MOTOR_FRAME_TYPE_PLUS);
}

#[test]
fn types_follow_the_member_wrappers() {
    assert_eq!(
        find_log_gobject_var("LOG_BITMASK").expect("log").ptype,
        VarType::Int32
    );
    assert_eq!(
        find_log_gobject_var("ESC_CALIBRATION").expect("esc").ptype,
        VarType::Int8
    );
    assert_eq!(
        find_log_gobject_var("FRAME_TYPE").expect("frame").ptype,
        VarType::Int8
    );
    assert_eq!(
        find_log_gobject_var("ARMING_").expect("arming").ptype,
        VarType::Group
    );
}

#[test]
fn first_leftover_still_stops_before_this_slice() {
    assert!(find_first_var("LOG_BITMASK").is_none());
    assert!(find_first_var("ESC_CALIBRATION").is_none());
    assert!(find_first_var("FRAME_TYPE").is_none());
    assert!(find_first_var("ARMING_").is_none());
}

#[test]
fn later_rows_are_not_in_this_slice() {
    assert!(find_log_gobject_var("TUNE").is_none());
    assert!(find_log_gobject_var("DISARM_DELAY").is_none());
    assert!(find_log_gobject_var("CAM").is_none());
    assert!(find_log_gobject_var("COMPASS_").is_none());
}

#[test]
fn ap_param_finds_the_gscalars_and_not_the_empty_group() {
    let mut table = [ap_param::info::ParamInfo {
        name: "",
        key: 0,
        ptype: 0,
        flags: 0,
        group: None,
    }; 4];
    let mut n = 0_usize;
    for_each_log_gobject_param_info(&mut |info| {
        table[n] = info;
        n += 1;
    });
    assert_eq!(n, 4);

    let filter = EnumFilter::for_frame(0);
    let log = find_by_name(&table, filter, "LOG_BITMASK").expect("LOG_BITMASK");
    assert_eq!(log.key, K_PARAM_LOG_BITMASK);
    assert_eq!(log.ptype, VarType::Int32.as_u8());

    let frame = find_by_name(&table, filter, "FRAME_TYPE").expect("FRAME_TYPE");
    assert_eq!(frame.key, K_PARAM_FRAME_TYPE);

    // Nested `AP_Arming_Copter::var_info` is not this leftover, so the
    // group contributes no children and `ARMING_` itself is not a value.
    assert!(find_by_name(&table, filter, "ARMING_").is_none());
    assert!(find_by_name(&table, filter, "DISARM_DELAY").is_none());
}
