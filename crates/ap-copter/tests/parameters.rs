//! First `Copter::var_info` leftover, upstream `ArduCopter/Parameters.cpp`.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::parameters::{
    find_first_var, first_var_info_entry, for_each_first_param_info, CH_MODE_DEFAULT,
    FIRST_VAR_INFO, FLIGHT_MODE_STABILIZE, FS_GCS_DISABLED, FS_THR_ENABLED_ALWAYS_RTL,
    FS_THR_VALUE_DEFAULT, GPS_HDOP_GOOD_DEFAULT, K_FORMAT_VERSION, K_PARAM_FAILSAFE_GCS,
    K_PARAM_FAILSAFE_THROTTLE, K_PARAM_FAILSAFE_THROTTLE_VALUE, K_PARAM_FLIGHT_MODE1,
    K_PARAM_FLIGHT_MODE_CHAN, K_PARAM_FORMAT_VERSION, K_PARAM_GCS_PID_MASK, K_PARAM_GPS_HDOP_GOOD,
    K_PARAM_INITIAL_MODE, K_PARAM_RTL_ALT_TYPE, K_PARAM_RTL_CONE_SLOPE, K_PARAM_RTL_LOITER_TIME,
    K_PARAM_SIMPLE_MODES, K_PARAM_SUPER_SIMPLE, K_PARAM_THROTTLE_BEHAVIOR,
    K_PARAM_THROTTLE_DEADZONE, K_PARAM_THROTTLE_FILT, K_PARAM_WP_YAW_BEHAVIOR,
    RTL_CONE_SLOPE_DEFAULT, RTL_LOITER_TIME_MS, THR_DZ_DEFAULT, WP_YAW_BEHAVIOR_DEFAULT,
    WP_YAW_BEHAVIOR_LOOK_AHEAD, WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL,
};
use ap_copter::radio::FS_THR_VALUE_COPTER_DEFAULT;
use ap_param::info::{find_by_name, EnumFilter};
use ap_param::VarType;

#[test]
fn table_starts_with_format_version() {
    let first = first_var_info_entry().expect("first GSCALAR");
    assert_eq!(first.name, "FORMAT_VERSION");
    assert_eq!(first.key, K_PARAM_FORMAT_VERSION);
    assert_eq!(first.key, 0);
    assert_eq!(first.ptype, VarType::Int16);
    assert_eq!(first.default.to_bits(), 0.0f32.to_bits());
}

#[test]
fn format_version_default_is_not_the_layout_version() {
    assert_eq!(K_FORMAT_VERSION, 120);
    let first = first_var_info_entry().expect("first GSCALAR");
    assert_eq!(first.default.to_bits(), 0.0f32.to_bits());
    assert_ne!(K_FORMAT_VERSION, first.default as u16);
}

#[test]
fn first_slice_is_twenty_three_gscalars() {
    assert_eq!(FIRST_VAR_INFO.len(), 23);
}

#[test]
fn keys_match_the_parameters_enum() {
    let want = [
        ("FORMAT_VERSION", K_PARAM_FORMAT_VERSION),
        ("PILOT_THR_FILT", K_PARAM_THROTTLE_FILT),
        ("PILOT_THR_BHV", K_PARAM_THROTTLE_BEHAVIOR),
        ("GCS_PID_MASK", K_PARAM_GCS_PID_MASK),
        ("RTL_CONE_SLOPE", K_PARAM_RTL_CONE_SLOPE),
        ("RTL_LOIT_TIME", K_PARAM_RTL_LOITER_TIME),
        ("RTL_ALT_TYPE", K_PARAM_RTL_ALT_TYPE),
        ("FS_GCS_ENABLE", K_PARAM_FAILSAFE_GCS),
        ("GPS_HDOP_GOOD", K_PARAM_GPS_HDOP_GOOD),
        ("SUPER_SIMPLE", K_PARAM_SUPER_SIMPLE),
        ("WP_YAW_BEHAVIOR", K_PARAM_WP_YAW_BEHAVIOR),
        ("FS_THR_ENABLE", K_PARAM_FAILSAFE_THROTTLE),
        ("FS_THR_VALUE", K_PARAM_FAILSAFE_THROTTLE_VALUE),
        ("THR_DZ", K_PARAM_THROTTLE_DEADZONE),
        ("FLTMODE1", K_PARAM_FLIGHT_MODE1),
        ("FLTMODE_CH", K_PARAM_FLIGHT_MODE_CHAN),
        ("INITIAL_MODE", K_PARAM_INITIAL_MODE),
        ("SIMPLE", K_PARAM_SIMPLE_MODES),
    ];
    for (name, key) in want {
        let entry = find_first_var(name).unwrap_or_else(|| panic!("missing {name}"));
        assert_eq!(entry.key, key, "{name}");
    }
}

#[test]
fn names_and_keys_are_unique() {
    let mut names = std::collections::BTreeSet::new();
    let mut keys = std::collections::BTreeSet::new();
    for entry in FIRST_VAR_INFO {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(keys.insert(entry.key), "duplicate key {}", entry.key);
    }
}

#[test]
fn rtl_sits_between_gcs_pid_mask_and_fs_gcs() {
    let names: Vec<_> = FIRST_VAR_INFO.iter().map(|e| e.name).collect();
    let gcs_pid = names
        .iter()
        .position(|&n| n == "GCS_PID_MASK")
        .expect("GCS_PID_MASK");
    assert_eq!(names[gcs_pid + 1], "RTL_CONE_SLOPE");
    assert_eq!(names[gcs_pid + 2], "RTL_LOIT_TIME");
    assert_eq!(names[gcs_pid + 3], "RTL_ALT_TYPE");
    assert_eq!(names[gcs_pid + 4], "FS_GCS_ENABLE");
}

#[test]
fn simple_follows_initial_mode_but_has_the_earlier_key() {
    let names: Vec<_> = FIRST_VAR_INFO.iter().map(|e| e.name).collect();
    let initial = names
        .iter()
        .position(|&n| n == "INITIAL_MODE")
        .expect("INITIAL_MODE");
    assert_eq!(names[initial + 1], "SIMPLE");
    let simple = find_first_var("SIMPLE").expect("SIMPLE");
    let initial_mode = find_first_var("INITIAL_MODE").expect("INITIAL_MODE");
    assert_eq!(simple.key, K_PARAM_SIMPLE_MODES);
    assert_eq!(initial_mode.key, K_PARAM_INITIAL_MODE);
    assert!(simple.key < initial_mode.key);
}

#[test]
fn stock_defaults_are_the_gscalar_values() {
    let bits = |v: f32| v.to_bits();
    let entry = |name| find_first_var(name).unwrap_or_else(|| panic!("missing {name}"));
    assert_eq!(
        entry("RTL_CONE_SLOPE").default.to_bits(),
        bits(RTL_CONE_SLOPE_DEFAULT)
    );
    assert_eq!(
        entry("RTL_LOIT_TIME").default.to_bits(),
        bits(RTL_LOITER_TIME_MS as f32)
    );
    assert_eq!(
        entry("GPS_HDOP_GOOD").default.to_bits(),
        bits(GPS_HDOP_GOOD_DEFAULT as f32)
    );
    assert_eq!(
        entry("WP_YAW_BEHAVIOR").default.to_bits(),
        bits(WP_YAW_BEHAVIOR_DEFAULT as f32)
    );
    assert_eq!(
        entry("FS_GCS_ENABLE").default.to_bits(),
        bits(FS_GCS_DISABLED as f32)
    );
    assert_eq!(
        entry("FS_THR_ENABLE").default.to_bits(),
        bits(FS_THR_ENABLED_ALWAYS_RTL as f32)
    );
    assert_eq!(
        entry("FS_THR_VALUE").default.to_bits(),
        bits(FS_THR_VALUE_DEFAULT as f32)
    );
    assert_eq!(
        entry("THR_DZ").default.to_bits(),
        bits(THR_DZ_DEFAULT as f32)
    );
    assert_eq!(
        entry("FLTMODE_CH").default.to_bits(),
        bits(CH_MODE_DEFAULT as f32)
    );
}

#[test]
fn flight_modes_default_to_stabilize() {
    for name in [
        "FLTMODE1",
        "FLTMODE2",
        "FLTMODE3",
        "FLTMODE4",
        "FLTMODE5",
        "FLTMODE6",
        "INITIAL_MODE",
    ] {
        let entry = find_first_var(name).unwrap_or_else(|| panic!("missing {name}"));
        assert_eq!(entry.ptype, VarType::Int8, "{name}");
        assert_eq!(
            entry.default.to_bits(),
            (FLIGHT_MODE_STABILIZE as f32).to_bits(),
            "{name}"
        );
    }
}

#[test]
fn yaw_default_is_multicopter_not_heli() {
    assert_eq!(
        WP_YAW_BEHAVIOR_DEFAULT,
        WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL
    );
    assert_ne!(WP_YAW_BEHAVIOR_DEFAULT, WP_YAW_BEHAVIOR_LOOK_AHEAD);
}

#[test]
fn fs_thr_value_matches_the_radio_leftover() {
    assert_eq!(FS_THR_VALUE_DEFAULT, 975);
    assert_eq!(
        u16::try_from(FS_THR_VALUE_DEFAULT).expect("pwm"),
        FS_THR_VALUE_COPTER_DEFAULT
    );
}

#[test]
fn types_follow_the_member_wrappers() {
    assert_eq!(
        find_first_var("PILOT_THR_FILT").expect("filt").ptype,
        VarType::Float
    );
    assert_eq!(
        find_first_var("PILOT_THR_BHV").expect("bhv").ptype,
        VarType::Int16
    );
    assert_eq!(
        find_first_var("RTL_LOIT_TIME").expect("loit").ptype,
        VarType::Int32
    );
    assert_eq!(
        find_first_var("RTL_ALT_TYPE").expect("alt").ptype,
        VarType::Int8
    );
}

#[test]
fn later_rows_are_not_in_this_slice() {
    assert!(find_first_var("LOG_BITMASK").is_none());
    assert!(find_first_var("ESC_CALIBRATION").is_none());
    assert!(find_first_var("FRAME_TYPE").is_none());
}

#[test]
fn ap_param_finds_the_first_slice_by_name() {
    let mut table = [ap_param::info::ParamInfo {
        name: "",
        key: 0,
        ptype: 0,
        flags: 0,
        group: None,
    }; 23];
    let mut n = 0_usize;
    for_each_first_param_info(&mut |info| {
        table[n] = info;
        n += 1;
    });
    assert_eq!(n, 23);

    let filter = EnumFilter::for_frame(0);
    let found = find_by_name(&table, filter, "FORMAT_VERSION").expect("FORMAT_VERSION");
    assert_eq!(found.key, K_PARAM_FORMAT_VERSION);
    assert_eq!(found.ptype, VarType::Int16.as_u8());

    let filt = find_by_name(&table, filter, "PILOT_THR_FILT").expect("PILOT_THR_FILT");
    assert_eq!(filt.key, K_PARAM_THROTTLE_FILT);
    assert!(find_by_name(&table, filter, "LOG_BITMASK").is_none());
}
