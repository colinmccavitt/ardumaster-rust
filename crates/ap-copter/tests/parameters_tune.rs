//! `TUNE` leftover, upstream `ArduCopter/Parameters.cpp`.
//!
//! Compiled only when `AP_RC_TRANSMITTER_TUNING_ENABLED`. Sits between
//! `ESC_CALIBRATION` and `FRAME_TYPE` on the stock table, but is not a
//! row of the `LOG_BITMASK` leftover.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::parameters::{
    find_log_gobject_var, find_tune_var, for_each_tune_param_info, tune_var_info_entry,
    K_PARAM_ESC_CALIBRATE, K_PARAM_FRAME_TYPE, K_PARAM_RC_TUNING_PARAM,
    K_PARAM_RC_TUNING_PARAM_HIGH_OLD, K_PARAM_RC_TUNING_PARAM_LOW_OLD, TUNE_NONE, TUNE_VAR_INFO,
};
use ap_param::info::{find_by_name, EnumFilter};
use ap_param::VarType;

#[test]
fn table_is_the_tune_gscalar() {
    let first = tune_var_info_entry().expect("TUNE");
    assert_eq!(first.name, "TUNE");
    assert_eq!(first.key, K_PARAM_RC_TUNING_PARAM);
    assert_eq!(first.key, 187);
    assert_eq!(first.ptype, VarType::Int8);
    assert_eq!(first.default.to_bits(), 0.0f32.to_bits());
    assert_eq!(TUNE_VAR_INFO.len(), 1);
}

#[test]
fn tune_is_not_the_unused_old_keys() {
    assert_eq!(K_PARAM_RC_TUNING_PARAM_HIGH_OLD, 188);
    assert_eq!(K_PARAM_RC_TUNING_PARAM_LOW_OLD, 189);
    assert_ne!(K_PARAM_RC_TUNING_PARAM, K_PARAM_RC_TUNING_PARAM_HIGH_OLD);
    assert_ne!(K_PARAM_RC_TUNING_PARAM, K_PARAM_RC_TUNING_PARAM_LOW_OLD);
    let entry = find_tune_var("TUNE").expect("TUNE");
    assert_ne!(entry.key, K_PARAM_RC_TUNING_PARAM_HIGH_OLD);
    assert_ne!(entry.key, K_PARAM_RC_TUNING_PARAM_LOW_OLD);
}

#[test]
fn default_is_none_selected() {
    assert_eq!(TUNE_NONE, 0);
    let entry = find_tune_var("TUNE").expect("TUNE");
    assert_eq!(entry.default.to_bits(), (TUNE_NONE as f32).to_bits());
}

#[test]
fn sits_between_esc_and_frame_in_table_order() {
    // Stock table: ESC_CALIBRATION, TUNE, FRAME_TYPE. The LOG leftover
    // skips TUNE, so table-order neighbors live in that leftover.
    let esc = find_log_gobject_var("ESC_CALIBRATION").expect("ESC_CALIBRATION");
    let frame = find_log_gobject_var("FRAME_TYPE").expect("FRAME_TYPE");
    let tune = find_tune_var("TUNE").expect("TUNE");
    assert_eq!(esc.key, K_PARAM_ESC_CALIBRATE);
    assert_eq!(frame.key, K_PARAM_FRAME_TYPE);
    assert_eq!(esc.key, 186);
    assert_eq!(tune.key, esc.key + 1);
    // FRAME_TYPE is earlier in the enum than both.
    assert!(frame.key < esc.key);
    assert!(frame.key < tune.key);
}

#[test]
fn log_leftover_still_skips_tune() {
    assert!(find_log_gobject_var("TUNE").is_none());
    assert!(find_tune_var("ESC_CALIBRATION").is_none());
    assert!(find_tune_var("FRAME_TYPE").is_none());
    assert!(find_tune_var("DISARM_DELAY").is_none());
    assert!(find_tune_var("TUNE_MIN").is_none());
}

#[test]
fn names_and_keys_are_unique() {
    let mut names = std::collections::BTreeSet::new();
    let mut keys = std::collections::BTreeSet::new();
    for entry in TUNE_VAR_INFO {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(keys.insert(entry.key), "duplicate key {}", entry.key);
    }
}

#[test]
fn ap_param_finds_tune_by_name() {
    let mut table = [ap_param::info::ParamInfo {
        name: "",
        key: 0,
        ptype: 0,
        flags: 0,
        group: None,
    }; 1];
    let mut n = 0_usize;
    for_each_tune_param_info(&mut |info| {
        table[n] = info;
        n += 1;
    });
    assert_eq!(n, 1);

    let filter = EnumFilter::for_frame(0);
    let found = find_by_name(&table, filter, "TUNE").expect("TUNE");
    assert_eq!(found.key, K_PARAM_RC_TUNING_PARAM);
    assert_eq!(found.ptype, VarType::Int8.as_u8());
    assert!(find_by_name(&table, filter, "TUNE_MIN").is_none());
    assert!(find_by_name(&table, filter, "LOG_BITMASK").is_none());
}
