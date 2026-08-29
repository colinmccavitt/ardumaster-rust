//! Stock `COMPASS_` leftover, upstream `ArduCopter/Parameters.cpp`.
//!
//! The next Multi `GOBJECT` after `LGR_`. Heli `IM_` sits between
//! them on a tradheli build and is not a row of this leftover.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::parameters::{
    compass_gobject_var_info_entry, find_compass_gobject_var, find_disarm_gobject_var,
    find_log_gobject_var, find_relay_gobject_var, find_tune_var,
    for_each_compass_gobject_param_info, COMPASS_GOBJECT_VAR_INFO, DISARM_GOBJECT_VAR_INFO,
    FIRST_VAR_INFO, K_PARAM_COMPASS, K_PARAM_COMPASS_ENABLED_DEPRECATED, K_PARAM_INPUT_MANAGER,
    K_PARAM_INS, K_PARAM_LANDINGGEAR, LOG_GOBJECT_VAR_INFO, RELAY_GOBJECT_VAR_INFO, TUNE_VAR_INFO,
};
use ap_param::info::{find_by_name, EnumFilter};
use ap_param::VarType;

#[test]
fn table_starts_with_compass() {
    let first = compass_gobject_var_info_entry().expect("COMPASS_");
    assert_eq!(first.name, "COMPASS_");
    assert_eq!(first.key, K_PARAM_COMPASS);
    assert_eq!(first.key, 147);
    assert_eq!(first.ptype, VarType::Group);
    assert_eq!(first.default.to_bits(), 0.0f32.to_bits());
}

#[test]
fn slice_is_one_stock_gobject() {
    assert_eq!(COMPASS_GOBJECT_VAR_INFO.len(), 1);
    let names: Vec<_> = COMPASS_GOBJECT_VAR_INFO.iter().map(|e| e.name).collect();
    assert_eq!(names, ["COMPASS_"]);
    for entry in COMPASS_GOBJECT_VAR_INFO {
        assert_eq!(entry.ptype, VarType::Group);
        assert_eq!(entry.default.to_bits(), 0.0f32.to_bits());
    }
}

#[test]
fn keys_match_the_parameters_enum() {
    let entry = find_compass_gobject_var("COMPASS_").expect("COMPASS_");
    assert_eq!(entry.key, K_PARAM_COMPASS);
    assert_eq!(entry.key, 147);
}

#[test]
fn compass_is_not_the_deprecated_enabled_key() {
    assert_eq!(K_PARAM_COMPASS_ENABLED_DEPRECATED, 146);
    assert_ne!(K_PARAM_COMPASS, K_PARAM_COMPASS_ENABLED_DEPRECATED);
    let entry = find_compass_gobject_var("COMPASS_").expect("COMPASS_");
    assert_ne!(entry.key, K_PARAM_COMPASS_ENABLED_DEPRECATED);
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
    {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(keys.insert(entry.key), "duplicate key {}", entry.key);
    }
}

#[test]
fn sits_after_lgr_in_table_order() {
    let lgr = find_relay_gobject_var("LGR_").expect("LGR_");
    let compass = find_compass_gobject_var("COMPASS_").expect("COMPASS_");
    assert_eq!(lgr.key, K_PARAM_LANDINGGEAR);
    assert_eq!(lgr.ptype, VarType::Group);
    assert_eq!(compass.ptype, VarType::Group);
    // `LGR_` is 18; `COMPASS_` is a later enum slot that still sits
    // after `LGR_` on the compiled Multi table (heli `IM_` is skipped).
    assert!(compass.key > lgr.key);
}

#[test]
fn heli_im_is_not_this_leftover() {
    assert_eq!(K_PARAM_INPUT_MANAGER, 19);
    assert_eq!(K_PARAM_LANDINGGEAR + 1, K_PARAM_INPUT_MANAGER);
    assert!(find_compass_gobject_var("IM_").is_none());
}

#[test]
fn earlier_leftovers_do_not_include_this_slice() {
    assert!(find_relay_gobject_var("COMPASS_").is_none());
    assert!(find_disarm_gobject_var("COMPASS_").is_none());
    assert!(find_log_gobject_var("COMPASS_").is_none());
    assert!(find_tune_var("COMPASS_").is_none());
}

#[test]
fn later_rows_are_not_in_this_slice() {
    assert!(find_compass_gobject_var("LGR_").is_none());
    assert!(find_compass_gobject_var("IM_").is_none());
    assert!(find_compass_gobject_var("INS").is_none());
    assert!(find_compass_gobject_var("WP_").is_none());
    assert!(find_compass_gobject_var("LOIT_").is_none());
    assert!(find_compass_gobject_var("CIRCLE_").is_none());
    assert!(find_compass_gobject_var("TUNE_MIN").is_none());
}

#[test]
fn ins_key_is_not_this_leftover() {
    assert_eq!(K_PARAM_INS, 3);
    let entry = find_compass_gobject_var("COMPASS_").expect("COMPASS_");
    assert_ne!(entry.key, K_PARAM_INS);
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
    for_each_compass_gobject_param_info(&mut |info| {
        table[n] = info;
        n += 1;
    });
    assert_eq!(n, 1);

    let filter = EnumFilter::for_frame(0);
    // Nested `AP_Compass` `var_info` is not this leftover, so the group
    // contributes no children.
    assert!(find_by_name(&table, filter, "COMPASS_").is_none());
    assert!(find_by_name(&table, filter, "INS").is_none());
}
