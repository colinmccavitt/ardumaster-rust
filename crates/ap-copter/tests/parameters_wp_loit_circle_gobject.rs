//! Stock `WP_` / `LOIT_` / `CIRCLE_` leftover, upstream `ArduCopter/Parameters.cpp`.
//!
//! The next Multi `GOBJECTPTR` group after `INS`. `CIRCLE_` is
//! `MODE_CIRCLE_ENABLED`. Nested `AC_WPNav` / `AC_Loiter` / `AC_Circle`
//! `var_info` is not this leftover. `ATC_` stays later.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::parameters::{
    find_compass_gobject_var, find_disarm_gobject_var, find_ins_gobject_var, find_log_gobject_var,
    find_relay_gobject_var, find_tune_var, find_wp_loit_circle_gobject_var,
    for_each_wp_loit_circle_gobject_param_info, wp_loit_circle_gobject_var_info_entry,
    COMPASS_GOBJECT_VAR_INFO, DISARM_GOBJECT_VAR_INFO, FIRST_VAR_INFO, INS_GOBJECT_VAR_INFO,
    K_PARAM_ATTITUDE_CONTROL, K_PARAM_CIRCLE_NAV, K_PARAM_INERTIAL_NAV, K_PARAM_INS,
    K_PARAM_LOITER_NAV, K_PARAM_WP_NAV, LOG_GOBJECT_VAR_INFO, RELAY_GOBJECT_VAR_INFO,
    TUNE_VAR_INFO, WP_LOIT_CIRCLE_GOBJECT_VAR_INFO,
};
use ap_param::info::{find_by_name, EnumFilter};
use ap_param::VarType;

#[test]
fn table_starts_with_wp() {
    let first = wp_loit_circle_gobject_var_info_entry().expect("WP_");
    assert_eq!(first.name, "WP_");
    assert_eq!(first.key, K_PARAM_WP_NAV);
    assert_eq!(first.key, 101);
    assert_eq!(first.ptype, VarType::Group);
    assert_eq!(first.default.to_bits(), 0.0f32.to_bits());
}

#[test]
fn slice_is_three_contiguous_gobjectptrs() {
    assert_eq!(WP_LOIT_CIRCLE_GOBJECT_VAR_INFO.len(), 3);
    let names: Vec<_> = WP_LOIT_CIRCLE_GOBJECT_VAR_INFO
        .iter()
        .map(|e| e.name)
        .collect();
    assert_eq!(names, ["WP_", "LOIT_", "CIRCLE_"]);
    for entry in WP_LOIT_CIRCLE_GOBJECT_VAR_INFO {
        assert_eq!(entry.ptype, VarType::Group);
        assert_eq!(entry.default.to_bits(), 0.0f32.to_bits());
    }
}

#[test]
fn keys_match_the_parameters_enum() {
    let want = [
        ("WP_", K_PARAM_WP_NAV, 101_u16),
        ("LOIT_", K_PARAM_LOITER_NAV, 105),
        ("CIRCLE_", K_PARAM_CIRCLE_NAV, 104),
    ];
    for (name, key, raw) in want {
        let entry =
            find_wp_loit_circle_gobject_var(name).unwrap_or_else(|| panic!("missing {name}"));
        assert_eq!(entry.key, key, "{name}");
        assert_eq!(entry.key, raw, "{name}");
    }
}

#[test]
fn wp_is_not_the_deprecated_inertial_nav_key() {
    assert_eq!(K_PARAM_INERTIAL_NAV, 100);
    assert_ne!(K_PARAM_WP_NAV, K_PARAM_INERTIAL_NAV);
    let entry = find_wp_loit_circle_gobject_var("WP_").expect("WP_");
    assert_ne!(entry.key, K_PARAM_INERTIAL_NAV);
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
    {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(keys.insert(entry.key), "duplicate key {}", entry.key);
    }
}

#[test]
fn sits_after_ins_in_table_order() {
    let ins = find_ins_gobject_var("INS").expect("INS");
    let wp = find_wp_loit_circle_gobject_var("WP_").expect("WP_");
    assert_eq!(ins.key, K_PARAM_INS);
    assert_eq!(ins.ptype, VarType::Group);
    assert_eq!(wp.ptype, VarType::Group);
    // `INS` is 3; `WP_` is a later enum slot that sits after `INS` on
    // the compiled Multi table.
    assert!(wp.key > ins.key);
}

#[test]
fn loit_follows_wp_and_circle_follows_loit() {
    let names: Vec<_> = WP_LOIT_CIRCLE_GOBJECT_VAR_INFO
        .iter()
        .map(|e| e.name)
        .collect();
    let wp = names.iter().position(|&n| n == "WP_").expect("WP_");
    assert_eq!(names[wp + 1], "LOIT_");
    assert_eq!(names[wp + 2], "CIRCLE_");
    let loit = find_wp_loit_circle_gobject_var("LOIT_").expect("LOIT_");
    let circle = find_wp_loit_circle_gobject_var("CIRCLE_").expect("CIRCLE_");
    // Keys are enum order, not table order: `LOIT_` is 105, `CIRCLE_` is 104.
    assert_eq!(loit.key, 105);
    assert_eq!(circle.key, 104);
    assert!(circle.key < loit.key);
}

#[test]
fn circle_is_mode_circle_enabled() {
    // Stock Multi compiles `MODE_CIRCLE_ENABLED`, so `CIRCLE_` is a row.
    assert!(find_wp_loit_circle_gobject_var("CIRCLE_").is_some());
    assert_eq!(K_PARAM_CIRCLE_NAV, 104);
}

#[test]
fn earlier_leftovers_do_not_include_this_slice() {
    assert!(find_ins_gobject_var("WP_").is_none());
    assert!(find_ins_gobject_var("LOIT_").is_none());
    assert!(find_ins_gobject_var("CIRCLE_").is_none());
    assert!(find_compass_gobject_var("WP_").is_none());
    assert!(find_relay_gobject_var("WP_").is_none());
    assert!(find_disarm_gobject_var("WP_").is_none());
    assert!(find_log_gobject_var("WP_").is_none());
    assert!(find_tune_var("WP_").is_none());
}

#[test]
fn later_rows_are_not_in_this_slice() {
    assert!(find_wp_loit_circle_gobject_var("INS").is_none());
    assert!(find_wp_loit_circle_gobject_var("IM_").is_none());
    assert!(find_wp_loit_circle_gobject_var("ATC_").is_none());
    assert!(find_wp_loit_circle_gobject_var("PSC").is_none());
    assert!(find_wp_loit_circle_gobject_var("TUNE_MIN").is_none());
}

#[test]
fn atc_key_is_not_this_leftover() {
    assert_eq!(K_PARAM_ATTITUDE_CONTROL, 102);
    let entry = find_wp_loit_circle_gobject_var("WP_").expect("WP_");
    assert_ne!(entry.key, K_PARAM_ATTITUDE_CONTROL);
    assert!(find_wp_loit_circle_gobject_var("ATC_").is_none());
}

#[test]
fn ap_param_does_not_find_the_empty_groups() {
    let mut table = [ap_param::info::ParamInfo {
        name: "",
        key: 0,
        ptype: 0,
        flags: 0,
        group: None,
    }; 3];
    let mut n = 0_usize;
    for_each_wp_loit_circle_gobject_param_info(&mut |info| {
        table[n] = info;
        n += 1;
    });
    assert_eq!(n, 3);

    let filter = EnumFilter::for_frame(0);
    // Nested `AC_WPNav` / `AC_Loiter` / `AC_Circle` `var_info` is not
    // this leftover, so the groups contribute no children.
    assert!(find_by_name(&table, filter, "WP_").is_none());
    assert!(find_by_name(&table, filter, "LOIT_").is_none());
    assert!(find_by_name(&table, filter, "CIRCLE_").is_none());
    assert!(find_by_name(&table, filter, "ATC_").is_none());
}
