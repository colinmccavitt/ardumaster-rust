//! `RELAY` through `LGR_` leftover, upstream `ArduCopter/Parameters.cpp`.
//!
//! The next contiguous `GOBJECT` group after `CAM`. Stock multicopter
//! compiles all three (`AP_RELAY_ENABLED`, `HAL_PARACHUTE_ENABLED`,
//! `AP_LANDINGGEAR_ENABLED`). Heli `IM_` stays later.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::parameters::{
    find_disarm_gobject_var, find_log_gobject_var, find_relay_gobject_var, find_tune_var,
    for_each_relay_gobject_param_info, relay_gobject_var_info_entry, DISARM_GOBJECT_VAR_INFO,
    FIRST_VAR_INFO, K_PARAM_CAMERA, K_PARAM_EPM_UNUSED, K_PARAM_INPUT_MANAGER, K_PARAM_LANDINGGEAR,
    K_PARAM_PARACHUTE, K_PARAM_RELAY, LOG_GOBJECT_VAR_INFO, RELAY_GOBJECT_VAR_INFO, TUNE_VAR_INFO,
};
use ap_param::info::{find_by_name, EnumFilter};
use ap_param::VarType;

#[test]
fn table_starts_with_relay() {
    let first = relay_gobject_var_info_entry().expect("RELAY");
    assert_eq!(first.name, "RELAY");
    assert_eq!(first.key, K_PARAM_RELAY);
    assert_eq!(first.key, 13);
    assert_eq!(first.ptype, VarType::Group);
    assert_eq!(first.default.to_bits(), 0.0f32.to_bits());
}

#[test]
fn slice_is_three_contiguous_gobjects() {
    assert_eq!(RELAY_GOBJECT_VAR_INFO.len(), 3);
    let names: Vec<_> = RELAY_GOBJECT_VAR_INFO.iter().map(|e| e.name).collect();
    assert_eq!(names, ["RELAY", "CHUTE_", "LGR_"]);
    for entry in RELAY_GOBJECT_VAR_INFO {
        assert_eq!(entry.ptype, VarType::Group);
        assert_eq!(entry.default.to_bits(), 0.0f32.to_bits());
    }
}

#[test]
fn keys_match_the_parameters_enum() {
    let want = [
        ("RELAY", K_PARAM_RELAY, 13_u16),
        ("CHUTE_", K_PARAM_PARACHUTE, 17),
        ("LGR_", K_PARAM_LANDINGGEAR, 18),
    ];
    for (name, key, raw) in want {
        let entry = find_relay_gobject_var(name).unwrap_or_else(|| panic!("missing {name}"));
        assert_eq!(entry.key, key, "{name}");
        assert_eq!(entry.key, raw, "{name}");
    }
}

#[test]
fn relay_is_not_the_unused_epm_key() {
    assert_eq!(K_PARAM_EPM_UNUSED, 14);
    assert_ne!(K_PARAM_RELAY, K_PARAM_EPM_UNUSED);
    let entry = find_relay_gobject_var("RELAY").expect("RELAY");
    assert_ne!(entry.key, K_PARAM_EPM_UNUSED);
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
    {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(keys.insert(entry.key), "duplicate key {}", entry.key);
    }
}

#[test]
fn sits_after_cam_in_table_order() {
    let cam = find_disarm_gobject_var("CAM").expect("CAM");
    let relay = find_relay_gobject_var("RELAY").expect("RELAY");
    assert_eq!(cam.key, K_PARAM_CAMERA);
    assert_eq!(cam.ptype, VarType::Group);
    assert_eq!(relay.ptype, VarType::Group);
    // `CAM` is 165; `RELAY` is an earlier enum slot that still sits
    // after `CAM` on the compiled table.
    assert!(relay.key < cam.key);
}

#[test]
fn chute_follows_relay_and_lgr_follows_chute() {
    let names: Vec<_> = RELAY_GOBJECT_VAR_INFO.iter().map(|e| e.name).collect();
    let relay = names.iter().position(|&n| n == "RELAY").expect("RELAY");
    assert_eq!(names[relay + 1], "CHUTE_");
    assert_eq!(names[relay + 2], "LGR_");
    let chute = find_relay_gobject_var("CHUTE_").expect("CHUTE_");
    let lgr = find_relay_gobject_var("LGR_").expect("LGR_");
    assert_eq!(chute.key + 1, lgr.key);
}

#[test]
fn heli_im_is_not_this_leftover() {
    assert_eq!(K_PARAM_INPUT_MANAGER, 19);
    assert_eq!(K_PARAM_LANDINGGEAR + 1, K_PARAM_INPUT_MANAGER);
    assert!(find_relay_gobject_var("IM_").is_none());
}

#[test]
fn earlier_leftovers_do_not_include_this_slice() {
    assert!(find_disarm_gobject_var("RELAY").is_none());
    assert!(find_disarm_gobject_var("CHUTE_").is_none());
    assert!(find_disarm_gobject_var("LGR_").is_none());
    assert!(find_log_gobject_var("RELAY").is_none());
    assert!(find_tune_var("RELAY").is_none());
}

#[test]
fn later_rows_are_not_in_this_slice() {
    assert!(find_relay_gobject_var("CAM").is_none());
    assert!(find_relay_gobject_var("IM_").is_none());
    assert!(find_relay_gobject_var("COMPASS_").is_none());
    assert!(find_relay_gobject_var("INS").is_none());
    assert!(find_relay_gobject_var("TUNE_MIN").is_none());
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
    for_each_relay_gobject_param_info(&mut |info| {
        table[n] = info;
        n += 1;
    });
    assert_eq!(n, 3);

    let filter = EnumFilter::for_frame(0);
    // Nested `AP_Relay` / `AP_Parachute` / `AP_LandingGear` `var_info`
    // is not this leftover, so the groups contribute no children.
    assert!(find_by_name(&table, filter, "RELAY").is_none());
    assert!(find_by_name(&table, filter, "CHUTE_").is_none());
    assert!(find_by_name(&table, filter, "LGR_").is_none());
    assert!(find_by_name(&table, filter, "COMPASS_").is_none());
}
