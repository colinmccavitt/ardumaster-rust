//! `DISARM_DELAY` through next `GOBJECT` leftover, upstream `ArduCopter/Parameters.cpp`.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::parameters::{
    disarm_gobject_var_info_entry, find_disarm_gobject_var, find_log_gobject_var, find_tune_var,
    for_each_disarm_gobject_param_info, ACRO_BALANCE_PITCH, ACRO_BALANCE_ROLL,
    ACRO_TRAINER_LEVELING, ACRO_TRAINER_LIMITED, ACRO_TRAINER_OFF, AUTO_DISARMING_DELAY,
    DISARM_GOBJECT_VAR_INFO, FIRST_VAR_INFO, FS_EKF_ACTION_ALTHOLD, FS_EKF_ACTION_DEFAULT,
    FS_EKF_ACTION_LAND, FS_EKF_ACTION_LAND_EVEN_STABILIZE, FS_EKF_ACTION_REPORT_ONLY,
    FS_EKF_THRESHOLD_DEFAULT, K_PARAM_ACRO_BALANCE_PITCH, K_PARAM_ACRO_BALANCE_ROLL,
    K_PARAM_ACRO_TRAINER, K_PARAM_CAMERA, K_PARAM_DISARM_DELAY, K_PARAM_FS_CRASH_CHECK,
    K_PARAM_FS_EKF_ACTION, K_PARAM_FS_EKF_THRESH, K_PARAM_LAND_REPOSITIONING,
    K_PARAM_POSHOLD_BRAKE_RATE_DEGS, K_PARAM_RC_SPEED, LAND_REPOSITION_DEFAULT,
    LOG_GOBJECT_VAR_INFO, POSHOLD_BRAKE_RATE_DEFAULT, POSHOLD_BRAKE_RATE_HELI, RC_FAST_SPEED,
    RC_FAST_SPEED_HELI, TUNE_VAR_INFO,
};
use ap_param::info::{find_by_name, EnumFilter};
use ap_param::VarType;

#[test]
fn table_starts_with_disarm_delay() {
    let first = disarm_gobject_var_info_entry().expect("DISARM_DELAY");
    assert_eq!(first.name, "DISARM_DELAY");
    assert_eq!(first.key, K_PARAM_DISARM_DELAY);
    assert_eq!(first.key, 91);
    assert_eq!(first.ptype, VarType::Int8);
    assert_eq!(
        first.default.to_bits(),
        (AUTO_DISARMING_DELAY as f32).to_bits()
    );
}

#[test]
fn slice_is_ten_gscalars_and_the_next_gobject() {
    assert_eq!(DISARM_GOBJECT_VAR_INFO.len(), 11);
    let names: Vec<_> = DISARM_GOBJECT_VAR_INFO.iter().map(|e| e.name).collect();
    assert_eq!(
        names,
        [
            "DISARM_DELAY",
            "PHLD_BRK_RATE",
            "LAND_REPOSITION",
            "FS_EKF_ACTION",
            "FS_EKF_THRESH",
            "FS_CRASH_CHECK",
            "RC_SPEED",
            "ACRO_BAL_ROLL",
            "ACRO_BAL_PITCH",
            "ACRO_TRAINER",
            "CAM",
        ]
    );
    assert_eq!(DISARM_GOBJECT_VAR_INFO[10].ptype, VarType::Group);
}

#[test]
fn keys_match_the_parameters_enum() {
    let want = [
        ("DISARM_DELAY", K_PARAM_DISARM_DELAY, 91_u16),
        ("PHLD_BRK_RATE", K_PARAM_POSHOLD_BRAKE_RATE_DEGS, 46),
        ("LAND_REPOSITION", K_PARAM_LAND_REPOSITIONING, 52),
        ("FS_EKF_ACTION", K_PARAM_FS_EKF_ACTION, 248),
        ("FS_EKF_THRESH", K_PARAM_FS_EKF_THRESH, 54),
        ("FS_CRASH_CHECK", K_PARAM_FS_CRASH_CHECK, 92),
        ("RC_SPEED", K_PARAM_RC_SPEED, 192),
        ("ACRO_BAL_ROLL", K_PARAM_ACRO_BALANCE_ROLL, 242),
        ("ACRO_BAL_PITCH", K_PARAM_ACRO_BALANCE_PITCH, 243),
        ("ACRO_TRAINER", K_PARAM_ACRO_TRAINER, 27),
        ("CAM", K_PARAM_CAMERA, 165),
    ];
    for (name, key, raw) in want {
        let entry = find_disarm_gobject_var(name).unwrap_or_else(|| panic!("missing {name}"));
        assert_eq!(entry.key, key, "{name}");
        assert_eq!(entry.key, raw, "{name}");
    }
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
    {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(keys.insert(entry.key), "duplicate key {}", entry.key);
    }
}

#[test]
fn acro_trainer_follows_balance_but_has_the_earlier_key() {
    let names: Vec<_> = DISARM_GOBJECT_VAR_INFO.iter().map(|e| e.name).collect();
    let pitch = names
        .iter()
        .position(|&n| n == "ACRO_BAL_PITCH")
        .expect("ACRO_BAL_PITCH");
    assert_eq!(names[pitch + 1], "ACRO_TRAINER");
    let trainer = find_disarm_gobject_var("ACRO_TRAINER").expect("ACRO_TRAINER");
    let roll = find_disarm_gobject_var("ACRO_BAL_ROLL").expect("ACRO_BAL_ROLL");
    assert!(trainer.key < roll.key);
}

#[test]
fn next_gobject_is_cam_after_acro_trainer() {
    let names: Vec<_> = DISARM_GOBJECT_VAR_INFO.iter().map(|e| e.name).collect();
    let trainer = names
        .iter()
        .position(|&n| n == "ACRO_TRAINER")
        .expect("ACRO_TRAINER");
    assert_eq!(names[trainer + 1], "CAM");
    let cam = find_disarm_gobject_var("CAM").expect("CAM");
    assert_eq!(cam.ptype, VarType::Group);
    assert_eq!(cam.default.to_bits(), 0.0f32.to_bits());
}

#[test]
fn stock_defaults_are_multicopter_not_heli() {
    let bits = |v: f32| v.to_bits();
    let entry = |name| find_disarm_gobject_var(name).unwrap_or_else(|| panic!("missing {name}"));
    assert_eq!(AUTO_DISARMING_DELAY, 10);
    assert_eq!(POSHOLD_BRAKE_RATE_DEFAULT, 8);
    assert_eq!(POSHOLD_BRAKE_RATE_HELI, 4);
    assert_ne!(POSHOLD_BRAKE_RATE_DEFAULT, POSHOLD_BRAKE_RATE_HELI);
    assert_eq!(RC_FAST_SPEED, 490);
    assert_eq!(RC_FAST_SPEED_HELI, 125);
    assert_ne!(RC_FAST_SPEED, RC_FAST_SPEED_HELI);
    assert_eq!(
        entry("DISARM_DELAY").default.to_bits(),
        bits(AUTO_DISARMING_DELAY as f32)
    );
    assert_eq!(
        entry("PHLD_BRK_RATE").default.to_bits(),
        bits(POSHOLD_BRAKE_RATE_DEFAULT as f32)
    );
    assert_eq!(
        entry("LAND_REPOSITION").default.to_bits(),
        bits(LAND_REPOSITION_DEFAULT as f32)
    );
    assert_eq!(
        entry("FS_EKF_ACTION").default.to_bits(),
        bits(FS_EKF_ACTION_DEFAULT as f32)
    );
    assert_eq!(
        entry("FS_EKF_THRESH").default.to_bits(),
        bits(FS_EKF_THRESHOLD_DEFAULT)
    );
    assert_eq!(entry("FS_CRASH_CHECK").default.to_bits(), bits(1.0));
    assert_eq!(
        entry("RC_SPEED").default.to_bits(),
        bits(RC_FAST_SPEED as f32)
    );
    assert_eq!(
        entry("ACRO_BAL_ROLL").default.to_bits(),
        bits(ACRO_BALANCE_ROLL)
    );
    assert_eq!(
        entry("ACRO_BAL_PITCH").default.to_bits(),
        bits(ACRO_BALANCE_PITCH)
    );
    assert_eq!(
        entry("ACRO_TRAINER").default.to_bits(),
        bits(ACRO_TRAINER_LIMITED as f32)
    );
}

#[test]
fn ekf_action_default_is_land_not_report_only() {
    assert_eq!(FS_EKF_ACTION_REPORT_ONLY, 0);
    assert_eq!(FS_EKF_ACTION_LAND, 1);
    assert_eq!(FS_EKF_ACTION_ALTHOLD, 2);
    assert_eq!(FS_EKF_ACTION_LAND_EVEN_STABILIZE, 3);
    assert_eq!(FS_EKF_ACTION_DEFAULT, FS_EKF_ACTION_LAND);
    assert_ne!(FS_EKF_ACTION_DEFAULT, FS_EKF_ACTION_REPORT_ONLY);
}

#[test]
fn acro_trainer_default_is_limited() {
    assert_eq!(ACRO_TRAINER_OFF, 0);
    assert_eq!(ACRO_TRAINER_LEVELING, 1);
    assert_eq!(ACRO_TRAINER_LIMITED, 2);
}

#[test]
fn types_follow_the_member_wrappers() {
    assert_eq!(
        find_disarm_gobject_var("DISARM_DELAY")
            .expect("disarm")
            .ptype,
        VarType::Int8
    );
    assert_eq!(
        find_disarm_gobject_var("PHLD_BRK_RATE")
            .expect("phld")
            .ptype,
        VarType::Int16
    );
    assert_eq!(
        find_disarm_gobject_var("FS_EKF_THRESH").expect("ekf").ptype,
        VarType::Float
    );
    assert_eq!(
        find_disarm_gobject_var("RC_SPEED").expect("rc").ptype,
        VarType::Int16
    );
    assert_eq!(
        find_disarm_gobject_var("ACRO_BAL_ROLL")
            .expect("roll")
            .ptype,
        VarType::Float
    );
    assert_eq!(
        find_disarm_gobject_var("ACRO_TRAINER")
            .expect("trainer")
            .ptype,
        VarType::Int8
    );
    assert_eq!(
        find_disarm_gobject_var("CAM").expect("cam").ptype,
        VarType::Group
    );
}

#[test]
fn earlier_leftovers_do_not_include_this_slice() {
    assert!(find_log_gobject_var("DISARM_DELAY").is_none());
    assert!(find_log_gobject_var("CAM").is_none());
    assert!(find_tune_var("DISARM_DELAY").is_none());
    assert!(find_tune_var("CAM").is_none());
}

#[test]
fn later_rows_are_not_in_this_slice() {
    assert!(find_disarm_gobject_var("TUNE").is_none());
    assert!(find_disarm_gobject_var("RELAY").is_none());
    assert!(find_disarm_gobject_var("CHUTE_").is_none());
    assert!(find_disarm_gobject_var("COMPASS_").is_none());
    assert!(find_disarm_gobject_var("TUNE_MIN").is_none());
}

#[test]
fn ap_param_finds_the_gscalars_and_not_the_empty_group() {
    let mut table = [ap_param::info::ParamInfo {
        name: "",
        key: 0,
        ptype: 0,
        flags: 0,
        group: None,
    }; 11];
    let mut n = 0_usize;
    for_each_disarm_gobject_param_info(&mut |info| {
        table[n] = info;
        n += 1;
    });
    assert_eq!(n, 11);

    let filter = EnumFilter::for_frame(0);
    let disarm = find_by_name(&table, filter, "DISARM_DELAY").expect("DISARM_DELAY");
    assert_eq!(disarm.key, K_PARAM_DISARM_DELAY);
    assert_eq!(disarm.ptype, VarType::Int8.as_u8());

    let thresh = find_by_name(&table, filter, "FS_EKF_THRESH").expect("FS_EKF_THRESH");
    assert_eq!(thresh.key, K_PARAM_FS_EKF_THRESH);
    assert_eq!(thresh.ptype, VarType::Float.as_u8());

    // Nested `AP_Camera::var_info` is not this leftover, so the group
    // contributes no children and `CAM` itself is not a value.
    assert!(find_by_name(&table, filter, "CAM").is_none());
    assert!(find_by_name(&table, filter, "RELAY").is_none());
}
