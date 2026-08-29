//! `AC_AttitudeControl_Multi::var_info` leftover, upstream
//! `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp`.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_control::multi_var_info::{
    atc_table, find_multi_var, multi_var_info_entry, ACC_RP_MAX_DEFAULT, ACC_Y_MAX_DEFAULT,
    AC_PID_VAR_INFO, ANGLE_BOOST_DEFAULT, ANGLE_MAX_DEFAULT, ANGLE_P_DEFAULT, ANG_LIM_TC_DEFAULT,
    ATC_PARAM_INFO, ATC_PREFIX, ATTITUDE_CONTROL_VAR_INFO, INPUT_TC_DEFAULT,
    INPUT_TC_DEFAULT_PLANE, K_PARAM_ATTITUDE_CONTROL, LAND_MULT_DEFAULT, MULTI_SCALARS,
    MULTI_VAR_INFO, RATE_FF_ENAB_DEFAULT, RATE_IMAX, RATE_RPY_FILT_HZ, RATE_RP_D, RATE_RP_I,
    RATE_RP_P, RATE_WPY_MAX_DEFAULT, RATE_YAW_D, RATE_YAW_FILT_E_HZ, RATE_YAW_I, RATE_YAW_P,
    THR_G_BOOST_DEFAULT, THR_MIX_MAN_DEFAULT, THR_MIX_MAX_DEFAULT, THR_MIX_MIN_DEFAULT,
};
use ap_control::throttle_mix::{
    THR_MIX_MAX_DEFAULT as THROTTLE_THR_MIX_MAX, THR_MIX_MIN_DEFAULT as THROTTLE_THR_MIX_MIN,
};
use ap_param::info::{
    enumerate, find_by_name, group_id, EnumFilter, FLAG_DEFAULT_POINTER, FLAG_INFO_POINTER,
    FLAG_NESTED_OFFSET, FLAG_POINTER, GROUP_LEVEL_SHIFT, MAX_NAME_SIZE,
};
use ap_param::{ParamHeader, VarType};

fn table() -> [ap_param::info::ParamInfo<'static>; 1] {
    atc_table()
}

fn found(name: &str) -> ap_param::info::ParamRef {
    find_by_name(&table(), EnumFilter::for_frame(0), name)
        .unwrap_or_else(|| panic!("missing {name}"))
}

#[test]
fn table_starts_with_the_nested_parent() {
    let first = multi_var_info_entry().expect("nested parent");
    assert_eq!(first.name, "");
    assert_eq!(first.idx, 0);
    assert_eq!(first.ptype, VarType::Group.as_u8());
    assert_eq!(first.flags, 0);
    assert!(first.group.is_some());
}

#[test]
fn multi_is_eight_rows() {
    assert_eq!(MULTI_VAR_INFO.len(), 8);
}

#[test]
fn rows_match_upstream_order_and_idx() {
    let want = [
        ("", 0, VarType::Group, 0),
        ("RAT_RLL_", 1, VarType::Group, FLAG_NESTED_OFFSET),
        ("RAT_PIT_", 2, VarType::Group, FLAG_NESTED_OFFSET),
        ("RAT_YAW_", 3, VarType::Group, FLAG_NESTED_OFFSET),
        ("THR_MIX_MIN", 4, VarType::Float, 0),
        ("THR_MIX_MAX", 5, VarType::Float, 0),
        ("THR_MIX_MAN", 6, VarType::Float, 0),
        ("THR_G_BOOST", 7, VarType::Float, 0),
    ];
    assert_eq!(MULTI_VAR_INFO.len(), want.len());
    for (entry, (name, idx, ptype, flags)) in MULTI_VAR_INFO.iter().zip(want) {
        assert_eq!(entry.name, name, "{name}");
        assert_eq!(entry.idx, idx, "{name}");
        assert_eq!(entry.ptype, ptype.as_u8(), "{name}");
        assert_eq!(entry.flags, flags, "{name}");
    }
}

#[test]
fn nested_parent_is_not_a_subgroup_prefix() {
    let parent = find_multi_var("").expect("nested parent");
    assert_eq!(parent.flags, 0);
    assert_ne!(parent.flags, FLAG_NESTED_OFFSET);
    let roll = find_multi_var("RAT_RLL_").expect("RAT_RLL_");
    assert_eq!(roll.flags, FLAG_NESTED_OFFSET);
}

#[test]
fn names_and_indices_are_unique() {
    let mut names = std::collections::BTreeSet::new();
    let mut idxs = std::collections::BTreeSet::new();
    for entry in MULTI_VAR_INFO {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(idxs.insert(entry.idx), "duplicate idx {}", entry.idx);
    }
}

#[test]
fn later_rows_are_not_in_this_slice() {
    assert!(find_multi_var("RAT_YAW2_").is_none());
    assert!(find_multi_var("HELIRLL").is_none());
}

#[test]
fn mix_defaults_match_the_throttle_leftover() {
    assert_eq!(
        THR_MIX_MIN_DEFAULT.to_bits(),
        THROTTLE_THR_MIX_MIN.to_bits()
    );
    assert_eq!(
        THR_MIX_MAX_DEFAULT.to_bits(),
        THROTTLE_THR_MIX_MAX.to_bits()
    );
    assert_eq!(THR_MIX_MAN_DEFAULT.to_bits(), 0.1f32.to_bits());
    assert_eq!(THR_G_BOOST_DEFAULT.to_bits(), 0.0f32.to_bits());
}

#[test]
fn multi_scalars_are_the_four_mix_rows() {
    assert_eq!(MULTI_SCALARS.len(), 4);
    for spec in MULTI_SCALARS {
        let entry = find_multi_var(spec.name).unwrap_or_else(|| panic!("{}", spec.name));
        assert_eq!(entry.idx, spec.idx, "{}", spec.name);
        assert_eq!(entry.ptype, VarType::Float.as_u8(), "{}", spec.name);
    }
    assert_eq!(
        MULTI_SCALARS[0].default.to_bits(),
        THR_MIX_MIN_DEFAULT.to_bits()
    );
    assert_eq!(
        MULTI_SCALARS[1].default.to_bits(),
        THR_MIX_MAX_DEFAULT.to_bits()
    );
    assert_eq!(
        MULTI_SCALARS[2].default.to_bits(),
        THR_MIX_MAN_DEFAULT.to_bits()
    );
    assert_eq!(
        MULTI_SCALARS[3].default.to_bits(),
        THR_G_BOOST_DEFAULT.to_bits()
    );
}

#[test]
fn copter_input_tc_is_crisp_not_plane_medium() {
    assert_eq!(INPUT_TC_DEFAULT.to_bits(), 0.10f32.to_bits());
    assert_eq!(INPUT_TC_DEFAULT_PLANE.to_bits(), 0.15f32.to_bits());
    assert_ne!(INPUT_TC_DEFAULT.to_bits(), INPUT_TC_DEFAULT_PLANE.to_bits());
}

#[test]
fn parent_holes_stay_empty() {
    let used: std::collections::BTreeSet<u8> =
        ATTITUDE_CONTROL_VAR_INFO.iter().map(|e| e.idx).collect();
    for idx in [0u8, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11] {
        assert!(
            !used.contains(&idx),
            "historical hole {idx} must stay empty"
        );
    }
    assert!(used.contains(&5));
    assert!(used.contains(&12));
    assert!(used.contains(&28));
    assert_eq!(ATTITUDE_CONTROL_VAR_INFO.len(), 18);
}

#[test]
fn ac_pid_skips_historical_indices() {
    let used: std::collections::BTreeSet<u8> = AC_PID_VAR_INFO.iter().map(|e| e.idx).collect();
    for idx in [3u8, 6, 7, 8] {
        assert!(!used.contains(&idx), "AC_PID hole {idx} must stay empty");
    }
    assert_eq!(AC_PID_VAR_INFO.len(), 13);
    assert_eq!(AC_PID_VAR_INFO[0].name, "P");
    assert_eq!(AC_PID_VAR_INFO[0].idx, 0);
    assert_eq!(AC_PID_VAR_INFO[0].flags, FLAG_DEFAULT_POINTER);
    assert_eq!(
        AC_PID_VAR_INFO
            .iter()
            .find(|e| e.name == "PDMX")
            .expect("PDMX")
            .flags,
        0
    );
    assert_eq!(
        AC_PID_VAR_INFO
            .iter()
            .find(|e| e.name == "NTF")
            .expect("NTF")
            .ptype,
        VarType::Int8.as_u8()
    );
}

#[test]
fn gobject_key_and_flags_match_copter() {
    assert_eq!(K_PARAM_ATTITUDE_CONTROL, 102);
    assert_eq!(ATC_PARAM_INFO.key, 102);
    assert_eq!(ATC_PARAM_INFO.name, ATC_PREFIX);
    assert_eq!(ATC_PARAM_INFO.ptype, VarType::Group.as_u8());
    assert_eq!(ATC_PARAM_INFO.flags, FLAG_POINTER | FLAG_INFO_POINTER);
}

#[test]
fn enumerates_sixty_one_leaves() {
    let mut n = 0_usize;
    enumerate(&table(), EnumFilter::for_frame(0), &mut |_| {
        n += 1;
    });
    // 18 parent + 3*13 rate PID + 4 mix.
    assert_eq!(n, 61);
}

#[test]
fn names_are_unique_and_fit() {
    let mut names = std::collections::BTreeSet::new();
    enumerate(&table(), EnumFilter::for_frame(0), &mut |r| {
        let name = r.name.as_str();
        assert!(name.len() <= MAX_NAME_SIZE, "{name}");
        assert!(name.starts_with("ATC_"), "{name}");
        assert!(names.insert(name.to_string()), "duplicate {name}");
        assert!(r.behind_pointer, "{name} sits behind GOBJECTVARPTR");
        assert_eq!(r.key, K_PARAM_ATTITUDE_CONTROL, "{name}");
    });
    assert_eq!(names.len(), 61);
}

#[test]
fn find_by_name_resolves_mix_and_rate_and_parent() {
    let mix = found("ATC_THR_MIX_MIN");
    assert_eq!(mix.ptype, VarType::Float.as_u8());
    assert_eq!(mix.group_element, 4);

    let boost = found("ATC_THR_G_BOOST");
    assert_eq!(boost.group_element, 7);

    let ff = found("ATC_RATE_FF_ENAB");
    assert_eq!(ff.ptype, VarType::Int8.as_u8());
    assert_eq!(ff.group_element, 5 << GROUP_LEVEL_SHIFT);

    let ang = found("ATC_ANG_RLL_P");
    assert_eq!(ang.ptype, VarType::Float.as_u8());

    assert!(find_by_name(&table(), EnumFilter::for_frame(0), "ATC_").is_none());
    assert!(find_by_name(&table(), EnumFilter::for_frame(0), "THR_MIX_MIN").is_none());
}

#[test]
fn nested_parent_idx_zero_is_not_rewritten_at_shift_zero() {
    // Multi idx 0 at shift 0: group_id stays 0. Applying the 63 rewrite
    // here would move every parent parameter (ADR-0010).
    let base = group_id(0, 0, 0, 0);
    assert_eq!(base, 0);
    assert_ne!(base, 63);
    let ff = found("ATC_RATE_FF_ENAB");
    assert_eq!(ff.group_element, group_id(5, 0, GROUP_LEVEL_SHIFT, 0));
    assert_eq!(ff.group_element, 320);
    assert_ne!(ff.group_element, 63 + (5 << GROUP_LEVEL_SHIFT));
}

#[test]
fn rate_p_uses_the_index_zero_workaround() {
    let roll = found("ATC_RAT_RLL_P");
    let pitch = found("ATC_RAT_PIT_P");
    let yaw = found("ATC_RAT_YAW_P");
    let want_roll = 1 + (63 << GROUP_LEVEL_SHIFT);
    let want_pitch = 2 + (63 << GROUP_LEVEL_SHIFT);
    let want_yaw = 3 + (63 << GROUP_LEVEL_SHIFT);
    assert_eq!(roll.group_element, want_roll);
    assert_eq!(pitch.group_element, want_pitch);
    assert_eq!(yaw.group_element, want_yaw);
    assert_eq!(want_roll, 4033);
    assert_eq!(want_pitch, 4034);
    assert_eq!(want_yaw, 4035);
    // A roll/pitch mix-up would share a storage slot.
    assert_ne!(roll.group_element, pitch.group_element);
}

#[test]
fn rate_i_is_not_rewritten() {
    let roll_i = found("ATC_RAT_RLL_I");
    assert_eq!(roll_i.group_element, 1 + (1 << GROUP_LEVEL_SHIFT));
    assert_eq!(roll_i.group_element, 65);
}

#[test]
fn angle_p_uses_the_index_zero_workaround_at_the_third_level() {
    let p = found("ATC_ANG_RLL_P");
    let ang_base = 13 << GROUP_LEVEL_SHIFT;
    let want = ang_base + (63 << (2 * GROUP_LEVEL_SHIFT));
    assert_eq!(p.group_element, want);
    assert_eq!(want, 258_880);
    let pit = found("ATC_ANG_PIT_P");
    assert_ne!(p.group_element, pit.group_element);
}

#[test]
fn storage_header_bytes_are_pinned() {
    let mix = found("ATC_THR_MIX_MIN");
    let header = ParamHeader::new(mix.key, mix.ptype, mix.group_element);
    assert_eq!(header.to_bytes(), [0x66, 0x04, 0x01, 0x00]);

    let roll_p = found("ATC_RAT_RLL_P");
    let roll_header = ParamHeader::new(roll_p.key, roll_p.ptype, roll_p.group_element);
    // key=102, type=Float(4), group_element=4033
    assert_eq!(roll_header.to_word(), 102 | (4 << 8) | (4033 << 14));
    assert_eq!(
        roll_header.to_bytes(),
        ParamHeader::from_word(66_077_798).to_bytes()
    );
}

#[test]
fn rate_defaults_differ_on_yaw() {
    assert_eq!(RATE_RP_P.to_bits(), 0.135f32.to_bits());
    assert_eq!(RATE_RP_I.to_bits(), RATE_RP_P.to_bits());
    assert_eq!(RATE_RP_D.to_bits(), 0.0036f32.to_bits());
    assert_eq!(RATE_YAW_P.to_bits(), 0.180f32.to_bits());
    assert_eq!(RATE_YAW_I.to_bits(), 0.018f32.to_bits());
    assert_eq!(RATE_YAW_D.to_bits(), 0.0f32.to_bits());
    assert_ne!(RATE_YAW_P.to_bits(), RATE_RP_P.to_bits());
    assert_eq!(RATE_IMAX.to_bits(), 0.5f32.to_bits());
    assert_eq!(RATE_RPY_FILT_HZ.to_bits(), 20.0f32.to_bits());
    assert_eq!(RATE_YAW_FILT_E_HZ.to_bits(), 2.5f32.to_bits());
}

#[test]
fn parent_defaults_are_the_groupinfo_values() {
    assert_eq!(RATE_FF_ENAB_DEFAULT, 1);
    assert_eq!(ANGLE_BOOST_DEFAULT, 1);
    assert_eq!(ANGLE_P_DEFAULT.to_bits(), 4.5f32.to_bits());
    assert_eq!(ANG_LIM_TC_DEFAULT.to_bits(), 1.0f32.to_bits());
    assert_eq!(ANGLE_MAX_DEFAULT.to_bits(), 30.0f32.to_bits());
    assert_eq!(RATE_WPY_MAX_DEFAULT.to_bits(), 60.0f32.to_bits());
    assert_eq!(ACC_Y_MAX_DEFAULT.to_bits(), 270.0f32.to_bits());
    assert_eq!(ACC_RP_MAX_DEFAULT.to_bits(), 1100.0f32.to_bits());
    assert_eq!(LAND_MULT_DEFAULT.to_bits(), 1.0f32.to_bits());
}

#[test]
fn fltt_name_fills_the_sixteen_byte_buffer() {
    let n = found("ATC_RAT_RLL_FLTT");
    assert_eq!(n.name.as_str().len(), MAX_NAME_SIZE);
    assert_eq!(n.ptype, VarType::Float.as_u8());
}
