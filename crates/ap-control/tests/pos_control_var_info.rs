//! `AC_PosControl::var_info` leftover, upstream
//! `libraries/AC_AttitudeControl/AC_PosControl.cpp`.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes leftover table rows whose length is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_control::multi_var_info::{
    AC_PID_VAR_INFO, AC_P_VAR_INFO, ATC_PARAM_INFO, K_PARAM_ATTITUDE_CONTROL,
};
use ap_control::pos_control_ne::JERK_NE_MSSS;
use ap_control::pos_control_var_info::{
    find_pos_control_var, pos_control_var_info_entry, psc_table, AC_PID_2D_VAR_INFO,
    AC_PID_BASIC_VAR_INFO, ANGLE_MAX_DEFAULT, D_ACC_D, D_ACC_FILT_HZ, D_ACC_I, D_ACC_IMAX, D_ACC_P,
    D_ACC_P_PLANE, D_POS_P, D_VEL_FILT_HZ, D_VEL_IMAX, D_VEL_P, JERK_D_MSSS, K_PARAM_POS_CONTROL,
    NE_POS_P, NE_POS_P_PLANE, NE_VEL_D, NE_VEL_FILT_HZ, NE_VEL_I, NE_VEL_IMAX, NE_VEL_P,
    NE_VEL_P_PLANE, POS_CONTROL_SCALARS, POS_CONTROL_VAR_INFO, PSC_PARAM_INFO, PSC_PREFIX,
};
use ap_param::info::{
    enumerate, find_by_name, group_id, EnumFilter, FLAG_DEFAULT_POINTER, FLAG_INFO_POINTER,
    FLAG_NESTED_OFFSET, FLAG_POINTER, GROUP_LEVEL_SHIFT, MAX_NAME_SIZE,
};
use ap_param::{ParamHeader, VarType};

fn table() -> [ap_param::info::ParamInfo<'static>; 1] {
    psc_table()
}

fn found(name: &str) -> ap_param::info::ParamRef {
    find_by_name(&table(), EnumFilter::for_frame(0), name)
        .unwrap_or_else(|| panic!("missing {name}"))
}

#[test]
fn table_starts_with_d_pos() {
    let first = pos_control_var_info_entry().expect("_D_POS_");
    assert_eq!(first.name, "_D_POS_");
    assert_eq!(first.idx, 2);
    assert_eq!(first.ptype, VarType::Group.as_u8());
    assert_eq!(first.flags, FLAG_NESTED_OFFSET);
    assert!(first.group.is_some());
}

#[test]
fn pos_control_is_eight_rows() {
    assert_eq!(POS_CONTROL_VAR_INFO.len(), 8);
}

#[test]
fn rows_match_upstream_order_and_idx() {
    let want = [
        ("_D_POS_", 2, VarType::Group, FLAG_NESTED_OFFSET),
        ("_NE_POS_", 5, VarType::Group, FLAG_NESTED_OFFSET),
        ("_ANGLE_MAX", 7, VarType::Float, 0),
        ("_JERK_NE", 10, VarType::Float, 0),
        ("_JERK_D", 11, VarType::Float, 0),
        ("_D_VEL_", 12, VarType::Group, FLAG_NESTED_OFFSET),
        ("_D_ACC_", 13, VarType::Group, FLAG_NESTED_OFFSET),
        ("_NE_VEL_", 14, VarType::Group, FLAG_NESTED_OFFSET),
    ];
    assert_eq!(POS_CONTROL_VAR_INFO.len(), want.len());
    for (entry, (name, idx, ptype, flags)) in POS_CONTROL_VAR_INFO.iter().zip(want) {
        assert_eq!(entry.name, name, "{name}");
        assert_eq!(entry.idx, idx, "{name}");
        assert_eq!(entry.ptype, ptype.as_u8(), "{name}");
        assert_eq!(entry.flags, flags, "{name}");
    }
}

#[test]
fn historical_holes_stay_empty() {
    let used: std::collections::BTreeSet<u8> = POS_CONTROL_VAR_INFO.iter().map(|e| e.idx).collect();
    for idx in [0u8, 1, 3, 4, 6, 8, 9] {
        assert!(
            !used.contains(&idx),
            "historical hole {idx} must stay empty"
        );
    }
    for idx in [2u8, 5, 7, 10, 11, 12, 13, 14] {
        assert!(used.contains(&idx), "live idx {idx} must be present");
    }
}

#[test]
fn names_and_indices_are_unique() {
    let mut names = std::collections::BTreeSet::new();
    let mut idxs = std::collections::BTreeSet::new();
    for entry in POS_CONTROL_VAR_INFO {
        assert!(names.insert(entry.name), "duplicate name {}", entry.name);
        assert!(idxs.insert(entry.idx), "duplicate idx {}", entry.idx);
    }
}

#[test]
fn later_rows_are_not_in_this_slice() {
    assert!(find_pos_control_var("ATC_").is_none());
    assert!(find_pos_control_var("AHRS_").is_none());
    assert!(find_pos_control_var("_HOVER").is_none());
}

#[test]
fn p_tables_are_the_shared_ac_p_row() {
    let d_pos = find_pos_control_var("_D_POS_").expect("_D_POS_");
    let ne_pos = find_pos_control_var("_NE_POS_").expect("_NE_POS_");
    for child in [d_pos.group.expect("child"), ne_pos.group.expect("child")] {
        assert_eq!(child.len(), AC_P_VAR_INFO.len());
        assert_eq!(child[0].name, "P");
        assert_eq!(child[0].idx, 0);
        assert_eq!(child[0].flags, FLAG_DEFAULT_POINTER);
        assert_eq!(child[0].ptype, AC_P_VAR_INFO[0].ptype);
    }
    assert_eq!(AC_P_VAR_INFO.len(), 1);
}

#[test]
fn accel_reuses_ac_pid_and_vel_uses_basic() {
    let acc = find_pos_control_var("_D_ACC_").expect("_D_ACC_");
    let acc_child = acc.group.expect("child");
    assert_eq!(acc_child.len(), AC_PID_VAR_INFO.len());
    for (got, want) in acc_child.iter().zip(AC_PID_VAR_INFO) {
        assert_eq!(got.name, want.name);
        assert_eq!(got.idx, want.idx);
        assert_eq!(got.ptype, want.ptype);
        assert_eq!(got.flags, want.flags);
    }
    let vel = find_pos_control_var("_D_VEL_").expect("_D_VEL_");
    let vel_child = vel.group.expect("child");
    assert_eq!(vel_child.len(), AC_PID_BASIC_VAR_INFO.len());
    for (got, want) in vel_child.iter().zip(AC_PID_BASIC_VAR_INFO) {
        assert_eq!(got.name, want.name);
        assert_eq!(got.idx, want.idx);
    }
    let ne = find_pos_control_var("_NE_VEL_").expect("_NE_VEL_");
    let ne_child = ne.group.expect("child");
    assert_eq!(ne_child.len(), AC_PID_2D_VAR_INFO.len());
    for (got, want) in ne_child.iter().zip(AC_PID_2D_VAR_INFO) {
        assert_eq!(got.name, want.name);
        assert_eq!(got.idx, want.idx);
    }
    assert_eq!(AC_PID_BASIC_VAR_INFO.len(), 7);
    assert_eq!(AC_PID_2D_VAR_INFO.len(), 7);
    assert_eq!(AC_PID_VAR_INFO.len(), 13);
}

#[test]
fn pid_basic_order_matches_upstream() {
    let want = ["P", "I", "IMAX", "FLTE", "D", "FLTD", "FF"];
    assert_eq!(AC_PID_BASIC_VAR_INFO.len(), want.len());
    for (entry, name) in AC_PID_BASIC_VAR_INFO.iter().zip(want) {
        assert_eq!(entry.name, name, "{name}");
        assert_eq!(entry.ptype, VarType::Float.as_u8(), "{name}");
        assert_eq!(entry.flags, FLAG_DEFAULT_POINTER, "{name}");
    }
    for (i, entry) in AC_PID_BASIC_VAR_INFO.iter().enumerate() {
        assert_eq!(entry.idx, u8::try_from(i).expect("idx"), "{}", entry.name);
    }
}

#[test]
fn pos_control_scalars_are_the_three_float_rows() {
    assert_eq!(POS_CONTROL_SCALARS.len(), 3);
    for spec in POS_CONTROL_SCALARS {
        let entry = find_pos_control_var(spec.name).unwrap_or_else(|| panic!("{}", spec.name));
        assert_eq!(entry.idx, spec.idx, "{}", spec.name);
        assert_eq!(entry.ptype, VarType::Float.as_u8(), "{}", spec.name);
    }
    assert_eq!(
        POS_CONTROL_SCALARS[0].default.to_bits(),
        ANGLE_MAX_DEFAULT.to_bits()
    );
    assert_eq!(
        POS_CONTROL_SCALARS[1].default.to_bits(),
        JERK_NE_MSSS.to_bits()
    );
    assert_eq!(
        POS_CONTROL_SCALARS[2].default.to_bits(),
        JERK_D_MSSS.to_bits()
    );
}

#[test]
fn copter_gains_are_not_plane() {
    assert_eq!(NE_POS_P.to_bits(), 1.0f32.to_bits());
    assert_eq!(NE_POS_P_PLANE.to_bits(), 0.5f32.to_bits());
    assert_ne!(NE_POS_P.to_bits(), NE_POS_P_PLANE.to_bits());
    assert_eq!(NE_VEL_P.to_bits(), 2.0f32.to_bits());
    assert_eq!(NE_VEL_P_PLANE.to_bits(), 0.7f32.to_bits());
    assert_ne!(NE_VEL_P.to_bits(), NE_VEL_P_PLANE.to_bits());
    assert_eq!(D_ACC_P.to_bits(), 0.05f32.to_bits());
    assert_eq!(D_ACC_P_PLANE.to_bits(), 0.03f32.to_bits());
    assert_ne!(D_ACC_P.to_bits(), D_ACC_P_PLANE.to_bits());
}

#[test]
fn copter_defaults_match_the_constructor() {
    assert_eq!(D_POS_P.to_bits(), 1.0f32.to_bits());
    assert_eq!(D_VEL_P.to_bits(), 5.0f32.to_bits());
    assert_eq!(D_VEL_IMAX.to_bits(), 10.0f32.to_bits());
    assert_eq!(D_VEL_FILT_HZ.to_bits(), 5.0f32.to_bits());
    assert_eq!(D_ACC_I.to_bits(), 0.1f32.to_bits());
    assert_eq!(D_ACC_D.to_bits(), 0.0f32.to_bits());
    assert_eq!(D_ACC_IMAX.to_bits(), 0.8f32.to_bits());
    assert_eq!(D_ACC_FILT_HZ.to_bits(), 20.0f32.to_bits());
    assert_eq!(NE_VEL_I.to_bits(), 1.0f32.to_bits());
    assert_eq!(NE_VEL_D.to_bits(), 0.25f32.to_bits());
    assert_eq!(NE_VEL_IMAX.to_bits(), 10.0f32.to_bits());
    assert_eq!(NE_VEL_FILT_HZ.to_bits(), 5.0f32.to_bits());
    assert_eq!(ANGLE_MAX_DEFAULT.to_bits(), 0.0f32.to_bits());
    assert_eq!(JERK_D_MSSS.to_bits(), JERK_NE_MSSS.to_bits());
    assert_eq!(JERK_D_MSSS.to_bits(), 5.0f32.to_bits());
}

#[test]
fn gobject_key_and_flags_match_copter() {
    assert_eq!(K_PARAM_POS_CONTROL, 103);
    assert_eq!(PSC_PARAM_INFO.key, 103);
    assert_eq!(PSC_PARAM_INFO.name, PSC_PREFIX);
    assert_eq!(PSC_PARAM_INFO.ptype, VarType::Group.as_u8());
    assert_eq!(PSC_PARAM_INFO.flags, FLAG_POINTER);
    assert_ne!(PSC_PARAM_INFO.flags, FLAG_POINTER | FLAG_INFO_POINTER);
    assert_ne!(PSC_PARAM_INFO.flags, ATC_PARAM_INFO.flags);
    assert_eq!(K_PARAM_ATTITUDE_CONTROL, 102);
    assert_ne!(K_PARAM_POS_CONTROL, K_PARAM_ATTITUDE_CONTROL);
}

#[test]
fn enumerates_thirty_two_leaves() {
    let mut n = 0_usize;
    enumerate(&table(), EnumFilter::for_frame(0), &mut |_| {
        n += 1;
    });
    // 1 D_POS + 1 NE_POS + 3 scalars + 7 D_VEL + 13 D_ACC + 7 NE_VEL.
    assert_eq!(n, 32);
}

#[test]
fn names_are_unique_and_fit() {
    let mut names = std::collections::BTreeSet::new();
    enumerate(&table(), EnumFilter::for_frame(0), &mut |r| {
        let name = r.name.as_str();
        assert!(name.len() <= MAX_NAME_SIZE, "{name}");
        assert!(name.starts_with("PSC"), "{name}");
        assert!(names.insert(name.to_string()), "duplicate {name}");
        assert!(r.behind_pointer, "{name} sits behind GOBJECTPTR");
        assert_eq!(r.key, K_PARAM_POS_CONTROL, "{name}");
    });
    assert_eq!(names.len(), 32);
}

#[test]
fn find_by_name_resolves_scalars_and_pids() {
    let angle = found("PSC_ANGLE_MAX");
    assert_eq!(angle.ptype, VarType::Float.as_u8());
    assert_eq!(angle.group_element, 7);

    let jerk = found("PSC_JERK_NE");
    assert_eq!(jerk.group_element, 10);

    let d_pos = found("PSC_D_POS_P");
    assert_eq!(d_pos.ptype, VarType::Float.as_u8());

    assert!(find_by_name(&table(), EnumFilter::for_frame(0), "PSC").is_none());
    assert!(find_by_name(&table(), EnumFilter::for_frame(0), "ANGLE_MAX").is_none());
}

#[test]
fn pos_p_uses_the_index_zero_workaround() {
    let d = found("PSC_D_POS_P");
    let ne = found("PSC_NE_POS_P");
    let want_d = 2 + (63 << GROUP_LEVEL_SHIFT);
    let want_ne = 5 + (63 << GROUP_LEVEL_SHIFT);
    assert_eq!(d.group_element, want_d);
    assert_eq!(ne.group_element, want_ne);
    assert_eq!(want_d, 4034);
    assert_eq!(want_ne, 4037);
    assert_ne!(d.group_element, ne.group_element);
}

#[test]
fn d_vel_p_uses_the_index_zero_workaround() {
    let p = found("PSC_D_VEL_P");
    assert_eq!(p.group_element, 12 + (63 << GROUP_LEVEL_SHIFT));
    assert_eq!(p.group_element, 4044);
    let i = found("PSC_D_VEL_I");
    assert_eq!(i.group_element, 12 + (1 << GROUP_LEVEL_SHIFT));
    assert_eq!(i.group_element, 76);
}

#[test]
fn d_acc_p_uses_the_index_zero_workaround() {
    let p = found("PSC_D_ACC_P");
    assert_eq!(p.group_element, 13 + (63 << GROUP_LEVEL_SHIFT));
    assert_eq!(p.group_element, 4045);
    let ff = found("PSC_D_ACC_FF");
    assert_eq!(ff.group_element, group_id(4, 13, GROUP_LEVEL_SHIFT, 0));
    assert_eq!(ff.group_element, 13 + (4 << GROUP_LEVEL_SHIFT));
    assert_eq!(ff.group_element, 269);
}

#[test]
fn ne_vel_p_uses_the_index_zero_workaround() {
    let p = found("PSC_NE_VEL_P");
    assert_eq!(p.group_element, 14 + (63 << GROUP_LEVEL_SHIFT));
    assert_eq!(p.group_element, 4046);
}

#[test]
fn storage_header_bytes_are_pinned() {
    let angle = found("PSC_ANGLE_MAX");
    let header = ParamHeader::new(angle.key, angle.ptype, angle.group_element);
    assert_eq!(header.to_word(), 103 | (4 << 8) | (7 << 14));
    assert_eq!(
        header.to_bytes(),
        ParamHeader::from_word(115_815).to_bytes()
    );

    let d_pos = found("PSC_D_POS_P");
    let d_header = ParamHeader::new(d_pos.key, d_pos.ptype, d_pos.group_element);
    assert_eq!(d_header.to_word(), 103 | (4 << 8) | (4034 << 14));
}

#[test]
fn ac_pid_holes_stay_empty_under_d_acc() {
    assert!(find_by_name(&table(), EnumFilter::for_frame(0), "PSC_D_ACC_SMAX").is_some());
    // idx 3 is unused on AC_PID; a port that packed FF there would
    // collide with IMAX+1 and break EEPROM.
    let used: std::collections::BTreeSet<u8> = AC_PID_VAR_INFO.iter().map(|e| e.idx).collect();
    for idx in [3u8, 6, 7, 8] {
        assert!(!used.contains(&idx), "AC_PID hole {idx} must stay empty");
    }
}
