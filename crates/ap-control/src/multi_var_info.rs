//! `AC_AttitudeControl_Multi::var_info` leftover (COP-008).
//!
//! The last COP-008 leftover. Multi's table is the one Copter mounts at
//! `ATC_` (`GOBJECTVARPTR`, `Parameters::k_param_attitude_control` = 102).
//! ADR-0010 pins the names, indices and `group_element` encoding: those
//! are the storage address, including the historical holes and the
//! index-zero workaround.
//!
//! # What this table actually is
//!
//! Eight rows, then `AP_GROUPEND`:
//!
//! 1. `AP_NESTEDGROUPINFO(AC_AttitudeControl, 0)` — empty name, flags 0.
//!    Parent parameters appear as `ATC_RATE_FF_ENAB`, not under a nested
//!    prefix. At this depth `shift` is still 0, so idx 0 is *not* rewritten
//!    to 63. The rewrite fires one level down, on `AC_PID` / `AC_P` `"P"`.
//! 2. Three `AP_SUBGROUPINFO` rate PIDs (`RAT_RLL_` / `RAT_PIT_` /
//!    `RAT_YAW_`, idx 1-3) with `AP_PARAM_FLAG_NESTED_OFFSET`.
//! 3. Four `AP_Float` mix parameters, idx 4-7.
//!
//! Nested `AC_PID::var_info` and `AC_AttitudeControl::var_info` (and the
//! `AC_P` it embeds) are part of this leftover: Multi's first four rows
//! *are* those tables. Heli / 6DoF `var_info` pointers stay later.
//!
//! # Copter defaults, not Plane
//!
//! `INPUT_TC` is 0.10 on Copter (`Crisp`). Plane's 0.15 is a different
//! `APM_BUILD_TYPE` branch of the same file.

use ap_param::info::{
    GroupInfo, ParamInfo, FLAG_DEFAULT_POINTER, FLAG_INFO_POINTER, FLAG_NESTED_OFFSET, FLAG_POINTER,
};
use ap_param::VarType;

/// `Parameters::k_param_attitude_control`. After `k_param_wp_nav` (101).
pub const K_PARAM_ATTITUDE_CONTROL: u16 = 102;

/// Copter `GOBJECTVARPTR` prefix. Concatenated onto every name below.
pub const ATC_PREFIX: &str = "ATC_";

/// `AC_ATTITUDE_CONTROL_MIN_DEFAULT`.
pub const THR_MIX_MIN_DEFAULT: f32 = 0.1;

/// `AC_ATTITUDE_CONTROL_MAX_DEFAULT`.
pub const THR_MIX_MAX_DEFAULT: f32 = 0.5;

/// `AC_ATTITUDE_CONTROL_MAN_DEFAULT`.
pub const THR_MIX_MAN_DEFAULT: f32 = 0.1;

/// `THR_G_BOOST` GroupInfo default.
pub const THR_G_BOOST_DEFAULT: f32 = 0.0;

/// Copter `AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT` (`Crisp`). Not Plane's 0.15.
pub const INPUT_TC_DEFAULT: f32 = 0.10;

/// Plane `AC_ATTITUDE_CONTROL_INPUT_TC_DEFAULT`. Not this leftover.
pub const INPUT_TC_DEFAULT_PLANE: f32 = 0.15;

/// `AC_ATTITUDE_CONTROL_RATE_BF_FF_DEFAULT`.
pub const RATE_FF_ENAB_DEFAULT: i8 = 1;

/// `ANGLE_BOOST` GroupInfo default.
pub const ANGLE_BOOST_DEFAULT: i8 = 1;

/// `AC_ATTITUDE_CONTROL_ANGLE_LIMIT_TC_DEFAULT`.
pub const ANG_LIM_TC_DEFAULT: f32 = 1.0;

/// `AC_ATTITUDE_CONTROL_ANGLE_P`.
pub const ANGLE_P_DEFAULT: f32 = 4.5;

/// `AC_ATTITUDE_CONTROL_ANGLE_MAX_DEFAULT`.
pub const ANGLE_MAX_DEFAULT: f32 = 30.0;

/// `AC_ATTITUDE_CONTROL_RATE_WPY_MAX_DEFAULT`.
pub const RATE_WPY_MAX_DEFAULT: f32 = 60.0;

/// `AC_ATTITUDE_CONTROL_ACCEL_Y_MAX_DEFAULT_DEGSS`.
pub const ACC_Y_MAX_DEFAULT: f32 = 270.0;

/// `AC_ATTITUDE_CONTROL_ACCEL_RP_MAX_DEFAULT_DEGSS`.
pub const ACC_RP_MAX_DEFAULT: f32 = 1100.0;

/// Landed gain-multiplier GroupInfo default.
pub const LAND_MULT_DEFAULT: f32 = 1.0;

/// `AC_ATC_MULTI_RATE_RP_P` / `_I`.
pub const RATE_RP_P: f32 = 0.135;
/// Same as [`RATE_RP_P`]. Upstream's I default matches P on roll and pitch.
pub const RATE_RP_I: f32 = 0.135;
/// `AC_ATC_MULTI_RATE_RP_D`.
pub const RATE_RP_D: f32 = 0.0036;
/// `AC_ATC_MULTI_RATE_RP_IMAX` / yaw IMAX.
pub const RATE_IMAX: f32 = 0.5;
/// `AC_ATC_MULTI_RATE_RPY_FILT_HZ`.
pub const RATE_RPY_FILT_HZ: f32 = 20.0;
/// `AC_ATC_MULTI_RATE_YAW_P`.
pub const RATE_YAW_P: f32 = 0.180;
/// `AC_ATC_MULTI_RATE_YAW_I`.
pub const RATE_YAW_I: f32 = 0.018;
/// `AC_ATC_MULTI_RATE_YAW_D`.
pub const RATE_YAW_D: f32 = 0.0;
/// `AC_ATC_MULTI_RATE_YAW_FILT_HZ` — error filter only; target stays at 20 Hz.
pub const RATE_YAW_FILT_E_HZ: f32 = 2.5;

const fn group(
    name: &'static str,
    idx: u8,
    flags: u16,
    child: &'static [GroupInfo<'static>],
) -> GroupInfo<'static> {
    GroupInfo {
        name,
        idx,
        ptype: VarType::Group.as_u8(),
        flags,
        group: Some(child),
    }
}

const fn scalar(name: &'static str, idx: u8, ptype: VarType, flags: u16) -> GroupInfo<'static> {
    GroupInfo {
        name,
        idx,
        ptype: ptype.as_u8(),
        flags,
        group: None,
    }
}

/// `AC_P::var_info`. One `DEFAULT_POINTER` `"P"` at idx 0.
pub const AC_P_VAR_INFO: &[GroupInfo<'static>] =
    &[scalar("P", 0, VarType::Float, FLAG_DEFAULT_POINTER)];

/// `AC_PID::var_info` as Multi's rate subgroups embed it.
///
/// Indices 3 and 6-8 are unused. `"P"` is idx 0, so under `RAT_*_` it
/// is stored at `base + (63 << 6)`, not `base`.
pub const AC_PID_VAR_INFO: &[GroupInfo<'static>] = &[
    scalar("P", 0, VarType::Float, FLAG_DEFAULT_POINTER),
    scalar("I", 1, VarType::Float, FLAG_DEFAULT_POINTER),
    scalar("D", 2, VarType::Float, FLAG_DEFAULT_POINTER),
    scalar("FF", 4, VarType::Float, FLAG_DEFAULT_POINTER),
    scalar("IMAX", 5, VarType::Float, FLAG_DEFAULT_POINTER),
    scalar("FLTT", 9, VarType::Float, FLAG_DEFAULT_POINTER),
    scalar("FLTE", 10, VarType::Float, FLAG_DEFAULT_POINTER),
    scalar("FLTD", 11, VarType::Float, FLAG_DEFAULT_POINTER),
    scalar("SMAX", 12, VarType::Float, FLAG_DEFAULT_POINTER),
    scalar("PDMX", 13, VarType::Float, 0),
    scalar("D_FF", 14, VarType::Float, FLAG_DEFAULT_POINTER),
    scalar("NTF", 15, VarType::Int8, 0),
    scalar("NEF", 16, VarType::Int8, 0),
];

/// `AC_AttitudeControl::var_info`, nested at Multi idx 0.
///
/// Historical holes 0-4 and 6-11 stay empty. They used to hold
/// `RATE_RP_MAX`, `SLEW_YAW`, `ACCEL_*` in older units.
pub const ATTITUDE_CONTROL_VAR_INFO: &[GroupInfo<'static>] = &[
    scalar("RATE_FF_ENAB", 5, VarType::Int8, 0),
    scalar("ANGLE_BOOST", 12, VarType::Int8, 0),
    group("ANG_RLL_", 13, FLAG_NESTED_OFFSET, AC_P_VAR_INFO),
    group("ANG_PIT_", 14, FLAG_NESTED_OFFSET, AC_P_VAR_INFO),
    group("ANG_YAW_", 15, FLAG_NESTED_OFFSET, AC_P_VAR_INFO),
    scalar("ANG_LIM_TC", 16, VarType::Float, 0),
    scalar("RATE_R_MAX", 17, VarType::Float, 0),
    scalar("RATE_P_MAX", 18, VarType::Float, 0),
    scalar("RATE_Y_MAX", 19, VarType::Float, 0),
    scalar("INPUT_TC", 20, VarType::Float, 0),
    scalar("LAND_R_MULT", 21, VarType::Float, 0),
    scalar("LAND_P_MULT", 22, VarType::Float, 0),
    scalar("LAND_Y_MULT", 23, VarType::Float, 0),
    scalar("ANGLE_MAX", 24, VarType::Float, 0),
    scalar("RATE_WPY_MAX", 25, VarType::Float, 0),
    scalar("ACC_Y_MAX", 26, VarType::Float, 0),
    scalar("ACC_R_MAX", 27, VarType::Float, 0),
    scalar("ACC_P_MAX", 28, VarType::Float, 0),
];

/// `AC_AttitudeControl_Multi::var_info`.
pub const MULTI_VAR_INFO: &[GroupInfo<'static>] = &[
    group("", 0, 0, ATTITUDE_CONTROL_VAR_INFO),
    group("RAT_RLL_", 1, FLAG_NESTED_OFFSET, AC_PID_VAR_INFO),
    group("RAT_PIT_", 2, FLAG_NESTED_OFFSET, AC_PID_VAR_INFO),
    group("RAT_YAW_", 3, FLAG_NESTED_OFFSET, AC_PID_VAR_INFO),
    scalar("THR_MIX_MIN", 4, VarType::Float, 0),
    scalar("THR_MIX_MAX", 5, VarType::Float, 0),
    scalar("THR_MIX_MAN", 6, VarType::Float, 0),
    scalar("THR_G_BOOST", 7, VarType::Float, 0),
];

/// Copter `GOBJECTVARPTR(attitude_control, "ATC_", &var_info)`.
///
/// `AP_PARAM_FLAG_POINTER | AP_PARAM_FLAG_INFO_POINTER`: the object and
/// the table are both reached through pointers. A null object contributes
/// no parameters on a running vehicle; the table still describes them.
pub const ATC_PARAM_INFO: ParamInfo<'static> = ParamInfo {
    name: ATC_PREFIX,
    key: K_PARAM_ATTITUDE_CONTROL,
    ptype: VarType::Group.as_u8(),
    flags: FLAG_POINTER | FLAG_INFO_POINTER,
    group: Some(MULTI_VAR_INFO),
};

/// One-row vehicle table used by `find_by_name` / `enumerate`.
#[must_use]
pub const fn atc_table() -> [ParamInfo<'static>; 1] {
    [ATC_PARAM_INFO]
}

/// First row of Multi's table (the nested parent).
#[must_use]
pub fn multi_var_info_entry() -> Option<&'static GroupInfo<'static>> {
    MULTI_VAR_INFO.first()
}

/// Find a Multi row by its `GroupInfo` name fragment.
#[must_use]
pub fn find_multi_var(name: &str) -> Option<&'static GroupInfo<'static>> {
    MULTI_VAR_INFO.iter().find(|entry| entry.name == name)
}

/// One Multi-owned scalar: name, idx, GroupInfo default.
#[derive(Debug, Clone, Copy)]
pub struct MultiScalar {
    /// `@Param` fragment (`THR_MIX_MIN`, ...).
    pub name: &'static str,
    /// Identifier within Multi's table.
    pub idx: u8,
    /// `Info.def_value`. Stored as float, as every `AP_GROUPINFO` default is.
    pub default: f32,
}

/// The four `AP_Float` rows Multi itself owns.
pub const MULTI_SCALARS: &[MultiScalar] = &[
    MultiScalar {
        name: "THR_MIX_MIN",
        idx: 4,
        default: THR_MIX_MIN_DEFAULT,
    },
    MultiScalar {
        name: "THR_MIX_MAX",
        idx: 5,
        default: THR_MIX_MAX_DEFAULT,
    },
    MultiScalar {
        name: "THR_MIX_MAN",
        idx: 6,
        default: THR_MIX_MAN_DEFAULT,
    },
    MultiScalar {
        name: "THR_G_BOOST",
        idx: 7,
        default: THR_G_BOOST_DEFAULT,
    },
];
