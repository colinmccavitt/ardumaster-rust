//! `AC_PosControl::var_info` leftover (COP-009).
//!
//! Copter mounts this table at `PSC` (`GOBJECTPTR`,
//! `Parameters::k_param_pos_control` = 103). ADR-0010 pins the names,
//! indices and `group_element` encoding: those are the storage address,
//! including the historical holes and the index-zero workaround.
//!
//! # What this table actually is
//!
//! Eight rows, then `AP_GROUPEND`:
//!
//! 1. Five `AP_SUBGROUPINFO` PID/P groups (`_D_POS_` / `_NE_POS_` /
//!    `_D_VEL_` / `_D_ACC_` / `_NE_VEL_`, idx 2, 5, 12-14) with
//!    `AP_PARAM_FLAG_NESTED_OFFSET`.
//! 2. Three `AP_Float` scalars (`_ANGLE_MAX` / `_JERK_NE` / `_JERK_D`,
//!    idx 7, 10, 11).
//!
//! Nested `AC_P_1D` / `AC_P_2D` / `AC_PID_Basic` / `AC_PID` /
//! `AC_PID_2D` `var_info` are part of this leftover: those five
//! subgroups *are* those tables. `AC_P` and `AC_PID` already live in
//! [`crate::multi_var_info`] and are reused; the 1-D / 2-D P tables
//! match `AC_P` row-for-row.
//!
//! # Copter defaults, not Plane
//!
//! `POSCONTROL_NE_VEL_P` is 2.0 on Copter. Plane's 0.7 is a different
//! `APM_BUILD_TYPE` branch of the same file. `_ANGLE_MAX` defaults to
//! zero so the attitude controller's `ANGLE_MAX` is used instead.

use crate::multi_var_info::{AC_PID_VAR_INFO, AC_P_VAR_INFO};
use crate::pos_control_ne::JERK_NE_MSSS;

/// Copter `POSCONTROL_NE_POS_P`. Same value as [`crate::pos_control_ne::NE_POS_P`].
pub use crate::pos_control_ne::NE_POS_P;
use ap_param::info::{
    GroupInfo, ParamInfo, FLAG_DEFAULT_POINTER, FLAG_NESTED_OFFSET, FLAG_POINTER,
};
use ap_param::VarType;

/// `Parameters::k_param_pos_control`. After `k_param_attitude_control` (102).
pub const K_PARAM_POS_CONTROL: u16 = 103;

/// Copter `GOBJECTPTR` prefix. Concatenated onto every name below.
pub const PSC_PREFIX: &str = "PSC";

/// Copter `POSCONTROL_D_POS_P`.
pub const D_POS_P: f32 = 1.0;

/// Copter `POSCONTROL_D_VEL_P`.
pub const D_VEL_P: f32 = 5.0;

/// Copter `POSCONTROL_D_VEL_IMAX`.
pub const D_VEL_IMAX: f32 = 10.0;

/// Copter `POSCONTROL_D_VEL_FILT_HZ` / `POSCONTROL_D_VEL_FILT_D_HZ`.
pub const D_VEL_FILT_HZ: f32 = 5.0;

/// Copter `POSCONTROL_D_ACC_P`.
pub const D_ACC_P: f32 = 0.05;

/// Plane `POSCONTROL_D_ACC_P`. Not this leftover.
pub const D_ACC_P_PLANE: f32 = 0.03;

/// Copter `POSCONTROL_D_ACC_I`.
pub const D_ACC_I: f32 = 0.1;

/// Copter `POSCONTROL_D_ACC_D`.
pub const D_ACC_D: f32 = 0.0;

/// Copter `POSCONTROL_D_ACC_IMAX`.
pub const D_ACC_IMAX: f32 = 0.8;

/// Copter `POSCONTROL_D_ACC_FILT_HZ` — the error filter. Target and
/// derivative filters default to zero in the constructor.
pub const D_ACC_FILT_HZ: f32 = 20.0;

/// Plane `POSCONTROL_NE_POS_P`. Copter's value is [`NE_POS_P`].
pub const NE_POS_P_PLANE: f32 = 0.5;

/// Copter `POSCONTROL_NE_VEL_P`.
pub const NE_VEL_P: f32 = 2.0;

/// Plane `POSCONTROL_NE_VEL_P`. Not this leftover.
pub const NE_VEL_P_PLANE: f32 = 0.7;

/// Copter `POSCONTROL_NE_VEL_I`.
pub const NE_VEL_I: f32 = 1.0;

/// Copter `POSCONTROL_NE_VEL_D`.
pub const NE_VEL_D: f32 = 0.25;

/// Copter `POSCONTROL_NE_VEL_IMAX`.
pub const NE_VEL_IMAX: f32 = 10.0;

/// Copter `POSCONTROL_NE_VEL_FILT_HZ` / `POSCONTROL_NE_VEL_FILT_D_HZ`.
pub const NE_VEL_FILT_HZ: f32 = 5.0;

/// `_ANGLE_MAX` GroupInfo default. Zero means use the attitude
/// controller's `ANGLE_MAX`.
pub const ANGLE_MAX_DEFAULT: f32 = 0.0;

/// `POSCONTROL_JERK_D_MSSS`.
pub const JERK_D_MSSS: f32 = 5.0;

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

/// `AC_PID_Basic::var_info`. Seven `DEFAULT_POINTER` floats, idx 0-6.
pub const AC_PID_BASIC_VAR_INFO: &[GroupInfo<'static>] = &[
    scalar("P", 0, VarType::Float, FLAG_DEFAULT_POINTER),
    scalar("I", 1, VarType::Float, FLAG_DEFAULT_POINTER),
    scalar("IMAX", 2, VarType::Float, FLAG_DEFAULT_POINTER),
    scalar("FLTE", 3, VarType::Float, FLAG_DEFAULT_POINTER),
    scalar("D", 4, VarType::Float, FLAG_DEFAULT_POINTER),
    scalar("FLTD", 5, VarType::Float, FLAG_DEFAULT_POINTER),
    scalar("FF", 6, VarType::Float, FLAG_DEFAULT_POINTER),
];

/// `AC_PID_2D::var_info`. Same rows and indices as [`AC_PID_BASIC_VAR_INFO`].
pub const AC_PID_2D_VAR_INFO: &[GroupInfo<'static>] = AC_PID_BASIC_VAR_INFO;

/// `AC_PosControl::var_info`.
///
/// Historical holes 0-1, 3-4, 6, 8-9 stay empty. They used to hold
/// `HOVER`, `POS_ACC_XY_FILT`, `_VELZ_`, `_ACCZ_`, `_VELXY_`, `_TC_XY`,
/// `_TC_Z`.
pub const POS_CONTROL_VAR_INFO: &[GroupInfo<'static>] = &[
    group("_D_POS_", 2, FLAG_NESTED_OFFSET, AC_P_VAR_INFO),
    group("_NE_POS_", 5, FLAG_NESTED_OFFSET, AC_P_VAR_INFO),
    scalar("_ANGLE_MAX", 7, VarType::Float, 0),
    scalar("_JERK_NE", 10, VarType::Float, 0),
    scalar("_JERK_D", 11, VarType::Float, 0),
    group("_D_VEL_", 12, FLAG_NESTED_OFFSET, AC_PID_BASIC_VAR_INFO),
    group("_D_ACC_", 13, FLAG_NESTED_OFFSET, AC_PID_VAR_INFO),
    group("_NE_VEL_", 14, FLAG_NESTED_OFFSET, AC_PID_2D_VAR_INFO),
];

/// Copter `GOBJECTPTR(pos_control, "PSC", AC_PosControl)`.
///
/// `AP_PARAM_FLAG_POINTER` only: the object is reached through a
/// pointer, the table is not. A null object contributes no parameters
/// on a running vehicle; the table still describes them.
pub const PSC_PARAM_INFO: ParamInfo<'static> = ParamInfo {
    name: PSC_PREFIX,
    key: K_PARAM_POS_CONTROL,
    ptype: VarType::Group.as_u8(),
    flags: FLAG_POINTER,
    group: Some(POS_CONTROL_VAR_INFO),
};

/// One-row vehicle table used by `find_by_name` / `enumerate`.
#[must_use]
pub const fn psc_table() -> [ParamInfo<'static>; 1] {
    [PSC_PARAM_INFO]
}

/// First row of PosControl's table (`_D_POS_`).
#[must_use]
pub fn pos_control_var_info_entry() -> Option<&'static GroupInfo<'static>> {
    POS_CONTROL_VAR_INFO.first()
}

/// Find a PosControl row by its `GroupInfo` name fragment.
#[must_use]
pub fn find_pos_control_var(name: &str) -> Option<&'static GroupInfo<'static>> {
    POS_CONTROL_VAR_INFO.iter().find(|entry| entry.name == name)
}

/// One PosControl-owned scalar: name, idx, GroupInfo default.
#[derive(Debug, Clone, Copy)]
pub struct PosControlScalar {
    /// `@Param` fragment (`_ANGLE_MAX`, ...).
    pub name: &'static str,
    /// Identifier within PosControl's table.
    pub idx: u8,
    /// `Info.def_value`. Stored as float, as every `AP_GROUPINFO` default is.
    pub default: f32,
}

/// The three `AP_Float` rows PosControl itself owns.
pub const POS_CONTROL_SCALARS: &[PosControlScalar] = &[
    PosControlScalar {
        name: "_ANGLE_MAX",
        idx: 7,
        default: ANGLE_MAX_DEFAULT,
    },
    PosControlScalar {
        name: "_JERK_NE",
        idx: 10,
        default: JERK_NE_MSSS,
    },
    PosControlScalar {
        name: "_JERK_D",
        idx: 11,
        default: JERK_D_MSSS,
    },
];
