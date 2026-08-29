//! Copter parameter-table leftover, upstream `ArduCopter/Parameters.cpp`.
//!
//! Tracked as **COP-023**. The leftover is the first `Copter::var_info`
//! GSCALAR slice — `FORMAT_VERSION` through `SIMPLE` — before
//! `LOG_BITMASK` and the first `GOBJECT`. Keys come from
//! `Parameters::k_param_*`, not from table order: `SIMPLE` is 206 even
//! though it sits after `INITIAL_MODE` (208). Later groups, G2,
//! `load_parameters` conversions, and the rest of the enum stay later.
//!
//! # The GSCALAR default is not `k_format_version`
//!
//! `FORMAT_VERSION`'s table default is 0. `Parameters::k_format_version`
//! is 120. `Copter::load_parameters` compares the stored value to that
//! 120, not to the GSCALAR default. A port that treated 0 as the layout
//! version would wipe a current EEPROM on every boot.
//!
//! # RTL sits in this slice, heli yaw does not
//!
//! `MODE_RTL_ENABLED` is 1 on a stock multicopter, so `RTL_CONE_SLOPE`,
//! `RTL_LOIT_TIME`, and `RTL_ALT_TYPE` are in the compiled table. The
//! heli `WP_YAW_BEHAVIOR` default (`LOOK_AHEAD`) is a `FRAME_CONFIG`
//! rewrite, not a row. This leftover keeps the multicopter default.

use ap_param::info::ParamInfo;
use ap_param::VarType;

/// Layout version, upstream `Parameters::k_format_version`.
///
/// Compared by `load_parameters`. Not the `FORMAT_VERSION` GSCALAR
/// default, which is 0.
pub const K_FORMAT_VERSION: u16 = 120;

/// `Parameters::k_param_format_version`. Always key zero.
pub const K_PARAM_FORMAT_VERSION: u16 = 0;

/// `Parameters::k_param_throttle_filt` — `PILOT_THR_FILT`.
pub const K_PARAM_THROTTLE_FILT: u16 = 62;

/// `Parameters::k_param_throttle_behavior` — `PILOT_THR_BHV`.
pub const K_PARAM_THROTTLE_BEHAVIOR: u16 = 63;

/// `Parameters::k_param_gcs_pid_mask` — `GCS_PID_MASK`.
pub const K_PARAM_GCS_PID_MASK: u16 = 126;

/// `Parameters::k_param_rtl_cone_slope` — `RTL_CONE_SLOPE`.
pub const K_PARAM_RTL_CONE_SLOPE: u16 = 137;

/// `Parameters::k_param_rtl_loiter_time` — `RTL_LOIT_TIME`.
pub const K_PARAM_RTL_LOITER_TIME: u16 = 162;

/// `Parameters::k_param_rtl_alt_type` — `RTL_ALT_TYPE`.
pub const K_PARAM_RTL_ALT_TYPE: u16 = 94;

/// `Parameters::k_param_failsafe_gcs` — `FS_GCS_ENABLE`.
pub const K_PARAM_FAILSAFE_GCS: u16 = 198;

/// `Parameters::k_param_gps_hdop_good` — `GPS_HDOP_GOOD`.
pub const K_PARAM_GPS_HDOP_GOOD: u16 = 35;

/// `Parameters::k_param_super_simple` — `SUPER_SIMPLE`.
pub const K_PARAM_SUPER_SIMPLE: u16 = 155;

/// `Parameters::k_param_wp_yaw_behavior` — `WP_YAW_BEHAVIOR`.
pub const K_PARAM_WP_YAW_BEHAVIOR: u16 = 26;

/// `Parameters::k_param_failsafe_throttle` — `FS_THR_ENABLE`.
pub const K_PARAM_FAILSAFE_THROTTLE: u16 = 182;

/// `Parameters::k_param_failsafe_throttle_value` — `FS_THR_VALUE`.
pub const K_PARAM_FAILSAFE_THROTTLE_VALUE: u16 = 184;

/// `Parameters::k_param_throttle_deadzone` — `THR_DZ`.
pub const K_PARAM_THROTTLE_DEADZONE: u16 = 57;

/// `Parameters::k_param_flight_mode1` — `FLTMODE1`.
pub const K_PARAM_FLIGHT_MODE1: u16 = 200;

/// `Parameters::k_param_flight_mode2` — `FLTMODE2`.
pub const K_PARAM_FLIGHT_MODE2: u16 = 201;

/// `Parameters::k_param_flight_mode3` — `FLTMODE3`.
pub const K_PARAM_FLIGHT_MODE3: u16 = 202;

/// `Parameters::k_param_flight_mode4` — `FLTMODE4`.
pub const K_PARAM_FLIGHT_MODE4: u16 = 203;

/// `Parameters::k_param_flight_mode5` — `FLTMODE5`.
pub const K_PARAM_FLIGHT_MODE5: u16 = 204;

/// `Parameters::k_param_flight_mode6` — `FLTMODE6`.
pub const K_PARAM_FLIGHT_MODE6: u16 = 205;

/// `Parameters::k_param_simple_modes` — `SIMPLE`.
pub const K_PARAM_SIMPLE_MODES: u16 = 206;

/// `Parameters::k_param_flight_mode_chan` — `FLTMODE_CH`.
pub const K_PARAM_FLIGHT_MODE_CHAN: u16 = 207;

/// `Parameters::k_param_initial_mode` — `INITIAL_MODE`.
pub const K_PARAM_INITIAL_MODE: u16 = 208;

/// `RTL_CONE_SLOPE_DEFAULT` from Copter `config.h`.
pub const RTL_CONE_SLOPE_DEFAULT: f32 = 3.0;

/// `RTL_LOITER_TIME` from Copter `config.h`, milliseconds.
pub const RTL_LOITER_TIME_MS: i32 = 5_000;

/// `GPS_HDOP_GOOD_DEFAULT` from Copter `config.h`.
pub const GPS_HDOP_GOOD_DEFAULT: i16 = 140;

/// `WP_YAW_BEHAVIOR_NONE`.
pub const WP_YAW_BEHAVIOR_NONE: u8 = 0;

/// `WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP`.
pub const WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP: u8 = 1;

/// `WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL` — stock multicopter default.
pub const WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL: u8 = 2;

/// `WP_YAW_BEHAVIOR_LOOK_AHEAD` — heli `FRAME_CONFIG` rewrite, not this leftover.
pub const WP_YAW_BEHAVIOR_LOOK_AHEAD: u8 = 3;

/// `WP_YAW_BEHAVIOR_DEFAULT` when `FRAME_CONFIG != HELI_FRAME`.
pub const WP_YAW_BEHAVIOR_DEFAULT: u8 = WP_YAW_BEHAVIOR_LOOK_AT_NEXT_WP_EXCEPT_RTL;

/// `FS_GCS_DISABLED`.
pub const FS_GCS_DISABLED: u8 = 0;

/// `FS_THR_ENABLED_ALWAYS_RTL`.
pub const FS_THR_ENABLED_ALWAYS_RTL: u8 = 1;

/// Copter `FS_THR_VALUE_DEFAULT`.
pub const FS_THR_VALUE_DEFAULT: i16 = 975;

/// `THR_DZ_DEFAULT` from Copter `config.h`.
pub const THR_DZ_DEFAULT: i16 = 100;

/// `CH_MODE_DEFAULT` — stock `FLTMODE_CH`.
pub const CH_MODE_DEFAULT: u8 = 5;

/// `Mode::Number::STABILIZE` — stock `FLIGHT_MODE_1`..`6` and `INITIAL_MODE`.
pub const FLIGHT_MODE_STABILIZE: u8 = 0;

/// One `Copter::var_info[]` GSCALAR row in the first leftover slice.
#[derive(Debug, Clone, Copy)]
pub struct VarInfoSpec {
    /// `@Param` name as stored in `Info.name`.
    pub name: &'static str,
    /// Nine-bit storage key, upstream `Parameters::k_param_*`.
    pub key: u16,
    /// `AP_ParamT::vtype` of the member.
    pub ptype: VarType,
    /// `Info.def_value`. Upstream stores every GSCALAR default as float.
    pub default: f32,
}

impl VarInfoSpec {
    /// Descriptor row for [`ap_param::info::find_by_name`].
    #[must_use]
    pub const fn param_info(self) -> ParamInfo<'static> {
        ParamInfo {
            name: self.name,
            key: self.key,
            ptype: self.ptype.as_u8(),
            flags: 0,
            group: None,
        }
    }
}

const fn scalar(name: &'static str, key: u16, ptype: VarType, default: f32) -> VarInfoSpec {
    VarInfoSpec {
        name,
        key,
        ptype,
        default,
    }
}

/// First `Copter::var_info[]` GSCALAR leftover catalog.
///
/// Order is table order, not key order. `MODE_RTL_ENABLED` rows stay
/// here because a stock multicopter compiles them in.
pub const FIRST_VAR_INFO: &[VarInfoSpec] = &[
    scalar(
        "FORMAT_VERSION",
        K_PARAM_FORMAT_VERSION,
        VarType::Int16,
        0.0,
    ),
    scalar("PILOT_THR_FILT", K_PARAM_THROTTLE_FILT, VarType::Float, 0.0),
    scalar(
        "PILOT_THR_BHV",
        K_PARAM_THROTTLE_BEHAVIOR,
        VarType::Int16,
        0.0,
    ),
    scalar("GCS_PID_MASK", K_PARAM_GCS_PID_MASK, VarType::Int16, 0.0),
    scalar(
        "RTL_CONE_SLOPE",
        K_PARAM_RTL_CONE_SLOPE,
        VarType::Float,
        RTL_CONE_SLOPE_DEFAULT,
    ),
    scalar(
        "RTL_LOIT_TIME",
        K_PARAM_RTL_LOITER_TIME,
        VarType::Int32,
        RTL_LOITER_TIME_MS as f32,
    ),
    scalar("RTL_ALT_TYPE", K_PARAM_RTL_ALT_TYPE, VarType::Int8, 0.0),
    scalar(
        "FS_GCS_ENABLE",
        K_PARAM_FAILSAFE_GCS,
        VarType::Int8,
        FS_GCS_DISABLED as f32,
    ),
    scalar(
        "GPS_HDOP_GOOD",
        K_PARAM_GPS_HDOP_GOOD,
        VarType::Int16,
        GPS_HDOP_GOOD_DEFAULT as f32,
    ),
    scalar("SUPER_SIMPLE", K_PARAM_SUPER_SIMPLE, VarType::Int8, 0.0),
    scalar(
        "WP_YAW_BEHAVIOR",
        K_PARAM_WP_YAW_BEHAVIOR,
        VarType::Int8,
        WP_YAW_BEHAVIOR_DEFAULT as f32,
    ),
    scalar(
        "FS_THR_ENABLE",
        K_PARAM_FAILSAFE_THROTTLE,
        VarType::Int8,
        FS_THR_ENABLED_ALWAYS_RTL as f32,
    ),
    scalar(
        "FS_THR_VALUE",
        K_PARAM_FAILSAFE_THROTTLE_VALUE,
        VarType::Int16,
        FS_THR_VALUE_DEFAULT as f32,
    ),
    scalar(
        "THR_DZ",
        K_PARAM_THROTTLE_DEADZONE,
        VarType::Int16,
        THR_DZ_DEFAULT as f32,
    ),
    scalar(
        "FLTMODE1",
        K_PARAM_FLIGHT_MODE1,
        VarType::Int8,
        FLIGHT_MODE_STABILIZE as f32,
    ),
    scalar(
        "FLTMODE2",
        K_PARAM_FLIGHT_MODE2,
        VarType::Int8,
        FLIGHT_MODE_STABILIZE as f32,
    ),
    scalar(
        "FLTMODE3",
        K_PARAM_FLIGHT_MODE3,
        VarType::Int8,
        FLIGHT_MODE_STABILIZE as f32,
    ),
    scalar(
        "FLTMODE4",
        K_PARAM_FLIGHT_MODE4,
        VarType::Int8,
        FLIGHT_MODE_STABILIZE as f32,
    ),
    scalar(
        "FLTMODE5",
        K_PARAM_FLIGHT_MODE5,
        VarType::Int8,
        FLIGHT_MODE_STABILIZE as f32,
    ),
    scalar(
        "FLTMODE6",
        K_PARAM_FLIGHT_MODE6,
        VarType::Int8,
        FLIGHT_MODE_STABILIZE as f32,
    ),
    scalar(
        "FLTMODE_CH",
        K_PARAM_FLIGHT_MODE_CHAN,
        VarType::Int8,
        CH_MODE_DEFAULT as f32,
    ),
    scalar(
        "INITIAL_MODE",
        K_PARAM_INITIAL_MODE,
        VarType::Int8,
        FLIGHT_MODE_STABILIZE as f32,
    ),
    scalar("SIMPLE", K_PARAM_SIMPLE_MODES, VarType::Int8, 0.0),
];

/// First leftover GSCALAR, `FORMAT_VERSION`.
#[must_use]
pub fn first_var_info_entry() -> Option<&'static VarInfoSpec> {
    FIRST_VAR_INFO.first()
}

/// Find a row in this leftover slice by `@Param` name.
#[must_use]
pub fn find_first_var(name: &str) -> Option<&'static VarInfoSpec> {
    FIRST_VAR_INFO.iter().find(|entry| entry.name == name)
}

/// Walk this leftover slice as `ParamInfo` rows.
pub fn for_each_first_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in FIRST_VAR_INFO {
        visit(entry.param_info());
    }
}
