//! Copter parameter-table leftover, upstream `ArduCopter/Parameters.cpp`.
//!
//! Tracked as **COP-023**. The first leftover is `Copter::var_info`
//! GSCALAR `FORMAT_VERSION` through `SIMPLE`. The next leftover is
//! `LOG_BITMASK` through the first `GOBJECT` (`ARMING_`). Keys come from
//! `Parameters::k_param_*`, not from table order: `SIMPLE` is 206 even
//! though it sits after `INITIAL_MODE` (208). `TUNE` is the
//! `AP_RC_TRANSMITTER_TUNING_ENABLED` leftover: it sits between
//! `ESC_CALIBRATION` and `FRAME_TYPE` on the stock table, but is not a
//! row of the `LOG_BITMASK` leftover. After `ARMING_` the next leftover
//! is `DISARM_DELAY` through the next `GOBJECT` (`CAM`). After `CAM`
//! the next leftover is the contiguous `RELAY` / `CHUTE_` / `LGR_`
//! `GOBJECT` group. After `LGR_` the next leftover is the stock
//! `COMPASS_` `GOBJECT` (heli `IM_` is a `FRAME_CONFIG` row, not a
//! Multi leftover). After `COMPASS_` the next leftover is the stock
//! `INS` `GOBJECT`. After `INS` the next leftover is the
//! `WP_` / `LOIT_` / `CIRCLE_` `GOBJECTPTR` group (`CIRCLE_` is
//! `MODE_CIRCLE_ENABLED`). After `CIRCLE_` the next leftover is the
//! stock `ATC_` `GOBJECT` (`GOBJECTVARPTR`). After `ATC_` the next leftover
//! is the stock `PSC` `GOBJECT` (`GOBJECTPTR`). After `PSC` the next leftover
//! is the stock `AHRS_` `GOBJECT`. After `AHRS_` the next leftover
//! is the stock `MNT` `GOBJECT` (`HAL_MOUNT_ENABLED`). After `MNT` the
//! next leftover is the stock `BATT` `GOBJECT`. After `BATT` the
//! next leftover is the stock `BRD_` `GOBJECT`. After `BRD_` the
//! next leftover is the stock `CAN_` `GOBJECT`
//! (`HAL_MAX_CAN_PROTOCOL_DRIVERS`). After `CAN_` the
//! next leftover is the stock `SPRAY_` `GOBJECT`
//! (`HAL_SPRAYER_ENABLED`). After `SPRAY_` the
//! next leftover is the stock `SIM_` `GOBJECT`
//! (`AP_SIM_ENABLED`). After `SIM_` the
//! next leftover is the stock `BARO` `GOBJECT`. After `BARO` the
//! next leftover is the stock `GPS` `GOBJECT`. After `GPS` the
//! next leftover is the stock `SCHED_` `GOBJECT`. After `SCHED_` the
//! next leftover is the stock `AVOID_` `GOBJECT`
//! (`AP_AVOIDANCE_ENABLED`). Later groups, G2,
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

/// `Parameters::k_param_log_bitmask_old`. Deprecated. Not `LOG_BITMASK`.
pub const K_PARAM_LOG_BITMASK_OLD: u16 = 20;

/// `Parameters::k_param_log_bitmask` — `LOG_BITMASK`.
pub const K_PARAM_LOG_BITMASK: u16 = 60;

/// `Parameters::k_param_esc_calibrate` — `ESC_CALIBRATION`.
pub const K_PARAM_ESC_CALIBRATE: u16 = 186;

/// `Parameters::k_param_frame_type` — `FRAME_TYPE`.
pub const K_PARAM_FRAME_TYPE: u16 = 149;

/// `Parameters::k_param_arming` — first `GOBJECT`, prefix `ARMING_`.
pub const K_PARAM_ARMING: u16 = 252;

/// `MASK_LOG_ATTITUDE_FAST`. Not in the Copter default.
pub const MASK_LOG_ATTITUDE_FAST: u32 = 1 << 0;

/// `MASK_LOG_ATTITUDE_MED`.
pub const MASK_LOG_ATTITUDE_MED: u32 = 1 << 1;

/// `MASK_LOG_GPS`.
pub const MASK_LOG_GPS: u32 = 1 << 2;

/// `MASK_LOG_PM`.
pub const MASK_LOG_PM: u32 = 1 << 3;

/// `MASK_LOG_CTUN`.
pub const MASK_LOG_CTUN: u32 = 1 << 4;

/// `MASK_LOG_NTUN`.
pub const MASK_LOG_NTUN: u32 = 1 << 5;

/// `MASK_LOG_RCIN`.
pub const MASK_LOG_RCIN: u32 = 1 << 6;

/// `MASK_LOG_IMU`.
pub const MASK_LOG_IMU: u32 = 1 << 7;

/// `MASK_LOG_CMD`.
pub const MASK_LOG_CMD: u32 = 1 << 8;

/// `MASK_LOG_CURRENT`.
pub const MASK_LOG_CURRENT: u32 = 1 << 9;

/// `MASK_LOG_RCOUT`.
pub const MASK_LOG_RCOUT: u32 = 1 << 10;

/// `MASK_LOG_OPTFLOW`.
pub const MASK_LOG_OPTFLOW: u32 = 1 << 11;

/// `MASK_LOG_PID`.
pub const MASK_LOG_PID: u32 = 1 << 12;

/// `MASK_LOG_COMPASS`.
pub const MASK_LOG_COMPASS: u32 = 1 << 13;

/// `MASK_LOG_INAV`. Deprecated. Not in the Copter default.
pub const MASK_LOG_INAV: u32 = 1 << 14;

/// `MASK_LOG_CAMERA`.
pub const MASK_LOG_CAMERA: u32 = 1 << 15;

/// `MASK_LOG_MOTBATT`.
pub const MASK_LOG_MOTBATT: u32 = 1 << 17;

/// `MASK_LOG_ANY`. Plane's `DEFAULT_LOG_BITMASK` (`0xffff`). Not Copter's.
pub const MASK_LOG_ANY: u32 = 0xFFFF;

/// Copter `DEFAULT_LOG_BITMASK` from `config.h`.
///
/// This is not Plane's `0xffff` and not `MASK_LOG_ANY`. Bit 0 (fast
/// attitude) is off; bit 17 (`MOTBATT`) is on.
pub const DEFAULT_LOG_BITMASK: u32 = MASK_LOG_ATTITUDE_MED
    | MASK_LOG_GPS
    | MASK_LOG_PM
    | MASK_LOG_CTUN
    | MASK_LOG_NTUN
    | MASK_LOG_RCIN
    | MASK_LOG_IMU
    | MASK_LOG_CMD
    | MASK_LOG_CURRENT
    | MASK_LOG_RCOUT
    | MASK_LOG_OPTFLOW
    | MASK_LOG_PID
    | MASK_LOG_COMPASS
    | MASK_LOG_CAMERA
    | MASK_LOG_MOTBATT;

/// `AP_Motors::MOTOR_FRAME_TYPE_PLUS`.
pub const MOTOR_FRAME_TYPE_PLUS: u8 = 0;

/// `AP_Motors::MOTOR_FRAME_TYPE_X` — stock `HAL_FRAME_TYPE_DEFAULT`.
pub const MOTOR_FRAME_TYPE_X: u8 = 1;

/// `HAL_FRAME_TYPE_DEFAULT` when the board does not override it.
pub const HAL_FRAME_TYPE_DEFAULT: u8 = MOTOR_FRAME_TYPE_X;

const fn group(name: &'static str, key: u16) -> VarInfoSpec {
    VarInfoSpec {
        name,
        key,
        ptype: VarType::Group,
        default: 0.0,
    }
}

/// `LOG_BITMASK` through the first `GOBJECT` leftover catalog.
///
/// Order is table order, not key order. `ESC_CALIBRATION` and
/// `FRAME_TYPE` sit between `LOG_BITMASK` and `ARMING_` on the stock
/// table. `TUNE` is a separate leftover: it is compiled only when
/// `AP_RC_TRANSMITTER_TUNING_ENABLED`.
pub const LOG_GOBJECT_VAR_INFO: &[VarInfoSpec] = &[
    scalar(
        "LOG_BITMASK",
        K_PARAM_LOG_BITMASK,
        VarType::Int32,
        DEFAULT_LOG_BITMASK as f32,
    ),
    scalar("ESC_CALIBRATION", K_PARAM_ESC_CALIBRATE, VarType::Int8, 0.0),
    scalar(
        "FRAME_TYPE",
        K_PARAM_FRAME_TYPE,
        VarType::Int8,
        HAL_FRAME_TYPE_DEFAULT as f32,
    ),
    group("ARMING_", K_PARAM_ARMING),
];

/// First row of the `LOG_BITMASK` leftover, `LOG_BITMASK`.
#[must_use]
pub fn log_gobject_var_info_entry() -> Option<&'static VarInfoSpec> {
    LOG_GOBJECT_VAR_INFO.first()
}

/// Find a row in the `LOG_BITMASK` leftover by `@Param` name or group prefix.
#[must_use]
pub fn find_log_gobject_var(name: &str) -> Option<&'static VarInfoSpec> {
    LOG_GOBJECT_VAR_INFO.iter().find(|entry| entry.name == name)
}

/// Walk the `LOG_BITMASK` leftover as `ParamInfo` rows.
pub fn for_each_log_gobject_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in LOG_GOBJECT_VAR_INFO {
        visit(entry.param_info());
    }
}

/// `Parameters::k_param_rc_tuning_param` — `TUNE`.
pub const K_PARAM_RC_TUNING_PARAM: u16 = 187;

/// `Parameters::k_param_rc_tuning_param_high_old`. Unused. Not `TUNE`.
pub const K_PARAM_RC_TUNING_PARAM_HIGH_OLD: u16 = 188;

/// `Parameters::k_param_rc_tuning_param_low_old`. Unused. Not `TUNE`.
pub const K_PARAM_RC_TUNING_PARAM_LOW_OLD: u16 = 189;

/// Stock `TUNE` default — no transmitter knob selected.
pub const TUNE_NONE: u8 = 0;

/// `TUNE` leftover catalog.
///
/// Compiled only when `AP_RC_TRANSMITTER_TUNING_ENABLED` (defaults to
/// `AP_RC_CHANNEL_ENABLED`, which is 1). Sits between `ESC_CALIBRATION`
/// and `FRAME_TYPE` on the stock table, but is not a row of the
/// `LOG_BITMASK` leftover. `TUNE_MIN` / `TUNE_MAX` live in G2.
pub const TUNE_VAR_INFO: &[VarInfoSpec] =
    &[scalar("TUNE", K_PARAM_RC_TUNING_PARAM, VarType::Int8, 0.0)];

/// First (only) row of the `TUNE` leftover.
#[must_use]
pub fn tune_var_info_entry() -> Option<&'static VarInfoSpec> {
    TUNE_VAR_INFO.first()
}

/// Find a row in the `TUNE` leftover by `@Param` name.
#[must_use]
pub fn find_tune_var(name: &str) -> Option<&'static VarInfoSpec> {
    TUNE_VAR_INFO.iter().find(|entry| entry.name == name)
}

/// Walk the `TUNE` leftover as `ParamInfo` rows.
pub fn for_each_tune_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in TUNE_VAR_INFO {
        visit(entry.param_info());
    }
}

/// `Parameters::k_param_disarm_delay` — `DISARM_DELAY`.
pub const K_PARAM_DISARM_DELAY: u16 = 91;

/// `Parameters::k_param_poshold_brake_rate_degs` — `PHLD_BRK_RATE`.
pub const K_PARAM_POSHOLD_BRAKE_RATE_DEGS: u16 = 46;

/// `Parameters::k_param_land_repositioning` — `LAND_REPOSITION`.
pub const K_PARAM_LAND_REPOSITIONING: u16 = 52;

/// `Parameters::k_param_fs_ekf_action` — `FS_EKF_ACTION`.
pub const K_PARAM_FS_EKF_ACTION: u16 = 248;

/// `Parameters::k_param_fs_ekf_thresh` — `FS_EKF_THRESH`.
pub const K_PARAM_FS_EKF_THRESH: u16 = 54;

/// `Parameters::k_param_fs_crash_check` — `FS_CRASH_CHECK`.
pub const K_PARAM_FS_CRASH_CHECK: u16 = 92;

/// `Parameters::k_param_rc_speed` — `RC_SPEED`.
pub const K_PARAM_RC_SPEED: u16 = 192;

/// `Parameters::k_param_acro_balance_roll` — `ACRO_BAL_ROLL`.
pub const K_PARAM_ACRO_BALANCE_ROLL: u16 = 242;

/// `Parameters::k_param_acro_balance_pitch` — `ACRO_BAL_PITCH`.
pub const K_PARAM_ACRO_BALANCE_PITCH: u16 = 243;

/// `Parameters::k_param_acro_trainer` — `ACRO_TRAINER`.
pub const K_PARAM_ACRO_TRAINER: u16 = 27;

/// `Parameters::k_param_camera` — next `GOBJECT`, prefix `CAM`.
pub const K_PARAM_CAMERA: u16 = 165;

/// `AUTO_DISARMING_DELAY` from Copter `config.h`, seconds.
pub const AUTO_DISARMING_DELAY: u8 = 10;

/// Multicopter `POSHOLD_BRAKE_RATE_DEFAULT` from Copter `config.h`.
pub const POSHOLD_BRAKE_RATE_DEFAULT: i16 = 8;

/// Tradheli `POSHOLD_BRAKE_RATE_DEFAULT`. Not this leftover.
pub const POSHOLD_BRAKE_RATE_HELI: i16 = 4;

/// `LAND_REPOSITION_DEFAULT` from Copter `config.h`.
pub const LAND_REPOSITION_DEFAULT: u8 = 1;

/// `FS_EKF_ACTION_REPORT_ONLY`.
pub const FS_EKF_ACTION_REPORT_ONLY: u8 = 0;

/// `FS_EKF_ACTION_LAND` — stock `FS_EKF_ACTION` default.
pub const FS_EKF_ACTION_LAND: u8 = 1;

/// `FS_EKF_ACTION_ALTHOLD`.
pub const FS_EKF_ACTION_ALTHOLD: u8 = 2;

/// `FS_EKF_ACTION_LAND_EVEN_STABILIZE`.
pub const FS_EKF_ACTION_LAND_EVEN_STABILIZE: u8 = 3;

/// `FS_EKF_ACTION_DEFAULT` from Copter `config.h`.
pub const FS_EKF_ACTION_DEFAULT: u8 = FS_EKF_ACTION_LAND;

/// `FS_EKF_THRESHOLD_DEFAULT` from Copter `config.h`.
pub const FS_EKF_THRESHOLD_DEFAULT: f32 = 0.8;

/// Multicopter `RC_FAST_SPEED` from Copter `config.h`.
pub const RC_FAST_SPEED: i16 = 490;

/// Tradheli `RC_FAST_SPEED`. Not this leftover.
pub const RC_FAST_SPEED_HELI: i16 = 125;

/// `ACRO_BALANCE_ROLL` from Copter `config.h`.
pub const ACRO_BALANCE_ROLL: f32 = 1.0;

/// `ACRO_BALANCE_PITCH` from Copter `config.h`.
pub const ACRO_BALANCE_PITCH: f32 = 1.0;

/// `ModeAcro::Trainer::OFF`.
pub const ACRO_TRAINER_OFF: u8 = 0;

/// `ModeAcro::Trainer::LEVELING`.
pub const ACRO_TRAINER_LEVELING: u8 = 1;

/// `ModeAcro::Trainer::LIMITED` — stock `ACRO_TRAINER` default.
pub const ACRO_TRAINER_LIMITED: u8 = 2;

/// `DISARM_DELAY` through the next `GOBJECT` leftover catalog.
///
/// Order is table order, not key order. `PHLD_BRK_RATE` is
/// `MODE_POSHOLD_ENABLED`, the ACRO balance / trainer rows are
/// `MODE_ACRO_ENABLED` (sport alone is SITL-only), and `CAM` is
/// `AP_CAMERA_ENABLED`. A stock multicopter compiles all of them.
/// Heli `RC_FAST_SPEED` (125) and heli `POSHOLD_BRAKE_RATE` (4) are
/// `FRAME_CONFIG` rewrites, not rows. `RELAY` stays later.
pub const DISARM_GOBJECT_VAR_INFO: &[VarInfoSpec] = &[
    scalar(
        "DISARM_DELAY",
        K_PARAM_DISARM_DELAY,
        VarType::Int8,
        AUTO_DISARMING_DELAY as f32,
    ),
    scalar(
        "PHLD_BRK_RATE",
        K_PARAM_POSHOLD_BRAKE_RATE_DEGS,
        VarType::Int16,
        POSHOLD_BRAKE_RATE_DEFAULT as f32,
    ),
    scalar(
        "LAND_REPOSITION",
        K_PARAM_LAND_REPOSITIONING,
        VarType::Int8,
        LAND_REPOSITION_DEFAULT as f32,
    ),
    scalar(
        "FS_EKF_ACTION",
        K_PARAM_FS_EKF_ACTION,
        VarType::Int8,
        FS_EKF_ACTION_DEFAULT as f32,
    ),
    scalar(
        "FS_EKF_THRESH",
        K_PARAM_FS_EKF_THRESH,
        VarType::Float,
        FS_EKF_THRESHOLD_DEFAULT,
    ),
    scalar("FS_CRASH_CHECK", K_PARAM_FS_CRASH_CHECK, VarType::Int8, 1.0),
    scalar(
        "RC_SPEED",
        K_PARAM_RC_SPEED,
        VarType::Int16,
        RC_FAST_SPEED as f32,
    ),
    scalar(
        "ACRO_BAL_ROLL",
        K_PARAM_ACRO_BALANCE_ROLL,
        VarType::Float,
        ACRO_BALANCE_ROLL,
    ),
    scalar(
        "ACRO_BAL_PITCH",
        K_PARAM_ACRO_BALANCE_PITCH,
        VarType::Float,
        ACRO_BALANCE_PITCH,
    ),
    scalar(
        "ACRO_TRAINER",
        K_PARAM_ACRO_TRAINER,
        VarType::Int8,
        ACRO_TRAINER_LIMITED as f32,
    ),
    group("CAM", K_PARAM_CAMERA),
];

/// First row of the `DISARM_DELAY` leftover, `DISARM_DELAY`.
#[must_use]
pub fn disarm_gobject_var_info_entry() -> Option<&'static VarInfoSpec> {
    DISARM_GOBJECT_VAR_INFO.first()
}

/// Find a row in the `DISARM_DELAY` leftover by `@Param` name or group prefix.
#[must_use]
pub fn find_disarm_gobject_var(name: &str) -> Option<&'static VarInfoSpec> {
    DISARM_GOBJECT_VAR_INFO
        .iter()
        .find(|entry| entry.name == name)
}

/// Walk the `DISARM_DELAY` leftover as `ParamInfo` rows.
pub fn for_each_disarm_gobject_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in DISARM_GOBJECT_VAR_INFO {
        visit(entry.param_info());
    }
}

/// `Parameters::k_param_relay` — next `GOBJECT`, prefix `RELAY`.
pub const K_PARAM_RELAY: u16 = 13;

/// `Parameters::k_param_epm_unused`. Unused. Not `RELAY`.
pub const K_PARAM_EPM_UNUSED: u16 = 14;

/// `Parameters::k_param_parachute` — `CHUTE_`.
pub const K_PARAM_PARACHUTE: u16 = 17;

/// `Parameters::k_param_landinggear` — `LGR_`.
pub const K_PARAM_LANDINGGEAR: u16 = 18;

/// `Parameters::k_param_input_manager` — heli `IM_`. Not this leftover.
pub const K_PARAM_INPUT_MANAGER: u16 = 19;

/// `RELAY` through `LGR_` leftover catalog.
///
/// Order is table order, not key order. `RELAY` is `AP_RELAY_ENABLED`,
/// `CHUTE_` is `HAL_PARACHUTE_ENABLED`, and `LGR_` is
/// `AP_LANDINGGEAR_ENABLED`. A stock multicopter compiles all three.
/// Heli `IM_` is a `FRAME_CONFIG` row, not this leftover.
pub const RELAY_GOBJECT_VAR_INFO: &[VarInfoSpec] = &[
    group("RELAY", K_PARAM_RELAY),
    group("CHUTE_", K_PARAM_PARACHUTE),
    group("LGR_", K_PARAM_LANDINGGEAR),
];

/// First row of the `RELAY` leftover, `RELAY`.
#[must_use]
pub fn relay_gobject_var_info_entry() -> Option<&'static VarInfoSpec> {
    RELAY_GOBJECT_VAR_INFO.first()
}

/// Find a row in the `RELAY` leftover by group prefix.
#[must_use]
pub fn find_relay_gobject_var(name: &str) -> Option<&'static VarInfoSpec> {
    RELAY_GOBJECT_VAR_INFO
        .iter()
        .find(|entry| entry.name == name)
}

/// Walk the `RELAY` leftover as `ParamInfo` rows.
pub fn for_each_relay_gobject_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in RELAY_GOBJECT_VAR_INFO {
        visit(entry.param_info());
    }
}

/// `Parameters::k_param_compass_enabled_deprecated`. Unused. Not `COMPASS_`.
pub const K_PARAM_COMPASS_ENABLED_DEPRECATED: u16 = 146;

/// `Parameters::k_param_compass` — next `GOBJECT`, prefix `COMPASS_`.
pub const K_PARAM_COMPASS: u16 = 147;

/// `Parameters::k_param_ins_old`. Deprecated. Not `INS`.
pub const K_PARAM_INS_OLD: u16 = 2;

/// `Parameters::k_param_ins` — next `GOBJECT`, prefix `INS`.
pub const K_PARAM_INS: u16 = 3;

/// Stock `COMPASS_` leftover catalog.
///
/// The next contiguous Multi `GOBJECT` after `LGR_`. Heli `IM_` sits
/// between them on a tradheli build (`FRAME_CONFIG == HELI_FRAME`) and
/// is not a row of this leftover. Nested `AP_Compass` `var_info` is
/// not this leftover. `INS` stays later.
pub const COMPASS_GOBJECT_VAR_INFO: &[VarInfoSpec] = &[group("COMPASS_", K_PARAM_COMPASS)];

/// First (only) row of the `COMPASS_` leftover.
#[must_use]
pub fn compass_gobject_var_info_entry() -> Option<&'static VarInfoSpec> {
    COMPASS_GOBJECT_VAR_INFO.first()
}

/// Find a row in the `COMPASS_` leftover by group prefix.
#[must_use]
pub fn find_compass_gobject_var(name: &str) -> Option<&'static VarInfoSpec> {
    COMPASS_GOBJECT_VAR_INFO
        .iter()
        .find(|entry| entry.name == name)
}

/// Walk the `COMPASS_` leftover as `ParamInfo` rows.
pub fn for_each_compass_gobject_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in COMPASS_GOBJECT_VAR_INFO {
        visit(entry.param_info());
    }
}

/// `Parameters::k_param_inertial_nav`. Deprecated. Not `WP_`.
pub const K_PARAM_INERTIAL_NAV: u16 = 100;

/// `Parameters::k_param_wp_nav` — next `GOBJECTPTR`, prefix `WP_`.
pub const K_PARAM_WP_NAV: u16 = 101;

/// `Parameters::k_param_loiter_nav` — `LOIT_`.
pub const K_PARAM_LOITER_NAV: u16 = 105;

/// `Parameters::k_param_circle_nav` — `CIRCLE_`.
pub const K_PARAM_CIRCLE_NAV: u16 = 104;

/// Stock `INS` leftover catalog.
///
/// The next contiguous Multi `GOBJECT` after `COMPASS_`. Nested
/// `AP_InertialSensor` `var_info` is not this leftover. `WP_` /
/// `LOIT_` / `CIRCLE_` stay later.
pub const INS_GOBJECT_VAR_INFO: &[VarInfoSpec] = &[group("INS", K_PARAM_INS)];

/// First (only) row of the `INS` leftover.
#[must_use]
pub fn ins_gobject_var_info_entry() -> Option<&'static VarInfoSpec> {
    INS_GOBJECT_VAR_INFO.first()
}

/// Find a row in the `INS` leftover by group prefix.
#[must_use]
pub fn find_ins_gobject_var(name: &str) -> Option<&'static VarInfoSpec> {
    INS_GOBJECT_VAR_INFO.iter().find(|entry| entry.name == name)
}

/// Walk the `INS` leftover as `ParamInfo` rows.
pub fn for_each_ins_gobject_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in INS_GOBJECT_VAR_INFO {
        visit(entry.param_info());
    }
}

/// `Parameters::k_param_attitude_control` — next `GOBJECT`, prefix `ATC_`.
pub const K_PARAM_ATTITUDE_CONTROL: u16 = 102;

/// Stock `WP_` / `LOIT_` / `CIRCLE_` leftover catalog.
///
/// The next contiguous Multi `GOBJECTPTR` group after `INS`. Order is
/// table order, not key order. `CIRCLE_` is `MODE_CIRCLE_ENABLED` and
/// a stock multicopter compiles it in. Nested `AC_WPNav` / `AC_Loiter`
/// / `AC_Circle` `var_info` is not this leftover. `ATC_` stays later.
pub const WP_LOIT_CIRCLE_GOBJECT_VAR_INFO: &[VarInfoSpec] = &[
    group("WP_", K_PARAM_WP_NAV),
    group("LOIT_", K_PARAM_LOITER_NAV),
    group("CIRCLE_", K_PARAM_CIRCLE_NAV),
];

/// First row of the `WP_` leftover, `WP_`.
#[must_use]
pub fn wp_loit_circle_gobject_var_info_entry() -> Option<&'static VarInfoSpec> {
    WP_LOIT_CIRCLE_GOBJECT_VAR_INFO.first()
}

/// Find a row in the `WP_` leftover by group prefix.
#[must_use]
pub fn find_wp_loit_circle_gobject_var(name: &str) -> Option<&'static VarInfoSpec> {
    WP_LOIT_CIRCLE_GOBJECT_VAR_INFO
        .iter()
        .find(|entry| entry.name == name)
}

/// Walk the `WP_` leftover as `ParamInfo` rows.
pub fn for_each_wp_loit_circle_gobject_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in WP_LOIT_CIRCLE_GOBJECT_VAR_INFO {
        visit(entry.param_info());
    }
}

/// `Parameters::k_param_pos_control` — next `GOBJECT`, prefix `PSC`.
pub const K_PARAM_POS_CONTROL: u16 = 103;

/// Stock `ATC_` leftover catalog.
///
/// The next contiguous Multi `GOBJECT` after `CIRCLE_`. Upstream is
/// `GOBJECTVARPTR`. Nested `AC_AttitudeControl` `var_info` is not this
/// leftover. `PSC` stays later. Heli `IM_` is not a row of this leftover.
pub const ATC_GOBJECT_VAR_INFO: &[VarInfoSpec] = &[group("ATC_", K_PARAM_ATTITUDE_CONTROL)];

/// First (only) row of the `ATC_` leftover.
#[must_use]
pub fn atc_gobject_var_info_entry() -> Option<&'static VarInfoSpec> {
    ATC_GOBJECT_VAR_INFO.first()
}

/// Find a row in the `ATC_` leftover by group prefix.
#[must_use]
pub fn find_atc_gobject_var(name: &str) -> Option<&'static VarInfoSpec> {
    ATC_GOBJECT_VAR_INFO.iter().find(|entry| entry.name == name)
}

/// Walk the `ATC_` leftover as `ParamInfo` rows.
pub fn for_each_atc_gobject_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in ATC_GOBJECT_VAR_INFO {
        visit(entry.param_info());
    }
}

/// `Parameters::k_param_ahrs` — next `GOBJECT`, prefix `AHRS_`.
pub const K_PARAM_AHRS: u16 = 159;

/// Stock `PSC` leftover catalog.
///
/// The next contiguous Multi `GOBJECT` after `ATC_`. Upstream is
/// `GOBJECTPTR`. Nested `AC_PosControl` `var_info` is not this leftover.
/// `AHRS_` stays later. Heli `IM_` is not a row of this leftover.
pub const PSC_GOBJECT_VAR_INFO: &[VarInfoSpec] = &[group("PSC", K_PARAM_POS_CONTROL)];

/// First (only) row of the `PSC` leftover.
#[must_use]
pub fn psc_gobject_var_info_entry() -> Option<&'static VarInfoSpec> {
    PSC_GOBJECT_VAR_INFO.first()
}

/// Find a row in the `PSC` leftover by group prefix.
#[must_use]
pub fn find_psc_gobject_var(name: &str) -> Option<&'static VarInfoSpec> {
    PSC_GOBJECT_VAR_INFO.iter().find(|entry| entry.name == name)
}

/// Walk the `PSC` leftover as `ParamInfo` rows.
pub fn for_each_psc_gobject_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in PSC_GOBJECT_VAR_INFO {
        visit(entry.param_info());
    }
}

/// `Parameters::k_param_camera_mount` — next `GOBJECT`, prefix `MNT`.
pub const K_PARAM_CAMERA_MOUNT: u16 = 166;

/// Stock `AHRS_` leftover catalog.
///
/// The next contiguous Multi `GOBJECT` after `PSC`. Nested `AP_AHRS`
/// `var_info` is not this leftover. Later groups, G2, and
/// `load_parameters` stay later. Heli `IM_` is not a row of this leftover.
pub const AHRS_GOBJECT_VAR_INFO: &[VarInfoSpec] = &[group("AHRS_", K_PARAM_AHRS)];

/// First (only) row of the `AHRS_` leftover.
#[must_use]
pub fn ahrs_gobject_var_info_entry() -> Option<&'static VarInfoSpec> {
    AHRS_GOBJECT_VAR_INFO.first()
}

/// Find a row in the `AHRS_` leftover by group prefix.
#[must_use]
pub fn find_ahrs_gobject_var(name: &str) -> Option<&'static VarInfoSpec> {
    AHRS_GOBJECT_VAR_INFO
        .iter()
        .find(|entry| entry.name == name)
}

/// Walk the `AHRS_` leftover as `ParamInfo` rows.
pub fn for_each_ahrs_gobject_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in AHRS_GOBJECT_VAR_INFO {
        visit(entry.param_info());
    }
}

/// `Parameters::k_param_battery` — next `GOBJECT`, prefix `BATT`.
pub const K_PARAM_BATTERY: u16 = 36;

/// Stock `MNT` leftover catalog.
///
/// The next contiguous Multi `GOBJECT` after `AHRS_`. `MNT` is
/// `HAL_MOUNT_ENABLED` and a stock multicopter compiles it in. Nested
/// `AP_Mount` `var_info` is not this leftover. Later groups, G2, and
/// `load_parameters` stay later. Heli `IM_` is not a row of this leftover.
pub const MOUNT_GOBJECT_VAR_INFO: &[VarInfoSpec] = &[group("MNT", K_PARAM_CAMERA_MOUNT)];

/// First (only) row of the `MNT` leftover.
#[must_use]
pub fn mount_gobject_var_info_entry() -> Option<&'static VarInfoSpec> {
    MOUNT_GOBJECT_VAR_INFO.first()
}

/// Find a row in the `MNT` leftover by group prefix.
#[must_use]
pub fn find_mount_gobject_var(name: &str) -> Option<&'static VarInfoSpec> {
    MOUNT_GOBJECT_VAR_INFO
        .iter()
        .find(|entry| entry.name == name)
}

/// Walk the `MNT` leftover as `ParamInfo` rows.
pub fn for_each_mount_gobject_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in MOUNT_GOBJECT_VAR_INFO {
        visit(entry.param_info());
    }
}

/// `Parameters::k_param_BoardConfig` — next `GOBJECT`, prefix `BRD_`.
pub const K_PARAM_BOARDCONFIG: u16 = 15;

/// Stock `BATT` leftover catalog.
///
/// The next contiguous Multi `GOBJECT` after `MNT`. Nested
/// `AP_BattMonitor` `var_info` is not this leftover. `BRD_` stays later.
/// Heli `IM_` is not a row of this leftover.
pub const BATT_GOBJECT_VAR_INFO: &[VarInfoSpec] = &[group("BATT", K_PARAM_BATTERY)];

/// First (only) row of the `BATT` leftover.
#[must_use]
pub fn batt_gobject_var_info_entry() -> Option<&'static VarInfoSpec> {
    BATT_GOBJECT_VAR_INFO.first()
}

/// Find a row in the `BATT` leftover by group prefix.
#[must_use]
pub fn find_batt_gobject_var(name: &str) -> Option<&'static VarInfoSpec> {
    BATT_GOBJECT_VAR_INFO
        .iter()
        .find(|entry| entry.name == name)
}

/// Walk the `BATT` leftover as `ParamInfo` rows.
pub fn for_each_batt_gobject_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in BATT_GOBJECT_VAR_INFO {
        visit(entry.param_info());
    }
}

/// `Parameters::k_param_can_mgr` — next `GOBJECT`, prefix `CAN_`.
pub const K_PARAM_CAN_MGR: u16 = 8;

/// Stock `BRD_` leftover catalog.
///
/// The next contiguous Multi `GOBJECT` after `BATT`. Nested
/// `AP_BoardConfig` `var_info` is not this leftover. `CAN_` stays later.
/// Heli `IM_` is not a row of this leftover.
pub const BRD_GOBJECT_VAR_INFO: &[VarInfoSpec] = &[group("BRD_", K_PARAM_BOARDCONFIG)];

/// First (only) row of the `BRD_` leftover.
#[must_use]
pub fn brd_gobject_var_info_entry() -> Option<&'static VarInfoSpec> {
    BRD_GOBJECT_VAR_INFO.first()
}

/// Find a row in the `BRD_` leftover by group prefix.
#[must_use]
pub fn find_brd_gobject_var(name: &str) -> Option<&'static VarInfoSpec> {
    BRD_GOBJECT_VAR_INFO.iter().find(|entry| entry.name == name)
}

/// Walk the `BRD_` leftover as `ParamInfo` rows.
pub fn for_each_brd_gobject_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in BRD_GOBJECT_VAR_INFO {
        visit(entry.param_info());
    }
}

/// `Parameters::k_param_sprayer` — next `GOBJECT`, prefix `SPRAY_`.
pub const K_PARAM_SPRAYER: u16 = 33;

/// Stock `CAN_` leftover catalog.
///
/// The next contiguous Multi `GOBJECT` after `BRD_`. `CAN_` is
/// `HAL_MAX_CAN_PROTOCOL_DRIVERS` and a stock multicopter compiles it in. Nested
/// `AP_CANManager` `var_info` is not this leftover. `SPRAY_` stays later.
/// Heli `IM_` is not a row of this leftover.
pub const CAN_GOBJECT_VAR_INFO: &[VarInfoSpec] = &[group("CAN_", K_PARAM_CAN_MGR)];

/// First (only) row of the `CAN_` leftover.
#[must_use]
pub fn can_gobject_var_info_entry() -> Option<&'static VarInfoSpec> {
    CAN_GOBJECT_VAR_INFO.first()
}

/// Find a row in the `CAN_` leftover by group prefix.
#[must_use]
pub fn find_can_gobject_var(name: &str) -> Option<&'static VarInfoSpec> {
    CAN_GOBJECT_VAR_INFO.iter().find(|entry| entry.name == name)
}

/// Walk the `CAN_` leftover as `ParamInfo` rows.
pub fn for_each_can_gobject_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in CAN_GOBJECT_VAR_INFO {
        visit(entry.param_info());
    }
}

/// `Parameters::k_param_sitl` — next `GOBJECT`, prefix `SIM_`.
pub const K_PARAM_SITL: u16 = 10;

/// Stock `SPRAY_` leftover catalog.
///
/// The next contiguous Multi `GOBJECT` after `CAN_`. `SPRAY_` is
/// `HAL_SPRAYER_ENABLED` and a stock multicopter compiles it in. Nested
/// `AC_Sprayer` `var_info` is not this leftover. `SIM_` stays later.
/// Heli `IM_` is not a row of this leftover.
pub const SPRAY_GOBJECT_VAR_INFO: &[VarInfoSpec] = &[group("SPRAY_", K_PARAM_SPRAYER)];

/// First (only) row of the `SPRAY_` leftover.
#[must_use]
pub fn spray_gobject_var_info_entry() -> Option<&'static VarInfoSpec> {
    SPRAY_GOBJECT_VAR_INFO.first()
}

/// Find a row in the `SPRAY_` leftover by group prefix.
#[must_use]
pub fn find_spray_gobject_var(name: &str) -> Option<&'static VarInfoSpec> {
    SPRAY_GOBJECT_VAR_INFO
        .iter()
        .find(|entry| entry.name == name)
}

/// Walk the `SPRAY_` leftover as `ParamInfo` rows.
pub fn for_each_spray_gobject_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in SPRAY_GOBJECT_VAR_INFO {
        visit(entry.param_info());
    }
}

/// `Parameters::k_param_barometer` — next `GOBJECT`, prefix `BARO`.
pub const K_PARAM_BAROMETER: u16 = 11;

/// Stock `SIM_` leftover catalog.
///
/// The next contiguous Multi `GOBJECT` after `SPRAY_`. `SIM_` is
/// `AP_SIM_ENABLED` and a stock multicopter compiles it in. Nested
/// `SITL::SIM` `var_info` is not this leftover. `BARO` stays later.
/// Heli `IM_` is not a row of this leftover.
pub const SIM_GOBJECT_VAR_INFO: &[VarInfoSpec] = &[group("SIM_", K_PARAM_SITL)];

/// First (only) row of the `SIM_` leftover.
#[must_use]
pub fn sim_gobject_var_info_entry() -> Option<&'static VarInfoSpec> {
    SIM_GOBJECT_VAR_INFO.first()
}

/// Find a row in the `SIM_` leftover by group prefix.
#[must_use]
pub fn find_sim_gobject_var(name: &str) -> Option<&'static VarInfoSpec> {
    SIM_GOBJECT_VAR_INFO.iter().find(|entry| entry.name == name)
}

/// Walk the `SIM_` leftover as `ParamInfo` rows.
pub fn for_each_sim_gobject_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in SIM_GOBJECT_VAR_INFO {
        visit(entry.param_info());
    }
}

/// `Parameters::k_param_gps` — next `GOBJECT`, prefix `GPS`.
pub const K_PARAM_GPS: u16 = 16;

/// Stock `BARO` leftover catalog.
///
/// The next contiguous Multi `GOBJECT` after `SIM_`. Nested
/// `AP_Baro` `var_info` is not this leftover. `GPS` stays later.
/// Heli `IM_` is not a row of this leftover.
pub const BARO_GOBJECT_VAR_INFO: &[VarInfoSpec] = &[group("BARO", K_PARAM_BAROMETER)];

/// First (only) row of the `BARO` leftover.
#[must_use]
pub fn baro_gobject_var_info_entry() -> Option<&'static VarInfoSpec> {
    BARO_GOBJECT_VAR_INFO.first()
}

/// Find a row in the `BARO` leftover by group prefix.
#[must_use]
pub fn find_baro_gobject_var(name: &str) -> Option<&'static VarInfoSpec> {
    BARO_GOBJECT_VAR_INFO
        .iter()
        .find(|entry| entry.name == name)
}

/// Walk the `BARO` leftover as `ParamInfo` rows.
pub fn for_each_baro_gobject_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in BARO_GOBJECT_VAR_INFO {
        visit(entry.param_info());
    }
}

/// `Parameters::k_param_scheduler` — next `GOBJECT`, prefix `SCHED_`.
pub const K_PARAM_SCHEDULER: u16 = 12;

/// Stock `GPS` leftover catalog.
///
/// The next contiguous Multi `GOBJECT` after `BARO`. Nested
/// `AP_GPS` `var_info` is not this leftover. `SCHED_` stays later.
/// Heli `IM_` is not a row of this leftover.
pub const GPS_GOBJECT_VAR_INFO: &[VarInfoSpec] = &[group("GPS", K_PARAM_GPS)];

/// First (only) row of the `GPS` leftover.
#[must_use]
pub fn gps_gobject_var_info_entry() -> Option<&'static VarInfoSpec> {
    GPS_GOBJECT_VAR_INFO.first()
}

/// Find a row in the `GPS` leftover by group prefix.
#[must_use]
pub fn find_gps_gobject_var(name: &str) -> Option<&'static VarInfoSpec> {
    GPS_GOBJECT_VAR_INFO.iter().find(|entry| entry.name == name)
}

/// Walk the `GPS` leftover as `ParamInfo` rows.
pub fn for_each_gps_gobject_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in GPS_GOBJECT_VAR_INFO {
        visit(entry.param_info());
    }
}

/// `Parameters::k_param_avoid` — next `GOBJECT`, prefix `AVOID_`.
pub const K_PARAM_AVOID: u16 = 95;

/// Stock `SCHED_` leftover catalog.
///
/// The next contiguous Multi `GOBJECT` after `GPS`. Nested
/// `AP_Scheduler` `var_info` is not this leftover. `AVOID_` stays later.
/// Heli `IM_` is not a row of this leftover.
pub const SCHED_GOBJECT_VAR_INFO: &[VarInfoSpec] = &[group("SCHED_", K_PARAM_SCHEDULER)];

/// First (only) row of the `SCHED_` leftover.
#[must_use]
pub fn sched_gobject_var_info_entry() -> Option<&'static VarInfoSpec> {
    SCHED_GOBJECT_VAR_INFO.first()
}

/// Find a row in the `SCHED_` leftover by group prefix.
#[must_use]
pub fn find_sched_gobject_var(name: &str) -> Option<&'static VarInfoSpec> {
    SCHED_GOBJECT_VAR_INFO
        .iter()
        .find(|entry| entry.name == name)
}

/// Walk the `SCHED_` leftover as `ParamInfo` rows.
pub fn for_each_sched_gobject_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in SCHED_GOBJECT_VAR_INFO {
        visit(entry.param_info());
    }
}

/// `Parameters::k_param_rally` — next leftover after `AVOID_`. Not this leftover.
pub const K_PARAM_RALLY: u16 = 45;

/// Stock `AVOID_` leftover catalog.
///
/// The next contiguous Multi `GOBJECT` after `SCHED_`. `AVOID_` is
/// `AP_AVOIDANCE_ENABLED` and a stock multicopter compiles it in. Nested
/// `AC_Avoid` `var_info` is not this leftover. `RALLY_` stays later.
/// Heli `IM_` is not a row of this leftover.
pub const AVOID_GOBJECT_VAR_INFO: &[VarInfoSpec] = &[group("AVOID_", K_PARAM_AVOID)];

/// First (only) row of the `AVOID_` leftover.
#[must_use]
pub fn avoid_gobject_var_info_entry() -> Option<&'static VarInfoSpec> {
    AVOID_GOBJECT_VAR_INFO.first()
}

/// Find a row in the `AVOID_` leftover by group prefix.
#[must_use]
pub fn find_avoid_gobject_var(name: &str) -> Option<&'static VarInfoSpec> {
    AVOID_GOBJECT_VAR_INFO
        .iter()
        .find(|entry| entry.name == name)
}

/// Walk the `AVOID_` leftover as `ParamInfo` rows.
pub fn for_each_avoid_gobject_param_info(visit: &mut dyn FnMut(ParamInfo<'static>)) {
    for entry in AVOID_GOBJECT_VAR_INFO {
        visit(entry.param_info());
    }
}
