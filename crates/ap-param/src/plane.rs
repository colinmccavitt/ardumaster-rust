//! Plane parameter conversion tables, upstream `ArduPlane/Parameters.cpp`.

use crate::conversion::{ConvertFlags, G2ObjectConversionEntry, NamedParameterMigration, RcOptionConversion};
use crate::VarType;

/// Top-level key for `ParametersG2`, upstream `k_param_g2`.
pub const PLANE_G2_OLD_KEY: u16 = 4;

/// Old top-level fence parameters renamed into the `FENCE` group, upstream
/// `conversion_table` in `Parameters.cpp`.
pub const PLANE_FENCE_CONVERSIONS: &[NamedParameterMigration] = &[
    NamedParameterMigration {
        old_key: 228, // k_param_fence_minalt
        old_group_element: 0,
        old_type: VarType::Int16,
        new_name: "FENCE_ALT_MIN",
        scaler: 1.0,
        flags: ConvertFlags::NONE,
    },
    NamedParameterMigration {
        old_key: 229, // k_param_fence_maxalt
        old_group_element: 0,
        old_type: VarType::Int16,
        new_name: "FENCE_ALT_MAX",
        scaler: 1.0,
        flags: ConvertFlags::NONE,
    },
    NamedParameterMigration {
        old_key: 105, // k_param_fence_retalt
        old_group_element: 0,
        old_type: VarType::Int16,
        new_name: "FENCE_RET_ALT",
        scaler: 1.0,
        flags: ConvertFlags::NONE,
    },
    NamedParameterMigration {
        old_key: 107, // k_param_fence_ret_rally
        old_group_element: 0,
        old_type: VarType::Int8,
        new_name: "FENCE_RET_RALLY",
        scaler: 1.0,
        flags: ConvertFlags::NONE,
    },
    NamedParameterMigration {
        old_key: 106, // k_param_fence_autoenable
        old_group_element: 0,
        old_type: VarType::Int8,
        new_name: "FENCE_AUTOENABLE",
        scaler: 1.0,
        flags: ConvertFlags::NONE,
    },
];

/// Old telemetry/sysid parameters renamed into the `MAV_` namespace, upstream
/// `gcs_conversion_info` in `Parameters.cpp` (Mar-2025, ArduPilot-4.7).
pub const PLANE_GCS_CONVERSIONS: &[NamedParameterMigration] = &[
    NamedParameterMigration {
        old_key: 112, // k_param_sysid_this_mav_old
        old_group_element: 0,
        old_type: VarType::Int16,
        new_name: "MAV_SYSID",
        scaler: 1.0,
        flags: ConvertFlags::NONE,
    },
    NamedParameterMigration {
        old_key: 113, // k_param_sysid_my_gcs_old
        old_group_element: 0,
        old_type: VarType::Int16,
        new_name: "MAV_GCS_SYSID",
        scaler: 1.0,
        flags: ConvertFlags::NONE,
    },
    NamedParameterMigration {
        old_key: 4, // k_param_g2
        old_group_element: 4,
        old_type: VarType::Int8,
        new_name: "MAV_OPTIONS",
        scaler: 1.0,
        flags: ConvertFlags::NONE,
    },
    NamedParameterMigration {
        old_key: 115, // k_param_telem_delay_old
        old_group_element: 0,
        old_type: VarType::Int8,
        new_name: "MAV_TELEM_DELAY",
        scaler: 1.0,
        flags: ConvertFlags::NONE,
    },
];


/// Old INS harmonic-notch parameters moved to `INS_HNTC2_*`, upstream
/// `notchfilt_conversion_info` in `Parameters.cpp` (ArduPlane-4.2.x).
pub const PLANE_NOTCH_CONVERSIONS: &[NamedParameterMigration] = &[
    NamedParameterMigration {
        old_key: 109, // k_param_ins
        old_group_element: 101,
        old_type: VarType::Int8,
        new_name: "INS_HNTC2_ENABLE",
        scaler: 1.0,
        flags: ConvertFlags::NONE,
    },
    NamedParameterMigration {
        old_key: 109,
        old_group_element: 293,
        old_type: VarType::Float,
        new_name: "INS_HNTC2_ATT",
        scaler: 1.0,
        flags: ConvertFlags::NONE,
    },
    NamedParameterMigration {
        old_key: 109,
        old_group_element: 357,
        old_type: VarType::Float,
        new_name: "INS_HNTC2_FREQ",
        scaler: 1.0,
        flags: ConvertFlags::NONE,
    },
    NamedParameterMigration {
        old_key: 109,
        old_group_element: 421,
        old_type: VarType::Float,
        new_name: "INS_HNTC2_BW",
        scaler: 1.0,
        flags: ConvertFlags::NONE,
    },
];

/// Old dedicated channel params mapped to `RCx_OPTION`, upstream
/// `rc_option_conversion` in `Parameters.cpp`.
pub const PLANE_RC_OPTION_CONVERSIONS: &[RcOptionConversion] = &[
    RcOptionConversion {
        old_key: 58, // k_param_flapin_channel_old
        old_group_element: 0,
        aux_func: 208, // AUX_FUNC::FLAP
    },
    RcOptionConversion {
        old_key: PLANE_G2_OLD_KEY,
        old_group_element: 968,
        aux_func: 88, // AUX_FUNC::SOARING
    },
    RcOptionConversion {
        old_key: 227, // k_param_fence_channel (AP_FENCE_ENABLED)
        old_group_element: 0,
        aux_func: 11, // AUX_FUNC::FENCE
    },
    RcOptionConversion {
        old_key: 26, // k_param_reset_mission_chan (AP_MISSION_ENABLED)
        old_group_element: 0,
        aux_func: 24, // AUX_FUNC::MISSION_RESET
    },
    RcOptionConversion {
        old_key: 101, // k_param_parachute_channel (HAL_PARACHUTE_ENABLED)
        old_group_element: 0,
        aux_func: 22, // AUX_FUNC::PARACHUTE_RELEASE
    },
    RcOptionConversion {
        old_key: 78, // k_param_fbwa_tdrag_chan
        old_group_element: 0,
        aux_func: 95, // AUX_FUNC::FBWA_TAILDRAGGER
    },
    RcOptionConversion {
        old_key: 21, // k_param_reset_switch_chan
        old_group_element: 0,
        aux_func: 96, // AUX_FUNC::MODE_SWITCH_RESET
    },
];

/// G2 sub-objects moved to AP_Vehicle, upstream `g2_conversions` in
/// `Parameters.cpp` (Plane-4.6).
pub const PLANE_G2_CONVERSIONS: &[G2ObjectConversionEntry] = &[
    G2ObjectConversionEntry {
        old_index: 22,
        object_name: "EFI",
    },
    G2ObjectConversionEntry {
        old_index: 5,
        object_name: "STAT",
    },
    G2ObjectConversionEntry {
        old_index: 14,
        object_name: "SCR",
    },
    G2ObjectConversionEntry {
        old_index: 12,
        object_name: "GRIP",
    },
];
