//! Plane parameter conversion tables, upstream `ArduPlane/Parameters.cpp`.

use crate::conversion::{ConvertFlags, NamedParameterMigration};
use crate::VarType;

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
