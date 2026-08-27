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
