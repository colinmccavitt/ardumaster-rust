//! VT-007 Tailsitter completeness: surfaces already on main vs leftover.
//!
//! Catalogs the `ArduPlane/tailsitter.cpp` / `.h` port. Items marked
//! [`PortStatus::OnMain`] landed in earlier slices and must not be
//! redone. [`PortStatus::ThisSlice`] is this table plus leftover-API
//! contract helpers. [`PortStatus::Remaining`] are the leftover
//! `tailsitter.cpp` / `.h` surfaces not yet stubbed (AP_Param
//! `var_info`, `defaults_table_tailsitter`, live `output()` SRV /
//! motors writes, `speed_scaling` SRV apply + `MIN_VO`, TSIT logger
//! backend, `Tailsitter_Transition` heap allocation).
//!
//! This module does not rewrite [`crate::tailsitter`],
//! [`crate::transition`], or [`crate::transition_fsm`].

use crate::tailsitter::{GRAVITY_MSS, SSL_AIR_DENSITY, TAILSITTER_MIXING_GAIN_DEFAULT};

/// Whether a catalog row is already hooked up or left for later work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortStatus {
    /// Present on `main` before this closing slice.
    OnMain,
    /// Added by the VT-007 closing slice (this table).
    ThisSlice,
    /// Leftover `tailsitter.cpp` / `.h` surface, not yet stubbed.
    Remaining,
}

/// One Tailsitter surface in the completeness table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TailsitterPortItem {
    /// Surface name.
    pub name: &'static str,
    /// Hooked up on main / this slice, or remaining.
    pub status: PortStatus,
    /// Short note (upstream symbol or why remaining).
    pub note: &'static str,
}

/// Completeness table: ported tailsitter stubs vs leftover API.
///
/// Row names match the closer catalog: the nine OnMain slices, this
/// table, then leftover `tailsitter.cpp` / `.h` surfaces.
pub const TAILSITTER_COMPLETENESS: &[TailsitterPortItem] = &[
    TailsitterPortItem {
        name: "enable / input-type",
        status: PortStatus::OnMain,
        note: "Tailsitter::enabled / setup heuristic / InputType VectoredYaw vs ControlSurfaces",
    },
    TailsitterPortItem {
        name: "transition pitch/throttle ramp",
        status: PortStatus::OnMain,
        note: "transition.rs TransitionRamp Q_TAILSIT_ANGLE / THR_VT / get_transition_angle_vtol",
    },
    TailsitterPortItem {
        name: "VectoredYawMix",
        status: PortStatus::OnMain,
        note: "VectoredYawMix Q_TAILSIT_VHGAIN / VFGAIN / VHPOW hover and forward tilt mix",
    },
    TailsitterPortItem {
        name: "Q_TAILSIT_INPUT",
        status: PortStatus::OnMain,
        note: "TailsitInput PlaneMode / BodyFrameRoll / Tailsitter::check_input stick swap",
    },
    TailsitterPortItem {
        name: "MOTMX / output_motor_mask",
        status: PortStatus::OnMain,
        note: "output_kind / mask_motor_actuator / Q_TAILSIT_MOTMX",
    },
    TailsitterPortItem {
        name: "GSCMSK / relax_pitch",
        status: PortStatus::OnMain,
        note: "GainScaling Q_TAILSIT_GSCMSK + Tailsitter::relax_pitch",
    },
    TailsitterPortItem {
        name: "pitch-forward / pitch-down limit",
        status: PortStatus::OnMain,
        note: "PitchLimit set_VTOL_roll_pitch_limit + fw_limit leftover + in_vtol_transition",
    },
    TailsitterPortItem {
        name: "Tailsitter_Transition FSM",
        status: PortStatus::OnMain,
        note: "TailsitterTransition ANGLE_WAIT_* / DONE complete predicates",
    },
    TailsitterPortItem {
        name: "copter mix / write_log / setup leftover",
        status: PortStatus::OnMain,
        note: "CopterOutputMix elevon/V-tail / TsitLog / SurfaceAssign / enable_always_setup",
    },
    TailsitterPortItem {
        name: "completeness table",
        status: PortStatus::ThisSlice,
        note: "this catalog + leftover API contract helpers",
    },
    TailsitterPortItem {
        name: "var_info / AP_Param object defaults",
        status: PortStatus::Remaining,
        note: "Tailsitter::var_info + AP_Param::setup_object_defaults (not stubbed)",
    },
    TailsitterPortItem {
        name: "defaults_table_tailsitter",
        status: PortStatus::Remaining,
        note: "AP_Param::set_defaults_from_table(defaults_table_tailsitter) (not stubbed)",
    },
    TailsitterPortItem {
        name: "output() live SRV/motors write",
        status: PortStatus::Remaining,
        note: "motors->output / output_motor_mask / SRV_Channels::set_output_scaled (not stubbed)",
    },
    TailsitterPortItem {
        name: "speed_scaling SRV apply + MIN_VO",
        status: PortStatus::Remaining,
        note: "speed_scaling SRV apply loop + disk_loading_min_outflow MIN_VO (not stubbed)",
    },
    TailsitterPortItem {
        name: "TSIT logger backend",
        status: PortStatus::Remaining,
        note: "plane.logger.WriteBlock LOG_TSIT_MSG (not stubbed)",
    },
    TailsitterPortItem {
        name: "transition object allocation",
        status: PortStatus::Remaining,
        note: "NEW_NOTHROW Tailsitter_Transition + quadplane.transition (not stubbed)",
    },
];

/// `Q_TAILSIT_*` `var_info` parameter names, in GROUPINFO order.
///
/// Unused slots 5 (`MASK`) and 6 (`MASKCH`) are omitted, matching
/// upstream comments.
pub const VAR_INFO_PARAMS: &[&str] = &[
    "ENABLE", "ANGLE", "ANG_VT", "INPUT", "VFGAIN", "VHGAIN", "VHPOW", "GSCMAX", "RLL_MX", "MOTMX",
    "GSCMSK", "GSCMIN", "DSKLD", "RAT_FW", "RAT_VT", "THR_VT", "VT_R_P", "VT_P_P", "VT_Y_P",
    "MIN_VO",
];

/// One `defaults_table_tailsitter` row.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DefaultsTableRow {
    /// Plane / QuadPlane parameter name.
    pub name: &'static str,
    /// Value `set_defaults_from_table` would write.
    pub value: f32,
}

/// Upstream `defaults_table_tailsitter`.
pub const DEFAULTS_TABLE_TAILSITTER: &[DefaultsTableRow] = &[
    DefaultsTableRow {
        name: "KFF_RDDRMIX",
        value: 0.02,
    },
    DefaultsTableRow {
        name: "Q_A_RAT_PIT_FF",
        value: 0.2,
    },
    DefaultsTableRow {
        name: "Q_A_RAT_YAW_FF",
        value: 0.2,
    },
    DefaultsTableRow {
        name: "Q_A_RAT_YAW_I",
        value: 0.18,
    },
    DefaultsTableRow {
        name: "Q_A_ANGLE_BOOST",
        value: 0.0,
    },
    DefaultsTableRow {
        name: "PTCH_LIM_MAX_DEG",
        value: 30.0,
    },
    DefaultsTableRow {
        name: "PTCH_LIM_MIN_DEG",
        value: -30.0,
    },
    DefaultsTableRow {
        name: "MIXING_GAIN",
        value: TAILSITTER_MIXING_GAIN_DEFAULT,
    },
    DefaultsTableRow {
        name: "RUDD_DT_GAIN",
        value: 10.0,
    },
    DefaultsTableRow {
        name: "Q_TRANSITION_MS",
        value: 2000.0,
    },
    DefaultsTableRow {
        name: "Q_TRANS_DECEL",
        value: 6.0,
    },
    DefaultsTableRow {
        name: "Q_A_ACC_P_MAX",
        value: 300.0,
    },
    DefaultsTableRow {
        name: "Q_A_ACC_R_MAX",
        value: 300.0,
    },
    DefaultsTableRow {
        name: "Q_P_NE_POS_P",
        value: 0.5,
    },
    DefaultsTableRow {
        name: "Q_P_NE_VEL_P",
        value: 1.0,
    },
    DefaultsTableRow {
        name: "Q_P_NE_VEL_I",
        value: 0.5,
    },
    DefaultsTableRow {
        name: "Q_P_NE_VEL_D",
        value: 0.25,
    },
];

/// `SRV_Channel::Function` names `speed_scaling` walks.
///
/// Tilt motors take `throttle_scaler`; the rest take `spd_scaler`.
pub const SPEED_SCALE_SRV_FUNCTIONS: &[&str] = &[
    "k_aileron",
    "k_elevator",
    "k_rudder",
    "k_tiltMotorLeft",
    "k_tiltMotorRight",
];

/// Logger message name. Upstream `@LoggerMessage: TSIT`.
pub const LOG_TSIT_NAME: &str = "TSIT";

/// TSIT field names. Upstream `"TimeUS,Ts,Ss,Tmin"`.
pub const LOG_TSIT_FIELDS: &str = "TimeUS,Ts,Ss,Tmin";

/// True when `speed_scaling` applies `throttle_scaler` (tilt motors).
#[must_use]
pub fn speed_scale_uses_throttle_scaler(function: &str) -> bool {
    function == "k_tiltMotorLeft" || function == "k_tiltMotorRight"
}

/// `Q_TAILSIT_MIN_VO` leftover: throttle that keeps outflow at `min_outflow`.
///
/// Upstream `disk_loading_min_throttle` inside the disk-theory branch of
/// `Tailsitter::speed_scaling`. Zero `min_outflow` disables the boost.
/// Forward `airspeed` uses `Ue^2 - U0^2`; zero/negative airspeed uses
/// `Ue^2 + reverse^2` with `reverse = MIN(body-x, 0)`. Result is
/// `MAX(..., 0)` and is later pushed to
/// `AP_MotorsTailsitter::set_min_throttle`.
#[must_use]
pub fn disk_loading_min_throttle(
    min_outflow: f32,
    airspeed: f32,
    reverse_airspeed: f32,
    disk_loading: f32,
    hover: f32,
    density_ratio: f32,
) -> f32 {
    if min_outflow <= 0.0 || disk_loading <= 0.0 {
        return 0.0;
    }
    let rho = SSL_AIR_DENSITY * density_ratio;
    let num = if airspeed > 0.0 {
        (min_outflow * min_outflow - airspeed * airspeed) * (0.5 * rho)
    } else {
        let reverse = if reverse_airspeed < 0.0 {
            reverse_airspeed
        } else {
            0.0
        };
        (min_outflow * min_outflow + reverse * reverse) * (0.5 * rho)
    };
    let throttle = (num / (disk_loading * GRAVITY_MSS)) * hover;
    if throttle > 0.0 {
        throttle
    } else {
        0.0
    }
}

/// Value in [`DEFAULTS_TABLE_TAILSITTER`] for `name`, if present.
#[must_use]
pub fn defaults_table_value(name: &str) -> Option<f32> {
    let mut i = 0;
    while i < DEFAULTS_TABLE_TAILSITTER.len() {
        if DEFAULTS_TABLE_TAILSITTER[i].name == name {
            return Some(DEFAULTS_TABLE_TAILSITTER[i].value);
        }
        i += 1;
    }
    None
}

/// Rows already hooked up on `main` (must not be redone).
#[must_use]
pub fn on_main_items() -> impl Iterator<Item = &'static TailsitterPortItem> {
    TAILSITTER_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::OnMain)
}

/// Rows added by this closing slice.
#[must_use]
pub fn this_slice_items() -> impl Iterator<Item = &'static TailsitterPortItem> {
    TAILSITTER_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::ThisSlice)
}

/// Leftover `tailsitter.cpp` / `.h` surfaces not yet stubbed.
#[must_use]
pub fn remaining_items() -> impl Iterator<Item = &'static TailsitterPortItem> {
    TAILSITTER_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::Remaining)
}

/// Count rows in each status bucket.
#[must_use]
pub fn completeness_counts() -> (usize, usize, usize) {
    let mut on_main = 0;
    let mut this_slice = 0;
    let mut remaining = 0;
    for item in TAILSITTER_COMPLETENESS {
        match item.status {
            PortStatus::OnMain => on_main += 1,
            PortStatus::ThisSlice => this_slice += 1,
            PortStatus::Remaining => remaining += 1,
        }
    }
    (on_main, this_slice, remaining)
}

/// True when `name` is listed with `status`.
#[must_use]
pub fn completeness_has(name: &str, status: PortStatus) -> bool {
    TAILSITTER_COMPLETENESS
        .iter()
        .any(|item| item.name == name && item.status == status)
}

/// True when every name in the table appears once.
#[must_use]
pub fn completeness_unique_names() -> bool {
    for (i, item) in TAILSITTER_COMPLETENESS.iter().enumerate() {
        for other in TAILSITTER_COMPLETENESS.iter().skip(i + 1) {
            if item.name == other.name {
                return false;
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_covers_main_surfaces_and_leftover_api() {
        assert!(completeness_unique_names());
        let (on_main, this_slice, remaining) = completeness_counts();
        assert_eq!(on_main, 9);
        assert_eq!(this_slice, 1);
        assert_eq!(remaining, 6);
        assert!(completeness_has("enable / input-type", PortStatus::OnMain));
        assert!(completeness_has(
            "transition pitch/throttle ramp",
            PortStatus::OnMain
        ));
        assert!(completeness_has("VectoredYawMix", PortStatus::OnMain));
        assert!(completeness_has("Q_TAILSIT_INPUT", PortStatus::OnMain));
        assert!(completeness_has(
            "MOTMX / output_motor_mask",
            PortStatus::OnMain
        ));
        assert!(completeness_has("GSCMSK / relax_pitch", PortStatus::OnMain));
        assert!(completeness_has(
            "pitch-forward / pitch-down limit",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "Tailsitter_Transition FSM",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "copter mix / write_log / setup leftover",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "completeness table",
            PortStatus::ThisSlice
        ));
        assert!(completeness_has(
            "var_info / AP_Param object defaults",
            PortStatus::Remaining
        ));
        assert!(completeness_has(
            "defaults_table_tailsitter",
            PortStatus::Remaining
        ));
        assert!(completeness_has(
            "output() live SRV/motors write",
            PortStatus::Remaining
        ));
        assert!(completeness_has(
            "speed_scaling SRV apply + MIN_VO",
            PortStatus::Remaining
        ));
        assert!(completeness_has(
            "TSIT logger backend",
            PortStatus::Remaining
        ));
        assert!(completeness_has(
            "transition object allocation",
            PortStatus::Remaining
        ));
        assert_eq!(on_main_items().count(), 9);
        assert_eq!(this_slice_items().count(), 1);
        assert_eq!(remaining_items().count(), 6);
    }

    #[test]
    fn remaining_does_not_repeat_hooked_surfaces() {
        for item in remaining_items() {
            assert!(
                !completeness_has(item.name, PortStatus::OnMain),
                "{} listed remaining but already on main",
                item.name
            );
            assert!(
                !completeness_has(item.name, PortStatus::ThisSlice),
                "{} listed remaining but added this slice",
                item.name
            );
        }
    }

    #[test]
    fn leftover_var_info_and_defaults_table_match_upstream() {
        assert_eq!(VAR_INFO_PARAMS.len(), 20);
        assert_eq!(VAR_INFO_PARAMS[0], "ENABLE");
        assert_eq!(VAR_INFO_PARAMS[19], "MIN_VO");
        assert_eq!(DEFAULTS_TABLE_TAILSITTER.len(), 17);
        assert_eq!(defaults_table_value("MIXING_GAIN"), Some(1.0));
        assert_eq!(defaults_table_value("Q_TRANSITION_MS"), Some(2000.0));
        assert_eq!(defaults_table_value("PTCH_LIM_MIN_DEG"), Some(-30.0));
        assert_eq!(defaults_table_value("Q_TAILSIT_ENABLE"), None);
    }

    #[test]
    fn leftover_speed_scale_srv_and_tsit_names() {
        assert_eq!(SPEED_SCALE_SRV_FUNCTIONS.len(), 5);
        assert!(!speed_scale_uses_throttle_scaler("k_aileron"));
        assert!(!speed_scale_uses_throttle_scaler("k_elevator"));
        assert!(!speed_scale_uses_throttle_scaler("k_rudder"));
        assert!(speed_scale_uses_throttle_scaler("k_tiltMotorLeft"));
        assert!(speed_scale_uses_throttle_scaler("k_tiltMotorRight"));
        assert_eq!(LOG_TSIT_NAME, "TSIT");
        assert_eq!(LOG_TSIT_FIELDS, "TimeUS,Ts,Ss,Tmin");
    }

    #[test]
    fn leftover_min_vo_zero_or_matched_airspeed_is_zero() {
        assert_eq!(disk_loading_min_throttle(0.0, 0.0, 0.0, 5.0, 0.4, 1.0), 0.0);
        assert_eq!(
            disk_loading_min_throttle(10.0, 10.0, 0.0, 5.0, 0.4, 1.0),
            0.0
        );
        assert_eq!(
            disk_loading_min_throttle(10.0, 12.0, 0.0, 5.0, 0.4, 1.0),
            0.0
        );
    }

    #[test]
    fn leftover_min_vo_hover_and_reverse_are_positive() {
        let hover = disk_loading_min_throttle(10.0, 0.0, 0.0, 5.0, 0.4, 1.0);
        assert!(hover > 0.0);
        let reverse = disk_loading_min_throttle(10.0, 0.0, -4.0, 5.0, 0.4, 1.0);
        assert!(reverse > hover);
        let ignore_pos_reverse = disk_loading_min_throttle(10.0, 0.0, 4.0, 5.0, 0.4, 1.0);
        assert_eq!(ignore_pos_reverse, hover);
    }
}
