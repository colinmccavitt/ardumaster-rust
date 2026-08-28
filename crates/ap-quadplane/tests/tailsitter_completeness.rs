//! VT-007 completeness: tailsitter.cpp/h surfaces already on main vs leftover API.

use ap_quadplane::completeness::{
    completeness_counts, completeness_has, completeness_unique_names, defaults_table_value,
    disk_loading_min_throttle, on_main_items, remaining_items, speed_scale_uses_throttle_scaler,
    this_slice_items, DefaultsTableRow, PortStatus, TailsitterPortItem, DEFAULTS_TABLE_TAILSITTER,
    LOG_TSIT_FIELDS, LOG_TSIT_NAME, SPEED_SCALE_SRV_FUNCTIONS, TAILSITTER_COMPLETENESS,
    VAR_INFO_PARAMS,
};
use ap_quadplane::tailsitter::{
    InputType, OutputContext, OutputKind, Tailsitter, TailsitterConfig, MOTOR_FRAME_TAILSITTER,
    TAILSITTER_MIXING_GAIN_DEFAULT,
};

/// Surfaces already on main — do not redo these slices.
const ON_MAIN: &[&str] = &[
    "enable / input-type",
    "transition pitch/throttle ramp",
    "VectoredYawMix",
    "Q_TAILSIT_INPUT",
    "MOTMX / output_motor_mask",
    "GSCMSK / relax_pitch",
    "pitch-forward / pitch-down limit",
    "Tailsitter_Transition FSM",
    "copter mix / write_log / setup leftover",
];

const THIS_SLICE: &[&str] = &["completeness table"];

/// Leftover `tailsitter.cpp` / `.h` surfaces not yet stubbed.
const REMAINING: &[&str] = &[
    "var_info / AP_Param object defaults",
    "defaults_table_tailsitter",
    "output() live SRV/motors write",
    "speed_scaling SRV apply + MIN_VO",
    "TSIT logger backend",
    "transition object allocation",
];

#[test]
fn completeness_table_matches_main_versus_leftover_api() {
    assert!(completeness_unique_names());
    assert_eq!(
        TAILSITTER_COMPLETENESS.len(),
        ON_MAIN.len() + THIS_SLICE.len() + REMAINING.len()
    );
    let (on_main, this_slice, remaining) = completeness_counts();
    assert_eq!(on_main, ON_MAIN.len());
    assert_eq!(this_slice, THIS_SLICE.len());
    assert_eq!(remaining, REMAINING.len());
    for name in ON_MAIN {
        assert!(
            completeness_has(name, PortStatus::OnMain),
            "{name} must stay listed as already on main"
        );
    }
    for name in THIS_SLICE {
        assert!(
            completeness_has(name, PortStatus::ThisSlice),
            "{name} must be the closing-slice row"
        );
    }
    for name in REMAINING {
        assert!(
            completeness_has(name, PortStatus::Remaining),
            "{name} is leftover tailsitter.cpp/h API not yet stubbed"
        );
    }
    assert_eq!(on_main_items().count(), ON_MAIN.len());
    assert_eq!(this_slice_items().count(), THIS_SLICE.len());
    assert_eq!(remaining_items().count(), REMAINING.len());
}

#[test]
fn leftover_api_rows_name_upstream_surfaces() {
    let leftover: Vec<&TailsitterPortItem> = remaining_items().collect();
    assert_eq!(leftover.len(), 6);
    assert!(leftover
        .iter()
        .any(|item| item.note.contains("var_info") && item.note.contains("setup_object_defaults")));
    assert!(leftover
        .iter()
        .any(|item| item.note.contains("defaults_table_tailsitter")));
    assert!(leftover
        .iter()
        .any(|item| item.note.contains("output_motor_mask")
            && item.note.contains("SRV_Channels::set_output_scaled")));
    assert!(
        leftover
            .iter()
            .any(|item| item.note.contains("disk_loading_min_outflow")
                && item.note.contains("MIN_VO"))
    );
    assert!(leftover
        .iter()
        .any(|item| item.note.contains("LOG_TSIT_MSG")));
    assert!(leftover
        .iter()
        .any(|item| item.note.contains("NEW_NOTHROW Tailsitter_Transition")));
}

#[test]
fn leftover_var_info_defaults_table_and_tsit_contract() {
    assert_eq!(VAR_INFO_PARAMS.len(), 20);
    assert_eq!(VAR_INFO_PARAMS[0], "ENABLE");
    assert_eq!(VAR_INFO_PARAMS[9], "MOTMX");
    assert_eq!(VAR_INFO_PARAMS[19], "MIN_VO");
    assert_eq!(DEFAULTS_TABLE_TAILSITTER.len(), 17);
    assert_eq!(
        defaults_table_value("MIXING_GAIN"),
        Some(TAILSITTER_MIXING_GAIN_DEFAULT)
    );
    assert_eq!(defaults_table_value("Q_TRANSITION_MS"), Some(2000.0));
    assert_eq!(defaults_table_value("KFF_RDDRMIX"), Some(0.02));
    assert!(DEFAULTS_TABLE_TAILSITTER
        .iter()
        .any(|DefaultsTableRow { name, .. }| *name == "Q_A_RAT_YAW_I"));
    assert_eq!(LOG_TSIT_NAME, "TSIT");
    assert_eq!(LOG_TSIT_FIELDS, "TimeUS,Ts,Ss,Tmin");
}

#[test]
fn leftover_speed_scale_srv_apply_and_min_vo() {
    assert_eq!(
        SPEED_SCALE_SRV_FUNCTIONS,
        [
            "k_aileron",
            "k_elevator",
            "k_rudder",
            "k_tiltMotorLeft",
            "k_tiltMotorRight",
        ]
    );
    assert!(!speed_scale_uses_throttle_scaler("k_aileron"));
    assert!(speed_scale_uses_throttle_scaler("k_tiltMotorLeft"));
    assert_eq!(disk_loading_min_throttle(0.0, 0.0, 0.0, 5.0, 0.4, 1.0), 0.0);
    assert_eq!(
        disk_loading_min_throttle(10.0, 10.0, 0.0, 5.0, 0.4, 1.0),
        0.0
    );
    let hover = disk_loading_min_throttle(10.0, 0.0, 0.0, 5.0, 0.4, 1.0);
    assert!(hover > 0.0);
    let reverse = disk_loading_min_throttle(10.0, 0.0, -3.0, 5.0, 0.4, 1.0);
    assert!(reverse > hover);
}

#[test]
fn closer_does_not_rewrite_on_main_enable_and_output_kind() {
    let ts = Tailsitter::setup(TailsitterConfig::tailsitter_frame());
    assert!(ts.enabled());
    assert_eq!(ts.enable(), 1);
    assert_eq!(ts.input_type(), Some(InputType::ControlSurfaces));
    assert_eq!(
        ts.output_kind(OutputContext::fw_cruise()),
        OutputKind::MotorMask
    );
    assert_eq!(
        ts.output_kind(OutputContext::vtol_hover()),
        OutputKind::Copter
    );

    let mut vectored = TailsitterConfig::tailsitter_frame();
    vectored.frame_class = MOTOR_FRAME_TAILSITTER;
    vectored.tilt_motor_left = true;
    let ts = Tailsitter::setup(vectored);
    assert_eq!(ts.input_type(), Some(InputType::VectoredYaw));
    assert!(!ts.relax_pitch(0));
    assert!(ts.relax_pitch(1));
}
