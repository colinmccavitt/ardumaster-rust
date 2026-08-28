//! VT-008 leftover-complete catalog: tiltrotor.cpp/h surfaces on main
//! vs this closer. Remaining must stay empty.

use ap_quadplane::tiltrotor::{
    LOG_TILT_FIELDS, LOG_TILT_NAME, LOG_TILT_UNITS, TiltType, Tiltrotor, TiltrotorConfig,
    TILT_SERVO_MAX,
};
use ap_quadplane::tiltrotor_completeness::{
    bicopter_servo_max, completeness_counts, completeness_has, completeness_unique_names,
    leftover_api_contract, log_tilt_fields, log_tilt_name, on_main_items, remaining_items,
    this_slice_items, tiltrotor_surfaces_complete, PortStatus, TiltrotorPortItem,
    TILTROTOR_COMPLETENESS,
};

/// Surfaces already on main — do not redo these slices.
const ON_MAIN: &[&str] = &[
    "enable / type",
    "tilt-angle / slew",
    "vectored-yaw / flap mix",
    "fully_fwd / fully_up / tilt predicates",
];

const THIS_SLICE: &[&str] = &[
    "update / continuous / binary",
    "tilt_compensate",
    "bicopter_output",
    "write_log",
    "get_forward_throttle",
    "Tiltrotor_Transition",
    "tilt_max_change fast-tilt / flap-range",
    "is_motor_tilting / motors_active / has_*_motor",
    "completeness table",
];

/// Leftover `tiltrotor.cpp` / `.h` surfaces not yet stubbed.
const REMAINING: &[&str] = &[];

#[test]
fn completeness_table_matches_main_versus_leftover_api() {
    assert!(completeness_unique_names());
    assert_eq!(
        TILTROTOR_COMPLETENESS.len(),
        ON_MAIN.len() + THIS_SLICE.len() + REMAINING.len()
    );
    let (on_main, this_slice, remaining) = completeness_counts();
    assert_eq!(on_main, ON_MAIN.len());
    assert_eq!(this_slice, THIS_SLICE.len());
    assert_eq!(remaining, REMAINING.len());
    assert_eq!(remaining, 0);
    for name in ON_MAIN {
        assert!(
            completeness_has(name, PortStatus::OnMain),
            "{name} must stay listed as already on main"
        );
    }
    for name in THIS_SLICE {
        assert!(
            completeness_has(name, PortStatus::ThisSlice),
            "{name} must be the this-slice row"
        );
    }
    assert!(REMAINING.is_empty());
    assert_eq!(on_main_items().count(), ON_MAIN.len());
    assert_eq!(this_slice_items().count(), THIS_SLICE.len());
    assert_eq!(remaining_items().count(), 0);
    assert!(tiltrotor_surfaces_complete());
}

#[test]
fn leftover_api_rows_name_upstream_surfaces() {
    let leftover: Vec<&TiltrotorPortItem> = this_slice_items().collect();
    assert_eq!(leftover.len(), THIS_SLICE.len());
    assert!(leftover
        .iter()
        .any(|item| item.note.contains("continuous_update") && item.note.contains("binary_slew")));
    assert!(leftover
        .iter()
        .any(|item| item.note.contains("tilt_compensate_angle")));
    assert!(leftover
        .iter()
        .any(|item| item.note.contains("bicopter_output")));
    assert!(leftover
        .iter()
        .any(|item| item.note.contains("LOG_TILT_MSG") && item.note.contains("TILT")));
    assert!(leftover
        .iter()
        .any(|item| item.note.contains("get_forward_throttle")));
    assert!(leftover.iter().any(|item| item
        .note
        .contains("use_multirotor_control_in_fwd_transition")
        && item.note.contains("allow_vfwd")));
}

#[test]
fn leftover_complete_has_no_remaining() {
    assert_eq!(remaining_items().count(), 0);
    assert!(tiltrotor_surfaces_complete());
    assert!(leftover_api_contract());
}

#[test]
fn tilt_log_contract_matches_upstream() {
    assert_eq!(log_tilt_name(), "TILT");
    assert_eq!(log_tilt_fields(), "TimeUS,Tilt,FL,FR");
    assert_eq!(LOG_TILT_NAME, "TILT");
    assert_eq!(LOG_TILT_FIELDS, "TimeUS,Tilt,FL,FR");
    assert_eq!(LOG_TILT_UNITS, "sddd");
    assert!((bicopter_servo_max() - 4500.0).abs() < f32::EPSILON);
    assert!((TILT_SERVO_MAX - 4500.0).abs() < f32::EPSILON);
}

#[test]
fn on_main_rows_still_live() {
    let tr = Tiltrotor::setup(TiltrotorConfig::with_tilt_mask(0b0011));
    assert!(tr.enabled());
    assert_eq!(tr.tilt_type(), Some(TiltType::Continuous));
    assert!(tr.fully_up());
    assert!(!tr.fully_fwd());
    assert!(!tr.tilt_over_max_angle(0.0));
    // Continuous + enabled starts with angle_achieved false until slew hits.
    assert!(!tr.tilt_angle_achieved());
}
