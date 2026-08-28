//! VT-002 completeness: VTOL_Assist surfaces already on main vs leftover API.

use ap_vtol_assist::completeness::{
    assist_active, clear_delay_ms, completeness_counts, completeness_has,
    completeness_unique_names, default_clear_delay_ms, default_trigger_delay_ms,
    fw_recovery_option_blocked, on_main_items, remaining_items, spin_recovery_option_blocked,
    this_slice_items, trigger_delay_ms, AssistPortItem, PortStatus, ASSIST_COMPLETENESS,
    GCS_ALT_ASSIST_PREFIX, GCS_ANGLE_ASSIST_PREFIX, LOGGING_GETTERS, RECOVERY_ANGLE_MULT,
    SPIN_PITCH_DEG, SPIN_PITCH_RATE_DEG, SPIN_ROLL_RATE_DEG, SPIN_YAW_RATE_DEG,
};
use ap_vtol_assist::{
    angle_check_enabled, evaluate_angle, evaluate_force, evaluate_speed_alt, AssistOption,
    AssistState, ForceSample, VtolAssist, ASSIST_DELAY_DEFAULT,
};

/// Surfaces already on main — do not redo these slices.
const ON_MAIN: &[&str] = &[
    "enable/check",
    "speed/alt trigger",
    "force/option-bit",
    "angle-error",
];

const THIS_SLICE: &[&str] = &["completeness table"];

/// Leftover `VTOL_Assist.cpp` / `.h` surfaces not yet stubbed.
const REMAINING: &[&str] = &[
    "state update tick",
    "assist active latch",
    "recovery",
    "logging/GCS bits",
    "leftover option paths",
];

#[test]
fn completeness_table_matches_main_versus_leftover_api() {
    assert!(completeness_unique_names());
    assert_eq!(
        ASSIST_COMPLETENESS.len(),
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
            "{name} is leftover VTOL_Assist.cpp/h API not yet stubbed"
        );
    }
    assert_eq!(on_main_items().count(), ON_MAIN.len());
    assert_eq!(this_slice_items().count(), THIS_SLICE.len());
    assert_eq!(remaining_items().count(), REMAINING.len());
}

#[test]
fn leftover_api_rows_name_upstream_surfaces() {
    let leftover: Vec<&AssistPortItem> = remaining_items().collect();
    assert_eq!(leftover.len(), 5);
    assert!(leftover
        .iter()
        .any(|item| item.note.contains("Assist_Hysteresis::update")));
    assert!(leftover
        .iter()
        .any(|item| item.note.contains("should_assist") && item.note.contains("reset()")));
    assert!(leftover
        .iter()
        .any(|item| item.note.contains("check_VTOL_recovery")));
    assert!(leftover
        .iter()
        .any(|item| item.note.contains("in_*") || item.note.contains("STATUSTEXT")));
    assert!(leftover
        .iter()
        .any(|item| item.note.contains("FW_FORCE_DISABLED")));
}

#[test]
fn leftover_state_update_tick_delay_contract() {
    assert_eq!(ASSIST_DELAY_DEFAULT, 0.5);
    assert_eq!(trigger_delay_ms(ASSIST_DELAY_DEFAULT), 500);
    assert_eq!(clear_delay_ms(ASSIST_DELAY_DEFAULT), 1000);
    assert_eq!(default_trigger_delay_ms(), 500);
    assert_eq!(default_clear_delay_ms(), 1000);
}

#[test]
fn leftover_assist_active_latch_or() {
    assert!(!assist_active(false, false, false, false));
    assert!(assist_active(true, false, false, false));
    assert!(assist_active(false, true, false, false));
    assert!(assist_active(false, false, true, false));
    assert!(assist_active(false, false, false, true));
}

#[test]
fn leftover_logging_gcs_and_recovery_option_paths() {
    assert_eq!(GCS_ALT_ASSIST_PREFIX, "Alt assist");
    assert_eq!(GCS_ANGLE_ASSIST_PREFIX, "Angle assist");
    assert_eq!(
        LOGGING_GETTERS,
        [
            "in_force_assist",
            "in_speed_assist",
            "in_alt_assist",
            "in_angle_assist",
        ]
    );
    assert_eq!(RECOVERY_ANGLE_MULT, 2.0);
    assert_eq!(SPIN_YAW_RATE_DEG, 10.0);
    assert_eq!(SPIN_ROLL_RATE_DEG, 30.0);
    assert_eq!(SPIN_PITCH_RATE_DEG, 30.0);
    assert_eq!(SPIN_PITCH_DEG, -45.0);
    assert!(fw_recovery_option_blocked(
        AssistOption::FwForceDisabled.as_i16()
    ));
    assert!(spin_recovery_option_blocked(
        AssistOption::SpinDisabled.as_i16()
    ));
    assert!(!fw_recovery_option_blocked(0));
    assert!(!spin_recovery_option_blocked(0));
}

#[test]
fn closer_does_not_rewrite_on_main_evaluate_paths() {
    let mut assist = VtolAssist::new();
    assist.set_speed(8.0);
    assist.set_alt(15);
    assist.set_angle(30);
    assist.set_state(AssistState::AssistEnabled);
    assert!(assist.should_check());
    assert!(assist.is_enabled());
    assert!(angle_check_enabled(&assist));

    let speed = evaluate_speed_alt(&assist, ap_vtol_assist::SpeedAltSample::new(4.0, true, 5.0));
    assert!(speed.speed_assist());
    assert!(speed.alt_assist());

    let force = evaluate_force(&assist, ForceSample::new(0, true));
    assert!(!force.force_assist());

    assist.set_state(AssistState::ForceEnabled);
    let force = evaluate_force(&assist, ForceSample::new(0, true));
    assert!(force.force_assist());
    assert!(force.spin_while_armed());

    assist.set_state(AssistState::AssistEnabled);
    let angle = evaluate_angle(
        &assist,
        ap_vtol_assist::AngleSample::new(80.0, 0.0, 0, 0, 45.0, 20.0, -25.0),
    );
    assert!(angle.angle_assist());
}
