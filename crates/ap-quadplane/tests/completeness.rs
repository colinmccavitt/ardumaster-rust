//! VT-001 completeness: QuadPlane surfaces already on main vs leftover API.

use ap_quadplane::poscontrol::PositionControlState;
use ap_quadplane::quadplane_completeness::{
    airbrake_state, completeness_counts, completeness_has, completeness_unique_names,
    land_approach_state, land_descent_state, land_final_state, land_poscontrol_state,
    land_sequence, leftover_option_is_set, on_main_items, qrtl_approach_state, qtun_assist_flags,
    remaining_items, this_slice_items, LeftoverQOption, PortStatus, QuadPlanePortItem, RtlMode,
    ARMING_DELAY_MS, LOG_MESSAGES, QPOS_FIELDS, QTUN_ASSIST_FW_FORCE,
    QTUN_ASSIST_IN_ASSISTED_FLIGHT, QTUN_ASSIST_SPIN_RECOVERY, QTUN_FIELDS, QTUN_PERIOD_MS,
    QUADPLANE_COMPLETENESS,
};
use ap_quadplane::QuadPlane;

/// Surfaces already on main — do not redo these slices.
const ON_MAIN: &[&str] = &[
    "setup / Q_FRAME_CLASS",
    "mode_enter / poscontrol",
    "vtol_mode",
    "air_mode",
    "weathervane",
    "throttle mix / tilt-wait",
    "motor_test",
    "landing-detect / do_user_takeoff",
    "completeness table",
    "AUTO mission VTOL",
    "logging",
    "leftover Q_OPTIONS bits",
    "assisted-flight latch extras",
    "position / takeoff / waypoint controllers",
    "land-sequence predicates",
    "motors_output / hold / set_armed",
    "guided / QRTL / RTL_MODE",
];

const THIS_SLICE: &[&str] = &["thrust-loss / ESC-cal / takeoff-failure"];

/// Leftover `quadplane.cpp` / `.h` surfaces not yet stubbed.
const REMAINING: &[&str] = &["TECS / stick-mix / stopping-distance leftovers"];

#[test]
fn completeness_table_matches_main_versus_leftover_api() {
    assert!(completeness_unique_names());
    assert_eq!(
        QUADPLANE_COMPLETENESS.len(),
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
            "{name} must be the this-slice row"
        );
    }
    for name in REMAINING {
        assert!(
            completeness_has(name, PortStatus::Remaining),
            "{name} is leftover quadplane.cpp/h API not yet stubbed"
        );
    }
    assert_eq!(on_main_items().count(), ON_MAIN.len());
    assert_eq!(this_slice_items().count(), THIS_SLICE.len());
    assert_eq!(remaining_items().count(), REMAINING.len());
}

#[test]
fn leftover_api_rows_name_upstream_surfaces() {
    let leftover: Vec<&QuadPlanePortItem> = remaining_items().collect();
    assert_eq!(leftover.len(), 1);
    assert!(completeness_has(
        "leftover Q_OPTIONS bits",
        PortStatus::OnMain
    ));
    assert!(completeness_has(
        "assisted-flight latch extras",
        PortStatus::OnMain
    ));
    assert!(completeness_has(
        "position / takeoff / waypoint controllers",
        PortStatus::OnMain
    ));
    assert!(completeness_has(
        "land-sequence predicates",
        PortStatus::OnMain
    ));
    assert!(completeness_has(
        "motors_output / hold / set_armed",
        PortStatus::OnMain
    ));
    assert!(completeness_has(
        "guided / QRTL / RTL_MODE",
        PortStatus::OnMain
    ));
    assert!(completeness_has(
        "thrust-loss / ESC-cal / takeoff-failure",
        PortStatus::ThisSlice
    ));
    assert!(QUADPLANE_COMPLETENESS.iter().any(|item| {
        item.name == "leftover Q_OPTIONS bits"
            && item.note.contains("LEVEL_TRANSITION")
            && item.note.contains("FS_QRTL")
    }));
    assert!(QUADPLANE_COMPLETENESS.iter().any(|item| {
        item.name == "assisted-flight latch extras"
            && item.note.contains("force_fw_control_recovery")
            && item.note.contains("in_spin_recovery")
    }));
    assert!(completeness_has("logging", PortStatus::OnMain));
    assert!(QUADPLANE_COMPLETENESS
        .iter()
        .any(|item| item.name == "logging" && item.note.contains("Log_Write_QControl_Tuning")));
    assert!(QUADPLANE_COMPLETENESS.iter().any(|item| {
        item.name == "position / takeoff / waypoint controllers"
            && item.note.contains("vtol_position_controller")
    }));
    assert!(QUADPLANE_COMPLETENESS.iter().any(|item| {
        item.name == "land-sequence predicates" && item.note.contains("in_vtol_land_approach")
    }));
    assert!(completeness_has("AUTO mission VTOL", PortStatus::OnMain));
    assert!(QUADPLANE_COMPLETENESS.iter().any(|item| {
        item.name == "motors_output / hold / set_armed"
            && item.note.contains("hold_hover")
            && item.note.contains("set_armed")
    }));
    assert!(QUADPLANE_COMPLETENESS.iter().any(|item| {
        item.name == "guided / QRTL / RTL_MODE"
            && item.note.contains("guided_start")
            && item.note.contains("RTL_MODE")
    }));
    assert!(QUADPLANE_COMPLETENESS.iter().any(|item| {
        item.name == "thrust-loss / ESC-cal / takeoff-failure"
            && item.note.contains("thrust_loss_check")
            && item.note.contains("run_esc_calibration")
    }));
    assert!(leftover
        .iter()
        .any(|item| item.note.contains("should_disable_TECS")));
}

#[test]
fn leftover_q_options_and_rtl_mode_contract() {
    assert_eq!(LeftoverQOption::LevelTransition.as_i32(), 1);
    assert_eq!(LeftoverQOption::AllowFwTakeoff.as_i32(), 2);
    assert_eq!(LeftoverQOption::AllowFwLand.as_i32(), 4);
    assert_eq!(LeftoverQOption::FsQrtl.as_i32(), 1 << 5);
    assert_eq!(LeftoverQOption::DelayArming.as_i32(), 1 << 11);
    assert_eq!(LeftoverQOption::ThrLandingControl.as_i32(), 1 << 15);
    assert_eq!(LeftoverQOption::FsRtl.as_i32(), 1 << 20);
    assert!(leftover_option_is_set(
        LeftoverQOption::DelayArming.as_i32() | LeftoverQOption::FsRtl.as_i32(),
        LeftoverQOption::DelayArming
    ));
    assert!(!leftover_option_is_set(
        LeftoverQOption::DelayArming.as_i32(),
        LeftoverQOption::DisarmedTilt
    ));
    assert_eq!(RtlMode::from_i8(0), Some(RtlMode::None));
    assert_eq!(RtlMode::from_i8(3), Some(RtlMode::QrtlAlways));
    assert_eq!(RtlMode::from_i8(-1), None);
    assert_eq!(ARMING_DELAY_MS, 2000);
}

#[test]
fn leftover_logging_and_assist_latch_extras() {
    assert_eq!(QTUN_PERIOD_MS, 40);
    assert_eq!(LOG_MESSAGES, ["QTUN", "QPOS", "QBRK", "FWDT"]);
    assert_eq!(
        QPOS_FIELDS,
        ["TimeUS", "State", "Dist", "TSpd", "TAcc", "OShoot"]
    );
    assert_eq!(QTUN_FIELDS[0], "throttle_in");
    assert_eq!(QTUN_FIELDS[11], "assist");
    assert_eq!(
        qtun_assist_flags(true, false, false, false, false, true, true),
        QTUN_ASSIST_IN_ASSISTED_FLIGHT | QTUN_ASSIST_FW_FORCE | QTUN_ASSIST_SPIN_RECOVERY
    );
    assert_eq!(
        qtun_assist_flags(false, false, false, false, false, false, false),
        0
    );
}

#[test]
fn leftover_land_sequence_state_contract() {
    assert!(land_descent_state(PositionControlState::LandDescend));
    assert!(land_final_state(true, PositionControlState::LandFinal));
    assert!(!land_final_state(true, PositionControlState::LandDescend));
    assert!(land_approach_state(PositionControlState::Airbrake));
    assert!(qrtl_approach_state(PositionControlState::Position2));
    assert!(!qrtl_approach_state(PositionControlState::LandFinal));
    assert!(airbrake_state(PositionControlState::Airbrake));
    assert!(land_poscontrol_state(PositionControlState::LandComplete));
    assert!(!land_poscontrol_state(PositionControlState::Approach));
    assert!(land_sequence(false, false, true, false));
    assert!(!land_sequence(false, false, false, false));
}

#[test]
fn table_does_not_rewrite_on_main_setup_or_landing() {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    assert!(qp.available());
    assert!(qp.enabled());
    qp.mode_enter();
    assert_eq!(qp.poscontrol().state(), PositionControlState::None);
    assert!(!qp.in_assisted_flight());
    assert!(!qp.air_mode_active());
}
