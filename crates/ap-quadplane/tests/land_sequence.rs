//! Leftover VTOL land-sequence predicate stub.

use ap_quadplane::land_sequence::LandSequenceView;
use ap_quadplane::poscontrol::PositionControlState;
use ap_quadplane::quadplane_completeness::{completeness_counts, completeness_has, PortStatus};
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

#[test]
fn qrtl_approach_descent_final_and_always_sequence() {
    let mut qp = available_qp();
    let view = LandSequenceView::qrtl();

    qp.poscontrol_mut().set_state(PositionControlState::None);
    assert!(qp.in_vtol_land_approach(view));
    assert!(!qp.in_vtol_land_descent(view));
    assert!(!qp.in_vtol_land_final(view));
    assert!(qp.in_vtol_land_sequence(view));
    assert!(!qp.in_vtol_land_poscontrol(view));
    assert!(!qp.in_vtol_airbrake(view));

    qp.poscontrol_mut()
        .set_state(PositionControlState::Position2);
    assert!(qp.in_vtol_land_approach(view));
    assert!(!qp.in_vtol_land_descent(view));

    qp.poscontrol_mut()
        .set_state(PositionControlState::LandDescend);
    assert!(!qp.in_vtol_land_approach(view));
    assert!(qp.in_vtol_land_descent(view));
    assert!(!qp.in_vtol_land_final(view));
    assert!(qp.in_vtol_land_sequence(view));

    qp.poscontrol_mut()
        .set_state(PositionControlState::LandFinal);
    assert!(qp.in_vtol_land_descent(view));
    assert!(qp.in_vtol_land_final(view));
    assert!(qp.in_vtol_land_sequence(view));

    qp.poscontrol_mut()
        .set_state(PositionControlState::Airbrake);
    assert!(qp.in_vtol_land_approach(view));
    assert!(qp.in_vtol_airbrake(view));
}

#[test]
fn auto_vtol_land_gates_approach_descent_poscontrol_airbrake() {
    let mut qp = available_qp();
    let view = LandSequenceView::auto_vtol_land();

    qp.poscontrol_mut()
        .set_state(PositionControlState::Approach);
    assert!(qp.in_vtol_land_approach(view));
    assert!(!qp.in_vtol_land_descent(view));
    assert!(!qp.in_vtol_land_poscontrol(view));
    assert!(!qp.in_vtol_airbrake(view));
    assert!(qp.in_vtol_land_sequence(view));

    qp.poscontrol_mut()
        .set_state(PositionControlState::Airbrake);
    assert!(qp.in_vtol_land_approach(view));
    assert!(qp.in_vtol_airbrake(view));

    qp.poscontrol_mut()
        .set_state(PositionControlState::Position1);
    assert!(qp.in_vtol_land_approach(view));
    assert!(qp.in_vtol_land_poscontrol(view));
    assert!(!qp.in_vtol_airbrake(view));

    qp.poscontrol_mut()
        .set_state(PositionControlState::LandDescend);
    assert!(!qp.in_vtol_land_approach(view));
    assert!(qp.in_vtol_land_descent(view));
    assert!(!qp.in_vtol_land_final(view));
    assert!(qp.in_vtol_land_poscontrol(view));

    qp.poscontrol_mut()
        .set_state(PositionControlState::LandFinal);
    assert!(qp.in_vtol_land_final(view));
    assert!(qp.in_vtol_land_sequence(view));
}

#[test]
fn auto_without_vtol_land_and_airbrake_uses_mode_auto() {
    let mut qp = available_qp();
    qp.poscontrol_mut()
        .set_state(PositionControlState::Airbrake);

    let fw = LandSequenceView::new();
    assert!(!qp.in_vtol_land_approach(fw));
    assert!(!qp.in_vtol_land_sequence(fw));
    assert!(!qp.in_vtol_airbrake(fw));

    // `in_vtol_auto` without `mode_auto` is not airbrake upstream.
    let vtol_auto_not_mode_auto = LandSequenceView {
        in_qrtl: false,
        in_auto: false,
        in_vtol_auto: true,
        is_vtol_land: true,
    };
    assert!(qp.in_vtol_land_approach(vtol_auto_not_mode_auto));
    assert!(!qp.in_vtol_airbrake(vtol_auto_not_mode_auto));

    let auto_not_vtol_land = LandSequenceView {
        in_qrtl: false,
        in_auto: true,
        in_vtol_auto: false,
        is_vtol_land: false,
    };
    assert!(!qp.in_vtol_land_approach(auto_not_vtol_land));
    assert!(!qp.in_vtol_airbrake(auto_not_vtol_land));

    let auto_land_cmd_not_vtol_auto = LandSequenceView {
        in_qrtl: false,
        in_auto: true,
        in_vtol_auto: false,
        is_vtol_land: true,
    };
    assert!(!qp.in_vtol_land_approach(auto_land_cmd_not_vtol_auto));
    assert!(qp.in_vtol_airbrake(auto_land_cmd_not_vtol_auto));
}

#[test]
fn catalog_marks_land_sequence_this_slice_and_leaves_other_rows() {
    let (on_main, this_slice, remaining) = completeness_counts();
    assert_eq!(on_main, 14);
    assert_eq!(this_slice, 1);
    assert_eq!(remaining, 4);
    assert!(completeness_has(
        "land-sequence predicates",
        PortStatus::ThisSlice
    ));
    assert!(completeness_has(
        "position / takeoff / waypoint controllers",
        PortStatus::OnMain
    ));
    assert!(completeness_has(
        "motors_output / hold / set_armed",
        PortStatus::Remaining
    ));
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    assert!(qp.available());
    assert!(!qp.in_vtol_land_sequence(LandSequenceView::new()));
    assert!(!qp.in_assisted_flight());
}
