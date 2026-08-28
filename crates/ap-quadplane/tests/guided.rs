//! Leftover guided_start / guided_update / RTL_MODE stub.

use ap_quadplane::guided::{
    GuidedModeView, GuidedStartView, GuidedUpdateAction, GuidedUpdateView, Q_GUIDED_MODE_DEFAULT,
    Q_RTL_MODE_DEFAULT,
};
use ap_quadplane::poscontrol::PositionControlState;
use ap_quadplane::quadplane_completeness::{
    completeness_counts, completeness_has, guided_mode_enabled, guided_slow_descent,
    guided_update_climbing, rtl_mode_qrtl_always, rtl_mode_vtol_landing, PortStatus, RtlMode,
};
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

#[test]
fn guided_start_clears_takeoff_and_latches_slow_descent() {
    let mut qp = available_qp();
    qp.set_guided_takeoff(true);
    assert!(qp.guided_takeoff());
    assert!(!qp.slow_descent());

    let descend = qp.guided_start(GuidedStartView::abs(20000, 15000));
    assert!(!qp.guided_takeoff());
    assert!(descend.setup_target);
    assert!(descend.approach_inited);
    assert!(descend.slow_descent);
    assert!(qp.slow_descent());
    assert!(guided_slow_descent(20000, 15000));
    assert_eq!(qp.poscontrol().state(), PositionControlState::Approach);

    let climb = qp.guided_start(GuidedStartView::loc_alt(10000, 12000));
    assert!(!climb.slow_descent);
    assert!(!qp.slow_descent());
    assert!(!guided_slow_descent(10000, 12000));
}

#[test]
fn guided_update_climbs_then_holds_position2() {
    let mut qp = available_qp();
    qp.set_guided_takeoff(true);
    qp.set_throttle_wait(true);

    let climb = qp.guided_update(GuidedUpdateView::climbing(8000, 12000));
    assert_eq!(climb.action, GuidedUpdateAction::TakeoffClimb);
    assert!(!climb.throttle_wait);
    assert!(climb.spool_unlimited);
    assert!(!climb.entered_position2);
    assert!(!qp.throttle_wait());
    assert!(qp.guided_takeoff());
    assert!(guided_update_climbing(true, true, 8000, 12000));
    assert!(!guided_update_climbing(true, true, 12000, 12000));

    let hold = qp.guided_update(GuidedUpdateView::arrived(12000, 12000));
    assert_eq!(hold.action, GuidedUpdateAction::PositionHold);
    assert!(hold.entered_position2);
    assert!(!hold.spool_unlimited);
    assert!(!qp.guided_takeoff());
    assert_eq!(qp.poscontrol().state(), PositionControlState::Position2);

    let again = qp.guided_update(GuidedUpdateView::arrived(12000, 12000));
    assert_eq!(again.action, GuidedUpdateAction::PositionHold);
    assert!(!again.entered_position2);
}

#[test]
fn guided_mode_enabled_needs_available_and_q_guided_mode() {
    let mut qp = QuadPlane::with_enable(1);
    qp.set_guided_mode(1);
    assert!(!qp.guided_mode_enabled(GuidedModeView::guided()));
    assert_eq!(qp.guided_mode(), Q_GUIDED_MODE_DEFAULT + 1);

    assert!(qp.setup());
    assert!(qp.guided_mode_enabled(GuidedModeView::guided()));
    assert!(qp.guided_mode_enabled(GuidedModeView::auto()));
    assert!(!qp.guided_mode_enabled(GuidedModeView::auto_loiter_turns()));

    qp.set_guided_mode(0);
    assert!(!qp.guided_mode_enabled(GuidedModeView::guided()));
    assert!(!guided_mode_enabled(true, false, false, false, 1));
    assert!(guided_mode_enabled(true, true, false, false, 1));
}

#[test]
fn rtl_mode_qrtl_always_and_vtol_landing() {
    let mut qp = available_qp();
    assert_eq!(qp.rtl_mode(), Q_RTL_MODE_DEFAULT);
    assert_eq!(qp.rtl_mode_enum(), Some(RtlMode::None));
    assert!(!qp.rtl_qrtl_always());
    assert!(!qp.rtl_vtol_landing());

    qp.set_rtl_mode(RtlMode::SwitchQrtl.as_i8());
    assert!(qp.rtl_vtol_landing());
    assert!(!qp.rtl_qrtl_always());
    assert!(rtl_mode_vtol_landing(RtlMode::SwitchQrtl));
    assert!(rtl_mode_vtol_landing(RtlMode::VtolApproachQrtl));
    assert!(!rtl_mode_vtol_landing(RtlMode::QrtlAlways));

    qp.set_rtl_mode(RtlMode::QrtlAlways.as_i8());
    assert!(qp.rtl_qrtl_always());
    assert!(!qp.rtl_vtol_landing());
    assert!(rtl_mode_qrtl_always(RtlMode::QrtlAlways));
    assert!(!rtl_mode_qrtl_always(RtlMode::None));

    qp.set_rtl_mode(4);
    assert_eq!(qp.rtl_mode_enum(), None);
    assert!(!qp.rtl_qrtl_always());
    assert!(!qp.rtl_vtol_landing());
}

#[test]
fn catalog_marks_guided_this_slice_and_leaves_other_rows() {
    let (on_main, this_slice, remaining) = completeness_counts();
    assert_eq!(on_main, 16);
    assert_eq!(this_slice, 1);
    assert_eq!(remaining, 2);
    assert!(completeness_has(
        "guided / QRTL / RTL_MODE",
        PortStatus::ThisSlice
    ));
    assert!(completeness_has(
        "motors_output / hold / set_armed",
        PortStatus::OnMain
    ));
    assert!(completeness_has(
        "thrust-loss / ESC-cal / takeoff-failure",
        PortStatus::Remaining
    ));
    assert!(completeness_has(
        "TECS / stick-mix / stopping-distance leftovers",
        PortStatus::Remaining
    ));
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    assert!(qp.available());
    assert!(!qp.guided_mode_enabled(GuidedModeView::guided()));
    assert_eq!(qp.rtl_mode_enum(), Some(RtlMode::None));
}
