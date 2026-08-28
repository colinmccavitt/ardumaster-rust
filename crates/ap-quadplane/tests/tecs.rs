//! Leftover should_disable_TECS / allow_stick_mixing / stopping_distance_m stub.

use ap_quadplane::land_sequence::LandSequenceView;
use ap_quadplane::poscontrol::PositionControlState;
use ap_quadplane::quadplane_completeness::{
    accel_needed, allow_stick_mixing, completeness_counts, completeness_has,
    leftover_stopping_distance_m, leftover_transition_threshold_m, remaining_items,
    should_disable_tecs, PortStatus, TRANSITION_THRESHOLD_SCALE,
};
use ap_quadplane::tecs::{StickMixView, TecsView};
use ap_quadplane::transition_fsm::Q_TRANS_DECEL_DEFAULT;
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

#[test]
fn should_disable_tecs_on_land_descent_or_guided_vtol_loiter() {
    let mut qp = available_qp();
    assert!(!qp.should_disable_tecs(TecsView::new()));
    assert!(!should_disable_tecs(false, false));

    assert!(qp.should_disable_tecs(TecsView::guided_vtol_loiter()));
    assert!(should_disable_tecs(false, true));
    let guided_only = TecsView {
        land: LandSequenceView::new(),
        in_guided: true,
        vtol_loiter: false,
    };
    assert!(!qp.should_disable_tecs(guided_only));
    let loiter_not_guided = TecsView {
        land: LandSequenceView::new(),
        in_guided: false,
        vtol_loiter: true,
    };
    assert!(!qp.should_disable_tecs(loiter_not_guided));

    qp.poscontrol_mut()
        .set_state(PositionControlState::LandDescend);
    assert!(qp.should_disable_tecs(TecsView::qrtl()));
    assert!(should_disable_tecs(true, false));
    qp.poscontrol_mut()
        .set_state(PositionControlState::LandFinal);
    assert!(qp.should_disable_tecs(TecsView::auto_vtol_land()));
    qp.poscontrol_mut().set_state(PositionControlState::None);
    assert!(!qp.should_disable_tecs(TecsView::qrtl()));
}

#[test]
fn allow_stick_mixing_true_when_unavailable_else_asks_transition() {
    let disabled = QuadPlane::new();
    assert!(!disabled.available());
    assert!(disabled.allow_stick_mixing(StickMixView::tailsitter_blocked()));
    assert!(allow_stick_mixing(false, false));
    assert!(allow_stick_mixing(false, true));

    let qp = available_qp();
    assert!(qp.available());
    assert!(qp.allow_stick_mixing(StickMixView::slt()));
    assert!(!qp.allow_stick_mixing(StickMixView::tailsitter_blocked()));
    assert!(allow_stick_mixing(true, true));
    assert!(!allow_stick_mixing(true, false));
}

#[test]
fn stopping_distance_m_is_v_squared_over_two_decel() {
    let mut qp = available_qp();
    assert_eq!(qp.tecs().transition_decel_mss() as i32, 2);
    assert_eq!(Q_TRANS_DECEL_DEFAULT as i32, 2);
    assert_eq!(qp.stopping_distance_m(100.0) as i32, 25);
    assert_eq!(leftover_stopping_distance_m(100.0, 2.0) as i32, 25);
    assert_eq!(qp.stopping_distance_from_groundspeed(10.0) as i32, 25);

    qp.set_transition_decel_mss(4.0);
    assert_eq!(qp.stopping_distance_m(100.0) as i32, 12);
    assert_eq!(qp.accel_needed(10.0, 100.0) as i32, 5);
    assert_eq!(accel_needed(0.5, 100.0) as i32, 50);
    assert_eq!(qp.transition_threshold_m(10.0) as i32, 18);
    assert_eq!(
        leftover_transition_threshold_m(10.0, 4.0, TRANSITION_THRESHOLD_SCALE) as i32,
        18
    );
    assert_eq!((TRANSITION_THRESHOLD_SCALE * 10.0) as i32, 15);
}

#[test]
fn catalog_marks_tecs_this_slice_and_remaining_empty() {
    let (on_main, this_slice, remaining) = completeness_counts();
    assert_eq!(on_main, 18);
    assert_eq!(this_slice, 1);
    assert_eq!(remaining, 0);
    assert!(completeness_has(
        "thrust-loss / ESC-cal / takeoff-failure",
        PortStatus::OnMain
    ));
    assert!(completeness_has(
        "TECS / stick-mix / stopping-distance leftovers",
        PortStatus::ThisSlice
    ));
    assert_eq!(remaining_items().count(), 0);
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    assert!(qp.available());
    assert!(!qp.should_disable_tecs(TecsView::new()));
    assert!(qp.allow_stick_mixing(StickMixView::slt()));
}
