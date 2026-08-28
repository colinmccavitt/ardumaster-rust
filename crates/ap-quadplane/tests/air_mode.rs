//! Air-mode / QuadPlane-side transition hooks — upstream
//! `QuadPlane::air_mode_active`, `Q_OPTIONS` `AIRMODE_UNUSED`, and
//! `QuadPlane::update` / `in_frwd_transition` / `handle_do_vtol_transition`.

use ap_quadplane::air_mode::{
    AirMode, MavVtolState, QOption, TransitionHook, TransitionUpdateView, Q_OPTIONS_AIRMODE_UNUSED,
    Q_OPTIONS_DEFAULT,
};
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

#[test]
fn q_options_default_is_zero() {
    let qp = QuadPlane::new();
    assert_eq!(qp.options(), Q_OPTIONS_DEFAULT);
    assert_eq!(qp.options(), 0);
    assert!(!qp.option_is_set(QOption::AirmodeUnused));
}

#[test]
fn airmode_unused_bit_is_one_shl_nine() {
    assert_eq!(Q_OPTIONS_AIRMODE_UNUSED, 1 << 9);
    assert_eq!(Q_OPTIONS_AIRMODE_UNUSED, 512);
    assert_eq!(QOption::AirmodeUnused as i32, Q_OPTIONS_AIRMODE_UNUSED);
}

#[test]
fn option_is_set_reads_bit_nine() {
    let mut qp = QuadPlane::new();
    qp.set_options(Q_OPTIONS_AIRMODE_UNUSED);
    assert!(qp.option_is_set(QOption::AirmodeUnused));
    qp.set_options(0);
    assert!(!qp.option_is_set(QOption::AirmodeUnused));
}

#[test]
fn air_mode_defaults_off_and_inactive() {
    let qp = QuadPlane::new();
    assert_eq!(qp.air_mode(), AirMode::Off);
    assert!(!qp.air_mode_active());
    assert!(!qp.assisted_flight());
    assert_eq!(AirMode::Off as u8, 0);
    assert_eq!(AirMode::On as u8, 1);
    assert_eq!(AirMode::AssistedFlightOnly as u8, 2);
}

#[test]
fn air_mode_on_is_active_without_assist() {
    let mut qp = QuadPlane::new();
    qp.set_air_mode(AirMode::On);
    assert!(qp.air_mode_active());
    assert!(!qp.assisted_flight());
}

#[test]
fn assisted_flight_only_needs_assist() {
    let mut qp = QuadPlane::new();
    qp.set_air_mode(AirMode::AssistedFlightOnly);
    assert!(!qp.air_mode_active());
    qp.set_assisted_flight(true);
    assert!(qp.air_mode_active());
    qp.set_assisted_flight(false);
    assert!(!qp.air_mode_active());
}

#[test]
fn air_mode_off_stays_inactive_while_assisting() {
    let mut qp = QuadPlane::new();
    qp.set_air_mode(AirMode::Off);
    qp.set_assisted_flight(true);
    assert!(!qp.air_mode_active());
}

#[test]
fn air_mode_active_does_not_require_available() {
    // Upstream `air_mode_active` does not call `available()`.
    let mut qp = QuadPlane::with_enable(1);
    assert!(!qp.available());
    qp.set_air_mode(AirMode::On);
    assert!(qp.air_mode_active());
}

#[test]
fn armdisarm_converts_only_when_enabled_bit_set_and_no_aux() {
    let mut qp = QuadPlane::new();
    assert!(!qp.armdisarm_converts_to_airmode(false));
    qp.set_enable(1);
    assert!(!qp.armdisarm_converts_to_airmode(false));
    qp.set_options(Q_OPTIONS_AIRMODE_UNUSED);
    assert!(qp.armdisarm_converts_to_airmode(false));
    assert!(!qp.armdisarm_converts_to_airmode(true));
    qp.set_enable(0);
    assert!(!qp.armdisarm_converts_to_airmode(false));
}

#[test]
fn update_without_setup_calls_no_hook() {
    let mut qp = QuadPlane::new();
    assert_eq!(
        qp.update(&TransitionUpdateView::vtol()),
        TransitionHook::None
    );
    assert!(!qp.available());
}

#[test]
fn update_sets_up_then_dispatches_fw_update() {
    let mut qp = QuadPlane::with_enable(1);
    assert_eq!(
        qp.update(&TransitionUpdateView::new()),
        TransitionHook::Update
    );
    assert!(qp.available());
}

#[test]
fn update_fw_manual_forces_complete_and_clears_assist() {
    let mut qp = available_qp();
    qp.set_assisted_flight(true);
    assert_eq!(
        qp.update(&TransitionUpdateView::fw_manual()),
        TransitionHook::ForceComplete
    );
    assert!(!qp.assisted_flight());
}

#[test]
fn update_vtol_calls_vtol_update_and_clears_assist() {
    let mut qp = available_qp();
    qp.set_assisted_flight(true);
    assert_eq!(
        qp.update(&TransitionUpdateView::vtol()),
        TransitionHook::VtolUpdate
    );
    // `assisted_flight = in_vtol_airbrake()` — airbrake is false here.
    assert!(!qp.assisted_flight());
}

#[test]
fn update_airbrake_calls_vtol_update_and_sets_assist() {
    let mut qp = available_qp();
    assert_eq!(
        qp.update(&TransitionUpdateView::vtol_airbrake()),
        TransitionHook::VtolUpdate
    );
    assert!(qp.assisted_flight());
    assert!(qp.in_assisted_flight());
}

#[test]
fn in_frwd_transition_needs_available_and_active_frwd() {
    let qp = QuadPlane::with_enable(1);
    assert!(!qp.available());
    assert!(!qp.in_frwd_transition(true));
    let qp = available_qp();
    assert!(!qp.in_frwd_transition(false));
    assert!(qp.in_frwd_transition(true));
}

#[test]
fn handle_do_vtol_transition_rejects_until_available_auto() {
    let qp = QuadPlane::with_enable(1);
    assert!(qp
        .handle_do_vtol_transition(MavVtolState::Mc, true)
        .is_none());
    let qp = available_qp();
    assert!(qp
        .handle_do_vtol_transition(MavVtolState::Mc, false)
        .is_none());
}

#[test]
fn handle_do_vtol_transition_mc_enters_vtol() {
    let qp = available_qp();
    assert_eq!(
        qp.handle_do_vtol_transition(MavVtolState::Mc, true),
        Some(true)
    );
}

#[test]
fn handle_do_vtol_transition_fw_exits_vtol() {
    let qp = available_qp();
    assert_eq!(
        qp.handle_do_vtol_transition(MavVtolState::Fw, true),
        Some(false)
    );
}

#[test]
fn handle_do_vtol_transition_rejects_undefined_and_in_between() {
    let qp = available_qp();
    for state in [
        MavVtolState::Undefined,
        MavVtolState::TransitionToFw,
        MavVtolState::TransitionToMc,
    ] {
        assert!(
            qp.handle_do_vtol_transition(state, true).is_none(),
            "{state:?}"
        );
    }
}

#[test]
fn mav_vtol_state_values_match_mavlink() {
    assert_eq!(MavVtolState::Undefined as u8, 0);
    assert_eq!(MavVtolState::TransitionToFw as u8, 1);
    assert_eq!(MavVtolState::TransitionToMc as u8, 2);
    assert_eq!(MavVtolState::Mc as u8, 3);
    assert_eq!(MavVtolState::Fw as u8, 4);
}

#[test]
fn in_assisted_flight_requires_available() {
    let mut qp = QuadPlane::with_enable(1);
    qp.set_assisted_flight(true);
    assert!(!qp.in_assisted_flight());
    assert!(qp.setup());
    assert!(qp.in_assisted_flight());
}
