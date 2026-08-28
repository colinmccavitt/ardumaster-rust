//! QLOITER / QLAND / LoiterAltQLand `_enter` — upstream
//! `mode_qloiter.cpp` / `mode_qland.cpp` / `mode_LoiterAltQLand.cpp`.

use ap_quadplane::mode_qland::{
    already_in_a_loiter, loiter_alt_qland_enter, qland_enter, qloiter_enter, switch_qland,
    GuidedAltFrame, LoiterAltQLandAction, LoiterAltQLandEnterView, LoiterAltSeed, QLandEnterState,
    QLandEnterView, QLandFamily, QLoiterEnterState, QLoiterEnterView, MODE_LOITER_ALT_QLAND,
    MODE_QLAND, MODE_QLOITER, MODE_REASON_LOITER_ALT_IN_VTOL, MODE_REASON_LOITER_ALT_REACHED_QLAND,
    Q_RTL_ALT_DEFAULT_M,
};
use ap_quadplane::poscontrol::{PositionControlState, THROTTLE_WAIT_INPUT_MIN};
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

fn dirty_for_mode_enter(qp: &mut QuadPlane) {
    qp.set_lean_angle_max_cd(4500);
    qp.set_throttle_wait(true);
    qp.set_guided_wait_takeoff(true);
    qp.poscontrol_mut()
        .set_state(PositionControlState::Approach);
    qp.poscontrol_mut().set_correction_ne_m(4.0, -2.0);
}

#[test]
fn qland_family_numbers_match_upstream() {
    assert_eq!(MODE_QLOITER, 19);
    assert_eq!(MODE_QLAND, 20);
    assert_eq!(MODE_LOITER_ALT_QLAND, 25);
    assert_eq!(MODE_REASON_LOITER_ALT_REACHED_QLAND, 46);
    assert_eq!(MODE_REASON_LOITER_ALT_IN_VTOL, 47);
    assert_eq!(Q_RTL_ALT_DEFAULT_M, 15.0);
    assert_eq!(QLandFamily::Loiter.mode_number(), MODE_QLOITER);
    assert_eq!(QLandFamily::Land.mode_number(), MODE_QLAND);
    assert_eq!(
        QLandFamily::LoiterAltQLand.mode_number(),
        MODE_LOITER_ALT_QLAND
    );
    assert_eq!(QLandFamily::from_number(19), Some(QLandFamily::Loiter));
    assert_eq!(QLandFamily::from_number(20), Some(QLandFamily::Land));
    assert_eq!(
        QLandFamily::from_number(25),
        Some(QLandFamily::LoiterAltQLand)
    );
    assert_eq!(QLandFamily::from_number(17), None);
    assert_eq!(QLandFamily::from_number(18), None);
}

#[test]
fn qland_family_vtol_flags() {
    assert!(QLandFamily::Loiter.is_vtol_mode());
    assert!(QLandFamily::Loiter.is_vtol_man_mode());
    assert!(!QLandFamily::Loiter.is_vtol_man_throttle());

    assert!(QLandFamily::Land.is_vtol_mode());
    assert!(!QLandFamily::Land.is_vtol_man_mode());
    assert!(!QLandFamily::Land.is_vtol_man_throttle());
    assert!(QLandFamily::Land.qland_pre_arm_refuses());

    assert!(!QLandFamily::LoiterAltQLand.is_vtol_mode());
    assert!(!QLandFamily::LoiterAltQLand.is_vtol_man_mode());
    assert!(!QLandFamily::LoiterAltQLand.is_vtol_man_throttle());
    assert!(!QLandFamily::LoiterAltQLand.qland_pre_arm_refuses());
}

#[test]
fn qloiter_enter_parked_idle_sets_throttle_wait() {
    let mut qp = available_qp();
    dirty_for_mode_enter(&mut qp);
    qp.set_throttle_wait(false);
    let mut state = QLoiterEnterState::new();

    assert!(qloiter_enter(
        &mut qp,
        QLoiterEnterView::parked_idle(),
        &mut state
    ));

    assert!(qp.throttle_wait());
    assert!(state.loiter_accel_cleared);
    assert!(state.loiter_target_inited);
    assert!(state.d_speed_accel_set);
    assert!(state.d_correction_set);
    assert_eq!(state.last_loiter_ms, 1_000);
    assert_eq!(state.last_target_loc_set_ms, 0);
    assert_eq!(qp.lean_angle_max_cd(), 0);
    assert!(qp.poscontrol().mode_enter_cleared());
}

#[test]
fn qloiter_enter_clears_wait_when_stick_or_flying() {
    let mut qp = available_qp();
    qp.set_throttle_wait(true);
    let mut state = QLoiterEnterState::new();
    assert!(qloiter_enter(
        &mut qp,
        QLoiterEnterView::new(THROTTLE_WAIT_INPUT_MIN, false, 2_000),
        &mut state
    ));
    assert!(!qp.throttle_wait());
    assert_eq!(state.last_loiter_ms, 2_000);

    qp.set_throttle_wait(true);
    assert!(qloiter_enter(
        &mut qp,
        QLoiterEnterView::new(0, true, 3_000),
        &mut state
    ));
    assert!(!qp.throttle_wait());
    assert_eq!(state.last_loiter_ms, 3_000);
}

#[test]
fn qland_enter_forces_descend_and_clears_wait() {
    let mut qp = available_qp();
    dirty_for_mode_enter(&mut qp);
    let mut state = QLandEnterState::new();
    let view = QLandEnterView::new(QLoiterEnterView::parked_idle(), 12.5);

    assert!(qland_enter(&mut qp, view, &mut state));

    // Nested QLoiter `_enter` would have set throttle_wait from parked
    // idle; QLand then forces it false.
    assert!(!qp.throttle_wait());
    assert!(state.qloiter.loiter_target_inited);
    assert!(state.qloiter.d_speed_accel_set);
    assert_eq!(state.qloiter.last_loiter_ms, 1_000);
    assert!(state.target_position_setup);
    assert_eq!(state.last_land_final_agl_m, 12.5);
    assert!(state.land_detect_cleared);
    assert!(state.landing_gear_deployed);
    assert_eq!(qp.lean_angle_max_cd(), 0);
    assert_eq!(qp.poscontrol().state(), PositionControlState::LandDescend);
    assert_eq!(qp.poscontrol().correction_north_m(), 0.0);
    assert!(!qp.guided_wait_takeoff());
}

#[test]
fn qland_enter_does_not_leave_poscontrol_at_none() {
    // mode_enter resets to QPOS_NONE; QLand `_enter` must then move
    // to QPOS_LAND_DESCEND. A stub that only called mode_enter would
    // leave the vehicle with no land state.
    let mut qp = available_qp();
    qp.poscontrol_mut()
        .set_state(PositionControlState::Position1);
    let mut state = QLandEnterState::new();
    assert!(qland_enter(
        &mut qp,
        QLandEnterView::parked_idle(),
        &mut state
    ));
    assert_eq!(qp.poscontrol().state(), PositionControlState::LandDescend);
    assert_ne!(qp.poscontrol().state(), PositionControlState::None);
}

#[test]
fn loiter_alt_qland_from_vtol_hands_off() {
    let mut qp = available_qp();
    dirty_for_mode_enter(&mut qp);
    let mut view = LoiterAltQLandEnterView::fw_above();
    view.previous_is_vtol = true;

    let out = loiter_alt_qland_enter(&mut qp, view);
    assert_eq!(out.action, LoiterAltQLandAction::HandoffInVtol);
    assert!(!out.loiter_enter);
    assert!(!out.guided_request);
    assert_eq!(out.mode_reason, Some(MODE_REASON_LOITER_ALT_IN_VTOL));
    assert!(out.qland.is_some());
    assert!(!qp.throttle_wait());
    assert_eq!(qp.poscontrol().state(), PositionControlState::LandDescend);

    view.previous_is_vtol = false;
    view.in_vtol_mode = true;
    let again = loiter_alt_qland_enter(&mut qp, view);
    assert_eq!(again.action, LoiterAltQLandAction::HandoffInVtol);
}

#[test]
fn loiter_alt_qland_fw_above_stays_in_loiter() {
    let mut qp = available_qp();
    let out = loiter_alt_qland_enter(&mut qp, LoiterAltQLandEnterView::fw_above());
    assert_eq!(out.action, LoiterAltQLandAction::StayLoiter);
    assert!(out.loiter_enter);
    assert!(out.guided_request);
    assert_eq!(out.seed, Some(LoiterAltSeed::CurrentLoc));
    assert_eq!(out.guided_alt_m, Q_RTL_ALT_DEFAULT_M);
    assert_eq!(out.guided_frame, GuidedAltFrame::AboveHome);
    assert!(out.mode_reason.is_none());
    assert!(out.qland.is_none());
    // mode_enter ran; no QLand `_enter`, so poscontrol stays at None.
    assert!(qp.poscontrol().mode_enter_cleared());
}

#[test]
fn loiter_alt_qland_reached_below_hands_off() {
    let mut qp = available_qp();
    let mut view = LoiterAltQLandEnterView::fw_above();
    view.reached_loiter_target = true;
    view.height_above_next_wp_m = Some(-0.5);
    view.terrain_enabled_in_qland = true;

    let out = loiter_alt_qland_enter(&mut qp, view);
    assert_eq!(out.action, LoiterAltQLandAction::HandoffReachedQland);
    assert!(out.loiter_enter);
    assert!(out.guided_request);
    assert_eq!(out.seed, Some(LoiterAltSeed::NextWp));
    assert_eq!(out.guided_frame, GuidedAltFrame::AboveTerrain);
    assert_eq!(out.mode_reason, Some(MODE_REASON_LOITER_ALT_REACHED_QLAND));
    assert_eq!(qp.poscontrol().state(), PositionControlState::LandDescend);
}

#[test]
fn switch_qland_needs_reached_and_at_or_below() {
    assert!(!switch_qland(Some(5.0), true));
    assert!(!switch_qland(Some(-1.0), false));
    assert!(switch_qland(Some(-1.0), true));
    assert!(switch_qland(None, true));
    assert!(!switch_qland(None, false));
    assert!(!switch_qland(Some(0.0), true));
}

#[test]
fn already_in_a_loiter_requires_fresh_nav() {
    assert!(already_in_a_loiter(true, false));
    assert!(!already_in_a_loiter(true, true));
    assert!(!already_in_a_loiter(false, false));
    assert!(!already_in_a_loiter(false, true));
}

#[test]
fn loiter_alt_qland_stale_nav_uses_current_loc() {
    let mut qp = available_qp();
    let mut view = LoiterAltQLandEnterView::fw_above();
    view.reached_loiter_target = true;
    view.nav_data_stale = true;
    view.height_above_next_wp_m = Some(8.0);

    let out = loiter_alt_qland_enter(&mut qp, view);
    assert_eq!(out.action, LoiterAltQLandAction::StayLoiter);
    assert_eq!(out.seed, Some(LoiterAltSeed::CurrentLoc));
}
