//! QLOITER / QLAND / LoiterAltQLand `_enter` — upstream
//! `mode_qloiter.cpp` / `mode_qland.cpp` / `mode_LoiterAltQLand.cpp`.

use ap_quadplane::air_mode::QOption;
use ap_quadplane::landing::{LandDetectView, LAND_COMPLETE_TIMEOUT_MS, LAND_RELAX_MS};
use ap_quadplane::mode_q::{qstabilize_update, QManualUpdateView};
use ap_quadplane::mode_qland::{
    already_in_a_loiter, landing_descent_rate_ms, linear_interpolate, loiter_alt_qland_enter,
    loiter_alt_qland_navigate, loiter_target_stale, pos_before_land_final,
    qland_completeness_counts, qland_completeness_has, qland_enter, qland_run,
    qland_surfaces_unique_names, qland_update, qloiter_enter, qloiter_run, qloiter_update,
    switch_qland, GuidedAltFrame, LoiterAltQLandAction, LoiterAltQLandEnterView,
    LoiterAltQLandNavView, LoiterAltSeed, QLandEnterState, QLandEnterView, QLandFamily,
    QLandPortStatus, QLandRunAction, QLandRunView, QLoiterClimb, QLoiterEnterState,
    QLoiterEnterView, QLoiterRunAction, QLoiterRunView, LOITER_REINIT_MS, MODE_LOITER_ALT_QLAND,
    MODE_QLAND, MODE_QLOITER, MODE_REASON_LOITER_ALT_IN_VTOL, MODE_REASON_LOITER_ALT_REACHED_QLAND,
    PRECLAND_TIMEOUT_MS, QLAND_CPP_SURFACES, Q_LAND_FINAL_SPD_DEFAULT_MS, Q_RTL_ALT_DEFAULT_M,
    Q_WP_SPD_DN_DEFAULT_MS,
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

fn mm(v: f32) -> i32 {
    (v * 1000.0) as i32
}

fn latch_land_start(qp: &mut QuadPlane, t0: u32, height_m: f32, timeout_ms: u32) -> u32 {
    assert_ne!(t0, 0);
    assert!(!qp.land_detector(LandDetectView::settled(t0, height_m), timeout_ms));
    let t_land = t0 + LAND_RELAX_MS + 1;
    assert!(!qp.land_detector(LandDetectView::settled(t_land, height_m), timeout_ms));
    assert_eq!(qp.landing_detect().land_start_ms(), t_land);
    t_land
}

#[test]
fn qland_run_uses_qloiter_and_descends() {
    let mut qp = available_qp();
    let mut state = QLandEnterState::new();
    assert!(qland_enter(
        &mut qp,
        QLandEnterView::new(QLoiterEnterView::parked_idle(), 12.5),
        &mut state
    ));
    assert_eq!(qp.poscontrol().state(), PositionControlState::LandDescend);

    let out = qland_run(&mut qp, QLandRunView::descending());
    assert_eq!(out.action, QLandRunAction::Descend);
    assert!(out.used_qloiter);
    assert!(!out.switched_land_final);
    assert!(!out.touchdown_expected);
    assert!(!out.land_complete.complete);
    assert_eq!(qp.poscontrol().state(), PositionControlState::LandDescend);
    // 12.5 m is at/above land_final_alt+6, so the interpolate is WP down-speed.
    assert_eq!(mm(out.descent_rate_ms), mm(Q_WP_SPD_DN_DEFAULT_MS));
    assert_eq!(mm(out.climb_rate_target_ms), mm(-Q_WP_SPD_DN_DEFAULT_MS));
}

#[test]
fn qland_run_switches_land_final_then_can_complete() {
    let mut qp = available_qp();
    let mut state = QLandEnterState::new();
    assert!(qland_enter(
        &mut qp,
        QLandEnterView::parked_idle(),
        &mut state
    ));

    let switched = qland_run(&mut qp, QLandRunView::below_final());
    assert_eq!(switched.action, QLandRunAction::Descend);
    assert!(switched.used_qloiter);
    assert!(switched.switched_land_final);
    assert!(switched.target_position_setup);
    assert!(switched.touchdown_expected);
    assert!(!switched.land_complete.complete);
    assert_eq!(qp.poscontrol().state(), PositionControlState::LandFinal);
    // LAND_FINAL clamps height to Q_LAND_FINAL_ALT, and 3 m is below that,
    // so the interpolate is Q_LAND_FINAL_SPD.
    assert_eq!(
        mm(switched.descent_rate_ms),
        mm(Q_LAND_FINAL_SPD_DEFAULT_MS)
    );
    assert_eq!(
        mm(switched.climb_rate_target_ms),
        mm(-Q_LAND_FINAL_SPD_DEFAULT_MS)
    );

    qp.set_options(QOption::DisableGroundEffectComp as i32);
    let t_land = latch_land_start(&mut qp, 200, 1.0, LAND_COMPLETE_TIMEOUT_MS);
    let complete = qland_run(
        &mut qp,
        QLandRunView::settled_complete(t_land + LAND_COMPLETE_TIMEOUT_MS, 1.0),
    );
    assert!(!complete.touchdown_expected);
    assert!(complete.land_complete.complete);
    assert!(complete.land_complete.disarm);
    assert_eq!(qp.poscontrol().state(), PositionControlState::LandComplete);
}

#[test]
fn qland_run_throttle_wait_skips_descent() {
    let mut qp = available_qp();
    let mut state = QLandEnterState::new();
    assert!(qland_enter(
        &mut qp,
        QLandEnterView::parked_idle(),
        &mut state
    ));
    qp.set_throttle_wait(true);

    let out = qland_run(&mut qp, QLandRunView::below_final());
    assert_eq!(out.action, QLandRunAction::ThrottleWait);
    assert!(out.used_qloiter);
    assert!(!out.switched_land_final);
    assert_eq!(qp.poscontrol().state(), PositionControlState::LandDescend);
}

#[test]
fn landing_descent_rate_clamps_in_final_and_stops_on_reposition() {
    let mut qp = available_qp();
    qp.poscontrol_mut()
        .set_state(PositionControlState::LandFinal);
    let mut view = QLandRunView::descending();
    view.height_above_ground_m = 20.0;
    // LAND_FINAL + high AGL uses land_final_alt for the interpolate → final spd.
    assert_eq!(
        mm(landing_descent_rate_ms(&qp, &view)),
        mm(Q_LAND_FINAL_SPD_DEFAULT_MS)
    );

    qp.poscontrol_mut()
        .set_state(PositionControlState::LandDescend);
    view.height_above_ground_m = 9.0;
    let mid = linear_interpolate(
        Q_LAND_FINAL_SPD_DEFAULT_MS,
        Q_WP_SPD_DN_DEFAULT_MS,
        9.0,
        qp.land_final_alt_m(),
        qp.land_final_alt_m() + 6.0,
    );
    assert_eq!(mm(landing_descent_rate_ms(&qp, &view)), mm(mid));
    assert_eq!(mm(mid), 1000);

    qp.poscontrol_mut().set_pilot_correction(true, true);
    assert_eq!(mm(landing_descent_rate_ms(&qp, &view)), 0);

    assert!(pos_before_land_final(PositionControlState::LandDescend));
    assert!(!pos_before_land_final(PositionControlState::LandFinal));
    assert!(!pos_before_land_final(PositionControlState::LandComplete));
}

#[test]
fn loiter_target_stale_uses_wrapping_sub() {
    assert!(!loiter_target_stale(1_000, 1_000));
    assert!(!loiter_target_stale(1_500, 1_000));
    assert!(loiter_target_stale(1_501, 1_000));
    assert!(loiter_target_stale(0, u32::MAX - 600));
    assert_eq!(LOITER_REINIT_MS, 500);
    assert_eq!(PRECLAND_TIMEOUT_MS, 250);
}

#[test]
fn qloiter_run_poshold_updates_last_loiter() {
    let mut qp = available_qp();
    let view = QLoiterRunView::flying();
    let out = qloiter_run(&mut qp, view);
    assert_eq!(out.action, QLoiterRunAction::PosHold);
    assert!(!out.loiter_reinit);
    assert!(!out.softened);
    assert!(!out.unarmed_reenter);
    assert!(!out.ne_inited);
    assert_eq!(out.last_loiter_ms, 1_000);
    assert_eq!(out.climb, QLoiterClimb::Pilot);
    assert_eq!(mm(out.climb_rate_ms), 0);
    assert!(out.enter.is_none());
}

#[test]
fn qloiter_run_stale_loiter_reinits_and_inits_ne() {
    let mut qp = available_qp();
    let mut view = QLoiterRunView::flying();
    view.last_loiter_ms = 400;
    view.now_ms = 1_000;
    view.ne_active = false;
    view.should_relax = true;
    view.pilot_climb_rate_cms = 150.0;

    let out = qloiter_run(&mut qp, view);
    assert_eq!(out.action, QLoiterRunAction::PosHold);
    assert!(out.loiter_reinit);
    assert!(out.softened);
    assert!(out.ne_inited);
    assert_eq!(out.last_loiter_ms, 1_000);
    assert_eq!(out.climb, QLoiterClimb::Pilot);
    assert_eq!(mm(out.climb_rate_ms), 1_500);
}

#[test]
fn qloiter_run_unarmed_reenters_and_skips_same_tick_reinit() {
    let mut qp = available_qp();
    qp.set_throttle_wait(false);
    let mut view = QLoiterRunView::flying();
    view.motors_armed = false;
    view.last_loiter_ms = 0;
    view.now_ms = 2_000;
    view.enter = QLoiterEnterView::new(0, true, 2_000);

    let out = qloiter_run(&mut qp, view);
    assert_eq!(out.action, QLoiterRunAction::PosHold);
    assert!(out.unarmed_reenter);
    assert!(!out.loiter_reinit);
    assert_eq!(out.last_loiter_ms, 2_000);
    assert!(!qp.throttle_wait());
    let enter = out.enter.expect("unarmed re-enter latches _enter");
    assert!(enter.loiter_target_inited);
    assert_eq!(enter.last_loiter_ms, 2_000);
}

#[test]
fn qloiter_run_guided_takeoff_holds_climb() {
    let mut qp = available_qp();
    let mut view = QLoiterRunView::flying();
    view.in_guided = true;
    view.guided_takeoff = true;
    view.pilot_climb_rate_cms = 200.0;

    let out = qloiter_run(&mut qp, view);
    assert_eq!(out.action, QLoiterRunAction::PosHold);
    assert_eq!(out.climb, QLoiterClimb::GuidedTakeoffHold);
    assert_eq!(mm(out.climb_rate_ms), 0);
}

#[test]
fn qloiter_run_early_outs_skip_poscontrol() {
    let mut qp = available_qp();
    let mut view = QLoiterRunView::flying();
    view.vtol_recovery = true;
    let out = qloiter_run(&mut qp, view);
    assert_eq!(out.action, QLoiterRunAction::VtolRecovery);
    assert_eq!(out.last_loiter_ms, view.last_loiter_ms);

    view.vtol_recovery = false;
    view.tailsitter_in_vtol_transition = true;
    let out = qloiter_run(&mut qp, view);
    assert_eq!(out.action, QLoiterRunAction::FwControllers);

    view.tailsitter_in_vtol_transition = false;
    qp.set_throttle_wait(true);
    let out = qloiter_run(&mut qp, view);
    assert_eq!(out.action, QLoiterRunAction::ThrottleWait);
}

#[test]
fn loiter_alt_qland_navigate_hands_off_when_reached_below() {
    let mut qp = available_qp();
    dirty_for_mode_enter(&mut qp);

    let stay = loiter_alt_qland_navigate(&mut qp, LoiterAltQLandNavView::fw_above());
    assert_eq!(stay.action, LoiterAltQLandAction::StayLoiter);
    assert!(stay.loiter_navigate);
    assert!(stay.mode_reason.is_none());
    assert!(stay.qland.is_none());

    let handoff = loiter_alt_qland_navigate(&mut qp, LoiterAltQLandNavView::reached_below());
    assert_eq!(handoff.action, LoiterAltQLandAction::HandoffReachedQland);
    assert!(!handoff.loiter_navigate);
    assert_eq!(
        handoff.mode_reason,
        Some(MODE_REASON_LOITER_ALT_REACHED_QLAND)
    );
    assert!(handoff.qland.is_some());
    assert!(!qp.throttle_wait());
    assert_eq!(qp.poscontrol().state(), PositionControlState::LandDescend);
}

#[test]
fn qloiter_and_qland_update_delegate_to_qstabilize() {
    let view = QManualUpdateView::flying();
    let expected = qstabilize_update(&view);
    let qloiter = qloiter_update(&view);
    let qland = qland_update(&view);
    assert_eq!(qloiter.nav_roll_cd, expected.nav_roll_cd);
    assert_eq!(qloiter.nav_pitch_cd, expected.nav_pitch_cd);
    assert_eq!(qloiter.path, expected.path);
    assert_eq!(qland.nav_roll_cd, expected.nav_roll_cd);
    assert_eq!(qland.path, expected.path);
}

#[test]
fn qland_cpp_surfaces_closer_catalog() {
    assert!(qland_surfaces_unique_names());
    assert_eq!(QLAND_CPP_SURFACES.len(), 16);
    let (on_main, this_slice, remaining) = qland_completeness_counts();
    assert_eq!(on_main, 7);
    assert_eq!(this_slice, 5);
    assert_eq!(remaining, 4);
    assert!(qland_completeness_has(
        "QLoiter _enter",
        QLandPortStatus::OnMain
    ));
    assert!(qland_completeness_has(
        "QLoiter run QLAND leftover",
        QLandPortStatus::OnMain
    ));
    assert!(qland_completeness_has("QLand run", QLandPortStatus::OnMain));
    assert!(qland_completeness_has(
        "LoiterAltQLand _enter",
        QLandPortStatus::OnMain
    ));
    assert!(qland_completeness_has(
        "QLoiter run poscontrol leftover",
        QLandPortStatus::ThisSlice
    ));
    assert!(qland_completeness_has(
        "LoiterAltQLand navigate",
        QLandPortStatus::ThisSlice
    ));
    assert!(qland_completeness_has(
        "completeness table",
        QLandPortStatus::ThisSlice
    ));
    assert!(qland_completeness_has(
        "QLoiter run precland override",
        QLandPortStatus::Remaining
    ));
    assert!(qland_completeness_has(
        "QLoiter run live COP writes",
        QLandPortStatus::Remaining
    ));
}
