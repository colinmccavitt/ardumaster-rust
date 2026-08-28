//! QuadPlane landing-detect / GUIDED user-takeoff — upstream
//! `should_relax`, `land_detector`, `check_land_complete`,
//! `check_land_final`, and `do_user_takeoff`.

use ap_quadplane::air_mode::QOption;
use ap_quadplane::landing::{
    LandCompleteView, LandDetectView, LandFinalView, RelaxView, UserTakeoffView,
    LAND_COMPLETE_TIMEOUT_MS, LAND_FINAL_MAX_CHANGE_M, LAND_FINAL_TIMEOUT_MS, LAND_RELAX_MS,
    Q_LAND_ALTCHG_DEFAULT_M, Q_LAND_FINAL_ALT_DEFAULT_M, Q_OPTIONS_DISABLE_GROUND_EFFECT_COMP,
};
use ap_quadplane::poscontrol::PositionControlState;
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

/// Walk `land_detector` so lower-limit then land-start both latch.
///
/// `lower_limit_start_ms == 0` is the idle sentinel, so tests start at a
/// non-zero `t0`. `land_start_ms` only latches once `should_relax` is
/// already true.
fn latch_land_start(qp: &mut QuadPlane, t0: u32, height_m: f32, timeout_ms: u32) -> u32 {
    assert_ne!(t0, 0);
    assert!(!qp.land_detector(LandDetectView::settled(t0, height_m), timeout_ms));
    let t_land = t0 + LAND_RELAX_MS + 1;
    assert!(!qp.land_detector(LandDetectView::settled(t_land, height_m), timeout_ms));
    assert_eq!(qp.landing_detect().land_start_ms(), t_land);
    t_land
}

#[test]
fn landing_detect_defaults_match_upstream() {
    let qp = QuadPlane::new();
    assert_eq!(qp.landing_detect().lower_limit_start_ms(), 0);
    assert_eq!(qp.landing_detect().land_start_ms(), 0);
    assert_eq!(
        (qp.landing_detect().detect_alt_change_m() * 10.0) as i32,
        (Q_LAND_ALTCHG_DEFAULT_M * 10.0) as i32
    );
    assert_eq!(
        qp.land_final_alt_m() as i32,
        Q_LAND_FINAL_ALT_DEFAULT_M as i32
    );
    assert!(!qp.guided_takeoff());
    assert_eq!(Q_OPTIONS_DISABLE_GROUND_EFFECT_COMP, 1 << 13);
    assert_eq!(
        QOption::DisableGroundEffectComp as i32,
        Q_OPTIONS_DISABLE_GROUND_EFFECT_COMP
    );
}

#[test]
fn should_relax_clears_when_motors_are_not_at_lower_limit() {
    let mut qp = available_qp();
    assert!(!qp.should_relax(RelaxView::flying()));
    assert_eq!(qp.landing_detect().lower_limit_start_ms(), 0);
}

#[test]
fn should_relax_true_after_one_second_at_lower_limit() {
    let mut qp = available_qp();
    assert!(!qp.should_relax(RelaxView::lower_limit(1000)));
    assert_eq!(qp.landing_detect().lower_limit_start_ms(), 1000);
    assert!(!qp.should_relax(RelaxView::lower_limit(1000 + LAND_RELAX_MS)));
    assert!(qp.should_relax(RelaxView::lower_limit(1000 + LAND_RELAX_MS + 1)));
}

#[test]
fn should_relax_treats_near_zero_throttle_as_lower_limit() {
    let mut qp = available_qp();
    let view = RelaxView {
        now_ms: 100,
        throttle: 0.0,
        throttle_lower: false,
        throttle_mix_min: false,
    };
    assert!(!qp.should_relax(view));
    let mut later = view;
    later.now_ms = 100 + LAND_RELAX_MS + 1;
    assert!(qp.should_relax(later));
}

#[test]
fn land_detector_false_while_pilot_correction_active() {
    let mut qp = available_qp();
    qp.poscontrol_mut().set_pilot_correction(true, true);
    let view = LandDetectView::settled(LAND_RELAX_MS + 1, 10.0);
    assert!(!qp.land_detector(view, LAND_COMPLETE_TIMEOUT_MS));
    assert_eq!(qp.landing_detect().land_start_ms(), 0);
}

#[test]
fn land_detector_cancels_when_height_moves() {
    let mut qp = available_qp();
    let t_land = latch_land_start(&mut qp, 1_000, 10.0, LAND_COMPLETE_TIMEOUT_MS);
    let moved = LandDetectView::settled(t_land + 100, 10.0 + Q_LAND_ALTCHG_DEFAULT_M + 0.05);
    assert!(!qp.land_detector(moved, LAND_COMPLETE_TIMEOUT_MS));
    assert_eq!(qp.landing_detect().land_start_ms(), 0);
}

#[test]
fn land_detector_true_after_timeout_and_extra_lower_limit() {
    let mut qp = available_qp();
    let t0 = 500;
    let t_land = latch_land_start(&mut qp, t0, 4.0, LAND_COMPLETE_TIMEOUT_MS);
    let ready = t_land + LAND_COMPLETE_TIMEOUT_MS;
    assert!(qp.land_detector(LandDetectView::settled(ready, 4.0), LAND_COMPLETE_TIMEOUT_MS));
}

#[test]
fn check_land_complete_idle_unless_land_final() {
    let mut qp = available_qp();
    qp.poscontrol_mut()
        .set_state(PositionControlState::LandDescend);
    let t0 = 200;
    let t_land = latch_land_start(&mut qp, t0, 1.0, LAND_COMPLETE_TIMEOUT_MS);
    let ready = t_land + LAND_COMPLETE_TIMEOUT_MS;
    let result = qp.check_land_complete(LandCompleteView::qland(ready, 1.0));
    assert!(!result.complete);
    assert!(!result.state_complete);
    assert_eq!(qp.poscontrol().state(), PositionControlState::LandDescend);
}

#[test]
fn check_land_complete_disarms_on_qland() {
    let mut qp = available_qp();
    qp.poscontrol_mut()
        .set_state(PositionControlState::LandFinal);
    let t_land = latch_land_start(&mut qp, 200, 1.0, LAND_COMPLETE_TIMEOUT_MS);
    let result = qp.check_land_complete(LandCompleteView::qland(
        t_land + LAND_COMPLETE_TIMEOUT_MS,
        1.0,
    ));
    assert!(result.complete);
    assert!(result.state_complete);
    assert!(result.disarm);
    assert!(!result.spool_shutdown);
    assert_eq!(qp.poscontrol().state(), PositionControlState::LandComplete);
}

#[test]
fn check_land_complete_payload_place_shuts_down_without_complete() {
    let mut qp = available_qp();
    qp.poscontrol_mut()
        .set_state(PositionControlState::LandFinal);
    let t_land = latch_land_start(&mut qp, 200, 1.0, LAND_COMPLETE_TIMEOUT_MS);
    let mut view = LandCompleteView::qland(t_land + LAND_COMPLETE_TIMEOUT_MS, 1.0);
    view.payload_place = true;
    view.in_auto = true;
    let result = qp.check_land_complete(view);
    assert!(!result.complete);
    assert!(result.state_complete);
    assert!(result.spool_shutdown);
    assert!(!result.disarm);
}

#[test]
fn check_land_complete_auto_continue_skips_disarm() {
    let mut qp = available_qp();
    qp.poscontrol_mut()
        .set_state(PositionControlState::LandFinal);
    let t_land = latch_land_start(&mut qp, 200, 1.0, LAND_COMPLETE_TIMEOUT_MS);
    let mut view = LandCompleteView::qland(t_land + LAND_COMPLETE_TIMEOUT_MS, 1.0);
    view.in_auto = true;
    view.continue_after_land = true;
    let result = qp.check_land_complete(view);
    assert!(result.complete);
    assert!(!result.disarm);
}

#[test]
fn check_land_final_true_when_agl_stable_below_final_alt() {
    let mut qp = available_qp();
    qp.set_land_final_alt_m(Q_LAND_FINAL_ALT_DEFAULT_M);
    let view = LandFinalView {
        detect: LandDetectView::settled(0, 10.0),
        height_above_ground_m: 3.0,
    };
    assert!(3.0 < Q_LAND_FINAL_ALT_DEFAULT_M);
    assert!(3.0 < LAND_FINAL_MAX_CHANGE_M);
    assert!(qp.check_land_final(view));
}

#[test]
fn check_land_final_falls_through_to_long_detector() {
    let mut qp = available_qp();
    let t0 = 300;
    let t_land = latch_land_start(&mut qp, t0, 20.0, LAND_FINAL_TIMEOUT_MS);
    let ready = t_land + LAND_FINAL_TIMEOUT_MS;
    let still_high = LandFinalView {
        detect: LandDetectView::settled(ready, 20.0),
        height_above_ground_m: 20.0,
    };
    assert!(qp.check_land_final(still_high));
}

#[test]
fn do_user_takeoff_rejects_non_guided_unarmed_or_flying() {
    let mut qp = available_qp();
    let mut view = UserTakeoffView::armed_guided(10.0);
    view.in_guided = false;
    assert!(!qp.do_user_takeoff(view).accepted);

    view = UserTakeoffView::armed_guided(10.0);
    view.armed_and_safety_off = false;
    assert!(!qp.do_user_takeoff(view).accepted);

    view = UserTakeoffView::armed_guided(10.0);
    view.is_flying = true;
    assert!(!qp.do_user_takeoff(view).accepted);
    assert!(!qp.guided_takeoff());
}

#[test]
fn do_user_takeoff_sets_guided_takeoff_and_clears_wait() {
    let mut qp = available_qp();
    qp.set_guided_wait_takeoff(true);
    let result = qp.do_user_takeoff(UserTakeoffView::armed_guided(12.5));
    assert!(result.accepted);
    assert!(result.vtol_loiter);
    assert!(result.takeoff_expected);
    assert_eq!((result.climb_m * 10.0) as i32, 125);
    assert!(qp.guided_takeoff());
    assert!(!qp.guided_wait_takeoff());
}

#[test]
fn do_user_takeoff_skips_takeoff_expected_when_ground_effect_disabled() {
    let mut qp = available_qp();
    qp.set_options(Q_OPTIONS_DISABLE_GROUND_EFFECT_COMP);
    assert!(qp.option_is_set(QOption::DisableGroundEffectComp));
    let result = qp.do_user_takeoff(UserTakeoffView::armed_guided(5.0));
    assert!(result.accepted);
    assert!(!result.takeoff_expected);
}
