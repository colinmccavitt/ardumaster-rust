//! RTL-to-QRTL VTOL handoff (upstream `ModeRTL::switch_QRTL` plus its real
//! call-site debounce in `ModeRTL::navigate`).

use ap_math::location::Location;
use ap_math::Ftype;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};
use ap_plane::rtl_mode_hookup::{rtl_mode_switch_qrtl_tick, RtlModeSwitchQrtlInputs};
use ap_quadplane::quadplane_completeness::RtlMode;

fn origin() -> Location {
    Location::new(-35_000_000, 149_000_000)
}

fn north_of(base: Location, metres: Ftype) -> Location {
    let mut loc = base;
    loc.offset(metres, Ftype::from(0));
    loc
}

/// A baseline set of inputs where none of the three OR disjuncts trigger a
/// switch: far from home, loiter target not reached, and well short of the
/// finish line.
fn far_from_home_inp() -> RtlModeSwitchQrtlInputs {
    let prev = origin();
    let next = north_of(prev, 1_000.0);
    let current = north_of(prev, 100.0);
    RtlModeSwitchQrtlInputs {
        control_mode: ModeNumber::Rtl.as_number(),
        features: BuildFeatures::default(),
        rtl_mode: RtlMode::SwitchQrtl,
        rtl_radius_m: 100,
        loiter_radius_m: 60,
        reached_loiter_target: false,
        current_loc: current,
        prev_wp_loc: prev,
        next_wp_loc: next,
        wp_distance_m: 900.0,
        stopping_distance_m: 50.0,
        millis_since_last_mode_change: 5_000,
    }
}

#[test]
fn no_switch_when_nothing_qualifies() {
    let out = rtl_mode_switch_qrtl_tick(&far_from_home_inp());
    assert!(out.applied);
    assert!(out.debounce_elapsed);
    assert_eq!(out.qrtl_radius_m, 100);
    assert!(!out.switch_to_qrtl);
}

#[test]
fn early_out_when_rtl_mode_is_not_switch_qrtl() {
    // QRTL_ALWAYS and VTOL_APPROACH_QRTL are two different, separately
    // handled real code paths; switch_QRTL() itself must return false for
    // either regardless of every other input.
    for other in [
        RtlMode::QrtlAlways,
        RtlMode::VtolApproachQrtl,
        RtlMode::None,
    ] {
        let mut inp = far_from_home_inp();
        inp.rtl_mode = other;
        // Make every other condition maximally "should switch" to prove the
        // early-out is what's suppressing it, not an accident of the other
        // inputs.
        inp.reached_loiter_target = true;
        inp.wp_distance_m = 0.0;
        let out = rtl_mode_switch_qrtl_tick(&inp);
        assert!(out.applied);
        assert!(!out.switch_to_qrtl, "rtl_mode {other:?} must not switch");
        assert_eq!(
            out.qrtl_radius_m, 0,
            "switch_QRTL() never ran for {other:?}"
        );
    }
}

#[test]
fn radius_fallback_uses_loiter_radius_when_rtl_radius_is_zero() {
    let mut inp = far_from_home_inp();
    inp.rtl_radius_m = 0;
    inp.loiter_radius_m = 75;
    let out = rtl_mode_switch_qrtl_tick(&inp);
    assert_eq!(out.qrtl_radius_m, 75);
}

#[test]
fn radius_fallback_not_used_when_rtl_radius_is_nonzero() {
    let mut inp = far_from_home_inp();
    inp.rtl_radius_m = -120;
    inp.loiter_radius_m = 75;
    let out = rtl_mode_switch_qrtl_tick(&inp);
    // abs(rtl_radius_m), independent of loiter_radius_m entirely.
    assert_eq!(out.qrtl_radius_m, 120);
}

#[test]
fn disjunct_reached_loiter_target_alone_triggers_switch() {
    let mut inp = far_from_home_inp();
    inp.reached_loiter_target = true;
    // Keep the other two disjuncts false: far short of the finish line and
    // outside both the radius and stopping distance.
    let out = rtl_mode_switch_qrtl_tick(&inp);
    assert!(out.switch_to_qrtl);
}

#[test]
fn disjunct_past_interval_finish_line_alone_triggers_switch() {
    let mut inp = far_from_home_inp();
    // 1000 m leg; 1100 m along it is past the finish line, but wp_distance_m
    // and reached_loiter_target are left at their far-from-home values.
    inp.current_loc = north_of(inp.prev_wp_loc, 1_100.0);
    assert!(!inp.reached_loiter_target);
    let out = rtl_mode_switch_qrtl_tick(&inp);
    assert!(out.switch_to_qrtl);
}

#[test]
fn disjunct_wp_distance_alone_triggers_switch() {
    let mut inp = far_from_home_inp();
    // qrtl_radius_m (100) MAX stopping_distance_m (50) = 100; go inside it.
    inp.wp_distance_m = 50.0;
    let out = rtl_mode_switch_qrtl_tick(&inp);
    assert_eq!(out.qrtl_radius_m, 100);
    assert!(out.switch_to_qrtl);
}

#[test]
fn distance_gate_uses_max_of_radius_and_stopping_distance_radius_wins() {
    let mut inp = far_from_home_inp();
    inp.rtl_radius_m = 300; // qrtl_radius_m
    inp.stopping_distance_m = 20.0;
    // Between the smaller stopping distance (20) and the larger radius
    // (300): 250 is inside the radius but outside the stopping distance,
    // proving MAX (not MIN) picked the radius.
    inp.wp_distance_m = 250.0;
    let out = rtl_mode_switch_qrtl_tick(&inp);
    assert_eq!(out.qrtl_radius_m, 300);
    assert!(out.switch_to_qrtl);
}

#[test]
fn distance_gate_uses_max_of_radius_and_stopping_distance_stopping_distance_wins() {
    let mut inp = far_from_home_inp();
    inp.rtl_radius_m = 20; // qrtl_radius_m
    inp.stopping_distance_m = 300.0;
    // Between the smaller radius (20) and the larger stopping distance
    // (300): 250 is outside the radius but inside the stopping distance,
    // proving MAX (not MIN) picked the stopping distance.
    inp.wp_distance_m = 250.0;
    let out = rtl_mode_switch_qrtl_tick(&inp);
    assert_eq!(out.qrtl_radius_m, 20);
    assert!(out.switch_to_qrtl);
}

#[test]
fn debounce_suppresses_a_switch_that_would_otherwise_trigger() {
    let mut inp = far_from_home_inp();
    inp.reached_loiter_target = true;
    inp.millis_since_last_mode_change = 200;
    let out = rtl_mode_switch_qrtl_tick(&inp);
    assert!(out.applied);
    assert!(!out.debounce_elapsed);
    assert!(!out.switch_to_qrtl);
    // switch_QRTL() itself never ran: no real radius was computed.
    assert_eq!(out.qrtl_radius_m, 0);
}

#[test]
fn debounce_allows_the_same_switch_once_elapsed() {
    let mut inp = far_from_home_inp();
    inp.reached_loiter_target = true;
    inp.millis_since_last_mode_change = 1_001;
    let out = rtl_mode_switch_qrtl_tick(&inp);
    assert!(out.debounce_elapsed);
    assert!(out.switch_to_qrtl);
}

#[test]
fn debounce_boundary_at_exactly_one_second_does_not_elapse() {
    // Real upstream is a strict `>`, not `>=`.
    let mut inp = far_from_home_inp();
    inp.reached_loiter_target = true;
    inp.millis_since_last_mode_change = 1_000;
    let out = rtl_mode_switch_qrtl_tick(&inp);
    assert!(!out.debounce_elapsed);
    assert!(!out.switch_to_qrtl);
}

#[test]
fn not_rtl_mode_short_circuits_everything() {
    let mut inp = far_from_home_inp();
    inp.control_mode = ModeNumber::Auto.as_number();
    inp.reached_loiter_target = true;
    inp.wp_distance_m = 0.0;
    let out = rtl_mode_switch_qrtl_tick(&inp);
    assert!(!out.applied);
    assert!(!out.debounce_elapsed);
    assert!(!out.switch_to_qrtl);
    assert_eq!(out.qrtl_radius_m, 0);
}
