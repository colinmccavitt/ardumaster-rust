//! Rangefinder bump orchestration.

use ap_landing::go_around::{LandingFlags, SlopeLandingFlags};
use ap_landing::rangefinder_bump::{
    adjust_landing_slope_for_rangefinder_bump, RangefinderBumpConfig, RangefinderBumpInputs,
    RangefinderBumpState,
};
use ap_landing::slope_stage::RangefinderState;
use ap_landing::{SlopeConfig, SlopeInputs};
use ap_math::location::{AltContext, AltFrame, Location};

fn bump_cfg() -> RangefinderBumpConfig {
    RangefinderBumpConfig {
        shallow_threshold: 1.0,
        steep_threshold_deg: 1.0,
    }
}

fn slope_cfg() -> SlopeConfig {
    SlopeConfig {
        flare_sec: 2.0,
        flare_alt: 3.0,
        flare_effectivness_pct: 50,
    }
}

fn approach_locations() -> (Location, Location, Location) {
    let prev = Location::new_with_alt(0, 0, 10_000, AltFrame::Absolute);
    let mut next = prev;
    next.offset(1000.0, 0.0);
    next.set_alt_cm(0, AltFrame::Absolute);
    let current = prev;
    (prev, next, current)
}

fn slope_inputs() -> SlopeInputs {
    let (prev, next, current) = approach_locations();
    SlopeInputs {
        prev_wp: prev,
        next_wp: next,
        current,
        groundspeed: 20.0,
        land_sinkrate: 1.0,
        alt_ctx: AltContext {
            home_alt_cm: Some(0),
            origin_alt_cm: Some(0),
            terrain_alt_cm: Some(0),
        },
    }
}

fn alt_ctx() -> AltContext {
    AltContext {
        home_alt_cm: Some(0),
        origin_alt_cm: Some(0),
        terrain_alt_cm: Some(0),
    }
}

fn bump_state(slope: f32) -> RangefinderBumpState {
    RangefinderBumpState {
        slope,
        initial_slope: 0.02,
        landing: LandingFlags {
            in_progress: true,
            ..LandingFlags::default()
        },
        slope_flags: SlopeLandingFlags::default(),
        rf: RangefinderState {
            in_use: true,
            correction: 0.0,
            last_stable_correction: 0.0,
        },
    }
}

fn bump_inputs(
    rf: RangefinderState,
    prev: Location,
    next: Location,
    current: Location,
) -> RangefinderBumpInputs {
    RangefinderBumpInputs {
        rf,
        prev_wp: prev,
        next_wp: next,
        current,
        wp_distance_m: 300.0,
        adjusted_altitude_cm: 10_000,
        alt_ctx: alt_ctx(),
    }
}

#[test]
fn bump_skipped_when_rangefinder_not_in_use() {
    let (prev, next, current) = approach_locations();
    let mut state = bump_state(0.05);
    state.rf.in_use = false;
    state.rf.correction = 5.0;
    state.rf.last_stable_correction = 0.0;
    let rf = state.rf;

    let result = adjust_landing_slope_for_rangefinder_bump(
        &bump_cfg(),
        &slope_cfg(),
        &slope_inputs(),
        &mut state,
        &bump_inputs(rf, prev, next, current),
    );
    assert!(!result.recalculated);
    assert!(!result.go_around);
}

#[test]
fn bump_skipped_when_correction_change_is_small() {
    let (prev, next, current) = approach_locations();
    let mut state = bump_state(0.05);
    state.rf.correction = 0.5;
    state.rf.last_stable_correction = 0.0;
    let rf = state.rf;

    let result = adjust_landing_slope_for_rangefinder_bump(
        &RangefinderBumpConfig {
            shallow_threshold: 15.0,
            steep_threshold_deg: 1.0,
        },
        &slope_cfg(),
        &slope_inputs(),
        &mut state,
        &bump_inputs(rf, prev, next, current),
    );
    assert!(!result.recalculated);
}

#[test]
fn positive_correction_recalculates_without_abort() {
    let (prev, next, current) = approach_locations();
    let mut state = bump_state(0.05);
    state.rf.correction = 6.0;
    state.rf.last_stable_correction = 0.0;
    let rf = state.rf;

    let result = adjust_landing_slope_for_rangefinder_bump(
        &bump_cfg(),
        &slope_cfg(),
        &slope_inputs(),
        &mut state,
        &bump_inputs(rf, prev, next, current),
    );
    assert!(result.recalculated);
    assert!(!result.go_around);
    assert!(result.slope_setup.is_some());
    assert_eq!(state.rf.last_stable_correction, 6.0);
}

#[test]
fn steep_negative_correction_commands_go_around_once() {
    let (prev, next, current) = approach_locations();
    let mut state = bump_state(0.05);
    state.initial_slope = 0.0;
    state.rf.correction = -40.0;
    state.rf.last_stable_correction = 0.0;
    let inp = bump_inputs(state.rf, prev, next, current);

    let result = adjust_landing_slope_for_rangefinder_bump(
        &bump_cfg(),
        &slope_cfg(),
        &slope_inputs(),
        &mut state,
        &inp,
    );
    assert!(result.recalculated);
    assert!(result.go_around);
    assert!(state.landing.commanded_go_around);
    assert!(state.slope_flags.has_aborted_due_to_slope_recalc);
    assert!((result.alt_offset - (-40.0)).abs() < 1e-6);

    let mut state2 = state;
    state2.rf.correction = -40.0;
    state2.rf.last_stable_correction = 0.0;
    let rf2 = state2.rf;
    let result2 = adjust_landing_slope_for_rangefinder_bump(
        &bump_cfg(),
        &slope_cfg(),
        &slope_inputs(),
        &mut state2,
        &RangefinderBumpInputs { rf: rf2, ..inp },
    );
    assert!(result2.recalculated);
    assert!(!result2.go_around);
}
