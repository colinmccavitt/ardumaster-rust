//! verify_land HAL hookup dispatch wiring.

use ap_landing::deepstall_stage::{
    DeepstallStage, DeepstallVerifyInputs, DeepstallVerifyState, LOITER_COMPLETE_CD,
};
use ap_landing::go_around::LandingType;
use ap_landing::landing_state_machine::{
    slope_transition_from_hal, verify_land_step, LandingMachineState, VerifyLandCommonInputs,
};
use ap_landing::slope_stage::{FlareConfig, SlopeStage, TransitionInputs};
use ap_math::location::Location;
use ap_math::vector2::Vector2f;

fn common() -> VerifyLandCommonInputs {
    VerifyLandCommonInputs {
        height_m: 5.0,
        sink_rate_ms: 2.0,
        wp_proportion: 0.6,
        is_flying: true,
        rangefinder_in_range: true,
    }
}

fn flare_cfg() -> FlareConfig {
    FlareConfig {
        flare_alt: 3.0,
        flare_sec: 2.0,
        pre_flare_alt: 8.0,
        pre_flare_sec: 0.0,
        pre_flare_airspeed: 12.0,
    }
}

fn slope_transition() -> TransitionInputs {
    slope_transition_from_hal(&common(), 500, 1.0, false, false, false, false)
}

fn deepstall_inputs() -> DeepstallVerifyInputs {
    DeepstallVerifyInputs {
        distance_to_landing_m: 50.0,
        distance_to_arc_entry_m: 150.0,
        loiter_radius_m: 100.0,
        loiter_ccw: false,
        reached_loiter: true,
        height_error_m: 1.0,
        target_bearing_cd: 500,
        heading_error_deg: 5.0,
        target_heading_deg: 0.0,
        groundspeed_ne: Vector2f::new(10.0, 0.0),
        current: Location::new(-35_000_000, 149_000_000),
        arc_exit: Location::new(-35_000_000, 149_000_000),
        arc_entry: Location::new(-35_000_000, 149_000_000),
        extended_approach: Location::new(-35_000_000, 149_000_000),
        entry_point: Location::new(-35_000_000, 149_000_000),
    }
}

#[test]
fn slope_hal_transition_maps_common_fields() {
    let c = common();
    let t = slope_transition_from_hal(&c, 800, 2.5, true, true, true, true);
    assert!((t.height - c.height_m).abs() < 1e-6);
    assert!((t.sink_rate - c.sink_rate_ms).abs() < 1e-6);
    assert!((t.wp_proportion - c.wp_proportion).abs() < 1e-6);
    assert_eq!(t.bearing_error_cd, 800);
    assert!((t.crosstrack_error_m - 2.5).abs() < 1e-6);
    assert!(t.nav_data_is_stale);
    assert!(t.below_prev_wp);
    assert!(t.prev_cmd_is_loiter_to_alt);
    assert!(t.rangefinder_in_range);
    assert!(t.is_flying);
    assert!(t.crash_detection_enable);
}

#[test]
fn slope_verify_land_enters_approach_past_halfway() {
    let mut c = common();
    c.wp_proportion = 0.6;
    c.height_m = 20.0;
    let transition = slope_transition_from_hal(&c, 2000, 10.0, false, false, false, false);
    let state = LandingMachineState::default();
    let step = verify_land_step(
        LandingType::StandardGlideSlope,
        state,
        &transition,
        &flare_cfg(),
        &deepstall_inputs(),
    );
    assert_eq!(step.state.slope_stage, SlopeStage::Approach);
    assert!(!step.effects.entered_slope_final);
}

#[test]
fn slope_verify_land_flares_below_altitude() {
    let mut c = common();
    c.height_m = 2.0;
    c.wp_proportion = 0.7;
    let mut transition = slope_transition_from_hal(&c, 500, 1.0, false, false, false, false);
    let state = LandingMachineState {
        slope_stage: SlopeStage::Approach,
        ..Default::default()
    };
    let step = verify_land_step(
        LandingType::StandardGlideSlope,
        state,
        &transition,
        &flare_cfg(),
        &deepstall_inputs(),
    );
    assert_eq!(step.state.slope_stage, SlopeStage::Final);
    assert!(step.effects.entered_slope_final);

    transition.wp_proportion = 0.7;
    let state = step.state;
    let step = verify_land_step(
        LandingType::StandardGlideSlope,
        state,
        &transition,
        &flare_cfg(),
        &deepstall_inputs(),
    );
    assert!(!step.effects.entered_slope_final);
}

#[test]
fn slope_verify_land_preflare_from_approach() {
    let mut c = common();
    c.height_m = 7.0;
    let transition = slope_transition_from_hal(&c, 500, 1.0, false, false, false, false);
    let state = LandingMachineState {
        slope_stage: SlopeStage::Approach,
        ..Default::default()
    };
    let step = verify_land_step(
        LandingType::StandardGlideSlope,
        state,
        &transition,
        &flare_cfg(),
        &deepstall_inputs(),
    );
    assert_eq!(step.state.slope_stage, SlopeStage::Preflare);
    assert!(step.effects.entered_slope_preflare);
}

#[test]
fn deepstall_verify_land_advances_via_hal_dispatch() {
    let mut inp = deepstall_inputs();
    inp.distance_to_landing_m = 150.0;
    let state = LandingMachineState::default();
    let step = verify_land_step(
        LandingType::Deepstall,
        state,
        &slope_transition(),
        &flare_cfg(),
        &inp,
    );
    assert_eq!(step.state.deepstall.stage, DeepstallStage::EstimateWind);
    assert!(!step.effects.rebuild_approach_path);
}

#[test]
fn deepstall_verify_land_reports_breakout_effect() {
    let state = LandingMachineState {
        deepstall: DeepstallVerifyState {
            stage: DeepstallStage::EstimateWind,
            loiter_sum_cd: LOITER_COMPLETE_CD - 100,
            last_target_bearing_cd: 0,
        },
        ..Default::default()
    };
    let step = verify_land_step(
        LandingType::Deepstall,
        state,
        &slope_transition(),
        &flare_cfg(),
        &deepstall_inputs(),
    );
    assert_eq!(step.state.deepstall.stage, DeepstallStage::WaitForBreakout);

    let step = verify_land_step(
        LandingType::Deepstall,
        step.state,
        &slope_transition(),
        &flare_cfg(),
        &deepstall_inputs(),
    );
    assert_eq!(step.state.deepstall.stage, DeepstallStage::FlyToArc);
    assert!(step.effects.record_breakout_at_current);
}

#[test]
fn deepstall_verify_land_rebuilds_path_during_wait() {
    let state = LandingMachineState {
        deepstall: DeepstallVerifyState {
            stage: DeepstallStage::WaitForBreakout,
            loiter_sum_cd: 0,
            last_target_bearing_cd: 0,
        },
        ..Default::default()
    };
    let mut inp = deepstall_inputs();
    inp.heading_error_deg = 30.0;
    inp.height_error_m = 20.0;
    let step = verify_land_step(
        LandingType::Deepstall,
        state,
        &slope_transition(),
        &flare_cfg(),
        &inp,
    );
    assert!(step.effects.rebuild_approach_path);
    assert_eq!(step.state.deepstall.stage, DeepstallStage::WaitForBreakout);
}

#[test]
fn verify_land_leaves_other_type_state_unchanged() {
    let state = LandingMachineState {
        slope_stage: SlopeStage::Approach,
        deepstall: DeepstallVerifyState {
            stage: DeepstallStage::Arc,
            loiter_sum_cd: 100,
            last_target_bearing_cd: 50,
        },
    };
    let step = verify_land_step(
        LandingType::StandardGlideSlope,
        state,
        &slope_transition(),
        &flare_cfg(),
        &deepstall_inputs(),
    );
    assert_eq!(step.state.deepstall.stage, DeepstallStage::Arc);
    assert_eq!(step.state.deepstall.loiter_sum_cd, 100);
}
