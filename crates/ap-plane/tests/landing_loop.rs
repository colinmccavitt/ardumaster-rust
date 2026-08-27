//! Vehicle loop landing integration wiring.

use ap_landing::deepstall_stage::{
    DeepstallStage, DeepstallVerifyInputs, DeepstallVerifyState, LOITER_COMPLETE_CD,
};
use ap_landing::go_around::{LandingFlags, LandingType};
use ap_landing::landing_controller::TargetAirspeedInputs;
use ap_landing::landing_state_machine::LandingMachineState;
use ap_landing::slope_stage::{FlareConfig, LandingAirspeedParams, SlopeStage};
use ap_math::location::Location;
use ap_math::vector2::Vector2f;
use ap_plane::landing_loop::{
    auto_land_run, fly_forward_during_land, landing_override_servos,
    landing_target_airspeed_cm, target_altitude_landing_inputs, verify_land_height,
    verify_land_tick, AutoLandRunInputs, LandingContext, VerifyLandVehicleInputs,
};
use ap_plane::target_altitude::{target_altitude, TargetAltitude};

fn landing_ctx() -> LandingContext {
    LandingContext {
        flags: LandingFlags {
            in_progress: true,
            commanded_go_around: false,
        },
        landing_type: LandingType::StandardGlideSlope,
        machine: LandingMachineState::default(),
    }
}

fn verify_inputs() -> VerifyLandVehicleInputs {
    VerifyLandVehicleInputs {
        height_above_target_m: 5.0,
        terrain_correction_m: 1.0,
        sink_rate_ms: 2.0,
        wp_proportion: 0.6,
        is_flying: true,
        rangefinder_in_range: true,
        bearing_error_cd: 500,
        crosstrack_error_m: 1.0,
        nav_data_is_stale: false,
        below_prev_wp: false,
        prev_cmd_is_loiter_to_alt: false,
        crash_detection_enable: false,
        flare_cfg: FlareConfig {
            flare_alt: 3.0,
            flare_sec: 2.0,
            pre_flare_alt: 8.0,
            pre_flare_sec: 0.0,
            pre_flare_airspeed: 12.0,
        },
        deepstall: DeepstallVerifyInputs {
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
        },
    }
}

fn airspeed_inputs() -> TargetAirspeedInputs {
    TargetAirspeedInputs {
        cruise_cm: 1500,
        pre_flare_cm: 1200,
        slope_params: LandingAirspeedParams {
            airspeed_cruise_ms: 15.0,
            airspeed_min_ms: 10.0,
            airspeed_max_ms: 25.0,
            land_airspeed_ms: -1.0,
            pre_flare_airspeed_ms: 12.0,
            wind_comp_pct: 50.0,
            allow_max_airspeed: false,
        },
        head_wind_ms: 0.0,
    }
}

#[test]
fn verify_land_height_subtracts_terrain_correction() {
    let inp = verify_inputs();
    assert!((verify_land_height(&inp) - 4.0).abs() < 1e-6);
}

#[test]
fn verify_land_tick_advances_slope_stage() {
    let mut ctx = landing_ctx();
    let mut inp = verify_inputs();
    inp.height_above_target_m = 20.0;
    inp.terrain_correction_m = 0.0;
    let effects = verify_land_tick(&mut ctx, &inp);
    assert_eq!(ctx.machine.slope_stage, SlopeStage::Approach);
    assert!(!effects.entered_slope_final);
}

#[test]
fn auto_land_run_constrains_roll_in_flare() {
    let mut ctx = landing_ctx();
    ctx.machine.slope_stage = SlopeStage::Final;
    let out = auto_land_run(
        &ctx,
        AutoLandRunInputs {
            nav_roll_cd: 6000,
            level_roll_limit_cd: 4500,
        },
    );
    assert_eq!(out.nav_roll_cd, 4500);
    assert!(out.throttle_suppressed);
}

#[test]
fn deepstall_fly_forward_is_false_in_land_stage() {
    let mut ctx = landing_ctx();
    ctx.landing_type = LandingType::Deepstall;
    ctx.machine.deepstall.stage = DeepstallStage::Land;
    assert!(!fly_forward_during_land(&ctx));
}

#[test]
fn deepstall_override_servos_when_throttle_suppressed() {
    let mut ctx = landing_ctx();
    ctx.landing_type = LandingType::Deepstall;
    ctx.machine.deepstall.stage = DeepstallStage::Land;
    assert!(landing_override_servos(&ctx));
}

#[test]
fn slope_does_not_override_servos() {
    let mut ctx = landing_ctx();
    ctx.machine.slope_stage = SlopeStage::Final;
    assert!(!landing_override_servos(&ctx));
}

#[test]
fn landing_target_airspeed_uses_controller_dispatch() {
    let mut ctx = landing_ctx();
    ctx.machine.slope_stage = SlopeStage::Preflare;
    let cm = landing_target_airspeed_cm(&ctx, &airspeed_inputs());
    assert_eq!(cm, 1200);
}

#[test]
fn target_altitude_inputs_wire_landing_on_approach() {
    let mut ctx = landing_ctx();
    ctx.machine.slope_stage = SlopeStage::Approach;
    let inputs = target_altitude_landing_inputs(&ctx, Location::new(0, 0));
    assert_eq!(
        target_altitude(&inputs, || false),
        TargetAltitude::LandingGlideSlope
    );
}

#[test]
fn deepstall_target_altitude_uses_landing_point() {
    let mut ctx = landing_ctx();
    ctx.landing_type = LandingType::Deepstall;
    ctx.machine.deepstall.stage = DeepstallStage::FlyToLanding;
    let inputs = target_altitude_landing_inputs(&ctx, Location::new(0, 0));
    assert_eq!(
        target_altitude(&inputs, || false),
        TargetAltitude::FromLandingTarget
    );
}

#[test]
fn deepstall_verify_land_tick_reports_breakout() {
    let mut ctx = landing_ctx();
    ctx.landing_type = LandingType::Deepstall;
    ctx.machine.deepstall = DeepstallVerifyState {
        stage: DeepstallStage::EstimateWind,
        loiter_sum_cd: LOITER_COMPLETE_CD - 100,
        last_target_bearing_cd: 0,
    };
    let mut inp = verify_inputs();
    let _ = verify_land_tick(&mut ctx, &inp);
    let effects = verify_land_tick(&mut ctx, &inp);
    assert!(effects.record_breakout_at_current);
    assert_eq!(ctx.machine.deepstall.stage, DeepstallStage::FlyToArc);
}
