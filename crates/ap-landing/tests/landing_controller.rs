//! AP_Landing-level dispatch wiring.

use ap_landing::deepstall_stage::DeepstallStage;
use ap_landing::go_around::{LandingFlags, LandingType};
use ap_landing::landing_controller::{
    constrain_roll, get_target_airspeed_cm, get_target_altitude_location, is_complete,
    is_expecting_impact, is_flaring, is_flying_forward, is_ground_steering_allowed, is_on_approach,
    is_on_final, is_throttle_suppressed, TargetAirspeedInputs,
};
use ap_landing::slope_stage::{LandingAirspeedParams, SlopeStage};
use ap_math::location::Location;

fn landing() -> LandingFlags {
    LandingFlags {
        in_progress: true,
        commanded_go_around: false,
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
fn deepstall_never_flares() {
    let flags = landing();
    assert!(!is_flaring(
        &flags,
        LandingType::Deepstall,
        SlopeStage::Final,
    ));
}

#[test]
fn slope_flares_only_in_final() {
    let flags = landing();
    assert!(!is_flaring(
        &flags,
        LandingType::StandardGlideSlope,
        SlopeStage::Approach,
    ));
    assert!(is_flaring(
        &flags,
        LandingType::StandardGlideSlope,
        SlopeStage::Final,
    ));
}

#[test]
fn deepstall_on_approach_only_in_land_stage() {
    let flags = landing();
    assert!(!is_on_approach(
        &flags,
        LandingType::Deepstall,
        SlopeStage::Normal,
        DeepstallStage::Approach,
    ));
    assert!(is_on_approach(
        &flags,
        LandingType::Deepstall,
        SlopeStage::Normal,
        DeepstallStage::Land,
    ));
}

#[test]
fn slope_on_approach_covers_approach_and_preflare() {
    let flags = landing();
    assert!(is_on_approach(
        &flags,
        LandingType::StandardGlideSlope,
        SlopeStage::Approach,
        DeepstallStage::FlyToLanding,
    ));
    assert!(is_on_approach(
        &flags,
        LandingType::StandardGlideSlope,
        SlopeStage::Preflare,
        DeepstallStage::FlyToLanding,
    ));
    assert!(!is_on_approach(
        &flags,
        LandingType::StandardGlideSlope,
        SlopeStage::Final,
        DeepstallStage::FlyToLanding,
    ));
}

#[test]
fn deepstall_disables_ground_steering() {
    let flags = landing();
    assert!(!is_ground_steering_allowed(
        &flags,
        LandingType::Deepstall,
        SlopeStage::Approach,
    ));
}

#[test]
fn deepstall_target_location_is_the_landing_point() {
    let flags = landing();
    let point = Location::new(-35_000_000, 149_000_000);
    assert_eq!(
        get_target_altitude_location(&flags, LandingType::Deepstall, point),
        Some(point),
    );
    assert!(get_target_altitude_location(
        &flags,
        LandingType::StandardGlideSlope,
        point,
    )
    .is_none());
}

#[test]
fn deepstall_target_airspeed_uses_pre_flare_on_approach() {
    let flags = landing();
    let inp = airspeed_inputs();
    assert_eq!(
        get_target_airspeed_cm(
            &flags,
            LandingType::Deepstall,
            SlopeStage::Normal,
            DeepstallStage::Approach,
            &inp,
        ),
        1200,
    );
    assert_eq!(
        get_target_airspeed_cm(
            &flags,
            LandingType::Deepstall,
            SlopeStage::Normal,
            DeepstallStage::FlyToLanding,
            &inp,
        ),
        1500,
    );
}

#[test]
fn not_landing_returns_cruise_airspeed() {
    let flags = LandingFlags::default();
    let inp = airspeed_inputs();
    assert_eq!(
        get_target_airspeed_cm(
            &flags,
            LandingType::Deepstall,
            SlopeStage::Normal,
            DeepstallStage::Land,
            &inp,
        ),
        1500,
    );
}

#[test]
fn throttle_suppressed_in_slope_final_and_deepstall_land() {
    let flags = landing();
    assert!(is_throttle_suppressed(
        &flags,
        LandingType::StandardGlideSlope,
        SlopeStage::Final,
        DeepstallStage::Approach,
    ));
    assert!(is_throttle_suppressed(
        &flags,
        LandingType::Deepstall,
        SlopeStage::Normal,
        DeepstallStage::Land,
    ));
    assert!(!is_throttle_suppressed(
        &flags,
        LandingType::Deepstall,
        SlopeStage::Normal,
        DeepstallStage::Approach,
    ));
}

#[test]
fn deepstall_stops_flying_forward_in_land() {
    let flags = landing();
    assert!(is_flying_forward(
        &flags,
        LandingType::Deepstall,
        DeepstallStage::Approach,
    ));
    assert!(!is_flying_forward(
        &flags,
        LandingType::Deepstall,
        DeepstallStage::Land,
    ));
}

#[test]
fn deepstall_is_never_complete() {
    let flags = landing();
    assert!(!is_complete(
        &flags,
        LandingType::Deepstall,
        SlopeStage::Final,
    ));
    assert!(is_complete(
        &flags,
        LandingType::StandardGlideSlope,
        SlopeStage::Final,
    ));
}

#[test]
fn constrain_roll_only_limits_slope_flare() {
    assert_eq!(
        constrain_roll(LandingType::Deepstall, SlopeStage::Final, 6000, 3000),
        6000,
    );
    assert_eq!(
        constrain_roll(
            LandingType::StandardGlideSlope,
            SlopeStage::Final,
            6000,
            3000,
        ),
        3000,
    );
}

#[test]
fn queries_gate_on_in_progress() {
    assert!(!is_on_final(
        &LandingFlags::default(),
        LandingType::StandardGlideSlope,
        SlopeStage::Final,
    ));
    assert!(!is_expecting_impact(
        &LandingFlags::default(),
        LandingType::StandardGlideSlope,
        SlopeStage::Final,
    ));
}
