//! Vehicle loop landing servo override hookup.

use ap_landing::deepstall_override::DeepstallOverrideInputs;
use ap_landing::deepstall_stage::DeepstallStage;
use ap_landing::go_around::{LandingFlags, LandingType};
use ap_plane::landing_hookup::{
    landing_servo_hookup, LandingServoHookupInputs, ServoOutputState,
};

fn deepstall_land() -> DeepstallOverrideInputs {
    DeepstallOverrideInputs {
        stage: DeepstallStage::Land,
        stall_entry_ms: 0,
        now_ms: 5000,
        slew_speed: 1.0,
        initial_elevator_pwm: 1500,
        target_elevator_pwm: 1900,
        airspeed_ms: Some(10.0),
        handoff_airspeed_ms: 12.0,
        handoff_lower_limit_ms: 8.0,
        steering_pid: 0.5,
        aileron_scalar: 1.0,
        elevator_present: true,
    }
}

#[test]
fn hookup_skipped_when_not_in_land_stage() {
    let base = ServoOutputState {
        elevator_pwm: 1600,
        aileron_scaled: 100.0,
        ..ServoOutputState::default()
    };
    let inp = LandingServoHookupInputs {
        flight_stage_is_land: false,
        landing_flags: LandingFlags {
            in_progress: true,
            ..LandingFlags::default()
        },
        landing_type: LandingType::Deepstall,
        deepstall_stage: DeepstallStage::Land,
        deepstall: deepstall_land(),
    };
    let r = landing_servo_hookup(base, &inp);
    assert!(!r.applied_override);
    assert_eq!(r.outputs, base);
}

#[test]
fn hookup_skipped_for_slope_landings() {
    let base = ServoOutputState::default();
    let inp = LandingServoHookupInputs {
        flight_stage_is_land: true,
        landing_flags: LandingFlags {
            in_progress: true,
            ..LandingFlags::default()
        },
        landing_type: LandingType::StandardGlideSlope,
        deepstall_stage: DeepstallStage::Land,
        deepstall: deepstall_land(),
    };
    let r = landing_servo_hookup(base, &inp);
    assert!(!r.applied_override);
}

#[test]
fn deepstall_land_overrides_elevator_and_steering() {
    let base = ServoOutputState {
        elevator_pwm: 1500,
        aileron_scaled: 0.0,
        rudder_scaled: 0.0,
        throttle_scaled: 50.0,
    };
    let inp = LandingServoHookupInputs {
        flight_stage_is_land: true,
        landing_flags: LandingFlags {
            in_progress: true,
            ..LandingFlags::default()
        },
        landing_type: LandingType::Deepstall,
        deepstall_stage: DeepstallStage::Land,
        deepstall: deepstall_land(),
    };
    let r = landing_servo_hookup(base, &inp);
    assert!(r.applied_override);
    assert_eq!(r.outputs.elevator_pwm, 1900);
    assert!((r.outputs.aileron_scaled - 2250.0).abs() < 1.0);
    assert_eq!(r.outputs.throttle_scaled, 0.0);
}

#[test]
fn missing_elevator_requests_go_around() {
    let mut ds = deepstall_land();
    ds.elevator_present = false;
    let inp = LandingServoHookupInputs {
        flight_stage_is_land: true,
        landing_flags: LandingFlags {
            in_progress: true,
            ..LandingFlags::default()
        },
        landing_type: LandingType::Deepstall,
        deepstall_stage: DeepstallStage::Land,
        deepstall: ds,
    };
    let r = landing_servo_hookup(ServoOutputState::default(), &inp);
    assert!(r.request_go_around);
    assert!(!r.applied_override);
}
