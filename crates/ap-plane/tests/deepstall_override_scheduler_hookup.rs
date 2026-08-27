
//! Deepstall override scheduler hookup wiring.

use ap_landing::deepstall_override::DeepstallOverrideInputs;
use ap_landing::deepstall_stage::DeepstallStage;
use ap_landing::go_around::{LandingFlags, LandingType};
use ap_plane::deepstall_override_scheduler_hookup::{
    deepstall_override_scheduler_tick, DeepstallOverrideSchedulerInputs,
};
use ap_plane::landing_hookup::ServoOutputState;
use ap_plane::landing_loop::LandingContext;

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
fn scheduler_tick_skipped_when_not_in_land_stage() {
    let mut landing = LandingContext {
        flags: LandingFlags {
            in_progress: true,
            ..LandingFlags::default()
        },
        landing_type: LandingType::Deepstall,
        ..LandingContext::default()
    };
    landing.machine.deepstall.stage = DeepstallStage::Land;
    let base = ServoOutputState {
        elevator_pwm: 1600,
        aileron_scaled: 100.0,
        ..ServoOutputState::default()
    };
    let out = deepstall_override_scheduler_tick(
        &mut landing,
        base,
        &DeepstallOverrideSchedulerInputs {
            flight_stage_is_land: false,
            deepstall: deepstall_land(),
        },
    );
    assert!(!out.applied_override);
    assert_eq!(out.servos, base);
}

#[test]
fn scheduler_tick_overrides_elevator_in_deepstall_land() {
    let mut landing = LandingContext {
        flags: LandingFlags {
            in_progress: true,
            ..LandingFlags::default()
        },
        landing_type: LandingType::Deepstall,
        ..LandingContext::default()
    };
    landing.machine.deepstall.stage = DeepstallStage::Land;
    let base = ServoOutputState {
        elevator_pwm: 1500,
        throttle_scaled: 50.0,
        ..ServoOutputState::default()
    };
    let out = deepstall_override_scheduler_tick(
        &mut landing,
        base,
        &DeepstallOverrideSchedulerInputs {
            flight_stage_is_land: true,
            deepstall: deepstall_land(),
        },
    );
    assert!(out.applied_override);
    assert_eq!(out.servos.elevator_pwm, 1900);
    assert_eq!(out.servos.throttle_scaled, 0.0);
}

#[test]
fn scheduler_tick_requests_go_around_when_elevator_missing() {
    let mut landing = LandingContext {
        flags: LandingFlags {
            in_progress: true,
            ..LandingFlags::default()
        },
        landing_type: LandingType::Deepstall,
        ..LandingContext::default()
    };
    landing.machine.deepstall.stage = DeepstallStage::Land;
    let mut ds = deepstall_land();
    ds.elevator_present = false;
    let out = deepstall_override_scheduler_tick(
        &mut landing,
        ServoOutputState::default(),
        &DeepstallOverrideSchedulerInputs {
            flight_stage_is_land: true,
            deepstall: ds,
        },
    );
    assert!(out.request_go_around);
    assert!(landing.flags.commanded_go_around);
    assert!(!out.applied_override);
}
