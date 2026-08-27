//! Landing throttle suppression scheduler hookup.

use ap_plane::landing_hookup::ServoOutputState;
use ap_plane::landing_throttle_scheduler_hookup::{
    landing_throttle_scheduler_tick, LandingThrottleSchedulerInputs,
};

#[test]
fn suppresses_throttle_in_land_when_requested() {
    let base = ServoOutputState {
        throttle_scaled: 75.0,
        aileron_scaled: 100.0,
        ..ServoOutputState::default()
    };
    let out = landing_throttle_scheduler_tick(
        base,
        &LandingThrottleSchedulerInputs {
            flight_stage_is_land: true,
            throttle_suppressed: true,
        },
    );
    assert!(out.applied);
    assert_eq!(out.servos.throttle_scaled, 0.0);
    assert_eq!(out.servos.aileron_scaled, 100.0);
}

#[test]
fn leaves_throttle_outside_land() {
    let base = ServoOutputState {
        throttle_scaled: 75.0,
        ..ServoOutputState::default()
    };
    let out = landing_throttle_scheduler_tick(
        base,
        &LandingThrottleSchedulerInputs {
            flight_stage_is_land: false,
            throttle_suppressed: true,
        },
    );
    assert!(!out.applied);
    assert_eq!(out.servos.throttle_scaled, 75.0);
}

#[test]
fn leaves_throttle_when_not_suppressed() {
    let base = ServoOutputState {
        throttle_scaled: 75.0,
        ..ServoOutputState::default()
    };
    let out = landing_throttle_scheduler_tick(
        base,
        &LandingThrottleSchedulerInputs {
            flight_stage_is_land: true,
            throttle_suppressed: false,
        },
    );
    assert!(!out.applied);
    assert_eq!(out.servos.throttle_scaled, 75.0);
}
