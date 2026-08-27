use ap_plane::landing_hookup::ServoOutputState;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};
use ap_plane::suppress_throttle_scheduler_hookup::{
    suppress_throttle_scheduler_tick, SuppressThrottleSchedulerInputs,
};

#[test]
fn manual_mode_never_suppresses() {
    let servos = ServoOutputState {
        throttle_scaled: 50.0,
        ..ServoOutputState::default()
    };
    let out = suppress_throttle_scheduler_tick(
        servos,
        &SuppressThrottleSchedulerInputs {
            control_mode: ModeNumber::Manual.as_number(),
            throttle_suppressed: true,
            features: BuildFeatures::default(),
        },
    );
    assert!(!out.applied);
    assert_eq!(out.servos.throttle_scaled, 50.0);
}

#[test]
fn auto_mode_zeros_throttle_when_suppressed() {
    let servos = ServoOutputState {
        throttle_scaled: 75.0,
        ..ServoOutputState::default()
    };
    let out = suppress_throttle_scheduler_tick(
        servos,
        &SuppressThrottleSchedulerInputs {
            control_mode: ModeNumber::Auto.as_number(),
            throttle_suppressed: true,
            features: BuildFeatures::default(),
        },
    );
    assert!(out.applied);
    assert_eq!(out.servos.throttle_scaled, 0.0);
}

#[test]
fn auto_mode_passes_through_when_unsuppressed() {
    let servos = ServoOutputState {
        throttle_scaled: 60.0,
        ..ServoOutputState::default()
    };
    let out = suppress_throttle_scheduler_tick(
        servos,
        &SuppressThrottleSchedulerInputs {
            control_mode: ModeNumber::Auto.as_number(),
            throttle_suppressed: false,
            features: BuildFeatures::default(),
        },
    );
    assert!(!out.applied);
    assert_eq!(out.servos.throttle_scaled, 60.0);
}
