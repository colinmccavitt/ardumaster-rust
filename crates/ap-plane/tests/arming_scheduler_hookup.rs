use ap_plane::arming_scheduler_hookup::{
    arming_scheduler_tick, ArmingSchedulerInputs,
};
use ap_plane::landing_hookup::ServoOutputState;

#[test]
fn disarmed_zeros_throttle() {
    let servos = ServoOutputState {
        throttle_scaled: 75.0,
        ..ServoOutputState::default()
    };
    let out = arming_scheduler_tick(
        servos,
        &ArmingSchedulerInputs { soft_armed: false },
    );
    assert!(out.applied);
    assert_eq!(out.servos.throttle_scaled, 0.0);
    assert_eq!(out.servos.aileron_scaled, servos.aileron_scaled);
}

#[test]
fn armed_passes_throttle_through() {
    let servos = ServoOutputState {
        throttle_scaled: 75.0,
        ..ServoOutputState::default()
    };
    let out = arming_scheduler_tick(
        servos,
        &ArmingSchedulerInputs { soft_armed: true },
    );
    assert!(!out.applied);
    assert_eq!(out.servos.throttle_scaled, 75.0);
}
