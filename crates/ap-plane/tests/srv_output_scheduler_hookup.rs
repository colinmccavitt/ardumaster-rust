//! SRV output scheduler hookup: speed/stage flap logic and set_servos wiring.

use ap_plane::landing_hookup::ServoOutputState;
use ap_plane::srv_output_hookup::MixingParams;
use ap_plane::srv_output_scheduler_hookup::{
    auto_flap_percent_from_speed, srv_output_scheduler_tick, FlapSpeedParams,
    SrvOutputHookupState, SrvOutputSchedulerInputs,
};

#[test]
fn auto_flap_uses_second_speed_threshold_when_slower() {
    let params = FlapSpeedParams {
        flap_1_speed: 20,
        flap_1_percent: 50,
        flap_2_speed: 15,
        flap_2_percent: 100,
        ..FlapSpeedParams::default()
    };
    assert_eq!(
        auto_flap_percent_from_speed(14.0, &params, false, false),
        100
    );
    assert_eq!(
        auto_flap_percent_from_speed(18.0, &params, false, false),
        50
    );
    assert_eq!(auto_flap_percent_from_speed(25.0, &params, false, false), 0);
}

#[test]
fn takeoff_flap_overrides_speed_schedule() {
    let params = FlapSpeedParams {
        flap_1_speed: 20,
        flap_1_percent: 50,
        takeoff_flap_percent: 75,
        ..FlapSpeedParams::default()
    };
    assert_eq!(
        auto_flap_percent_from_speed(30.0, &params, true, false),
        75
    );
}

#[test]
fn scheduler_tick_applies_elevon_mixing_in_set_servos_path() {
    let mixing = MixingParams {
        mixing_gain: 1.0,
        mixing_offset: 0,
    };
    let mut state = SrvOutputHookupState {
        apply_elevon_mixing: true,
        mixing,
        ..SrvOutputHookupState::default()
    };
    let servos = ServoOutputState {
        aileron_scaled: 1000.0,
        ..ServoOutputState::default()
    };
    let out = srv_output_scheduler_tick(
        servos,
        &mut state,
        &SrvOutputSchedulerInputs {
            mixing,
            flap_params: FlapSpeedParams::default(),
            manual_flap_percent: 0,
            flap_speed_source_ms: 0.0,
            has_auto_flap_schedule: false,
            flight_stage_is_takeoff: false,
            flight_stage_is_land: false,
            apply_elevon_mixing: true,
            apply_vtail_mixing: false,
            dt: 0.02,
            elevator_scaled: 500.0,
        },
    );
    assert_eq!(out.auto_flap_percent, 0);
    assert_eq!(state.registry.output_scaled(ap_servo::function::Function::ELEVON_LEFT), -500.0);
    assert_eq!(state.registry.output_scaled(ap_servo::function::Function::ELEVON_RIGHT), 1500.0);
}
