//! SRV PWM publish hookup: registry scaled outputs to pulse widths.

use ap_plane::srv_output_hookup::{set_servos_flaps, FlapDeployInputs};
use ap_plane::srv_pwm_publish_hookup::{
    channel_pwm, configure_channels, srv_pwm_publish_tick,
    sync_pwm_channels_from_registry, SrvPwmPublishInputs, SrvPwmPublishState,
};
use ap_servo::function::Function;
use ap_servo::registry::Registry;

#[test]
fn publish_tick_writes_flap_pwm_from_registry_scaled() {
    let mut reg = Registry::new();
    reg.assign(Function::FLAP, 1 << 0);
    reg.assign(Function::FLAP_AUTO, 1 << 1);
    set_servos_flaps(
        &mut reg,
        FlapDeployInputs {
            manual_flap_percent: 100,
            auto_flap_percent: 0,
            flap_slewrate: 75.0,
            dt: 0.02,
        },
    );

    let mut pwm_state = SrvPwmPublishState::default();
    configure_channels(
        &reg,
        &mut pwm_state,
        &[Function::FLAP, Function::FLAP_AUTO],
    );
    let out = srv_pwm_publish_tick(
        &mut reg,
        &mut pwm_state,
        &SrvPwmPublishInputs::default(),
    );
    assert!(out.ran);
    let flap_pwm = channel_pwm(&pwm_state, Function::FLAP).expect("flap pwm");
    assert!(flap_pwm > 1900, "full flap should be near max, got {flap_pwm}");
}

#[test]
fn sync_pwm_channels_skips_unassigned_functions() {
    let mut reg = Registry::new();
    reg.assign(Function::THROTTLE, 1 << 0);

    let mut pwm_state = SrvPwmPublishState::default();
    sync_pwm_channels_from_registry(&reg, &mut pwm_state);

    assert_eq!(pwm_state.channel_count, 1);
    assert_eq!(pwm_state.channels[0].function, Function::THROTTLE);
}

