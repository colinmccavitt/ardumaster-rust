//! SRV output mapping hookup: elevon mixing and flap/auto-flap stub.

use ap_plane::srv_output_hookup::{
    apply_elevon_mixing, set_servos_flaps, update_flaperons, FlapDeployInputs, MixingParams,
};
use ap_servo::function::Function;
use ap_servo::registry::Registry;

#[test]
fn elevon_mixing_reads_aileron_and_elevator() {
    let mut reg = Registry::new();
    reg.assign(Function::AILERON, 1 << 0);
    reg.assign(Function::ELEVATOR, 1 << 1);
    reg.assign(Function::ELEVON_LEFT, 1 << 2);
    reg.assign(Function::ELEVON_RIGHT, 1 << 3);
    reg.set_output_scaled(Function::AILERON, 1000.0);
    reg.set_output_scaled(Function::ELEVATOR, 500.0);

    apply_elevon_mixing(
        &mut reg,
        MixingParams {
            mixing_gain: 1.0,
            mixing_offset: 0,
        },
    );

    assert_eq!(reg.output_scaled(Function::ELEVON_LEFT), -500.0);
    assert_eq!(reg.output_scaled(Function::ELEVON_RIGHT), 1500.0);
}

#[test]
fn flaperons_use_slew_limited_flap_auto() {
    let mut reg = Registry::new();
    reg.assign(Function::AILERON, 1 << 0);
    reg.assign(Function::FLAP_AUTO, 1 << 1);
    reg.assign(Function::FLAPERON_LEFT, 1 << 2);
    reg.assign(Function::FLAPERON_RIGHT, 1 << 3);
    reg.set_output_scaled(Function::AILERON, 1000.0);
    reg.set_output_scaled(Function::FLAP_AUTO, 50.0);
    reg.set_slew_rate(Function::FLAP_AUTO, 0.0, 100, 0.02);

    update_flaperons(&mut reg);

    assert_eq!(reg.output_scaled(Function::FLAPERON_LEFT), 3250.0);
    assert_eq!(reg.output_scaled(Function::FLAPERON_RIGHT), -1250.0);
}

#[test]
fn manual_flap_overrides_auto_when_larger() {
    let mut reg = Registry::new();
    reg.assign(Function::AILERON, 1 << 0);
    reg.assign(Function::FLAP, 1 << 1);
    reg.assign(Function::FLAP_AUTO, 1 << 2);
    reg.assign(Function::FLAPERON_LEFT, 1 << 3);
    reg.assign(Function::FLAPERON_RIGHT, 1 << 4);
    reg.set_output_scaled(Function::AILERON, 0.0);

    set_servos_flaps(
        &mut reg,
        FlapDeployInputs {
            manual_flap_percent: 80,
            auto_flap_percent: 30,
            flap_slewrate: 75.0,
            dt: 0.02,
        },
    );

    assert_eq!(reg.output_scaled(Function::FLAP), 80.0);
    assert_eq!(reg.output_scaled(Function::FLAP_AUTO), 80.0);
    assert_eq!(reg.slew_entries(), 2);
}

#[test]
fn dspoiler_stub_writes_four_registry_functions_after_elevon_mix() {
    use ap_plane::srv_output_hookup::{update_dspoilers, DspoilerHookupInputs};
    use ap_plane::servo_mix::CrowFlapWeights;

    let mut reg = Registry::new();
    for (idx, func) in [
        Function::ELEVON_LEFT,
        Function::ELEVON_RIGHT,
        Function::RUDDER,
        Function::FLAP_AUTO,
        Function::DSPOILERLEFT1,
        Function::DSPOILERLEFT2,
        Function::DSPOILERRIGHT1,
        Function::DSPOILERRIGHT2,
    ]
    .into_iter()
    .enumerate()
    {
        reg.assign(func, 1_u32 << idx);
    }
    reg.set_output_scaled(Function::ELEVON_LEFT, 0.0);
    reg.set_output_scaled(Function::ELEVON_RIGHT, 0.0);
    reg.set_output_scaled(Function::RUDDER, 1000.0);
    reg.set_output_scaled(Function::FLAP_AUTO, 0.0);

    update_dspoilers(
        &mut reg,
        DspoilerHookupInputs {
            rudder_rate_pct: 100,
            ..DspoilerHookupInputs::default()
        },
    );

    assert_eq!(reg.output_scaled(Function::DSPOILERRIGHT1), 1000.0);
    assert_eq!(reg.output_scaled(Function::DSPOILERRIGHT2), -1000.0);
}

#[test]
fn dspoiler_stub_noops_when_channels_unassigned() {
    use ap_plane::srv_output_hookup::{update_dspoilers, DspoilerHookupInputs};

    let mut reg = Registry::new();
    reg.set_output_scaled(Function::RUDDER, 1000.0);
    update_dspoilers(&mut reg, DspoilerHookupInputs::default());
    assert_eq!(reg.output_scaled(Function::DSPOILERLEFT1), 0.0);
}

