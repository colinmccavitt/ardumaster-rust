//! SRV_Channel output mapping: elevon/V-tail mixing and flap/auto-flap.
//!
//! Upstream `Plane::channel_function_mixer`, `flaperon_update`, and
//! `set_servos_flaps` in `ArduPlane/servos.cpp`. FW-018 vehicle mixing slice.

use ap_servo::function::Function;
use ap_servo::registry::Registry;

use crate::servo_mix::{
    channel_function_mixer, dspoiler_outputs, flaperon_outputs, CrowFlapWeights,
    DspoilerInputs, MixerInputs,
};

/// Mixing parameters from `MIXING_GAIN` and `MIXING_OFFSET`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct MixingParams {
    /// Upstream `MIXING_GAIN`.
    pub mixing_gain: f32,
    /// Upstream `MIXING_OFFSET`, percent.
    pub mixing_offset: i8,
}

/// Flap deployment inputs for the `set_servos_flaps` stub.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct FlapDeployInputs {
    /// Manual flap percent from RC, upstream `channel_flap->percent_input()`.
    pub manual_flap_percent: i8,
    /// Auto flap percent from speed/stage logic (caller computes).
    pub auto_flap_percent: i8,
    /// Upstream `FLAP_SLEWRATE`, percent of range per second.
    pub flap_slewrate: f32,
    /// Loop delta, upstream `G_Dt`.
    pub dt: f32,
}

/// Mix two registry functions into two outputs, upstream `channel_function_mixer`.
pub fn mix_channel_functions(
    reg: &mut Registry,
    func1_in: Function,
    func2_in: Function,
    func1_out: Function,
    func2_out: Function,
    params: MixingParams,
) {
    let in1 = reg.output_scaled(func1_in);
    let in2 = reg.output_scaled(func2_in);
    let mixed = channel_function_mixer(MixerInputs {
        in1,
        in2,
        mixing_gain: params.mixing_gain,
        mixing_offset: params.mixing_offset,
    });
    reg.set_output_scaled(func1_out, mixed.out1);
    reg.set_output_scaled(func2_out, mixed.out2);
}

/// Write flaperon outputs from aileron and slew-limited flap-auto, upstream
/// `Plane::flaperon_update`.
pub fn update_flaperons(reg: &mut Registry) {
    let aileron = reg.output_scaled(Function::AILERON);
    let flap_percent = reg.slew_limited_output_scaled(Function::FLAP_AUTO);
    let out = flaperon_outputs(aileron, flap_percent);
    reg.set_output_scaled(Function::FLAPERON_LEFT, out.out1);
    reg.set_output_scaled(Function::FLAPERON_RIGHT, out.out2);
}

/// Merge manual and auto flap, publish flap channels, update flaperons, upstream
/// `Plane::set_servos_flaps` (speed/stage logic stays with the caller).
pub fn set_servos_flaps(reg: &mut Registry, inp: FlapDeployInputs) {
    let mut auto = inp.auto_flap_percent;
    let manual = inp.manual_flap_percent;
    if manual.unsigned_abs() > auto.unsigned_abs() {
        auto = manual;
    }
    reg.set_output_scaled(Function::FLAP_AUTO, f32::from(auto));
    reg.set_output_scaled(Function::FLAP, f32::from(manual));
    let _ = reg.set_slew_rate(Function::FLAP_AUTO, inp.flap_slewrate, 100, inp.dt);
    let _ = reg.set_slew_rate(Function::FLAP, inp.flap_slewrate, 100, inp.dt);
    update_flaperons(reg);
}

/// Elevon mixing pass, upstream the flying-wing branch of `set_servos`.
pub fn apply_elevon_mixing(reg: &mut Registry, params: MixingParams) {
    mix_channel_functions(
        reg,
        Function::AILERON,
        Function::ELEVATOR,
        Function::ELEVON_LEFT,
        Function::ELEVON_RIGHT,
        params,
    );
}

/// V-tail mixing pass, upstream `channel_function_mixer` for rudder/elevator.
pub fn apply_vtail_mixing(reg: &mut Registry, params: MixingParams) {
    mix_channel_functions(
        reg,
        Function::RUDDER,
        Function::ELEVATOR,
        Function::VTAIL_RIGHT,
        Function::VTAIL_LEFT,
        params,
    );
}

/// Inputs for differential spoiler / crow mixing, upstream `Plane::dspoiler_update`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DspoilerHookupInputs {
    /// Upstream `DSPOILER_RUD_RATE`, percent.
    pub rudder_rate_pct: i8,
    /// Upstream `DSPOILER_OPTS` bit for full-span aileron.
    pub full_span_aileron: bool,
    /// Upstream `DSPOILER_AILMTCH`, 0–100.
    pub aileron_matching_pct: i8,
    /// Upstream `DSPOILER_CROW_W1` / `DSPOILER_CROW_W2`.
    pub weights: CrowFlapWeights,
    /// Upstream `DSPOILER_OPTS` progressive crow bit.
    pub progressive_crow: bool,
    /// Crow RC switch disabled — zeroes outer weight.
    pub crow_disabled: bool,
}

impl Default for DspoilerHookupInputs {
    fn default() -> Self {
        Self {
            rudder_rate_pct: 0,
            full_span_aileron: false,
            aileron_matching_pct: 100,
            weights: CrowFlapWeights::default(),
            progressive_crow: false,
            crow_disabled: false,
        }
    }
}

/// Write differential spoiler outputs from elevon/rudder/flap state, upstream
/// `Plane::dspoiler_update`.
pub fn update_dspoilers(reg: &mut Registry, inp: DspoilerHookupInputs) {
    if !reg.function_assigned(Function::DSPOILERLEFT1)
        && !reg.function_assigned(Function::DSPOILERLEFT2)
    {
        return;
    }

    let elevon_left = if reg.function_assigned(Function::ELEVON_LEFT) {
        reg.output_scaled(Function::ELEVON_LEFT)
    } else {
        reg.output_scaled(Function::AILERON)
    };
    let elevon_right = if reg.function_assigned(Function::ELEVON_RIGHT) {
        reg.output_scaled(Function::ELEVON_RIGHT)
    } else {
        reg.output_scaled(Function::AILERON)
    };

    let out = dspoiler_outputs(DspoilerInputs {
        elevon_left,
        elevon_right,
        rudder: reg.output_scaled(Function::RUDDER),
        rudder_rate_pct: inp.rudder_rate_pct,
        full_span_aileron: inp.full_span_aileron,
        aileron_matching_pct: inp.aileron_matching_pct,
        weights: inp.weights,
        flap_percent: reg.slew_limited_output_scaled(Function::FLAP_AUTO),
        progressive_crow: inp.progressive_crow,
        crow_disabled: inp.crow_disabled,
    });

    reg.set_output_scaled(Function::DSPOILERLEFT1, out.outer_left);
    reg.set_output_scaled(Function::DSPOILERLEFT2, out.inner_left);
    reg.set_output_scaled(Function::DSPOILERRIGHT1, out.outer_right);
    reg.set_output_scaled(Function::DSPOILERRIGHT2, out.inner_right);
}

