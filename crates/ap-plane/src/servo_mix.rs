//! Elevon and V-tail mixing, upstream `Plane::channel_function_mixer` in
//! `ArduPlane/servos.cpp`. FW-018 / vehicle mixing slice.

use ap_math::scalar::{constrain_value, is_negative};

/// Scaled servo inputs and mixing parameters.
#[derive(Debug, Clone, Copy)]
pub struct MixerInputs {
    /// First input channel, scaled −4500..4500.
    pub in1: f32,
    /// Second input channel, scaled −4500..4500.
    pub in2: f32,
    /// Mixing gain, upstream `MIXING_GAIN`.
    pub mixing_gain: f32,
    /// Mixing offset percent, upstream `MIXING_OFFSET`.
    pub mixing_offset: i8,
}

/// Mixed outputs written to the two output channels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MixerOutputs {
    /// First mixed output, scaled −4500..4500.
    pub out1: f32,
    /// Second mixed output, scaled −4500..4500.
    pub out2: f32,
}

/// Elevon or V-tail mixer, upstream `Plane::channel_function_mixer`.
///
/// Operates on scaled values only — trim and limits stay on the channels.
#[must_use]
pub fn channel_function_mixer(inp: MixerInputs) -> MixerOutputs {
    let mut in1 = inp.in1;
    let mut in2 = inp.in2;

    if inp.mixing_offset < 0 {
        in2 *= (100 - i32::from(inp.mixing_offset)) as f32 * 0.01;
    } else if inp.mixing_offset > 0 {
        in1 *= (100 + i32::from(inp.mixing_offset)) as f32 * 0.01;
    }

    let out1 = constrain_value((in2 - in1) * inp.mixing_gain, -4500.0, 4500.0);
    let out2 = constrain_value((in2 + in1) * inp.mixing_gain, -4500.0, 4500.0);
    MixerOutputs { out1, out2 }
}

/// Flaperon mixer, upstream `Plane::flaperon_update`.
///
/// Flaps add equally to both surfaces; aileron differential is preserved by
/// adding flap on the left and subtracting on the right.
#[must_use]
pub fn flaperon_outputs(aileron: f32, flap_percent: f32) -> MixerOutputs {
    let left = constrain_value(aileron + flap_percent * 45.0, -4500.0, 4500.0);
    let right = constrain_value(aileron - flap_percent * 45.0, -4500.0, 4500.0);
    MixerOutputs {
        out1: left,
        out2: right,
    }
}

/// Crow flap weighting for differential spoilers.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct CrowFlapWeights {
    pub weight_outer: i16,
    pub weight_inner: i16,
}

/// Inputs for differential spoiler mixing, upstream `Plane::dspoiler_update`.
#[derive(Debug, Clone, Copy)]
pub struct DspoilerInputs {
    pub elevon_left: f32,
    pub elevon_right: f32,
    /// Scaled rudder output, upstream `k_rudder`.
    pub rudder: f32,
    /// Upstream `DSPOILER_RUD_RATE`, percent.
    pub rudder_rate_pct: i8,
    pub full_span_aileron: bool,
    /// Upstream `crow_flap_aileron_matching`, 0–100.
    pub aileron_matching_pct: i8,
    pub weights: CrowFlapWeights,
    /// Slew-limited flap-auto percent, upstream `k_flap_auto`.
    pub flap_percent: f32,
    pub progressive_crow: bool,
    /// Crow RC switch disabled — zeroes outer weight.
    pub crow_disabled: bool,
}

/// Four differential spoiler outputs, scaled −4500..4500.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DspoilerOutputs {
    pub outer_left: f32,
    pub inner_left: f32,
    pub outer_right: f32,
    pub inner_right: f32,
}

/// Differential spoiler mixer, upstream `Plane::dspoiler_update`.
#[must_use]
pub fn dspoiler_outputs(inp: DspoilerInputs) -> DspoilerOutputs {
    let rudder = inp.rudder * f32::from(inp.rudder_rate_pct) * 0.01;
    let mut outer_left = inp.elevon_left;
    let mut outer_right = inp.elevon_right;
    let mut inner_left = 0.0;
    let mut inner_right = 0.0;

    if inp.full_span_aileron {
        inner_left = inp.elevon_left;
        inner_right = inp.elevon_right;
    }

    if rudder > 0.0 {
        outer_right = constrain_value(outer_right + rudder, -4500.0, 4500.0);
        inner_right = constrain_value(inner_right - rudder, -4500.0, 4500.0);
    } else {
        outer_left = constrain_value(outer_left - rudder, -4500.0, 4500.0);
        inner_left = constrain_value(inner_left + rudder, -4500.0, 4500.0);
    }

    if inp.aileron_matching_pct < 100 {
        let scale = f32::from(inp.aileron_matching_pct) * 0.01;
        if is_negative(inner_left) {
            inner_left *= scale;
        }
        if is_negative(inner_right) {
            inner_right *= scale;
        }
    }

    let mut weight_outer = inp.weights.weight_outer;
    if inp.crow_disabled {
        weight_outer = 0;
    }
    let weight_inner = inp.weights.weight_inner;

    if (weight_outer > 0 || weight_inner > 0) && inp.flap_percent > 0.0 {
        let mut inner_flap = inp.flap_percent;
        let mut outer_flap = inp.flap_percent;
        if inp.progressive_crow {
            inner_flap = constrain_value(inner_flap * 2.0, 0.0, 100.0);
            outer_flap = constrain_value(outer_flap - 50.0, 0.0, 50.0) * 2.0;
        }
        let flap_scale = 0.45;
        outer_left = constrain_value(
            outer_left + outer_flap * f32::from(weight_outer) * flap_scale,
            -4500.0,
            4500.0,
        );
        inner_left = constrain_value(
            inner_left - inner_flap * f32::from(weight_inner) * flap_scale,
            -4500.0,
            4500.0,
        );
        outer_right = constrain_value(
            outer_right + outer_flap * f32::from(weight_outer) * flap_scale,
            -4500.0,
            4500.0,
        );
        inner_right = constrain_value(
            inner_right - inner_flap * f32::from(weight_inner) * flap_scale,
            -4500.0,
            4500.0,
        );
    }

    DspoilerOutputs {
        outer_left,
        inner_left,
        outer_right,
        inner_right,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn differential_mixing_at_unity_gain() {
        let o = channel_function_mixer(MixerInputs {
            in1: 1000.0,
            in2: 500.0,
            mixing_gain: 1.0,
            mixing_offset: 0,
        });
        assert_eq!(o.out1, -500.0);
        assert_eq!(o.out2, 1500.0);
    }

    #[test]
    fn outputs_are_clamped() {
        let o = channel_function_mixer(MixerInputs {
            in1: 0.0,
            in2: 3000.0,
            mixing_gain: 2.0,
            mixing_offset: 0,
        });
        assert_eq!(o.out1, 4500.0);
        assert_eq!(o.out2, 4500.0);
    }

    #[test]
    fn negative_mixing_offset_scales_the_second_input() {
        let base = channel_function_mixer(MixerInputs {
            in1: 1000.0,
            in2: 1000.0,
            mixing_gain: 1.0,
            mixing_offset: 0,
        });
        let scaled = channel_function_mixer(MixerInputs {
            in1: 1000.0,
            in2: 1000.0,
            mixing_gain: 1.0,
            mixing_offset: -50,
        });
        assert_eq!(base.out2, 2000.0);
        assert_eq!(scaled.out2, 2500.0);
    }

    #[test]
    fn flaperons_mix_aileron_and_flaps() {
        let o = flaperon_outputs(1000.0, 50.0);
        assert_eq!(o.out1, 1000.0 + 50.0 * 45.0);
        assert_eq!(o.out1, 3250.0);
        assert_eq!(o.out2, 1000.0 - 50.0 * 45.0);
        assert_eq!(o.out2, -1250.0);
    }

    #[test]
    fn dspoiler_rudder_splits_outer_and_inner() {
        let o = dspoiler_outputs(DspoilerInputs {
            elevon_left: 0.0,
            elevon_right: 0.0,
            rudder: 1000.0,
            rudder_rate_pct: 100,
            full_span_aileron: false,
            aileron_matching_pct: 100,
            weights: CrowFlapWeights {
                weight_outer: 0,
                weight_inner: 0,
            },
            flap_percent: 0.0,
            progressive_crow: false,
            crow_disabled: false,
        });
        assert_eq!(o.outer_right, 1000.0);
        assert_eq!(o.inner_right, -1000.0);
        assert_eq!(o.outer_left, 0.0);
        assert_eq!(o.inner_left, 0.0);
    }

    #[test]
    fn dspoiler_crow_flaps_apply_with_progressive_split() {
        let o = dspoiler_outputs(DspoilerInputs {
            elevon_left: 0.0,
            elevon_right: 0.0,
            rudder: 0.0,
            rudder_rate_pct: 100,
            full_span_aileron: false,
            aileron_matching_pct: 100,
            weights: CrowFlapWeights {
                weight_outer: 100,
                weight_inner: 100,
            },
            flap_percent: 75.0,
            progressive_crow: true,
            crow_disabled: false,
        });
        // inner: 75*2=100; outer: (75-50)*2=50; weights 100 → scale 0.45
        assert_eq!(o.inner_left, -4500.0);
        assert_eq!(o.outer_left, 2250.0);
    }
}
