//! Elevon and V-tail mixing, upstream `Plane::channel_function_mixer` in
//! `ArduPlane/servos.cpp`. FW-018 / vehicle mixing slice.

use ap_math::scalar::constrain_value;

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
}
