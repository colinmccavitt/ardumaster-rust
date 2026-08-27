//! SRV PWM publish hookup for the `set_servos` tick.
//!
//! Upstream `SRV_Channels::calc_pwm` at the end of `Plane::set_servos`.

use ap_servo::function::Function;
use ap_servo::output_channel::OutputChannel;
use ap_servo::registry::Registry;
use ap_servo::{OutputType, ServoChannel, NUM_SERVO_CHANNELS};

fn default_servo_config() -> ServoChannel {
    ServoChannel {
        servo_min: 1000,
        servo_max: 2000,
        servo_trim: 1500,
        reversed: false,
        output_type: OutputType::Range,
        high_out: 100,
    }
}

/// HAL inputs for one PWM publish tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SrvPwmPublishInputs {
    pub emergency_stop: bool,
}

/// Persistent output channel state for PWM publish.
#[derive(Debug, Clone)]
pub struct SrvPwmPublishState {
    pub channels: [OutputChannel; NUM_SERVO_CHANNELS],
    pub channel_count: u8,
}

impl Default for SrvPwmPublishState {
    fn default() -> Self {
        Self {
            channels: core::array::from_fn(|i| {
                OutputChannel::new(
                    default_servo_config(),
                    Function::NONE,
                    u8::try_from(i).expect("channel index fits u8"),
                )
            }),
            channel_count: 0,
        }
    }
}

/// Result of one PWM publish tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SrvPwmPublishOutput {
    pub ran: bool,
}

/// Assign the first `channel_count` entries to functions from the registry mask.
pub fn configure_channels(reg: &Registry, state: &mut SrvPwmPublishState, functions: &[Function]) {
    state.channel_count = u8::try_from(functions.len().min(NUM_SERVO_CHANNELS))
        .expect("function list fits channel table");
    for (idx, function) in functions.iter().enumerate() {
        let ch = &mut state.channels[idx];
        ch.function = *function;
        if reg.function_assigned(*function) {
            ch.config = default_servo_config();
        }
    }
}

/// Publish scaled registry outputs as PWM, upstream `SRV_Channels::calc_pwm`.
#[must_use]
pub fn srv_pwm_publish_tick(
    reg: &mut Registry,
    state: &mut SrvPwmPublishState,
    inp: &SrvPwmPublishInputs,
) -> SrvPwmPublishOutput {
    let count = usize::from(state.channel_count);
    if count == 0 {
        return SrvPwmPublishOutput { ran: false };
    }
    reg.calc_pwm(&mut state.channels[..count], inp.emergency_stop);
    SrvPwmPublishOutput { ran: true }
}

/// Pulse width for the first channel with this function after publish.
#[must_use]
pub fn channel_pwm(state: &SrvPwmPublishState, function: Function) -> Option<u16> {
    let count = usize::from(state.channel_count);
    state.channels[..count]
        .iter()
        .find(|ch| ch.function == function)
        .map(OutputChannel::output_pwm)
}
