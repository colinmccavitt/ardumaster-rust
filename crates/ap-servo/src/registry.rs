//! The output-function registry, upstream `SRV_Channels`. COP-030.
//!
//! Between "the mixer wants motor 3 at 0.62" and "channel 7 gets a 1620
//! microsecond pulse" sits a level of indirection: a motor number maps to a
//! *function*, and a function maps to whichever channels the operator assigned
//! it to. That is what lets someone move a motor to a different output by
//! changing a parameter rather than rewiring.
//!
//! # Per function, not per channel
//!
//! The registry is indexed by function, and each entry holds a *mask* of
//! channels. One function can drive several channels — two servos on one
//! surface — and writing the function writes all of them. A port that stored a
//! single channel per function would work on every ordinary airframe and fail
//! silently on the ones that need it.
//!
//! Per ADR-0004 this is an owned object rather than the static upstream keeps,
//! so a test can have one without disturbing another.

use crate::function::{Function, NR_AUX_SERVO_FUNCTIONS};

/// A mask of output channels, upstream `SRV_Channel::servo_mask_t`.
pub type ChannelMask = u32;

/// What `get_output_channel_mask` returns for a function this build does not
/// define, upstream `invalid_mask`.
///
/// All ones rather than zero. Zero would read as "no channels", which is a
/// legitimate answer for a function nobody assigned; this is "the question was
/// meaningless", and a caller that ignores the difference will notice, because
/// acting on every channel at once is not subtle.
pub const INVALID_MASK: ChannelMask = ChannelMask::MAX;

/// One function's registry entry, upstream `srv_function`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SrvFunction {
    /// Which channels this function drives.
    pub channel_mask: ChannelMask,
    /// The most recent scaled value written to it.
    pub output_scaled: f32,
}

/// The function registry, upstream `SRV_Channels::functions`.
#[derive(Debug, Clone)]
pub struct Registry {
    functions: [SrvFunction; NR_AUX_SERVO_FUNCTIONS],
    /// Channels whose value was last set as a pulse width rather than a scaled
    /// value, upstream `SRV_Channel::have_pwm_mask`.
    have_pwm_mask: ChannelMask,
}

impl Default for Registry {
    fn default() -> Self {
        Self::new()
    }
}

impl Registry {
    /// An empty registry: no function drives any channel.
    #[must_use]
    pub fn new() -> Self {
        Self {
            functions: [SrvFunction::default(); NR_AUX_SERVO_FUNCTIONS],
            have_pwm_mask: 0,
        }
    }

    /// Assign a function to a set of channels.
    ///
    /// Upstream builds this from the `SERVOn_FUNCTION` parameters in
    /// `update_aux_servo_function`; here it is set directly, because the
    /// parameter sweep is a separate concern from the mapping it produces.
    pub fn assign(&mut self, function: Function, channel_mask: ChannelMask) {
        if let Some(entry) = self.functions.get_mut(usize::from(function.0)) {
            entry.channel_mask = channel_mask;
        }
    }

    /// The channels a function drives, upstream `get_output_channel_mask`.
    ///
    /// [`INVALID_MASK`] for a function this build does not define — which is
    /// not the same as a function nobody assigned, and upstream is careful to
    /// distinguish them.
    #[must_use]
    pub fn output_channel_mask(&self, function: Function) -> ChannelMask {
        if function.valid() {
            self.functions
                .get(usize::from(function.0))
                .map_or(INVALID_MASK, |f| f.channel_mask)
        } else {
            INVALID_MASK
        }
    }

    /// Write a function's scaled value, upstream `set_output_scaled`.
    ///
    /// Also clears this function's channels from the pulse-width mask. That
    /// second effect is the load-bearing one: it records that these channels
    /// are now driven by a scaled value, so whatever converts them to pulses
    /// knows to do the conversion rather than pass through a stale width.
    /// Dropping it leaves a channel holding the last pulse it was given.
    pub fn set_output_scaled(&mut self, function: Function, value: f32) {
        if !function.valid() {
            return;
        }
        let Some(entry) = self.functions.get_mut(usize::from(function.0)) else {
            return;
        };
        entry.output_scaled = value;
        self.have_pwm_mask &= !entry.channel_mask;
    }

    /// Read a function's scaled value, upstream `get_output_scaled`.
    ///
    /// Zero for an undefined function. Note this is a plain zero, not
    /// [`INVALID_MASK`]'s equivalent — upstream distinguishes invalid from
    /// unset for the channel mask but not for the value, and the port
    /// reproduces that rather than tidying it.
    #[must_use]
    pub fn output_scaled(&self, function: Function) -> f32 {
        if function.valid() {
            self.functions
                .get(usize::from(function.0))
                .map_or(0.0, |f| f.output_scaled)
        } else {
            0.0
        }
    }

    /// Channels last written as a pulse width, upstream `have_pwm_mask`.
    #[must_use]
    pub fn have_pwm_mask(&self) -> ChannelMask {
        self.have_pwm_mask
    }

    /// Mark channels as carrying a pulse width, upstream's writes to
    /// `have_pwm_mask` from `set_output_pwm`.
    pub fn set_have_pwm(&mut self, channel_mask: ChannelMask) {
        self.have_pwm_mask |= channel_mask;
    }

    /// Whether every channel in `mask` is digital, upstream
    /// `have_digital_outputs(mask)`.
    ///
    /// An empty mask is false, not vacuously true. Upstream tests `mask != 0`
    /// first, and it matters: "all of no channels are digital" would otherwise
    /// send a vehicle with no motors assigned down the digital path.
    #[must_use]
    pub fn have_digital_outputs(mask: ChannelMask, digital_mask: ChannelMask) -> bool {
        mask != 0 && (mask & digital_mask) == mask
    }
}
