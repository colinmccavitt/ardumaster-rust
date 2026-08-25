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
    /// Channels whose own `SERVOn_FUNCTION` is not a function this build
    /// defines, upstream `invalid_mask`.
    ///
    /// Not a sentinel. It is a real set of channels, rebuilt by
    /// [`Self::update_aux_servo_function`], and it is what
    /// [`Self::output_channel_mask`] answers with when asked about an
    /// undefined function.
    invalid_mask: ChannelMask,
    /// Whether the masks have been built at least once, upstream
    /// `initialised`.
    initialised: bool,
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
            invalid_mask: 0,
            initialised: false,
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
    /// For a function this build does not define, the answer is
    /// [`Self::invalid_mask`] — the channels whose own assignment is invalid.
    /// That is a genuine set, not a sentinel: asking about a meaningless
    /// function returns the channels that are themselves meaningless, which is
    /// odd but is what upstream does and what callers see.
    ///
    /// Upstream also builds the masks lazily here if they have never been
    /// built. This does not, because the channel assignments are an argument
    /// to [`Self::update_aux_servo_function`] rather than global state it
    /// could reach for — so a caller that has not built them gets the honest
    /// answer for an empty registry instead of a hidden rebuild. See
    /// [`Self::initialised`].
    #[must_use]
    pub fn output_channel_mask(&self, function: Function) -> ChannelMask {
        if function.valid() {
            self.functions
                .get(usize::from(function.0))
                .map_or(self.invalid_mask, |f| f.channel_mask)
        } else {
            self.invalid_mask
        }
    }

    /// The channels whose own `SERVOn_FUNCTION` is not a defined function,
    /// upstream `invalid_mask`.
    #[must_use]
    pub fn invalid_mask(&self) -> ChannelMask {
        self.invalid_mask
    }

    /// Whether the channel masks have been built, upstream `initialised`.
    #[must_use]
    pub fn initialised(&self) -> bool {
        self.initialised
    }

    /// Rebuild every function's channel mask from the channel assignments,
    /// upstream `update_aux_servo_function`.
    ///
    /// `channel_functions[i]` is channel `i`'s `SERVOn_FUNCTION`. A channel
    /// whose function this build does not define is not silently skipped: it
    /// goes into [`Self::invalid_mask`], which is how a misconfigured output
    /// stays visible instead of looking like an unused one.
    ///
    /// Everything is cleared first, so this is a rebuild rather than a merge —
    /// a channel moved from one function to another leaves no trace on the
    /// old one.
    ///
    /// Upstream also calls `aux_servo_function_setup()` per channel here,
    /// which configures that channel's range or angle limits. That belongs to
    /// the channel, not the registry, so it is not done here.
    pub fn update_aux_servo_function(&mut self, channel_functions: &[Function]) {
        for f in &mut self.functions {
            f.channel_mask = 0;
        }
        self.invalid_mask = 0;

        for (i, &function) in channel_functions.iter().enumerate() {
            if i >= 32 {
                break;
            }
            let bit = 1_u32 << i;
            if !function.valid() {
                self.invalid_mask |= bit;
                continue;
            }
            if let Some(entry) = self.functions.get_mut(usize::from(function.0)) {
                entry.channel_mask |= bit;
            }
        }

        self.initialised = true;
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
