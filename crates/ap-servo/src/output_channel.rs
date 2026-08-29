//! A channel's runtime output state, upstream `SRV_Channel`'s mutable half.
//! COP-030.
//!
//! [`crate::ServoChannel`] is the configuration — the `SERVOn_*` parameters and
//! the conversion they define. This is what changes every iteration: the pulse
//! currently on the wire, and the two things that can stop the normal
//! conversion from deciding it. Loading those parameters from storage is
//! [`crate::upgrade`]; sending the pulse is [`crate::push`].
//!
//! # Three ways a pulse gets decided
//!
//! In priority order, which is the part worth getting right:
//!
//! 1. **A direct pulse write wins.** Once something has called
//!    [`OutputChannel::set_output_pwm`], that channel is in the registry's
//!    pulse-width mask and [`OutputChannel::calc_pwm`] leaves it alone
//!    entirely — even for an emergency stop. Upstream notes that as a known
//!    wart rather than a design, and says why it is awkward to fix: E-stopping
//!    such a channel would have to stop it to `SERVOn_MIN` rather than
//!    `MOT_PWM_MIN`, which is the wrong value on a multirotor.
//! 2. **Emergency stop beats an override.** For a function an E-stop applies
//!    to, the scaled value is forced to zero and the override is bypassed.
//! 3. **An override beats the normal path.** Otherwise, an active override
//!    keeps whatever it set.
//!
//! Collapsing any two of those changes what an emergency stop can reach.

use crate::function::Function;
use crate::ServoChannel;

/// The registry-level state `calc_pwm` consults.
///
/// Static on upstream; passed explicitly here per ADR-0004, which also makes
/// the interaction between the two visible at the call site rather than
/// implied.
#[derive(Debug, Clone, Copy, Default)]
pub struct OutputContext {
    /// Channels whose pulse was set directly rather than computed. Upstream
    /// `SRV_Channel::have_pwm_mask`.
    pub have_pwm_mask: u32,
    /// Whether an emergency stop is active. Upstream
    /// `SRV_Channels::emergency_stop`.
    pub emergency_stop: bool,
}

/// One output channel's mutable state, upstream `SRV_Channel`.
#[derive(Debug, Clone, Copy)]
pub struct OutputChannel {
    /// The `SERVOn_*` configuration and the conversion it defines.
    pub config: ServoChannel,
    /// What this channel is for.
    pub function: Function,
    /// Which output this is, zero-based. Indexes the masks.
    pub ch_num: u8,
    output_pwm: u16,
    override_active: bool,
}

impl OutputChannel {
    /// A channel with no pulse set and no override.
    #[must_use]
    pub fn new(config: ServoChannel, function: Function, ch_num: u8) -> Self {
        Self {
            config,
            function,
            ch_num,
            output_pwm: 0,
            override_active: false,
        }
    }

    /// The pulse currently on the wire, upstream `output_pwm`.
    #[must_use]
    pub fn output_pwm(&self) -> u16 {
        self.output_pwm
    }

    /// Whether an override is holding this channel, upstream
    /// `override_active`.
    #[must_use]
    pub fn override_active(&self) -> bool {
        self.override_active
    }

    /// Upstream `set_override`.
    pub fn set_override(&mut self, active: bool) {
        self.override_active = active;
    }

    /// Set the pulse directly, upstream `set_output_pwm`.
    ///
    /// Returns whether it took. An active override refuses an unforced write,
    /// and the caller needs to know, because upstream's version also sets this
    /// channel's bit in the shared pulse-width mask — and only when the write
    /// actually happened.
    pub fn set_output_pwm(&mut self, pwm: u16, force: bool) -> bool {
        if self.override_active && !force {
            return false;
        }
        self.output_pwm = pwm;
        true
    }

    /// Compute the pulse from a scaled value, upstream `calc_pwm`.
    ///
    /// Does nothing at all if this channel is in the context's pulse-width
    /// mask: something wrote a pulse directly and that wins. See the module
    /// docs for why that outranks even an emergency stop.
    pub fn calc_pwm(&mut self, output_scaled: f32, ctx: &OutputContext) {
        if self.ch_num < 32 && (ctx.have_pwm_mask & (1_u32 << self.ch_num)) != 0 {
            return;
        }

        let mut output_scaled = output_scaled;
        let mut force = false;
        if ctx.emergency_stop && self.function.should_e_stop() {
            output_scaled = 0.0;
            force = true;
        }

        if !force && self.override_active {
            return;
        }

        self.output_pwm = self.config.pwm_from_scaled_value(output_scaled);
    }

    /// Read the output as a normalised value, upstream `get_output_norm`.
    ///
    /// Minus one to plus one about the midpoint of the travel — the midpoint,
    /// not the trim. A channel trimmed away from centre reads non-zero at
    /// rest, which is correct: this reports where the surface sits within its
    /// range, not how far it is from where it likes to sit.
    ///
    /// The arithmetic lives on [`ServoChannel::output_norm`], since it depends
    /// only on the configuration and the width.
    #[must_use]
    pub fn output_norm(&self) -> f32 {
        self.config.output_norm(self.output_pwm)
    }

    /// Set a normalised output, upstream `set_output_norm`.
    ///
    /// `-1` to `1` about the mid point. Goes through
    /// [`Self::set_output_pwm`], so it is a direct pulse write and takes the
    /// mask with it — not the scaled path.
    pub fn set_output_norm(&mut self, value: f32) -> bool {
        let scaled = value * f32::from(self.config.high_out);
        self.set_output_pwm(self.config.pwm_from_scaled_value(scaled), false)
    }
}
