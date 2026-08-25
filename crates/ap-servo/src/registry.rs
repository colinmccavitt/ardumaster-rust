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
use crate::output_channel::{OutputChannel, OutputContext};
use crate::Limit;
use crate::NUM_SERVO_CHANNELS;
use ap_math::scalar::is_positive;

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
    /// Per-function slew limits, upstream's `_slew` linked list.
    slew: [Option<SlewEntry>; MAX_SLEW_ENTRIES],
    /// Loops of override remaining per channel, upstream `override_counter`.
    override_counter: [u16; NUM_SERVO_CHANNELS],
}

/// How many functions may carry a slew limit at once.
///
/// Upstream keeps a heap-allocated linked list with no bound. Plane installs
/// five — throttle, its left and right variants, and the two flap functions —
/// so this has room to spare for a vehicle that wants more.
///
/// Overflow is not a new failure mode. Upstream already has a "cannot record
/// this one" path: `NEW_NOTHROW` returning null, after which it returns
/// without adding the entry and that function simply goes unlimited. A full
/// table does the same thing, reached by a different route.
pub const MAX_SLEW_ENTRIES: usize = 16;

/// One function's slew state, upstream `slew_list`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct SlewEntry {
    func: Function,
    last_scaled_output: f32,
    max_change: f32,
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
            slew: [None; MAX_SLEW_ENTRIES],
            override_counter: [0; NUM_SERVO_CHANNELS],
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

    /// Install or update a slew limit, upstream `set_slew_rate`.
    ///
    /// `slew_rate` is a percentage of `range` per second, so a step may move by
    /// `range * rate * 0.01 * dt`.
    ///
    /// An entry is created even when the rate is zero, which upstream calls out
    /// in a comment. Zero means no limiting, but the entry keeps tracking the
    /// output, so a rate installed later starts slewing from where the output
    /// actually is rather than from wherever it stood when limiting was last
    /// switched off. Without that, enabling a slew limit mid-flight would begin
    /// with a jump — the one thing a slew limit exists to prevent.
    ///
    /// Returns false only for an invalid function or a full table.
    pub fn set_slew_rate(
        &mut self,
        function: Function,
        slew_rate: f32,
        range: u16,
        dt: f32,
    ) -> bool {
        if !function.valid() {
            return false;
        }
        let max_change = f32::from(range) * slew_rate * 0.01 * dt;

        for entry in self.slew.iter_mut().flatten() {
            if entry.func == function {
                entry.max_change = max_change;
                return true;
            }
        }

        let current = self.output_scaled(function);
        for slot in &mut self.slew {
            if slot.is_none() {
                *slot = Some(SlewEntry {
                    func: function,
                    last_scaled_output: current,
                    max_change,
                });
                return true;
            }
        }
        false
    }

    /// Force a function's slew history, upstream `set_slew_last_scaled_output`.
    ///
    /// For a caller that has moved the output by some route the slew limiter
    /// did not see, and wants the next limited step measured from there rather
    /// than from the stale value.
    pub fn set_slew_last_scaled_output(&mut self, function: Function, value: f32) {
        for entry in self.slew.iter_mut().flatten() {
            if entry.func == function {
                entry.last_scaled_output = value;
                return;
            }
        }
    }

    /// Read a function's output through its slew limit, upstream
    /// `get_slew_limited_output_scaled`.
    ///
    /// Read-only, and that is the part worth knowing. It clamps against
    /// `last_scaled_output` without advancing it and without writing the result
    /// back — only [`Self::apply_slew_limits`] does either.
    ///
    /// So two calls in one cycle give the same answer, and a caller who used
    /// this every cycle but never ran `calc_pwm` would clamp forever against a
    /// value that never moves. Both follow from what this is: a question about
    /// what the output *would* be, not a step of the filter.
    ///
    /// The `&self` is load-bearing: folding the peek and the step together is
    /// not a bug this port can express. Upstream's equivalent is a non-const
    /// static method and relies on the author not writing the assignment.
    #[must_use]
    pub fn slew_limited_output_scaled(&self, function: Function) -> f32 {
        if !function.valid() {
            return 0.0;
        }
        let value = self.output_scaled(function);
        for entry in self.slew.iter().flatten() {
            if entry.func == function {
                if !is_positive(entry.max_change) {
                    // Zero or negative reads as disabled. Upstream breaks
                    // rather than continuing the search, which would only
                    // differ if a function could appear twice — it cannot,
                    // because `set_slew_rate` updates in place.
                    break;
                }
                return value.clamp(
                    entry.last_scaled_output - entry.max_change,
                    entry.last_scaled_output + entry.max_change,
                );
            }
        }
        value
    }

    /// Enforce every slew limit and advance the history, upstream the first
    /// half of `calc_pwm`.
    ///
    /// The counterpart to the read-only peek above: this writes the clamped
    /// value back into the function's scaled output, so the limit binds on
    /// everything that reads it afterwards.
    ///
    /// The history advances even when the limit is disabled — upstream's update
    /// sits outside its `is_positive` check, and that is what makes installing
    /// a rate later safe.
    pub fn apply_slew_limits(&mut self) {
        for slot in &mut self.slew {
            let Some(entry) = slot else { continue };
            if !entry.func.valid() {
                continue;
            }
            let Some(f) = self.functions.get_mut(usize::from(entry.func.0)) else {
                continue;
            };
            if is_positive(entry.max_change) {
                f.output_scaled = f.output_scaled.clamp(
                    entry.last_scaled_output - entry.max_change,
                    entry.last_scaled_output + entry.max_change,
                );
            }
            entry.last_scaled_output = f.output_scaled;
        }
    }

    /// Hold a channel at a pulse width for a while, upstream
    /// `set_output_pwm_chan_timeout`.
    ///
    /// The timeout is in milliseconds but the mechanism counts *loops*, so it
    /// is converted with a deliberate round-up: any non-zero request gets at
    /// least one loop rather than being rounded away to nothing. A scripted
    /// override asking for a millisecond on a 2.5 ms loop still happens.
    ///
    /// A `timeout_ms` of zero is documented upstream as clearing the override.
    /// The flag is set true here regardless, which reads as though the channel
    /// is held for the rest of the loop — but `calc_pwm` steps the counters
    /// before it converts anything, sees zero, and clears the flag first. So
    /// the pulse width never reflects the override at all: the recording shows
    /// the output following the scaled value in the very same loop.
    ///
    /// What does survive is the write itself and the mask clearing, so a zero
    /// timeout is a way to push a width and immediately hand the channel back
    /// to its scaled value rather than a very short hold.
    ///
    /// Upstream's `had_pwm` handling is not reproduced literally; see the
    /// comment in the body for why it is a no-op here. Its *intent* holds: a
    /// channel that was not already driven by a pulse width returns to its
    /// scaled value once the override lapses, while one that was stays frozen,
    /// because the pre-override width is not stored anywhere.
    pub fn set_output_pwm_chan_timeout(
        &mut self,
        channels: &mut [OutputChannel],
        chan: usize,
        value: u16,
        timeout_ms: u16,
        loop_period_us: u32,
    ) {
        let Some(counter) = self.override_counter.get_mut(chan) else {
            return;
        };
        let Some(channel) = channels.get_mut(chan) else {
            return;
        };
        if loop_period_us == 0 {
            return;
        }

        // Round up, so a non-zero request is never rounded away.
        // Upstream spells the round-up out as `(x + period - 1) / period`;
        // this is the same arithmetic with the intent on the surface.
        let loop_count = (u32::from(timeout_ms) * 1000).div_ceil(loop_period_us);
        *counter = u16::try_from(loop_count).unwrap_or(u16::MAX);

        channel.set_override(true);
        channel.set_output_pwm(value, true);
        // The pulse-width mask is deliberately left alone.
        //
        // Upstream looks like it does something here: it reads whether the
        // channel already had a width, writes the new one, and clears the bit
        // if it had not. That dance exists only because its
        // `SRV_Channel::set_output_pwm` sets the bit unconditionally, so the
        // clear is undoing a side effect of the line above it. Follow both
        // branches and the mask ends up exactly as it started.
        //
        // Its comment explains the intent, which is worth keeping: a channel
        // that was not already driven by a width returns to its scaled value
        // once the override lapses, while one that was stays frozen, because
        // the pre-override width is not stored anywhere and there is nothing
        // to restore.
        //
        // Here the mask lives on the registry and the channel write does not
        // touch it, so that intent is already satisfied by doing nothing. A
        // transcription of the dance would be dead code that reads as
        // load-bearing — which is how it was written first, and a mutation
        // deleting it changed no test.
    }

    /// Step the override counters, upstream the second half of `calc_pwm`.
    ///
    /// A channel with loops remaining stays overridden and spends one; a
    /// channel at zero has its override cleared. Both happen in the same pass,
    /// so a counter of one buys exactly one more loop.
    fn step_override_counters(&mut self, channels: &mut [OutputChannel]) {
        for (chan, channel) in channels.iter_mut().enumerate() {
            let Some(counter) = self.override_counter.get_mut(chan) else {
                break;
            };
            if *counter == 0 {
                channel.set_override(false);
            } else {
                channel.set_override(true);
                *counter -= 1;
            }
        }
    }

    /// Loops of override left on a channel, for callers that want to know.
    #[must_use]
    pub fn override_counter(&self, chan: usize) -> u16 {
        self.override_counter.get(chan).copied().unwrap_or(0)
    }

    /// How many slew entries are in use, so a caller can tell it has not
    /// silently exhausted the table.
    #[must_use]
    pub fn slew_entries(&self) -> usize {
        self.slew.iter().flatten().count()
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

    /// Claim a channel for a function if nothing else wants either, upstream
    /// `set_aux_channel_default`.
    ///
    /// Returns whether the function ended up assigned. Three outcomes, and the
    /// distinction between the last two is the point:
    ///
    /// - The function already drives some channel: nothing to do, `true`.
    ///   Upstream checks this first, so a second call for the same function is
    ///   a no-op rather than a second assignment.
    /// - The channel is free (holding `k_none`): assigned, `true`.
    /// - The channel already holds a *different* function: refused, `false`.
    ///   Upstream prints the conflict and leaves the existing assignment
    ///   alone — the operator configured that channel deliberately, and a
    ///   default has no business overwriting it.
    ///
    /// The same function on the same channel is `true` without complaint,
    /// because that is not a conflict.
    pub fn set_aux_channel_default(
        &mut self,
        channel_functions: &mut [Function],
        function: Function,
        channel: u8,
    ) -> bool {
        if self.output_channel_mask(function) != 0 {
            return true;
        }

        let Some(current) = channel_functions.get_mut(usize::from(channel)) else {
            return false;
        };

        if *current != Function::NONE {
            return *current == function;
        }

        *current = function;
        if function.valid() && channel < 32 {
            if let Some(entry) = self.functions.get_mut(usize::from(function.0)) {
                entry.channel_mask |= 1_u32 << channel;
            }
        }
        true
    }

    /// Write a pulse to every channel a function drives, upstream
    /// `SRV_Channels::set_output_pwm`.
    ///
    /// Does nothing if the function drives no channels -- upstream checks
    /// `function_assigned` first, so an unassigned function is a silent no-op
    /// rather than a write to nothing.
    ///
    /// A channel held by an override refuses the write, and only the channels
    /// that accepted it enter the pulse-width mask. Marking a refused channel
    /// would make the scaled path skip a channel it should be driving.
    pub fn set_output_pwm(
        &mut self,
        channels: &mut [OutputChannel],
        function: Function,
        value: u16,
    ) {
        if self.output_channel_mask(function) == 0 {
            return;
        }
        for ch in channels.iter_mut() {
            if ch.function != function {
                continue;
            }
            if ch.set_output_pwm(value, false) && ch.ch_num < 32 {
                self.have_pwm_mask |= 1_u32 << ch.ch_num;
            }
        }
    }

    /// Convert every channel's scaled value to a pulse, upstream
    /// `SRV_Channels::calc_pwm`.
    ///
    /// Each channel reads the scaled value of its own function. A channel
    /// whose function this build does not define is left alone: there is no
    /// scaled value to convert, and writing a zero would drive an output on
    /// the strength of a configuration error.
    pub fn calc_pwm(&mut self, channels: &mut [OutputChannel], emergency_stop: bool) {
        // Upstream's order: slew limits first, then the override counters,
        // then the per-channel conversion. The slew pass rewrites the scaled
        // values the conversion is about to read, so it has to come first.
        self.apply_slew_limits();
        self.step_override_counters(channels);

        let ctx = OutputContext {
            have_pwm_mask: self.have_pwm_mask,
            emergency_stop,
        };
        for ch in channels.iter_mut() {
            if !ch.function.valid() {
                continue;
            }
            ch.calc_pwm(self.output_scaled(ch.function), &ctx);
        }
    }

    /// Whether any channel carries this function, upstream
    /// `function_assigned`.
    #[must_use]
    pub fn function_assigned(&self, function: Function) -> bool {
        function.valid() && self.output_channel_mask(function) != 0
    }

    /// The first channel carrying a function, upstream `find_channel`.
    ///
    /// First, not only — a function may drive several channels, and the
    /// callers that use this are asking a question with one answer (what is
    /// this surface doing) rather than commanding all of them.
    #[must_use]
    pub fn find_channel(&self, function: Function) -> Option<usize> {
        if !function.valid() {
            return None;
        }
        let mask = self.output_channel_mask(function);
        if mask == 0 {
            None
        } else {
            Some(mask.trailing_zeros() as usize)
        }
    }

    /// Drive every channel with this function to a named endpoint, upstream
    /// `set_output_limit`.
    ///
    /// Endpoints are resolved per channel, so a reversed channel and an
    /// upright one given the same limit travel in opposite directions — which
    /// is the point: `Min` means the minimum of the *surface's* travel, not
    /// the smaller pulse width.
    pub fn set_output_limit(
        &mut self,
        channels: &mut [OutputChannel],
        function: Function,
        limit: Limit,
    ) {
        if !self.function_assigned(function) {
            return;
        }
        for ch in channels.iter_mut() {
            if ch.function == function {
                let pwm = ch.config.limit_pwm(limit);
                ch.set_output_pwm(pwm, false);
            }
        }
    }

    /// Drive every channel with this function to its trim, upstream
    /// `set_output_to_trim`.
    ///
    /// Note this does not go through `function_assigned` upstream, unlike its
    /// neighbours. The difference is almost never observable: an unassigned
    /// function matches no channel, so the loop does nothing either way.
    ///
    /// Almost. `function_assigned` consults the channel mask, which is rebuilt
    /// by [`Self::update_aux_servo_function`] — so between a channel's function
    /// changing and that rebuild, the guard and the loop disagree, and the
    /// guarded version would skip work the unguarded one does. Reproduced
    /// unguarded, matching upstream, rather than making the port stricter than
    /// the thing it is meant to reproduce.
    pub fn set_output_to_trim(&mut self, channels: &mut [OutputChannel], function: Function) {
        for ch in channels.iter_mut() {
            if ch.function == function {
                let trim = ch.config.servo_trim;
                ch.set_output_pwm(trim, false);
            }
        }
    }

    /// Move the trim of every channel with this function, upstream
    /// `set_trim_to_pwm_for`.
    pub fn set_trim_to_pwm_for(channels: &mut [OutputChannel], function: Function, pwm: u16) {
        for ch in channels.iter_mut() {
            if ch.function == function {
                ch.config.servo_trim = pwm;
            }
        }
    }

    /// Trim every channel with this function to its minimum, upstream
    /// `set_trim_to_min_for`.
    ///
    /// `ignore_reversed` decides which minimum. Honouring the reversal picks
    /// the endpoint the *surface* treats as minimum, which for a reversed
    /// channel is the larger pulse width; ignoring it picks the smaller width
    /// regardless. Callers setting a mechanical rest position want the former;
    /// callers driving a specific pulse want the latter.
    pub fn set_trim_to_min_for(
        channels: &mut [OutputChannel],
        function: Function,
        ignore_reversed: bool,
    ) {
        for ch in channels.iter_mut() {
            if ch.function == function {
                ch.config.servo_trim = if ch.config.reversed && !ignore_reversed {
                    ch.config.servo_max
                } else {
                    ch.config.servo_min
                };
            }
        }
    }

    /// The normalised output of the first channel with this function, upstream
    /// `get_output_norm`.
    ///
    /// Recomputes the channel's pulse width before reading it, so the answer
    /// reflects the scaled value written this cycle rather than the width left
    /// by the last `calc_pwm`. That means this is not a pure read — it writes
    /// the channel — which is worth knowing before calling it in a log path.
    pub fn output_norm(
        &self,
        channels: &mut [OutputChannel],
        function: Function,
        emergency_stop: bool,
    ) -> f32 {
        let Some(chan) = self.find_channel(function) else {
            return 0.0;
        };
        let Some(ch) = channels.get_mut(chan) else {
            return 0.0;
        };
        if function.valid() {
            let ctx = OutputContext {
                have_pwm_mask: self.have_pwm_mask,
                emergency_stop,
            };
            let scaled = self.output_scaled(function);
            ch.calc_pwm(scaled, &ctx);
        }
        ch.output_norm()
    }

    /// The pulse width of the first channel with this function, upstream
    /// `get_output_pwm`.
    ///
    /// Recomputes first, for the same reason and with the same caveat as
    /// [`Self::output_norm`].
    pub fn output_pwm_for(
        &self,
        channels: &mut [OutputChannel],
        function: Function,
        emergency_stop: bool,
    ) -> Option<u16> {
        let chan = self.find_channel(function)?;
        if !function.valid() {
            return None;
        }
        let ch = channels.get_mut(chan)?;
        let ctx = OutputContext {
            have_pwm_mask: self.have_pwm_mask,
            emergency_stop,
        };
        let scaled = self.output_scaled(function);
        ch.calc_pwm(scaled, &ctx);
        Some(ch.output_pwm())
    }

    /// Visit each channel that should be given a failsafe pulse width,
    /// upstream `set_failsafe_pwm` and `set_failsafe_limit`.
    ///
    /// Upstream calls straight into `hal.rcout->set_failsafe_pwm`. That is the
    /// HAL's business, so this reports which channel wants which width and
    /// leaves the writing to whatever owns the outputs.
    pub fn for_each_failsafe_target<F>(
        &self,
        channels: &[OutputChannel],
        function: Function,
        limit: Option<Limit>,
        pwm: u16,
        mut visit: F,
    ) where
        F: FnMut(u8, u16),
    {
        if !self.function_assigned(function) {
            return;
        }
        for ch in channels {
            if ch.function == function {
                let value = match limit {
                    Some(l) => ch.config.limit_pwm(l),
                    None => pwm,
                };
                visit(ch.ch_num, value);
            }
        }
    }

    /// Visit every non-motor channel with its trim, upstream
    /// `setup_failsafe_trim_all_non_motors`.
    ///
    /// Motors are excluded because a failsafe that drove them to trim would
    /// command whatever trim happens to mean for a motor — on a multirotor,
    /// mid-throttle. Leaving them out means their failsafe stays wherever it
    /// was set deliberately.
    pub fn for_each_non_motor_trim<F>(channels: &[OutputChannel], mut visit: F)
    where
        F: FnMut(u8, u16),
    {
        for ch in channels {
            if !ch.function.is_motor() {
                visit(ch.ch_num, ch.config.servo_trim);
            }
        }
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
