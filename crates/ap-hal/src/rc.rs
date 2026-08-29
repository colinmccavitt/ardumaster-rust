//! RC input and servo output, ported from `AP_HAL/RCInput.h` and `RCOutput.h`.
//!
//! Channel values are PWM microseconds throughout, matching upstream. They are
//! deliberately left as plain `u16` rather than a newtype: unlike timestamps,
//! there is no second unit in play to confuse them with, and upstream's own
//! naming already carries the unit.

use crate::Result;

/// Maximum RC channels the port handles. Upstream varies this by board; 16 is
/// the SITL and common-board value.
pub const MAX_RC_CHANNELS: usize = 16;

/// Receiver input. Upstream `AP_HAL::RCInput`.
pub trait RcInput {
    /// Whether new frame data has arrived since the last call.
    ///
    /// Upstream `new_input()` is destructive — it clears the flag — so calling
    /// it twice returns false the second time even though the data is still
    /// valid. Preserved, because callers depend on the edge rather than the
    /// level.
    fn new_input(&mut self) -> bool;

    /// Number of channels the receiver is providing. Upstream `num_channels()`.
    fn num_channels(&self) -> u8;

    /// PWM microseconds for channel `ch`, or `None` if not provided.
    ///
    /// Upstream `read(uint8_t ch)` returns 0 for an out-of-range channel, which
    /// is indistinguishable from a genuine zero reading. `None` separates them.
    fn read(&self, ch: u8) -> Option<u16>;

    /// Received signal strength, 0-255, or `None` if unknown.
    ///
    /// Upstream `get_rssi()` returns -1 for unknown, encoding absence in the
    /// value. `None` makes that explicit and unmissable.
    fn rssi(&self) -> Option<u8> {
        None
    }

    /// Receiver link quality, 0-100, or `None` if unknown.
    fn link_quality(&self) -> Option<u8> {
        None
    }
}

/// How an output channel is driven. Upstream `AP_HAL::RCOutput::output_mode`.
///
/// The three LED modes are here because upstream puts them here: a NeoPixel
/// string is driven by the same timer hardware as a DShot ESC, so the output
/// layer treats them as modes of one thing rather than as a separate device.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OutputMode {
    /// No output.
    #[default]
    None = 0,
    /// Ordinary servo-rate PWM.
    Normal = 1,
    /// One pulse per loop, sent as soon as the value is known.
    OneShot = 2,
    /// OneShot with the pulse width divided by eight.
    OneShot125 = 3,
    /// A raw duty cycle, for brushed motors.
    Brushed = 4,
    /// Digital, 150 kbit/s.
    DShot150 = 5,
    /// Digital, 300 kbit/s.
    DShot300 = 6,
    /// Digital, 600 kbit/s.
    DShot600 = 7,
    /// Digital, 1200 kbit/s.
    DShot1200 = 8,
    /// A NeoPixel string: DShot timing at 800 kHz, driving LEDs.
    NeoPixel = 9,
    /// ProfiLED: DShot timing with separate clock and data lines.
    ProfiLed = 10,
    /// NeoPixel with RGB rather than GRB ordering.
    NeoPixelRgb = 11,
}

/// Servo and motor output. Upstream `AP_HAL::RCOutput`.
pub trait RcOutput {
    /// Select how the channels in `chmask` are driven, upstream
    /// `set_output_mode`.
    ///
    /// Defaulted to a no-op: a backend that only does ordinary PWM has nothing
    /// to switch, and upstream's base class does the same.
    fn set_output_mode(&mut self, _chmask: u32, _mode: OutputMode) {}

    /// Set the update rate, in Hz, for the channels selected by `chmask`.
    fn set_freq(&mut self, chmask: u32, freq_hz: u16) -> Result<()>;

    /// Write `period_us` to channel `ch`.
    fn write(&mut self, ch: u8, period_us: u16) -> Result<()>;

    /// Read back the last value written to channel `ch`.
    fn read(&self, ch: u8) -> Option<u16>;

    /// Enable output on channel `ch`.
    fn enable_ch(&mut self, ch: u8) -> Result<()>;

    /// Disable output on channel `ch`.
    fn disable_ch(&mut self, ch: u8) -> Result<()>;

    /// Hold writes until [`push`] is called, upstream `cork`.
    ///
    /// Defaulted to a no-op: a backend that writes immediately has nothing
    /// to buffer, and the pair is still visible at the call site so a
    /// frame cannot be half-committed by forgetting one of them.
    fn cork(&mut self) {}

    /// Push buffered writes to the hardware.
    ///
    /// Upstream pairs `cork()`/`push()` so a whole servo frame is written
    /// atomically. Keeping `push` explicit preserves that: partial frames are
    /// visible to the aircraft as mixed-timestamp surface commands.
    fn push(&mut self) {}
}

/// An in-memory [`RcInput`], for tests and SITL bring-up.
///
/// Separate from [`MockRcOutput`] because upstream keeps `RCInput` and
/// `RCOutput` as distinct objects, and the HAL context holds them separately.
/// A single type implementing both would make `read` ambiguous at every call
/// site for no benefit.
#[derive(Debug, Clone, Copy)]
pub struct MockRcInput {
    inputs: [u16; MAX_RC_CHANNELS],
    channels: u8,
    has_new_input: bool,
}

impl Default for MockRcInput {
    #[inline]
    fn default() -> Self {
        Self {
            inputs: [0; MAX_RC_CHANNELS],
            channels: 0,
            has_new_input: false,
        }
    }
}

impl MockRcInput {
    /// An idle receiver.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed a frame of channel values, raising the new-input flag.
    pub fn set_input_frame(&mut self, values: &[u16]) {
        let n = values.len().min(MAX_RC_CHANNELS);
        for (i, v) in values.iter().take(n).enumerate() {
            if let Some(slot) = self.inputs.get_mut(i) {
                *slot = *v;
            }
        }
        self.channels = n as u8;
        self.has_new_input = true;
    }
}

impl RcInput for MockRcInput {
    fn new_input(&mut self) -> bool {
        core::mem::replace(&mut self.has_new_input, false)
    }

    fn num_channels(&self) -> u8 {
        self.channels
    }

    fn read(&self, ch: u8) -> Option<u16> {
        if ch >= self.channels {
            return None;
        }
        self.inputs.get(ch as usize).copied()
    }
}

/// An in-memory [`RcOutput`], for tests and SITL bring-up.
#[derive(Debug, Clone, Copy)]
pub struct MockRcOutput {
    outputs: [u16; MAX_RC_CHANNELS],
    enabled: u32,
    pushes: u32,
    corks: u32,
}

impl Default for MockRcOutput {
    #[inline]
    fn default() -> Self {
        Self {
            outputs: [0; MAX_RC_CHANNELS],
            enabled: 0,
            pushes: 0,
            corks: 0,
        }
    }
}

impl MockRcOutput {
    /// An idle output stage.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// Whether channel `ch` is enabled for output.
    pub fn is_enabled(&self, ch: u8) -> bool {
        if ch as usize >= MAX_RC_CHANNELS {
            return false;
        }
        self.enabled & (1u32 << ch) != 0
    }

    /// How many times [`RcOutput::push`] has been called, so a test can check
    /// that a servo frame was actually committed.
    pub fn push_count(&self) -> u32 {
        self.pushes
    }

    /// How many times [`RcOutput::cork`] has been called.
    pub fn cork_count(&self) -> u32 {
        self.corks
    }
}

impl RcOutput for MockRcOutput {
    fn set_freq(&mut self, _chmask: u32, _freq_hz: u16) -> Result<()> {
        Ok(())
    }

    fn write(&mut self, ch: u8, period_us: u16) -> Result<()> {
        let slot = self
            .outputs
            .get_mut(ch as usize)
            .ok_or(crate::Error::NotPresent)?;
        *slot = period_us;
        Ok(())
    }

    fn read(&self, ch: u8) -> Option<u16> {
        self.outputs.get(ch as usize).copied()
    }

    fn enable_ch(&mut self, ch: u8) -> Result<()> {
        if ch as usize >= MAX_RC_CHANNELS {
            return Err(crate::Error::NotPresent);
        }
        self.enabled |= 1u32 << ch;
        Ok(())
    }

    fn disable_ch(&mut self, ch: u8) -> Result<()> {
        if ch as usize >= MAX_RC_CHANNELS {
            return Err(crate::Error::NotPresent);
        }
        self.enabled &= !(1u32 << ch);
        Ok(())
    }

    fn cork(&mut self) {
        self.corks += 1;
    }

    fn push(&mut self) {
        self.pushes += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Upstream's new_input() clears the flag, so it reports an edge not a
    /// level. Preserved deliberately - callers depend on it.
    #[test]
    fn new_input_is_destructive_like_upstream() {
        let mut rc = MockRcInput::new();
        assert!(!rc.new_input(), "idle receiver has no new input");

        rc.set_input_frame(&[1500, 1500, 1000, 1500]);
        assert!(rc.new_input(), "first read sees the frame");
        assert!(!rc.new_input(), "second read must not see it again");

        // the data is still readable after the flag is consumed
        assert_eq!(rc.read(0), Some(1500));
    }

    /// Upstream returns 0 for an out-of-range channel, which collides with a
    /// genuine zero reading. None separates the two.
    #[test]
    fn out_of_range_channel_is_none_not_zero() {
        let mut rc = MockRcInput::new();
        rc.set_input_frame(&[1500, 0]);

        assert_eq!(rc.read(0), Some(1500));
        assert_eq!(rc.read(1), Some(0), "a real zero reading is Some(0)");
        assert_eq!(rc.read(2), None, "beyond num_channels is None");
        assert_eq!(rc.read(200), None);
        assert_eq!(rc.num_channels(), 2);
    }

    #[test]
    fn output_writes_and_reads_back() {
        let mut rc = MockRcOutput::new();
        rc.write(3, 1750).unwrap();
        assert_eq!(rc.read(3), Some(1750));

        // beyond the channel count is an error, not a silent no-op
        assert!(rc.write(MAX_RC_CHANNELS as u8, 1500).is_err());
    }

    #[test]
    fn channel_enable_is_tracked() {
        let mut rc = MockRcOutput::new();
        assert!(!rc.is_enabled(2));
        rc.enable_ch(2).unwrap();
        assert!(rc.is_enabled(2));
        rc.disable_ch(2).unwrap();
        assert!(!rc.is_enabled(2));
        assert!(rc.enable_ch(99).is_err());
    }

    /// push() is explicit so a servo frame commits atomically; a test can
    /// assert the frame was actually committed rather than just written.
    #[test]
    fn push_commits_a_frame() {
        let mut rc = MockRcOutput::new();
        assert_eq!(rc.push_count(), 0);
        rc.write(0, 1500).unwrap();
        rc.write(1, 1600).unwrap();
        rc.cork();
        rc.push();
        assert_eq!(rc.cork_count(), 1);
        assert_eq!(rc.push_count(), 1);
    }

    #[test]
    fn rssi_absence_is_none_not_negative_one() {
        let rc = MockRcInput::new();
        assert_eq!(rc.rssi(), None);
        assert_eq!(rc.link_quality(), None);
    }
}
