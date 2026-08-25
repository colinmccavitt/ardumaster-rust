//! Port of `SRV_Channel`'s output conversion. Tracked as FW-018.
//!
//! This is the last step of the control path: the controllers produce a
//! surface demand in centidegrees or a throttle in percent, and this turns it
//! into the pulse width a servo actually receives.
//!
//! # Two kinds of output
//!
//! A channel is either an **angle** output, symmetric about a trim — an
//! aileron sits at trim with zero demand and swings either way — or a
//! **range** output running from the minimum upward, which is what a throttle
//! wants. The distinction is not a scaling detail: they use different
//! endpoints, and only the angle form uses the trim at all.
//!
//! # Truncation is part of the answer
//!
//! Upstream casts the scaled offset to `uint16_t` before adding it to the
//! endpoint, so the fractional pulse width is discarded rather than rounded.
//! Reproducing that matters: rounding instead would shift every surface by up
//! to half a microsecond, which is small but systematic and would show up in
//! any comparison against a real aircraft.

#![no_std]

use ap_math::scalar::constrain_value;

/// How a channel maps a scaled value onto pulse widths.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputType {
    /// Symmetric about the trim, `-high_out..high_out`. Upstream
    /// `set_angle`.
    Angle,
    /// From the minimum upward, `0..high_out`. Upstream `set_range`.
    Range,
}

/// Upstream's `uint16_t(x)` on a float, without the undefined behaviour.
///
/// A negative float converted to an unsigned integer is undefined in C++.
/// Upstream does it anyway in `pwm_from_angle`, where an inverted channel
/// makes the span negative, and on x86-64 GCC the result wraps — which turns
/// out to be exactly the reversed deflection, so inverting the endpoints is a
/// working way to reverse a servo. Going through `i32` first reproduces that
/// value by defined means.
#[inline]
fn truncate_to_u16(v: f32) -> u16 {
    #[allow(
        clippy::cast_possible_truncation,
        reason = "upstream truncates here; a pulse-width offset is far inside i32"
    )]
    let as_int = v as i32;
    #[allow(
        clippy::cast_sign_loss,
        clippy::cast_possible_truncation,
        reason = "reproduces upstream's uint16_t conversion, wrap included"
    )]
    let out = as_int as u16;
    out
}

/// One servo output channel, upstream `SRV_Channel`'s conversion state.
///
/// The function assignment, the override and E-stop paths, and the parameter
/// plumbing are not here: this is the arithmetic that turns a demand into a
/// pulse width, which is what the control path needs and what can be compared
/// against upstream without standing up a vehicle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ServoChannel {
    /// Minimum pulse width, microseconds. Upstream `SERVOn_MIN`.
    pub servo_min: u16,
    /// Maximum pulse width, microseconds. Upstream `SERVOn_MAX`.
    pub servo_max: u16,
    /// Neutral pulse width, microseconds. Upstream `SERVOn_TRIM`. Used only
    /// by angle outputs.
    pub servo_trim: u16,
    /// Whether the channel's sense is inverted. Upstream `SERVOn_REVERSED`.
    pub reversed: bool,
    /// Which mapping this channel uses.
    pub output_type: OutputType,
    /// The scaled value corresponding to full deflection: the angle limit for
    /// an angle output, the top of the range for a range output.
    pub high_out: u16,
}

impl ServoChannel {
    /// A channel configured as an angle output, upstream `set_angle`.
    #[must_use]
    pub const fn angle(servo_min: u16, servo_trim: u16, servo_max: u16, angle: u16) -> Self {
        Self {
            servo_min,
            servo_max,
            servo_trim,
            reversed: false,
            output_type: OutputType::Angle,
            high_out: angle,
        }
    }

    /// A channel configured as a range output, upstream `set_range`.
    #[must_use]
    pub const fn range(servo_min: u16, servo_max: u16, high: u16) -> Self {
        Self {
            servo_min,
            servo_max,
            servo_trim: servo_min,
            reversed: false,
            output_type: OutputType::Range,
            high_out: high,
        }
    }

    /// Pulse width for a `0..high_out` value, upstream `pwm_from_range`.
    ///
    /// A misconfigured channel — maximum not above minimum, or no range set —
    /// produces the minimum rather than an error. That is upstream's choice
    /// and it is the safe direction for a throttle.
    #[must_use]
    pub fn pwm_from_range(&self, scaled_value: f32) -> u16 {
        if self.servo_max <= self.servo_min || self.high_out == 0 {
            return self.servo_min;
        }
        let high = f32::from(self.high_out);
        let mut v = constrain_value(scaled_value, 0.0, high);
        if self.reversed {
            v = high - v;
        }
        let span = f32::from(self.servo_max) - f32::from(self.servo_min);
        self.servo_min
            .wrapping_add(truncate_to_u16((v * span) / high))
    }

    /// Pulse width for a `-high_out..high_out` value, upstream
    /// `pwm_from_angle`.
    ///
    /// The two halves are scaled independently, against `trim..max` above and
    /// `min..trim` below, so a trim that is not centred still reaches both
    /// endpoints at full deflection.
    #[must_use]
    pub fn pwm_from_angle(&self, scaled_value: f32) -> u16 {
        if self.high_out == 0 {
            return self.servo_trim;
        }
        let mut v = scaled_value;
        if self.reversed {
            v = -v;
        }
        let high = f32::from(self.high_out);
        v = constrain_value(v, -high, high);

        if v > 0.0 {
            let span = f32::from(self.servo_max) - f32::from(self.servo_trim);
            self.servo_trim
                .wrapping_add(truncate_to_u16((v * span) / high))
        } else {
            let span = f32::from(self.servo_trim) - f32::from(self.servo_min);
            self.servo_trim
                .wrapping_sub(truncate_to_u16((-v * span) / high))
        }
    }

    /// Pulse width for this channel's configured mapping, upstream
    /// `pwm_from_scaled_value`.
    #[must_use]
    pub fn pwm_from_scaled_value(&self, scaled_value: f32) -> u16 {
        match self.output_type {
            OutputType::Angle => self.pwm_from_angle(scaled_value),
            OutputType::Range => self.pwm_from_range(scaled_value),
        }
    }

    /// Normalised output in `-1..1` about the midpoint of the travel,
    /// upstream `get_output_norm`.
    ///
    /// Note the midpoint here is the mean of minimum and maximum, *not* the
    /// trim — so a channel with an off-centre trim reports a non-zero
    /// normalised output when it is sitting at its own neutral.
    #[must_use]
    pub fn output_norm(&self, output_pwm: u16) -> f32 {
        let mid = (self.servo_max + self.servo_min) / 2;
        if mid <= self.servo_min {
            return 0.0;
        }
        let ret = if output_pwm < mid {
            f32::from(mid - output_pwm) / -f32::from(mid - self.servo_min)
        } else if output_pwm > mid {
            f32::from(output_pwm - mid) / f32::from(self.servo_max - mid)
        } else {
            0.0
        };
        if self.reversed {
            -ret
        } else {
            ret
        }
    }
}
