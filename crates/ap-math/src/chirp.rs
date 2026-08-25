//! Frequency-sweep generator for system identification, upstream
//! `AP_Math/chirp.cpp`. FW-038.
//!
//! A chirp is a sine wave whose frequency rises from a minimum to a maximum.
//! Injected into a control axis and logged alongside the response, it measures
//! the vehicle's frequency response directly — which is what autotune and
//! handling-qualities work need and what a step input cannot give.
//!
//! # Why the frequency rises exponentially
//!
//! A linear sweep spends as long between 1 and 2 Hz as between 10 and 11 Hz,
//! but the second interval is a twentieth of an octave and the first is a
//! whole one. Sweeping exponentially spends equal time per octave, so the
//! measurement is evenly distributed across the range that actually matters.
//!
//! # The window
//!
//! The magnitude is faded in and out with a raised cosine. Starting a sine
//! wave at full amplitude is a step, and a step excites everything at once —
//! which is precisely what the measurement is trying to separate.
//!
//! # The optional dwell
//!
//! Before the sweep begins the generator can hold at the minimum frequency for
//! `time_const_freq` seconds. Low frequencies need several cycles to say
//! anything, and at 0.2 Hz a sweep would be past them before one cycle
//! finished.

use crate::scalar::{is_equal, Real};

/// Two pi, upstream `M_2PI`.
const TWO_PI: f32 = core::f32::consts::TAU;

/// A frequency sweep, upstream `Chirp`.
#[derive(Debug, Clone, Copy, Default)]
pub struct Chirp {
    record: f32,
    w_min: f32,
    w_max: f32,
    fade_in: f32,
    fade_out: f32,
    const_freq: f32,
    b: f32,

    magnitude: f32,
    window: f32,
    output: f32,
    waveform_freq_rads: f32,
    complete: bool,
}

impl Chirp {
    /// An unconfigured chirp. [`Chirp::init`] must be called before
    /// [`Chirp::update`].
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Configure the sweep, upstream `init`.
    ///
    /// `time_record` is the whole duration including the dwell. Setting
    /// `frequency_start_hz` equal to `frequency_stop_hz` gives a constant-
    /// frequency sine instead of a sweep, which upstream supports explicitly.
    pub fn init(
        &mut self,
        time_record: f32,
        frequency_start_hz: f32,
        frequency_stop_hz: f32,
        time_fade_in: f32,
        time_fade_out: f32,
        time_const_freq: f32,
    ) {
        self.record = time_record;
        self.w_min = TWO_PI * frequency_start_hz;
        self.w_max = TWO_PI * frequency_stop_hz;
        self.fade_in = time_fade_in;
        self.fade_out = time_fade_out;
        self.const_freq = time_const_freq;

        // The exponent that takes the sweep from w_min to w_max over its
        // length. Zero for a constant-frequency run, which is why the update
        // path tests the two frequencies for equality before dividing by it.
        self.b = Real::log(self.w_max / self.w_min);

        self.complete = false;
    }

    /// The signal at a time, upstream `update`.
    ///
    /// `time` is seconds since the sweep began; the caller owns the clock.
    pub fn update(&mut self, time: f32, waveform_magnitude: f32) -> f32 {
        self.magnitude = waveform_magnitude;

        // Raised-cosine fade at each end, unity in between.
        self.window = if time <= 0.0 {
            0.0
        } else if time <= self.fade_in {
            0.5 - 0.5 * Real::cos(core::f32::consts::PI * time / self.fade_in)
        } else if time <= self.record - self.fade_out {
            1.0
        } else if time <= self.record {
            0.5 - 0.5
                * Real::cos(
                    core::f32::consts::PI * (time - (self.record - self.fade_out)) / self.fade_out
                        + core::f32::consts::PI,
                )
        } else {
            0.0
        };

        if time <= 0.0 {
            self.waveform_freq_rads = self.w_min;
            self.output = 0.0;
        } else if time <= self.const_freq {
            // The dwell. The phase offset makes the sine cross zero at the end
            // of the dwell rather than at its start, so the sweep continues
            // from a zero crossing.
            self.waveform_freq_rads = self.w_min;
            self.output = self.window
                * self.magnitude
                * Real::sin(self.w_min * time - self.w_min * self.const_freq);
        } else if time <= self.record {
            if is_equal(self.w_min, self.w_max) {
                // A constant-frequency run. Tested before dividing by `b`,
                // which is zero here.
                self.waveform_freq_rads = self.w_min;
                self.output = self.window * self.magnitude * Real::sin(self.w_min * time);
            } else {
                let progress = (time - self.const_freq) / (self.record - self.const_freq);
                self.waveform_freq_rads = self.w_min * Real::exp(self.b * progress);
                // The phase is the integral of the frequency, which for an
                // exponential sweep has this closed form.
                self.output = self.window
                    * self.magnitude
                    * Real::sin(
                        (self.w_min * (self.record - self.const_freq) / self.b)
                            * (Real::exp(self.b * progress) - 1.0),
                    );
            }
        } else {
            self.waveform_freq_rads = self.w_max;
            self.output = 0.0;
        }

        self.complete = time > self.record;

        self.output
    }

    /// The instantaneous frequency, radians per second. Upstream
    /// `get_frequency_rads`.
    #[must_use]
    pub const fn frequency_rads(&self) -> f32 {
        self.waveform_freq_rads
    }

    /// Whether the sweep has run past its record length, upstream
    /// `completed`.
    #[must_use]
    pub const fn completed(&self) -> bool {
        self.complete
    }

    /// The fade window in force at the last update, 0 to 1.
    ///
    /// Not an upstream accessor. Exposed because the window is otherwise
    /// invisible and it is half of what the output is.
    #[must_use]
    pub const fn window(&self) -> f32 {
        self.window
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "the silence outside the record and the unity window in the middle \nare exact values, not approximations; an epsilon would accept a signal leaking outside \nits record, which is the thing being ruled out"
    )]

    use super::*;

    /// A sweep from 0.5 to 10 Hz over 30 seconds, fading over 2 at each end,
    /// after a 3 second dwell.
    fn sweep() -> Chirp {
        let mut c = Chirp::new();
        c.init(30.0, 0.5, 10.0, 2.0, 2.0, 3.0);
        c
    }

    /// Nothing before the start and nothing after the end.
    #[test]
    fn the_signal_is_silent_outside_its_record() {
        let mut c = sweep();
        assert_eq!(c.update(-1.0, 1.0), 0.0);
        assert_eq!(c.update(0.0, 1.0), 0.0);
        assert_eq!(c.update(31.0, 1.0), 0.0);
        assert!(c.completed());
    }

    /// The window rises from nothing to unity over the fade-in, holds, and
    /// falls back over the fade-out.
    #[test]
    fn the_window_fades_in_and_out() {
        let mut c = sweep();

        c.update(0.001, 1.0);
        assert!(
            c.window() < 0.01,
            "barely open at the start: {}",
            c.window()
        );

        c.update(1.0, 1.0);
        let half = c.window();
        assert!(
            (half - 0.5).abs() < 0.01,
            "half way through a 2 s fade-in: {half}"
        );

        c.update(15.0, 1.0);
        assert_eq!(c.window(), 1.0, "fully open in the middle");

        c.update(29.0, 1.0);
        assert!(
            (c.window() - 0.5).abs() < 0.01,
            "half way through the fade-out: {}",
            c.window()
        );

        c.update(30.0, 1.0);
        assert!(c.window() < 0.01, "closed at the end: {}", c.window());
    }

    /// The frequency holds at the minimum through the dwell, then rises.
    #[test]
    fn the_frequency_dwells_then_sweeps() {
        let mut c = sweep();

        c.update(1.0, 1.0);
        assert!(
            (c.frequency_rads() - TWO_PI * 0.5).abs() < 1e-4,
            "dwelling at 0.5 Hz: {}",
            c.frequency_rads()
        );

        c.update(3.0, 1.0);
        assert!(
            (c.frequency_rads() - TWO_PI * 0.5).abs() < 1e-4,
            "still dwelling"
        );

        c.update(30.0, 1.0);
        assert!(
            (c.frequency_rads() - TWO_PI * 10.0).abs() < 0.1,
            "at the top by the end: {}",
            c.frequency_rads()
        );
    }

    /// Equal time per octave is the point of an exponential sweep. The time to
    /// double the frequency should be the same wherever in the sweep it is
    /// measured.
    #[test]
    fn the_sweep_spends_equal_time_per_octave() {
        let mut c = sweep();

        // Find when the frequency first passes 1, 2 and 4 Hz.
        let cross = |c: &mut Chirp, hz: f32| -> f32 {
            let mut t = 3.0_f32;
            while t < 30.0 {
                c.update(t, 1.0);
                if c.frequency_rads() >= TWO_PI * hz {
                    return t;
                }
                t += 0.001;
            }
            f32::NAN
        };

        let t1 = cross(&mut c, 1.0);
        let t2 = cross(&mut c, 2.0);
        let t4 = cross(&mut c, 4.0);

        let first_octave = t2 - t1;
        let second_octave = t4 - t2;
        assert!(
            (first_octave - second_octave).abs() < 0.05,
            "octaves should take equal time: {first_octave} then {second_octave}"
        );
    }

    /// Equal start and stop frequencies give a constant-frequency sine, which
    /// is the path that would divide by zero if it were not tested for.
    #[test]
    fn equal_frequencies_give_a_steady_sine() {
        let mut c = Chirp::new();
        c.init(10.0, 2.0, 2.0, 1.0, 1.0, 0.0);

        let mut peak = 0.0_f32;
        let mut t = 0.0_f32;
        while t <= 10.0 {
            let v = c.update(t, 1.0);
            assert!(v.is_finite(), "at {t}");
            if (2.0..8.0).contains(&t) {
                peak = peak.max(v.abs());
                assert!(
                    (c.frequency_rads() - TWO_PI * 2.0).abs() < 1e-4,
                    "frequency should not move: {}",
                    c.frequency_rads()
                );
            }
            t += 0.01;
        }
        assert!(peak > 0.95, "a full-amplitude sine in the middle: {peak}");
    }

    /// The dwell's phase offset puts a zero crossing at the moment the sweep
    /// takes over, so the two halves join without a step.
    #[test]
    fn the_dwell_hands_over_at_a_zero_crossing() {
        let mut c = sweep();
        let at_handover = c.update(3.0, 1.0);
        assert!(
            at_handover.abs() < 1e-3,
            "the dwell should end at a zero crossing, got {at_handover}"
        );
    }

    /// The output never exceeds the magnitude asked for. An injected signal
    /// that overshot its commanded amplitude would be a real hazard on a
    /// vehicle.
    #[test]
    fn the_output_never_exceeds_its_magnitude() {
        let mut c = sweep();
        let mut t = -1.0_f32;
        while t <= 32.0 {
            let v = c.update(t, 0.25);
            assert!(
                v.abs() <= 0.25 + 1e-6,
                "output {v} exceeded the 0.25 magnitude at {t}"
            );
            t += 0.002;
        }
    }

    /// Nothing anywhere in or around the record produces a NaN.
    #[test]
    fn the_whole_record_is_finite() {
        for (start, stop) in [(0.5_f32, 10.0_f32), (2.0, 2.0), (0.1, 40.0), (10.0, 0.5)] {
            let mut c = Chirp::new();
            c.init(20.0, start, stop, 1.0, 1.0, 2.0);
            let mut t = -1.0_f32;
            while t <= 22.0 {
                let v = c.update(t, 1.0);
                assert!(v.is_finite(), "{start} to {stop} Hz at {t}: {v}");
                assert!(c.frequency_rads().is_finite());
                t += 0.01;
            }
        }
    }

    /// completed() turns over exactly at the record length, not before.
    #[test]
    fn completion_is_reported_at_the_record_length() {
        let mut c = sweep();
        c.update(29.999, 1.0);
        assert!(!c.completed());
        c.update(30.0, 1.0);
        assert!(
            !c.completed(),
            "at exactly the record length it is not past it"
        );
        c.update(30.001, 1.0);
        assert!(c.completed());
    }
}
