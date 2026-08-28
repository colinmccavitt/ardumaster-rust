//! DSP / FFT and vector ops, ported from `AP_HAL/DSP.h`.
//!
//! Harmonic-notch and gyro-FFT paths call `fft_init` / `fft_start` /
//! `fft_analyse` plus the four vector helpers. Heap-backed
//! `FFTWindowState` bins (`_freq_bins`, Hanning window, rfft scratch)
//! stay off this stub so the crate remains `no_std` without `alloc`.
//! The mock keeps window metadata and a small last-sample buffer so
//! `fft_analyse` can still report a peak bin.

use crate::{Error, Result};

/// Maximum tolerated cycles with a missing FFT signal.
/// Upstream `FFT_MAX_MISSED_UPDATES`.
pub const FFT_MAX_MISSED_UPDATES: u8 = 5;

/// Sliding-window depth cap. Upstream `MAX_SLIDING_WINDOW_SIZE`.
pub const MAX_SLIDING_WINDOW_SIZE: u8 = 8;

/// How many peaks [`FrequencyPeak::MaxTrackedPeaks`] tracks.
pub const MAX_TRACKED_PEAKS: usize = 3;

/// Mock last-sample buffer. Large enough for a tiny FFT window in tests;
/// a real backend stores `_window_size` bins on the heap.
pub const MOCK_SAMPLE_CAP: usize = 32;

/// Tracked FFT peak slot. Upstream `AP_HAL::DSP::FrequencyPeak`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FrequencyPeak {
    /// Highest-energy bin.
    Center = 0,
    /// Shoulder below the center.
    LowerShoulder = 1,
    /// Shoulder above the center.
    UpperShoulder = 2,
    /// Count of tracked peaks (not a slot).
    MaxTrackedPeaks = 3,
    /// No peak.
    None = 4,
}

/// One estimated peak. Upstream `AP_HAL::DSP::FrequencyPeakData`.
#[derive(Debug, Clone, Copy)]
pub struct FrequencyPeakData {
    /// Estimate of FFT peak frequency (Hz).
    pub freq_hz: f32,
    /// FFT bin with maximum energy.
    pub bin: u16,
    /// Width of the peak (Hz).
    pub noise_width_hz: f32,
}

impl Default for FrequencyPeakData {
    fn default() -> Self {
        Self {
            freq_hz: 0.0,
            bin: 0,
            noise_width_hz: 0.0,
        }
    }
}

/// FFT analysis window metadata. Upstream `AP_HAL::DSP::FFTWindowState`.
///
/// Heap arrays (`_freq_bins`, `_hanning_window`, `_rfft_data`,
/// `_sliding_window`) are not ported. Callers that need the spectrum
/// use a board backend; this struct carries the sizes and the three
/// tracked peaks so a mock can run `fft_analyse` without `alloc`.
#[derive(Debug, Clone)]
pub struct FftWindowState {
    /// Frequency width of one FFT bin (Hz). `sample_rate / window_size`.
    pub bin_resolution: f32,
    /// Number of FFT bins (`window_size / 2`).
    pub bin_count: u16,
    /// Stored frequencies including DC (`bin_count + 1`).
    pub num_stored_freqs: u16,
    /// Size of the FFT window.
    pub window_size: u16,
    /// Sliding-window depth (0 = off). Capped at [`MAX_SLIDING_WINDOW_SIZE`].
    pub sliding_window_size: u8,
    /// Three highest peaks (center / shoulders).
    pub peak_data: [FrequencyPeakData; MAX_TRACKED_PEAKS],
    /// Averaging is ongoing.
    pub averaging: bool,
    /// Number of samples in the average.
    pub averaging_samples: u32,
}

impl FftWindowState {
    /// Build metadata the way upstream's ctor does, without allocating bins.
    ///
    /// `window_size == 0` or `sample_rate == 0` is [`Error::Unsupported`].
    /// `sliding_window_size` above [`MAX_SLIDING_WINDOW_SIZE`] is clipped.
    pub fn new(window_size: u16, sample_rate: u16, sliding_window_size: u8) -> Result<Self> {
        if window_size == 0 || sample_rate == 0 {
            return Err(Error::Unsupported);
        }
        let sliding = if sliding_window_size > MAX_SLIDING_WINDOW_SIZE {
            MAX_SLIDING_WINDOW_SIZE
        } else {
            sliding_window_size
        };
        Ok(Self {
            bin_resolution: (sample_rate as f32) / (window_size as f32),
            bin_count: window_size / 2,
            num_stored_freqs: (window_size / 2).saturating_add(1),
            window_size,
            sliding_window_size: sliding,
            peak_data: [FrequencyPeakData::default(); MAX_TRACKED_PEAKS],
            averaging: false,
            averaging_samples: 0,
        })
    }
}

/// DSP / FFT backend. Upstream `AP_HAL::DSP`.
///
/// Vector helpers are `protected` virtuals upstream; they are on the
/// trait here so a mock can implement them without a C++ subclass.
pub trait Dsp {
    /// Initialise an FFT instance. Upstream `fft_init`.
    ///
    /// Empty returns `nullptr`; the port returns [`Error::Unsupported`]
    /// when the backend cannot open a window (zero size / rate).
    fn fft_init(
        &mut self,
        window_size: u16,
        sample_rate: u16,
        sliding_window_size: u8,
    ) -> Result<FftWindowState>;

    /// Start an FFT analysis with a sample slice. Upstream `fft_start`
    /// takes a `FloatBuffer&`; the port takes a slice so a mock can
    /// copy without the ring-buffer type.
    fn fft_start(&mut self, state: &mut FftWindowState, samples: &[f32], advance: u16);

    /// Finish the analysis and return the center peak bin.
    /// Upstream `fft_analyse`.
    fn fft_analyse(
        &mut self,
        state: &mut FftWindowState,
        start_bin: u16,
        end_bin: u16,
        noise_att_cutoff: f32,
    ) -> u16;

    /// Begin averaging windows. Upstream `fft_start_average`.
    /// Returns `false` if already averaging.
    fn fft_start_average(&mut self, state: &mut FftWindowState) -> bool {
        if state.averaging {
            return false;
        }
        state.averaging_samples = 0;
        state.averaging = true;
        true
    }

    /// Stop averaging and write peak frequencies into `peaks`.
    /// Upstream `fft_stop_average`. Returns how many peaks were written.
    fn fft_stop_average(
        &mut self,
        state: &mut FftWindowState,
        start_bin: u16,
        end_bin: u16,
        peaks: &mut [f32],
    ) -> u16 {
        let _ = (start_bin, end_bin);
        if !state.averaging {
            return 0;
        }
        state.averaging = false;
        if let Some(slot) = peaks.get_mut(0) {
            if let Some(center) = state.peak_data.get(FrequencyPeak::Center as usize) {
                *slot = center.freq_hz;
                return 1;
            }
        }
        0
    }

    /// Find the maximum value and its index. Upstream `vector_max_float`.
    fn vector_max_float(&self, vin: &[f32]) -> Result<(f32, u16)>;

    /// Mean of `vin`. Upstream `vector_mean_float`.
    fn vector_mean_float(&self, vin: &[f32]) -> Result<f32>;

    /// `vout[i] = vin[i] * scale`. Upstream `vector_scale_float`.
    fn vector_scale_float(&self, vin: &[f32], scale: f32, vout: &mut [f32]) -> Result<()>;

    /// `vout[i] = vin1[i] + vin2[i]`. Upstream `vector_add_float`.
    fn vector_add_float(&self, vin1: &[f32], vin2: &[f32], vout: &mut [f32]) -> Result<()>;
}

/// In-memory [`Dsp`] for tests and SITL bring-up.
///
/// Vector ops are real. FFT analyse is a stub: it treats the last
/// `fft_start` samples as a magnitude spectrum and reports the max
/// bin in `[start_bin, end_bin]`. No DFT is computed.
#[derive(Debug)]
pub struct MockDsp {
    last_len: usize,
    last_samples: [f32; MOCK_SAMPLE_CAP],
    started: bool,
}

impl Default for MockDsp {
    fn default() -> Self {
        Self {
            last_len: 0,
            last_samples: [0.0; MOCK_SAMPLE_CAP],
            started: false,
        }
    }
}

impl MockDsp {
    /// A fresh DSP with no pending window.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Whether [`Dsp::fft_start`] has been called since init.
    #[must_use]
    pub const fn has_started(&self) -> bool {
        self.started
    }
}

impl Dsp for MockDsp {
    fn fft_init(
        &mut self,
        window_size: u16,
        sample_rate: u16,
        sliding_window_size: u8,
    ) -> Result<FftWindowState> {
        self.started = false;
        self.last_len = 0;
        FftWindowState::new(window_size, sample_rate, sliding_window_size)
    }

    fn fft_start(&mut self, state: &mut FftWindowState, samples: &[f32], advance: u16) {
        let _ = advance;
        let n = samples
            .len()
            .min(MOCK_SAMPLE_CAP)
            .min(usize::from(state.window_size));
        self.last_len = n;
        if let (Some(dst), Some(src)) = (self.last_samples.get_mut(..n), samples.get(..n)) {
            dst.copy_from_slice(src);
        }
        self.started = true;
        if state.averaging {
            state.averaging_samples = state.averaging_samples.saturating_add(1);
        }
    }

    fn fft_analyse(
        &mut self,
        state: &mut FftWindowState,
        start_bin: u16,
        end_bin: u16,
        noise_att_cutoff: f32,
    ) -> u16 {
        let _ = noise_att_cutoff;
        if !self.started || self.last_len == 0 {
            return 0;
        }
        let last = self.last_len as u16;
        let hi_cap = last.saturating_sub(1);
        let lo = start_bin.min(hi_cap);
        let hi = end_bin.min(hi_cap).max(lo);
        let window = match self.last_samples.get(usize::from(lo)..=usize::from(hi)) {
            Some(w) if !w.is_empty() => w,
            _ => return 0,
        };
        let Ok((_max_val, rel)) = self.vector_max_float(window) else {
            return 0;
        };
        let bin = lo.saturating_add(rel);
        if let Some(center) = state.peak_data.get_mut(FrequencyPeak::Center as usize) {
            center.bin = bin;
            center.freq_hz = (bin as f32) * state.bin_resolution;
            center.noise_width_hz = state.bin_resolution;
        }
        bin
    }

    fn vector_max_float(&self, vin: &[f32]) -> Result<(f32, u16)> {
        let first = vin.first().copied().ok_or(Error::Unsupported)?;
        let mut max_value = first;
        let mut max_index: u16 = 0;
        for (i, v) in vin.iter().enumerate().skip(1) {
            if *v > max_value {
                max_value = *v;
                max_index = u16::try_from(i).map_err(|_| Error::Unsupported)?;
            }
        }
        Ok((max_value, max_index))
    }

    fn vector_mean_float(&self, vin: &[f32]) -> Result<f32> {
        if vin.is_empty() {
            return Err(Error::Unsupported);
        }
        let mut sum = 0.0f32;
        for v in vin {
            sum += *v;
        }
        Ok(sum / (vin.len() as f32))
    }

    fn vector_scale_float(&self, vin: &[f32], scale: f32, vout: &mut [f32]) -> Result<()> {
        if vin.len() != vout.len() {
            return Err(Error::Unsupported);
        }
        for (out, inp) in vout.iter_mut().zip(vin.iter()) {
            *out = *inp * scale;
        }
        Ok(())
    }

    fn vector_add_float(&self, vin1: &[f32], vin2: &[f32], vout: &mut [f32]) -> Result<()> {
        if vin1.len() != vin2.len() || vin1.len() != vout.len() {
            return Err(Error::Unsupported);
        }
        for ((out, a), b) in vout.iter_mut().zip(vin1.iter()).zip(vin2.iter()) {
            *out = *a + *b;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[allow(clippy::float_cmp)]
    fn assert_f32(got: f32, expected: f32) {
        assert!(
            (got - expected).abs() < 1.0e-4,
            "expected {expected}, got {got}"
        );
    }

    #[test]
    fn frequency_peak_slots_match_upstream() {
        assert_eq!(FrequencyPeak::Center as u8, 0);
        assert_eq!(FrequencyPeak::LowerShoulder as u8, 1);
        assert_eq!(FrequencyPeak::UpperShoulder as u8, 2);
        assert_eq!(FrequencyPeak::MaxTrackedPeaks as u8, 3);
        assert_eq!(FrequencyPeak::None as u8, 4);
        assert_eq!(MAX_TRACKED_PEAKS, 3);
        assert_eq!(MAX_SLIDING_WINDOW_SIZE, 8);
        assert_eq!(FFT_MAX_MISSED_UPDATES, 5);
    }

    #[test]
    fn fft_init_sets_bin_metadata() {
        let mut dsp = MockDsp::new();
        let state = dsp.fft_init(8, 1000, 0).expect("window 8 @ 1 kHz is valid");
        assert_eq!(state.window_size, 8);
        assert_eq!(state.bin_count, 4);
        assert_eq!(state.num_stored_freqs, 5);
        assert_eq!(state.sliding_window_size, 0);
        assert_f32(state.bin_resolution, 125.0);
        assert!(!dsp.has_started());
    }

    #[test]
    fn fft_init_rejects_zero_and_clips_sliding_window() {
        let mut dsp = MockDsp::new();
        assert_eq!(dsp.fft_init(0, 1000, 0).err(), Some(Error::Unsupported));
        assert_eq!(dsp.fft_init(8, 0, 0).err(), Some(Error::Unsupported));
        let state = dsp.fft_init(16, 1600, 99).expect("nonzero window/rate");
        assert_eq!(state.sliding_window_size, MAX_SLIDING_WINDOW_SIZE);
        assert_eq!(state.bin_count, 8);
        assert_f32(state.bin_resolution, 100.0);
    }

    #[test]
    fn fft_start_analyse_reports_peak_bin() {
        let mut dsp = MockDsp::new();
        let mut state = dsp.fft_init(8, 800, 0).expect("init");
        // Treat samples as a magnitude spectrum: bin 2 is the peak.
        let samples = [1.0, 2.0, 9.0, 3.0, 1.0, 0.5, 0.25, 0.1];
        dsp.fft_start(&mut state, &samples, 1);
        assert!(dsp.has_started());
        let bin = dsp.fft_analyse(&mut state, 0, 7, 0.0);
        assert_eq!(bin, 2);
        let center = state
            .peak_data
            .get(FrequencyPeak::Center as usize)
            .expect("center slot");
        assert_eq!(center.bin, 2);
        assert_f32(center.freq_hz, 200.0);
    }

    #[test]
    fn fft_analyse_without_start_is_zero() {
        let mut dsp = MockDsp::new();
        let mut state = dsp.fft_init(8, 800, 0).expect("init");
        assert_eq!(dsp.fft_analyse(&mut state, 0, 7, 0.0), 0);
    }

    #[test]
    fn vector_max_mean_scale_add() {
        let dsp = MockDsp::new();
        let vin = [1.0, 4.0, 2.0, -1.0];
        let (max_v, max_i) = dsp.vector_max_float(&vin).expect("non-empty");
        assert_f32(max_v, 4.0);
        assert_eq!(max_i, 1);
        assert_f32(dsp.vector_mean_float(&vin).expect("mean"), 1.5);

        let mut scaled = [0.0; 4];
        assert!(dsp.vector_scale_float(&vin, 2.0, &mut scaled).is_ok());
        assert_f32(*scaled.first().expect("len 4"), 2.0);
        assert_f32(*scaled.get(1).expect("len 4"), 8.0);

        let mut sum = [0.0; 4];
        assert!(dsp.vector_add_float(&vin, &scaled, &mut sum).is_ok());
        assert_f32(*sum.get(1).expect("len 4"), 12.0);
    }

    #[test]
    fn vector_ops_reject_empty_and_length_mismatch() {
        let dsp = MockDsp::new();
        assert_eq!(dsp.vector_max_float(&[]), Err(Error::Unsupported));
        assert_eq!(dsp.vector_mean_float(&[]), Err(Error::Unsupported));
        let vin = [1.0, 2.0];
        let mut short = [0.0; 1];
        assert_eq!(
            dsp.vector_scale_float(&vin, 1.0, &mut short),
            Err(Error::Unsupported)
        );
        assert_eq!(
            dsp.vector_add_float(&vin, &[1.0], &mut [0.0, 0.0]),
            Err(Error::Unsupported)
        );
    }

    #[test]
    fn fft_average_start_stop() {
        let mut dsp = MockDsp::new();
        let mut state = dsp.fft_init(8, 800, 1).expect("init");
        assert!(dsp.fft_start_average(&mut state));
        assert!(state.averaging);
        assert!(!dsp.fft_start_average(&mut state), "already averaging");
        dsp.fft_start(&mut state, &[0.0, 1.0, 5.0, 1.0], 1);
        assert_eq!(state.averaging_samples, 1);
        let _ = dsp.fft_analyse(&mut state, 0, 3, 0.0);
        let mut peaks = [0.0f32; MAX_TRACKED_PEAKS];
        assert_eq!(dsp.fft_stop_average(&mut state, 0, 3, &mut peaks), 1);
        assert!(!state.averaging);
        assert_eq!(dsp.fft_stop_average(&mut state, 0, 3, &mut peaks), 0);
        assert_f32(*peaks.first().expect("peak slot"), 200.0);
    }

    /// The trait stays object-safe so `&dyn Dsp` can sit in the HAL
    /// context. A future method that breaks object safety fails here.
    #[test]
    fn dsp_trait_is_object_safe() {
        let mut dsp = MockDsp::new();
        let d: &mut dyn Dsp = &mut dsp;
        let mut state = d.fft_init(4, 400, 0).expect("init");
        d.fft_start(&mut state, &[1.0, 3.0, 2.0, 0.0], 1);
        assert_eq!(d.fft_analyse(&mut state, 0, 3, 0.0), 1);
        let (max_v, max_i) = d.vector_max_float(&[1.0, 3.0, 2.0]).expect("max");
        assert_f32(max_v, 3.0);
        assert_eq!(max_i, 1);
    }
}
