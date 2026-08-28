//! Optical flow sensor, ported from `AP_HAL/OpticalFlow.h`.
//!
//! A quality / flow-rate sample is one [`DataFrame`]: integrated pixel
//! flow, integrated body-rate gyro, `delta_time` (µs), and surface
//! `quality`. `AP_OpticalFlow_Onboard` turns those integrals into
//! `flowRate` (mrad/s) and `bodyRate` (rad/s). Empty HAL `read`
//! returns false; this mock holds one pending sample so tests can
//! exercise that path without a camera backend.

/// One integrated sample. Upstream `AP_HAL::OpticalFlow::Data_Frame`.
///
/// `pixel_flow_*_integral` is milliradians over `delta_time`.
/// `gyro_*_integral` is radians over `delta_time`. `delta_time` is
/// microseconds. `quality` is surface quality (0 to 255).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DataFrame {
    /// Integrated X pixel flow (mrad). Upstream `pixel_flow_x_integral`.
    pub pixel_flow_x_integral: f32,
    /// Integrated Y pixel flow (mrad). Upstream `pixel_flow_y_integral`.
    pub pixel_flow_y_integral: f32,
    /// Integrated X gyro (rad). Upstream `gyro_x_integral`.
    pub gyro_x_integral: f32,
    /// Integrated Y gyro (rad). Upstream `gyro_y_integral`.
    pub gyro_y_integral: f32,
    /// Integration span in microseconds. Upstream `delta_time`.
    pub delta_time: u32,
    /// Surface quality 0 to 255. Upstream `quality`.
    pub quality: u8,
}

impl Default for DataFrame {
    fn default() -> Self {
        Self::zero()
    }
}

impl DataFrame {
    /// Empty sample: no flow, no gyro, zero quality.
    #[must_use]
    pub const fn zero() -> Self {
        Self {
            pixel_flow_x_integral: 0.0,
            pixel_flow_y_integral: 0.0,
            gyro_x_integral: 0.0,
            gyro_y_integral: 0.0,
            delta_time: 0,
            quality: 0,
        }
    }

    /// Quality / flow-rate sample. Flow integrals are milliradians over
    /// `delta_time` microseconds; `quality` is surface quality.
    #[must_use]
    pub const fn sample(
        quality: u8,
        pixel_flow_x_integral: f32,
        pixel_flow_y_integral: f32,
        gyro_x_integral: f32,
        gyro_y_integral: f32,
        delta_time: u32,
    ) -> Self {
        Self {
            pixel_flow_x_integral,
            pixel_flow_y_integral,
            gyro_x_integral,
            gyro_y_integral,
            delta_time,
            quality,
        }
    }

    /// X flow rate in milliradians per second.
    ///
    /// Matches `AP_OpticalFlow_Onboard::update`:
    /// `1000 / delta_time * pixel_flow_x_integral`. Zero `delta_time`
    /// yields 0 (upstream zeros `flowRate` in that case).
    #[must_use]
    pub fn flow_rate_x(&self) -> f32 {
        flow_rate(self.pixel_flow_x_integral, self.delta_time)
    }

    /// Y flow rate in milliradians per second. See [`flow_rate_x`].
    #[must_use]
    pub fn flow_rate_y(&self) -> f32 {
        flow_rate(self.pixel_flow_y_integral, self.delta_time)
    }
}

/// `1000 / dt_us * integral` — onboard conversion to mrad/s.
fn flow_rate(integral: f32, delta_time_us: u32) -> f32 {
    if delta_time_us == 0 {
        0.0
    } else {
        1000.0 / (delta_time_us as f32) * integral
    }
}

/// Optical-flow HAL. Upstream `AP_HAL::OpticalFlow`.
///
/// `read` returns `bool` because that is what upstream returns
/// (Empty HAL is always `false`). Widening it to [`crate::Result`]
/// would be a behavior change (ADR-0003).
pub trait OpticalFlow {
    /// Prepare the backend. Upstream `init()`.
    fn init(&mut self);

    /// Copy the latest quality / flow-rate sample into `frame`.
    ///
    /// Returns `true` when a sample was available. Upstream `read()`.
    fn read(&mut self, frame: &mut DataFrame) -> bool;

    /// Integrate a gyro sample. Upstream `push_gyro()`.
    ///
    /// `dt` is the interval in seconds. The mock subtracts the last
    /// [`push_gyro_bias`](Self::push_gyro_bias) the way Linux onboard
    /// does (`(gyro - bias) * dt`).
    fn push_gyro(&mut self, gyro_x: f32, gyro_y: f32, dt: f32);

    /// Set the gyro bias subtracted by [`push_gyro`](Self::push_gyro).
    /// Upstream `push_gyro_bias()`.
    fn push_gyro_bias(&mut self, gyro_bias_x: f32, gyro_bias_y: f32);
}

/// An in-memory [`OpticalFlow`] for tests and SITL bring-up.
///
/// Holds one pending [`DataFrame`]. [`OpticalFlow::read`] consumes it
/// (Linux onboard also clears integrals after a successful read).
/// There is no camera thread: queue a sample with [`set_sample`].
#[derive(Debug, Clone)]
pub struct MockOpticalFlow {
    initialized: bool,
    pending: Option<DataFrame>,
    gyro_bias_x: f32,
    gyro_bias_y: f32,
    gyro_x_integral: f32,
    gyro_y_integral: f32,
}

impl Default for MockOpticalFlow {
    fn default() -> Self {
        Self {
            initialized: false,
            pending: None,
            gyro_bias_x: 0.0,
            gyro_bias_y: 0.0,
            gyro_x_integral: 0.0,
            gyro_y_integral: 0.0,
        }
    }
}

impl MockOpticalFlow {
    /// Uninitialized backend with no pending sample (Empty HAL shape).
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Queue a quality / flow-rate sample for the next [`OpticalFlow::read`].
    pub fn set_sample(&mut self, frame: DataFrame) {
        self.pending = Some(frame);
    }

    /// Whether [`OpticalFlow::init`] has been called.
    #[must_use]
    pub const fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Accumulated X gyro integral from [`OpticalFlow::push_gyro`].
    #[must_use]
    pub const fn gyro_x_integral(&self) -> f32 {
        self.gyro_x_integral
    }

    /// Accumulated Y gyro integral from [`OpticalFlow::push_gyro`].
    #[must_use]
    pub const fn gyro_y_integral(&self) -> f32 {
        self.gyro_y_integral
    }
}

impl OpticalFlow for MockOpticalFlow {
    fn init(&mut self) {
        self.initialized = true;
    }

    fn read(&mut self, frame: &mut DataFrame) -> bool {
        match self.pending.take() {
            Some(sample) => {
                *frame = sample;
                true
            }
            None => false,
        }
    }

    fn push_gyro(&mut self, gyro_x: f32, gyro_y: f32, dt: f32) {
        self.gyro_x_integral += (gyro_x - self.gyro_bias_x) * dt;
        self.gyro_y_integral += (gyro_y - self.gyro_bias_y) * dt;
    }

    fn push_gyro_bias(&mut self, gyro_bias_x: f32, gyro_bias_y: f32) {
        self.gyro_bias_x = gyro_bias_x;
        self.gyro_bias_y = gyro_bias_y;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_read_is_false() {
        let mut flow = MockOpticalFlow::new();
        assert!(!flow.is_initialized());
        flow.init();
        assert!(flow.is_initialized());
        let mut frame = DataFrame::zero();
        assert!(!flow.read(&mut frame));
        assert_eq!(frame.quality, 0);
        assert!((frame.flow_rate_x() - 0.0).abs() < 1e-6);
        assert!((frame.flow_rate_y() - 0.0).abs() < 1e-6);
    }

    #[test]
    fn quality_and_flow_rate_sample() {
        let mut flow = MockOpticalFlow::new();
        flow.init();
        // 20 mrad over 10_000 us -> 2 mrad/s (onboard 1000/dt * integral).
        let sample = DataFrame::sample(180, 20.0, -10.0, 0.05, -0.02, 10_000);
        flow.set_sample(sample);

        let mut frame = DataFrame::zero();
        assert!(flow.read(&mut frame));
        assert_eq!(frame.quality, 180);
        assert!((frame.flow_rate_x() - 2.0).abs() < 1e-5);
        assert!((frame.flow_rate_y() - -1.0).abs() < 1e-5);
        assert!((frame.pixel_flow_x_integral - 20.0).abs() < 1e-5);
        assert!((frame.gyro_x_integral - 0.05).abs() < 1e-5);
        assert_eq!(frame.delta_time, 10_000);

        // Consumed: next read is Empty-shaped.
        assert!(!flow.read(&mut frame));
    }

    #[test]
    fn zero_delta_time_has_zero_flow_rate() {
        let frame = DataFrame::sample(51, 8.0, 4.0, 0.0, 0.0, 0);
        assert_eq!(frame.quality, 51);
        assert!((frame.flow_rate_x() - 0.0).abs() < 1e-6);
        assert!((frame.flow_rate_y() - 0.0).abs() < 1e-6);
    }

    #[test]
    fn push_gyro_subtracts_bias() {
        let mut flow = MockOpticalFlow::new();
        flow.push_gyro_bias(0.1, 0.2);
        flow.push_gyro(1.1, 1.2, 0.01);
        assert!((flow.gyro_x_integral() - 0.01).abs() < 1e-6);
        assert!((flow.gyro_y_integral() - 0.01).abs() < 1e-6);
        flow.push_gyro(1.1, 1.2, 0.01);
        assert!((flow.gyro_x_integral() - 0.02).abs() < 1e-6);
    }

    /// The trait stays object-safe, which is what allows `&dyn` in the HAL
    /// context. If a future method breaks object safety this fails to compile
    /// here rather than at some distant call site.
    #[test]
    fn optical_flow_trait_is_object_safe() {
        let mut flow = MockOpticalFlow::new();
        let of: &mut dyn OpticalFlow = &mut flow;
        of.init();
        of.push_gyro_bias(0.0, 0.0);
        of.push_gyro(0.5, -0.25, 0.02);
        let mut frame = DataFrame::zero();
        assert!(!of.read(&mut frame));
    }
}
