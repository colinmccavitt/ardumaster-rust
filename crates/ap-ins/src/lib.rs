//! Inertial sensor sample accumulation, upstream `AP_InertialSensor`. FW-011.
//!
//! This is the seam where sensor data enters the vehicle. Raw gyro and
//! accelerometer samples arrive here at the sensor's own rate -- faster than
//! the flight loop, and not synchronised to it -- and are accumulated into the
//! *delta angle* and *delta velocity* that [`ap_ahrs`] consumes once per loop.
//!
//! # Why deltas rather than rates
//!
//! An estimator that sampled the gyro once per loop would miss everything that
//! happened between samples. Integrating at the sensor rate and handing over
//! the integral instead means a vibration or a fast transient contributes its
//! actual rotation rather than whatever it happened to read at the instant the
//! loop looked. It also decouples the two rates: the loop can run at 50 Hz
//! against an 8 kHz sensor without aliasing.
//!
//! # Coning
//!
//! Rotations do not commute, so the integral of the rate vector is *not* the
//! rotation that occurred -- unless the rate vector keeps a fixed direction.
//! When it sweeps, as it does under vibration or any manoeuvre combining axes,
//! the naive integral is systematically wrong and the error accumulates in one
//! direction rather than averaging out. That is coning, and
//! [`ImuInstance::notify_gyro_raw_sample`] carries the correction for it.
//!
//! Upstream cites Tian et al (2010), *Three-loop Integration of GPS and
//! Strapdown INS with Coning and Sculling Compensation*, and departs from the
//! paper in one respect it documents: the paper accumulates the angles and the
//! coning corrections separately, upstream accumulates them together, having
//! found little difference in simulation.
//!
//! # What this slice does not include
//!
//! The **harmonic notch**. The two-pole low pass is here; the notch runs
//! ahead of it in upstream's chain, so when it lands it goes in front of
//! [`ImuInstance::set_gyro_filter`]'s filter rather than after. Also absent:
//! sculling compensation (upstream has none -- the delta velocity is a plain
//! rectangular sum), multi-instance selection, vibration and clipping
//! metrics, temperature calibration, board orientation, gyro/accel offset and
//! scale calibration, and the FFT window.

#![no_std]

use ap_filter::biquad::LowPassFilter2p;
use ap_math::vector3::Vector3f;

/// A gap this long between samples means the sensor was unhealthy, so the
/// accumulator is discarded rather than carried across the hole. Microseconds.
pub const UNHEALTHY_GAP_US: u64 = 100_000;

/// Below this measured rate a sensor with no per-sample timestamp is refused,
/// upstream's "don't accept below 40Hz". Hz.
pub const MIN_RAW_SAMPLE_RATE_HZ: u16 = 40;

/// Default gyro filter cutoff for a fixed-wing build, upstream
/// `DEFAULT_GYRO_FILTER`. Hz.
///
/// Upstream picks this per frame class: 20 for Copter and for everything that
/// is neither Copter nor Rover, and 4 for Rover. Plane takes the `#else`
/// branch, so 20. It is the default of a parameter (`INS_GYRO_FILTER`), not a
/// constant in the arithmetic -- the cutoff is passed to
/// [`ImuInstance::set_gyro_filter`] rather than read from here.
pub const DEFAULT_GYRO_FILTER_HZ: f32 = 20.0;

/// Default accelerometer filter cutoff for a fixed-wing build, upstream
/// `DEFAULT_ACCEL_FILTER`. Hz. Rover uses 10; Plane takes 20.
pub const DEFAULT_ACCEL_FILTER_HZ: f32 = 20.0;

/// Upstream's `MIN` macro, which is a ternary and therefore returns `b` when
/// either operand is NaN. `f32::min` returns the *non*-NaN operand instead, so
/// it is not a substitute here.
fn min_macro(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}

/// The flight loop's timing, as the accessors need it.
///
/// Upstream keeps these on the frontend; they are passed explicitly here per
/// ADR-0004.
#[derive(Debug, Clone, Copy)]
pub struct LoopTiming {
    /// Seconds since the previous loop's sample, upstream `_delta_time`.
    pub delta_time: f32,
    /// Ten times the nominal loop interval, upstream `_loop_delta_t_max`.
    /// Every interval handed to the estimator is clamped to this, so a stalled
    /// loop cannot ask the estimator to integrate over a huge step.
    pub loop_delta_t_max: f32,
}

impl LoopTiming {
    /// Timing for a loop running at `loop_delta_t` seconds per iteration.
    #[must_use]
    pub fn new(loop_delta_t: f32) -> Self {
        Self {
            delta_time: 0.0,
            loop_delta_t_max: 10.0 * loop_delta_t,
        }
    }

    /// The clamped loop interval, upstream `get_delta_time()`.
    #[must_use]
    pub fn delta_time(&self) -> f32 {
        min_macro(self.delta_time, self.loop_delta_t_max)
    }
}

/// One IMU's accumulation and published state.
///
/// Upstream splits this across `AP_InertialSensor` (the published values) and
/// `AP_InertialSensor_Backend` (the accumulation), with the backend reaching
/// into the frontend's per-instance arrays. There is no polymorphism in that
/// split to preserve -- the backend methods operate entirely on frontend state
/// -- so the port keeps one instance's worth of both in one place.
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuInstance {
    // ---- gyro accumulation ----
    delta_angle_acc: Vector3f,
    delta_angle_acc_dt: f32,
    /// Previous sample's delta angle, for the coning correction. Deliberately
    /// stored *without* its own coning term, following the paper.
    last_delta_angle: Vector3f,
    last_raw_gyro: Vector3f,
    gyro_last_sample_us: u64,

    // ---- accel accumulation ----
    delta_velocity_acc: Vector3f,
    delta_velocity_acc_dt: f32,
    accel_last_sample_us: u64,

    // ---- published once per loop ----
    delta_angle: Vector3f,
    delta_angle_dt: f32,
    delta_angle_valid: bool,
    delta_velocity: Vector3f,
    delta_velocity_dt: f32,
    delta_velocity_valid: bool,

    /// The most recent filtered gyro sample, published to the frontend on
    /// the next [`ImuInstance::update_gyro`].
    gyro_filtered: Vector3f,
    /// Likewise for the accelerometer.
    accel_filtered: Vector3f,

    gyro_filter: LowPassFilter2p<Vector3f>,
    accel_filter: LowPassFilter2p<Vector3f>,

    gyro: Vector3f,
    accel: Vector3f,
    gyro_healthy: bool,
    accel_healthy: bool,
    new_gyro_data: bool,
    new_accel_data: bool,
}

impl ImuInstance {
    /// A fresh instance with nothing accumulated.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Work out this sample's interval and timestamp, upstream's two-class
    /// split.
    ///
    /// FIFO-based sensors deliver samples in bunches at a predictable *overall*
    /// rate, so their interval comes from the measured rate; sensors that
    /// timestamp each sample get the interval from consecutive timestamps.
    /// Upstream distinguishes them by whether `sample_us` is supplied, and
    /// falls into the rate path for the first sample of either class because
    /// there is no previous timestamp to subtract.
    ///
    /// Returns `None` when the sample is refused.
    fn sample_interval(
        last_sample_us: &mut u64,
        sample_us: u64,
        raw_rate_hz: u16,
        now_us: u64,
    ) -> Option<(f32, u64)> {
        if sample_us != 0 && *last_sample_us != 0 {
            #[allow(
                clippy::cast_precision_loss,
                reason = "upstream converts the uint64 difference to float the same way; \
the difference is a sample interval, not an absolute time"
            )]
            let dt = (sample_us - *last_sample_us) as f32 * 1.0e-6;
            *last_sample_us = sample_us;
            return Some((dt, sample_us));
        }

        if raw_rate_hz < MIN_RAW_SAMPLE_RATE_HZ {
            // Record the timestamp anyway. Without this the bootstrap
            // deadlocks: the rate is measured from samples, and samples are
            // refused because the rate is too low.
            if sample_us != 0 {
                *last_sample_us = sample_us;
            }
            return None;
        }

        let dt = 1.0 / f32::from(raw_rate_hz);
        *last_sample_us = now_us;
        Some((dt, now_us))
    }

    /// Accumulate one raw gyro sample, upstream
    /// `_notify_new_gyro_raw_sample`.
    ///
    /// `now_us` is only consulted for sensors that do not timestamp their own
    /// samples; pass the current monotonic clock.
    ///
    /// # The first sample is always discarded
    ///
    /// With no previous timestamp the interval comes from the rate path, which
    /// stamps the sample with `now_us` -- and `now_us - 0` is far past the
    /// unhealthy-gap threshold, so the gap branch fires. That is upstream's
    /// behaviour and it is the right one: the trapezoidal average would
    /// otherwise pair the first real reading with a zeroed `last_raw_gyro` and
    /// report half the rotation that occurred.
    pub fn notify_gyro_raw_sample(
        &mut self,
        gyro: Vector3f,
        sample_us: u64,
        raw_rate_hz: u16,
        now_us: u64,
    ) {
        let last_sample_us = self.gyro_last_sample_us;
        let Some((mut dt, sample_us)) = Self::sample_interval(
            &mut self.gyro_last_sample_us,
            sample_us,
            raw_rate_hz,
            now_us,
        ) else {
            return;
        };

        // Trapezoidal rather than rectangular: the rate is taken as the
        // average of this sample and the last, which is exact for a rate
        // changing linearly across the interval.
        let mut delta_angle = (gyro + self.last_raw_gyro) * 0.5 * dt;

        // D-019: upstream computes the coning correction HERE, before the gap
        // check below invalidates the state it was computed from. The check is
        // hoisted above the correction instead. In the healthy case nothing
        // between the two touches the accumulator, so this is identical to
        // upstream; on a gap it stops a stale correction being written into a
        // freshly cleared accumulator.
        if sample_us.wrapping_sub(last_sample_us) > UNHEALTHY_GAP_US {
            self.delta_angle_acc = Vector3f::zero();
            self.delta_angle_acc_dt = 0.0;
            dt = 0.0;
            delta_angle = Vector3f::zero();
        }

        // The coning correction proper. Cross the accumulated rotation so far
        // (plus a sixth of the previous step, which is the paper's weighting)
        // with this step's rotation: the part of one that is perpendicular to
        // the other is exactly the rotation the naive integral loses.
        let delta_coning =
            (self.delta_angle_acc + self.last_delta_angle * (1.0 / 6.0)).cross(delta_angle) * 0.5;

        self.delta_angle_acc += delta_angle + delta_coning;
        self.delta_angle_acc_dt += dt;

        // Without its coning term, per the paper.
        self.last_delta_angle = delta_angle;
        self.last_raw_gyro = gyro;

        // Upstream runs the harmonic notch first and the low pass last, so
        // the low pass attenuates whatever noise the notch introduces. The
        // notch is not ported; when it lands it goes ahead of this.
        let filtered = self.gyro_filter.apply(gyro);
        if filtered.is_nan() || filtered.is_inf() {
            // Reset and keep the last good value rather than publish a NaN.
            self.gyro_filter.reset();
        } else {
            self.gyro_filtered = filtered;
        }
        self.new_gyro_data = true;
    }

    /// Accumulate one raw accelerometer sample, upstream
    /// `_notify_new_accel_raw_sample`.
    ///
    /// There is no sculling correction -- the delta velocity is a plain
    /// rectangular sum. That is upstream's choice, not an omission here: the
    /// sculling term matters at vibration amplitudes where the accelerometer
    /// is already clipping.
    pub fn notify_accel_raw_sample(
        &mut self,
        accel: Vector3f,
        sample_us: u64,
        raw_rate_hz: u16,
        now_us: u64,
    ) {
        let last_sample_us = self.accel_last_sample_us;
        let Some((mut dt, sample_us)) = Self::sample_interval(
            &mut self.accel_last_sample_us,
            sample_us,
            raw_rate_hz,
            now_us,
        ) else {
            return;
        };

        if sample_us.wrapping_sub(last_sample_us) > UNHEALTHY_GAP_US {
            self.delta_velocity_acc = Vector3f::zero();
            self.delta_velocity_acc_dt = 0.0;
            dt = 0.0;
        }

        // Note the contrast with the gyro path: zeroing `dt` is enough here,
        // because it multiplies the only term. Nothing survives the gap. That
        // asymmetry is why the gyro path's stale coning term reads as an
        // oversight rather than a decision.
        self.delta_velocity_acc += accel * dt;
        self.delta_velocity_acc_dt += dt;

        // D-019's sibling. Upstream assigns the filter's output to
        // `_accel_filtered` and only *then* checks it, resetting the filter but
        // leaving the NaN in place to be published. The gyro path a few
        // hundred lines up guards against exactly that by restoring the
        // previous value; this mirrors it. See D-020.
        let filtered = self.accel_filter.apply(accel);
        if filtered.is_nan() || filtered.is_inf() {
            self.accel_filter.reset();
        } else {
            self.accel_filtered = filtered;
        }
        self.new_accel_data = true;
    }

    /// Hand the accumulated rotation to the flight loop, upstream
    /// `update_gyro` and `_publish_gyro`.
    ///
    /// Does nothing without a new sample, which is what leaves the accumulator
    /// intact across a sensor stall -- and is what makes the gap branch in
    /// [`Self::notify_gyro_raw_sample`] reachable with a non-empty
    /// accumulator.
    pub fn update_gyro(&mut self) {
        if !self.new_gyro_data {
            return;
        }
        self.gyro = self.gyro_filtered;
        self.gyro_healthy = true;

        self.delta_angle = self.delta_angle_acc;
        self.delta_angle_dt = self.delta_angle_acc_dt;
        self.delta_angle_valid = true;

        self.delta_angle_acc = Vector3f::zero();
        self.delta_angle_acc_dt = 0.0;
        self.new_gyro_data = false;
    }

    /// Hand the accumulated velocity change to the flight loop, upstream
    /// `update_accel` and `_publish_accel`.
    pub fn update_accel(&mut self) {
        if !self.new_accel_data {
            return;
        }
        self.accel = self.accel_filtered;
        self.accel_healthy = true;

        self.delta_velocity = self.delta_velocity_acc;
        self.delta_velocity_dt = self.delta_velocity_acc_dt;
        self.delta_velocity_valid = true;

        self.delta_velocity_acc = Vector3f::zero();
        self.delta_velocity_acc_dt = 0.0;
        self.new_accel_data = false;
    }

    /// The rotation since the last loop and the interval it covers, upstream
    /// `get_delta_angle`.
    ///
    /// Falls back to `gyro * loop_interval` when no accumulated value is
    /// available, so the estimator can use one code path either way. The
    /// fallback is a rectangular approximation with no coning correction, and
    /// is strictly worse -- it exists so a sensor that reports only rates
    /// still flies.
    #[must_use]
    pub fn get_delta_angle(&self, timing: &LoopTiming) -> Option<(Vector3f, f32)> {
        let dt = if self.delta_angle_valid && self.delta_angle_dt > 0.0 {
            self.delta_angle_dt
        } else {
            timing.delta_time()
        };
        let dt = min_macro(dt, timing.loop_delta_t_max);

        if self.delta_angle_valid {
            Some((self.delta_angle, dt))
        } else if self.gyro_healthy {
            Some((self.gyro * timing.delta_time(), dt))
        } else {
            None
        }
    }

    /// The velocity change since the last loop and the interval it covers,
    /// upstream `get_delta_velocity`.
    #[must_use]
    pub fn get_delta_velocity(&self, timing: &LoopTiming) -> Option<(Vector3f, f32)> {
        let dt = if self.delta_velocity_valid {
            self.delta_velocity_dt
        } else {
            timing.delta_time()
        };
        let dt = min_macro(dt, timing.loop_delta_t_max);

        if self.delta_velocity_valid {
            Some((self.delta_velocity, dt))
        } else if self.accel_healthy {
            Some((self.accel * timing.delta_time(), dt))
        } else {
            None
        }
    }

    /// Set the gyro low-pass cutoff, upstream `update_gyro_filters`.
    ///
    /// Until this is called the filter passes samples through untouched, which
    /// is upstream's zero-initialised state too. Retuning does not disturb the
    /// filter's state -- upstream retunes in flight and a reset would step the
    /// gyro signal each time.
    pub fn set_gyro_filter(&mut self, sample_rate_hz: f32, cutoff_hz: f32) {
        self.gyro_filter
            .set_cutoff_frequency(sample_rate_hz, cutoff_hz);
    }

    /// Set the accelerometer low-pass cutoff, upstream
    /// `update_accel_filters`.
    pub fn set_accel_filter(&mut self, sample_rate_hz: f32, cutoff_hz: f32) {
        self.accel_filter
            .set_cutoff_frequency(sample_rate_hz, cutoff_hz);
    }

    /// The most recent published gyro reading, upstream `get_gyro`.
    #[must_use]
    pub const fn gyro(&self) -> Vector3f {
        self.gyro
    }

    /// The most recent published accelerometer reading, upstream `get_accel`.
    #[must_use]
    pub const fn accel(&self) -> Vector3f {
        self.accel
    }

    /// Whether a gyro sample has been published, upstream `get_gyro_health`.
    #[must_use]
    pub const fn gyro_healthy(&self) -> bool {
        self.gyro_healthy
    }

    /// Whether an accelerometer sample has been published, upstream
    /// `get_accel_health`.
    #[must_use]
    pub const fn accel_healthy(&self) -> bool {
        self.accel_healthy
    }

    /// The rotation accumulated but not yet published, and its interval.
    ///
    /// Not an upstream interface. Exposed because the accumulate/publish split
    /// is otherwise invisible from outside, and the gap behaviour is stated in
    /// terms of it.
    #[must_use]
    pub const fn pending_delta_angle(&self) -> (Vector3f, f32) {
        (self.delta_angle_acc, self.delta_angle_acc_dt)
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "every comparison here asserts a value is exactly zero -- nothing \naccumulated, no interval credited. An epsilon would let through precisely the leftovers \nthese tests exist to catch"
    )]

    use super::*;

    /// 8 kHz sensor, 400 Hz loop.
    const DT_US: u64 = 125;

    fn feed(imu: &mut ImuInstance, gyro: Vector3f, t_us: &mut u64) {
        *t_us += DT_US;
        imu.notify_gyro_raw_sample(gyro, *t_us, 8000, *t_us);
    }

    /// PORT-DERIVED. The first sample is discarded, because pairing it with a
    /// zeroed `last_raw_gyro` would report half the rotation.
    #[test]
    fn the_first_sample_is_discarded() {
        let mut imu = ImuInstance::new();
        imu.notify_gyro_raw_sample(Vector3f::new(1.0, 0.0, 0.0), 1_000_000, 8000, 1_000_000);
        let (acc, dt) = imu.pending_delta_angle();
        assert_eq!(acc, Vector3f::zero());
        assert_eq!(dt, 0.0);
    }

    /// PORT-DERIVED. A steady rate about one axis has no coning at all -- the
    /// rate vector never changes direction -- so the accumulated angle is
    /// exactly rate times time, and any spurious coning term would show up
    /// here immediately.
    #[test]
    fn a_single_axis_rate_accumulates_exactly() {
        let mut imu = ImuInstance::new();
        let mut t = 1_000_000_u64;
        let gyro = Vector3f::new(0.5, 0.0, 0.0);

        // One discarded, then 800 samples at 125us = 0.1s
        feed(&mut imu, gyro, &mut t);
        for _ in 0..800 {
            feed(&mut imu, gyro, &mut t);
        }

        let (acc, dt) = imu.pending_delta_angle();
        assert!((dt - 0.1).abs() < 1e-5, "interval {dt}");
        assert!((acc.x - 0.05).abs() < 1e-5, "0.5 rad/s for 0.1s: {}", acc.x);
        assert_eq!(acc.y, 0.0);
        assert_eq!(acc.z, 0.0);
    }

    /// The divergence, stated as a test. After a stall the accumulator must be
    /// empty -- upstream seeds it with a coning term computed from the
    /// pre-stall state it has just declared invalid.
    #[test]
    fn a_stall_leaves_nothing_behind() {
        let mut imu = ImuInstance::new();
        let mut t = 1_000_000_u64;

        // Build a real accumulation on a sweeping rate, so both the
        // accumulator and the last delta angle are non-zero and not parallel.
        feed(&mut imu, Vector3f::new(0.0, 1.0, 0.0), &mut t);
        for i in 0_i16..100 {
            let f = f32::from(i) * 0.01;
            feed(&mut imu, Vector3f::new(0.0, 1.0 - f, f), &mut t);
        }
        let (before, _) = imu.pending_delta_angle();
        assert!(
            before.length() > 0.001,
            "the setup must accumulate something"
        );

        // Stall for 200ms, then a sample arrives.
        t += 200_000;
        imu.notify_gyro_raw_sample(Vector3f::new(0.0, 0.0, 1.0), t, 8000, t);

        let (acc, dt) = imu.pending_delta_angle();
        assert_eq!(dt, 0.0, "no interval should be credited across the gap");
        assert_eq!(
            acc,
            Vector3f::zero(),
            "the accumulator must be empty after a stall, got {acc:?} -- upstream \
leaves a coning term here computed from state it just discarded"
        );
    }

    /// PORT-DERIVED. The published value is the accumulation, and the
    /// accumulator restarts empty.
    #[test]
    fn publishing_drains_the_accumulator() {
        let mut imu = ImuInstance::new();
        let mut t = 1_000_000_u64;
        feed(&mut imu, Vector3f::new(0.5, 0.0, 0.0), &mut t);
        for _ in 0..80 {
            feed(&mut imu, Vector3f::new(0.5, 0.0, 0.0), &mut t);
        }
        let (pending, pending_dt) = imu.pending_delta_angle();

        imu.update_gyro();
        let timing = LoopTiming::new(0.0025);
        let (published, dt) = imu
            .get_delta_angle(&timing)
            .expect("a sample was published");

        assert_eq!(published, pending);
        assert!((dt - pending_dt).abs() < 1e-7);
        assert_eq!(imu.pending_delta_angle(), (Vector3f::zero(), 0.0));
    }

    /// PORT-DERIVED. Nothing published and no healthy gyro means no answer --
    /// the estimator must not be handed a zero it would mistake for "no
    /// rotation".
    #[test]
    fn nothing_published_and_no_gyro_gives_nothing() {
        let imu = ImuInstance::new();
        let timing = LoopTiming::new(0.0025);
        assert!(imu.get_delta_angle(&timing).is_none());
        assert!(imu.get_delta_velocity(&timing).is_none());
    }

    /// A stalled flight loop must not ask the estimator to integrate over a
    /// huge step, so the interval is clamped at ten loop times.
    #[test]
    fn a_stalled_loop_interval_is_clamped() {
        let mut imu = ImuInstance::new();
        let mut t = 1_000_000_u64;
        feed(&mut imu, Vector3f::new(0.5, 0.0, 0.0), &mut t);
        for _ in 0..8000 {
            feed(&mut imu, Vector3f::new(0.5, 0.0, 0.0), &mut t);
        }
        imu.update_gyro();

        // 1s of accumulation against a 2.5ms loop: clamped to 25ms.
        let timing = LoopTiming::new(0.0025);
        let (_, dt) = imu.get_delta_angle(&timing).expect("published");
        assert!(
            (dt - 0.025).abs() < 1e-7,
            "expected the 25ms clamp, got {dt}"
        );
    }

    /// PORT-DERIVED. Delta velocity is a plain rectangular sum, and a stall
    /// clears it -- the accel path has no coning term to leave behind.
    #[test]
    fn delta_velocity_accumulates_and_clears() {
        let mut imu = ImuInstance::new();
        let mut t = 1_000_000_u64;
        let accel = Vector3f::new(0.0, 0.0, -9.80665);

        t += DT_US;
        imu.notify_accel_raw_sample(accel, t, 8000, t);
        for _ in 0..800 {
            t += DT_US;
            imu.notify_accel_raw_sample(accel, t, 8000, t);
        }
        imu.update_accel();
        let timing = LoopTiming::new(0.0025);
        let (dv, dt) = imu.get_delta_velocity(&timing).expect("published");
        // 0.025 is 0.024999999 as an f32; a tighter tolerance than this is
        // below the type's own resolution.
        assert!((dt - 0.025).abs() < 1e-7, "accel dt {dt}");
        assert!(
            (dv.z - (-9.80665 * 0.1)).abs() < 1e-4,
            "one g for 0.1s: {}",
            dv.z
        );

        t += 200_000;
        imu.notify_accel_raw_sample(accel, t, 8000, t);
        let (acc_after, dt_after) = (imu.delta_velocity_acc, imu.delta_velocity_acc_dt);
        assert_eq!(acc_after, Vector3f::zero());
        assert_eq!(dt_after, 0.0);
    }

    /// The filter is in the path now, not just a field. Unconfigured it passes
    /// through, which is what keeps every other test in this module valid.
    #[test]
    fn an_unconfigured_filter_passes_samples_through() {
        let mut imu = ImuInstance::new();
        let mut t = 1_000_000_u64;
        feed(&mut imu, Vector3f::new(0.5, 0.0, 0.0), &mut t);
        feed(&mut imu, Vector3f::new(0.5, 0.0, 0.0), &mut t);
        imu.update_gyro();
        assert_eq!(imu.gyro(), Vector3f::new(0.5, 0.0, 0.0));
    }

    /// Configured, it actually filters: a step does not reach the published
    /// gyro immediately.
    #[test]
    fn a_configured_filter_smooths_a_step() {
        let mut imu = ImuInstance::new();
        imu.set_gyro_filter(8000.0, DEFAULT_GYRO_FILTER_HZ);
        let mut t = 1_000_000_u64;

        // Seed at zero, then step to 1 rad/s.
        feed(&mut imu, Vector3f::zero(), &mut t);
        feed(&mut imu, Vector3f::zero(), &mut t);
        imu.update_gyro();
        feed(&mut imu, Vector3f::new(1.0, 0.0, 0.0), &mut t);
        imu.update_gyro();
        let just_after = imu.gyro().x;
        assert!(
            just_after < 0.01,
            "a step should be attenuated, got {just_after}"
        );

        for _ in 0..8000 {
            feed(&mut imu, Vector3f::new(1.0, 0.0, 0.0), &mut t);
        }
        imu.update_gyro();
        assert!(
            (imu.gyro().x - 1.0).abs() < 0.01,
            "and it should settle at unity gain, got {}",
            imu.gyro().x
        );
    }

    /// D-020. A NaN out of the accelerometer filter must not be published.
    /// Upstream assigns it first and checks second, so it publishes the NaN
    /// for one cycle and hands it to the AHRS.
    #[test]
    fn d020_a_nan_does_not_reach_the_published_accel() {
        let mut imu = ImuInstance::new();
        imu.set_accel_filter(8000.0, DEFAULT_ACCEL_FILTER_HZ);
        let mut t = 1_000_000_u64;
        let good = Vector3f::new(0.0, 0.0, -9.80665);

        t += DT_US;
        imu.notify_accel_raw_sample(good, t, 8000, t);
        for _ in 0..100 {
            t += DT_US;
            imu.notify_accel_raw_sample(good, t, 8000, t);
        }
        imu.update_accel();
        let before = imu.accel();
        assert!(!before.is_nan());

        t += DT_US;
        imu.notify_accel_raw_sample(Vector3f::new(f32::NAN, 0.0, 0.0), t, 8000, t);
        imu.update_accel();

        assert!(
            !imu.accel().is_nan(),
            "a NaN sample must not be published, got {:?}",
            imu.accel()
        );
        assert_eq!(imu.accel(), before, "the last good value should stand");
    }

    /// And the same guard on the gyro side, which upstream does have.
    #[test]
    fn a_nan_does_not_reach_the_published_gyro() {
        let mut imu = ImuInstance::new();
        imu.set_gyro_filter(8000.0, DEFAULT_GYRO_FILTER_HZ);
        let mut t = 1_000_000_u64;
        feed(&mut imu, Vector3f::new(0.5, 0.0, 0.0), &mut t);
        for _ in 0..100 {
            feed(&mut imu, Vector3f::new(0.5, 0.0, 0.0), &mut t);
        }
        imu.update_gyro();
        let before = imu.gyro();

        feed(&mut imu, Vector3f::new(f32::INFINITY, 0.0, 0.0), &mut t);
        imu.update_gyro();
        assert!(!imu.gyro().is_nan() && !imu.gyro().is_inf());
        assert_eq!(imu.gyro(), before);
    }

    /// PORT-DERIVED. A sensor that neither timestamps its samples nor reports
    /// a usable rate is refused outright.
    #[test]
    fn an_untimestamped_slow_sensor_is_refused() {
        let mut imu = ImuInstance::new();
        imu.notify_gyro_raw_sample(Vector3f::new(1.0, 0.0, 0.0), 0, 30, 1_000_000);
        assert_eq!(imu.pending_delta_angle(), (Vector3f::zero(), 0.0));
        assert!(!imu.gyro_healthy());
    }
}
