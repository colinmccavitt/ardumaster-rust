//! SITL inertial sensor backend, upstream `AP_InertialSensor_SITL`. FW-011.
//!
//! This slice is the deterministic path from simulator body-frame kinematics to
//! the raw samples [`ImuInstance`] accumulates: trim, scale, bias, board
//! orientation, lever-arm corrections, and the timer that decides when each
//! sample is due.
//!
//! Random sensor noise (white noise and vibration), RPM-scaled motor
//! harmonics are implemented; temperature *calibration* application and
//! file playback are not here yet. The IMU warm-up temperature curve and
//! per-instance fail masks are implemented.

use ap_math::rotations_gen::{rotate, Rotation};
use ap_math::scalar::{is_zero, radians, wrap_pi, Real};
use ap_math::vector3::Vector3f;

use crate::ImuInstance;

/// Body-frame kinematics as the simulator reports them.
///
/// Rates are degrees per second; accelerations are m/s² in body frame.
#[derive(Debug, Clone, Copy, Default)]
pub struct SitlBodyState {
    /// Roll rate, deg/s.
    pub roll_rate_dps: f32,
    /// Pitch rate, deg/s.
    pub pitch_rate_dps: f32,
    /// Yaw rate, deg/s.
    pub yaw_rate_dps: f32,
    /// Body X acceleration, m/s².
    pub x_accel: f32,
    /// Body Y acceleration, m/s².
    pub y_accel: f32,
    /// Body Z acceleration, m/s².
    pub z_accel: f32,
    /// Roll angular acceleration, deg/s² — used for lever-arm correction.
    pub roll_accel_dps2: f32,
    /// Pitch angular acceleration, deg/s².
    pub pitch_accel_dps2: f32,
    /// Yaw angular acceleration, deg/s².
    pub yaw_accel_dps2: f32,
}

/// Calibration and mounting applied to SITL samples before accumulation.
#[derive(Debug, Clone, Copy)]
pub struct SitlImuCalibration {
    /// Board rotation relative to the airframe.
    pub orientation: Rotation,
    /// Accelerometer trim, radians.
    pub accel_trim: Vector3f,
    /// Per-axis scale divisors.
    pub accel_scale: Vector3f,
    /// Constant accelerometer bias, m/s².
    pub accel_bias: Vector3f,
    /// Per-axis gyro scale error, percent.
    pub gyro_scale: Vector3f,
    /// Constant gyro bias, rad/s.
    pub gyro_bias: Vector3f,
    /// IMU position offset from body origin, metres.
    pub imu_pos_offset: Vector3f,
    /// When non-zero, every accel axis is replaced with this value.
    pub accel_fail: f32,
}

impl Default for SitlImuCalibration {
    fn default() -> Self {
        Self {
            orientation: Rotation::None,
            accel_trim: Vector3f::zero(),
            accel_scale: Vector3f::zero(),
            accel_bias: Vector3f::zero(),
            gyro_scale: Vector3f::zero(),
            gyro_bias: Vector3f::zero(),
            imu_pos_offset: Vector3f::zero(),
            accel_fail: 0.0,
        }
    }
}

/// IMU warm-up temperature parameters, upstream SITL `imu_temp_*`.
#[derive(Debug, Clone, Copy)]
pub struct SitlImuTemperature {
    /// When non-zero, temperature is fixed at this value (°C).
    pub temp_fixed_c: f32,
    /// Starting temperature before warm-up (°C).
    pub temp_start_c: f32,
    /// Asymptotic temperature after warm-up (°C).
    pub temp_end_c: f32,
    /// Time constant for the exponential warm-up curve (seconds).
    pub temp_tconst_s: f32,
}

impl Default for SitlImuTemperature {
    fn default() -> Self {
        Self {
            temp_fixed_c: 0.0,
            temp_start_c: 20.0,
            temp_end_c: 45.0,
            temp_tconst_s: 300.0,
        }
    }
}

/// IMU temperature at `elapsed_ms` since the backend started, upstream
/// `get_temperature`.
#[must_use]
pub fn sitl_imu_temperature(config: &SitlImuTemperature, elapsed_ms: u32) -> f32 {
    if !is_zero(config.temp_fixed_c) {
        return config.temp_fixed_c;
    }
    #[allow(clippy::cast_precision_loss, reason = "milliseconds fit in f32 for SITL dt")]
    let tsec = elapsed_ms as f32 * 0.001;
    let t0 = config.temp_start_c;
    let t1 = config.temp_end_c;
    let tconst = config.temp_tconst_s;
    t1 - (t1 - t0) * Real::exp(-tsec / tconst)
}

/// Return true when instance `index` is masked out of sample generation.
#[must_use]
pub fn sitl_instance_failed(fail_mask: u32, index: u8) -> bool {
    (fail_mask & (1_u32 << index)) != 0
}

/// Motor vibration parameters for SITL noise injection.
#[derive(Debug, Clone, Copy)]
pub struct SitlVibeConfig {
    /// Per-axis vibration frequency, Hz.
    pub vibe_freq_hz: Vector3f,
    /// Base accelerometer noise amplitude, m/s².
    pub accel_noise: f32,
    /// Fractional variation applied via `sitl_calculate_noise`.
    pub noise_variation: f32,
    /// True when throttle is above the INS noise threshold.
    pub motors_on: bool,
}

/// Default accelerometer white-noise floor, upstream the 0.01 m/s² minimum.
pub const SITL_DEFAULT_ACCEL_NOISE: f32 = 0.01;

/// Default gyro white-noise floor, upstream `radians(0.04f)`.
pub const SITL_DEFAULT_GYRO_NOISE_RAD: f32 = 0.04 * core::f32::consts::PI / 180.0;

/// Scale a noise amplitude by a deterministic random unit, upstream
/// `calculate_noise` with `rand_float()` replaced by `rand_unit` in [-1, 1].
#[must_use]
pub fn sitl_calculate_noise(noise: f32, noise_variation: f32, rand_unit: f32) -> f32 {
    noise * (1.0 + noise_variation * rand_unit)
}

/// Per-axis white noise offset, upstream `Vector3f{rand_float(), ...} * noise`.
#[must_use]
pub fn sitl_white_noise_offset(rand_unit: Vector3f, amplitude: f32) -> Vector3f {
    Vector3f::new(
        rand_unit.x * amplitude,
        rand_unit.y * amplitude,
        rand_unit.z * amplitude,
    )
}

/// Active accelerometer noise amplitude, upstream the motor-on branch in
/// `generate_accel`.
#[must_use]
pub fn sitl_accel_noise_amplitude(base: f32, motor: f32, motors_on: bool) -> f32 {
    if motors_on { motor } else { base }
}

/// Active gyro noise amplitude, upstream the motor-on branch in `generate_gyro`.
#[must_use]
pub fn sitl_gyro_noise_amplitude(
    base_rad: f32,
    motor_noise_deg: f32,
    throttle: f32,
    motors_on: bool,
) -> f32 {
    if motors_on {
        radians(motor_noise_deg) * throttle
    } else {
        base_rad
    }
}

/// Whether gyro gets an extra background noise term, upstream the block that
/// runs when both `vibe_freq` and `vibe_motor` are zero.
#[must_use]
pub fn sitl_gyro_needs_background_noise(vibe_freq_zero: bool, vibe_motor_zero: bool) -> bool {
    vibe_freq_zero && vibe_motor_zero
}

/// Fixed-frequency vibration offset for one accel/gyro sample, upstream
/// `sinf(time * 2 * PI * vibe_freq) * calculate_noise(...)`.
#[must_use]
pub fn sitl_vibe_freq_offset(config: &SitlVibeConfig, time_s: f32, rand_unit: f32) -> Vector3f {
    if !config.motors_on || config.vibe_freq_hz.is_zero() {
        return Vector3f::zero();
    }
    let amp = sitl_calculate_noise(config.accel_noise, config.noise_variation, rand_unit);
    let two_pi = core::f32::consts::PI * 2.0;
    Vector3f::new(
        libm::sinf(time_s * two_pi * config.vibe_freq_hz.x) * amp,
        libm::sinf(time_s * two_pi * config.vibe_freq_hz.y) * amp,
        libm::sinf(time_s * two_pi * config.vibe_freq_hz.z) * amp,
    )
}

/// Motor RPM-scaled vibration parameters, upstream the `VIB_MOT_MAX` block.
#[derive(Debug, Clone, Copy)]
pub struct SitlMotorVibeConfig {
    /// Master enable; zero disables motor vibration.
    pub vibe_motor: f32,
    /// Scales accelerometer noise for motor harmonics.
    pub vibe_motor_scale: f32,
    /// Harmonic bitmask — set bit *n* (1-based) adds a term at `phase * n`.
    pub vibe_motor_harmonics: u32,
    pub accel_noise: f32,
    pub noise_variation: f32,
    pub freq_variation: f32,
    pub motors_on: bool,
}

/// Harmonic vibration offset for one motor at the current phase, upstream the
/// inner loop over `vibe_motor_harmonics`.
#[must_use]
pub fn sitl_motor_vibe_harmonics_offset(
    motor_phase: f32,
    harmonics: u32,
    accel_noise: f32,
    vibe_motor_scale: f32,
    noise_variation: f32,
    rand_unit: f32,
) -> Vector3f {
    if harmonics == 0 {
        return Vector3f::zero();
    }
    let amp = sitl_calculate_noise(accel_noise * vibe_motor_scale, noise_variation, rand_unit);
    let mut offset = Vector3f::zero();
    let mut remaining = harmonics;
    while remaining != 0 {
        let bit = remaining.trailing_zeros() + 1;
        remaining &= !(1_u32 << (bit - 1));
        let s = libm::sinf(motor_phase * bit as f32) * amp;
        offset.x += s;
        offset.y += s;
        offset.z += s;
    }
    offset
}

/// Advance one motor's vibration phase after a sample, upstream
/// `accel_motor_phase[motor] = wrap_PI(phase + phase_incr)`.
#[must_use]
pub fn sitl_motor_phase_advance(
    rpm: f32,
    freq_variation: f32,
    rand_unit: f32,
    motor_phase: f32,
    sample_dt_s: f32,
) -> f32 {
    let base_freq = sitl_calculate_noise(rpm / 60.0, freq_variation, rand_unit);
    let phase_incr = base_freq * 2.0 * core::f32::consts::PI * sample_dt_s;
    wrap_pi(motor_phase + phase_incr)
}

/// Accumulate motor vibration across every bit set in `motor_mask`.
#[must_use]
pub fn sitl_motor_vibe_offset(
    config: &SitlMotorVibeConfig,
    motor_mask: u32,
    motor_rpm: &[f32],
    motor_phases: &mut [f32],
    sample_dt_s: f32,
    rand_unit: f32,
) -> Vector3f {
    if !config.motors_on || is_zero(config.vibe_motor) || config.vibe_motor_harmonics == 0 {
        return Vector3f::zero();
    }
    let mut total = Vector3f::zero();
    let mut mask = motor_mask;
    while mask != 0 {
        let motor = mask.trailing_zeros() as usize;
        mask &= !(1_u32 << motor);
        if motor >= motor_rpm.len() || motor >= motor_phases.len() {
            continue;
        }
        total += sitl_motor_vibe_harmonics_offset(
            motor_phases[motor],
            config.vibe_motor_harmonics,
            config.accel_noise,
            config.vibe_motor_scale,
            config.noise_variation,
            rand_unit,
        );
        motor_phases[motor] = sitl_motor_phase_advance(
            motor_rpm[motor],
            config.freq_variation,
            rand_unit,
            motor_phases[motor],
            sample_dt_s,
        );
    }
    total
}

/// The slow triangular gyro drift SITL can inject, upstream `gyro_drift()`.
///
/// Returns zero when either parameter is zero. `now_us` is monotonic time.
#[must_use]
pub fn sitl_gyro_drift(now_us: u64, drift_speed_dps: f32, drift_time_min: f32) -> f32 {
    if is_zero(drift_speed_dps) || is_zero(drift_time_min) {
        return 0.0;
    }
    let period = f64::from(drift_time_min) * 2.0;
    let minutes = libm::fmod(now_us as f64 / 60.0e6, period);
    if minutes < period / 2.0 {
        (minutes * f64::from(radians(drift_speed_dps))) as f32
    } else {
        ((period - minutes) * f64::from(radians(drift_speed_dps))) as f32
    }
}

/// Transform simulator accelerometer data into a raw sample, upstream
/// `generate_accel` without noise or vibration.
#[must_use]
pub fn sitl_accel_sample(
    state: &SitlBodyState,
    cal: &SitlImuCalibration,
) -> Vector3f {
    let mut accel = Vector3f::new(state.x_accel, state.y_accel, state.z_accel);

    if !cal.accel_trim.is_zero() {
        accel = apply_accel_trim(accel, cal.accel_trim);
    }

    if !is_zero(cal.accel_scale.x) {
        accel.x /= cal.accel_scale.x;
    }
    if !is_zero(cal.accel_scale.y) {
        accel.y /= cal.accel_scale.y;
    }
    if !is_zero(cal.accel_scale.z) {
        accel.z /= cal.accel_scale.z;
    }

    accel += cal.accel_bias;

    if !cal.imu_pos_offset.is_zero() {
        let angular_accel = Vector3f::new(
            radians(state.roll_accel_dps2),
            radians(state.pitch_accel_dps2),
            radians(state.yaw_accel_dps2),
        );
        let angular_rate = Vector3f::new(
            radians(state.roll_rate_dps),
            radians(state.pitch_rate_dps),
            radians(state.yaw_rate_dps),
        );
        let lever_arm = angular_accel.cross(cal.imu_pos_offset);
        let centripetal = angular_rate.cross(angular_rate.cross(cal.imu_pos_offset));
        accel += lever_arm + centripetal;
    }

    if libm::fabsf(cal.accel_fail) > 1.0e-6 {
        accel = Vector3f::new(cal.accel_fail, cal.accel_fail, cal.accel_fail);
    }

    let mut rotated = accel;
    let _ = rotate(&mut rotated, cal.orientation);
    rotated
}

/// Transform simulator gyro data into a raw sample, upstream `generate_gyro`
/// without noise or vibration.
#[must_use]
pub fn sitl_gyro_sample(
    state: &SitlBodyState,
    cal: &SitlImuCalibration,
    gyro_drift: f32,
) -> Vector3f {
    let mut gyro = Vector3f::new(
        radians(state.roll_rate_dps) + gyro_drift,
        radians(state.pitch_rate_dps) + gyro_drift,
        radians(state.yaw_rate_dps) + gyro_drift,
    );

    gyro.x *= 1.0 + cal.gyro_scale.x * 0.01;
    gyro.y *= 1.0 + cal.gyro_scale.y * 0.01;
    gyro.z *= 1.0 + cal.gyro_scale.z * 0.01;
    gyro += cal.gyro_bias;

    let mut rotated = gyro;
    let _ = rotate(&mut rotated, cal.orientation);
    rotated
}

fn apply_accel_trim(accel: Vector3f, trim: Vector3f) -> Vector3f {
    // Upstream: trim_rotation.from_euler(accel_trim.x, accel_trim.y, 0);
    // accel = trim_rotation.transposed() * accel
    // For small angles this is equivalent to rotating by -trim around x then y.
    let mut out = accel;
    for (angle, axis) in [(trim.x, 0), (trim.y, 1)] {
        if is_zero(angle) {
            continue;
        }
        let c = Real::cos(-angle);
        let s = Real::sin(-angle);
        match axis {
            0 => {
                let y = out.y * c - out.z * s;
                let z = out.y * s + out.z * c;
                out.y = y;
                out.z = z;
            }
            _ => {
                let x = out.x * c + out.z * s;
                let z = -out.x * s + out.z * c;
                out.x = x;
                out.z = z;
            }
        }
    }
    out
}

/// Sample scheduling and delivery for one SITL IMU instance.
#[derive(Debug, Clone)]
pub struct SitlImuBackend {
    /// The IMU instance this backend feeds.
    pub imu: ImuInstance,
    /// Trim, scale, bias, and mounting.
    pub cal: SitlImuCalibration,
    /// Gyro sample rate, Hz.
    pub gyro_rate_hz: u16,
    /// Accelerometer sample rate, Hz.
    pub accel_rate_hz: u16,
    next_gyro_sample_us: u64,
    next_accel_sample_us: u64,
    /// SITL gyro drift speed, deg/s.
    pub drift_speed_dps: f32,
    /// SITL gyro drift period, minutes.
    pub drift_time_min: f32,
    /// Backend instance index for fail-mask checks.
    pub instance_index: u8,
    /// Bit mask: set bits skip accelerometer sample generation.
    pub accel_fail_mask: u32,
    /// Bit mask: set bits skip gyro sample generation.
    pub gyro_fail_mask: u32,
    /// Warm-up temperature model parameters.
    pub temperature: SitlImuTemperature,
    temp_start_ms: Option<u32>,
    /// Most recently computed IMU temperature (°C).
    pub last_temperature_c: f32,
}

impl SitlImuBackend {
    /// A backend running at the given sample rates.
    #[must_use]
    pub fn new(gyro_rate_hz: u16, accel_rate_hz: u16) -> Self {
        Self {
            imu: ImuInstance::new(),
            cal: SitlImuCalibration::default(),
            gyro_rate_hz,
            accel_rate_hz,
            next_gyro_sample_us: 0,
            next_accel_sample_us: 0,
            drift_speed_dps: 0.0,
            drift_time_min: 0.0,
            instance_index: 0,
            accel_fail_mask: 0,
            gyro_fail_mask: 0,
            temperature: SitlImuTemperature::default(),
            temp_start_ms: None,
            last_temperature_c: 20.0,
        }
    }

    /// Advance the timer and feed any due samples, upstream `timer_update`.
    ///
    /// Returns how many gyro and accel samples were delivered.
    pub fn timer_update(&mut self, now_us: u64, state: &SitlBodyState) -> (u32, u32) {
        let mut gyro_count = 0_u32;
        let mut accel_count = 0_u32;

        let now_ms = (now_us / 1000) as u32;
        if self.temp_start_ms.is_none() {
            self.temp_start_ms = Some(now_ms);
        }
        let elapsed_ms = now_ms.wrapping_sub(self.temp_start_ms.unwrap_or(now_ms));
        self.last_temperature_c = sitl_imu_temperature(&self.temperature, elapsed_ms);

        if now_us >= self.next_accel_sample_us
            && !sitl_instance_failed(self.accel_fail_mask, self.instance_index)
        {
            let sample = sitl_accel_sample(state, &self.cal);
            self.imu
                .notify_accel_raw_sample(sample, now_us, self.accel_rate_hz, now_us);
            self.advance_accel_schedule(now_us);
            accel_count = 1;
        }

        if now_us >= self.next_gyro_sample_us
            && !sitl_instance_failed(self.gyro_fail_mask, self.instance_index)
        {
            let drift = sitl_gyro_drift(now_us, self.drift_speed_dps, self.drift_time_min);
            let sample = sitl_gyro_sample(state, &self.cal, drift);
            self.imu
                .notify_gyro_raw_sample(sample, now_us, self.gyro_rate_hz, now_us);
            self.advance_gyro_schedule(now_us);
            gyro_count = 1;
        }

        (gyro_count, accel_count)
    }

    fn advance_accel_schedule(&mut self, now_us: u64) {
        let period_us = 1_000_000_u64 / u64::from(self.accel_rate_hz);
        if self.next_accel_sample_us == 0 {
            self.next_accel_sample_us = now_us + period_us;
        } else {
            while now_us >= self.next_accel_sample_us {
                self.next_accel_sample_us += period_us;
            }
        }
    }

    fn advance_gyro_schedule(&mut self, now_us: u64) {
        let period_us = 1_000_000_u64 / u64::from(self.gyro_rate_hz);
        if self.next_gyro_sample_us == 0 {
            self.next_gyro_sample_us = now_us + period_us;
        } else {
            while now_us >= self.next_gyro_sample_us {
                self.next_gyro_sample_us += period_us;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::LoopTiming;

    #[test]
    fn sitl_gyro_drift_is_zero_without_parameters() {
        assert_eq!(sitl_gyro_drift(1_000_000, 0.0, 5.0), 0.0);
        assert_eq!(sitl_gyro_drift(1_000_000, 1.0, 0.0), 0.0);
    }

    #[test]
    fn a_level_hover_produces_one_g_down() {
        let state = SitlBodyState {
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };
        let sample = sitl_accel_sample(&state, &SitlImuCalibration::default());
        assert!((sample.z + 9.80665).abs() < 1e-3, "got {}", sample.z);
    }

    #[test]
    fn body_rates_are_converted_to_radians() {
        let state = SitlBodyState {
            roll_rate_dps: 57.295_78,
            ..SitlBodyState::default()
        };
        let sample = sitl_gyro_sample(&state, &SitlImuCalibration::default(), 0.0);
        assert!((sample.x - 1.0).abs() < 1e-4, "got {}", sample.x);
    }

    #[test]
    fn the_timer_delivers_samples_at_the_configured_rate() {
        let mut backend = SitlImuBackend::new(8000, 1000);
        let state = SitlBodyState {
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };

        let (g0, a0) = backend.timer_update(0, &state);
        assert_eq!(g0, 1);
        assert_eq!(a0, 1);

        // Before the next gyro tick nothing new arrives.
        let (g1, a1) = backend.timer_update(100, &state);
        assert_eq!(g1, 0);
        assert_eq!(a1, 0);

        // One millisecond later both the 8 kHz gyro and 1 kHz accel are due.
        let (g2, a2) = backend.timer_update(1000, &state);
        assert_eq!(g2, 1);
        assert_eq!(a2, 1);

        // After enough gyro ticks, accumulation reaches the frontend.
        let mut t = 1000_u64;
        for _ in 0..8000 {
            t += 125;
            backend.timer_update(t, &state);
        }
        backend.imu.update_gyro();
        backend.imu.update_accel();
        let timing = LoopTiming::new(0.0025);
        assert!(backend.imu.get_delta_angle(&timing).is_some());
        assert!(backend.imu.get_delta_velocity(&timing).is_some());
    }

    #[test]
    fn imu_temperature_warmup_follows_exponential_curve() {
        let config = SitlImuTemperature {
            temp_start_c: 20.0,
            temp_end_c: 45.0,
            temp_tconst_s: 100.0,
            ..SitlImuTemperature::default()
        };
        let t0 = sitl_imu_temperature(&config, 0);
        let t_mid = sitl_imu_temperature(&config, 100_000);
        let t_late = sitl_imu_temperature(&config, 1_000_000);
        assert!((t0 - 20.0).abs() < 0.01, "starts at temp_start, got {t0}");
        assert!(t_mid > t0 && t_mid < 45.0, "mid warmup {t_mid}");
        assert!((t_late - 45.0).abs() < 0.1, "settles at temp_end, got {t_late}");
    }

    #[test]
    fn imu_temperature_honours_fixed_override() {
        let config = SitlImuTemperature {
            temp_fixed_c: 35.0,
            ..SitlImuTemperature::default()
        };
        assert!((sitl_imu_temperature(&config, 0) - 35.0).abs() < 1e-6);
        assert!((sitl_imu_temperature(&config, 999_999) - 35.0).abs() < 1e-6);
    }

    #[test]
    fn fail_mask_suppresses_samples_for_masked_instance() {
        let mut backend = SitlImuBackend::new(1000, 1000);
        backend.instance_index = 1;
        backend.accel_fail_mask = 1 << 1;
        backend.gyro_fail_mask = 1 << 1;
        let state = SitlBodyState {
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };
        let (g, a) = backend.timer_update(0, &state);
        assert_eq!(g, 0);
        assert_eq!(a, 0);

        backend.accel_fail_mask = 0;
        backend.gyro_fail_mask = 0;
        let (g2, a2) = backend.timer_update(0, &state);
        assert_eq!(g2, 1);
        assert_eq!(a2, 1);
    }

    #[test]
    fn the_backend_tracks_warmup_temperature() {
        let mut backend = SitlImuBackend::new(1000, 1000);
        backend.temperature = SitlImuTemperature {
            temp_start_c: 20.0,
            temp_end_c: 45.0,
            temp_tconst_s: 100.0,
            ..SitlImuTemperature::default()
        };
        let state = SitlBodyState::default();
        backend.timer_update(0, &state);
        assert!((backend.last_temperature_c - 20.0).abs() < 0.01);
        backend.timer_update(600_000_000, &state);
        assert!(backend.last_temperature_c > 40.0);
    }

    #[test]
    fn calculate_noise_scales_with_rand_unit() {
        assert!((sitl_calculate_noise(1.0, 0.05, 0.0) - 1.0).abs() < 1e-6);
        assert!((sitl_calculate_noise(1.0, 0.05, 1.0) - 1.05).abs() < 1e-6);
    }

    #[test]
    fn white_noise_scales_each_axis() {
        let off = sitl_white_noise_offset(Vector3f::new(1.0, -1.0, 0.5), 0.01);
        assert!((off.x - 0.01).abs() < 1e-6);
        assert!((off.y + 0.01).abs() < 1e-6);
    }

    #[test]
    fn accel_noise_switches_with_motors() {
        assert!((sitl_accel_noise_amplitude(0.01, 0.5, false) - 0.01).abs() < 1e-6);
        assert!((sitl_accel_noise_amplitude(0.01, 0.5, true) - 0.5).abs() < 1e-6);
    }

    #[test]
    fn gyro_noise_scales_with_throttle_when_motors_on() {
        let off = sitl_gyro_noise_amplitude(SITL_DEFAULT_GYRO_NOISE_RAD, 20.0, 0.5, true);
        let expected = radians(20.0) * 0.5;
        assert!((off - expected).abs() < 1e-6);
    }

    #[test]
    fn gyro_background_noise_only_without_vibration() {
        assert!(sitl_gyro_needs_background_noise(true, true));
        assert!(!sitl_gyro_needs_background_noise(false, true));
    }

    #[test]
    fn vibe_freq_is_zero_without_motors() {
        let cfg = SitlVibeConfig {
            vibe_freq_hz: Vector3f::new(10.0, 0.0, 0.0),
            accel_noise: 0.5,
            noise_variation: 0.05,
            motors_on: false,
        };
        assert!(sitl_vibe_freq_offset(&cfg, 0.1, 0.0).is_zero());
    }

    #[test]
    fn motor_vibe_harmonic_uses_phase_times_bit() {
        let off = sitl_motor_vibe_harmonics_offset(
            core::f32::consts::FRAC_PI_2,
            0b1,
            1.0,
            1.0,
            0.0,
            0.0,
        );
        assert!((off.x - 1.0).abs() < 1e-5);
    }

    #[test]
    fn motor_vibe_offset_advances_phase_with_rpm() {
        let cfg = SitlMotorVibeConfig {
            vibe_motor: 1.0,
            vibe_motor_scale: 1.0,
            vibe_motor_harmonics: 0b1,
            accel_noise: 0.5,
            noise_variation: 0.0,
            freq_variation: 0.0,
            motors_on: true,
        };
        let mut phases = [0.0_f32; 4];
        let rpm = [6000.0, 0.0, 0.0, 0.0];
        let _ = sitl_motor_vibe_offset(&cfg, 0b1, &rpm, &mut phases, 0.001, 0.0);
        assert!(phases[0] > 0.0);
    }

    #[test]
    fn motor_vibe_disabled_when_motors_off() {
        let cfg = SitlMotorVibeConfig {
            vibe_motor: 1.0,
            vibe_motor_scale: 1.0,
            vibe_motor_harmonics: 0b1,
            accel_noise: 0.5,
            noise_variation: 0.0,
            freq_variation: 0.0,
            motors_on: false,
        };
        let mut phases = [0.0_f32; 1];
        assert!(sitl_motor_vibe_offset(&cfg, 0b1, &[1000.0], &mut phases, 0.001, 0.0).is_zero());
    }
}
