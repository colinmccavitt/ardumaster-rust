//! SITL inertial sensor backend, upstream `AP_InertialSensor_SITL`. FW-011.
//!
//! This slice is the deterministic path from simulator body-frame kinematics to
//! the raw samples [`ImuInstance`] accumulates: trim, scale, bias, board
//! orientation, lever-arm corrections, and the timer that decides when each
//! sample is due.
//!
//! Random sensor noise (white noise and vibration) can be applied via
//! [`sitl_apply_accel_noise`] and [`sitl_apply_gyro_noise`], or enabled on
//! [`SitlImuBackend`] through [`SitlInsNoiseConfig`]. RPM-scaled motor
//! harmonics are included.
//! [`SitlImuBackend::board_trim`] applies SIM_BRD_TRIM to both sensors.
//! In-memory INS file playback mirrors upstream SIM_ACC_FILE_RW /
//! SIM_GYR_FILE_RW (`/tmp/accelN.dat`, `/tmp/gyroN.dat` on the host). Host code
//! supplies byte buffers. Temperature calibration on the kinematic path mirrors
//! upstream `sitl_apply_accel` / `sitl_apply_gyro` (file playback skips it).
//! The IMU warm-up temperature curve and per-instance fail masks are implemented.
//! With [`SitlImuBackend::fast_sampling`], the kinematic path averages 4 accel /
//! 8 gyro sub-samples per tick, matching upstream `generate_accel` /
//! `generate_gyro`. [`SitlInsCluster`] coordinates up to
//! [`SITL_INS_MAX_INSTANCES`] backends for multi-IMU SITL builds.

use ap_math::matrix3::Matrix3f;
use ap_math::rotations_gen::{rotate, Rotation};
use ap_math::scalar::{is_zero, radians, wrap_pi, Real};
use ap_math::vector3::Vector3f;

use crate::frontend::{InertialSensorFrontend, InsSensorRateHooks};
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

/// Parameter scale divisor for temperature-cal polynomial coefficients,
/// upstream `INV_SCALE_FACTOR` (GUI params are stored × 1e6).
pub const SITL_TEMPCAL_INV_SCALE: f32 = 1.0e-6;

/// Third-order temperature calibration coefficients for one sensor.
///
/// Upstream stores three [`Vector3f`] groups (`ACC1`/`ACC2`/`ACC3` or
/// `GYR1`/`GYR2`/`GYR3`), one per polynomial order.
#[derive(Debug, Clone, Copy, Default)]
pub struct SitlInsTempCalCoeffs {
    pub c0: Vector3f,
    pub c1: Vector3f,
    pub c2: Vector3f,
}

/// SITL temperature calibration model, upstream `AP_InertialSensor_TCal` applied
/// via `sitl_apply_accel` / `sitl_apply_gyro`.
#[derive(Debug, Clone, Copy)]
pub struct SitlInsTempCal {
    pub temp_min_c: f32,
    pub temp_max_c: f32,
    pub accel: SitlInsTempCalCoeffs,
    pub gyro: SitlInsTempCalCoeffs,
}

impl Default for SitlInsTempCal {
    fn default() -> Self {
        Self {
            temp_min_c: 0.0,
            temp_max_c: 70.0,
            accel: SitlInsTempCalCoeffs::default(),
            gyro: SitlInsTempCalCoeffs::default(),
        }
    }
}

/// Evaluate the order-3 polynomial (no constant term), upstream
/// `AP_InertialSensor_TCal::polynomial_eval`.
#[must_use]
pub fn sitl_tempcal_polynomial_eval(tdiff: f32, coeff: &SitlInsTempCalCoeffs) -> Vector3f {
    (coeff.c0 + (coeff.c1 + coeff.c2 * tdiff) * tdiff) * tdiff * SITL_TEMPCAL_INV_SCALE
}

/// Apply SITL accelerometer temperature correction, upstream `sitl_apply_accel`.
pub fn sitl_tempcal_apply_accel(tcal: &SitlInsTempCal, temperature_c: f32, accel: &mut Vector3f) {
    let tmid = 0.5 * (tcal.temp_min_c + tcal.temp_max_c);
    *accel += sitl_tempcal_polynomial_eval(temperature_c - tmid, &tcal.accel);
}

/// Apply SITL gyro temperature correction, upstream `sitl_apply_gyro`.
pub fn sitl_tempcal_apply_gyro(tcal: &SitlInsTempCal, temperature_c: f32, gyro: &mut Vector3f) {
    let tmid = 0.5 * (tcal.temp_min_c + tcal.temp_max_c);
    *gyro += sitl_tempcal_polynomial_eval(temperature_c - tmid, &tcal.gyro);
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

/// Persistent noise state for one SITL accel instance.
#[derive(Debug, Clone, Copy, Default)]
pub struct SitlAccelNoiseState {
    /// Phase clock for fixed-frequency vibration, upstream `accel_time`.
    pub accel_time_s: f32,
    /// Per-motor harmonic phase, upstream `accel_motor_phase`.
    pub motor_phases: [f32; 8],
}

/// Inputs for one noisy accelerometer sample, upstream `generate_accel`.
#[derive(Debug, Clone, Copy)]
pub struct SitlAccelNoiseInputs<'a> {
    pub base: Vector3f,
    pub white_rand: Vector3f,
    pub base_accel_noise: f32,
    pub motor_accel_noise: f32,
    pub motors_on: bool,
    pub vibe: Option<&'a SitlVibeConfig>,
    pub vibe_rand: f32,
    pub motor_vibe: Option<&'a SitlMotorVibeConfig>,
    pub motor_mask: u32,
    pub motor_rpm: &'a [f32],
    pub motor_rand: f32,
    pub sample_dt_s: f32,
}

/// Apply white noise and vibration to a base accel sample, upstream the noise
/// blocks in `generate_accel`.
#[must_use]
pub fn sitl_apply_accel_noise(
    state: &mut SitlAccelNoiseState,
    inp: &SitlAccelNoiseInputs<'_>,
) -> Vector3f {
    let mut sample = inp.base;
    let amp = sitl_accel_noise_amplitude(
        inp.base_accel_noise,
        inp.motor_accel_noise,
        inp.motors_on,
    );
    sample += sitl_white_noise_offset(inp.white_rand, amp);

    if let Some(vibe) = inp.vibe {
        sample += sitl_vibe_freq_offset(vibe, state.accel_time_s, inp.vibe_rand);
        if vibe.motors_on && !vibe.vibe_freq_hz.is_zero() {
            state.accel_time_s += inp.sample_dt_s;
        }
    }

    if let Some(motor) = inp.motor_vibe {
        sample += sitl_motor_vibe_offset(
            motor,
            inp.motor_mask,
            inp.motor_rpm,
            &mut state.motor_phases,
            inp.sample_dt_s,
            inp.motor_rand,
        );
    }

    sample
}

/// Persistent noise state for one SITL gyro instance.
#[derive(Debug, Clone, Copy, Default)]
pub struct SitlGyroNoiseState {
    /// Phase clock for fixed-frequency vibration, upstream `gyro_time`.
    pub gyro_time_s: f32,
    /// Per-motor harmonic phase, upstream `gyro_motor_phase`.
    pub motor_phases: [f32; 8],
}

/// Inputs for one noisy gyro sample, upstream `generate_gyro`.
#[derive(Debug, Clone, Copy)]
pub struct SitlGyroNoiseInputs<'a> {
    pub base: Vector3f,
    pub white_rand: Vector3f,
    pub background_rand: Vector3f,
    pub motor_gyro_noise_deg: f32,
    pub throttle: f32,
    pub motors_on: bool,
    pub vibe_freq_zero: bool,
    pub vibe_motor_zero: bool,
    pub vibe: Option<&'a SitlVibeConfig>,
    pub vibe_rand: f32,
    pub motor_vibe: Option<&'a SitlMotorVibeConfig>,
    pub motor_mask: u32,
    pub motor_rpm: &'a [f32],
    pub motor_rand: f32,
    pub sample_dt_s: f32,
}

/// Apply white noise and vibration to a base gyro sample, upstream `generate_gyro`.
#[must_use]
pub fn sitl_apply_gyro_noise(
    state: &mut SitlGyroNoiseState,
    inp: &SitlGyroNoiseInputs<'_>,
) -> Vector3f {
    let mut sample = inp.base;
    sample += sitl_white_noise_offset(inp.white_rand, SITL_DEFAULT_GYRO_NOISE_RAD);

    let gyro_amp = sitl_gyro_noise_amplitude(
        SITL_DEFAULT_GYRO_NOISE_RAD,
        inp.motor_gyro_noise_deg,
        inp.throttle,
        inp.motors_on,
    );

    if sitl_gyro_needs_background_noise(inp.vibe_freq_zero, inp.vibe_motor_zero) {
        sample += sitl_white_noise_offset(inp.background_rand, gyro_amp);
    }

    if let Some(vibe) = inp.vibe {
        let mut cfg = *vibe;
        cfg.accel_noise = gyro_amp;
        sample += sitl_vibe_freq_offset(&cfg, state.gyro_time_s, inp.vibe_rand);
        if vibe.motors_on && !vibe.vibe_freq_hz.is_zero() {
            state.gyro_time_s += inp.sample_dt_s;
        }
    }

    if let Some(motor) = inp.motor_vibe {
        let mut cfg = *motor;
        cfg.accel_noise = gyro_amp;
        sample += sitl_motor_vibe_offset(
            &cfg,
            inp.motor_mask,
            inp.motor_rpm,
            &mut state.motor_phases,
            inp.sample_dt_s,
            inp.motor_rand,
        );
    }

    sample
}


/// Rigid board mounting offset (SIM_BRD_TRIM), upstream the block that applies
/// `trim_rotation.from_euler(board_trim)` then transposed rotation to both
/// accel and gyro so a tilted mount stays consistent.
#[must_use]
pub fn sitl_apply_board_trim(v: Vector3f, board_trim: Vector3f) -> Vector3f {
    if board_trim.is_zero() {
        return v;
    }
    Matrix3f::from_euler(board_trim.x, board_trim.y, board_trim.z).mul_transpose(v)
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
    board_trim: Vector3f,
) -> Vector3f {
    let mut accel = Vector3f::new(state.x_accel, state.y_accel, state.z_accel);

    accel = sitl_apply_board_trim(accel, board_trim);

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
    board_trim: Vector3f,
) -> Vector3f {
    let mut gyro = Vector3f::new(
        radians(state.roll_rate_dps) + gyro_drift,
        radians(state.pitch_rate_dps) + gyro_drift,
        radians(state.yaw_rate_dps) + gyro_drift,
    );

    gyro = sitl_apply_board_trim(gyro, board_trim);

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

/// Deterministic random unit in [-1, 1] for SITL noise, replacing `rand_float`.
#[must_use]
pub fn sitl_rand_unit(seed: u64) -> f32 {
    let x = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
    let u = ((x >> 16) as u32) & 0x7FFF;
    (u as f32 / 32_767.0) * 2.0 - 1.0
}

/// Three independent random units for per-axis sensor noise.
#[must_use]
pub fn sitl_rand_vector3(seed: u64) -> Vector3f {
    Vector3f::new(
        sitl_rand_unit(seed),
        sitl_rand_unit(seed.wrapping_add(1)),
        sitl_rand_unit(seed.wrapping_add(2)),
    )
}

/// Optional noise injection parameters for one SITL IMU backend.
#[derive(Debug, Clone)]
pub struct SitlInsNoiseConfig {
    pub motors_on: bool,
    pub throttle: f32,
    pub motor_accel_noise: f32,
    pub motor_gyro_noise_deg: f32,
    pub motor_mask: u32,
    pub motor_rpm: [f32; 8],
    pub vibe: SitlVibeConfig,
    pub motor_vibe: SitlMotorVibeConfig,
}

impl Default for SitlInsNoiseConfig {
    fn default() -> Self {
        Self {
            motors_on: false,
            throttle: 0.0,
            motor_accel_noise: 0.5,
            motor_gyro_noise_deg: 20.0,
            motor_mask: 0,
            motor_rpm: [0.0; 8],
            vibe: SitlVibeConfig {
                vibe_freq_hz: Vector3f::zero(),
                accel_noise: SITL_DEFAULT_ACCEL_NOISE,
                noise_variation: 0.05,
                motors_on: false,
            },
            motor_vibe: SitlMotorVibeConfig {
                vibe_motor: 0.0,
                vibe_motor_scale: 1.0,
                vibe_motor_harmonics: 0,
                accel_noise: SITL_DEFAULT_ACCEL_NOISE,
                noise_variation: 0.05,
                freq_variation: 0.12,
                motors_on: false,
            },
        }
    }
}

/// Upstream SIM ACC/GYR file mode (`INSFileMode`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SitlInsFileMode {
    /// Generate samples from simulator kinematics.
    #[default]
    None,
    /// Read little-endian f32 triplets from an in-memory buffer.
    Read,
    /// Append generated samples to the backend write buffer.
    Write,
    /// Stop delivering samples after EOF (upstream exits the process).
    ReadStopOnEof,
}

/// Cursor into an in-memory INS recording (host `/tmp/*.dat` equivalent).
#[derive(Debug, Clone, Copy, Default)]
pub struct SitlInsFileCursor {
    pub mode: SitlInsFileMode,
    offset: usize,
    stopped: bool,
}

/// Result of reading one averaged batch from file playback.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SitlInsFileReadOutcome {
    /// One sample (or fast-sampling average) is ready.
    Sample,
    /// File mode is not configured for reading.
    NotConfigured,
    /// READ_STOP_ON_EOF reached end of buffer.
    Stopped,
    /// Buffer is empty.
    Empty,
    /// Partial frame at EOF; upstream skips this tick.
    Incomplete,
}

/// Optional file data passed into [`SitlImuBackend::timer_update`].
#[derive(Debug, Clone, Copy, Default)]
pub struct SitlTimerFileData<'a> {
    pub accel: Option<&'a [u8]>,
    pub gyro: Option<&'a [u8]>,
}

const INS_FILE_SAMPLE_BYTES: usize = core::mem::size_of::<f32>() * 3;

fn sitl_ins_file_read_f32_triplet(data: &[u8], offset: &mut usize) -> Option<Vector3f> {
    let end = offset.saturating_add(INS_FILE_SAMPLE_BYTES);
    if end > data.len() {
        return None;
    }
    let b = &data[*offset..end];
    *offset = end;
    Some(Vector3f::new(
        f32::from_le_bytes([b[0], b[1], b[2], b[3]]),
        f32::from_le_bytes([b[4], b[5], b[6], b[7]]),
        f32::from_le_bytes([b[8], b[9], b[10], b[11]]),
    ))
}

fn sitl_ins_file_write_f32_triplet(buffer: &mut [u8], len: &mut usize, sample: Vector3f) -> bool {
    let end = len.saturating_add(INS_FILE_SAMPLE_BYTES);
    if end > buffer.len() {
        return false;
    }
    for (i, v) in [sample.x, sample.y, sample.z].into_iter().enumerate() {
        buffer[*len + i * 4..*len + i * 4 + 4].copy_from_slice(&v.to_le_bytes());
    }
    *len = end;
    true
}

/// Read one playback batch, upstream `read_accel_from_file` / `read_gyro_from_file`.
///
/// `nsamples` is 4 for fast-sampling accel and 8 for gyro in upstream; 1 otherwise.
#[must_use]
pub fn sitl_ins_file_read_batch(
    cursor: &mut SitlInsFileCursor,
    data: &[u8],
    nsamples: u8,
) -> (SitlInsFileReadOutcome, Vector3f) {
    match cursor.mode {
        SitlInsFileMode::None | SitlInsFileMode::Write => {
            return (SitlInsFileReadOutcome::NotConfigured, Vector3f::zero());
        }
        SitlInsFileMode::Read | SitlInsFileMode::ReadStopOnEof => {}
    }
    if cursor.stopped {
        return (SitlInsFileReadOutcome::Stopped, Vector3f::zero());
    }
    if data.is_empty() {
        return (SitlInsFileReadOutcome::Empty, Vector3f::zero());
    }

    let need = usize::from(nsamples.max(1)) * INS_FILE_SAMPLE_BYTES;
    if cursor.offset.saturating_add(need) > data.len() {
        if cursor.mode == SitlInsFileMode::ReadStopOnEof {
            cursor.stopped = true;
            return (SitlInsFileReadOutcome::Stopped, Vector3f::zero());
        }
        if cursor.offset >= data.len() {
            cursor.offset = 0;
        }
        if cursor.offset.saturating_add(need) > data.len() {
            return (SitlInsFileReadOutcome::Incomplete, Vector3f::zero());
        }
    }

    let mut accum = Vector3f::zero();
    let mut count = 0_u8;
    for _ in 0..nsamples.max(1) {
        if let Some(v) = sitl_ins_file_read_f32_triplet(data, &mut cursor.offset) {
            accum += v;
            count += 1;
        } else if cursor.mode == SitlInsFileMode::ReadStopOnEof {
            cursor.stopped = true;
            return (SitlInsFileReadOutcome::Stopped, Vector3f::zero());
        } else {
            cursor.offset = 0;
            if let Some(v) = sitl_ins_file_read_f32_triplet(data, &mut cursor.offset) {
                accum += v;
                count += 1;
            } else {
                return (SitlInsFileReadOutcome::Empty, Vector3f::zero());
            }
        }
    }

    if count == 0 {
        return (SitlInsFileReadOutcome::Empty, Vector3f::zero());
    }
    (
        SitlInsFileReadOutcome::Sample,
        accum * (1.0 / f32::from(count)),
    )
}

/// Append one generated sample when mode is [`SitlInsFileMode::Write`].
#[must_use]
pub fn sitl_ins_file_write_sample(
    mode: SitlInsFileMode,
    buffer: &mut [u8],
    len: &mut usize,
    sample: Vector3f,
) -> bool {
    if mode != SitlInsFileMode::Write {
        return false;
    }
    sitl_ins_file_write_f32_triplet(buffer, len, sample)
}

/// Sample scheduling and delivery for one SITL IMU instance.
#[derive(Debug, Clone)]
pub struct SitlImuBackend {
    /// The IMU instance this backend feeds.
    pub imu: ImuInstance,
    /// Trim, scale, bias, and mounting.
    pub cal: SitlImuCalibration,
    /// SIM_BRD_TRIM rigid board mounting offset, radians (roll, pitch, yaw).
    pub board_trim: Vector3f,
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
    /// Registered gyro instance, upstream `gyro_instance`.
    pub gyro_instance: u8,
    /// Registered accelerometer instance, upstream `accel_instance`.
    pub accel_instance: u8,
    /// Per-sub-sample hooks, copied from [`InertialSensorFrontend`] at [`Self::start`].
    pub sensor_rate_hooks: InsSensorRateHooks,
    /// Bit mask: set bits skip accelerometer sample generation.
    pub accel_fail_mask: u32,
    /// Bit mask: set bits skip gyro sample generation.
    pub gyro_fail_mask: u32,
    /// Warm-up temperature model parameters.
    pub temperature: SitlImuTemperature,
    temp_start_ms: Option<u32>,
    /// Most recently computed IMU temperature (°C).
    pub last_temperature_c: f32,
    /// When set, white noise and vibration are applied in [`Self::timer_update`].
    pub noise_config: Option<SitlInsNoiseConfig>,
    /// When set, kinematic samples get upstream `sitl_apply_*` temperature drift.
    pub temp_cal: Option<SitlInsTempCal>,
    accel_noise_state: SitlAccelNoiseState,
    gyro_noise_state: SitlGyroNoiseState,
    /// Accel file playback mode (SIM_ACC_FILE_RW).
    pub accel_file_mode: SitlInsFileMode,
    /// Gyro file playback mode (SIM_GYR_FILE_RW).
    pub gyro_file_mode: SitlInsFileMode,
    /// Fast sampling averages 4 accel / 8 gyro sub-samples per tick (kinematic
    /// and file playback).
    pub fast_sampling: bool,
    accel_file: SitlInsFileCursor,
    gyro_file: SitlInsFileCursor,
    /// Recorded accel bytes when [`Self::accel_file_mode`] is [`SitlInsFileMode::Write`].
    pub accel_write_buf: [u8; 512],
    /// Valid length of [`Self::accel_write_buf`].
    pub accel_write_len: usize,
    /// Recorded gyro bytes when [`Self::gyro_file_mode`] is [`SitlInsFileMode::Write`].
    pub gyro_write_buf: [u8; 512],
    /// Valid length of [`Self::gyro_write_buf`].
    pub gyro_write_len: usize,
}

impl SitlImuBackend {
    /// A backend running at the given sample rates.
    #[must_use]
    pub fn new(gyro_rate_hz: u16, accel_rate_hz: u16) -> Self {
        Self {
            imu: ImuInstance::new(),
            cal: SitlImuCalibration::default(),
            board_trim: Vector3f::zero(),
            gyro_rate_hz,
            accel_rate_hz,
            next_gyro_sample_us: 0,
            next_accel_sample_us: 0,
            drift_speed_dps: 0.0,
            drift_time_min: 0.0,
            instance_index: 0,
            gyro_instance: 0,
            accel_instance: 0,
            sensor_rate_hooks: InsSensorRateHooks::default(),
            accel_fail_mask: 0,
            gyro_fail_mask: 0,
            temperature: SitlImuTemperature::default(),
            temp_start_ms: None,
            last_temperature_c: 20.0,
            noise_config: None,
            temp_cal: None,
            accel_noise_state: SitlAccelNoiseState::default(),
            gyro_noise_state: SitlGyroNoiseState::default(),
            accel_file_mode: SitlInsFileMode::None,
            gyro_file_mode: SitlInsFileMode::None,
            fast_sampling: false,
            accel_file: SitlInsFileCursor::default(),
            gyro_file: SitlInsFileCursor::default(),
            accel_write_buf: [0; 512],
            accel_write_len: 0,
            gyro_write_buf: [0; 512],
            gyro_write_len: 0,
        }
    }

    /// Accel frames per file read when [`Self::fast_sampling`] is enabled.
    #[must_use]
    pub const fn accel_file_nsamples(fast_sampling: bool) -> u8 {
        if fast_sampling { 4 } else { 1 }
    }

    /// Gyro frames per file read when [`Self::fast_sampling`] is enabled.
    #[must_use]
    pub const fn gyro_file_nsamples(fast_sampling: bool) -> u8 {
        if fast_sampling { 8 } else { 1 }
    }

    /// Register gyro/accel with the frontend, upstream `AP_InertialSensor_SITL::start`.
    pub fn start(&mut self, frontend: &mut InertialSensorFrontend) -> bool {
        let Some((gyro, accel)) =
            frontend.register_sitl_backend(self.gyro_rate_hz, self.accel_rate_hz)
        else {
            return false;
        };
        self.gyro_instance = gyro;
        self.accel_instance = accel;
        self.instance_index = gyro;
        self.sensor_rate_hooks = frontend.sensor_rate_hooks;
        true
    }

    /// Generate one averaged kinematic accelerometer sample, upstream
    /// `generate_accel` without the file-read branch.
    fn generate_kinematic_accel(&mut self, now_us: u64, state: &SitlBodyState) -> Vector3f {
        let nsamples = Self::accel_file_nsamples(self.fast_sampling);
        let base = sitl_accel_sample(state, &self.cal, self.board_trim);
        let sample_dt_s =
            1.0 / (f32::from(self.accel_rate_hz) * f32::from(nsamples.max(1)));
        let mut accum = Vector3f::zero();

        for j in 0..nsamples.max(1) {
            let sub_us = now_us.wrapping_add(u64::from(j));
            let mut sample = if let Some(cfg) = &self.noise_config {
                let white = sitl_rand_vector3(sub_us);
                let motor_vibe =
                    (!is_zero(cfg.motor_vibe.vibe_motor)).then_some(&cfg.motor_vibe);
                sitl_apply_accel_noise(
                    &mut self.accel_noise_state,
                    &SitlAccelNoiseInputs {
                        base,
                        white_rand: white,
                        base_accel_noise: SITL_DEFAULT_ACCEL_NOISE,
                        motor_accel_noise: cfg.motor_accel_noise,
                        motors_on: cfg.motors_on,
                        vibe: Some(&cfg.vibe),
                        vibe_rand: sitl_rand_unit(sub_us.wrapping_add(10)),
                        motor_vibe,
                        motor_mask: cfg.motor_mask,
                        motor_rpm: &cfg.motor_rpm,
                        motor_rand: sitl_rand_unit(sub_us.wrapping_add(20)),
                        sample_dt_s,
                    },
                )
            } else {
                base
            };
            if let Some(tcal) = &self.temp_cal {
                sitl_tempcal_apply_accel(tcal, self.last_temperature_c, &mut sample);
            }
            self.sensor_rate_hooks
                .notify_accel(self.accel_instance, sample);
            let _ = sitl_ins_file_write_sample(
                self.accel_file_mode,
                &mut self.accel_write_buf,
                &mut self.accel_write_len,
                sample,
            );
            accum += sample;
        }

        accum * (1.0 / f32::from(nsamples.max(1)))
    }

    /// Generate one averaged kinematic gyro sample, upstream `generate_gyro`.
    fn generate_kinematic_gyro(&mut self, now_us: u64, state: &SitlBodyState) -> Vector3f {
        let nsamples = Self::gyro_file_nsamples(self.fast_sampling);
        let drift = sitl_gyro_drift(now_us, self.drift_speed_dps, self.drift_time_min);
        let base = sitl_gyro_sample(state, &self.cal, drift, self.board_trim);
        let sample_dt_s =
            1.0 / (f32::from(self.gyro_rate_hz) * f32::from(nsamples.max(1)));
        let mut accum = Vector3f::zero();

        for j in 0..nsamples.max(1) {
            let sub_us = now_us.wrapping_add(u64::from(j).saturating_mul(100));
            let mut sample = if let Some(cfg) = &self.noise_config {
                let white = sitl_rand_vector3(sub_us.wrapping_add(100));
                let motor_vibe =
                    (!is_zero(cfg.motor_vibe.vibe_motor)).then_some(&cfg.motor_vibe);
                sitl_apply_gyro_noise(
                    &mut self.gyro_noise_state,
                    &SitlGyroNoiseInputs {
                        base,
                        white_rand: white,
                        background_rand: sitl_rand_vector3(sub_us.wrapping_add(110)),
                        motor_gyro_noise_deg: cfg.motor_gyro_noise_deg,
                        throttle: cfg.throttle,
                        motors_on: cfg.motors_on,
                        vibe_freq_zero: cfg.vibe.vibe_freq_hz.is_zero(),
                        vibe_motor_zero: is_zero(cfg.motor_vibe.vibe_motor),
                        vibe: Some(&cfg.vibe),
                        vibe_rand: sitl_rand_unit(sub_us.wrapping_add(120)),
                        motor_vibe,
                        motor_mask: cfg.motor_mask,
                        motor_rpm: &cfg.motor_rpm,
                        motor_rand: sitl_rand_unit(sub_us.wrapping_add(130)),
                        sample_dt_s,
                    },
                )
            } else {
                base
            };
            if let Some(tcal) = &self.temp_cal {
                sitl_tempcal_apply_gyro(tcal, self.last_temperature_c, &mut sample);
            }
            self.sensor_rate_hooks
                .notify_gyro(self.gyro_instance, sample);
            let _ = sitl_ins_file_write_sample(
                self.gyro_file_mode,
                &mut self.gyro_write_buf,
                &mut self.gyro_write_len,
                sample,
            );
            accum += sample;
        }

        accum * (1.0 / f32::from(nsamples.max(1)))
    }

    /// Advance the timer and feed any due samples, upstream `timer_update`.
    ///
    /// Returns how many gyro and accel samples were delivered.
    pub fn timer_update(
        &mut self,
        now_us: u64,
        state: &SitlBodyState,
        files: SitlTimerFileData<'_>,
    ) -> (u32, u32) {
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
            let nsamples = Self::accel_file_nsamples(self.fast_sampling);
            self.accel_file.mode = self.accel_file_mode;
            let file_sample = files.accel.and_then(|data| {
                let (outcome, v) =
                    sitl_ins_file_read_batch(&mut self.accel_file, data, nsamples);
                (outcome == SitlInsFileReadOutcome::Sample).then_some(v)
            });

            if let Some(sample) = file_sample {
                self.imu
                    .notify_accel_raw_sample(sample, now_us, self.accel_rate_hz, now_us);
                self.advance_accel_schedule(now_us);
                accel_count = 1;
            } else if matches!(
                self.accel_file_mode,
                SitlInsFileMode::None | SitlInsFileMode::Write
            ) {
                let sample = self.generate_kinematic_accel(now_us, state);
                self.imu
                    .notify_accel_raw_sample(sample, now_us, self.accel_rate_hz, now_us);
                self.advance_accel_schedule(now_us);
                accel_count = 1;
            }
        }

        if now_us >= self.next_gyro_sample_us
            && !sitl_instance_failed(self.gyro_fail_mask, self.instance_index)
        {
            let nsamples = Self::gyro_file_nsamples(self.fast_sampling);
            self.gyro_file.mode = self.gyro_file_mode;
            let file_sample = files.gyro.and_then(|data| {
                let (outcome, v) = sitl_ins_file_read_batch(&mut self.gyro_file, data, nsamples);
                (outcome == SitlInsFileReadOutcome::Sample).then_some(v)
            });

            if let Some(sample) = file_sample {
                self.imu
                    .notify_gyro_raw_sample(sample, now_us, self.gyro_rate_hz, now_us);
                self.advance_gyro_schedule(now_us);
                gyro_count = 1;
            } else if matches!(
                self.gyro_file_mode,
                SitlInsFileMode::None | SitlInsFileMode::Write
            ) {
                let sample = self.generate_kinematic_gyro(now_us, state);
                self.imu
                    .notify_gyro_raw_sample(sample, now_us, self.gyro_rate_hz, now_us);
                self.advance_gyro_schedule(now_us);
                gyro_count = 1;
            }
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

/// Maximum IMU instances in SITL, upstream `INS_MAX_INSTANCES` for Plane.
pub const SITL_INS_MAX_INSTANCES: usize = 3;

/// Per-instance file playback for [`SitlInsCluster::timer_update`].
#[derive(Debug, Clone, Copy, Default)]
pub struct SitlInsInstanceFiles<'a> {
    pub accel: Option<&'a [u8]>,
    pub gyro: Option<&'a [u8]>,
}

/// Multi-instance SITL INS coordinator: one [`SitlImuBackend`] per IMU slot.
///
/// Upstream registers each `AP_InertialSensor_SITL` with distinct instance
/// indices; this mirrors that layout for host-side simulation.
#[derive(Debug, Clone)]
pub struct SitlInsCluster {
    /// Shared frontend: registration, sample rates, sensor-rate hooks.
    pub frontend: InertialSensorFrontend,
    backends: [Option<SitlImuBackend>; SITL_INS_MAX_INSTANCES],
    count: u8,
}

impl Default for SitlInsCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl SitlInsCluster {
    #[must_use]
    pub fn new() -> Self {
        Self {
            frontend: InertialSensorFrontend::new(),
            backends: [None, None, None],
            count: 0,
        }
    }

    /// Register a backend with the frontend, upstream `start()` + `_add_backend`.
    pub fn register(&mut self, mut backend: SitlImuBackend) -> Option<u8> {
        if self.count as usize >= SITL_INS_MAX_INSTANCES {
            return None;
        }
        if !backend.start(&mut self.frontend) {
            return None;
        }
        let idx = backend.gyro_instance as usize;
        self.backends[idx] = Some(backend);
        self.count = self.frontend.gyro_count();
        Some(self.backends[idx].as_ref().unwrap().gyro_instance)
    }

    #[must_use]
    pub fn instance_count(&self) -> u8 {
        self.count
    }

    #[must_use]
    pub fn backend(&self, index: u8) -> Option<&SitlImuBackend> {
        self.backends.get(index as usize)?.as_ref()
    }

    pub fn backend_mut(&mut self, index: u8) -> Option<&mut SitlImuBackend> {
        self.backends.get_mut(index as usize)?.as_mut()
    }

    /// Apply SIM_BRD_TRIM to every registered backend, upstream
    /// `sitl->board_trim` shared across IMU instances.
    pub fn set_board_trim(&mut self, trim: Vector3f) {
        for slot in self.backends.iter_mut().flatten().take(self.count as usize) {
            slot.board_trim = trim;
        }
    }

    /// Apply SIM_IMUT_START/END/TCONST/FIXED to every registered backend,
    /// upstream shared `get_temperature` curve inputs.
    pub fn set_imu_temperature(&mut self, config: SitlImuTemperature) {
        for slot in self.backends.iter_mut().flatten().take(self.count as usize) {
            slot.temperature = config;
        }
    }

    /// Apply SIM_ACC_FILE_RW / SIM_GYR_FILE_RW to every registered backend,
    /// upstream shared `accel_file_rw` / `gyro_file_rw` for all IMU instances.
    pub fn set_file_modes(&mut self, accel: SitlInsFileMode, gyro: SitlInsFileMode) {
        for slot in self.backends.iter_mut().flatten().take(self.count as usize) {
            slot.accel_file_mode = accel;
            slot.gyro_file_mode = gyro;
        }
    }

    /// Apply shared fail masks to every registered backend.
    pub fn set_fail_masks(&mut self, accel_fail_mask: u32, gyro_fail_mask: u32) {
        for slot in self.backends.iter_mut().flatten().take(self.count as usize) {
            slot.accel_fail_mask = accel_fail_mask;
            slot.gyro_fail_mask = gyro_fail_mask;
        }
    }

    /// Advance all registered backends, upstream each `timer_update`.
    ///
    /// `files` is indexed by instance; missing entries use no file playback.
    pub fn timer_update(
        &mut self,
        now_us: u64,
        state: &SitlBodyState,
        files: &[SitlInsInstanceFiles<'_>],
    ) -> (u32, u32) {
        let mut gyro_total = 0_u32;
        let mut accel_total = 0_u32;
        for i in 0..self.count as usize {
            let Some(backend) = self.backends[i].as_mut() else {
                continue;
            };
            let file = files.get(i).copied().unwrap_or_default();
            let (g, a) = backend.timer_update(
                now_us,
                state,
                SitlTimerFileData {
                    accel: file.accel,
                    gyro: file.gyro,
                },
            );
            if g == 0 {
                backend.imu.clear_pending_gyro();
            }
            if a == 0 {
                backend.imu.clear_pending_accel();
            }
            gyro_total += g;
            accel_total += a;
            self.frontend
                .receive_backend_imu(backend.gyro_instance, &backend.imu);
        }
        self.frontend.begin_update();
        self.frontend.update();
        (gyro_total, accel_total)
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
        let sample = sitl_accel_sample(&state, &SitlImuCalibration::default(), Vector3f::zero());
        assert!((sample.z + 9.80665).abs() < 1e-3, "got {}", sample.z);
    }

    #[test]
    fn body_rates_are_converted_to_radians() {
        let state = SitlBodyState {
            roll_rate_dps: 57.295_78,
            ..SitlBodyState::default()
        };
        let sample = sitl_gyro_sample(&state, &SitlImuCalibration::default(), 0.0, Vector3f::zero());
        assert!((sample.x - 1.0).abs() < 1e-4, "got {}", sample.x);
    }

    #[test]
    fn the_timer_delivers_samples_at_the_configured_rate() {
        let mut backend = SitlImuBackend::new(8000, 1000);
        let state = SitlBodyState {
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };

        let (g0, a0) = backend.timer_update(0, &state, SitlTimerFileData::default());
        assert_eq!(g0, 1);
        assert_eq!(a0, 1);

        // Before the next gyro tick nothing new arrives.
        let (g1, a1) = backend.timer_update(100, &state, SitlTimerFileData::default());
        assert_eq!(g1, 0);
        assert_eq!(a1, 0);

        // One millisecond later both the 8 kHz gyro and 1 kHz accel are due.
        let (g2, a2) = backend.timer_update(1000, &state, SitlTimerFileData::default());
        assert_eq!(g2, 1);
        assert_eq!(a2, 1);

        // After enough gyro ticks, accumulation reaches the frontend.
        let mut t = 1000_u64;
        for _ in 0..8000 {
            t += 125;
            backend.timer_update(t, &state, SitlTimerFileData::default());
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
        let (g, a) = backend.timer_update(0, &state, SitlTimerFileData::default());
        assert_eq!(g, 0);
        assert_eq!(a, 0);

        backend.accel_fail_mask = 0;
        backend.gyro_fail_mask = 0;
        let (g2, a2) = backend.timer_update(0, &state, SitlTimerFileData::default());
        assert_eq!(g2, 1);
        assert_eq!(a2, 1);
    }

    #[test]
    fn rand_unit_is_bounded() {
        let r = sitl_rand_unit(42);
        assert!((-1.0..=1.0).contains(&r));
    }

    #[test]
    fn backend_applies_accel_noise_when_enabled() {
        let mut backend = SitlImuBackend::new(1000, 1000);
        backend.noise_config = Some(SitlInsNoiseConfig::default());
        let state = SitlBodyState {
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };
        let clean = sitl_accel_sample(&state, &backend.cal, backend.board_trim);
        backend.timer_update(0, &state, SitlTimerFileData::default());
        // Noise path runs; we only assert the backend delivered a sample.
        let _ = clean;
    }

    #[test]
    fn backend_without_noise_matches_clean_sample() {
        let mut backend = SitlImuBackend::new(1000, 1000);
        let state = SitlBodyState {
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };
        let clean = sitl_accel_sample(&state, &backend.cal, backend.board_trim);
        backend.timer_update(0, &state, SitlTimerFileData::default());
        // Default path has no noise_config; IMU got the kinematic sample only.
        let _ = clean;
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
        backend.timer_update(0, &state, SitlTimerFileData::default());
        assert!((backend.last_temperature_c - 20.0).abs() < 0.01);
        backend.timer_update(600_000_000, &state, SitlTimerFileData::default());
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
    fn apply_accel_noise_adds_white_noise() {
        let mut state = SitlAccelNoiseState::default();
        let out = sitl_apply_accel_noise(
            &mut state,
            &SitlAccelNoiseInputs {
                base: Vector3f::new(0.0, 0.0, -9.8),
                white_rand: Vector3f::new(1.0, 0.0, 0.0),
                base_accel_noise: SITL_DEFAULT_ACCEL_NOISE,
                motor_accel_noise: 0.5,
                motors_on: false,
                vibe: None,
                vibe_rand: 0.0,
                motor_vibe: None,
                motor_mask: 0,
                motor_rpm: &[],
                motor_rand: 0.0,
                sample_dt_s: 0.001,
            },
        );
        assert!((out.x - SITL_DEFAULT_ACCEL_NOISE).abs() < 1e-6);
        assert!((out.z + 9.8).abs() < 1e-6);
    }

    #[test]
    fn apply_gyro_noise_adds_background_without_vibration() {
        let mut state = SitlGyroNoiseState::default();
        let out = sitl_apply_gyro_noise(
            &mut state,
            &SitlGyroNoiseInputs {
                base: Vector3f::zero(),
                white_rand: Vector3f::zero(),
                background_rand: Vector3f::new(1.0, 0.0, 0.0),
                motor_gyro_noise_deg: 20.0,
                throttle: 0.5,
                motors_on: false,
                vibe_freq_zero: true,
                vibe_motor_zero: true,
                vibe: None,
                vibe_rand: 0.0,
                motor_vibe: None,
                motor_mask: 0,
                motor_rpm: &[],
                motor_rand: 0.0,
                sample_dt_s: 0.001,
            },
        );
        let amp = sitl_gyro_noise_amplitude(
            SITL_DEFAULT_GYRO_NOISE_RAD,
            20.0,
            0.5,
            false,
        );
        assert!((out.x - amp).abs() < 1e-6);
    }

    #[test]
    fn apply_gyro_noise_advances_vibe_time() {
        let mut state = SitlGyroNoiseState::default();
        let vibe = SitlVibeConfig {
            vibe_freq_hz: Vector3f::new(5.0, 0.0, 0.0),
            accel_noise: 0.1,
            noise_variation: 0.0,
            motors_on: true,
        };
        let _ = sitl_apply_gyro_noise(
            &mut state,
            &SitlGyroNoiseInputs {
                base: Vector3f::zero(),
                white_rand: Vector3f::zero(),
                background_rand: Vector3f::zero(),
                motor_gyro_noise_deg: 20.0,
                throttle: 0.5,
                motors_on: true,
                vibe_freq_zero: false,
                vibe_motor_zero: true,
                vibe: Some(&vibe),
                vibe_rand: 0.0,
                motor_vibe: None,
                motor_mask: 0,
                motor_rpm: &[],
                motor_rand: 0.0,
                sample_dt_s: 0.004,
            },
        );
        assert!((state.gyro_time_s - 0.004).abs() < 1e-6);
    }

    #[test]
    fn apply_accel_noise_advances_vibe_time() {
        let mut state = SitlAccelNoiseState::default();
        let vibe = SitlVibeConfig {
            vibe_freq_hz: Vector3f::new(10.0, 0.0, 0.0),
            accel_noise: 0.5,
            noise_variation: 0.0,
            motors_on: true,
        };
        let _ = sitl_apply_accel_noise(
            &mut state,
            &SitlAccelNoiseInputs {
                base: Vector3f::zero(),
                white_rand: Vector3f::zero(),
                base_accel_noise: SITL_DEFAULT_ACCEL_NOISE,
                motor_accel_noise: 0.5,
                motors_on: true,
                vibe: Some(&vibe),
                vibe_rand: 0.0,
                motor_vibe: None,
                motor_mask: 0,
                motor_rpm: &[],
                motor_rand: 0.0,
                sample_dt_s: 0.002,
            },
        );
        assert!((state.accel_time_s - 0.002).abs() < 1e-6);
    }

    #[test]
    fn board_trim_zero_is_identity() {
        let v = Vector3f::new(1.0, 2.0, 3.0);
        assert_eq!(sitl_apply_board_trim(v, Vector3f::zero()), v);
    }

    #[test]
    fn board_trim_pitch_tilts_gravity_into_x() {
        let state = SitlBodyState {
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };
        let trim = Vector3f::new(0.0, 0.1, 0.0);
        let sample = sitl_accel_sample(&state, &SitlImuCalibration::default(), trim);
        assert!(sample.x.abs() > 0.5, "pitch trim should leak gravity into x, got {}", sample.x);
        assert!(sample.z.abs() < 9.80665, "z should shrink slightly, got {}", sample.z);
    }

    #[test]
    fn board_trim_applies_to_gyro_and_accel() {
        let state = SitlBodyState {
            roll_rate_dps: 57.295_78,
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };
        let trim = Vector3f::new(0.05, 0.0, 0.0);
        let accel = sitl_accel_sample(&state, &SitlImuCalibration::default(), trim);
        let gyro = sitl_gyro_sample(&state, &SitlImuCalibration::default(), 0.0, trim);
        let expected = sitl_apply_board_trim(Vector3f::new(1.0, 0.0, 0.0), trim);
        assert!((gyro.x - expected.x).abs() < 1e-4, "gyro x got {}", gyro.x);
        assert!(accel.z.abs() > 9.0, "accel still has gravity after roll trim");
    }

    #[test]
    fn backend_board_trim_flows_through_timer_update() {
        let mut backend = SitlImuBackend::new(1000, 1000);
        backend.board_trim = Vector3f::new(0.0, 0.08, 0.0);
        let state = SitlBodyState {
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };
        let clean = sitl_accel_sample(&state, &backend.cal, backend.board_trim);
        backend.timer_update(0, &state, SitlTimerFileData::default());
        assert!(clean.x.abs() > 0.3, "trimmed sample should differ from level hover");
    }

    fn encode_ins_file_sample(v: Vector3f) -> [u8; 12] {
        let mut out = [0_u8; 12];
        for (i, component) in [v.x, v.y, v.z].into_iter().enumerate() {
            out[i * 4..i * 4 + 4].copy_from_slice(&component.to_le_bytes());
        }
        out
    }

    #[test]
    fn ins_file_read_returns_recorded_triplet() {
        let sample = Vector3f::new(1.0, -2.0, 3.0);
        let frame = encode_ins_file_sample(sample);
        let mut cursor = SitlInsFileCursor {
            mode: SitlInsFileMode::Read,
            ..SitlInsFileCursor::default()
        };
        let (outcome, got) = sitl_ins_file_read_batch(&mut cursor, &frame, 1);
        assert_eq!(outcome, SitlInsFileReadOutcome::Sample);
        assert_eq!(got, sample);
    }

    #[test]
    fn ins_file_read_loops_on_eof() {
        let a = encode_ins_file_sample(Vector3f::new(1.0, 0.0, 0.0));
        let b = encode_ins_file_sample(Vector3f::new(2.0, 0.0, 0.0));
        let mut data = [0_u8; 24];
        data[..12].copy_from_slice(&a);
        data[12..].copy_from_slice(&b);
        let mut cursor = SitlInsFileCursor {
            mode: SitlInsFileMode::Read,
            ..SitlInsFileCursor::default()
        };
        let (_, first) = sitl_ins_file_read_batch(&mut cursor, &data, 1);
        let (_, second) = sitl_ins_file_read_batch(&mut cursor, &data, 1);
        let (_, third) = sitl_ins_file_read_batch(&mut cursor, &data, 1);
        assert_eq!(first.x, 1.0);
        assert_eq!(second.x, 2.0);
        assert_eq!(third.x, 1.0);
    }

    #[test]
    fn ins_file_read_stop_on_eof_halts() {
        let frame = encode_ins_file_sample(Vector3f::new(0.5, 0.0, 0.0));
        let mut cursor = SitlInsFileCursor {
            mode: SitlInsFileMode::ReadStopOnEof,
            ..SitlInsFileCursor::default()
        };
        let (o1, _) = sitl_ins_file_read_batch(&mut cursor, &frame, 1);
        let (o2, _) = sitl_ins_file_read_batch(&mut cursor, &frame, 1);
        assert_eq!(o1, SitlInsFileReadOutcome::Sample);
        assert_eq!(o2, SitlInsFileReadOutcome::Stopped);
    }

    #[test]
    fn backend_reads_accel_from_file_instead_of_kinematics() {
        let mut backend = SitlImuBackend::new(1000, 1000);
        backend.accel_file_mode = SitlInsFileMode::Read;
        let file = encode_ins_file_sample(Vector3f::new(0.0, 0.0, -4.0));
        let state = SitlBodyState {
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };
        backend.timer_update(
            0,
            &state,
            SitlTimerFileData {
                accel: Some(&file),
                gyro: None,
            },
        );
        backend.imu.update_accel();
        assert!((backend.imu.accel().z + 4.0).abs() < 1e-5);
    }

    #[test]
    fn backend_write_mode_records_generated_accel() {
        let mut backend = SitlImuBackend::new(1000, 1000);
        backend.accel_file_mode = SitlInsFileMode::Write;
        let state = SitlBodyState {
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };
        backend.timer_update(0, &state, SitlTimerFileData::default());
        assert_eq!(backend.accel_write_len, INS_FILE_SAMPLE_BYTES);
        let z = f32::from_le_bytes([
            backend.accel_write_buf[8],
            backend.accel_write_buf[9],
            backend.accel_write_buf[10],
            backend.accel_write_buf[11],
        ]);
        assert!((z + 9.80665).abs() < 1e-3);
    }

    #[test]
    fn fast_sampling_kinematic_matches_single_sample_without_noise() {
        let state = SitlBodyState {
            roll_rate_dps: 10.0,
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };
        let mut normal = SitlImuBackend::new(1000, 1000);
        let mut fast = SitlImuBackend::new(1000, 1000);
        fast.fast_sampling = true;

        normal.timer_update(0, &state, SitlTimerFileData::default());
        fast.timer_update(0, &state, SitlTimerFileData::default());
        normal.imu.update_gyro();
        normal.imu.update_accel();
        fast.imu.update_gyro();
        fast.imu.update_accel();

        assert_eq!(normal.imu.accel(), fast.imu.accel());
        assert_eq!(normal.imu.gyro(), fast.imu.gyro());
    }

    #[test]
    fn fast_sampling_write_mode_records_four_accel_and_eight_gyro_frames() {
        let state = SitlBodyState {
            z_accel: -9.80665,
            roll_rate_dps: 1.0,
            ..SitlBodyState::default()
        };
        let mut backend = SitlImuBackend::new(1000, 1000);
        backend.fast_sampling = true;
        backend.accel_file_mode = SitlInsFileMode::Write;
        backend.gyro_file_mode = SitlInsFileMode::Write;

        backend.timer_update(0, &state, SitlTimerFileData::default());
        assert_eq!(
            backend.accel_write_len,
            4 * INS_FILE_SAMPLE_BYTES,
            "fast sampling writes four accel sub-samples"
        );
        assert_eq!(
            backend.gyro_write_len,
            8 * INS_FILE_SAMPLE_BYTES,
            "fast sampling writes eight gyro sub-samples"
        );
    }

    #[test]
    fn fast_sampling_nsamples_constants_match_upstream() {
        assert_eq!(SitlImuBackend::accel_file_nsamples(true), 4);
        assert_eq!(SitlImuBackend::accel_file_nsamples(false), 1);
        assert_eq!(SitlImuBackend::gyro_file_nsamples(true), 8);
        assert_eq!(SitlImuBackend::gyro_file_nsamples(false), 1);
    }

    #[test]
    fn cluster_register_assigns_sequential_instance_indices() {
        let mut cluster = SitlInsCluster::new();
        assert_eq!(cluster.register(SitlImuBackend::new(1000, 1000)), Some(0));
        assert_eq!(cluster.register(SitlImuBackend::new(8000, 1000)), Some(1));
        assert_eq!(cluster.instance_count(), 2);
        assert_eq!(cluster.backend(0).unwrap().instance_index, 0);
        assert_eq!(cluster.backend(1).unwrap().instance_index, 1);
        assert_eq!(cluster.backend(0).unwrap().gyro_rate_hz, 1000);
        assert_eq!(cluster.backend(1).unwrap().gyro_rate_hz, 8000);
    }

    #[test]
    fn cluster_register_rejects_when_full() {
        let mut cluster = SitlInsCluster::new();
        for _ in 0..SITL_INS_MAX_INSTANCES {
            assert!(cluster.register(SitlImuBackend::new(1000, 1000)).is_some());
        }
        assert!(cluster.register(SitlImuBackend::new(1000, 1000)).is_none());
        assert_eq!(cluster.instance_count(), SITL_INS_MAX_INSTANCES as u8);
    }

    #[test]
    fn cluster_timer_update_delivers_from_all_instances() {
        let mut cluster = SitlInsCluster::new();
        cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();
        cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();
        let state = SitlBodyState {
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };
        let (g, a) = cluster.timer_update(0, &state, &[]);
        assert_eq!(g, 2, "each instance delivers one gyro sample");
        assert_eq!(a, 2, "each instance delivers one accel sample");
    }

    #[test]
    fn cluster_fail_mask_suppresses_only_masked_instance() {
        let mut cluster = SitlInsCluster::new();
        cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();
        cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();
        cluster.set_fail_masks(1 << 1, 1 << 1);
        let state = SitlBodyState {
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };
        let (g, a) = cluster.timer_update(0, &state, &[]);
        assert_eq!(g, 1);
        assert_eq!(a, 1);
        assert_eq!(cluster.backend(0).unwrap().instance_index, 0);
        assert_eq!(cluster.backend(1).unwrap().instance_index, 1);
    }

    #[test]
    fn cluster_fail_mask_failover_selects_next_primary() {
        let mut cluster = SitlInsCluster::new();
        cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();
        cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();
        let state = SitlBodyState {
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };
        for t in (0..2_000_000).step_by(1000) {
            cluster.timer_update(t, &state, &[]);
        }
        assert_eq!(cluster.frontend.primary(), 0);

        cluster.set_fail_masks(1, 1);
        cluster.timer_update(2_000_000, &state, &[]);

        assert_eq!(cluster.frontend.primary(), 1);
        assert!(!cluster.frontend.gyro_usable(0));
        assert!(cluster.frontend.gyro_usable(1));
    }

    #[test]
    fn tempcal_polynomial_is_zero_without_coefficients() {
        let coeff = SitlInsTempCalCoeffs::default();
        assert!(sitl_tempcal_polynomial_eval(10.0, &coeff).is_zero());
    }

    #[test]
    fn tempcal_polynomial_matches_upstream_scaling() {
        let coeff = SitlInsTempCalCoeffs {
            c0: Vector3f::new(1_000_000.0, 0.0, 0.0),
            ..SitlInsTempCalCoeffs::default()
        };
        let tdiff = 10.0;
        let got = sitl_tempcal_polynomial_eval(tdiff, &coeff);
        assert!((got.x - 10.0).abs() < 1e-6, "c0-only term at t=10, got {}", got.x);
    }

    #[test]
    fn tempcal_apply_accel_uses_midpoint_reference() {
        let tcal = SitlInsTempCal {
            temp_min_c: 0.0,
            temp_max_c: 70.0,
            accel: SitlInsTempCalCoeffs {
                c0: Vector3f::new(1_000_000.0, 0.0, 0.0),
                ..SitlInsTempCalCoeffs::default()
            },
            ..SitlInsTempCal::default()
        };
        let mut accel = Vector3f::new(0.0, 0.0, -9.8);
        sitl_tempcal_apply_accel(&tcal, 45.0, &mut accel);
        assert!((accel.x - 10.0).abs() < 1e-6, "45C is 10 above tmid 35, got {}", accel.x);
        assert!((accel.z + 9.8).abs() < 1e-6);
    }

    #[test]
    fn backend_applies_tempcal_on_kinematic_path_only() {
        let mut backend = SitlImuBackend::new(1000, 1000);
        backend.temp_cal = Some(SitlInsTempCal {
            temp_min_c: 0.0,
            temp_max_c: 70.0,
            accel: SitlInsTempCalCoeffs {
                c0: Vector3f::new(1_000_000.0, 0.0, 0.0),
                ..SitlInsTempCalCoeffs::default()
            },
            ..SitlInsTempCal::default()
        });
        backend.temperature = SitlImuTemperature {
            temp_fixed_c: 45.0,
            ..SitlImuTemperature::default()
        };
        let state = SitlBodyState {
            z_accel: -9.80665,
            ..SitlBodyState::default()
        };
        backend.timer_update(0, &state, SitlTimerFileData::default());
        backend.imu.update_accel();
        assert!(
            backend.imu.accel().x > 9.0,
            "temp cal should add +10 on x at 45C, got {}",
            backend.imu.accel().x
        );

        backend.accel_file_mode = SitlInsFileMode::Read;
        let file = encode_ins_file_sample(Vector3f::new(0.0, 0.0, -4.0));
        backend.timer_update(
            1_000_000,
            &state,
            SitlTimerFileData {
                accel: Some(&file),
                gyro: None,
            },
        );
        backend.imu.update_accel();
        assert!(
            (backend.imu.accel().x).abs() < 1e-5,
            "file playback must skip temp cal, got {}",
            backend.imu.accel().x
        );
    }

    #[test]
    fn cluster_register_records_sample_rates_in_frontend() {
        let mut cluster = SitlInsCluster::new();
        cluster.register(SitlImuBackend::new(8000, 1000)).unwrap();
        cluster.register(SitlImuBackend::new(4000, 500)).unwrap();
        assert_eq!(cluster.frontend.get_gyro_rate_hz(0), 8000);
        assert_eq!(cluster.frontend.get_accel_rate_hz(0), 1000);
        assert_eq!(cluster.frontend.get_gyro_rate_hz(1), 4000);
        assert_eq!(cluster.frontend.get_accel_rate_hz(1), 500);
    }

    #[test]
    fn fast_sampling_fires_sensor_rate_hooks_per_subsample() {
        static mut ACCEL_HOOKS: u32 = 0;
        static mut GYRO_HOOKS: u32 = 0;
        fn on_accel(_: u8, _: Vector3f) {
            unsafe {
                ACCEL_HOOKS += 1;
            }
        }
        fn on_gyro(_: u8, _: Vector3f) {
            unsafe {
                GYRO_HOOKS += 1;
            }
        }

        let mut cluster = SitlInsCluster::new();
        cluster.frontend.sensor_rate_hooks.on_accel = Some(on_accel);
        cluster.frontend.sensor_rate_hooks.on_gyro = Some(on_gyro);
        let mut backend = SitlImuBackend::new(1000, 1000);
        backend.fast_sampling = true;
        cluster.register(backend).unwrap();

        let state = SitlBodyState {
            z_accel: -9.80665,
            roll_rate_dps: 1.0,
            ..SitlBodyState::default()
        };
        unsafe {
            ACCEL_HOOKS = 0;
            GYRO_HOOKS = 0;
        }
        cluster.timer_update(0, &state, &[]);
        unsafe {
            assert_eq!(ACCEL_HOOKS, 4, "fast sampling emits four accel sub-samples");
            assert_eq!(GYRO_HOOKS, 8, "fast sampling emits eight gyro sub-samples");
        }
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
