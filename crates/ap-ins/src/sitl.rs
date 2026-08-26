//! SITL inertial sensor backend, upstream `AP_InertialSensor_SITL`. FW-011.
//!
//! This slice is the deterministic path from simulator body-frame kinematics to
//! the raw samples [`ImuInstance`] accumulates: trim, scale, bias, board
//! orientation, lever-arm corrections, and the timer that decides when each
//! sample is due.
//!
//! Random sensor noise, motor vibration, temperature calibration, and file
//! playback are not here yet — they need a noise source and more upstream
//! surface area.

use ap_math::rotations_gen::{rotate, Rotation};
use ap_math::scalar::{is_zero, radians, Real};
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
        }
    }

    /// Advance the timer and feed any due samples, upstream `timer_update`.
    ///
    /// Returns how many gyro and accel samples were delivered.
    pub fn timer_update(&mut self, now_us: u64, state: &SitlBodyState) -> (u32, u32) {
        let mut gyro_count = 0_u32;
        let mut accel_count = 0_u32;

        if now_us >= self.next_accel_sample_us {
            let sample = sitl_accel_sample(state, &self.cal);
            self.imu
                .notify_accel_raw_sample(sample, now_us, self.accel_rate_hz, now_us);
            self.advance_accel_schedule(now_us);
            accel_count = 1;
        }

        if now_us >= self.next_gyro_sample_us {
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
}
