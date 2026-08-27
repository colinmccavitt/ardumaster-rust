//! Wind estimation for DCM drift correction, upstream `AP_AHRS_DCM::estimate_wind`.
//!
//! The wind triangle feeds the no-GPS drift branch, which estimates ground
//! velocity as fuselage-direction airspeed plus wind, and yaw consistency
//! checks that read horizontal wind speed.

use ap_math::scalar::{degrees, radians, safe_sqrt, Real};
use ap_math::vector3::Vector3f;

/// Minimum interval between wind updates, upstream 100 ms rate limit.
const MIN_ESTIMATE_INTERVAL_MS: u32 = 100;
/// Fuselage direction change that triggers the turning estimate, upstream 0.2f.
const TURN_FUSE_DIFF_MIN: f32 = 0.2;
/// Straight-flight airspeed branch interval, upstream 2000 ms.
const STRAIGHT_WIND_INTERVAL_MS: u32 = 2000;
/// Reset turning state after this gap, upstream 10000 ms.
const TURN_STATE_TIMEOUT_MS: u32 = 10_000;
/// Spike rejection margin above current wind length, upstream 20 m/s.
const WIND_SPIKE_MARGIN_M_S: f32 = 20.0;

/// Inputs for one wind estimate call.
#[derive(Debug, Clone, Copy, Default)]
pub struct WindEstimateInputs {
    pub now_ms: u32,
    pub velocity: Vector3f,
    pub fuselage_direction: Vector3f,
    pub airspeed_eas: Option<f32>,
    pub eas2tas: f32,
    pub enabled: bool,
}

/// Running wind estimate, upstream `_wind` and turning-state buffers.
#[derive(Debug, Clone, Copy, Default)]
pub struct WindEstimator {
    pub wind: Vector3f,
    last_fuse: Vector3f,
    last_vel: Vector3f,
    last_wind_time_ms: u32,
    last_estimate_ms: u32,
}

impl WindEstimator {
    /// Empty wind estimate.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Update the wind estimate, upstream `estimate_wind`.
    pub fn estimate(&mut self, inp: WindEstimateInputs) {
        if !inp.enabled {
            return;
        }
        if inp.now_ms.wrapping_sub(self.last_estimate_ms) < MIN_ESTIMATE_INTERVAL_MS {
            return;
        }
        self.last_estimate_ms = inp.now_ms;

        let fuse_diff = inp.fuselage_direction - self.last_fuse;
        let diff_length = fuse_diff.length();

        if inp.now_ms.wrapping_sub(self.last_wind_time_ms) > TURN_STATE_TIMEOUT_MS {
            self.last_wind_time_ms = inp.now_ms;
            self.last_fuse = inp.fuselage_direction;
            self.last_vel = inp.velocity;
            return;
        }

        if diff_length > TURN_FUSE_DIFF_MIN {
            self.estimate_turning(inp, fuse_diff, diff_length);
            return;
        }

        if let Some(eas) = inp.airspeed_eas {
            if inp.now_ms.wrapping_sub(self.last_wind_time_ms) > STRAIGHT_WIND_INTERVAL_MS {
                let airspeed = inp.fuselage_direction * (eas * inp.eas2tas);
                let sample = inp.velocity - airspeed;
                self.wind = self.wind * 0.92 + sample * 0.08;
            }
        }
    }

    fn estimate_turning(
        &mut self,
        inp: WindEstimateInputs,
        fuse_diff: Vector3f,
        diff_length: f32,
    ) {
        let velocity_diff = inp.velocity - self.last_vel;
        let v = velocity_diff.length() / diff_length;

        let fuse_sum = inp.fuselage_direction + self.last_fuse;
        let vel_sum = inp.velocity + self.last_vel;

        self.last_fuse = inp.fuselage_direction;
        self.last_vel = inp.velocity;

        let theta = velocity_diff.y.atan2(velocity_diff.x)
            - fuse_diff.y.atan2(fuse_diff.x);
        let sintheta = theta.sin();
        let costheta = theta.cos();

        let sample = Vector3f::new(
            vel_sum.x - v * (costheta * fuse_sum.x - sintheta * fuse_sum.y),
            vel_sum.y - v * (sintheta * fuse_sum.x + costheta * fuse_sum.y),
            vel_sum.z - v * fuse_sum.z,
        ) * 0.5;

        if sample.length() < self.wind.length() + WIND_SPIKE_MARGIN_M_S {
            self.wind = self.wind * 0.95 + sample * 0.05;
        }
        self.last_wind_time_ms = inp.now_ms;
    }

    /// Ground velocity when GPS is unavailable, upstream no-GPS branch of
    /// `drift_correction`.
    #[must_use]
    pub fn ground_velocity_no_gps(
        &self,
        fuselage_direction: Vector3f,
        airspeed_tas: f32,
    ) -> Vector3f {
        fuselage_direction * airspeed_tas + self.wind
    }

    /// Horizontal wind speed for yaw consistency checks.
    #[must_use]
    pub fn wind_speed_xy(&self) -> f32 {
        safe_sqrt(self.wind.x * self.wind.x + self.wind.y * self.wind.y)
    }
}

/// Wind alignment with a heading in degrees, upstream `AP_AHRS::wind_alignment`.
#[must_use]
pub fn wind_alignment(heading_deg: f32, wind: Vector3f) -> f32 {
    if wind.x == 0.0 && wind.y == 0.0 {
        return 0.0;
    }
    let wind_heading_rad = (-wind.y).atan2(-wind.x);
    Real::cos(wind_heading_rad - radians(heading_deg))
}

/// Head-wind component along vehicle yaw, upstream `AP_AHRS::head_wind`.
#[must_use]
pub fn head_wind_from_yaw(yaw_rad: f32, wind: Vector3f) -> f32 {
    wind_alignment(degrees(yaw_rad), wind) * safe_sqrt(wind.x * wind.x + wind.y * wind.y)
}

/// True-wind sample from a wind vane, upstream `AP_WindVane` direction/speed.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct WindVaneSample {
    /// Wind direction, radians, 0 = north. Upstream `_direction_true`.
    pub direction_true_rad: f32,
    /// True wind speed, m/s. Upstream `_speed_true`.
    pub speed_true_mps: f32,
}

impl WindVaneSample {
    /// Convert to NED wind velocity for AHRS drift, upstream wind triangle.
    #[must_use]
    pub fn to_wind_ned(self) -> Vector3f {
        Vector3f::new(
            -self.speed_true_mps * Real::cos(self.direction_true_rad),
            -self.speed_true_mps * Real::sin(self.direction_true_rad),
            0.0,
        )
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use ap_math::matrix3::Matrix3f;

    #[test]
    fn straight_flight_airspeed_branch_blends_wind() {
        let mut est = WindEstimator::new();
        est.last_fuse = Vector3f::new(1.0, 0.0, 0.0);
        est.last_vel = Vector3f::new(12.0, 3.0, 0.0);
        est.last_wind_time_ms = 0;
        est.estimate(WindEstimateInputs {
            now_ms: 3000,
            velocity: Vector3f::new(12.0, 3.0, 0.0),
            fuselage_direction: Vector3f::new(1.0, 0.0, 0.0),
            airspeed_eas: Some(10.0),
            eas2tas: 1.0,
            enabled: true,
        });
        assert!(
            est.wind.y > 0.0,
            "wind should pick up the crosswind component, got {:?}",
            est.wind
        );
    }

    #[test]
    fn turning_branch_updates_wind_from_velocity_change() {
        let mut est = WindEstimator::new();
        est.last_fuse = Vector3f::new(1.0, 0.0, 0.0);
        est.last_vel = Vector3f::new(10.0, 0.0, 0.0);
        est.last_wind_time_ms = 1000;
        est.estimate(WindEstimateInputs {
            now_ms: 1100,
            velocity: Vector3f::new(10.0, 4.0, 0.0),
            fuselage_direction: Vector3f::new(0.7, 0.7, 0.0).normalized_or_zero(),
            airspeed_eas: None,
            eas2tas: 1.0,
            enabled: true,
        });
        assert!(
            est.wind.length() > 0.0,
            "turning estimate should move wind off zero, got {:?}",
            est.wind
        );
    }

    #[test]
    fn ground_velocity_no_gps_adds_wind_to_airspeed_vector() {
        let mut est = WindEstimator::new();
        est.wind = Vector3f::new(0.0, 2.0, 0.0);
        let ground = est.ground_velocity_no_gps(Vector3f::new(1.0, 0.0, 0.0), 10.0);
        assert_eq!(ground.x, 10.0);
        assert_eq!(ground.y, 2.0);
    }

    #[test]
    #[test]
    fn head_wind_from_north_wind() {
        let wind = Vector3f::new(-5.0, 0.0, 0.0);
        assert!((head_wind_from_yaw(0.0, wind) - 5.0).abs() < 0.01);
    }

    fn body_x_column_is_fuselage_direction() {
        let m = Matrix3f::from_euler(0.0, 0.0, core::f32::consts::FRAC_PI_2);
        let fuse = m.colx();
        assert!(fuse.x.abs() < 1e-5);
        assert!(fuse.y > 0.9);
    }
}
