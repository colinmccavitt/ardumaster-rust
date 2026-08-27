//! Multi-accelerometer selection for DCM drift correction, upstream
//! `AP_AHRS_DCM::drift_correction` per-instance `_ra_sum` loop and `besti`.
//!
//! Upstream evaluates gravity error for every healthy accelerometer and picks
//! the instance with the smallest error to reduce aliasing from vibration.

use ap_math::vector3::Vector3f;

use crate::GpsLagBuffer;

/// Maximum IMU instances, upstream `INS_MAX_INSTANCES`.
pub const INS_MAX_INSTANCES: usize = 3;

/// Per-accelerometer earth-frame accumulation, upstream `_ra_sum[i]`.
#[derive(Debug, Clone, Copy, Default)]
pub struct MultiAccelAccumulator {
    pub ra_sum: [Vector3f; INS_MAX_INSTANCES],
}

impl MultiAccelAccumulator {
    /// Clear every instance sum, upstream post-correction memset of `_ra_sum`.
    pub fn reset(&mut self) {
        for slot in &mut self.ra_sum {
            *slot = Vector3f::zero();
        }
    }
}

/// Outcome of picking the best accelerometer for one correction cycle.
#[derive(Debug, Clone, Copy)]
pub struct MultiAccelSelection {
    /// Attitude error in earth frame from the winning sensor, upstream `error[besti]`.
    pub error_ef: Vector3f,
    /// Filtered error magnitude, upstream `best_error`.
    pub best_error: f32,
    /// Winning instance, upstream `_active_accel_instance`.
    pub active_instance: i8,
}

impl MultiAccelSelection {
    /// Evaluate each healthy accelerometer and return the smallest-error winner.
    #[must_use]
    pub fn select(
        ra_sum: &[Vector3f; INS_MAX_INSTANCES],
        accel_count: u8,
        accel_healthy: impl Fn(u8) -> bool,
        ga_e: Vector3f,
        ra_scale: f32,
        using_gps_corrections: bool,
        gps_lag: &mut GpsLagBuffer,
    ) -> Option<Self> {
        let mut besti: i8 = -1;
        let mut best_error = 0.0_f32;
        let mut best_error_ef = Vector3f::zero();
        let mut best_error_dirn = 0.0_f32;

        for i in 0..accel_count {
            if !accel_healthy(i) {
                continue;
            }
            let mut ga_b = ra_sum[i as usize] * ra_scale;
            if using_gps_corrections {
                ga_b = gps_lag.ra_delayed(ga_b);
            }
            if ga_b.is_zero() || !ga_b.normalize() || ga_b.is_inf() {
                continue;
            }

            let error_ef = ga_b.cross(ga_e);
            let error_dirn = ga_b.dot(ga_e);
            let error_length = error_ef.length();

            if besti < 0 || error_length < best_error {
                besti = i as i8;
                best_error = error_length;
                best_error_ef = error_ef;
                best_error_dirn = error_dirn;
            }
        }

        if besti < 0 {
            return None;
        }

        if best_error_dirn < 0.0 {
            best_error = 1.0;
        }

        Some(Self {
            error_ef: best_error_ef,
            best_error,
            active_instance: besti,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn picks_the_sensor_with_smallest_error() {
        let mut ra_sum = [Vector3f::zero(); INS_MAX_INSTANCES];
        ra_sum[0] = Vector3f::new(0.0, 0.0, -9.80665);
        ra_sum[1] = Vector3f::new(0.5, 0.0, -9.80665);

        let ga_e = Vector3f::new(0.0, 0.0, -1.0);
        let sel = MultiAccelSelection::select(
            &ra_sum,
            2,
            |i| i < 2,
            ga_e,
            1.0 / 9.80665,
            false,
            &mut GpsLagBuffer::default(),
        )
        .expect("selection");

        assert_eq!(sel.active_instance, 0);
        assert!(sel.best_error < 0.2);
    }

    #[test]
    fn skips_unhealthy_instances() {
        let mut ra_sum = [Vector3f::zero(); INS_MAX_INSTANCES];
        ra_sum[0] = Vector3f::new(0.5, 0.0, -9.80665);
        ra_sum[1] = Vector3f::new(0.0, 0.0, -9.80665);

        let sel = MultiAccelSelection::select(
            &ra_sum,
            2,
            |i| i == 1,
            Vector3f::new(0.0, 0.0, -1.0),
            1.0 / 9.80665,
            false,
            &mut GpsLagBuffer::default(),
        )
        .expect("healthy only");

        assert_eq!(sel.active_instance, 1);
    }
}
