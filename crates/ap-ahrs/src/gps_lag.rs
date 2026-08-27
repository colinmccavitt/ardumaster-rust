//! GPS lag buffer for DCM drift correction, upstream `AP_AHRS_DCM::ra_delayed`.
//!
//! GPS velocity arrives one cycle late relative to the IMU-integrated gravity
//! estimate. A one-sample delay line on the scaled `ra_sum` vector aligns the
//! two before comparing directions.

use ap_math::vector3::Vector3f;

/// One-sample delay line matching GPS lag, upstream `_ra_delay_buffer`.
#[derive(Debug, Clone, Copy, Default)]
pub struct GpsLagBuffer {
    ra_delay_buffer: Vector3f,
}

impl GpsLagBuffer {
    /// Return the previous sample and store `ra`, upstream `ra_delayed()`.
    #[must_use]
    pub fn ra_delayed(&mut self, ra: Vector3f) -> Vector3f {
        let ret = self.ra_delay_buffer;
        self.ra_delay_buffer = ra;
        if ret.is_zero() {
            // Upstream passthrough on first call when buffer is exactly zero.
            ra
        } else {
            ret
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_call_passthrough_then_delays_by_one_sample() {
        let mut lag = GpsLagBuffer::default();
        let v1 = Vector3f::new(1.0, 2.0, 3.0);
        assert_eq!(lag.ra_delayed(v1), v1);

        let v2 = Vector3f::new(4.0, 5.0, 6.0);
        assert_eq!(lag.ra_delayed(v2), v1);

        let v3 = Vector3f::new(7.0, 8.0, 9.0);
        assert_eq!(lag.ra_delayed(v3), v2);
    }
}
