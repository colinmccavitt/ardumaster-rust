//! Port of `AP_AHRS_DCM`, ArduPilot's fallback attitude estimator. FW-008.
//!
//! DCM carries the vehicle's attitude as a direction cosine matrix and rotates
//! it by each gyro sample. That integration drifts, in two ways that need
//! different treatment: the matrix loses orthonormality to accumulated
//! rounding, which [`Dcm::normalize`] repairs, and the attitude itself drifts
//! against gravity and the compass, which drift correction repairs. This slice
//! is the first of those — the matrix maintenance.
//!
//! # Why DCM still matters when there is an EKF
//!
//! It is the fallback. When the EKF is unhealthy or not yet converged the
//! vehicle flies on DCM, so it is not dead code on a modern airframe — it is
//! what is holding the aircraft up on the worst day.
//!
//! # The renormalisation is deliberately not the fast one
//!
//! Upstream's comment is worth preserving: the DCM IMU paper offers a Taylor
//! expansion that avoids the square root, and ArduPilot declines it. On the
//! 2560 the sqrt cost 44 microseconds and the approximation was judged not
//! worth the extra error accumulation. Reproduced as written.

#![no_std]

use ap_math::matrix3::Matrix3f;
use ap_math::scalar::Real;
use ap_math::vector3::Vector3f;

/// Outcome of a matrix maintenance step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MatrixHealth {
    /// The matrix was usable, or was repaired in place.
    Ok,
    /// The matrix was beyond repair and the estimator must reset. Upstream
    /// calls `reset(true)`, recovering from the last Euler angles.
    NeedsReset,
}

/// The direction cosine matrix and the running renormalisation statistic.
///
/// Upstream keeps these in `AP_AHRS_DCM` alongside the drift-correction state;
/// they are separated here because matrix maintenance is self-contained and
/// can be verified on its own.
#[derive(Debug, Clone, Copy)]
pub struct Dcm {
    /// The attitude, body to earth.
    pub matrix: Matrix3f,
    renorm_val_sum: f32,
    renorm_val_count: u16,
}

impl Default for Dcm {
    fn default() -> Self {
        Self::new()
    }
}

impl Dcm {
    /// An identity attitude with no accumulated statistics.
    #[must_use]
    pub fn new() -> Self {
        Self {
            matrix: Matrix3f::identity(),
            renorm_val_sum: 0.0,
            renorm_val_count: 0,
        }
    }

    /// Rotate the matrix by a gyro sample, upstream's body of `matrix_update`.
    ///
    /// The rotation uses the corrected rate — the raw gyro plus the integral
    /// drift estimate plus both proportional terms. Upstream's own comment
    /// explains why `_omega` afterwards excludes the P terms: the spin rate is
    /// taken from its length and feeds the P gain calculation, so including
    /// them would be positive feedback.
    pub fn rotate(&mut self, omega: Vector3f, omega_p: Vector3f, omega_yaw_p: Vector3f, dt: f32) {
        self.matrix.rotate((omega + omega_p + omega_yaw_p) * dt);
    }

    /// Renormalise one row, upstream `renorm`.
    ///
    /// Returns `None` when the scale factor is so far from unity that the
    /// matrix is not worth rescuing. Note the accepted band is enormous —
    /// 1e-6 to 1e6 — because upstream would rather carry a badly scaled matrix
    /// into drift correction than reset the attitude in flight. The tighter
    /// 0.5..2.0 test exists only to mark the value as worth logging.
    fn renorm(&mut self, a: Vector3f) -> Option<Vector3f> {
        let renorm_val = 1.0 / a.length();

        self.renorm_val_sum += renorm_val;
        self.renorm_val_count += 1;

        if !(renorm_val < 2.0 && renorm_val > 0.5) && !(renorm_val < 1.0e6 && renorm_val > 1.0e-6) {
            return None;
        }
        Some(a * renorm_val)
    }

    /// Restore orthonormality, upstream `normalize`.
    ///
    /// Equations 18 to 21 of the DCM IMU paper: the dot product of the first
    /// two rows measures how far they have drifted from perpendicular, each is
    /// rotated half that error back toward the other, and the third row is
    /// rebuilt as their cross product rather than corrected — so it is exactly
    /// perpendicular to both by construction.
    pub fn normalize(&mut self) -> MatrixHealth {
        let error = self.matrix.a.dot(self.matrix.b);

        let t0 = self.matrix.a - (self.matrix.b * (0.5 * error));
        let t1 = self.matrix.b - (self.matrix.a * (0.5 * error));
        let t2 = t0.cross(t1);

        // Upstream evaluates all three with `||`, which short-circuits: a
        // failure on the first row leaves the later ones unrenormalised and
        // their statistics uncounted. Reproduced, because the count is
        // reported and the matrix is about to be reset anyway.
        let Some(a) = self.renorm(t0) else {
            return MatrixHealth::NeedsReset;
        };
        self.matrix.a = a;
        let Some(b) = self.renorm(t1) else {
            return MatrixHealth::NeedsReset;
        };
        self.matrix.b = b;
        let Some(c) = self.renorm(t2) else {
            return MatrixHealth::NeedsReset;
        };
        self.matrix.c = c;
        MatrixHealth::Ok
    }

    /// Check for values that would poison the attitude, upstream
    /// `check_matrix`.
    ///
    /// The specific danger is `c.x` outside `-1..1`: pitch is recovered with
    /// `asin(c.x)`, so an out-of-range value yields NaN, and that NaN feeds
    /// back through the course error into the rest of the matrix. Upstream
    /// tries a normalisation first and only resets if that fails to bring it
    /// back — and the threshold it accepts afterwards is 10.0, not 1.0, with a
    /// comment pointing at issue #20284 and declining to tighten it without
    /// evidence.
    pub fn check_matrix(&mut self) -> MatrixHealth {
        if self.matrix.is_nan() {
            return MatrixHealth::NeedsReset;
        }
        if self.matrix.c.x < 1.0 && self.matrix.c.x > -1.0 {
            return MatrixHealth::Ok;
        }
        // out of range: try to repair it
        if self.normalize() == MatrixHealth::NeedsReset {
            return MatrixHealth::NeedsReset;
        }
        if self.matrix.is_nan() || Real::abs(self.matrix.c.x) > 10.0 {
            return MatrixHealth::NeedsReset;
        }
        MatrixHealth::Ok
    }

    /// Mean renormalisation factor since the last call, upstream's
    /// `_renorm_val_sum / _renorm_val_count` reporting, resetting the running
    /// total.
    ///
    /// A value drifting away from 1.0 means the matrix is being pulled out of
    /// shape faster than the integration should be doing.
    pub fn take_renorm_average(&mut self) -> Option<f32> {
        if self.renorm_val_count == 0 {
            return None;
        }
        let avg = self.renorm_val_sum / f32::from(self.renorm_val_count);
        self.renorm_val_sum = 0.0;
        self.renorm_val_count = 0;
        Some(avg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn skewed() -> Matrix3f {
        // Rows drifted away from perpendicular, as integration does. Note the
        // signs: (1, 0.05, 0) and (-0.05, 1, 0) look skewed and are exactly
        // perpendicular, which is the wrong fixture -- only their lengths are
        // off. These two have a dot product of 0.2.
        Matrix3f {
            a: Vector3f::new(1.0, 0.1, 0.0),
            b: Vector3f::new(0.1, 1.0, 0.0),
            c: Vector3f::new(0.0, 0.0, 1.0),
        }
    }

    fn orthonormality_error(m: &Matrix3f) -> f32 {
        let mut worst: f32 = 0.0;
        for v in [m.a, m.b, m.c] {
            worst = worst.max((v.length() - 1.0).abs());
        }
        worst = worst.max(m.a.dot(m.b).abs());
        worst = worst.max(m.a.dot(m.c).abs());
        worst = worst.max(m.b.dot(m.c).abs());
        worst
    }

    /// PORT-DERIVED. The whole point of the correction: rows come back to unit
    /// length and mutual perpendicularity.
    #[test]
    fn normalize_restores_orthonormality() {
        let mut d = Dcm::new();
        d.matrix = skewed();
        assert!(
            orthonormality_error(&d.matrix) > 0.04,
            "the fixture should start visibly skewed"
        );
        assert_eq!(d.normalize(), MatrixHealth::Ok);
        assert!(
            orthonormality_error(&d.matrix) < 1e-6,
            "still skewed by {}",
            orthonormality_error(&d.matrix)
        );
    }

    /// PORT-DERIVED. The third row is rebuilt as a cross product rather than
    /// corrected, so it is exactly perpendicular to the other two by
    /// construction however far it had drifted.
    #[test]
    fn the_third_row_is_rebuilt_not_corrected() {
        let mut d = Dcm::new();
        d.matrix = skewed();
        d.matrix.c = Vector3f::new(0.3, 0.4, 0.5); // badly wrong
        assert_eq!(d.normalize(), MatrixHealth::Ok);
        assert!(d.matrix.a.dot(d.matrix.c).abs() < 1e-6);
        assert!(d.matrix.b.dot(d.matrix.c).abs() < 1e-6);
        assert!((d.matrix.c.length() - 1.0).abs() < 1e-6);
    }

    /// PORT-DERIVED. A uniformly scaled matrix is scaled back rather than
    /// rejected: renormalisation is exactly what handles it.
    #[test]
    fn a_uniformly_scaled_matrix_is_rescued() {
        let mut d = Dcm::new();
        d.matrix = Matrix3f::identity() * 0.5;
        assert_eq!(d.normalize(), MatrixHealth::Ok);
        assert!(orthonormality_error(&d.matrix) < 1e-6);
    }

    /// PORT-DERIVED. A zero row cannot be renormalised -- the scale factor is
    /// infinite -- and upstream answers that by resetting the estimator rather
    /// than propagating NaN into the attitude.
    #[test]
    fn a_degenerate_row_demands_a_reset() {
        let mut d = Dcm::new();
        d.matrix = Matrix3f {
            a: Vector3f::new(1.0, 0.0, 0.0),
            b: Vector3f::new(0.0, 0.0, 0.0),
            c: Vector3f::new(0.0, 0.0, 1.0),
        };
        assert_eq!(d.normalize(), MatrixHealth::NeedsReset);
    }

    /// PORT-DERIVED. The accept band is enormous on purpose: upstream would
    /// rather carry a badly scaled matrix into drift correction than reset the
    /// attitude in flight. A factor of a thousand is still accepted.
    #[test]
    fn the_accept_band_is_deliberately_wide() {
        let mut d = Dcm::new();
        d.matrix = Matrix3f::identity() * 0.001;
        assert_eq!(
            d.normalize(),
            MatrixHealth::Ok,
            "a thousandfold scaling is inside upstream's 1e-6..1e6 band"
        );
        assert!(orthonormality_error(&d.matrix) < 1e-5);
    }

    /// PORT-DERIVED. `c.x` outside -1..1 makes the pitch calculation produce
    /// NaN, so it is checked separately and repaired by normalising.
    #[test]
    fn an_out_of_range_c_x_is_repaired_rather_than_reset() {
        let mut d = Dcm::new();
        d.matrix = Matrix3f {
            a: Vector3f::new(1.0, 0.0, 0.0),
            b: Vector3f::new(0.0, 1.0, 0.0),
            c: Vector3f::new(1.5, 0.0, 1.0),
        };
        assert_eq!(d.check_matrix(), MatrixHealth::Ok);
        assert!(
            d.matrix.c.x.abs() <= 1.0,
            "c.x should be back in range, is {}",
            d.matrix.c.x
        );
    }

    /// PORT-DERIVED. A NaN anywhere is unrecoverable: there is nothing to
    /// normalise toward.
    #[test]
    fn a_nan_matrix_demands_a_reset() {
        let mut d = Dcm::new();
        d.matrix = Matrix3f::identity();
        d.matrix.a.x = f32::NAN;
        assert_eq!(d.check_matrix(), MatrixHealth::NeedsReset);
    }

    /// PORT-DERIVED. The renormalisation statistic averages the factors used
    /// and clears on read, which is how upstream reports it.
    #[test]
    fn the_renorm_average_clears_on_read() {
        let mut d = Dcm::new();
        assert_eq!(d.take_renorm_average(), None);
        d.matrix = Matrix3f::identity();
        assert_eq!(d.normalize(), MatrixHealth::Ok);
        let avg = d
            .take_renorm_average()
            .expect("three rows were renormalised");
        assert!(
            (avg - 1.0).abs() < 1e-6,
            "identity should need no scaling, got {avg}"
        );
        assert_eq!(d.take_renorm_average(), None, "reading should clear it");
    }

    /// PORT-DERIVED. The small-angle rotation is not a true rotation: it
    /// leaves the matrix slightly non-orthonormal, which is precisely why
    /// normalisation runs after it.
    #[test]
    fn the_rotation_step_leaves_work_for_the_normaliser() {
        let mut d = Dcm::new();
        let rate = Vector3f::new(0.0, 0.0, 1.0); // 1 rad/s in yaw
        d.rotate(rate, Vector3f::zero(), Vector3f::zero(), 0.1);
        let after_rotation = orthonormality_error(&d.matrix);
        assert!(
            after_rotation > 1e-4,
            "a first-order rotation should distort the matrix, error {after_rotation}"
        );
        assert_eq!(d.normalize(), MatrixHealth::Ok);
        assert!(orthonormality_error(&d.matrix) < 1e-6);
    }
}
