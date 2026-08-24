//! Port of `AP_Math/quaternion.{h,cpp}`, pinned to `Plane-4.7.0`.
//!
//! Components keep upstream's names `q1..q4`, where `q1` is the scalar part
//! and `q2,q3,q4` the vector part. That ordering is unusual — many libraries
//! put the scalar last — so the names are kept verbatim rather than renamed to
//! `w,x,y,z`, to stop a reader silently assuming the other convention.
//!
//! # Divergence from upstream
//!
//! `rotation_matrix()` returns a [`Matrix3`] instead of filling an
//! out-parameter, and `earth_to_body()` returns the rotated vector instead of
//! mutating it in place.
//!
//! [`QuaternionT::normalize`] is the significant one. Upstream guards against a
//! zero-length quaternion and raises `INTERNAL_ERROR(flow_of_control)` in that
//! branch, leaving the value untouched. The port has no error-reporting channel
//! yet, so it leaves the value untouched and returns `false` — the caller can
//! see what upstream would have reported. Wiring that to a real internal-error
//! path is tracked separately; it is deliberately not silently dropped.
//!
//! Note this differs from `Vector2`/`Vector3::normalized`, which have **no**
//! zero guard at all and produce NaN. That inconsistency is upstream's.

use core::ops::Mul;

use crate::matrix3::Matrix3;
use crate::scalar::{is_zero, safe_asin, Real};
use crate::vector3::Vector3;

/// Quaternion with the scalar part first. Upstream `QuaternionT<T>`.
#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct QuaternionT<T> {
    /// Scalar part.
    pub q1: T,
    /// Vector part, x.
    pub q2: T,
    /// Vector part, y.
    pub q3: T,
    /// Vector part, z.
    pub q4: T,
}

/// Upstream `Quaternion`.
pub type Quaternion = QuaternionT<f32>;
/// Upstream `QuaternionD`.
pub type QuaternionD = QuaternionT<f64>;

impl<T: Real> Default for QuaternionT<T> {
    /// The identity rotation, matching upstream's default constructor and
    /// `initialise()`.
    #[inline]
    fn default() -> Self {
        Self::identity()
    }
}

impl<T: Real> QuaternionT<T> {
    /// Construct from components, scalar part first.
    #[inline]
    pub fn new(q1: T, q2: T, q3: T, q4: T) -> Self {
        Self { q1, q2, q3, q4 }
    }

    /// The identity rotation `(1, 0, 0, 0)`. Upstream `initialise()`.
    #[inline]
    pub fn identity() -> Self {
        Self::new(T::one(), T::zero(), T::zero(), T::zero())
    }

    /// Set to the identity rotation, in place. Upstream `initialise()`.
    #[inline]
    pub fn initialise(&mut self) {
        *self = Self::identity();
    }

    /// All components zero. Upstream `zero()`.
    ///
    /// This is not a valid rotation; upstream uses it as a sentinel.
    #[inline]
    pub fn set_zero(&mut self) {
        *self = Self::new(T::zero(), T::zero(), T::zero(), T::zero());
    }

    /// Squared length. Upstream `length_squared()`.
    #[inline]
    pub fn length_squared(self) -> T {
        self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3 + self.q4 * self.q4
    }

    /// Length. Upstream `length()`.
    #[inline]
    pub fn length(self) -> T {
        self.length_squared().sqrt()
    }

    /// Normalize in place, returning false if the quaternion has zero length.
    ///
    /// Upstream `normalize()` raises `INTERNAL_ERROR(flow_of_control)` on the
    /// zero branch and leaves the value untouched. The boolean stands in for
    /// that report until the port has an internal-error channel.
    #[inline]
    pub fn normalize(&mut self) -> bool {
        let mag = self.length();
        if is_zero(mag) {
            return false;
        }
        let inv = T::one() / mag;
        self.q1 = self.q1 * inv;
        self.q2 = self.q2 * inv;
        self.q3 = self.q3 * inv;
        self.q4 = self.q4 * inv;
        true
    }

    /// True if within 1e-3 of unit length. Upstream `is_unit_length()`.
    ///
    /// Note the tolerance is a literal `1E-3` on the **squared** length, far
    /// looser than [`is_zero`]'s epsilon.
    #[inline]
    pub fn is_unit_length(self) -> bool {
        (self.length_squared() - T::one()).abs() < T::from_f64(1.0e-3)
    }

    /// True if all components are zero. Upstream `is_zero()`.
    #[inline]
    pub fn is_zero(self) -> bool {
        is_zero(self.q1) && is_zero(self.q2) && is_zero(self.q3) && is_zero(self.q4)
    }

    /// True if any component is NaN. Upstream `is_nan()`.
    #[inline]
    pub fn is_nan(self) -> bool {
        self.q1.is_nan() || self.q2.is_nan() || self.q3.is_nan() || self.q4.is_nan()
    }

    /// The reverse rotation. Upstream `inverse()`, the conjugate.
    #[inline]
    pub fn inverse(self) -> Self {
        Self::new(self.q1, -self.q2, -self.q3, -self.q4)
    }

    /// Reverse this rotation in place. Upstream `invert()`.
    #[inline]
    pub fn invert(&mut self) {
        *self = self.inverse();
    }

    /// Build from euler angles in radians. Upstream `from_euler()`.
    #[inline]
    pub fn from_euler(roll: T, pitch: T, yaw: T) -> Self {
        let h = T::from_f64(0.5);
        let cr2 = (roll * h).cos();
        let cp2 = (pitch * h).cos();
        let cy2 = (yaw * h).cos();
        let sr2 = (roll * h).sin();
        let sp2 = (pitch * h).sin();
        let sy2 = (yaw * h).sin();

        Self::new(
            cr2 * cp2 * cy2 + sr2 * sp2 * sy2,
            sr2 * cp2 * cy2 - cr2 * sp2 * sy2,
            cr2 * sp2 * cy2 + sr2 * cp2 * sy2,
            cr2 * cp2 * sy2 - sr2 * sp2 * cy2,
        )
    }

    /// Euler roll in radians. Upstream `get_euler_roll()`.
    #[inline]
    pub fn get_euler_roll(self) -> T {
        let two = T::one() + T::one();
        (two * (self.q1 * self.q2 + self.q3 * self.q4))
            .atan2(T::one() - two * (self.q2 * self.q2 + self.q3 * self.q3))
    }

    /// Euler pitch in radians. Upstream `get_euler_pitch()`.
    ///
    /// Uses [`safe_asin`], so gimbal-lock inputs clamp instead of producing NaN.
    #[inline]
    pub fn get_euler_pitch(self) -> T {
        let two = T::one() + T::one();
        safe_asin(two * (self.q1 * self.q3 - self.q4 * self.q2))
    }

    /// Euler yaw in radians. Upstream `get_euler_yaw()`.
    #[inline]
    pub fn get_euler_yaw(self) -> T {
        let two = T::one() + T::one();
        (two * (self.q1 * self.q4 + self.q2 * self.q3))
            .atan2(T::one() - two * (self.q3 * self.q3 + self.q4 * self.q4))
    }

    /// Euler angles as `(roll, pitch, yaw)` in radians. Upstream `to_euler()`.
    #[inline]
    pub fn to_euler(self) -> (T, T, T) {
        (
            self.get_euler_roll(),
            self.get_euler_pitch(),
            self.get_euler_yaw(),
        )
    }

    /// The equivalent rotation matrix. Upstream `rotation_matrix()`, which
    /// fills an out-parameter.
    #[inline]
    pub fn rotation_matrix(self) -> Matrix3<T> {
        let (q1, q2, q3, q4) = (self.q1, self.q2, self.q3, self.q4);
        let q3q3 = q3 * q3;
        let q3q4 = q3 * q4;
        let q2q2 = q2 * q2;
        let q2q3 = q2 * q3;
        let q2q4 = q2 * q4;
        let q1q2 = q1 * q2;
        let q1q3 = q1 * q3;
        let q1q4 = q1 * q4;
        let q4q4 = q4 * q4;
        let one = T::one();
        let two = one + one;

        Matrix3::new(
            one - two * (q3q3 + q4q4),
            two * (q2q3 - q1q4),
            two * (q2q4 + q1q3),
            two * (q2q3 + q1q4),
            one - two * (q2q2 + q4q4),
            two * (q3q4 - q1q2),
            two * (q2q4 - q1q3),
            two * (q3q4 + q1q2),
            one - two * (q2q2 + q3q3),
        )
    }

    /// Rotate a vector from the earth frame to the body frame.
    ///
    /// Upstream `earth_to_body()` mutates its argument; this returns the
    /// rotated vector. Same computation: `rotation_matrix() * v`.
    #[inline]
    pub fn earth_to_body(self, v: Vector3<T>) -> Vector3<T> {
        self.rotation_matrix() * v
    }
}

/// Quaternion product. Upstream `operator*(const QuaternionT<T>&)`.
impl<T: Real> Mul for QuaternionT<T> {
    type Output = Self;
    #[inline]
    fn mul(self, v: Self) -> Self {
        let (w1, x1, y1, z1) = (self.q1, self.q2, self.q3, self.q4);
        let (w2, x2, y2, z2) = (v.q1, v.q2, v.q3, v.q4);
        Self::new(
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )
    }
}

/// Component-wise equality, consistent with the vector types.
impl<T: Real> PartialEq for QuaternionT<T> {
    #[inline]
    fn eq(&self, o: &Self) -> bool {
        use crate::scalar::is_equal;
        is_equal(self.q1, o.q1)
            && is_equal(self.q2, o.q2)
            && is_equal(self.q3, o.q3)
            && is_equal(self.q4, o.q4)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;

    // Cases ported from upstream libraries/AP_Math/tests/test_quaternion.cpp
    // at Plane-4.7.0.

    /// Upstream indexes quaternions with operator[]; this stands in for it.
    fn comps(q: Quaternion) -> [f32; 4] {
        [q.q1, q.q2, q.q3, q.q4]
    }

    fn near(a: f32, b: f32) {
        assert!((a - b).abs() < 1.0e-6, "expected {b}, got {a}");
    }

    /// Compare all four components. Upstream loops `for (int a = 0; a < 4; ++a)`
    /// over `operator[]`; zipping avoids indexing so the workspace
    /// `indexing_slicing` lint keeps its value on real code.
    fn near_all(a: Quaternion, b: Quaternion) {
        for (x, y) in comps(a).into_iter().zip(comps(b)) {
            near(x, y);
        }
    }

    /// Component-wise negation, for upstream's `-unit` style expectations.
    fn neg(q: Quaternion) -> Quaternion {
        Quaternion::new(-q.q1, -q.q2, -q.q3, -q.q4)
    }

    /// UPSTREAM-PARITY: TEST(QuaternionTest, QuaternionMultiplicationOfBases)
    ///
    /// The full Hamilton table: i^2 = j^2 = k^2 = ijk = -1, ij = k, jk = i,
    /// ki = j, and the reversed products negate. This is the test that would
    /// catch a flipped multiplication convention, which is the classic
    /// quaternion porting bug.
    #[test]
    fn multiplication_of_bases_matches_upstream() {
        let unit = Quaternion::new(1.0, 0.0, 0.0, 0.0);
        let i = Quaternion::new(0.0, 1.0, 0.0, 0.0);
        let j = Quaternion::new(0.0, 0.0, 1.0, 0.0);
        let k = Quaternion::new(0.0, 0.0, 0.0, 1.0);

        let ii = i * i;
        let ij = i * j;
        let ik = i * k;
        let ji = j * i;
        let jj = j * j;
        let jk = j * k;
        let ki = k * i;
        let kj = k * j;
        let kk = k * k;
        let ijk = i * j * k;

        near_all(ii, jj);
        near_all(jj, kk);
        near_all(kk, ijk);
        near_all(ijk, neg(unit));
        near_all(ij, k);
        near_all(ii, neg(unit));
        near_all(ik, neg(j));
        near_all(ji, neg(k));
        near_all(jj, neg(unit));
        near_all(jk, i);
        near_all(ki, j);
        near_all(kj, neg(i));
        near_all(kk, neg(unit));
    }

    /// UPSTREAM-PARITY: TEST(QuaternionTest, QuaternionToRotationMatrix)
    ///
    /// Upstream cites "Why and How to Avoid the Flipped Quaternion
    /// Multiplication" (arxiv 1801.07478) for this case: a 90 degree yaw.
    #[test]
    fn to_rotation_matrix_matches_upstream() {
        let r = 0.5 * libm::sqrtf(2.0);
        let m = Quaternion::new(r, 0.0, 0.0, r).rotation_matrix();

        near(m.a.x, 0.0);
        near(m.a.y, -1.0);
        near(m.a.z, 0.0);
        near(m.b.x, 1.0);
        near(m.b.y, 0.0);
        near(m.b.z, 0.0);
        near(m.c.x, 0.0);
        near(m.c.y, 0.0);
        near(m.c.z, 1.0);
    }

    /// UPSTREAM-PARITY: TEST(QuaternionTest, QuaternionMultiplicationIsHomomorphism)
    ///
    /// C(q0 * q1) == C(q0) * C(q1).
    #[test]
    fn multiplication_is_homomorphism_with_matrices() {
        let q0 = Quaternion::from_euler(0.3, -0.2, 1.1);
        let q1 = Quaternion::from_euler(-0.7, 0.4, 0.2);

        let lhs = (q0 * q1).rotation_matrix();
        let rhs = q0.rotation_matrix() * q1.rotation_matrix();

        for (a, b) in [
            (lhs.a.x, rhs.a.x),
            (lhs.a.y, rhs.a.y),
            (lhs.a.z, rhs.a.z),
            (lhs.b.x, rhs.b.x),
            (lhs.b.y, rhs.b.y),
            (lhs.b.z, rhs.b.z),
            (lhs.c.x, rhs.c.x),
            (lhs.c.y, rhs.c.y),
            (lhs.c.z, rhs.c.z),
        ] {
            assert!((a - b).abs() < 1.0e-5, "{a} vs {b}");
        }
    }

    /// UPSTREAM-PARITY: TEST(QuaternionTest, Quaternion_is_zero)
    #[test]
    fn is_zero_matches_upstream() {
        assert!(Quaternion::new(0.0, 0.0, 0.0, 0.0).is_zero());
        assert!(!Quaternion::new(0.836_516_3, 0.482_962_9, 0.224_143_87, -0.129_409_52).is_zero());
        assert!(!Quaternion::new(0.9, 0.0, 0.0, 0.0).is_zero());
    }

    /// UPSTREAM-PARITY: TEST(QuaternionTest, Quaternion_is_unit_length)
    ///
    /// The tolerance is a literal 1e-3 on the SQUARED length, so lengths a
    /// little either side of 1 still count as unit.
    #[test]
    fn is_unit_length_matches_upstream() {
        // zero length
        assert!(!Quaternion::new(0.0, 0.0, 0.0, 0.0).is_unit_length());
        // length_squared == 1.0 - 0.0009, just inside tolerance
        assert!(
            Quaternion::new(0.836_139_8, 0.482_745_5, 0.224_043, -0.129_351_2).is_unit_length()
        );
        // unit length
        assert!(
            Quaternion::new(0.836_516_3, 0.482_962_9, 0.224_143_87, -0.129_409_52).is_unit_length()
        );
        assert!(Quaternion::new(1.0, 0.0, 0.0, 0.0).is_unit_length());
        // length_squared == 1.0 + 0.0009, just inside tolerance
        assert!(
            Quaternion::new(0.836_892_6, 0.483_180_2, 0.224_244_7, -0.129_467_7).is_unit_length()
        );
        // length 1.2, outside
        assert!(!Quaternion::new(1.003_82, 0.579_555, 0.268_973, -0.155_291).is_unit_length());
    }

    /// UPSTREAM-PARITY: TEST(QuaternionTest, Quaternion_length_squared)
    #[test]
    fn length_squared_matches_upstream() {
        near(Quaternion::new(0.0, 0.0, 0.0, 0.0).length_squared(), 0.0);
        assert!(
            (Quaternion::new(0.836_139_8, 0.482_745_5, 0.224_043, -0.129_351_2).length_squared()
                - (1.0 - 0.0009))
                .abs()
                < 1.0e-5
        );
        assert!(
            (Quaternion::new(0.836_516_3, 0.482_962_9, 0.224_143_87, -0.129_409_52)
                .length_squared()
                - 1.0)
                .abs()
                < 1.0e-5
        );
    }

    /// PORT-DERIVED: euler round trip through the quaternion.
    #[test]
    fn euler_roundtrip_derived() {
        let cases = [
            (0.0_f32, 0.0, 0.0),
            (0.1, 0.2, 0.3),
            (-0.4, 0.5, -1.2),
            (0.7, -0.6, 2.5),
        ];
        for (roll, pitch, yaw) in cases {
            let q = Quaternion::from_euler(roll, pitch, yaw);
            let (r, p, y) = q.to_euler();
            assert!((r - roll).abs() < 1.0e-5, "roll {roll} -> {r}");
            assert!((p - pitch).abs() < 1.0e-5, "pitch {pitch} -> {p}");
            assert!((y - yaw).abs() < 1.0e-5, "yaw {yaw} -> {y}");
        }
    }

    /// PORT-DERIVED: from_euler agrees with Matrix3::from_euler.
    ///
    /// Cross-checks the two independent euler formulations against each other,
    /// which is the kind of check that catches a transposed convention.
    #[test]
    fn quaternion_and_matrix_euler_agree_derived() {
        let (roll, pitch, yaw) = (0.3_f32, -0.2, 1.1);
        let qm = Quaternion::from_euler(roll, pitch, yaw).rotation_matrix();
        let mm = crate::matrix3::Matrix3f::from_euler(roll, pitch, yaw);
        for (a, b) in [
            (qm.a.x, mm.a.x),
            (qm.a.y, mm.a.y),
            (qm.a.z, mm.a.z),
            (qm.b.x, mm.b.x),
            (qm.b.y, mm.b.y),
            (qm.b.z, mm.b.z),
            (qm.c.x, mm.c.x),
            (qm.c.y, mm.c.y),
            (qm.c.z, mm.c.z),
        ] {
            assert!((a - b).abs() < 1.0e-5, "{a} vs {b}");
        }
    }

    /// PORT-DERIVED: inverse undoes the rotation, and normalize reports the
    /// zero case rather than producing NaN.
    #[test]
    fn inverse_and_normalize_derived() {
        let q = Quaternion::from_euler(0.3, -0.2, 1.1);
        let round = q * q.inverse();
        near(round.q1, 1.0);
        near(round.q2, 0.0);
        near(round.q3, 0.0);
        near(round.q4, 0.0);

        let mut n = Quaternion::new(2.0, 0.0, 0.0, 0.0);
        assert!(n.normalize());
        assert!(n.is_unit_length());

        // Upstream raises INTERNAL_ERROR here and leaves the value alone.
        // The port returns false and likewise does not modify it.
        let mut z = Quaternion::new(0.0, 0.0, 0.0, 0.0);
        assert!(!z.normalize());
        assert!(z.is_zero(), "a failed normalize must not modify the value");
        assert!(!z.is_nan());
    }

    /// PORT-DERIVED: earth_to_body agrees with applying the rotation matrix.
    #[test]
    fn earth_to_body_derived() {
        let q = Quaternion::from_euler(0.0, 0.0, core::f32::consts::FRAC_PI_2);
        let v = Vector3::new(1.0, 0.0, 0.0);
        let rotated = q.earth_to_body(v);
        let expected = q.rotation_matrix() * v;
        near(rotated.x, expected.x);
        near(rotated.y, expected.y);
        near(rotated.z, expected.z);
    }
}
