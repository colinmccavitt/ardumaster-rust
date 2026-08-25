//! Port of `AP_Math/matrix3.{h,cpp}`, pinned to `Plane-4.7.0`.
//!
//! A 3x3 matrix stored as three **row** vectors `a`, `b`, `c` — matching
//! upstream's layout exactly, so index expressions carry over unchanged.
//!
//! # Divergences from upstream
//!
//! Upstream overloads `operator*` for scalar, `Vector3`, and `Matrix3`
//! operands. Rust allows the distinct `Mul<T>`, `Mul<Vector3<T>>` and
//! `Mul<Matrix3<T>>` impls, so all three carry over as operators here.
//!
//! `inverse()` returns `Option<Matrix3<T>>` rather than upstream's
//! `bool` + out-parameter. The semantics are identical — `None` exactly where
//! upstream returns `false`, i.e. when `is_zero(det())` — but the Rust form
//! makes it impossible to read an uninitialised result, which is the failure
//! mode the out-parameter invites. This is a mechanical shape change, not a
//! behavior change, so it stays inside ADR-0003.
//!
//! `to_euler` returns `(roll, pitch, yaw)` rather than taking three nullable
//! out-pointers.

use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

use crate::scalar::{is_zero, safe_asin, Real};
use crate::vector2::Vector2;
use crate::vector3::Vector3;

/// 3x3 matrix of three row vectors. Upstream `Matrix3<T>`.
#[derive(Debug, Clone, Copy, Default)]
#[repr(C)]
pub struct Matrix3<T> {
    /// First row.
    pub a: Vector3<T>,
    /// Second row.
    pub b: Vector3<T>,
    /// Third row.
    pub c: Vector3<T>,
}

/// Upstream `Matrix3f`.
pub type Matrix3f = Matrix3<f32>;
/// Upstream `Matrix3d`.
pub type Matrix3d = Matrix3<f64>;

impl<T: Real> Matrix3<T> {
    /// Construct from three row vectors.
    #[inline]
    pub fn from_rows(a: Vector3<T>, b: Vector3<T>, c: Vector3<T>) -> Self {
        Self { a, b, c }
    }

    /// Construct from nine scalars in row-major order.
    #[allow(clippy::too_many_arguments)]
    #[inline]
    pub fn new(ax: T, ay: T, az: T, bx: T, by: T, bz: T, cx: T, cy: T, cz: T) -> Self {
        Self {
            a: Vector3::new(ax, ay, az),
            b: Vector3::new(bx, by, bz),
            c: Vector3::new(cx, cy, cz),
        }
    }

    /// The zero matrix. Upstream `zero()`.
    #[inline]
    pub fn zero() -> Self {
        Self {
            a: Vector3::zero(),
            b: Vector3::zero(),
            c: Vector3::zero(),
        }
    }

    /// Set every element to zero, in place.
    #[inline]
    pub fn set_zero(&mut self) {
        *self = Self::zero();
    }

    /// The identity matrix. Upstream `identity()`.
    #[inline]
    pub fn identity() -> Self {
        let (o, z) = (T::one(), T::zero());
        Self::new(o, z, z, z, o, z, z, z, o)
    }

    /// Set to the identity, in place. Upstream `identity()`.
    #[inline]
    pub fn set_identity(&mut self) {
        *self = Self::identity();
    }

    /// The x column. Upstream `colx()`.
    #[inline]
    pub fn colx(self) -> Vector3<T> {
        Vector3::new(self.a.x, self.b.x, self.c.x)
    }

    /// The y column. Upstream `coly()`.
    #[inline]
    pub fn coly(self) -> Vector3<T> {
        Vector3::new(self.a.y, self.b.y, self.c.y)
    }

    /// The z column. Upstream `colz()`.
    #[inline]
    pub fn colz(self) -> Vector3<T> {
        Vector3::new(self.a.z, self.b.z, self.c.z)
    }

    /// Transposed copy. Upstream `transposed()`.
    #[inline]
    pub fn transposed(self) -> Self {
        Self::from_rows(self.colx(), self.coly(), self.colz())
    }

    /// Transpose in place. Upstream `transpose()`.
    #[inline]
    pub fn transpose(&mut self) {
        *self = self.transposed();
    }

    /// Determinant. Upstream `det()`.
    #[inline]
    pub fn det(self) -> T {
        self.a.x * (self.b.y * self.c.z - self.b.z * self.c.y)
            + self.a.y * (self.b.z * self.c.x - self.b.x * self.c.z)
            + self.a.z * (self.b.x * self.c.y - self.b.y * self.c.x)
    }

    /// Matrix inverse, or `None` when the determinant is zero.
    ///
    /// Upstream `inverse()` returns `bool` and writes through an out-parameter;
    /// `None` here corresponds exactly to its `false`, which it returns when
    /// `is_zero(det())`. Note that uses the `FLT_EPSILON` threshold from
    /// [`is_zero`], not an exact zero test, so near-singular matrices are
    /// rejected too.
    #[inline]
    pub fn inverse(self) -> Option<Self> {
        let d = self.det();
        if is_zero(d) {
            return None;
        }
        let (a, b, c) = (self.a, self.b, self.c);
        Some(Self::new(
            (b.y * c.z - c.y * b.z) / d,
            (a.z * c.y - a.y * c.z) / d,
            (a.y * b.z - a.z * b.y) / d,
            (b.z * c.x - b.x * c.z) / d,
            (a.x * c.z - a.z * c.x) / d,
            (b.x * a.z - a.x * b.z) / d,
            (b.x * c.y - c.x * b.y) / d,
            (c.x * a.y - a.x * c.y) / d,
            (a.x * b.y - b.x * a.y) / d,
        ))
    }

    /// Invert in place, returning false and leaving `self` untouched when the
    /// matrix is singular. Upstream `invert()`.
    #[inline]
    pub fn invert(&mut self) -> bool {
        match self.inverse() {
            Some(inv) => {
                *self = inv;
                true
            }
            None => false,
        }
    }

    /// Multiply the transpose of this matrix by a vector.
    /// Upstream `mul_transpose()`.
    #[inline]
    pub fn mul_transpose(self, v: Vector3<T>) -> Vector3<T> {
        Vector3::new(
            self.a.x * v.x + self.b.x * v.y + self.c.x * v.z,
            self.a.y * v.x + self.b.y * v.y + self.c.y * v.z,
            self.a.z * v.x + self.b.z * v.y + self.c.z * v.z,
        )
    }

    /// Multiply by a vector, keeping only the xy components.
    /// Upstream `mulXY()`.
    #[inline]
    pub fn mul_xy(self, v: Vector3<T>) -> Vector2<T> {
        Vector2::new(
            self.a.x * v.x + self.a.y * v.y + self.a.z * v.z,
            self.b.x * v.x + self.b.y * v.y + self.b.z * v.z,
        )
    }

    /// Build a rotation matrix from euler angles in radians.
    ///
    /// Upstream `from_euler()`, a 321 (yaw-pitch-roll) sequence.
    #[inline]
    pub fn from_euler(roll: T, pitch: T, yaw: T) -> Self {
        let cp = pitch.cos();
        let sp = pitch.sin();
        let sr = roll.sin();
        let cr = roll.cos();
        let sy = yaw.sin();
        let cy = yaw.cos();

        Self::new(
            cp * cy,
            (sr * sp * cy) - (cr * sy),
            (cr * sp * cy) + (sr * sy),
            cp * sy,
            (sr * sp * sy) + (cr * cy),
            (cr * sp * sy) - (sr * cy),
            -sp,
            sr * cp,
            cr * cp,
        )
    }

    /// Extract euler angles as `(roll, pitch, yaw)` in radians.
    ///
    /// Upstream `to_euler()`, which writes through nullable out-pointers.
    /// Pitch uses [`safe_asin`], so a matrix slightly outside the valid domain
    /// clamps rather than producing NaN.
    #[inline]
    pub fn to_euler(self) -> (T, T, T) {
        let pitch = -safe_asin(self.c.x);
        let roll = self.c.y.atan2(self.c.z);
        let yaw = self.b.x.atan2(self.a.x);
        (roll, pitch, yaw)
    }

    /// True if any element is NaN. Upstream `is_nan()`.
    #[inline]
    /// Apply a small-angle rotation, upstream `Matrix3::rotate`.
    ///
    /// This is the first-order update the DCM estimator integrates with: each
    /// row is crossed with the rotation vector and the result added, which is
    /// `M += M x g` written out. It is deliberately not a proper rotation --
    /// the result is not orthonormal, and DCM renormalises afterwards to put
    /// that right. Using an exact rotation here would be slower and would not
    /// remove the need to renormalise, because the drift being corrected comes
    /// from accumulated rounding rather than from this approximation.
    pub fn rotate(&mut self, g: Vector3<T>) {
        let delta = Self {
            a: Vector3::new(
                self.a.y * g.z - self.a.z * g.y,
                self.a.z * g.x - self.a.x * g.z,
                self.a.x * g.y - self.a.y * g.x,
            ),
            b: Vector3::new(
                self.b.y * g.z - self.b.z * g.y,
                self.b.z * g.x - self.b.x * g.z,
                self.b.x * g.y - self.b.y * g.x,
            ),
            c: Vector3::new(
                self.c.y * g.z - self.c.z * g.y,
                self.c.z * g.x - self.c.x * g.z,
                self.c.x * g.y - self.c.y * g.x,
            ),
        };
        self.a += delta.a;
        self.b += delta.b;
        self.c += delta.c;
    }

    /// True if any component is NaN, upstream `Matrix3::is_nan`.
    pub fn is_nan(self) -> bool {
        self.a.is_nan() || self.b.is_nan() || self.c.is_nan()
    }
}

/// Row-wise equality, inheriting [`Vector3`]'s epsilon-based comparison.
impl<T: Real> PartialEq for Matrix3<T> {
    #[inline]
    fn eq(&self, m: &Self) -> bool {
        self.a == m.a && self.b == m.b && self.c == m.c
    }
}

impl<T: Real> Add for Matrix3<T> {
    type Output = Self;
    #[inline]
    fn add(self, m: Self) -> Self {
        Self::from_rows(self.a + m.a, self.b + m.b, self.c + m.c)
    }
}

impl<T: Real> Sub for Matrix3<T> {
    type Output = Self;
    #[inline]
    fn sub(self, m: Self) -> Self {
        Self::from_rows(self.a - m.a, self.b - m.b, self.c - m.c)
    }
}

impl<T: Real> Neg for Matrix3<T> {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::from_rows(-self.a, -self.b, -self.c)
    }
}

impl<T: Real> Mul<T> for Matrix3<T> {
    type Output = Self;
    #[inline]
    fn mul(self, num: T) -> Self {
        Self::from_rows(self.a * num, self.b * num, self.c * num)
    }
}

impl<T: Real> Div<T> for Matrix3<T> {
    type Output = Self;
    #[inline]
    fn div(self, num: T) -> Self {
        Self::from_rows(self.a / num, self.b / num, self.c / num)
    }
}

/// Matrix times column vector. Upstream `operator*(const Vector3<T>&)`.
impl<T: Real> Mul<Vector3<T>> for Matrix3<T> {
    type Output = Vector3<T>;
    #[inline]
    fn mul(self, v: Vector3<T>) -> Vector3<T> {
        Vector3::new(self.a.dot(v), self.b.dot(v), self.c.dot(v))
    }
}

/// Matrix product. Upstream `operator*(const Matrix3<T>&)`.
impl<T: Real> Mul<Matrix3<T>> for Matrix3<T> {
    type Output = Self;
    #[inline]
    fn mul(self, m: Self) -> Self {
        let (cx, cy, cz) = (m.colx(), m.coly(), m.colz());
        Self::new(
            self.a.dot(cx),
            self.a.dot(cy),
            self.a.dot(cz),
            self.b.dot(cx),
            self.b.dot(cy),
            self.b.dot(cz),
            self.c.dot(cx),
            self.c.dot(cy),
            self.c.dot(cz),
        )
    }
}

impl<T: Real> AddAssign for Matrix3<T> {
    #[inline]
    fn add_assign(&mut self, m: Self) {
        *self = *self + m;
    }
}

impl<T: Real> SubAssign for Matrix3<T> {
    #[inline]
    fn sub_assign(&mut self, m: Self) {
        *self = *self - m;
    }
}

impl<T: Real> MulAssign<T> for Matrix3<T> {
    #[inline]
    fn mul_assign(&mut self, num: T) {
        *self = *self * num;
    }
}

impl<T: Real> MulAssign<Matrix3<T>> for Matrix3<T> {
    #[inline]
    fn mul_assign(&mut self, m: Self) {
        *self = *self * m;
    }
}

impl<T: Real> DivAssign<T> for Matrix3<T> {
    #[inline]
    fn div_assign(&mut self, num: T) {
        *self = *self / num;
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;
    use core::f32::consts::PI;

    // Cases ported from upstream libraries/AP_Math/tests/test_matrix3.cpp
    // at Plane-4.7.0.
    //
    // Note on upstream naming: the array literally called `invertible[]`
    // holds the det == 0 matrix (which is NOT invertible), and
    // `non_invertible[]` holds the det == +/-732 matrices (which ARE).
    // The INSTANTIATE_TEST_CASE_P names are swapped the same way. The test
    // logic branches on det == 0 so it still checks the right thing; only
    // the labels are wrong. Names below describe the actual property.

    /// Upstream's `non_invertible[]` entries, which are in fact invertible.
    fn invertible_cases() -> [(Matrix3f, f32); 2] {
        [
            (
                Matrix3f::new(6.0, 2.0, 20.0, 1.0, -9.0, 4.0, -4.0, 7.0, -27.0),
                732.0,
            ),
            (
                Matrix3f::new(-6.0, -2.0, -20.0, -1.0, 9.0, -4.0, 4.0, -7.0, 27.0),
                -732.0,
            ),
        ]
    }

    /// Upstream's `invertible[]` entry, which is in fact singular.
    fn singular_case() -> (Matrix3f, f32) {
        (
            Matrix3f::new(1.0, 2.0, 3.0, 4.0, 6.0, 2.0, 9.0, 18.0, 27.0),
            0.0,
        )
    }

    fn expect_identity(m: Matrix3f) {
        let e = 1.0e-6;
        assert!((m.a.x - 1.0).abs() < e, "a.x = {}", m.a.x);
        assert!(m.a.y.abs() < e, "a.y = {}", m.a.y);
        assert!(m.a.z.abs() < e, "a.z = {}", m.a.z);
        assert!(m.b.x.abs() < e, "b.x = {}", m.b.x);
        assert!((m.b.y - 1.0).abs() < e, "b.y = {}", m.b.y);
        assert!(m.b.z.abs() < e, "b.z = {}", m.b.z);
        assert!(m.c.x.abs() < e, "c.x = {}", m.c.x);
        assert!(m.c.y.abs() < e, "c.y = {}", m.c.y);
        assert!((m.c.z - 1.0).abs() < e, "c.z = {}", m.c.z);
    }

    /// UPSTREAM-PARITY: TEST_P(Matrix3fTest, Determinants)
    #[test]
    fn determinants_match_upstream() {
        for (m, det) in invertible_cases() {
            assert_eq!(det, m.det());
        }
        let (m, det) = singular_case();
        assert_eq!(det, m.det());
    }

    /// UPSTREAM-PARITY: TEST_P(Matrix3fTest, Inverses)
    #[test]
    fn inverses_match_upstream() {
        for (m, _) in invertible_cases() {
            let inv = m.inverse().expect("should be invertible");
            expect_identity(inv * m);
        }
        // upstream expects inverse() to report failure for the singular case
        let (m, _) = singular_case();
        assert!(m.inverse().is_none());
    }

    /// The singular case must also leave the matrix untouched via invert().
    #[test]
    fn invert_in_place_reports_failure() {
        let (m, _) = singular_case();
        let mut n = m;
        assert!(!n.invert());
        assert_eq!(n, m, "a failed invert must not modify the matrix");

        let (m2, _) = invertible_cases()[0];
        let mut n2 = m2;
        assert!(n2.invert());
        expect_identity(n2 * m2);
    }

    /// PORT-DERIVED: no upstream unit test covers from_euler/to_euler
    /// directly; they are exercised through AHRS instead.
    #[test]
    fn euler_roundtrip_derived() {
        let cases = [
            (0.0_f32, 0.0, 0.0),
            (0.1, 0.2, 0.3),
            (-0.4, 0.5, -1.2),
            (PI / 4.0, -PI / 6.0, PI / 3.0),
        ];
        for (roll, pitch, yaw) in cases {
            let m = Matrix3f::from_euler(roll, pitch, yaw);
            let (r, p, y) = m.to_euler();
            let e = 1.0e-5;
            assert!((r - roll).abs() < e, "roll {roll} -> {r}");
            assert!((p - pitch).abs() < e, "pitch {pitch} -> {p}");
            assert!((y - yaw).abs() < e, "yaw {yaw} -> {y}");
        }
    }

    /// PORT-DERIVED: a rotation matrix is orthonormal, so its inverse equals
    /// its transpose. Cross-checks from_euler, inverse and transposed at once.
    #[test]
    fn rotation_inverse_equals_transpose_derived() {
        let m = Matrix3f::from_euler(0.3, -0.2, 1.1);
        let inv = m.inverse().expect("rotation matrices are invertible");
        let t = m.transposed();
        let e = 1.0e-5;
        for (i, j) in [
            (inv.a.x, t.a.x),
            (inv.a.y, t.a.y),
            (inv.a.z, t.a.z),
            (inv.b.x, t.b.x),
            (inv.b.y, t.b.y),
            (inv.b.z, t.b.z),
            (inv.c.x, t.c.x),
            (inv.c.y, t.c.y),
            (inv.c.z, t.c.z),
        ] {
            assert!((i - j).abs() < e, "{i} vs {j}");
        }
        // determinant of a rotation matrix is 1
        assert!((m.det() - 1.0).abs() < e);
    }

    /// PORT-DERIVED: structural operations with no dedicated upstream test.
    #[test]
    fn structure_and_products_derived() {
        let m = Matrix3f::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);

        assert_eq!(m.colx(), Vector3::new(1.0, 4.0, 7.0));
        assert_eq!(m.coly(), Vector3::new(2.0, 5.0, 8.0));
        assert_eq!(m.colz(), Vector3::new(3.0, 6.0, 9.0));

        assert_eq!(
            m.transposed(),
            Matrix3f::new(1.0, 4.0, 7.0, 2.0, 5.0, 8.0, 3.0, 6.0, 9.0)
        );
        assert_eq!(m.transposed().transposed(), m);

        // identity is a left and right unit
        assert_eq!(m * Matrix3f::identity(), m);
        assert_eq!(Matrix3f::identity() * m, m);

        // matrix times vector uses rows; mul_transpose uses columns
        let v = Vector3::new(1.0, 0.0, 0.0);
        assert_eq!(m * v, Vector3::new(1.0, 4.0, 7.0));
        assert_eq!(m.mul_transpose(v), Vector3::new(1.0, 2.0, 3.0));
        assert_eq!(m.mul_xy(v), Vector2::new(1.0, 4.0));

        // scalar arithmetic
        assert_eq!(
            m * 2.0,
            Matrix3f::new(2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0)
        );
        assert_eq!((m * 2.0) / 2.0, m);
        assert_eq!(m + m, m * 2.0);
        assert_eq!(m - m, Matrix3f::zero());
        assert_eq!(-m, m * -1.0);

        assert!(Matrix3f::new(f32::NAN, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).is_nan());
        assert!(!m.is_nan());
    }
}
