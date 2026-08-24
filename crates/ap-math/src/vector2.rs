//! Port of `AP_Math/vector2.{h,cpp}`, pinned to `Plane-4.7.0`.
//!
//! # Deliberate divergence from upstream
//!
//! Upstream overloads `operator*` for **both** scalar multiplication and the
//! dot product, and `operator%` for the 2D cross product. Rust cannot express
//! that pair on one trait, so:
//!
//! | upstream | here |
//! |---|---|
//! | `a * b` (both Vector2) | `a.dot(b)` |
//! | `a % b` | `a.cross(b)` |
//! | `a * s` (scalar) | `a * s` |
//!
//! This is the one place Vector2 call sites will not diff line-for-line against
//! the C++. Everything else keeps upstream naming.
//!
//! # Faithfully reproduced behavior
//!
//! - `==` compares with [`is_equal`] per component, **not** exact equality
//!   (`vector2.cpp:133`). Two vectors differing by less than an epsilon are
//!   equal, so `Vector2` is deliberately not `Eq`.
//! - [`Vector2::normalized`] divides by `length()` with **no zero guard**,
//!   matching `vector2.cpp`. A zero vector yields NaN components rather than
//!   an error. ADR-0003 requires reproducing this rather than fixing it.

use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

use crate::scalar::{is_equal, is_positive, is_zero, norm2, radians, Real};

/// Two-dimensional vector. Upstream `Vector2<T>`.
#[derive(Debug, Clone, Copy, Default)]
#[repr(C)]
pub struct Vector2<T> {
    /// X component.
    pub x: T,
    /// Y component.
    pub y: T,
}

/// Upstream `Vector2f`.
pub type Vector2f = Vector2<f32>;
/// Upstream `Vector2d`.
pub type Vector2d = Vector2<f64>;

impl<T: Real> Vector2<T> {
    /// Construct from components. Upstream's setting constructor.
    #[inline]
    pub fn new(x: T, y: T) -> Self {
        Self { x, y }
    }

    /// The zero vector. Upstream's trivial constructor.
    #[inline]
    pub fn zero() -> Self {
        Self {
            x: T::zero(),
            y: T::zero(),
        }
    }

    /// Set both components to zero. Upstream `zero()`.
    #[inline]
    pub fn set_zero(&mut self) {
        self.x = T::zero();
        self.y = T::zero();
    }

    /// Dot product. Upstream `operator*(const Vector2<T>&)` / `dot()`.
    #[inline]
    pub fn dot(self, v: Self) -> T {
        self.x * v.x + self.y * v.y
    }

    /// 2D cross product (a scalar). Upstream `operator%`.
    #[inline]
    pub fn cross(self, v: Self) -> T {
        self.x * v.y - self.y * v.x
    }

    /// Squared length. Upstream `length_squared()`.
    #[inline]
    pub fn length_squared(self) -> T {
        self.x * self.x + self.y * self.y
    }

    /// Length. Upstream `length()`, which routes through `norm(x, y)`.
    #[inline]
    pub fn length(self) -> T {
        norm2(self.x, self.y)
    }

    /// Limit to `max_length`, returning true if the vector was shortened.
    ///
    /// Upstream `vector2.cpp:37`. Note the guard is `len > max && is_positive(len)`,
    /// so a length below `FLT_EPSILON` is never scaled.
    #[inline]
    pub fn limit_length(&mut self, max_length: T) -> bool {
        let len = self.length();
        if len > max_length && is_positive(len) {
            let scale = max_length / len;
            self.x = self.x * scale;
            self.y = self.y * scale;
            return true;
        }
        false
    }

    /// Normalize in place. Upstream `normalize()`.
    #[inline]
    pub fn normalize(&mut self) {
        *self = self.normalized();
    }

    /// Return the normalized vector.
    ///
    /// Upstream `vector2.cpp` divides by `length()` with no zero check, so a
    /// zero vector produces NaN components. Preserved deliberately.
    #[inline]
    pub fn normalized(self) -> Self {
        self / self.length()
    }

    /// Angle between this vector and `v2`, in radians.
    ///
    /// Upstream `vector2.cpp:145`. Returns 0 if either vector has
    /// non-positive length, and clamps the cosine to the valid domain.
    #[inline]
    pub fn angle_to(self, v2: Self) -> T {
        let len = self.length() * v2.length();
        if len <= T::zero() {
            return T::zero();
        }
        let cosv = self.dot(v2) / len;
        if cosv >= T::one() {
            return T::zero();
        }
        if cosv <= -T::one() {
            return T::PI;
        }
        cosv.acos()
    }

    /// Angle of this vector from `(1, 0)`, in radians, over `-pi..pi`.
    ///
    /// Upstream `vector2.cpp:162`, `atan2F(y, x)`.
    #[inline]
    pub fn angle(self) -> T {
        self.y.atan2(self.x)
    }

    /// Rotate by `angle_rad`. Upstream `rotate()`.
    #[inline]
    pub fn rotate(&mut self, angle_rad: T) {
        let cs = angle_rad.cos();
        let sn = angle_rad.sin();
        let rx = self.x * cs - self.y * sn;
        let ry = self.x * sn + self.y * cs;
        self.x = rx;
        self.y = ry;
    }

    /// Project this vector onto `v`. Upstream `project()`.
    #[inline]
    pub fn project(&mut self, v: Self) {
        *self = self.projected(v);
    }

    /// This vector projected onto `v`. Upstream `projected()`.
    #[inline]
    pub fn projected(self, v: Self) -> Self {
        v * (self.dot(v) / v.dot(v))
    }

    /// Reflect this vector about `n`. Upstream `reflect()`.
    #[inline]
    pub fn reflect(&mut self, n: Self) {
        let orig = *self;
        self.project(n);
        *self = *self * (T::one() + T::one()) - orig;
    }

    /// Offset by `distance` along `bearing` degrees. Upstream `offset_bearing()`.
    #[inline]
    pub fn offset_bearing(&mut self, bearing: T, distance: T) {
        let r = radians(bearing);
        self.x = self.x + r.cos() * distance;
        self.y = self.y + r.sin() * distance;
    }

    /// True if either component is NaN. Upstream `is_nan()`.
    #[inline]
    pub fn is_nan(self) -> bool {
        self.x.is_nan() || self.y.is_nan()
    }

    /// True if either component is infinite. Upstream `is_inf()`.
    #[inline]
    pub fn is_inf(self) -> bool {
        self.x.is_infinite() || self.y.is_infinite()
    }

    /// True if both components are zero.
    ///
    /// Upstream specialises this for float and double to use `is_zero()`
    /// rather than exact comparison (`vector2.h`, after the class body).
    #[inline]
    pub fn is_zero(self) -> bool {
        is_zero(self.x) && is_zero(self.y)
    }
}

/// Component-wise equality using [`is_equal`], matching `vector2.cpp:133`.
///
/// Deliberately not `Eq`: the relation is not transitive under an epsilon.
impl<T: Real> PartialEq for Vector2<T> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        is_equal(self.x, other.x) && is_equal(self.y, other.y)
    }
}

impl<T: Real> Add for Vector2<T> {
    type Output = Self;
    #[inline]
    fn add(self, v: Self) -> Self {
        Self::new(self.x + v.x, self.y + v.y)
    }
}

impl<T: Real> Sub for Vector2<T> {
    type Output = Self;
    #[inline]
    fn sub(self, v: Self) -> Self {
        Self::new(self.x - v.x, self.y - v.y)
    }
}

impl<T: Real> Neg for Vector2<T> {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y)
    }
}

impl<T: Real> Mul<T> for Vector2<T> {
    type Output = Self;
    #[inline]
    fn mul(self, num: T) -> Self {
        Self::new(self.x * num, self.y * num)
    }
}

impl<T: Real> Div<T> for Vector2<T> {
    type Output = Self;
    #[inline]
    fn div(self, num: T) -> Self {
        Self::new(self.x / num, self.y / num)
    }
}

impl<T: Real> AddAssign for Vector2<T> {
    #[inline]
    fn add_assign(&mut self, v: Self) {
        *self = *self + v;
    }
}

impl<T: Real> SubAssign for Vector2<T> {
    #[inline]
    fn sub_assign(&mut self, v: Self) {
        *self = *self - v;
    }
}

impl<T: Real> MulAssign<T> for Vector2<T> {
    #[inline]
    fn mul_assign(&mut self, num: T) {
        *self = *self * num;
    }
}

impl<T: Real> DivAssign<T> for Vector2<T> {
    #[inline]
    fn div_assign(&mut self, num: T) {
        *self = *self / num;
    }
}

#[cfg(test)]
mod tests {
    // assert_eq! on Vector2 uses the epsilon-based PartialEq above, which is
    // exactly what upstream EXPECT_EQ does for these types.
    #![allow(clippy::float_cmp)]

    use super::*;
    use crate::scalar::radians;
    use core::f32::consts::PI;

    // Cases ported from upstream libraries/AP_Math/tests/test_vector2.cpp
    // at Plane-4.7.0. Names map to the upstream TEST() they came from.

    fn near(a: f32, b: f32) {
        assert!((a - b).abs() < 1.0e-5, "expected {b}, got {a}");
    }

    /// upstream TEST(Vector2Test, angle)
    #[test]
    fn angle_matches_upstream() {
        near(Vector2f::new(0.0, 1.0).angle(), PI / 2.0);
        near(Vector2f::new(1.0, 1.0).angle(), PI / 4.0);
        assert!(crate::scalar::is_zero(Vector2d::new(1.0, 0.0).angle()));
        near(Vector2f::new(-1.0, -1.0).angle(), -PI * 3.0 / 4.0);
        near(Vector2f::new(-5.0, -5.0).angle(), -PI * 3.0 / 4.0);

        // all cardinal and inter-cardinal points
        near(Vector2f::new(1.0, 0.0).angle(), 0.0);
        near(Vector2f::new(0.0, 1.0).angle(), PI * 2.0 / 4.0);
        near(Vector2f::new(-1.0, 1.0).angle(), PI * 3.0 / 4.0);
        near(Vector2f::new(-1.0, 0.0).angle(), PI);
        near(Vector2f::new(0.0, -1.0).angle(), -PI * 2.0 / 4.0);
        near(Vector2f::new(1.0, -1.0).angle(), -PI * 1.0 / 4.0);

        // angle between two vectors
        near(
            Vector2f::new(0.0, 1.0).angle_to(Vector2f::new(1.0, 0.0)),
            PI / 2.0,
        );
        near(
            Vector2f::new(0.5, 0.5).angle_to(Vector2f::new(0.5, 0.5)),
            0.0,
        );
        // antiparallel clamps to PI rather than producing NaN from acos
        near(
            Vector2f::new(0.5, -0.5).angle_to(Vector2f::new(-0.5, 0.5)),
            PI,
        );
        // zero-length input returns 0, not NaN
        near(
            Vector2f::new(-0.0, 0.0).angle_to(Vector2f::new(0.0, 1.0)),
            0.0,
        );
    }

    /// upstream TEST(Vector2Test, length)
    #[test]
    fn length_matches_upstream() {
        near(Vector2f::new(3.0, 4.0).length_squared(), 25.0);
        near(Vector2f::new(3.0, 4.0).length(), 5.0);

        let mut v = Vector2f::new(1.0, 1.0);
        assert!(v.limit_length(1.0));
        // zero length is not scaled: the is_positive(len) guard rejects it
        assert!(!Vector2f::new(-0.0, 0.0).limit_length(1.0));
    }

    /// upstream TEST(Vector2Test, normalized)
    #[test]
    fn normalized_matches_upstream() {
        let mut v = Vector2f::new(3.0, 3.0);
        v.normalize();
        assert_eq!(Vector2f::new(3.0, 3.0).normalized(), v);

        let r = libm::sqrtf(2.0) / 2.0;
        assert_eq!(Vector2f::new(r, r), Vector2f::new(5.0, 5.0).normalized());
        assert_eq!(
            Vector2f::new(3.0, 3.0).normalized(),
            Vector2f::new(5.0, 5.0).normalized()
        );
        assert_eq!(
            Vector2f::new(-3.0, 3.0).normalized(),
            Vector2f::new(-5.0, 5.0).normalized()
        );
        assert_ne!(
            Vector2f::new(-3.0, 3.0).normalized(),
            Vector2f::new(5.0, 5.0).normalized()
        );
    }

    /// Upstream divides by length with no zero guard, so this is NaN rather
    /// than an error or a zero vector. Reproduced deliberately (ADR-0003).
    #[test]
    fn normalized_zero_vector_is_nan_like_upstream() {
        assert!(Vector2f::zero().normalized().is_nan());
    }

    /// upstream TEST(Vector2Test, Project)
    #[test]
    fn project_matches_upstream() {
        let mut a = Vector2f::new(1.0, 1.0);
        let b = Vector2f::new(2.0, 1.0);
        a.project(b);
        assert_eq!(Vector2f::new(1.0, 1.0).projected(b), a);
    }

    /// upstream TEST(Vector2Test, reflect)
    #[test]
    fn reflect_and_rotate_match_upstream() {
        let mut r1 = Vector2f::new(3.0, 8.0);
        r1.reflect(Vector2f::new(0.0, 1.0));
        assert_eq!(r1, Vector2f::new(-3.0, 8.0));

        // colinear
        let mut r2 = Vector2f::new(3.0, 3.0);
        r2.reflect(Vector2f::new(1.0, 1.0));
        assert_eq!(r2, Vector2f::new(3.0, 3.0));

        // orthogonal
        let mut r3 = Vector2f::new(3.0, 3.0);
        r3.reflect(Vector2f::new(1.0, -1.0));
        assert_eq!(r3, Vector2f::new(-3.0, -3.0));

        // rotation
        let mut base = Vector2f::new(2.0, 1.0);
        base.rotate(radians(90.0_f32));
        near(base.x, -1.0);
        near(base.y, 2.0);
    }

    /// upstream TEST(Vector2Test, Offset_bearing)
    #[test]
    fn offset_bearing_matches_upstream() {
        let mut v = Vector2f::new(1.0, 0.0);
        v.offset_bearing(0.0, 1.0);
        assert_eq!(Vector2f::new(2.0, 0.0), v);
    }

    /// upstream TEST(Vector2Test, Operator), arithmetic arm
    #[test]
    fn operators_match_upstream() {
        let a = Vector2f::new(1.0, 1.0);
        let b = Vector2f::new(2.0, 3.0);
        assert_eq!(a + b, Vector2f::new(3.0, 4.0));
        assert_eq!(b - a, Vector2f::new(1.0, 2.0));
        assert_eq!(a * 2.0, Vector2f::new(2.0, 2.0));
        assert_eq!(b / 2.0, Vector2f::new(1.0, 1.5));
        assert_eq!(-a, Vector2f::new(-1.0, -1.0));
        // dot and cross are methods here, not operator* and operator%
        near(a.dot(b), 5.0);
        near(a.cross(b), 1.0);

        let mut m = a;
        m += b;
        assert_eq!(m, Vector2f::new(3.0, 4.0));
        m -= b;
        assert_eq!(m, a);
        m *= 3.0;
        assert_eq!(m, Vector2f::new(3.0, 3.0));
        m /= 3.0;
        assert_eq!(m, a);
    }

    /// Guards the epsilon-based equality noted in the module docs.
    #[test]
    fn equality_is_epsilon_based_not_exact() {
        let a = Vector2f::new(1.0, 1.0);
        let b = Vector2f::new(1.0 + f32::EPSILON / 2.0, 1.0);
        assert_eq!(a, b);
        assert!(Vector2f::zero().is_zero());
        assert!(Vector2f::new(f32::NAN, 0.0).is_nan());
        assert!(Vector2f::new(f32::INFINITY, 0.0).is_inf());
    }
}
