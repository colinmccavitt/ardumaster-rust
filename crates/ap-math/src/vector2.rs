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
//! - [`Vector2::normalized`] returns `Option`, diverging from upstream, which
//!   divides by `length()` unguarded and yields NaN. Registered as **D-002**
//!   in DIVERGENCES.md per ADR-0007.

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

    /// Normalize in place, returning false for a zero-length vector.
    ///
    /// DIVERGENCE D-002 - see DIVERGENCES.md. Upstream has no zero guard and
    /// produces NaN components, which then propagate silently.
    #[inline]
    pub fn normalize(&mut self) -> bool {
        match self.normalized() {
            Some(n) => {
                *self = n;
                true
            }
            None => false,
        }
    }

    /// The normalized vector, or `None` when it has zero length.
    ///
    /// DIVERGENCE D-002 - see DIVERGENCES.md. Upstream `vector2.cpp` divides
    /// by `length()` unguarded, so a zero vector yields NaN. Upstream is
    /// inconsistent with itself here: `QuaternionT::normalize` does guard the
    /// zero case. Surfacing it in the type is the clearest reason to port to
    /// Rust at all.
    #[inline]
    pub fn normalized(self) -> Option<Self> {
        let len = self.length();
        if is_zero(len) {
            return None;
        }
        Some(self / len)
    }

    /// The normalized vector, or the zero vector when it has zero length.
    ///
    /// For callers that want the lenient shape without NaN propagation.
    #[inline]
    pub fn normalized_or_zero(self) -> Self {
        self.normalized().unwrap_or_else(Self::zero)
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

impl<T: Real> Vector2<T> {
    /// Where two line segments cross, upstream `segment_intersection`.
    ///
    /// Returns `None` when the segments are parallel, collinear, or simply do
    /// not meet. Upstream returns a `bool` and writes through an out-parameter,
    /// leaving it untouched on failure; `Option` says the same thing without
    /// the caller having to know that.
    pub fn segment_intersection(
        seg1_start: Self,
        seg1_end: Self,
        seg2_start: Self,
        seg2_end: Self,
    ) -> Option<Self> {
        let r1 = seg1_end - seg1_start;
        let r2 = seg2_end - seg2_start;
        let ss2_ss1 = seg2_start - seg1_start;
        let r1xr2 = r1.cross(r2);
        let q_pxr = ss2_ss1.cross(r1);

        if is_zero(r1xr2) {
            // collinear, or parallel and non-intersecting
            return None;
        }

        let t = ss2_ss1.cross(r2) / r1xr2;
        let u = q_pxr / r1xr2;
        if u >= T::zero() && u <= T::one() && t >= T::zero() && t <= T::one() {
            Some(seg1_start + r1 * t)
        } else {
            None
        }
    }

    /// The point on segment `v`..`w` closest to `p`, upstream `closest_point`.
    ///
    /// A degenerate segment (`v == w`) returns `v`.
    pub fn closest_point(p: Self, v: Self, w: Self) -> Self {
        let l2 = (v - w).length_squared();
        if l2 < T::from_f64(crate::scalar::FLT_EPSILON) {
            return v;
        }
        // projection of p onto the line through v and w, clamped to the segment
        let t = (p - v).dot(w - v) / l2;
        if t <= T::zero() {
            v
        } else if t >= T::one() {
            w
        } else {
            v + (w - v) * t
        }
    }

    /// The point on the segment from the origin to `w` closest to `p`.
    ///
    /// Upstream's two-argument `closest_point`, a simplification of the
    /// three-argument form with `v` at the origin. Note it returns `w` rather
    /// than the origin for a degenerate segment, where the three-argument form
    /// returns `v` — the two agree, since there `v == w`.
    pub fn closest_point_radial(p: Self, w: Self) -> Self {
        let l2 = w.length_squared();
        if l2 < T::from_f64(crate::scalar::FLT_EPSILON) {
            return w;
        }
        let t = p.dot(w) / l2;
        if t <= T::zero() {
            Self::zero()
        } else if t >= T::one() {
            w
        } else {
            w * t
        }
    }

    /// Squared distance from the segment origin..`w` to point `p`.
    pub fn closest_distance_between_radial_and_point_squared(w: Self, p: Self) -> T {
        (Self::closest_point_radial(p, w) - p).length_squared()
    }

    /// Squared distance from segment `w1`..`w2` to point `p`.
    pub fn closest_distance_between_line_and_point_squared(w1: Self, w2: Self, p: Self) -> T {
        Self::closest_distance_between_radial_and_point_squared(w2 - w1, p - w1)
    }

    /// Distance from segment `w1`..`w2` to point `p`.
    pub fn closest_distance_between_line_and_point(w1: Self, w2: Self, p: Self) -> T {
        Self::closest_distance_between_line_and_point_squared(w1, w2, p).sqrt()
    }

    /// Squared distance between two line segments.
    ///
    /// The minimum over each endpoint against the opposite segment. That is
    /// exact only when the segments do not cross; upstream accepts this,
    /// because its callers test for crossing separately before asking for a
    /// distance.
    pub fn closest_distance_between_lines_squared(a1: Self, a2: Self, b1: Self, b2: Self) -> T {
        let d1 = Self::closest_distance_between_line_and_point_squared(b1, b2, a1);
        let d2 = Self::closest_distance_between_line_and_point_squared(b1, b2, a2);
        let d3 = Self::closest_distance_between_line_and_point_squared(a1, a2, b1);
        let d4 = Self::closest_distance_between_line_and_point_squared(a1, a2, b2);
        let m1 = if d1 < d2 { d1 } else { d2 };
        let m2 = if d3 < d4 { d3 } else { d4 };
        if m1 < m2 {
            m1
        } else {
            m2
        }
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
    ///
    /// Values match upstream; only the return shape differs, per D-002.
    #[test]
    fn normalized_matches_upstream() {
        let mut v = Vector2f::new(3.0, 3.0);
        assert!(v.normalize());
        assert_eq!(Vector2f::new(3.0, 3.0).normalized().unwrap(), v);

        let r = libm::sqrtf(2.0) / 2.0;
        assert_eq!(
            Vector2f::new(r, r),
            Vector2f::new(5.0, 5.0).normalized().unwrap()
        );
        assert_eq!(
            Vector2f::new(3.0, 3.0).normalized().unwrap(),
            Vector2f::new(5.0, 5.0).normalized().unwrap()
        );
        assert_eq!(
            Vector2f::new(-3.0, 3.0).normalized().unwrap(),
            Vector2f::new(-5.0, 5.0).normalized().unwrap()
        );
        assert_ne!(
            Vector2f::new(-3.0, 3.0).normalized().unwrap(),
            Vector2f::new(5.0, 5.0).normalized().unwrap()
        );
    }

    /// DIVERGENCE D-002, pinned.
    ///
    /// UPSTREAM: `vector2.cpp` computes `*this / length()` with no zero guard,
    /// so a zero vector yields NaN components that propagate silently.
    /// PORTED: `None`, with `normalize()` reporting false and leaving the
    /// value untouched.
    ///
    /// Do not "restore parity" here - the NaN behavior is the defect.
    #[test]
    fn d002_normalized_zero_is_none() {
        assert!(Vector2f::zero().normalized().is_none());

        let mut z = Vector2f::zero();
        assert!(!z.normalize());
        assert!(z.is_zero(), "a failed normalize must not modify the value");
        assert!(!z.is_nan(), "the upstream NaN must not appear");

        // lenient helper for callers that want the old shape without NaN
        assert_eq!(Vector2f::zero().normalized_or_zero(), Vector2f::zero());
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
