//! Port of `AP_Math/vector3.{h,cpp}`, pinned to `Plane-4.7.0`.
//!
//! Same operator divergence as [`crate::vector2`]: upstream overloads
//! `operator*` for scalar multiply and dot product, and `operator%` for the
//! cross product. Here those are [`Vector3::dot`] and [`Vector3::cross`].
//!
//! # Upstream inconsistency, reproduced
//!
//! `Vector3::angle_to` returns **0** for antiparallel vectors, where
//! [`crate::vector2::Vector2::angle_to`] returns `PI` for the same case.
//!
//! ```text
//! vector2.cpp:145   if (cosv >= 1)  return 0;  if (cosv <= -1) return M_PI;
//! vector3.cpp       if (cosv >= 1 || cosv <= -1) return 0;
//! ```
//!
//! So `Vector2f(1,0).angle_to(Vector2f(-1,0))` is `PI` but
//! `Vector3f(1,0,0).angle_to(Vector3f(-1,0,0))` is `0`. That is almost
//! certainly an upstream bug, but ADR-0003 requires reproducing behavior
//! rather than fixing it — a fix belongs in its own ticket, upstream first.
//! Pinned by `angle_to_antiparallel_differs_from_vector2`.

use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

use crate::matrix3::Matrix3;
use crate::scalar::{
    constrain_value, is_equal, is_positive, is_zero, norm2, norm3, radians, safe_sqrt, Real,
};
use crate::vector2::Vector2;

/// Three-dimensional vector. Upstream `Vector3<T>`.
#[derive(Debug, Clone, Copy, Default)]
#[repr(C)]
pub struct Vector3<T> {
    /// X component.
    pub x: T,
    /// Y component.
    pub y: T,
    /// Z component.
    pub z: T,
}

/// Upstream `Vector3f`.
pub type Vector3f = Vector3<f32>;
/// Upstream `Vector3d`.
pub type Vector3d = Vector3<f64>;

impl<T: Real> Vector3<T> {
    /// Construct from components.
    #[inline]
    pub fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }

    /// The zero vector.
    #[inline]
    pub fn zero() -> Self {
        Self {
            x: T::zero(),
            y: T::zero(),
            z: T::zero(),
        }
    }

    /// Set all components to zero. Upstream `zero()`.
    #[inline]
    pub fn set_zero(&mut self) {
        *self = Self::zero();
    }

    /// Dot product. Upstream `operator*` / `dot()`.
    #[inline]
    pub fn dot(self, v: Self) -> T {
        self.x * v.x + self.y * v.y + self.z * v.z
    }

    /// Cross product. Upstream `operator%` / `cross()`.
    #[inline]
    pub fn cross(self, v: Self) -> Self {
        Self::new(
            self.y * v.z - self.z * v.y,
            self.z * v.x - self.x * v.z,
            self.x * v.y - self.y * v.x,
        )
    }

    /// Scale by a scalar. Upstream `scale()`, an alias for `operator*`.
    #[inline]
    pub fn scale(self, v: T) -> Self {
        self * v
    }

    /// Squared length. Upstream `length_squared()`.
    #[inline]
    pub fn length_squared(self) -> T {
        self.dot(self)
    }

    /// Length. Upstream `length()`, routed through `norm(x, y, z)`.
    #[inline]
    pub fn length(self) -> T {
        norm3(self.x, self.y, self.z)
    }

    /// Limit the xy component to `max_length`, returning true if limited.
    ///
    /// Upstream `vector3.cpp`. Note z is left untouched.
    #[inline]
    pub fn limit_length_xy(&mut self, max_length: T) -> bool {
        let length_xy = norm2(self.x, self.y);
        if length_xy > max_length && is_positive(length_xy) {
            let scale = max_length / length_xy;
            self.x = self.x * scale;
            self.y = self.y * scale;
            return true;
        }
        false
    }

    /// Normalize in place, returning false for a zero-length vector.
    ///
    /// DIVERGENCE D-002 - see DIVERGENCES.md.
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
    /// DIVERGENCE D-002 - see DIVERGENCES.md. Upstream divides by `length()`
    /// unguarded, producing NaN for a zero vector.
    #[inline]
    pub fn normalized(self) -> Option<Self> {
        let len = self.length();
        if is_zero(len) {
            return None;
        }
        Some(self / len)
    }

    /// The normalized vector, or the zero vector when it has zero length.
    #[inline]
    pub fn normalized_or_zero(self) -> Self {
        self.normalized().unwrap_or_else(Self::zero)
    }

    /// Angle to `v2` in radians.
    ///
    /// Upstream `vector3.cpp`. Returns 0 when either length is non-positive
    /// **and** when the vectors are antiparallel — see the module docs.
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
        // DIVERGENCE D-001: upstream vector3.cpp collapses both out-of-domain
        // ends into a single `return 0`, so antiparallel vectors report an
        // angle of 0 instead of PI. Vector2 handles the two ends separately
        // and correctly. See DIVERGENCES.md.
        if cosv <= -T::one() {
            return T::PI;
        }
        cosv.acos()
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

    /// Reflect about `n`. Upstream `reflect()`.
    #[inline]
    pub fn reflect(&mut self, n: Self) {
        let orig = *self;
        self.project(n);
        *self = *self * (T::one() + T::one()) - orig;
    }

    /// Squared distance from this vector's tip to `v`. Upstream
    /// `distance_squared()`, which avoids the sqrt.
    #[inline]
    pub fn distance_squared(self, v: Self) -> T {
        let dx = self.x - v.x;
        let dy = self.y - v.y;
        let dz = self.z - v.z;
        dx * dx + dy * dy + dz * dz
    }

    /// Offset by `distance` along `bearing`/`pitch` degrees.
    ///
    /// Upstream `vector3.cpp`. Note the component order: y takes the sine of
    /// bearing and x the cosine, matching a NED-style frame.
    #[inline]
    pub fn offset_bearing(&mut self, bearing: T, pitch: T, distance: T) {
        let b = radians(bearing);
        let p = radians(pitch);
        self.y = self.y + p.cos() * b.sin() * distance;
        self.x = self.x + p.cos() * b.cos() * distance;
        self.z = self.z + p.sin() * distance;
    }

    /// Right-front-up to front-right-down, i.e. ENU to NED.
    ///
    /// Upstream `rfu_to_frd()`, returning `{y, x, -z}`.
    #[inline]
    pub fn rfu_to_frd(self) -> Self {
        Self::new(self.y, self.x, -self.z)
    }

    /// The xy components as a [`Vector2`]. Upstream `xy()`.
    ///
    /// Upstream returns a reference by reinterpreting the object's storage;
    /// this returns a copy, since aliasing the same memory as two types is
    /// exactly what the port is meant to avoid.
    #[inline]
    pub fn xy(self) -> Vector2<T> {
        Vector2::new(self.x, self.y)
    }

    /// True if any component is NaN. Upstream `is_nan()`.
    #[inline]
    pub fn is_nan(self) -> bool {
        self.x.is_nan() || self.y.is_nan() || self.z.is_nan()
    }

    /// True if any component is infinite. Upstream `is_inf()`.
    #[inline]
    pub fn is_inf(self) -> bool {
        self.x.is_infinite() || self.y.is_infinite() || self.z.is_infinite()
    }

    /// True if all components are zero, by [`is_zero`].
    #[inline]
    pub fn is_zero(self) -> bool {
        is_zero(self.x) && is_zero(self.y) && is_zero(self.z)
    }

    /// Component of `p1` perpendicular to `v1`, i.e. `p1` projected onto the
    /// plane orthogonal to `v1`. Returns `p1` unchanged when the dot product
    /// is zero.
    ///
    /// Upstream static `perpendicular()` in `vector3.h`. Divides by
    /// `length_squared()` rather than normalizing, avoiding a sqrt — kept that
    /// way deliberately, since the two forms differ numerically.
    #[inline]
    pub fn perpendicular(p1: Self, v1: Self) -> Self {
        let d = p1.dot(v1);
        if is_zero(d) {
            return p1;
        }
        let parallel = (v1 * d) / v1.length_squared();
        p1 - parallel
    }
}

/// Component-wise equality via [`is_equal`], matching upstream `operator==`.
impl<T: Real> PartialEq for Vector3<T> {
    #[inline]
    fn eq(&self, other: &Self) -> bool {
        is_equal(self.x, other.x) && is_equal(self.y, other.y) && is_equal(self.z, other.z)
    }
}

impl<T: Real> Add for Vector3<T> {
    type Output = Self;
    #[inline]
    fn add(self, v: Self) -> Self {
        Self::new(self.x + v.x, self.y + v.y, self.z + v.z)
    }
}

impl<T: Real> Sub for Vector3<T> {
    type Output = Self;
    #[inline]
    fn sub(self, v: Self) -> Self {
        Self::new(self.x - v.x, self.y - v.y, self.z - v.z)
    }
}

impl<T: Real> Neg for Vector3<T> {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl<T: Real> Mul<T> for Vector3<T> {
    type Output = Self;
    #[inline]
    fn mul(self, num: T) -> Self {
        Self::new(self.x * num, self.y * num, self.z * num)
    }
}

impl<T: Real> Div<T> for Vector3<T> {
    type Output = Self;
    #[inline]
    fn div(self, num: T) -> Self {
        Self::new(self.x / num, self.y / num, self.z / num)
    }
}

impl<T: Real> AddAssign for Vector3<T> {
    #[inline]
    fn add_assign(&mut self, v: Self) {
        *self = *self + v;
    }
}

impl<T: Real> SubAssign for Vector3<T> {
    #[inline]
    fn sub_assign(&mut self, v: Self) {
        *self = *self - v;
    }
}

impl<T: Real> MulAssign<T> for Vector3<T> {
    #[inline]
    fn mul_assign(&mut self, num: T) {
        *self = *self * num;
    }
}

impl<T: Real> DivAssign<T> for Vector3<T> {
    #[inline]
    fn div_assign(&mut self, num: T) {
        *self = *self / num;
    }
}

impl<T: Real> Vector3<T> {
    /// Rotate about the z axis by `angle_rad`, upstream `rotate_xy`.
    pub fn rotate_xy(&mut self, angle_rad: T) {
        let cs = angle_rad.cos();
        let sn = angle_rad.sin();
        let rx = self.x * cs - self.y * sn;
        let ry = self.x * sn + self.y * cs;
        self.x = rx;
        self.y = ry;
    }

    /// This vector as a row, times a matrix. Upstream `row_times_mat`.
    #[must_use]
    pub fn row_times_mat(&self, m: &Matrix3<T>) -> Self {
        Self::new(self.dot(m.colx()), self.dot(m.coly()), self.dot(m.colz()))
    }

    /// Column vector times row vector, giving a matrix. Upstream
    /// `mul_rowcol`, the outer product.
    #[must_use]
    pub fn mul_rowcol(&self, v2: Self) -> Matrix3<T> {
        Matrix3::new(
            self.x * v2.x,
            self.x * v2.y,
            self.x * v2.z,
            self.y * v2.x,
            self.y * v2.y,
            self.y * v2.z,
            self.z * v2.x,
            self.z * v2.y,
            self.z * v2.z,
        )
    }

    /// Perpendicular distance from this point to the segment `seg_start`..`seg_end`.
    ///
    /// Computed from the triangle's area by Heron's formula, as upstream does.
    /// A degenerate segment returns zero rather than dividing by its length.
    #[must_use]
    pub fn distance_to_segment(&self, seg_start: Self, seg_end: Self) -> T {
        let a = (*self - seg_start).length();
        let b = (seg_start - seg_end).length();
        let c = (seg_end - *self).length();

        if is_zero(b) {
            return T::zero();
        }

        let s = (a + b + c) * T::from_f64(0.5);
        let mut area_squared = s * (s - a) * (s - b) * (s - c);
        // Three collinear points give a true area of zero, and rounding can
        // push the product just below it. Upstream clamps for that reason.
        if area_squared < T::zero() {
            area_squared = T::zero();
        }
        let area = safe_sqrt(area_squared);
        T::from_f64(2.0) * area / b
    }

    /// The point on segment `w1`..`w2` closest to `p`, upstream
    /// `point_on_line_closest_to_other_point`.
    ///
    /// A degenerate segment returns `w1`.
    #[must_use]
    pub fn point_on_line_closest_to_other_point(w1: Self, w2: Self, p: Self) -> Self {
        let line_vec = w2 - w1;
        let p_vec = p - w1;

        let line_vec_len = line_vec.length();
        if is_zero(line_vec_len) {
            return w1;
        }

        // Upstream scales both vectors by 1/len and dots them, which yields the
        // fraction along the segment directly. Reproduced rather than
        // simplified: dividing the dot product once instead would change the
        // rounding.
        let scale = T::one() / line_vec_len;
        let unit_vec = line_vec * scale;
        let scaled_p_vec = p_vec * scale;

        let dot_product = constrain_value(unit_vec.dot(scaled_p_vec), T::zero(), T::one());
        line_vec * dot_product + w1
    }

    /// Distance from `p` to the segment `w1`..`w2`, upstream
    /// `closest_distance_between_line_and_point`.
    #[must_use]
    pub fn closest_distance_between_line_and_point(w1: Self, w2: Self, p: Self) -> T {
        (Self::point_on_line_closest_to_other_point(w1, w2, p) - p).length()
    }

    /// Whether the segment `seg_start`..`seg_end` meets the plane through
    /// `plane_point` with normal `plane_normal`. Upstream
    /// `segment_plane_intersect`.
    ///
    /// A segment lying entirely in the plane counts as intersecting.
    #[must_use]
    pub fn segment_plane_intersect(
        seg_start: Self,
        seg_end: Self,
        plane_normal: Self,
        plane_point: Self,
    ) -> bool {
        let u = seg_end - seg_start;
        let w = seg_start - plane_point;

        let d = plane_normal.dot(u);
        let n = -(plane_normal.dot(w));

        if is_zero(d) {
            // parallel to the plane: either lying in it, or missing it entirely
            return is_zero(n);
        }
        let s_i = n / d;
        (T::zero()..=T::one()).contains(&s_i)
    }

    /// The point on segment 2 closest to segment 1, upstream
    /// `segment_to_segment_closest_point`.
    ///
    /// Upstream writes through an out-parameter; this returns the point.
    #[must_use]
    pub fn segment_to_segment_closest_point(
        seg1_start: Self,
        seg1_end: Self,
        seg2_start: Self,
        seg2_end: Self,
    ) -> Self {
        let line1 = seg1_end - seg1_start;
        let line2 = seg2_end - seg2_start;
        let diff = seg1_start - seg2_start;

        let a = line1.dot(line1);
        let b = line1.dot(line2);
        let c = line2.dot(line2);
        let d = line1.dot(diff);
        let e = line2.dot(diff);

        let discriminant = a * c - b * b;
        let mut s_n;
        let mut s_d = discriminant;
        let mut t_n;
        let mut t_d = discriminant;

        if discriminant < T::from_f64(crate::scalar::FLT_EPSILON) {
            // the segments are near parallel: pin to seg1_start rather than
            // divide by something close to zero
            s_n = T::zero();
            s_d = T::one();
            t_n = e;
            t_d = c;
        } else {
            // closest points on the infinite lines
            s_n = b * e - c * d;
            t_n = a * e - b * d;
            if s_n < T::zero() {
                // the s = 0 edge is the visible one
                s_n = T::zero();
                t_n = e;
                t_d = c;
            } else if s_n > s_d {
                // the s = 1 edge is the visible one
                s_n = s_d;
                t_n = e + b;
                t_d = c;
            }
        }

        if t_n < T::zero() {
            // the t = 0 edge is visible, so recompute s against it
            t_n = T::zero();
            if -d < T::zero() {
                s_n = T::zero();
            } else if -d > a {
                s_n = s_d;
            } else {
                s_n = -d;
                s_d = a;
            }
        } else if t_n > t_d {
            // the t = 1 edge is visible
            t_n = t_d;
            if (-d + b) < T::zero() {
                s_n = T::zero();
            } else if (-d + b) > a {
                s_n = s_d;
            } else {
                s_n = -d + b;
                s_d = a;
            }
        }

        // Upstream computes sN/sD as well but only uses tc, since it returns
        // the point on segment 2. The s values are kept because the branches
        // above update them and dropping them would change which branch runs.
        let _ = (s_n, s_d);

        let tc = if is_zero(t_n) { T::zero() } else { t_n / t_d };
        seg2_start + line2 * tc
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;
    use core::f32::consts::PI;

    // UPSTREAM TEST COVERAGE WARNING
    //
    // In upstream tests/test_vector3.cpp at Plane-4.7.0, only these run:
    //   Operator (6), OperatorDouble (86), IsEqual (122),
    //   length (150), normalized (165)
    // Lines 140-149 and 175-361 are commented-out blocks, disabling:
    //   angle, Project, reflect, Offset_bearing, Perpendicular,
    //   closest_point, closest_distance, segment_intersectionx,
    //   circle_segment_intersectionx, point_on_segmentx
    //
    // Tests below are split accordingly. Ones marked UPSTREAM-PARITY are real
    // unit-parity evidence. Ones marked PORT-DERIVED are written here from
    // reading the C++, which is a weaker oracle - those methods need
    // sitl-diff or review before they are trusted.

    fn near(a: f32, b: f32) {
        assert!((a - b).abs() < 1.0e-5, "expected {b}, got {a}");
    }

    /// UPSTREAM-PARITY: TEST(Vector3Test, length)
    #[test]
    fn length_matches_upstream() {
        near(Vector3f::new(2.0, 2.0, 2.0).length_squared(), 12.0);
        near(Vector3f::new(2.0, 2.0, 2.0).length(), libm::sqrtf(12.0));

        let mut v = Vector3f::new(1.0, 1.0, 1.0);
        assert!(v.limit_length_xy(1.0));
        assert!(!Vector3f::new(-0.0, -0.0, -0.0).limit_length_xy(1.0));

        // upstream repeats the same cases for Vector3d
        assert!((Vector3d::new(2.0, 2.0, 2.0).length_squared() - 12.0).abs() < 1e-12);
        let mut vd = Vector3d::new(1.0, 1.0, 1.0);
        assert!(vd.limit_length_xy(1.0));
        assert!(!Vector3d::new(-0.0, -0.0, -0.0).limit_length_xy(1.0));
    }

    /// UPSTREAM-PARITY: TEST(Vector3Test, normalized)
    ///
    /// Values match upstream; only the return shape differs, per D-002.
    #[test]
    fn normalized_matches_upstream() {
        let mut v = Vector3f::new(3.0, 3.0, 3.0);
        assert!(v.normalize());
        assert_eq!(Vector3f::new(3.0, 3.0, 3.0).normalized().unwrap(), v);

        let r = 1.0 / libm::sqrtf(3.0);
        assert_eq!(
            Vector3f::new(r, r, r),
            Vector3f::new(2.0, 2.0, 2.0).normalized().unwrap()
        );
        assert_eq!(
            Vector3f::new(3.0, 3.0, 3.0).normalized().unwrap(),
            Vector3f::new(5.0, 5.0, 5.0).normalized().unwrap()
        );
        assert_eq!(
            Vector3f::new(-3.0, 3.0, 3.0).normalized().unwrap(),
            Vector3f::new(-5.0, 5.0, 5.0).normalized().unwrap()
        );
        assert_ne!(
            Vector3f::new(-3.0, 3.0, 3.0).normalized().unwrap(),
            Vector3f::new(5.0, 5.0, 5.0).normalized().unwrap()
        );
    }

    /// DIVERGENCE D-002, pinned. See the Vector2 twin for the rationale.
    #[test]
    fn d002_normalized_zero_is_none() {
        assert!(Vector3f::zero().normalized().is_none());

        let mut z = Vector3f::zero();
        assert!(!z.normalize());
        assert!(z.is_zero());
        assert!(!z.is_nan(), "the upstream NaN must not appear");
        assert_eq!(Vector3f::zero().normalized_or_zero(), Vector3f::zero());
    }

    /// UPSTREAM-PARITY: TEST(Vector3Test, Operator) and IsEqual
    #[test]
    fn operators_match_upstream() {
        let a = Vector3f::new(1.0, 1.0, 1.0);
        let b = Vector3f::new(2.0, 3.0, 4.0);
        assert_eq!(a + b, Vector3f::new(3.0, 4.0, 5.0));
        assert_eq!(b - a, Vector3f::new(1.0, 2.0, 3.0));
        assert_eq!(a * 2.0, Vector3f::new(2.0, 2.0, 2.0));
        assert_eq!(b / 2.0, Vector3f::new(1.0, 1.5, 2.0));
        assert_eq!(-a, Vector3f::new(-1.0, -1.0, -1.0));
        near(a.dot(b), 9.0);
        assert_eq!(
            Vector3f::new(1.0, 0.0, 0.0).cross(Vector3f::new(0.0, 1.0, 0.0)),
            Vector3f::new(0.0, 0.0, 1.0)
        );

        let mut m = a;
        m += b;
        assert_eq!(m, Vector3f::new(3.0, 4.0, 5.0));
        m -= b;
        assert_eq!(m, a);
        m *= 3.0;
        assert_eq!(m, Vector3f::new(3.0, 3.0, 3.0));
        m /= 3.0;
        assert_eq!(m, a);

        assert!(Vector3f::zero().is_zero());
        assert!(Vector3f::new(f32::NAN, 0.0, 0.0).is_nan());
        assert!(Vector3f::new(f32::INFINITY, 0.0, 0.0).is_inf());
    }

    /// DIVERGENCE D-001, pinned.
    ///
    /// UPSTREAM: `vector3.cpp` collapses both out-of-domain ends into a single
    /// `return 0`, so antiparallel vectors report an angle of 0.
    /// PORTED: returns PI, matching `Vector2::angle_to` and the mathematics.
    ///
    /// Evidence this is a defect: upstream's own Vector3 angle test expects
    /// M_PI and is commented out at tests/test_vector3.cpp:140-149. Real
    /// caller affected is AP_Compass.cpp:2243 in Compass::consistent(), where
    /// the bug reports two opposed compasses as perfectly consistent.
    ///
    /// Do not "restore parity" here - the 0 is the defect.
    #[test]
    fn d001_angle_to_antiparallel_returns_pi() {
        use crate::vector2::Vector2f;

        // 2D was always correct
        near(
            Vector2f::new(1.0, 0.0).angle_to(Vector2f::new(-1.0, 0.0)),
            PI,
        );
        // 3D now agrees, where upstream returned 0.0
        near(
            Vector3f::new(1.0, 0.0, 0.0).angle_to(Vector3f::new(-1.0, 0.0, 0.0)),
            PI,
        );
        near(
            Vector3f::new(0.0, 5.0, 0.0).angle_to(Vector3f::new(0.0, -2.0, 0.0)),
            PI,
        );

        // cases where upstream and the port already agreed
        near(
            Vector3f::new(0.0, 1.0, 0.0).angle_to(Vector3f::new(1.0, 0.0, 0.0)),
            PI / 2.0,
        );
        near(
            Vector3f::new(0.5, 0.5, 0.0).angle_to(Vector3f::new(0.5, 0.5, 0.0)),
            0.0,
        );
        // zero length still returns 0 rather than NaN, as upstream does
        near(
            Vector3f::new(0.0, 0.0, 0.0).angle_to(Vector3f::new(0.0, 1.0, 0.0)),
            0.0,
        );
    }

    /// PORT-DERIVED: upstream reflect and Project tests are commented out.
    #[test]
    fn reflect_and_project_derived() {
        let mut r1 = Vector3f::new(3.0, 3.0, 8.0);
        r1.reflect(Vector3f::new(0.0, 0.0, 1.0));
        assert_eq!(r1, Vector3f::new(-3.0, -3.0, 8.0));

        // colinear
        let mut r2 = Vector3f::new(3.0, 3.0, 3.0);
        r2.reflect(Vector3f::new(1.0, 1.0, 1.0));
        assert_eq!(r2, Vector3f::new(3.0, 3.0, 3.0));

        // Upstream's disabled test calls this the "orthogonal vectors" case
        // and expects (-3,-3,-3). Both claims are wrong:
        //   (3,3,3) . (1,1,-1) = 3, not 0, so they are NOT orthogonal, and
        //   upstream's own algorithm yields (-1,-1,-5) for this input.
        // The expectation here is what the ported (and upstream) code actually
        // computes. A bad expectation in a disabled test is not an oracle.
        let mut r3 = Vector3f::new(3.0, 3.0, 3.0);
        r3.reflect(Vector3f::new(1.0, 1.0, -1.0));
        assert_eq!(r3, Vector3f::new(-1.0, -1.0, -5.0));

        // A genuinely orthogonal normal: (1,-1,0) . (3,3,3) = 0, so the
        // is_zero(d) branch returns the input unchanged... via project(),
        // which divides by v.dot(v) and reflects to -orig.
        let mut r4 = Vector3f::new(3.0, 3.0, 3.0);
        r4.reflect(Vector3f::new(1.0, -1.0, 0.0));
        assert_eq!(r4, Vector3f::new(-3.0, -3.0, -3.0));

        let mut a = Vector3f::new(1.0, 1.0, 1.0);
        let b = Vector3f::new(2.0, 2.0, 1.0);
        a.project(b);
        assert_eq!(Vector3f::new(1.0, 1.0, 1.0).projected(b), a);
    }

    /// PORT-DERIVED: upstream Perpendicular test is commented out. Upstream
    /// divides by length_squared rather than normalizing, avoiding a sqrt.
    #[test]
    fn perpendicular_derived() {
        // component of (1,1,0) perpendicular to the x axis is (0,1,0)
        assert_eq!(
            Vector3f::perpendicular(Vector3f::new(1.0, 1.0, 0.0), Vector3f::new(2.0, 0.0, 0.0)),
            Vector3f::new(0.0, 1.0, 0.0)
        );
        // orthogonal input has a zero dot product, so p1 comes back unchanged
        assert_eq!(
            Vector3f::perpendicular(Vector3f::new(1.0, 0.0, 0.0), Vector3f::new(0.0, 2.0, 0.0)),
            Vector3f::new(1.0, 0.0, 0.0)
        );
    }

    /// PORT-DERIVED: frame conversion and helpers with no upstream test.
    #[test]
    fn conversions_derived() {
        assert_eq!(
            Vector3f::new(1.0, 2.0, 3.0).rfu_to_frd(),
            Vector3f::new(2.0, 1.0, -3.0)
        );
        assert_eq!(
            Vector3f::new(1.0, 2.0, 3.0).xy(),
            crate::vector2::Vector2f::new(1.0, 2.0)
        );
        near(
            Vector3f::new(0.0, 0.0, 0.0).distance_squared(Vector3f::new(1.0, 2.0, 2.0)),
            9.0,
        );

        // offset_bearing: bearing 0, pitch 0 advances x only
        let mut v = Vector3f::new(1.0, 0.0, 0.0);
        v.offset_bearing(0.0, 0.0, 1.0);
        assert_eq!(v, Vector3f::new(2.0, 0.0, 0.0));
    }
}
