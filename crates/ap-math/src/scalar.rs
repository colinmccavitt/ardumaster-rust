//! Scalar helpers, ported from `AP_Math/AP_Math.{h,cpp}` and `AP_Math/ftype.h`.
//!
//! Upstream expresses these as C++ templates with per-type overloads. Here they
//! are free functions over the [`Real`] trait, keeping upstream's names so call
//! sites stay diffable against the C++.
//!
//! # Faithfully reproduced quirks
//!
//! Upstream's epsilon choices are not self-consistent, and ADR-0003 requires
//! reproducing behavior rather than improving it:
//!
//! - [`is_zero`] compares against **`FLT_EPSILON` for both `f32` and `f64`**
//!   (`ftype.h:63,70`). A double is *not* compared against `DBL_EPSILON`.
//! - [`is_equal`] compares against the epsilon of the *common type*, so
//!   `f64` pairs use `DBL_EPSILON` (`AP_Math.cpp:32`) while `f32` pairs use
//!   `FLT_EPSILON` (`AP_Math.cpp:39`).
//!
//! So `is_zero(1e-10_f64)` is `true` but `is_equal(1e-10_f64, 0.0_f64)` is
//! `false`. That is upstream behavior and is deliberately preserved.

use crate::Ftype;

/// Upstream `FLT_EPSILON`, used by the `is_*` predicates regardless of width.
pub const FLT_EPSILON: f64 = f32::EPSILON as f64;

/// Float types the scalar helpers operate on. Mirrors upstream's template
/// instantiations for `float` and `double`.
pub trait Real: Copy + PartialOrd {
    /// Epsilon of this type, as `std::numeric_limits<T>::epsilon()`.
    const EPSILON: Self;
    /// Pi.
    const PI: Self;
    /// Two pi, upstream `M_2PI`.
    const TWO_PI: Self;
    /// Half pi, upstream `M_PI_2`.
    const FRAC_PI_2: Self;

    /// Absolute value.
    fn abs(self) -> Self;
    /// Square root.
    fn sqrt(self) -> Self;
    /// Arcsine.
    fn asin(self) -> Self;
    /// Floating point remainder, upstream `fmodf`/`fmod`.
    fn fmod(self, rhs: Self) -> Self;
    /// True if this is NaN.
    fn is_nan(self) -> bool;
    /// Convert from an `f64` literal.
    fn from_f64(v: f64) -> Self;
    /// Convert to `f64`.
    fn to_f64(self) -> f64;
    /// Zero.
    fn zero() -> Self;
}

impl Real for f32 {
    const EPSILON: Self = f32::EPSILON;
    const PI: Self = core::f32::consts::PI;
    const TWO_PI: Self = core::f32::consts::PI * 2.0;
    const FRAC_PI_2: Self = core::f32::consts::FRAC_PI_2;

    #[inline]
    fn abs(self) -> Self {
        libm::fabsf(self)
    }
    #[inline]
    fn sqrt(self) -> Self {
        libm::sqrtf(self)
    }
    #[inline]
    fn asin(self) -> Self {
        libm::asinf(self)
    }
    #[inline]
    fn fmod(self, rhs: Self) -> Self {
        libm::fmodf(self, rhs)
    }
    #[inline]
    fn is_nan(self) -> bool {
        // Inherent f32::is_nan, available in core; resolves ahead of this
        // trait method, so this is not recursive.
        <f32>::is_nan(self)
    }
    #[inline]
    fn from_f64(v: f64) -> Self {
        v as f32
    }
    #[inline]
    fn to_f64(self) -> f64 {
        self as f64
    }
    #[inline]
    fn zero() -> Self {
        0.0
    }
}

impl Real for f64 {
    const EPSILON: Self = f64::EPSILON;
    const PI: Self = core::f64::consts::PI;
    const TWO_PI: Self = core::f64::consts::PI * 2.0;
    const FRAC_PI_2: Self = core::f64::consts::FRAC_PI_2;

    #[inline]
    fn abs(self) -> Self {
        libm::fabs(self)
    }
    #[inline]
    fn sqrt(self) -> Self {
        libm::sqrt(self)
    }
    #[inline]
    fn asin(self) -> Self {
        libm::asin(self)
    }
    #[inline]
    fn fmod(self, rhs: Self) -> Self {
        libm::fmod(self, rhs)
    }
    #[inline]
    fn is_nan(self) -> bool {
        <f64>::is_nan(self)
    }
    #[inline]
    fn from_f64(v: f64) -> Self {
        v
    }
    #[inline]
    fn to_f64(self) -> f64 {
        self
    }
    #[inline]
    fn zero() -> Self {
        0.0
    }
}

/// Whether a float is zero, within `FLT_EPSILON`.
///
/// Upstream `ftype.h:63` (float) and `ftype.h:70` (double). Note both compare
/// against `FLT_EPSILON` — see the module docs.
#[inline]
pub fn is_zero<T: Real>(v: T) -> bool {
    v.abs().to_f64() < FLT_EPSILON
}

/// Whether a float is greater than zero, i.e. `>= FLT_EPSILON`.
///
/// Upstream `AP_Math.h:65`. Note this is not `> 0`: values in
/// `(0, FLT_EPSILON)` are neither positive nor negative by this definition.
#[inline]
pub fn is_positive<T: Real>(v: T) -> bool {
    v.to_f64() >= FLT_EPSILON
}

/// Whether a float is less than zero, i.e. `<= -FLT_EPSILON`.
///
/// Upstream `AP_Math.h:76`.
#[inline]
pub fn is_negative<T: Real>(v: T) -> bool {
    v.to_f64() <= -FLT_EPSILON
}

/// Whether two floats are equal, within the epsilon of their own type.
///
/// Upstream `AP_Math.cpp:26`. Unlike [`is_zero`], this uses `T::EPSILON`, so
/// `f64` comparisons use `DBL_EPSILON`.
#[inline]
pub fn is_equal<T: Real>(a: T, b: T) -> bool {
    let d = a.to_f64() - b.to_f64();
    let d = if d < 0.0 { -d } else { d };
    d < T::EPSILON.to_f64()
}

/// `sqrt` that returns 0 for negative or NaN input.
///
/// Upstream `AP_Math.cpp:72`, which uses `isgreaterequal` specifically so NaN
/// returns false and falls through to 0.
#[inline]
pub fn safe_sqrt<T: Real>(v: T) -> T {
    if v.is_nan() || v < T::zero() {
        return T::zero();
    }
    v.sqrt()
}

/// `asin` clamped to a valid domain; NaN input returns 0.
///
/// Upstream `AP_Math.cpp:50`.
#[inline]
pub fn safe_asin<T: Real>(v: T) -> T {
    if v.is_nan() {
        return T::zero();
    }
    let one = T::from_f64(1.0);
    if v >= one {
        return T::FRAC_PI_2;
    }
    if v <= T::from_f64(-1.0) {
        return T::from_f64(-T::FRAC_PI_2.to_f64());
    }
    v.asin()
}

/// Wrap an angle in degrees to `[0, 360)`.
///
/// Upstream `AP_Math.cpp:184`.
#[inline]
pub fn wrap_360<T: Real>(angle: T) -> T {
    let mut res = angle.fmod(T::from_f64(360.0));
    if res < T::zero() {
        res = T::from_f64(res.to_f64() + 360.0);
    }
    res
}

/// Wrap an angle in degrees to `(-180, 180]`.
///
/// Upstream `AP_Math.cpp:150`.
#[inline]
pub fn wrap_180<T: Real>(angle: T) -> T {
    let res = wrap_360(angle);
    if res > T::from_f64(180.0) {
        return T::from_f64(res.to_f64() - 360.0);
    }
    res
}

/// Wrap an angle in radians to `[0, 2*pi)`.
///
/// Upstream `AP_Math.cpp:251`.
#[inline]
pub fn wrap_2pi<T: Real>(radian: T) -> T {
    let mut res = radian.fmod(T::TWO_PI);
    if res < T::zero() {
        res = T::from_f64(res.to_f64() + T::TWO_PI.to_f64());
    }
    res
}

/// Wrap an angle in radians to `(-pi, pi]`.
///
/// Upstream `AP_Math.cpp:270`.
#[inline]
pub fn wrap_pi<T: Real>(radian: T) -> T {
    let res = wrap_2pi(radian);
    if res > T::PI {
        return T::from_f64(res.to_f64() - T::TWO_PI.to_f64());
    }
    res
}

/// Constrain a value to `[low, high]`, mapping NaN to the midpoint.
///
/// Upstream `AP_Math.cpp:283`. The NaN case is deliberate: it stops floating
/// point errors propagating through every consumer of `constrain_value`.
/// Upstream also raises an internal error there, which the port does not yet
/// have an equivalent for — tracked as a follow-up, not silently dropped.
#[inline]
#[allow(clippy::manual_clamp)] // clamp() propagates NaN; upstream returns the midpoint
pub fn constrain_value<T: Real>(amt: T, low: T, high: T) -> T {
    if amt.is_nan() {
        return T::from_f64((low.to_f64() + high.to_f64()) / 2.0);
    }
    if amt < low {
        return low;
    }
    if amt > high {
        return high;
    }
    amt
}

/// Degrees to radians. Upstream `AP_Math.h:227`.
#[inline]
pub fn radians(deg: Ftype) -> Ftype {
    deg * (core::f64::consts::PI / 180.0) as Ftype
}

/// Radians to degrees. Upstream `AP_Math.h:255`.
#[inline]
pub fn degrees(rad: Ftype) -> Ftype {
    rad * (180.0 / core::f64::consts::PI) as Ftype
}

/// Square of a value. Upstream `AP_Math.h:261`.
#[inline]
pub fn sq(v: Ftype) -> Ftype {
    v * v
}

/// Pythagorean norm of two components. Upstream's variadic `norm()`.
#[inline]
pub fn norm2(a: Ftype, b: Ftype) -> Ftype {
    safe_sqrt(sq(a) + sq(b))
}

/// Pythagorean norm of three components.
#[inline]
pub fn norm3(a: Ftype, b: Ftype, c: Ftype) -> Ftype {
    safe_sqrt(sq(a) + sq(b) + sq(c))
}

#[cfg(test)]
mod tests {
    // Upstream asserts exact equality with EXPECT_EQ on these cases, so the
    // port does too. Tolerance-based checks use near() where upstream used
    // EXPECT_NEAR.
    #![allow(clippy::float_cmp, clippy::manual_clamp)]

    use super::*;

    // Cases below are ported from upstream
    // libraries/AP_Math/tests/test_math.cpp at Plane-4.7.0.
    // Test names map to the upstream TEST() they came from, so a future
    // reader can diff them against the C++ directly.

    const ACCURACY: f64 = 1.0e-5;

    fn near(a: f64, b: f64) {
        assert!((a - b).abs() < ACCURACY, "expected {b}, got {a}");
    }

    /// upstream TEST(MathTest, IsZero)
    #[test]
    fn is_zero_matches_upstream() {
        assert!(!is_zero(0.1_f32));
        assert!(!is_zero(0.0001_f32));
        assert!(is_zero(0.0_f32));
        // FLT_MIN is ~1.18e-38, far below FLT_EPSILON, so it counts as zero.
        assert!(is_zero(f32::MIN_POSITIVE));
        assert!(is_zero(-f32::MIN_POSITIVE));
    }

    /// upstream TEST(MathTest, IsPositive)
    #[test]
    fn is_positive_matches_upstream() {
        assert!(is_positive(1.0_f32));
        assert!(is_positive(f32::EPSILON));
        assert!(!is_positive(0.0_f32));
        assert!(!is_positive(-1.0_f32));
    }

    /// upstream TEST(MathTest, IsNegative)
    #[test]
    fn is_negative_matches_upstream() {
        assert!(is_negative(-f32::EPSILON));
        assert!(is_negative(-1.0_f32));
        assert!(!is_negative(0.0_f32));
        assert!(!is_negative(1.0_f32));
    }

    /// upstream TEST(MathTest, IsEqual)
    #[test]
    fn is_equal_matches_upstream() {
        assert!(!is_equal(0.1_f64, 0.10001_f64));
        assert!(!is_equal(0.1_f64, -0.1001_f64));
        assert!(is_equal(0.0_f32, 0.0_f32));
        assert!(!is_equal(1.0_f32, 1.0_f32 + f32::EPSILON));
        assert!(is_equal(1.0_f32, 1.0_f32 + f32::EPSILON / 2.0));
        // upstream: "false because the common type is double"
        assert!(!is_equal(1.0_f64, 1.0 + 2.0 * f64::EPSILON));
        // upstream: "true because the common type is float"
        assert!(is_equal(1.0_f32, (1.0_f64 + f64::EPSILON) as f32));
    }

    /// Guards the module-level note: is_zero uses FLT_EPSILON even for f64,
    /// while is_equal uses that type's own epsilon. Upstream is inconsistent
    /// here and ADR-0003 requires reproducing it rather than fixing it.
    #[test]
    fn f64_epsilon_inconsistency_is_preserved() {
        assert!(is_zero(1e-10_f64));
        assert!(!is_equal(1e-10_f64, 0.0_f64));
    }

    /// upstream TEST(MathWrapTest, Angle360)
    #[test]
    fn wrap_360_matches_upstream() {
        assert_eq!(45.0_f32, wrap_360(45.0_f32));
        assert_eq!(90.0_f32, wrap_360(90.0_f32));
        assert_eq!(180.0_f32, wrap_360(180.0_f32));
        assert_eq!(270.0_f32, wrap_360(270.0_f32));
        assert_eq!(0.0_f32, wrap_360(360.0_f32));
        assert_eq!(1.0_f32, wrap_360(361.0_f32));
        assert_eq!(0.0_f32, wrap_360(720.0_f32));
        assert_eq!(0.0_f32, wrap_360(3600.0_f32));
        assert_eq!(0.0_f32, wrap_360(7200.0_f32));
        assert_eq!(260.0_f32, wrap_360(-100.0_f32));
    }

    /// upstream TEST(MathWrapTest, Angle180)
    #[test]
    fn wrap_180_matches_upstream() {
        assert_eq!(45.0_f32, wrap_180(45.0_f32));
        assert_eq!(90.0_f32, wrap_180(90.0_f32));
        assert_eq!(180.0_f32, wrap_180(180.0_f32));
        assert_eq!(-179.9_f32, wrap_180(180.1_f32));
        assert_eq!(-90.0_f32, wrap_180(270.0_f32));
        assert_eq!(0.0_f32, wrap_180(360.0_f32));
        assert_eq!(-45.0_f32, wrap_180(-45.0_f32));
        assert_eq!(180.0_f32, wrap_180(-180.0_f32));
    }

    /// upstream TEST(MathWrapTest, AnglePI)
    #[test]
    fn wrap_pi_matches_upstream() {
        use core::f64::consts::PI;
        near(wrap_pi(PI), PI);
        near(wrap_pi(2.0 * PI), 0.0);
        near(wrap_pi(PI * 10.0), 0.0);
        near(wrap_pi(PI + 1.0), -2.141_592_502_593_994);
        near(wrap_pi(1.0_f64), 1.0);
    }

    /// upstream TEST(MathWrapTest, Angle2PI)
    #[test]
    fn wrap_2pi_matches_upstream() {
        use core::f64::consts::PI;
        near(wrap_2pi(0.0_f64), 0.0);
        near(wrap_2pi(PI * 2.0), 0.0);
        near(wrap_2pi(-PI * 2.0), 0.0);
        near(wrap_2pi(PI), PI);
    }

    /// upstream TEST(MathTest, ASin)
    #[test]
    fn safe_asin_matches_upstream() {
        use core::f32::consts::{FRAC_PI_2, PI};
        near(safe_asin(0.0_f32) as f64, 0.0);
        near(safe_asin(FRAC_PI_2 * 0.5) as f64, 0.903_339_110_766_512_7);
        // out of domain clamps rather than returning NaN
        near(safe_asin(FRAC_PI_2) as f64, FRAC_PI_2 as f64);
        near(safe_asin(PI) as f64, FRAC_PI_2 as f64);
        near(safe_asin(2.0 * PI) as f64, FRAC_PI_2 as f64);
        near(safe_asin(-FRAC_PI_2) as f64, -FRAC_PI_2 as f64);
        near(safe_asin(-PI) as f64, -FRAC_PI_2 as f64);
        near(safe_asin(-FRAC_PI_2 * 0.5) as f64, -0.903_339_110_766_512_7);
        near(safe_asin(f32::NAN) as f64, 0.0);
    }

    /// upstream TEST(MathTest, Sqrt)
    #[test]
    fn safe_sqrt_matches_upstream() {
        near(safe_sqrt(4.0_f32) as f64, 2.0);
        near(safe_sqrt(0.0_f32) as f64, 0.0);
        // negative and NaN both fall through to zero rather than NaN
        near(safe_sqrt(-1.0_f32) as f64, 0.0);
        near(safe_sqrt(f32::NAN) as f64, 0.0);
    }

    /// upstream TEST(MathTest, Constrain), float arm
    #[test]
    fn constrain_value_matches_upstream() {
        for i in 0..1000 {
            let v = i as f32;
            let expected = if i < 250 {
                250.0
            } else if i > 500 {
                500.0
            } else {
                v
            };
            assert_eq!(expected, constrain_value(v, 250.0_f32, 500.0_f32));
        }
        for i in 0..=1000 {
            let c = (i - 1000) as f32;
            let expected = if c < -250.0 {
                -250.0
            } else if c > -50.0 {
                -50.0
            } else {
                c
            };
            assert_eq!(expected, constrain_value(c, -250.0_f32, -50.0_f32));
        }
    }

    /// Upstream maps NaN to the midpoint so float errors stop propagating.
    #[test]
    fn constrain_value_nan_returns_midpoint() {
        assert_eq!(375.0_f32, constrain_value(f32::NAN, 250.0_f32, 500.0_f32));
    }

    /// upstream TEST(MathTest, Square) and TEST(MathTest, Norm)
    #[test]
    fn sq_and_norm_match_upstream() {
        near(sq(0.0) as f64, 0.0);
        near(sq(1.0) as f64, 1.0);
        near(sq(2.0) as f64, 4.0);
        near(norm2(3.0, 4.0) as f64, 5.0);
        near(norm3(2.0, 3.0, 6.0) as f64, 7.0);
    }

    #[test]
    fn degrees_radians_roundtrip() {
        near(radians(180.0) as f64, core::f64::consts::PI);
        near(degrees(core::f32::consts::PI as Ftype) as f64, 180.0);
    }
}
