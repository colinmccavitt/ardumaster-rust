//! Small float helpers shared across the TECS modules.
//!
//! Deliberately **not** `f32::min`/`f32::max`. Those follow IEEE-754
//! `minNum`/`maxNum` semantics, which return the non-NaN operand when one side
//! is NaN. Upstream's `MIN`/`MAX` are macros expanding to a plain comparison,
//! so a NaN operand propagates differently. Since TECS runs on values that can
//! legitimately go NaN under sensor failure, the comparison form is what gets
//! reproduced.

/// Larger of two values, by plain comparison. Upstream `MAX`.
#[inline]
pub(crate) fn max_f32(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}

/// Smaller of two values, by plain comparison. Upstream `MIN`.
#[inline]
pub(crate) fn min_f32(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}

#[cfg(test)]
mod tests {
    // exact comparison is the property under test
    #![allow(clippy::float_cmp)]

    use super::*;

    #[test]
    fn ordinary_comparison() {
        assert_eq!(max_f32(2.0, 1.0), 2.0);
        assert_eq!(max_f32(1.0, 2.0), 2.0);
        assert_eq!(min_f32(2.0, 1.0), 1.0);
        assert_eq!(min_f32(1.0, 2.0), 1.0);
    }

    /// A plain comparison is false against NaN, so the SECOND operand is
    /// returned - unlike f32::max, which would return the non-NaN side. This
    /// matches the C++ macro and is the reason these exist.
    #[test]
    fn nan_follows_the_comparison_not_ieee_min_max() {
        assert!(max_f32(f32::NAN, 1.0) == 1.0);
        assert!(min_f32(f32::NAN, 1.0) == 1.0);
        // and NaN as the second operand propagates
        assert!(max_f32(1.0, f32::NAN).is_nan());
        assert!(min_f32(1.0, f32::NAN).is_nan());
        // whereas the std versions would not
        assert_eq!(1.0_f32.max(f32::NAN), 1.0);
    }
}
