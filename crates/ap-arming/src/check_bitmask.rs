//! Pre-4.7 `ARMING_CHECK` enable bitmask. FW-026.
//!
//! Upstream stored `ARMING_CHECK` as `checks_to_perform`: a bit *set* means
//! that named check runs. Bit 0 is `ALL` and enables every named check.
//! `0` disables them all. Plane-4.7.0 inverted this to `ARMING_SKIPCHK`
//! (`checks_to_skip`); this module is the old polarity plus that conversion.
//!
//! The shared registry still walks `ARMING_SKIPCHK`. Callers that still
//! speak `ARMING_CHECK` convert here, then hand the result to [`Arming`].

use crate::{Arming, Check, Required, CHECK_MASK};

/// Upstream `ARMING_CHECK` / `Check::ALL` — bit 0, enable every named check.
pub const ARMING_CHECK_ALL: u32 = 1;

/// Default `ARMING_CHECK` on pre-4.7 Plane (`ARMING_CHECK_ALL`).
pub const ARMING_CHECK_DEFAULT: u32 = ARMING_CHECK_ALL;

/// Upstream `check_enabled` against a stored `ARMING_CHECK` value.
///
/// `ALL` (bit 0) enables every named check. Otherwise the check runs only
/// when its own bit is set. `0` enables nothing.
#[must_use]
pub const fn arming_check_enabled(checks_to_perform: u32, check: Check) -> bool {
    if (checks_to_perform & ARMING_CHECK_ALL) != 0 {
        return true;
    }
    (checks_to_perform & check.as_u32()) != 0
}

/// Upstream `should_skip_all_checks` for a stored `ARMING_CHECK` value.
#[must_use]
pub const fn arming_check_skips_all(checks_to_perform: u32) -> bool {
    checks_to_perform == 0
}

/// Convert a stored pre-4.7 `ARMING_CHECK` value to `ARMING_SKIPCHK`.
///
/// Mirrors `AP_Arming::init` `PARAM_CONVERSION - 4.7 CHECK -> SKIPCHK`:
/// * `0` → skip every current and future check (`-1` / all bits)
/// * `ALL` set → skip nothing
/// * otherwise invert the known named-check bits
#[must_use]
pub const fn skipchk_from_arming_check(checks_to_perform: u32) -> u32 {
    if checks_to_perform == 0 {
        return u32::MAX;
    }
    if (checks_to_perform & ARMING_CHECK_ALL) == 0 {
        return (!checks_to_perform) & CHECK_MASK;
    }
    0
}

/// Build [`Arming`] from `ARMING_REQUIRE` and a pre-4.7 `ARMING_CHECK` mask.
#[must_use]
pub const fn arming_from_check(require: Required, checks_to_perform: u32) -> Arming {
    Arming {
        require,
        checks_to_skip: skipchk_from_arming_check(checks_to_perform),
        armed: false,
    }
}
