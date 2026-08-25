//! The per-function default output shape.
//!
//! Generated from `aux_servo_function_setup`, so the risk is not a typo but a
//! misparse — a case-range read as two entries, or a group whose `set_range`
//! got attached to the wrong labels.
//!
//! # How much of this is checked against upstream
//!
//! Partly, and it is worth being precise about which part. The motors result
//! below is corroborated by measurement rather than by reading: the
//! `srv_calc_pwm` fixture had to call `set_range` explicitly before the motor
//! channels would scale at all, which is upstream telling us those functions
//! have no default. The actuator range is not corroborated that way — it comes
//! from the generator alone.
//!
//! A per-function fixture sweeping all 190 through `aux_servo_function_setup`
//! would close that, and is recorded on COP-030 as the remaining verification.

use ap_servo::function::{DefaultOutput, Function};

/// Motor functions have no default output shape.
///
/// This is not an omission in the table — it is why `AP_Motors` calls
/// `set_range(1000)` during init. Confirmed independently: the `srv_calc_pwm`
/// fixture produced a flat 1000 for every scaled value until that call was
/// added, which is upstream saying the same thing.
#[test]
fn motors_have_no_default_output_shape() {
    for channel in 0..32_u8 {
        let f = Function::motor(channel);
        assert_eq!(
            f.default_output(),
            None,
            "motor channel {channel} (function {}) should have no default",
            f.0
        );
    }
}

/// Every actuator channel gets the same two-sided unit angle.
///
/// `case k_actuator1 ... k_actuator6:` is a GCC case-range. Read as two
/// entries it would leave four of the six without a default, and an actuator
/// with no default takes whatever the channel already had — a silent
/// full-scale error rather than a failure.
#[test]
fn every_actuator_gets_the_unit_angle() {
    for f in [
        Function::ACTUATOR1,
        Function::ACTUATOR2,
        Function::ACTUATOR3,
        Function::ACTUATOR4,
        Function::ACTUATOR5,
        Function::ACTUATOR6,
    ] {
        assert_eq!(
            f.default_output(),
            Some(DefaultOutput::Angle(1)),
            "actuator function {} should be a unit angle",
            f.0
        );
    }
}

/// Control surfaces are two-sided; flaps and throttles are one-sided.
///
/// The distinction is not cosmetic: an angle output is symmetric about trim
/// and a range output starts at the minimum. Giving a rudder a range would
/// leave it unable to deflect one way.
#[test]
fn surfaces_are_angles_and_throttles_are_ranges() {
    for f in [
        Function::AILERON,
        Function::ELEVATOR,
        Function::RUDDER,
        Function::ELEVON_LEFT,
        Function::ELEVON_RIGHT,
    ] {
        assert!(
            matches!(f.default_output(), Some(DefaultOutput::Angle(_))),
            "function {} should default to an angle, got {:?}",
            f.0,
            f.default_output()
        );
    }

    for f in [Function::FLAP, Function::FLAP_AUTO] {
        assert!(
            matches!(f.default_output(), Some(DefaultOutput::Range(_))),
            "function {} should default to a range, got {:?}",
            f.0,
            f.default_output()
        );
    }
}

/// A function with no entry returns `None` rather than a fallback.
///
/// Upstream's `default:` does nothing at all — it leaves the channel at
/// whatever it already had. A port that invented a fallback would silently
/// resize channels upstream leaves alone.
#[test]
fn an_unlisted_function_has_no_default() {
    assert_eq!(Function::NONE.default_output(), None);
    assert_eq!(Function(249).default_output(), None);
}

/// The lookup is sorted, which the binary search depends on.
#[test]
fn the_lookup_table_is_ordered() {
    let mut previous: Option<u8> = None;
    for value in 0..=u8::MAX {
        if Function(value).default_output().is_some() {
            if let Some(p) = previous {
                assert!(p < value, "table out of order at {value}");
            }
            previous = Some(value);
        }
    }
    assert!(previous.is_some(), "the table is empty");
}
