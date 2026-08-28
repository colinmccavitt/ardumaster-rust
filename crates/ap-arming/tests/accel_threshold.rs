//! ARMING_ACCTHRESH / accel-threshold named-check.

use ap_arming::accel_threshold::{
    accel_error_magnitude_sq, accel_magnitude_within_threshold, accel_threshold_named_check,
    accel_threshold_named_check_from_samples, accels_within_threshold, ACCEL_CHECK_NAME,
    ARMING_ACCTHRESH_DEFAULT,
};
use ap_arming::{Arming, Check, PreArmOutcome};

#[test]
fn plane_default_accthresh_is_upstream_075() {
    assert!(ARMING_ACCTHRESH_DEFAULT > 0.74);
    assert!(ARMING_ACCTHRESH_DEFAULT < 0.76);
}

#[test]
fn magnitude_inside_the_threshold_passes() {
    assert!(accel_magnitude_within_threshold(0.0, ARMING_ACCTHRESH_DEFAULT));
    assert!(accel_magnitude_within_threshold(0.75, ARMING_ACCTHRESH_DEFAULT));
    assert!(accel_magnitude_within_threshold(0.5, 0.5));
}

#[test]
fn magnitude_outside_the_threshold_fails() {
    assert!(!accel_magnitude_within_threshold(0.76, ARMING_ACCTHRESH_DEFAULT));
    assert!(!accel_magnitude_within_threshold(1.0, 0.75));
}

#[test]
fn identical_accels_are_within_threshold() {
    let g = [0.0, 0.0, 9.81];
    assert!(accels_within_threshold(g, g, ARMING_ACCTHRESH_DEFAULT));
    let named = accel_threshold_named_check_from_samples(g, g, ARMING_ACCTHRESH_DEFAULT);
    assert_eq!(named.check, Check::Ins);
    assert_eq!(named.name, ACCEL_CHECK_NAME);
    assert!(named.ok);
}

#[test]
fn xy_difference_outside_threshold_fails_the_ins_named_check() {
    let primary = [0.0, 0.0, 9.81];
    let drifted = [1.0, 0.0, 9.81];
    assert!(!accels_within_threshold(primary, drifted, ARMING_ACCTHRESH_DEFAULT));
    let named = accel_threshold_named_check_from_samples(primary, drifted, ARMING_ACCTHRESH_DEFAULT);
    assert_eq!(named.check, Check::Ins);
    assert!(!named.ok);
}

#[test]
fn z_is_half_weighted_like_upstream() {
    // Raw Z delta 1.0 m/s/s becomes 0.5 after the EKF Z scale, which is
    // inside the 0.75 default. The same 1.0 on X is outside.
    let primary = [0.0, 0.0, 9.81];
    let z_only = [0.0, 0.0, 10.81];
    let x_only = [1.0, 0.0, 9.81];
    assert!(accels_within_threshold(primary, z_only, ARMING_ACCTHRESH_DEFAULT));
    assert!(!accels_within_threshold(primary, x_only, ARMING_ACCTHRESH_DEFAULT));
    let z_mag_sq = accel_error_magnitude_sq(primary, z_only);
    assert!(z_mag_sq > 0.24);
    assert!(z_mag_sq < 0.26);
}

#[test]
fn named_check_from_magnitude_uses_the_ins_bit() {
    let ok = accel_threshold_named_check(0.1, ARMING_ACCTHRESH_DEFAULT);
    assert_eq!(ok.check, Check::Ins);
    assert_eq!(ok.name, ACCEL_CHECK_NAME);
    assert!(ok.ok);
    assert!(!accel_threshold_named_check(2.0, ARMING_ACCTHRESH_DEFAULT).ok);
}

#[test]
fn registry_refuses_when_accel_magnitude_is_outside_threshold() {
    let arming = Arming::new();
    let named = accel_threshold_named_check(1.0, ARMING_ACCTHRESH_DEFAULT);
    assert_eq!(
        arming.pre_arm_checks(&[named]),
        PreArmOutcome::Refused {
            check: Check::Ins,
            name: ACCEL_CHECK_NAME,
        }
    );
}

#[test]
fn registry_allows_when_accel_magnitude_is_inside_threshold() {
    let arming = Arming::new();
    let named = accel_threshold_named_check(0.2, ARMING_ACCTHRESH_DEFAULT);
    assert_eq!(arming.pre_arm_checks(&[named]), PreArmOutcome::Allowed);
}

#[test]
fn skipping_ins_lets_an_out_of_threshold_accel_through() {
    let arming = Arming {
        checks_to_skip: Check::Ins.as_u32(),
        ..Arming::new()
    };
    let named = accel_threshold_named_check(2.0, ARMING_ACCTHRESH_DEFAULT);
    assert_eq!(arming.pre_arm_checks(&[named]), PreArmOutcome::Allowed);
}
