//! ARMING_NEED_LOC / require-position-before-arm.

use ap_arming::need_loc::{
    gps_has_3d_fix, has_absolute_position, need_loc_named_check, require_location_allows_arm,
    RequireLocation, ARMING_NEED_LOC_DEFAULT, GPS_OK_FIX_3D, NEED_LOC_CHECK_NAME,
};
use ap_arming::{Arming, Check, PreArmOutcome};

#[test]
fn plane_default_need_loc_does_not_require_position() {
    assert_eq!(ARMING_NEED_LOC_DEFAULT, RequireLocation::No);
    assert_eq!(ARMING_NEED_LOC_DEFAULT.as_u8(), 0);
    assert!(!RequireLocation::No.required());
    assert!(RequireLocation::Yes.required());
}

#[test]
fn from_u8_decodes_only_the_two_upstream_values() {
    assert_eq!(RequireLocation::from_u8(0), Some(RequireLocation::No));
    assert_eq!(RequireLocation::from_u8(1), Some(RequireLocation::Yes));
    assert_eq!(RequireLocation::from_u8(2), None);
}

#[test]
fn gps_3d_fix_is_status_three_or_better() {
    assert_eq!(GPS_OK_FIX_3D, 3);
    assert!(!gps_has_3d_fix(0));
    assert!(!gps_has_3d_fix(2));
    assert!(gps_has_3d_fix(3));
    assert!(gps_has_3d_fix(6));
}

#[test]
fn no_home_or_no_3d_fix_is_not_an_absolute_position() {
    assert!(!has_absolute_position(false, GPS_OK_FIX_3D));
    assert!(!has_absolute_position(true, 2));
    assert!(has_absolute_position(true, GPS_OK_FIX_3D));
}

#[test]
fn need_loc_no_allows_arm_without_a_fix() {
    assert!(require_location_allows_arm(RequireLocation::No, false, 0));
}

#[test]
fn need_loc_yes_refuses_without_home_or_3d_fix() {
    assert!(!require_location_allows_arm(
        RequireLocation::Yes,
        false,
        GPS_OK_FIX_3D,
    ));
    assert!(!require_location_allows_arm(RequireLocation::Yes, true, 2));
    assert!(require_location_allows_arm(
        RequireLocation::Yes,
        true,
        GPS_OK_FIX_3D,
    ));
}

#[test]
fn registry_refuses_when_need_loc_yes_and_no_3d_fix() {
    let arming = Arming::new();
    let named = need_loc_named_check(RequireLocation::Yes, true, 2);
    assert_eq!(named.check, Check::Gps);
    assert_eq!(named.name, NEED_LOC_CHECK_NAME);
    assert!(!named.ok);
    assert_eq!(
        arming.pre_arm_checks(&[named]),
        PreArmOutcome::Refused {
            check: Check::Gps,
            name: NEED_LOC_CHECK_NAME,
        }
    );
}

#[test]
fn registry_refuses_when_need_loc_yes_and_home_unset() {
    let arming = Arming::new();
    let named = need_loc_named_check(RequireLocation::Yes, false, GPS_OK_FIX_3D);
    assert_eq!(
        arming.pre_arm_checks(&[named]),
        PreArmOutcome::Refused {
            check: Check::Gps,
            name: NEED_LOC_CHECK_NAME,
        }
    );
}

#[test]
fn registry_allows_when_need_loc_yes_and_home_plus_3d_fix() {
    let arming = Arming::new();
    let named = need_loc_named_check(RequireLocation::Yes, true, GPS_OK_FIX_3D);
    assert!(named.ok);
    assert_eq!(arming.pre_arm_checks(&[named]), PreArmOutcome::Allowed);
}

#[test]
fn registry_allows_when_need_loc_is_off_even_without_a_fix() {
    let arming = Arming::new();
    let named = need_loc_named_check(RequireLocation::No, false, 0);
    assert!(named.ok);
    assert_eq!(arming.pre_arm_checks(&[named]), PreArmOutcome::Allowed);
}
