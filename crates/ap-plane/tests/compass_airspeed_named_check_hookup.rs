//! Plane hookup: compass and airspeed named checks in the arming registry.

use ap_airspeed::sitl::AirspeedHealthFlags;
use ap_arming::{Arming, Check};
use ap_compass::sitl::CompassHealthFlags;
use ap_plane::compass_airspeed_named_check_hookup::{
    airspeed_named_check, compass_airspeed_named_checks, compass_named_check,
    plane_pre_arm_checks_compass_airspeed, AIRSPEED_CHECK_NAME, COMPASS_CHECK_NAME,
};
use ap_plane::mode_run::{pre_arm_checks, PreArmResult};

fn healthy_compass() -> CompassHealthFlags {
    CompassHealthFlags {
        instance_count: 1,
        healthy: [true, false],
        have_sample: [true, false],
        primary: 0,
    }
}

fn unhealthy_compass() -> CompassHealthFlags {
    CompassHealthFlags::default()
}

fn healthy_airspeed() -> AirspeedHealthFlags {
    AirspeedHealthFlags {
        instance_count: 1,
        healthy: [true, false],
        have_sample: [true, false],
        primary: 0,
    }
}

fn unhealthy_airspeed() -> AirspeedHealthFlags {
    AirspeedHealthFlags::default()
}

#[test]
fn compass_named_check_uses_the_compass_bit_and_existing_health_gate() {
    let ok = compass_named_check(healthy_compass(), true);
    assert_eq!(ok.check, Check::Compass);
    assert_eq!(ok.name, COMPASS_CHECK_NAME);
    assert!(ok.ok);

    let bad = compass_named_check(unhealthy_compass(), true);
    assert!(!bad.ok);

    // `require_compass = false` is the existing hookup's "no compass configured" skip.
    assert!(compass_named_check(unhealthy_compass(), false).ok);
}

#[test]
fn airspeed_named_check_uses_the_airspeed_bit_and_existing_health_gate() {
    let ok = airspeed_named_check(healthy_airspeed(), true);
    assert_eq!(ok.check, Check::Airspeed);
    assert_eq!(ok.name, AIRSPEED_CHECK_NAME);
    assert!(ok.ok);

    let bad = airspeed_named_check(unhealthy_airspeed(), true);
    assert!(!bad.ok);

    // `require_airspeed = false` is the existing hookup's "no airspeed configured" skip.
    assert!(airspeed_named_check(unhealthy_airspeed(), false).ok);
}

#[test]
fn registry_refuses_unhealthy_compass() {
    let mode = pre_arm_checks(true, "");
    assert_eq!(
        plane_pre_arm_checks_compass_airspeed(
            mode,
            Arming::new(),
            unhealthy_compass(),
            true,
            healthy_airspeed(),
            true,
        ),
        PreArmResult::Refused(COMPASS_CHECK_NAME)
    );
}

#[test]
fn registry_refuses_unhealthy_airspeed() {
    let mode = pre_arm_checks(true, "");
    assert_eq!(
        plane_pre_arm_checks_compass_airspeed(
            mode,
            Arming::new(),
            healthy_compass(),
            true,
            unhealthy_airspeed(),
            true,
        ),
        PreArmResult::Refused(AIRSPEED_CHECK_NAME)
    );
}

#[test]
fn skipping_compass_lets_an_unhealthy_compass_through() {
    let mode = pre_arm_checks(true, "");
    let arming = Arming {
        checks_to_skip: Check::Compass.as_u32(),
        ..Arming::new()
    };
    assert_eq!(
        plane_pre_arm_checks_compass_airspeed(
            mode,
            arming,
            unhealthy_compass(),
            true,
            healthy_airspeed(),
            true,
        ),
        PreArmResult::Allowed
    );
}

#[test]
fn skipping_airspeed_lets_an_unhealthy_airspeed_through() {
    let mode = pre_arm_checks(true, "");
    let arming = Arming {
        checks_to_skip: Check::Airspeed.as_u32(),
        ..Arming::new()
    };
    assert_eq!(
        plane_pre_arm_checks_compass_airspeed(
            mode,
            arming,
            healthy_compass(),
            true,
            unhealthy_airspeed(),
            true,
        ),
        PreArmResult::Allowed
    );
}

#[test]
fn mode_refusal_is_kept() {
    let mode = pre_arm_checks(false, "not armable here");
    assert_eq!(
        plane_pre_arm_checks_compass_airspeed(
            mode,
            Arming::new(),
            healthy_compass(),
            true,
            healthy_airspeed(),
            true,
        ),
        PreArmResult::Refused("not armable here")
    );
}

#[test]
fn both_healthy_allows() {
    let mode = pre_arm_checks(true, "");
    assert_eq!(
        plane_pre_arm_checks_compass_airspeed(
            mode,
            Arming::new(),
            healthy_compass(),
            true,
            healthy_airspeed(),
            true,
        ),
        PreArmResult::Allowed
    );
    let [compass, airspeed] =
        compass_airspeed_named_checks(healthy_compass(), true, healthy_airspeed(), true);
    assert!(compass.ok);
    assert!(airspeed.ok);
    assert_eq!(compass.check, Check::Compass);
    assert_eq!(airspeed.check, Check::Airspeed);
}
