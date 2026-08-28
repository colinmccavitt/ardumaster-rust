//! Plane hookup: barometer and AHRS named checks in the arming registry.

use ap_arming::{Arming, Check};
use ap_baro::sitl::BaroHealthFlags;
use ap_plane::baro_ahrs_named_check_hookup::{
    ahrs_named_check, baro_ahrs_named_checks, baro_named_check, plane_pre_arm_checks_baro_ahrs,
    AHRS_CHECK_NAME, BARO_CHECK_NAME,
};
use ap_plane::mode_run::{pre_arm_checks, PreArmResult};

fn healthy_baro() -> BaroHealthFlags {
    BaroHealthFlags {
        instance_count: 1,
        healthy: [true, false],
        have_sample: [true, false],
        primary: 0,
    }
}

fn unhealthy_baro() -> BaroHealthFlags {
    BaroHealthFlags::default()
}

#[test]
fn baro_named_check_uses_the_baro_bit_and_existing_health_gate() {
    let ok = baro_named_check(healthy_baro(), true);
    assert_eq!(ok.check, Check::Baro);
    assert_eq!(ok.name, BARO_CHECK_NAME);
    assert!(ok.ok);

    let bad = baro_named_check(unhealthy_baro(), true);
    assert!(!bad.ok);

    // `require_baro = false` is the existing hookup's "no baro configured" skip.
    assert!(baro_named_check(unhealthy_baro(), false).ok);
}

#[test]
fn ahrs_named_check_uses_the_ins_bit() {
    let ok = ahrs_named_check(true);
    assert_eq!(ok.check, Check::Ins);
    assert_eq!(ok.name, AHRS_CHECK_NAME);
    assert!(ok.ok);
    assert!(!ahrs_named_check(false).ok);
}

#[test]
fn registry_refuses_unhealthy_baro() {
    let mode = pre_arm_checks(true, "");
    assert_eq!(
        plane_pre_arm_checks_baro_ahrs(mode, Arming::new(), unhealthy_baro(), true, true),
        PreArmResult::Refused(BARO_CHECK_NAME)
    );
}

#[test]
fn registry_refuses_unhealthy_ahrs_under_ins() {
    let mode = pre_arm_checks(true, "");
    assert_eq!(
        plane_pre_arm_checks_baro_ahrs(mode, Arming::new(), healthy_baro(), true, false),
        PreArmResult::Refused(AHRS_CHECK_NAME)
    );
}

#[test]
fn skipping_baro_lets_an_unhealthy_baro_through() {
    let mode = pre_arm_checks(true, "");
    let arming = Arming {
        checks_to_skip: Check::Baro.as_u32(),
        ..Arming::new()
    };
    assert_eq!(
        plane_pre_arm_checks_baro_ahrs(mode, arming, unhealthy_baro(), true, true),
        PreArmResult::Allowed
    );
}

#[test]
fn skipping_ins_lets_an_unhealthy_ahrs_through() {
    let mode = pre_arm_checks(true, "");
    let arming = Arming {
        checks_to_skip: Check::Ins.as_u32(),
        ..Arming::new()
    };
    assert_eq!(
        plane_pre_arm_checks_baro_ahrs(mode, arming, healthy_baro(), true, false),
        PreArmResult::Allowed
    );
}

#[test]
fn mode_refusal_is_kept() {
    let mode = pre_arm_checks(false, "not armable here");
    assert_eq!(
        plane_pre_arm_checks_baro_ahrs(mode, Arming::new(), healthy_baro(), true, true),
        PreArmResult::Refused("not armable here")
    );
}

#[test]
fn both_healthy_allows() {
    let mode = pre_arm_checks(true, "");
    assert_eq!(
        plane_pre_arm_checks_baro_ahrs(mode, Arming::new(), healthy_baro(), true, true),
        PreArmResult::Allowed
    );
    let [baro, ahrs] = baro_ahrs_named_checks(healthy_baro(), true, true);
    assert!(baro.ok);
    assert!(ahrs.ok);
    assert_eq!(baro.check, Check::Baro);
    assert_eq!(ahrs.check, Check::Ins);
}
