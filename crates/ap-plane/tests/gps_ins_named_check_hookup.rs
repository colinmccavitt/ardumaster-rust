//! Plane hookup: GPS and INS named checks in the arming registry.

use ap_arming::{Arming, Check};
use ap_gps::GpsHealthFlags;
use ap_ins::InertialSensorFrontend;
use ap_plane::gps_ins_named_check_hookup::{
    gps_ins_named_checks, gps_named_check, ins_named_check, ins_named_check_from_frontend,
    plane_pre_arm_checks_gps_ins, GPS_CHECK_NAME, INS_CHECK_NAME,
};
use ap_plane::mode_run::{pre_arm_checks, PreArmResult};

fn healthy_gps() -> GpsHealthFlags {
    GpsHealthFlags {
        have_fix: true,
        has_3d_fix: true,
        num_sats_ok: true,
        velocity_valid: true,
        fix_fresh: true,
    }
}

fn unhealthy_gps() -> GpsHealthFlags {
    GpsHealthFlags::default()
}

#[test]
fn gps_named_check_uses_the_gps_bit_and_existing_health_gate() {
    let ok = gps_named_check(Some(healthy_gps()), true);
    assert_eq!(ok.check, Check::Gps);
    assert_eq!(ok.name, GPS_CHECK_NAME);
    assert!(ok.ok);

    let bad = gps_named_check(Some(unhealthy_gps()), true);
    assert!(!bad.ok);
    assert!(!gps_named_check(None, true).ok);

    // `require_gps = false` is the existing hookup's "no GPS configured" skip.
    assert!(gps_named_check(None, false).ok);
    assert!(gps_named_check(Some(unhealthy_gps()), false).ok);
}

#[test]
fn ins_named_check_uses_the_ins_bit_and_gyro_accel_health() {
    let ok = ins_named_check(true, true);
    assert_eq!(ok.check, Check::Ins);
    assert_eq!(ok.name, INS_CHECK_NAME);
    assert!(ok.ok);
    assert!(!ins_named_check(false, true).ok);
    assert!(!ins_named_check(true, false).ok);

    // An empty frontend has published nothing, so both sensors are unhealthy.
    let empty = InertialSensorFrontend::new();
    let from_fe = ins_named_check_from_frontend(&empty);
    assert_eq!(from_fe.check, Check::Ins);
    assert!(!from_fe.ok);
}

#[test]
fn registry_refuses_unhealthy_gps() {
    let mode = pre_arm_checks(true, "");
    assert_eq!(
        plane_pre_arm_checks_gps_ins(mode, Arming::new(), Some(unhealthy_gps()), true, true, true),
        PreArmResult::Refused(GPS_CHECK_NAME)
    );
}

#[test]
fn registry_refuses_unhealthy_ins() {
    let mode = pre_arm_checks(true, "");
    assert_eq!(
        plane_pre_arm_checks_gps_ins(mode, Arming::new(), Some(healthy_gps()), true, false, true),
        PreArmResult::Refused(INS_CHECK_NAME)
    );
}

#[test]
fn skipping_gps_lets_an_unhealthy_gps_through() {
    let mode = pre_arm_checks(true, "");
    let arming = Arming {
        checks_to_skip: Check::Gps.as_u32(),
        ..Arming::new()
    };
    assert_eq!(
        plane_pre_arm_checks_gps_ins(mode, arming, Some(unhealthy_gps()), true, true, true),
        PreArmResult::Allowed
    );
}

#[test]
fn skipping_ins_lets_an_unhealthy_ins_through() {
    let mode = pre_arm_checks(true, "");
    let arming = Arming {
        checks_to_skip: Check::Ins.as_u32(),
        ..Arming::new()
    };
    assert_eq!(
        plane_pre_arm_checks_gps_ins(mode, arming, Some(healthy_gps()), true, false, false),
        PreArmResult::Allowed
    );
}

#[test]
fn mode_refusal_is_kept() {
    let mode = pre_arm_checks(false, "not armable here");
    assert_eq!(
        plane_pre_arm_checks_gps_ins(mode, Arming::new(), Some(healthy_gps()), true, true, true),
        PreArmResult::Refused("not armable here")
    );
}

#[test]
fn both_healthy_allows() {
    let mode = pre_arm_checks(true, "");
    assert_eq!(
        plane_pre_arm_checks_gps_ins(mode, Arming::new(), Some(healthy_gps()), true, true, true),
        PreArmResult::Allowed
    );
    let [gps, ins] = gps_ins_named_checks(Some(healthy_gps()), true, true, true);
    assert!(gps.ok);
    assert!(ins.ok);
    assert_eq!(gps.check, Check::Gps);
    assert_eq!(ins.check, Check::Ins);
}
