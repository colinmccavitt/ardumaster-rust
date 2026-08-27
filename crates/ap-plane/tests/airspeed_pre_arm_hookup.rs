use ap_airspeed::sitl::AirspeedHealthFlags;
use ap_plane::airspeed_pre_arm_hookup::{
    airspeed_pre_arm_check, plane_pre_arm_checks_airspeed, AIRSPEED_REFUSAL,
};
use ap_plane::baro_pre_arm_hookup::plane_pre_arm_checks_baro;
use ap_plane::compass_pre_arm_hookup::plane_pre_arm_checks_compass;
use ap_plane::gps_pre_arm_hookup::plane_pre_arm_checks_gps;
use ap_plane::mode_run::{pre_arm_checks, PreArmResult};

#[test]
fn airspeed_pre_arm_skips_when_airspeed_not_required() {
    assert!(airspeed_pre_arm_check(AirspeedHealthFlags::default(), false));
}

#[test]
fn airspeed_pre_arm_requires_primary_healthy_when_configured() {
    let healthy = AirspeedHealthFlags {
        instance_count: 1,
        healthy: [true, false],
        have_sample: [true, false],
        primary: 0,
    };
    assert!(airspeed_pre_arm_check(healthy, true));
    assert!(!airspeed_pre_arm_check(AirspeedHealthFlags::default(), true));
    let dual_without_failover = AirspeedHealthFlags {
        instance_count: 2,
        healthy: [false, true],
        have_sample: [false, true],
        primary: 0,
    };
    assert!(!airspeed_pre_arm_check(dual_without_failover, true));
    let dual_after_failover = AirspeedHealthFlags {
        instance_count: 2,
        healthy: [false, true],
        have_sample: [false, true],
        primary: 1,
    };
    assert!(airspeed_pre_arm_check(dual_after_failover, true));
}

#[test]
fn airspeed_pre_arm_requires_failover_primary_healthy() {
    let pre_failover = AirspeedHealthFlags {
        instance_count: 2,
        healthy: [false, true],
        have_sample: [true, true],
        primary: 0,
    };
    assert!(!airspeed_pre_arm_check(pre_failover, true));
}

#[test]
fn airspeed_pre_arm_refuses_primary_without_sample() {
    let no_sample = AirspeedHealthFlags {
        instance_count: 1,
        healthy: [true, false],
        have_sample: [false, false],
        primary: 0,
    };
    assert!(!airspeed_pre_arm_check(no_sample, true));
}

#[test]
fn plane_pre_arm_checks_airspeed_preserves_prior_refusal() {
    let mode = pre_arm_checks(false, "mode blocked");
    assert_eq!(
        plane_pre_arm_checks_airspeed(mode, true),
        PreArmResult::Refused("mode blocked"),
    );
}

#[test]
fn plane_pre_arm_checks_airspeed_refuses_unhealthy_airspeed() {
    let mode = pre_arm_checks(true, "");
    let with_ahrs = ap_plane::ahrs_pre_arm_hookup::plane_pre_arm_checks(mode, true);
    let with_gps = plane_pre_arm_checks_gps(with_ahrs, true);
    let with_baro = plane_pre_arm_checks_baro(with_gps, true);
    let with_compass = plane_pre_arm_checks_compass(with_baro, true);
    assert_eq!(
        plane_pre_arm_checks_airspeed(with_compass, false),
        PreArmResult::Refused(AIRSPEED_REFUSAL),
    );
}