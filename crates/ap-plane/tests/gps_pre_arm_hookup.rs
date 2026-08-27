use ap_gps::GpsHealthFlags;
use ap_plane::gps_pre_arm_hookup::{
    gps_pre_arm_check, plane_pre_arm_checks_gps, GPS_REFUSAL,
};
use ap_plane::mode_run::{pre_arm_checks, PreArmResult};

#[test]
fn gps_pre_arm_skips_when_gps_not_required() {
    assert!(gps_pre_arm_check(None, false));
    assert!(gps_pre_arm_check(
        Some(GpsHealthFlags {
            have_fix: false,
            has_3d_fix: false,
            num_sats_ok: false,
            velocity_valid: false,
            fix_fresh: false,
        }),
        false,
    ));
}

#[test]
fn gps_pre_arm_requires_healthy_fix_when_configured() {
    let healthy = GpsHealthFlags {
        have_fix: true,
        has_3d_fix: true,
        num_sats_ok: true,
        velocity_valid: true,
        fix_fresh: true,
    };
    assert!(gps_pre_arm_check(Some(healthy), true));
    assert!(!gps_pre_arm_check(None, true));
    assert!(!gps_pre_arm_check(
        Some(GpsHealthFlags {
            have_fix: true,
            has_3d_fix: false,
            num_sats_ok: true,
            velocity_valid: false,
            fix_fresh: false,
        }),
        true,
    ));
}

#[test]
fn plane_pre_arm_checks_gps_preserves_prior_refusal() {
    let mode = pre_arm_checks(false, "mode blocked");
    assert_eq!(
        plane_pre_arm_checks_gps(mode, true),
        PreArmResult::Refused("mode blocked"),
    );
}

#[test]
fn plane_pre_arm_checks_gps_refuses_unhealthy_gps() {
    let mode = pre_arm_checks(true, "");
    let with_ahrs = ap_plane::ahrs_pre_arm_hookup::plane_pre_arm_checks(mode, true);
    assert_eq!(
        plane_pre_arm_checks_gps(with_ahrs, false),
        PreArmResult::Refused(GPS_REFUSAL),
    );
}
