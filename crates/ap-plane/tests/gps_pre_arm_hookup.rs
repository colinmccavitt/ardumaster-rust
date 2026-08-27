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

#[test]
fn gps_pre_arm_respects_param_min_nsats_in_health_flags() {
    use ap_gps::{FixType, GpsFixState, GpsHealthFlags, GpsStatus};
    use ap_math::vector3::Vector3f;

    let fix = GpsFixState {
        fix_type: FixType::Fix3D,
        num_sats: 5,
        velocity_ned: Vector3f::zero(),
        ground_speed: 0.0,
        ground_course_deg: 0.0,
        last_fix_time_ms: 200,
        latitude_deg: 51.0,
        longitude_deg: -0.1,
        altitude_m: 100.0,
        have_fix: true,
    };
    let status = GpsStatus::from_fix(&fix, 0.1);
    let default_health = GpsHealthFlags::from_status_at(&status, 200);
    assert!(!gps_pre_arm_check(Some(default_health), true));
    let relaxed = GpsHealthFlags::from_status_at_min(&status, 200, 5);
    assert!(gps_pre_arm_check(Some(relaxed), true));
}
#[test]
fn rtk_rover_yaw_stale_blocks_dual_pre_arm() {
    use ap_gps::{GPS_TYPE_UBLOX_RTK_BASE, GPS_TYPE_UBLOX_RTK_ROVER, GpsParams, GPS_YAW_TIMEOUT_MS};
    use ap_plane::sitl_gps_hookup::SitlGpsHookup;

    let mut hookup = SitlGpsHookup::default();
    let mut params = GpsParams::default();
    params.gps1.gps_type = GPS_TYPE_UBLOX_RTK_BASE;
    params.gps2.gps_type = GPS_TYPE_UBLOX_RTK_ROVER;
    hookup.apply_gps_params(params);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.secondary_truth.latitude_deg = dual.primary_truth.latitude_deg + 0.001;
        dual.secondary_truth.now_ms = 200;
    }
    let _ = hookup.gps_status_publish();
    assert!(hookup.gps_dual_pre_arm_ok());
    hookup.truth.now_ms = 200 + GPS_YAW_TIMEOUT_MS + 1;
    assert!(!hookup.gps_dual_pre_arm_ok());
}



#[test]
fn rtk_rover_yaw_poor_accuracy_blocks_dual_pre_arm() {
    use ap_gps::{
        GpsParams, GpsYawState, GPS_TYPE_UBLOX_RTK_BASE, GPS_TYPE_UBLOX_RTK_ROVER,
        GPS_YAW_MAX_ACCURACY_DEG,
    };
    use ap_plane::sitl_gps_hookup::SitlGpsHookup;

    let mut hookup = SitlGpsHookup::default();
    let mut params = GpsParams::default();
    params.gps1.gps_type = GPS_TYPE_UBLOX_RTK_BASE;
    params.gps2.gps_type = GPS_TYPE_UBLOX_RTK_ROVER;
    hookup.apply_gps_params(params);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.secondary_truth.latitude_deg = dual.primary_truth.latitude_deg + 0.001;
        dual.secondary_truth.now_ms = 200;
    }
    let _ = hookup.gps_status_publish();
    if let Some(dual) = hookup.dual.as_mut() {
        dual.set_rover_yaw_state(GpsYawState {
            have_gps_yaw: true,
            gps_yaw_deg: 90.0,
            gps_yaw_accuracy_deg: GPS_YAW_MAX_ACCURACY_DEG + 15.0,
            gps_yaw_time_ms: 200,
            have_gps_yaw_accuracy: true,
        });
    }
    assert!(!hookup.gps_dual_pre_arm_ok());
    if let Some(dual) = hookup.dual.as_mut() {
        dual.set_rover_yaw_state(GpsYawState {
            have_gps_yaw: true,
            gps_yaw_deg: 90.0,
            gps_yaw_accuracy_deg: GPS_YAW_MAX_ACCURACY_DEG,
            gps_yaw_time_ms: 200,
            have_gps_yaw_accuracy: true,
        });
    }
    assert!(hookup.gps_dual_pre_arm_ok());
}
