//! SITL GPS fix producer hookup into yaw publish.

use ap_ahrs::GPS_SPEED_MIN;
use ap_gps::SitlGpsBackend;
use ap_math::matrix3::Matrix3f;
use ap_math::vector3::Vector3f;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_gps_hookup::{hookup_with_disabled_primary, SitlGpsHookup, SitlGpsTruth};

#[test]
fn gps_backend_producer_fills_yaw_publish_at_200ms() {
    let mut hookup = SitlGpsHookup::default();
    hookup.truth = SitlGpsTruth {
        velocity_ned: Vector3f::new(0.0, GPS_SPEED_MIN + 3.0, 0.0),
        latitude_deg: 47.0,
        longitude_deg: -122.0,
        altitude_m: 50.0,
        now_ms: 200,
    };
    hookup.compass_use_for_yaw = false;
    let samples = hookup.publish_yaw_samples(Matrix3f::identity(), 0.0025);
    let gps = samples.gps_yaw.expect("gps fix produced");
    assert!((gps.ground_speed - (GPS_SPEED_MIN + 3.0)).abs() < 1e-3);
    assert!((gps.ground_course_deg - 90.0).abs() < 1e-2);
    assert_eq!(gps.last_fix_time_ms, 200);
    assert!(samples.yaw_ctx.have_gps);
}

#[test]
fn main_loop_uses_sitl_gps_producer_before_dcm() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    let mut hookup = SitlGpsHookup::default();
    hookup.truth.velocity_ned = Vector3f::new(10.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    hookup.compass_use_for_yaw = false;
    vehicle.sitl_gps = Some(hookup);

    vehicle.ahrs_update();

    let gps = vehicle.gps_yaw.expect("gps from producer");
    assert!((gps.ground_speed - 10.0).abs() < 1e-3);
    assert_eq!(vehicle.ticks.ahrs_update, 1);
}

#[test]
fn gps_lag_sec_exposed_for_drift_consumers() {
    let hookup = SitlGpsHookup::default();
    assert!((hookup.gps_lag_sec() - SitlGpsBackend::default().lag_sec()).abs() < 1e-6);
}

#[test]
fn lag_buffer_feeds_yaw_publish_with_delayed_velocity() {
    let mut hookup = SitlGpsHookup::default();
    hookup.truth.velocity_ned = Vector3f::new(10.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    hookup.compass_use_for_yaw = false;
    let _ = hookup.publish_yaw_samples(Matrix3f::identity(), 0.0025);

    hookup.truth.velocity_ned = Vector3f::new(25.0, 0.0, 0.0);
    hookup.truth.now_ms = 450;
    let samples = hookup.publish_yaw_samples(Matrix3f::identity(), 0.0025);
    let gps = samples.gps_yaw.expect("delayed gps fix");
    assert!((gps.ground_speed - 10.0).abs() < 1e-3, "yaw uses lag-buffered speed");
    assert!((hookup.current_fix().ground_speed - 25.0).abs() < 1e-3);
    assert!((hookup.delayed_fix().ground_speed - 10.0).abs() < 1e-3);
}

#[test]
fn gps_status_publish_exposes_lag_buffered_velocity() {
    let mut hookup = SitlGpsHookup::default();
    hookup.truth.velocity_ned = Vector3f::new(10.0, 0.0, -2.0);
    hookup.truth.now_ms = 200;
    let status = hookup.gps_status_publish();
    assert!(status.have_fix);
    assert!(status.has_3d_fix());
    assert!((status.velocity_ned.x - 10.0).abs() < 1e-3);
    assert!((status.velocity_ned.z - (-2.0)).abs() < 1e-3);
    assert_eq!(status.num_sats, 15);
}

#[test]
fn main_loop_publishes_gps_status_from_producer() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    let mut hookup = SitlGpsHookup::default();
    hookup.truth.velocity_ned = Vector3f::new(8.0, 6.0, 0.0);
    hookup.truth.now_ms = 200;
    hookup.compass_use_for_yaw = false;
    vehicle.sitl_gps = Some(hookup);

    vehicle.ahrs_update();

    let status = vehicle.gps_status.expect("gps status published");
    assert!(status.have_fix);
    assert!((status.ground_speed - 10.0).abs() < 1e-2);
    assert_eq!(vehicle.ticks.ahrs_update, 1);
}

#[test]
fn gps_velocity_publish_exposes_vertical_component() {
    let mut hookup = SitlGpsHookup::default();
    hookup.truth.velocity_ned = Vector3f::new(10.0, 0.0, -2.5);
    hookup.truth.now_ms = 200;
    let sample = hookup.gps_velocity_publish();
    assert!(sample.have_velocity);
    assert!((sample.velocity_ned.x - 10.0).abs() < 1e-3);
    assert!((sample.velocity_ned.z - (-2.5)).abs() < 1e-3);
}

#[test]
fn main_loop_publishes_gps_velocity_from_producer() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    let mut hookup = SitlGpsHookup::default();
    hookup.truth.velocity_ned = Vector3f::new(6.0, 8.0, -1.0);
    hookup.truth.now_ms = 200;
    hookup.compass_use_for_yaw = false;
    vehicle.sitl_gps = Some(hookup);

    vehicle.ahrs_update();

    let sample = vehicle.gps_velocity.expect("gps velocity published");
    assert!(sample.have_velocity);
    assert!((sample.velocity_ned.x - 6.0).abs() < 1e-2);
    assert!((sample.velocity_ned.z - (-1.0)).abs() < 1e-2);
}

#[test]
fn gps_health_publish_reflects_satellite_threshold() {
    let mut hookup = SitlGpsHookup::default();
    hookup.truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    let health = hookup.gps_health_publish();
    assert!(health.is_healthy());
    assert!(health.usable_for_drift());
}

#[test]
fn main_loop_gates_yaw_ctx_on_gps_health() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    let mut hookup = SitlGpsHookup::default();
    hookup.truth.velocity_ned = Vector3f::new(8.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    hookup.compass_use_for_yaw = false;
    vehicle.sitl_gps = Some(hookup);

    vehicle.ahrs_update();

    let health = vehicle.gps_health.expect("gps health published");
    assert!(health.is_healthy());
    assert!(vehicle.yaw_ctx.have_gps);
    assert!(vehicle.ahrs_using_gps || health.usable_for_drift());
}

#[test]
fn main_loop_pre_arm_passes_with_healthy_sitl_gps() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    let mut hookup = SitlGpsHookup::default();
    hookup.truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    hookup.compass_use_for_yaw = false;
    vehicle.sitl_gps = Some(hookup);
    vehicle.ahrs_pre_arm_ok = true;

    vehicle.ahrs_update();
    vehicle.update_control_mode();

    assert!(vehicle.gps_health.expect("health").is_healthy());
    assert!(vehicle.gps_pre_arm_ok);
    assert!(vehicle.pre_arm_ok);
}

#[test]
fn main_loop_pre_arm_refuses_when_gps_unhealthy() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.sitl_gps = Some(SitlGpsHookup::default());
    vehicle.ahrs_pre_arm_ok = true;
    vehicle.gps_health = Some(ap_gps::GpsHealthFlags {
        have_fix: false,
        has_3d_fix: false,
        num_sats_ok: false,
        velocity_valid: false,
        fix_fresh: false,
    });

    vehicle.update_control_mode();

    assert!(!vehicle.gps_pre_arm_ok);
    assert!(!vehicle.pre_arm_ok);
}

#[test]
fn dual_gps_blend_publishes_blended_output() {
    use ap_gps::GpsAutoSwitch;

    let mut hookup = SitlGpsHookup::default();
    hookup.enable_dual_gps(GpsAutoSwitch::Blend);
    hookup.truth.velocity_ned = Vector3f::new(10.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.secondary_truth.velocity_ned = Vector3f::new(6.0, 0.0, 0.0);
        dual.secondary_truth.now_ms = 200;
    }
    let status = hookup.gps_status_publish();
    assert!(status.have_fix);
    assert!((status.velocity_ned.x - 8.0).abs() < 0.5);
    assert!(hookup.gps_output_is_blended());
}

#[test]
fn main_loop_dual_gps_blend_sets_blended_flag() {
    use ap_gps::GpsAutoSwitch;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    let mut hookup = SitlGpsHookup::default();
    hookup.enable_dual_gps(GpsAutoSwitch::Blend);
    hookup.truth.velocity_ned = Vector3f::new(10.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.secondary_truth.velocity_ned = Vector3f::new(6.0, 0.0, 0.0);
        dual.secondary_truth.now_ms = 200;
    }
    hookup.compass_use_for_yaw = false;
    vehicle.sitl_gps = Some(hookup);

    vehicle.ahrs_update();

    assert!(vehicle.gps_output_is_blended);
    let vel = vehicle.gps_velocity.expect("velocity");
    assert!((vel.velocity_ned.x - 8.0).abs() < 0.5);
}

#[test]
fn dual_gps_yaw_publish_uses_blended_output() {
    use ap_gps::GpsAutoSwitch;

    let mut hookup = SitlGpsHookup::default();
    hookup.enable_dual_gps(GpsAutoSwitch::Blend);
    hookup.truth.velocity_ned = Vector3f::new(10.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.secondary_truth.velocity_ned = Vector3f::new(6.0, 0.0, 0.0);
        dual.secondary_truth.now_ms = 200;
    }
    let yaw = hookup.yaw_publish();
    assert!(yaw.have_gps);
    assert!((yaw.ground_speed_mps - 8.0).abs() < 0.5);
}

#[test]
fn dual_gps_pre_arm_requires_both_instances_for_blend() {
    use ap_gps::GpsAutoSwitch;

    let mut hookup = SitlGpsHookup::default();
    hookup.enable_dual_gps(GpsAutoSwitch::Blend);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.secondary_truth.now_ms = 200;
    }
    assert!(hookup.gps_dual_pre_arm_ok());

    let mut hookup2 = SitlGpsHookup::default();
    hookup2.enable_dual_gps(GpsAutoSwitch::Blend);
    hookup2.truth.now_ms = 200;
    // secondary never ticked — blend falls back to healthy primary.
    assert!(hookup2.gps_dual_pre_arm_ok());
}

#[test]
fn dual_gps_use_primary_pre_arm_follows_failover_output() {
    let mut hookup = hookup_with_disabled_primary();
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.secondary_truth.now_ms = 200;
    }
    assert_eq!(hookup.gps_active_instance(), 1);
    assert!(hookup.gps_dual_pre_arm_ok());
}

#[test]
fn main_loop_pre_arm_passes_after_gps_failover() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    let mut hookup = hookup_with_disabled_primary();
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.secondary_truth.now_ms = 200;
    }
    hookup.compass_use_for_yaw = false;
    vehicle.sitl_gps = Some(hookup);
    vehicle.ahrs_pre_arm_ok = true;

    vehicle.ahrs_update();
    vehicle.update_control_mode();

    assert_eq!(vehicle.gps_active_instance, 1);
    assert!(vehicle.gps_pre_arm_ok);
    assert!(vehicle.pre_arm_ok);
}

#[test]
fn dual_gps_use_best_selects_secondary() {
    use ap_gps::GpsAutoSwitch;

    let mut hookup = SitlGpsHookup::default();
    hookup.enable_dual_gps(GpsAutoSwitch::UseBest);
    hookup.truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.primary.num_sats = 10;
        dual.secondary.num_sats = 18;
        dual.secondary_truth.velocity_ned = Vector3f::new(9.0, 0.0, 0.0);
        dual.secondary_truth.now_ms = 200;
    }
    assert_eq!(hookup.gps_active_instance(), 1);
    let status = hookup.gps_status_publish();
    assert!((status.velocity_ned.x - 9.0).abs() < 1e-3);
    assert!(!hookup.gps_output_is_blended());
}

#[test]
fn main_loop_dual_gps_use_best_sets_active_instance() {
    use ap_gps::GpsAutoSwitch;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    let mut hookup = SitlGpsHookup::default();
    hookup.enable_dual_gps(GpsAutoSwitch::UseBest);
    hookup.truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.primary.num_sats = 10;
        dual.secondary.num_sats = 18;
        dual.secondary_truth.velocity_ned = Vector3f::new(9.0, 0.0, 0.0);
        dual.secondary_truth.now_ms = 200;
    }
    hookup.compass_use_for_yaw = false;
    vehicle.sitl_gps = Some(hookup);

    vehicle.ahrs_update();

    assert_eq!(vehicle.gps_active_instance, 1);
    let vel = vehicle.gps_velocity.expect("velocity");
    assert!((vel.velocity_ned.x - 9.0).abs() < 1e-3);
}

#[test]
fn dual_gps_use_primary_failover_selects_secondary() {
    use ap_gps::GpsAutoSwitch;

    let mut hookup = hookup_with_disabled_primary();
    hookup.truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.secondary_truth.velocity_ned = Vector3f::new(8.0, 0.0, 0.0);
        dual.secondary_truth.now_ms = 200;
    }
    assert_eq!(hookup.gps_active_instance(), 1);
    let status = hookup.gps_status_publish();
    assert!((status.velocity_ned.x - 8.0).abs() < 1e-3);
}

#[test]
fn main_loop_dual_gps_use_primary_failover() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    let mut hookup = hookup_with_disabled_primary();
    hookup.truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.secondary_truth.velocity_ned = Vector3f::new(8.0, 0.0, 0.0);
        dual.secondary_truth.now_ms = 200;
    }
    hookup.compass_use_for_yaw = false;
    vehicle.sitl_gps = Some(hookup);

    vehicle.ahrs_update();

    assert_eq!(vehicle.gps_active_instance, 1);
    let vel = vehicle.gps_velocity.expect("velocity");
    assert!((vel.velocity_ned.x - 8.0).abs() < 1e-3);
}

#[test]
fn gps_health_publish_marks_stale_fix_unhealthy() {
    let mut hookup = SitlGpsHookup::default();
    hookup.truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    let fresh = hookup.gps_health_publish();
    assert!(fresh.fix_fresh);
    assert!(fresh.is_healthy());

    hookup.truth.now_ms = 5000;
    let stale = hookup.gps_health_publish();
    assert!(!stale.fix_fresh);
    assert!(!stale.is_healthy());
}

#[test]
fn main_loop_pre_arm_refuses_when_gps_fix_stale() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    let mut hookup = SitlGpsHookup::default();
    hookup.truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    hookup.compass_use_for_yaw = false;
    vehicle.sitl_gps = Some(hookup);
    vehicle.ahrs_pre_arm_ok = true;

    vehicle.ahrs_update();
    vehicle.update_control_mode();
    assert!(vehicle.gps_pre_arm_ok);

    if let Some(gps) = vehicle.sitl_gps.as_mut() {
        gps.truth.now_ms = 5000;
    }
    vehicle.update_control_mode();

    assert!(!vehicle.gps_pre_arm_ok);
    assert!(!vehicle.pre_arm_ok);
}

#[test]
fn dual_gps_use_best_pre_arm_follows_fresh_output() {
    use ap_gps::GpsAutoSwitch;

    let mut hookup = SitlGpsHookup::default();
    hookup.enable_dual_gps(GpsAutoSwitch::UseBest);
    hookup.truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.primary.num_sats = 10;
        dual.secondary.num_sats = 18;
        dual.secondary_truth.velocity_ned = Vector3f::new(9.0, 0.0, 0.0);
        dual.secondary_truth.now_ms = 200;
    }
    assert_eq!(hookup.gps_active_instance(), 1);
    assert!(hookup.gps_dual_pre_arm_ok());
}

#[test]
fn main_loop_pre_arm_passes_after_gps_use_best() {
    use ap_gps::GpsAutoSwitch;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    let mut hookup = SitlGpsHookup::default();
    hookup.enable_dual_gps(GpsAutoSwitch::UseBest);
    hookup.truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.primary.num_sats = 10;
        dual.secondary.num_sats = 18;
        dual.secondary_truth.velocity_ned = Vector3f::new(9.0, 0.0, 0.0);
        dual.secondary_truth.now_ms = 200;
    }
    hookup.compass_use_for_yaw = false;
    vehicle.sitl_gps = Some(hookup);
    vehicle.ahrs_pre_arm_ok = true;

    vehicle.ahrs_update();
    vehicle.update_control_mode();

    assert_eq!(vehicle.gps_active_instance, 1);
    assert!(vehicle.gps_pre_arm_ok);
    assert!(vehicle.pre_arm_ok);
}

#[test]
fn dual_gps_use_primary_failover_skips_stale_primary() {
    use ap_gps::GpsAutoSwitch;

    let mut hookup = SitlGpsHookup::default();
    hookup.enable_dual_gps(GpsAutoSwitch::UsePrimary);
    hookup.truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.primary.num_sats = 18;
        dual.secondary.num_sats = 10;
        dual.secondary_truth.velocity_ned = Vector3f::new(8.0, 0.0, 0.0);
        dual.secondary_truth.now_ms = 200;
    }
    let _ = hookup.gps_status_publish();
    hookup.truth.now_ms = 5000;
    assert_eq!(hookup.gps_active_instance(), 1);
    assert!(hookup.gps_dual_pre_arm_ok());
}

#[test]
fn dual_gps_use_best_prefers_fresh_secondary_over_stale_primary() {
    use ap_gps::GpsAutoSwitch;

    let mut hookup = SitlGpsHookup::default();
    hookup.enable_dual_gps(GpsAutoSwitch::UseBest);
    hookup.truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.primary.num_sats = 18;
        dual.secondary.num_sats = 10;
        dual.secondary_truth.velocity_ned = Vector3f::new(9.0, 0.0, 0.0);
        dual.secondary_truth.now_ms = 200;
    }
    let _ = hookup.gps_status_publish();
    hookup.truth.now_ms = 5000;
    assert_eq!(hookup.gps_active_instance(), 1);
    let health = hookup.gps_health_publish();
    assert!(health.is_healthy());
}

#[test]
fn dual_gps_blend_falls_back_to_fresh_secondary_when_primary_stale() {
    use ap_gps::GpsAutoSwitch;

    let mut hookup = SitlGpsHookup::default();
    hookup.enable_dual_gps(GpsAutoSwitch::Blend);
    hookup.truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.primary.num_sats = 18;
        dual.secondary.num_sats = 12;
        dual.secondary_truth.velocity_ned = Vector3f::new(7.0, 0.0, 0.0);
        dual.secondary_truth.now_ms = 200;
    }
    let _ = hookup.gps_status_publish();
    hookup.truth.now_ms = 5000;
    let status = hookup.gps_status_publish();
    assert!((status.velocity_ned.x - 7.0).abs() < 1e-3);
    assert!(!hookup.gps_output_is_blended());
}

#[test]
fn main_loop_use_best_follows_fresh_secondary_when_primary_stale() {
    use ap_gps::GpsAutoSwitch;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    let mut hookup = SitlGpsHookup::default();
    hookup.enable_dual_gps(GpsAutoSwitch::UseBest);
    hookup.truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.primary.num_sats = 18;
        dual.secondary.num_sats = 10;
        dual.secondary_truth.velocity_ned = Vector3f::new(9.0, 0.0, 0.0);
        dual.secondary_truth.now_ms = 200;
    }
    hookup.compass_use_for_yaw = false;
    vehicle.sitl_gps = Some(hookup);
    vehicle.ahrs_pre_arm_ok = true;

    vehicle.ahrs_update();
    vehicle.update_control_mode();
    if let Some(gps) = vehicle.sitl_gps.as_mut() {
        gps.truth.now_ms = 5000;
    }
    vehicle.ahrs_update();
    vehicle.update_control_mode();

    assert_eq!(vehicle.gps_active_instance, 1);
    assert!(vehicle.gps_pre_arm_ok);
    assert!(vehicle.pre_arm_ok);
}

#[test]
fn main_loop_blend_follows_fresh_secondary_when_primary_stale() {
    use ap_gps::GpsAutoSwitch;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    let mut hookup = SitlGpsHookup::default();
    hookup.enable_dual_gps(GpsAutoSwitch::Blend);
    hookup.truth.velocity_ned = Vector3f::new(5.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.primary.num_sats = 18;
        dual.secondary.num_sats = 12;
        dual.secondary_truth.velocity_ned = Vector3f::new(7.0, 0.0, 0.0);
        dual.secondary_truth.now_ms = 200;
    }
    hookup.compass_use_for_yaw = false;
    vehicle.sitl_gps = Some(hookup);
    vehicle.ahrs_pre_arm_ok = true;

    vehicle.ahrs_update();
    if let Some(gps) = vehicle.sitl_gps.as_mut() {
        gps.truth.now_ms = 5000;
    }
    vehicle.ahrs_update();

    let vel = vehicle.gps_velocity.expect("velocity");
    assert!((vel.velocity_ned.x - 7.0).abs() < 1e-3);
}

#[test]
fn dual_gps_blend_pre_arm_refuses_when_both_stale() {
    use ap_gps::GpsAutoSwitch;

    let mut hookup = SitlGpsHookup::default();
    hookup.enable_dual_gps(GpsAutoSwitch::Blend);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.secondary_truth.now_ms = 200;
    }
    let _ = hookup.gps_status_publish();
    hookup.truth.now_ms = 5000;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.primary_truth.now_ms = 5000;
        dual.secondary_truth.now_ms = 5000;
    }
    assert!(!hookup.gps_dual_pre_arm_ok());
}

#[test]
fn dual_gps_blend_pre_arm_follows_fresh_secondary_when_primary_stale() {
    use ap_gps::GpsAutoSwitch;

    let mut hookup = SitlGpsHookup::default();
    hookup.enable_dual_gps(GpsAutoSwitch::Blend);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.primary.num_sats = 18;
        dual.secondary.num_sats = 12;
        dual.secondary_truth.now_ms = 200;
    }
    let _ = hookup.gps_status_publish();
    hookup.truth.now_ms = 5000;
    assert_eq!(hookup.gps_active_instance(), 1);
    assert!(hookup.gps_dual_pre_arm_ok());
}

#[test]
fn main_loop_blend_pre_arm_passes_when_primary_stale() {
    use ap_gps::GpsAutoSwitch;

    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    let mut hookup = SitlGpsHookup::default();
    hookup.enable_dual_gps(GpsAutoSwitch::Blend);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.primary.num_sats = 18;
        dual.secondary.num_sats = 12;
        dual.secondary_truth.now_ms = 200;
    }
    hookup.compass_use_for_yaw = false;
    vehicle.sitl_gps = Some(hookup);
    vehicle.ahrs_pre_arm_ok = true;

    vehicle.ahrs_update();
    vehicle.update_control_mode();
    if let Some(gps) = vehicle.sitl_gps.as_mut() {
        gps.truth.now_ms = 5000;
    }
    vehicle.ahrs_update();
    vehicle.update_control_mode();

    assert_eq!(vehicle.gps_active_instance, 1);
    assert!(vehicle.gps_pre_arm_ok);
    assert!(vehicle.pre_arm_ok);
}

#[test]
fn apply_gps_params_sets_lag_from_delay_ms() {
    let mut hookup = SitlGpsHookup::default();
    let mut params = ap_gps::GpsParams::default();
    params.gps1.delay_ms = 300;
    hookup.apply_gps_params(params);
    assert!((hookup.gps_lag_sec() - 0.3).abs() < 1e-6);
}

#[test]
fn apply_gps_params_enables_dual_from_table() {
    let mut hookup = SitlGpsHookup::default();
    let mut params = ap_gps::GpsParams::default();
    params.gps2.gps_type = ap_gps::GPS_TYPE_SITL;
    params.auto_switch = ap_gps::GpsAutoSwitch::Blend;
    hookup.apply_gps_params(params);
    assert!(hookup.dual.is_some());
    assert_eq!(hookup.dual.unwrap().auto_switch, ap_gps::GpsAutoSwitch::Blend);
}

#[test]
fn dual_gps_min_nsats_from_params_applies_to_both_instances() {
    let mut hookup = SitlGpsHookup::default();
    let mut params = ap_gps::GpsParams::default();
    params.gps2.gps_type = ap_gps::GPS_TYPE_SITL;
    params.min_nsats = 10;
    hookup.apply_gps_params(params);
    hookup.truth.now_ms = 200;
    if let Some(dual) = hookup.dual.as_mut() {
        dual.primary.num_sats = 8;
        dual.secondary.num_sats = 8;
    }
    let _ = hookup.gps_status_publish();
    assert!(!hookup.gps_dual_pre_arm_ok());
    if let Some(dual) = hookup.dual.as_mut() {
        dual.primary.num_sats = 12;
        dual.secondary.num_sats = 12;
        dual.primary_truth.now_ms = 201;
        dual.secondary_truth.now_ms = 201;
    }
    hookup.truth.now_ms = 201;
    let _ = hookup.gps_status_publish();
    assert!(hookup.gps_dual_pre_arm_ok());
}
