//! Full `AC_Avoid::adjust_velocity` leftover: NE / body proximity, vertical
//! fence tail, and NEU backup mix. Tracked as **COP-026**.
//!
//! Circle / polygon fence NE, beacon, `limit_accel_NEU_cm`, and the OA path
//! planner stay later leftovers.

use ap_avoidance::{
    AdjustVelocityContext, AdjustVelocityLeftover, AdjustVelocityZContext, Avoid,
    ProximityStopContext, ACCEL_CMSS_MAX, BACKUP_SPEED_MAX_NE_MS_DEFAULT,
    BACKUP_SPEED_MAX_U_MS_DEFAULT, BEHAVIOR_STOP, DISABLED, STOP_AT_FENCE,
};
use ap_fence::{TYPE_ALT_MAX, TYPE_ALT_MIN};
use ap_math::scalar::{is_equal, is_zero};
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;

fn almost(a: f32, b: f32) {
    assert!(is_equal(a, b), "{a} != {b}");
}

fn almost_neu(v: Vector3f, x: f32, y: f32, z: f32) {
    almost(v.x, x);
    almost(v.y, y);
    almost(v.z, z);
}

fn obstacle_ahead(dist_cm: f32) -> ProximityStopContext {
    ProximityStopContext {
        proximity_present: true,
        proximity_alt_enabled: true,
        obstacle_count: 1,
        yaw_rad: 0.0,
        obstacle_neu_cm: Some(Vector3f::new(dist_cm, 0.0, 0.0)),
        intersect_limit_neu_cm: Some(Vector3f::new(dist_cm, 0.0, 0.0)),
    }
}

fn alt_max_ctx(veh_alt_u_m: f32, safe_alt_max_m: f32) -> AdjustVelocityZContext {
    AdjustVelocityZContext {
        fence_present: true,
        fence_enabled: TYPE_ALT_MAX,
        alt_max_u_m: Some(veh_alt_u_m),
        safe_alt_max_m,
        ..AdjustVelocityZContext::default()
    }
}

fn alt_min_ctx(veh_alt_u_m: f32, safe_alt_min_m: f32) -> AdjustVelocityZContext {
    AdjustVelocityZContext {
        fence_present: true,
        fence_enabled: TYPE_ALT_MIN,
        alt_min_u_m: Some(veh_alt_u_m),
        safe_alt_min_m,
        ..AdjustVelocityZContext::default()
    }
}

#[test]
fn adjust_velocity_disabled_is_identity() {
    let off = Avoid::from_params(DISABLED, BACKUP_SPEED_MAX_U_MS_DEFAULT);
    let leftover = off.adjust_velocity(
        Vector3f::new(500.0, 80.0, 40.0),
        0.0,
        250.0,
        0.0,
        250.0,
        0.01,
        AdjustVelocityContext {
            proximity: obstacle_ahead(50.0),
            vertical: alt_max_ctx(100.5, 100.0),
        },
    );
    almost_neu(leftover.desired_vel_neu_cms, 500.0, 80.0, 40.0);
    assert!(!leftover.backing_up);
    assert!(!leftover.proximity_stopped);
    assert!(!leftover.limit_max_alt);
}

#[test]
fn adjust_velocity_ned_m_converts_through_neu_cms() {
    let off = Avoid::from_params(DISABLED, BACKUP_SPEED_MAX_U_MS_DEFAULT);
    let leftover = off.adjust_velocity_ned_m(
        Vector3f::new(5.0, 0.8, -0.4),
        0.0,
        2.5,
        0.0,
        2.5,
        0.01,
        AdjustVelocityContext::default(),
    );
    // NED +down → NEU +up: z_cms = -(-0.4)*100 = 40.
    almost_neu(leftover.desired_vel_neu_cms, 500.0, 80.0, 40.0);
    almost_neu(leftover.desired_vel_ned_ms(), 5.0, 0.8, -0.4);
}

#[test]
fn find_max_quadrant_velocity_bins_by_sign() {
    let mut q1 = Vector2f::zero();
    let mut q2 = Vector2f::zero();
    let mut q3 = Vector2f::zero();
    let mut q4 = Vector2f::zero();
    Avoid::find_max_quadrant_velocity(Vector2f::new(3.0, 4.0), &mut q1, &mut q2, &mut q3, &mut q4);
    Avoid::find_max_quadrant_velocity(Vector2f::new(1.0, 5.0), &mut q1, &mut q2, &mut q3, &mut q4);
    almost(q1.x, 3.0);
    almost(q1.y, 5.0);
    assert!(q2.is_zero() && q3.is_zero() && q4.is_zero());

    Avoid::find_max_quadrant_velocity(
        Vector2f::new(-2.0, -1.0),
        &mut q1,
        &mut q2,
        &mut q3,
        &mut q4,
    );
    almost(q3.x, -2.0);
    almost(q3.y, -1.0);
}

#[test]
fn find_max_quadrant_velocity_3d_keeps_max_up_and_min_down() {
    let mut q1 = Vector2f::zero();
    let mut q2 = Vector2f::zero();
    let mut q3 = Vector2f::zero();
    let mut q4 = Vector2f::zero();
    let mut up = 0.0_f32;
    let mut down = 0.0_f32;
    Avoid::find_max_quadrant_velocity_3d(
        Vector3f::new(0.0, 0.0, 40.0),
        &mut q1,
        &mut q2,
        &mut q3,
        &mut q4,
        &mut up,
        &mut down,
    );
    Avoid::find_max_quadrant_velocity_3d(
        Vector3f::new(0.0, 0.0, -70.0),
        &mut q1,
        &mut q2,
        &mut q3,
        &mut q4,
        &mut up,
        &mut down,
    );
    almost(up, 40.0);
    almost(down, -70.0);
}

#[test]
fn adjust_velocity_mixes_capped_ne_backup() {
    let mut avoid = Avoid::new();
    avoid.set_behavior(BEHAVIOR_STOP);
    let leftover: AdjustVelocityLeftover = avoid.adjust_velocity(
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        250.0,
        0.0,
        250.0,
        0.01,
        AdjustVelocityContext {
            proximity: ProximityStopContext {
                intersect_limit_neu_cm: None,
                ..obstacle_ahead(100.0)
            },
            vertical: AdjustVelocityZContext::default(),
        },
    );
    assert!(leftover.backing_up);
    let raw = Avoid::get_max_speed(0.0, 40.0, 100.0, 0.01);
    let capped = (-raw).max(-BACKUP_SPEED_MAX_NE_MS_DEFAULT * 100.0);
    almost(leftover.backup_vel_neu_cms.x, capped);
    almost(leftover.desired_vel_neu_cms.x, capped);
    assert!(is_zero(leftover.desired_vel_neu_cms.z));
}

#[test]
fn adjust_velocity_yaw_rotates_through_body_frame() {
    let mut avoid = Avoid::new();
    avoid.set_behavior(BEHAVIOR_STOP);
    // Earth +X, yaw +90° → body -Y. Obstacle on body +X does not face travel.
    let leftover = avoid.adjust_velocity(
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        100.0,
        0.0,
        100.0,
        0.01,
        AdjustVelocityContext {
            proximity: ProximityStopContext {
                yaw_rad: core::f32::consts::FRAC_PI_2,
                intersect_limit_neu_cm: None,
                obstacle_neu_cm: Some(Vector3f::new(400.0, 0.0, 0.0)),
                ..obstacle_ahead(400.0)
            },
            vertical: AdjustVelocityZContext::default(),
        },
    );
    almost_neu(leftover.desired_vel_neu_cms, 500.0, 0.0, 0.0);

    // Same yaw, obstacle on body -Y (earth +X) intersecting inside the margin.
    let leftover = avoid.adjust_velocity(
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        100.0,
        0.0,
        100.0,
        0.01,
        AdjustVelocityContext {
            proximity: ProximityStopContext {
                yaw_rad: core::f32::consts::FRAC_PI_2,
                intersect_limit_neu_cm: Some(Vector3f::new(0.0, -50.0, 0.0)),
                obstacle_neu_cm: Some(Vector3f::new(0.0, -50.0, 0.0)),
                ..obstacle_ahead(50.0)
            },
            vertical: AdjustVelocityZContext::default(),
        },
    );
    assert!(leftover.proximity_stopped);
    assert!(leftover.backing_up);
    almost(
        leftover.desired_vel_neu_cms.x,
        leftover.backup_vel_neu_cms.x,
    );
    assert!(leftover.desired_vel_neu_cms.x < 0.0);
}

#[test]
fn adjust_velocity_applies_ceiling_and_mixes_u_backup() {
    let mut avoid = Avoid::new();
    avoid.set_enabled(STOP_AT_FENCE);
    let leftover = avoid.adjust_velocity(
        Vector3f::new(0.0, 0.0, 200.0),
        0.0,
        250.0,
        0.0,
        250.0,
        0.01,
        AdjustVelocityContext {
            proximity: ProximityStopContext::default(),
            vertical: alt_max_ctx(100.5, 100.0),
        },
    );
    assert!(leftover.limit_max_alt);
    assert!(leftover.backing_up);
    let raw_backup = -Avoid::get_max_speed(0.0, ACCEL_CMSS_MAX, 50.0, 0.01);
    let capped = raw_backup.max(-BACKUP_SPEED_MAX_U_MS_DEFAULT * 100.0);
    almost(leftover.backup_vel_neu_cms.z, capped);
    almost(leftover.desired_vel_neu_cms.z, capped);
}

#[test]
fn adjust_velocity_applies_floor_and_mixes_u_backup() {
    let mut avoid = Avoid::new();
    avoid.set_enabled(STOP_AT_FENCE);
    let leftover = avoid.adjust_velocity(
        Vector3f::new(0.0, 0.0, -200.0),
        0.0,
        250.0,
        0.0,
        250.0,
        0.01,
        AdjustVelocityContext {
            proximity: ProximityStopContext::default(),
            vertical: alt_min_ctx(19.5, 20.0),
        },
    );
    assert!(leftover.limit_min_alt);
    assert!(leftover.backing_up);
    let raw_backup = Avoid::get_max_speed(0.0, ACCEL_CMSS_MAX, 50.0, 0.01);
    let capped = raw_backup.min(BACKUP_SPEED_MAX_U_MS_DEFAULT * 100.0);
    almost(leftover.backup_vel_neu_cms.z, capped);
    almost(leftover.desired_vel_neu_cms.z, capped);
}

#[test]
fn adjust_velocity_combines_proximity_ne_and_fence_z() {
    let mut avoid = Avoid::new();
    avoid.set_behavior(BEHAVIOR_STOP);
    let leftover = avoid.adjust_velocity(
        Vector3f::new(500.0, 0.0, 200.0),
        0.0,
        250.0,
        0.0,
        250.0,
        0.01,
        AdjustVelocityContext {
            proximity: ProximityStopContext {
                intersect_limit_neu_cm: None,
                ..obstacle_ahead(100.0)
            },
            vertical: alt_max_ctx(100.5, 100.0),
        },
    );
    assert!(leftover.backing_up);
    assert!(leftover.limit_max_alt);
    assert!(leftover.desired_vel_neu_cms.x < 0.0);
    assert!(leftover.desired_vel_neu_cms.z < 0.0);
}

#[test]
fn adjust_velocity_ned_m_ceiling_flips_down_positive() {
    let mut avoid = Avoid::new();
    avoid.set_enabled(STOP_AT_FENCE);
    let leftover = avoid.adjust_velocity_ned_m(
        Vector3f::new(0.0, 0.0, -2.0),
        0.0,
        2.5,
        0.0,
        2.5,
        0.01,
        AdjustVelocityContext {
            proximity: ProximityStopContext::default(),
            vertical: alt_max_ctx(100.5, 100.0),
        },
    );
    assert!(leftover.limit_max_alt);
    // Climb was NEU +200 cm/s; ceiling backup is NEU down → NED +down.
    assert!(leftover.desired_vel_ned_ms().z > 0.0);
}
