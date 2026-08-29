//! `limit_velocity_NE` leftover and the proximity-backed STOP arm.
//!
//! Tracked as **COP-026**. Fence / beacon / OA planner stay later.

use ap_avoidance::{
    AdjustVelocityNeLeftover, Avoid, ProximityStopContext, AVOID_DEFAULT,
    BACKUP_DEADZONE_M_DEFAULT, BACKUP_SPEED_MAX_NE_MS_DEFAULT, BACKUP_SPEED_MAX_U_MS_DEFAULT,
    BEHAVIOR_SLIDE, BEHAVIOR_STOP, DISABLED, MARGIN_M_DEFAULT, STOP_AT_FENCE,
};
use ap_math::scalar::{is_equal, is_zero};
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;

fn almost(a: f32, b: f32) {
    assert!(is_equal(a, b), "{a} != {b}");
}

fn almost_xy(v: Vector2f, x: f32, y: f32) {
    almost(v.x, x);
    almost(v.y, y);
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

#[test]
fn ne_defaults_and_proximity_enable_match_upstream() {
    assert_eq!(BEHAVIOR_SLIDE, 0);
    assert_eq!(BEHAVIOR_STOP, 1);
    almost(MARGIN_M_DEFAULT, 2.0);
    almost(BACKUP_DEADZONE_M_DEFAULT, 0.10);
    almost(BACKUP_SPEED_MAX_NE_MS_DEFAULT, 0.75);
    almost(BACKUP_SPEED_MAX_U_MS_DEFAULT, 0.75);

    let mut on = Avoid::new();
    assert!(on.enabled());
    assert_eq!(on.enabled_bits(), AVOID_DEFAULT);
    assert!(on.proximity_avoidance_enabled());
    assert_eq!(on.behavior(), BEHAVIOR_SLIDE);

    on.proximity_avoidance_enable(false);
    assert!(!on.proximity_avoidance_enabled());
    on.proximity_avoidance_enable(true);
    on.set_enabled(STOP_AT_FENCE);
    assert!(!on.proximity_avoidance_enabled());
}

#[test]
fn get_stopping_distance_linear_and_sqrt() {
    almost(Avoid::get_stopping_distance(0.0, 100.0, 0.0), 0.0);
    almost(Avoid::get_stopping_distance(2.0, 0.0, 200.0), 0.0);
    almost(Avoid::get_stopping_distance(0.0, 100.0, 200.0), 200.0);
    // speed 50 < accel/kP = 50, so distance is speed/kP.
    almost(Avoid::get_stopping_distance(2.0, 100.0, 50.0), 25.0);
    // speed 200 >= 50: accel/(2 kP^2) + v^2/(2 accel) = 12.5 + 200.
    almost(Avoid::get_stopping_distance(2.0, 100.0, 200.0), 212.5);
}

#[test]
fn limit_velocity_ne_leaves_perp_and_receding_unchanged() {
    let desired = Vector2f::new(0.0, 500.0);
    let limited =
        Avoid::limit_velocity_ne(0.0, 100.0, desired, Vector2f::new(1.0, 0.0), 200.0, 0.01);
    almost_xy(limited, 0.0, 500.0);

    let receding = Vector2f::new(-500.0, 0.0);
    let limited =
        Avoid::limit_velocity_ne(0.0, 100.0, receding, Vector2f::new(1.0, 0.0), 200.0, 0.01);
    almost_xy(limited, -500.0, 0.0);
}

#[test]
fn limit_velocity_ne_caps_speed_along_the_limit() {
    let desired = Vector2f::new(500.0, 0.0);
    let limited =
        Avoid::limit_velocity_ne(0.0, 100.0, desired, Vector2f::new(1.0, 0.0), 200.0, 0.01);
    let max_speed = Avoid::get_max_speed(0.0, 100.0, 200.0, 0.01);
    almost(max_speed, ap_math::scalar::safe_sqrt(40_000.0));
    almost_xy(limited, max_speed, 0.0);
}

#[test]
fn limit_velocity_neu_caps_horizontal_toward_the_obstacle() {
    let desired = Vector3f::new(500.0, 0.0, 0.0);
    let obstacle = Vector3f::new(400.0, 0.0, 0.0);
    let limited = Avoid::limit_velocity_neu(0.0, 100.0, desired, obstacle, 200.0, 0.0, 100.0, 0.01);
    // distance_from_fence_xy = max(400 - 200, 0) = 200.
    let max_speed = Avoid::get_max_speed(0.0, 100.0, 200.0, 0.01);
    almost_neu(limited, max_speed, 0.0, 0.0);
}

#[test]
fn proximity_off_or_empty_is_identity() {
    let avoid = Avoid::new();
    let desired = Vector3f::new(500.0, 0.0, 0.0);
    let leftover = avoid.adjust_velocity_proximity(
        0.0,
        100.0,
        desired,
        0.0,
        100.0,
        0.01,
        ProximityStopContext::default(),
    );
    almost_neu(leftover.desired_vel_neu_cms, 500.0, 0.0, 0.0);
    assert!(!leftover.stopped);
    assert!(!leftover.limited);

    let mut no_prox = Avoid::from_params(STOP_AT_FENCE, BACKUP_SPEED_MAX_U_MS_DEFAULT);
    no_prox.set_behavior(BEHAVIOR_STOP);
    let leftover = no_prox.adjust_velocity_proximity(
        0.0,
        100.0,
        desired,
        0.0,
        100.0,
        0.01,
        obstacle_ahead(100.0),
    );
    almost_neu(leftover.desired_vel_neu_cms, 500.0, 0.0, 0.0);
}

#[test]
fn proximity_stop_within_margin_zeros_velocity() {
    let mut avoid = Avoid::new();
    avoid.set_behavior(BEHAVIOR_STOP);
    let leftover = avoid.adjust_velocity_proximity(
        0.0,
        100.0,
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        100.0,
        0.01,
        obstacle_ahead(50.0),
    );
    assert!(leftover.stopped);
    almost_neu(leftover.desired_vel_neu_cms, 0.0, 0.0, 0.0);
}

#[test]
fn proximity_stop_on_the_edge_does_not_adjust() {
    let mut avoid = Avoid::new();
    avoid.set_behavior(BEHAVIOR_STOP);
    let leftover = avoid.adjust_velocity_proximity(
        0.0,
        100.0,
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        100.0,
        0.01,
        obstacle_ahead(0.0),
    );
    almost_neu(leftover.desired_vel_neu_cms, 500.0, 0.0, 0.0);
    assert!(!leftover.stopped);
}

#[test]
fn proximity_stop_without_intersection_is_identity() {
    let mut avoid = Avoid::new();
    avoid.set_behavior(BEHAVIOR_STOP);
    let ctx = ProximityStopContext {
        intersect_limit_neu_cm: None,
        // Far enough that backup does not arm.
        obstacle_neu_cm: Some(Vector3f::new(400.0, 0.0, 0.0)),
        ..obstacle_ahead(400.0)
    };
    let leftover = avoid.adjust_velocity_proximity(
        0.0,
        100.0,
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        100.0,
        0.01,
        ctx,
    );
    almost_neu(leftover.desired_vel_neu_cms, 500.0, 0.0, 0.0);
    assert!(!leftover.stopped);
    assert!(!leftover.limited);
}

#[test]
fn proximity_stop_beyond_margin_limits_to_stopping_speed() {
    let mut avoid = Avoid::new();
    avoid.set_behavior(BEHAVIOR_STOP);
    let leftover = avoid.adjust_velocity_proximity(
        0.0,
        100.0,
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        100.0,
        0.01,
        obstacle_ahead(400.0),
    );
    assert!(leftover.limited);
    assert!(!leftover.stopped);
    let expected = Avoid::limit_velocity_neu(
        0.0,
        100.0,
        Vector3f::new(500.0, 0.0, 0.0),
        Vector3f::new(400.0, 0.0, 0.0),
        MARGIN_M_DEFAULT * 100.0,
        0.0,
        100.0,
        0.01,
    );
    almost_neu(
        leftover.desired_vel_neu_cms,
        expected.x,
        expected.y,
        expected.z,
    );
}

#[test]
fn proximity_stop_stopping_point_uses_get_stopping_distance() {
    let mut avoid = Avoid::new();
    avoid.set_behavior(BEHAVIOR_STOP);
    let desired = Vector3f::new(500.0, 0.0, 0.0);
    let leftover = avoid.adjust_velocity_proximity(
        0.0,
        100.0,
        desired,
        0.0,
        100.0,
        0.01,
        obstacle_ahead(400.0),
    );
    let speed = desired.length();
    let stop = Avoid::get_stopping_distance(0.0, 100.0, speed);
    let margin_cm = MARGIN_M_DEFAULT * 100.0;
    let expected = desired * ((2.0 + margin_cm + stop) / speed);
    almost_neu(
        leftover.stopping_point_plus_margin_neu_cm,
        expected.x,
        expected.y,
        expected.z,
    );
}

#[test]
fn proximity_backup_arms_past_the_deadzone() {
    let mut avoid = Avoid::new();
    avoid.set_behavior(BEHAVIOR_STOP);
    // 1.0 m obstacle, 2.0 m margin → 100 cm breach, deadzone 10 cm.
    let leftover = avoid.adjust_velocity_proximity(
        0.0,
        100.0,
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        100.0,
        0.01,
        ProximityStopContext {
            // No intersection: STOP does not zero; backup still arms.
            intersect_limit_neu_cm: None,
            ..obstacle_ahead(100.0)
        },
    );
    let back = Avoid::get_max_speed(0.0, 40.0, 100.0, 0.01);
    almost(leftover.backup_vel_neu_cms.x, -back);
    almost(leftover.backup_vel_neu_cms.y, 0.0);
    assert!(is_zero(leftover.backup_vel_neu_cms.z));
}

#[test]
fn proximity_backup_skips_inside_the_deadzone() {
    let mut avoid = Avoid::new();
    avoid.set_behavior(BEHAVIOR_STOP);
    avoid.set_margin_m(0.15);
    // 10 cm obstacle, 15 cm margin → 5 cm breach < 10 cm deadzone.
    let leftover = avoid.adjust_velocity_proximity(
        0.0,
        100.0,
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        100.0,
        0.01,
        ProximityStopContext {
            intersect_limit_neu_cm: None,
            ..obstacle_ahead(10.0)
        },
    );
    almost_neu(leftover.backup_vel_neu_cms, 0.0, 0.0, 0.0);
}

#[test]
fn adjust_velocity_ne_disabled_is_identity() {
    let off = Avoid::from_params(DISABLED, BACKUP_SPEED_MAX_U_MS_DEFAULT);
    let leftover = off.adjust_velocity_ne(
        Vector3f::new(500.0, 80.0, 10.0),
        0.0,
        250.0,
        0.0,
        250.0,
        0.01,
        obstacle_ahead(50.0),
    );
    almost_neu(leftover.desired_vel_neu_cms, 500.0, 80.0, 10.0);
    assert!(!leftover.backing_up);
}

#[test]
fn adjust_velocity_ne_mixes_capped_backup() {
    let mut avoid = Avoid::new();
    avoid.set_behavior(BEHAVIOR_STOP);
    let leftover: AdjustVelocityNeLeftover = avoid.adjust_velocity_ne(
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        250.0,
        0.0,
        250.0,
        0.01,
        ProximityStopContext {
            intersect_limit_neu_cm: None,
            ..obstacle_ahead(100.0)
        },
    );
    assert!(leftover.backing_up);
    let raw = Avoid::get_max_speed(0.0, 40.0, 100.0, 0.01);
    let capped = (-raw).max(-BACKUP_SPEED_MAX_NE_MS_DEFAULT * 100.0);
    almost(leftover.backup_vel_ne_cms.x, capped);
    almost(leftover.desired_vel_neu_cms.x, capped);
}

#[test]
fn adjust_velocity_ne_yaw_rotates_through_body_frame() {
    let mut avoid = Avoid::new();
    avoid.set_behavior(BEHAVIOR_STOP);
    // Earth +X, yaw +90° → body -Y. Obstacle on body +X does not face the
    // travel direction, so STOP with no intersection leaves NE speed.
    let leftover = avoid.adjust_velocity_ne(
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        100.0,
        0.0,
        100.0,
        0.01,
        ProximityStopContext {
            yaw_rad: core::f32::consts::FRAC_PI_2,
            intersect_limit_neu_cm: None,
            obstacle_neu_cm: Some(Vector3f::new(400.0, 0.0, 0.0)),
            ..obstacle_ahead(400.0)
        },
    );
    almost_neu(leftover.desired_vel_neu_cms, 500.0, 0.0, 0.0);

    // Same yaw, obstacle on body -Y (earth +X) and intersecting inside the
    // margin: STOP zeros the body-frame velocity; earth follows before
    // the NE backup mix.
    let ctx = ProximityStopContext {
        yaw_rad: core::f32::consts::FRAC_PI_2,
        intersect_limit_neu_cm: Some(Vector3f::new(0.0, -50.0, 0.0)),
        obstacle_neu_cm: Some(Vector3f::new(0.0, -50.0, 0.0)),
        ..obstacle_ahead(50.0)
    };
    let prox = avoid.adjust_velocity_proximity(
        0.0,
        100.0,
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        100.0,
        0.01,
        ctx,
    );
    assert!(prox.stopped);
    almost_neu(prox.desired_vel_neu_cms, 0.0, 0.0, 0.0);
    // Backup is opposite the body-Y obstacle; +90° yaw sends it earth -X.
    let leftover = avoid.adjust_velocity_ne(
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        100.0,
        0.0,
        100.0,
        0.01,
        ctx,
    );
    assert!(leftover.backing_up);
    almost(leftover.desired_vel_neu_cms.x, leftover.backup_vel_ne_cms.x);
    assert!(leftover.desired_vel_neu_cms.x < 0.0);
}

#[test]
fn slide_limits_without_requiring_an_intersection() {
    let avoid = Avoid::new();
    assert_eq!(avoid.behavior(), BEHAVIOR_SLIDE);
    let leftover = avoid.adjust_velocity_proximity(
        0.0,
        100.0,
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        100.0,
        0.01,
        ProximityStopContext {
            intersect_limit_neu_cm: None,
            ..obstacle_ahead(400.0)
        },
    );
    assert!(leftover.limited);
    assert!(!leftover.stopped);
    let expected = Avoid::limit_velocity_neu(
        0.0,
        100.0,
        Vector3f::new(500.0, 0.0, 0.0),
        Vector3f::new(400.0, 0.0, 0.0),
        MARGIN_M_DEFAULT * 100.0,
        0.0,
        100.0,
        0.01,
    );
    almost_neu(
        leftover.desired_vel_neu_cms,
        expected.x,
        expected.y,
        expected.z,
    );
}
