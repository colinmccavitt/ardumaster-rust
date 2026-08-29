//! Circle / polygon / beacon fence NE leftover. Tracked as **COP-026**.
//!
//! `limit_accel_NEU_cm` is on `Avoid`; these cases keep `AVOID_ACCEL_MAX`
//! at zero so the fence-NE leftover stays visible. The OA path planner
//! stays later.

use ap_avoidance::{
    AdjustVelocityContext, AdjustVelocityZContext, Avoid, FenceCircle, FenceNeContext,
    FencePolygon, ProximityStopContext, ACCEL_CMSS_MAX, BEHAVIOR_SLIDE, BEHAVIOR_STOP, DISABLED,
    STOP_AT_BEACON_FENCE, STOP_AT_FENCE,
};
use ap_fence::{TYPE_CIRCLE, TYPE_POLYGON};
use ap_math::scalar::{is_equal, is_zero};
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;

fn almost(a: f32, b: f32) {
    assert!(is_equal(a, b), "{a} != {b}");
}

fn near(a: f32, b: f32) {
    assert!((a - b).abs() < 0.01, "{a} != {b}");
}

fn circle_home(dist_n_m: f32, dist_e_m: f32, radius_m: f32, margin_m: f32) -> FenceNeContext {
    FenceNeContext {
        fence_present: true,
        fence_enabled: TYPE_CIRCLE,
        position_ne_home_m: Some(Vector2f::new(dist_n_m, dist_e_m)),
        circle_radius_m: radius_m,
        margin_ne_m: margin_m,
        ..FenceNeContext::default()
    }
}

fn inclusion_square(pos_n_m: f32, pos_e_m: f32, half_m: f32, margin_m: f32) -> FenceNeContext {
    let h = half_m * 100.0;
    FenceNeContext {
        fence_present: true,
        fence_enabled: TYPE_POLYGON,
        position_ne_origin_m: Some(Vector2f::new(pos_n_m, pos_e_m)),
        margin_ne_m: margin_m,
        inclusion_polygon: Some(FencePolygon::from_slice(&[
            Vector2f::new(-h, -h),
            Vector2f::new(h, -h),
            Vector2f::new(h, h),
            Vector2f::new(-h, h),
        ])),
        ..FenceNeContext::default()
    }
}

#[test]
fn fence_ne_disabled_enable_bit_is_identity() {
    let avoid = Avoid::from_params(DISABLED, 0.75);
    let leftover = avoid.adjust_velocity_fence(
        0.0,
        250.0,
        Vector3f::new(500.0, 0.0, 40.0),
        0.0,
        250.0,
        0.01,
        circle_home(95.0, 0.0, 100.0, 2.0),
        AdjustVelocityZContext::default(),
    );
    almost(leftover.desired_vel_neu_cms.x, 500.0);
    almost(leftover.desired_vel_neu_cms.z, 40.0);
    assert!(!leftover.circle_limited);
}

#[test]
fn circle_fence_slide_redirects_before_margin() {
    let mut avoid = Avoid::new();
    avoid.set_enabled(STOP_AT_FENCE);
    avoid.set_behavior(BEHAVIOR_SLIDE);
    let leftover = avoid.adjust_velocity_fence(
        0.0,
        250.0,
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        250.0,
        0.01,
        circle_home(95.0, 0.0, 100.0, 2.0),
        AdjustVelocityZContext::default(),
    );
    assert!(leftover.circle_limited);
    // stop_dist = 0.5 * 500^2 / 100 = 1250. stopping_point = 10750.
    // Radial project onto the 9800 cm margin ring; float path matches the impl.
    let stopping_point = 9500.0 + 1250.0;
    let dist = stopping_point * (9800.0 / stopping_point) - 9500.0;
    let expected = Avoid::get_max_speed(0.0, ACCEL_CMSS_MAX, dist, 0.01);
    near(leftover.desired_vel_neu_cms.x, expected);
    assert!(is_zero(leftover.desired_vel_neu_cms.y));
}

#[test]
fn circle_fence_slide_safe_stop_is_identity() {
    let mut avoid = Avoid::new();
    avoid.set_enabled(STOP_AT_FENCE);
    let leftover = avoid.adjust_velocity_fence(
        0.0,
        250.0,
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        250.0,
        0.01,
        circle_home(80.0, 0.0, 100.0, 2.0),
        AdjustVelocityZContext::default(),
    );
    assert!(!leftover.circle_limited);
    almost(leftover.desired_vel_neu_cms.x, 500.0);
}

#[test]
fn circle_fence_already_breached_is_identity() {
    let mut avoid = Avoid::new();
    avoid.set_enabled(STOP_AT_FENCE);
    let mut ctx = circle_home(95.0, 0.0, 100.0, 2.0);
    ctx.fence_breaches = TYPE_CIRCLE;
    let leftover = avoid.adjust_velocity_fence(
        0.0,
        250.0,
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        250.0,
        0.01,
        ctx,
        AdjustVelocityZContext::default(),
    );
    assert!(!leftover.circle_limited);
    almost(leftover.desired_vel_neu_cms.x, 500.0);
}

#[test]
fn circle_fence_stop_zeroes_when_already_in_margin() {
    let mut avoid = Avoid::new();
    avoid.set_enabled(STOP_AT_FENCE);
    avoid.set_behavior(BEHAVIOR_STOP);
    let leftover = avoid.adjust_velocity_fence(
        0.0,
        250.0,
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        250.0,
        0.01,
        circle_home(99.0, 0.0, 100.0, 2.0),
        AdjustVelocityZContext::default(),
    );
    assert!(leftover.circle_limited);
    assert!(is_zero(leftover.desired_vel_neu_cms.x));
}

#[test]
fn circle_fence_zero_speed_skips_backup() {
    let mut avoid = Avoid::new();
    avoid.set_enabled(STOP_AT_FENCE);
    let leftover = avoid.adjust_velocity_fence(
        0.0,
        250.0,
        Vector3f::zero(),
        0.0,
        250.0,
        0.01,
        circle_home(99.0, 10.0, 100.0, 2.0),
        AdjustVelocityZContext::default(),
    );
    assert!(leftover.backup_vel_neu_cms.xy().is_zero());
}

#[test]
fn polygon_inclusion_slide_limits_toward_edge() {
    let mut avoid = Avoid::new();
    avoid.set_enabled(STOP_AT_FENCE);
    avoid.set_behavior(BEHAVIOR_SLIDE);
    // 100 m square, vehicle 1 m inside the +N edge, flying +N.
    let leftover = avoid.adjust_velocity_fence(
        0.0,
        250.0,
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        250.0,
        0.01,
        inclusion_square(99.0, 0.0, 100.0, 2.0),
        AdjustVelocityZContext::default(),
    );
    assert!(leftover.polygon_limited);
    assert!(leftover.desired_vel_neu_cms.x < 500.0);
}

#[test]
fn polygon_already_outside_is_identity() {
    let mut avoid = Avoid::new();
    avoid.set_enabled(STOP_AT_FENCE);
    let leftover = avoid.adjust_velocity_fence(
        0.0,
        250.0,
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        250.0,
        0.01,
        inclusion_square(150.0, 0.0, 100.0, 2.0),
        AdjustVelocityZContext::default(),
    );
    assert!(!leftover.polygon_limited);
    almost(leftover.desired_vel_neu_cms.x, 500.0);
}

#[test]
fn exclusion_polygon_limits_when_outside() {
    let mut avoid = Avoid::new();
    avoid.set_enabled(STOP_AT_FENCE);
    avoid.set_behavior(BEHAVIOR_SLIDE);
    let h = 20.0 * 100.0;
    let ctx = FenceNeContext {
        fence_present: true,
        fence_enabled: TYPE_POLYGON,
        position_ne_origin_m: Some(Vector2f::new(23.0, 0.0)),
        margin_ne_m: 2.0,
        exclusion_polygon: Some(FencePolygon::from_slice(&[
            Vector2f::new(-h, -h),
            Vector2f::new(h, -h),
            Vector2f::new(h, h),
            Vector2f::new(-h, h),
        ])),
        ..FenceNeContext::default()
    };
    let leftover = avoid.adjust_velocity_fence(
        0.0,
        250.0,
        Vector3f::new(-500.0, 0.0, 0.0),
        0.0,
        250.0,
        0.01,
        ctx,
        AdjustVelocityZContext::default(),
    );
    assert!(leftover.polygon_limited);
    assert!(leftover.desired_vel_neu_cms.x > -500.0);
}

#[test]
fn beacon_fence_mirrors_inclusion_polygon() {
    let mut avoid = Avoid::new();
    avoid.set_enabled(STOP_AT_BEACON_FENCE);
    avoid.set_behavior(BEHAVIOR_SLIDE);
    let mut ctx = inclusion_square(99.0, 0.0, 100.0, 2.0);
    ctx.fence_present = false;
    ctx.fence_enabled = 0;
    ctx.beacon_present = true;
    ctx.beacon_boundary = ctx.inclusion_polygon;
    ctx.inclusion_polygon = None;
    let leftover = avoid.adjust_velocity_fence(
        0.0,
        250.0,
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        250.0,
        0.01,
        ctx,
        AdjustVelocityZContext::default(),
    );
    assert!(leftover.beacon_limited);
    assert!(!leftover.polygon_limited);
    assert!(leftover.desired_vel_neu_cms.x < 500.0);
}

#[test]
fn beacon_absent_is_identity() {
    let mut avoid = Avoid::new();
    avoid.set_enabled(STOP_AT_BEACON_FENCE);
    let leftover = avoid.adjust_velocity_fence(
        0.0,
        250.0,
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        250.0,
        0.01,
        FenceNeContext {
            beacon_present: false,
            ..inclusion_square(99.0, 0.0, 100.0, 2.0)
        },
        AdjustVelocityZContext::default(),
    );
    assert!(!leftover.beacon_limited);
    almost(leftover.desired_vel_neu_cms.x, 500.0);
}

#[test]
fn inclusion_circle_slide_matches_classic_circle() {
    let mut avoid = Avoid::new();
    avoid.set_enabled(STOP_AT_FENCE);
    avoid.set_behavior(BEHAVIOR_SLIDE);
    let ctx = FenceNeContext {
        fence_present: true,
        fence_enabled: TYPE_POLYGON,
        position_ne_origin_m: Some(Vector2f::new(95.0, 0.0)),
        margin_ne_m: 2.0,
        inclusion_circle: Some(FenceCircle {
            center_ne_cm: Vector2f::zero(),
            radius_m: 100.0,
        }),
        ..FenceNeContext::default()
    };
    let leftover = avoid.adjust_velocity_fence(
        0.0,
        250.0,
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        250.0,
        0.01,
        ctx,
        AdjustVelocityZContext::default(),
    );
    assert!(leftover.poly_circle_limited);
    let stopping_point = 9500.0 + 1250.0;
    let dist = stopping_point * (9800.0 / stopping_point) - 9500.0;
    let expected = Avoid::get_max_speed(0.0, ACCEL_CMSS_MAX, dist, 0.01);
    near(leftover.desired_vel_neu_cms.x, expected);
}

#[test]
fn exclusion_circle_slide_limits_toward_center() {
    let mut avoid = Avoid::new();
    avoid.set_enabled(STOP_AT_FENCE);
    avoid.set_behavior(BEHAVIOR_SLIDE);
    let ctx = FenceNeContext {
        fence_present: true,
        fence_enabled: TYPE_POLYGON,
        position_ne_origin_m: Some(Vector2f::new(25.0, 0.0)),
        margin_ne_m: 2.0,
        exclusion_circle: Some(FenceCircle {
            center_ne_cm: Vector2f::zero(),
            radius_m: 20.0,
        }),
        ..FenceNeContext::default()
    };
    let leftover = avoid.adjust_velocity_fence(
        0.0,
        250.0,
        Vector3f::new(-500.0, 0.0, 0.0),
        0.0,
        250.0,
        0.01,
        ctx,
        AdjustVelocityZContext::default(),
    );
    assert!(leftover.poly_circle_limited);
    assert!(leftover.desired_vel_neu_cms.x > -500.0);
}

#[test]
fn adjust_velocity_wires_circle_fence_ne() {
    let mut avoid = Avoid::new();
    avoid.set_accel_max_mss(0.0);
    avoid.set_enabled(STOP_AT_FENCE);
    avoid.set_behavior(BEHAVIOR_SLIDE);
    let leftover = avoid.adjust_velocity(
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        250.0,
        0.0,
        250.0,
        0.01,
        AdjustVelocityContext {
            fence_ne: circle_home(95.0, 0.0, 100.0, 2.0),
            ..AdjustVelocityContext::default()
        },
    );
    let stopping_point = 9500.0 + 1250.0;
    let dist = stopping_point * (9800.0 / stopping_point) - 9500.0;
    let expected = Avoid::get_max_speed(0.0, ACCEL_CMSS_MAX, dist, 0.01);
    near(leftover.desired_vel_neu_cms.x, expected);
}

#[test]
fn adjust_velocity_keeps_proximity_when_fence_ne_idle() {
    let mut avoid = Avoid::new();
    avoid.set_accel_max_mss(0.0);
    avoid.set_behavior(BEHAVIOR_STOP);
    let leftover = avoid.adjust_velocity(
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        250.0,
        0.0,
        250.0,
        0.01,
        AdjustVelocityContext {
            proximity: ProximityStopContext {
                proximity_present: true,
                proximity_alt_enabled: true,
                obstacle_count: 1,
                yaw_rad: 0.0,
                obstacle_neu_cm: Some(Vector3f::new(100.0, 0.0, 0.0)),
                intersect_limit_neu_cm: None,
            },
            ..AdjustVelocityContext::default()
        },
    );
    assert!(leftover.backing_up);
    assert!(leftover.desired_vel_neu_cms.x < 0.0);
}
