//! Accel-jerk leftover `AC_Avoid::limit_accel_NEU_cm`. Tracked as **COP-026**.
//!
//! Fence NE / proximity / Z stay in the other `adjust_velocity*` tests.
//! Those cases zero `AVOID_ACCEL_MAX` so this leftover can be asserted here.

use ap_avoidance::{
    AdjustVelocityContext, Avoid, ProximityStopContext, ACCEL_MAX_MSS_DEFAULT, ACCEL_TIMEOUT_MS,
    ACTIVE_LIMIT_TIMEOUT_MS, BEHAVIOR_STOP,
};
use ap_math::scalar::{is_equal, is_zero};
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

#[test]
fn limit_accel_skips_identical_velocities() {
    let mut avoid = Avoid::new();
    let v = Vector3f::new(500.0, 80.0, 40.0);
    let leftover = avoid.limit_accel_neu_cm(v, v, 0.01, 1_000);
    assert!(leftover.skipped);
    assert!(!leftover.limited);
    almost_neu(leftover.modified_vel_neu_cms, 500.0, 80.0, 40.0);
    almost_neu(avoid.prev_avoid_vel_neu_cms(), 0.0, 0.0, 0.0);
}

#[test]
fn limit_accel_skips_zero_accel_max() {
    let mut avoid = Avoid::new();
    avoid.set_accel_max_mss(0.0);
    let leftover = avoid.limit_accel_neu_cm(
        Vector3f::new(500.0, 0.0, 0.0),
        Vector3f::new(0.0, 0.0, 0.0),
        0.01,
        1_000,
    );
    assert!(leftover.skipped);
    almost_neu(leftover.modified_vel_neu_cms, 0.0, 0.0, 0.0);
    almost_neu(avoid.prev_avoid_vel_neu_cms(), 0.0, 0.0, 0.0);
}

#[test]
fn limit_accel_skips_non_positive_dt() {
    let mut avoid = Avoid::new();
    let leftover = avoid.limit_accel_neu_cm(
        Vector3f::new(500.0, 0.0, 0.0),
        Vector3f::new(0.0, 0.0, 0.0),
        0.0,
        1_000,
    );
    assert!(leftover.skipped);
    almost_neu(leftover.modified_vel_neu_cms, 0.0, 0.0, 0.0);
}

#[test]
fn limit_accel_resets_prev_after_timeout_and_pulls_back() {
    let mut avoid = Avoid::new();
    almost(avoid.accel_max_mss(), ACCEL_MAX_MSS_DEFAULT);
    // last_limit_time is 0; now - 0 > 200 resets prev to the original 500 cm/s.
    let leftover = avoid.limit_accel_neu_cm(
        Vector3f::new(500.0, 0.0, 0.0),
        Vector3f::new(0.0, 0.0, 0.0),
        0.01,
        ACCEL_TIMEOUT_MS + 1,
    );
    assert!(leftover.prev_reset);
    assert!(leftover.limited);
    // accel = (0 - 500) / 0.01 = -50000 cm/s/s; cap is 3 m/s/s = 300 cm/s/s.
    // modified = (-1, 0, 0) * 300 * 0.01 + (500, 0, 0) = (497, 0, 0).
    almost_neu(leftover.modified_vel_neu_cms, 497.0, 0.0, 0.0);
    almost_neu(avoid.prev_avoid_vel_neu_cms(), 497.0, 0.0, 0.0);
}

#[test]
fn limit_accel_uses_stored_prev_when_still_active() {
    let mut avoid = Avoid::new();
    // now - last_limit_time(0) == 0, so prev stays the default zero vector.
    let leftover = avoid.limit_accel_neu_cm(
        Vector3f::new(0.0, 0.0, 0.0),
        Vector3f::new(500.0, 0.0, 0.0),
        0.01,
        0,
    );
    assert!(!leftover.prev_reset);
    assert!(leftover.limited);
    // accel = (500 - 0) / 0.01 = 50000; cap 300; modified = (1,0,0)*300*0.01 + 0 = (3, 0, 0).
    almost_neu(leftover.modified_vel_neu_cms, 3.0, 0.0, 0.0);

    // Second step still inside the timeout: prev is now 3 cm/s.
    let leftover = avoid.limit_accel_neu_cm(
        Vector3f::new(0.0, 0.0, 0.0),
        Vector3f::new(500.0, 0.0, 0.0),
        0.01,
        50,
    );
    assert!(!leftover.prev_reset);
    assert!(leftover.limited);
    almost_neu(leftover.modified_vel_neu_cms, 6.0, 0.0, 0.0);
}

#[test]
fn limit_accel_passes_through_when_under_cap() {
    let mut avoid = Avoid::new();
    // 10 cm/s change over 0.1 s = 100 cm/s/s = 1 m/s/s, under the 3 m/s/s cap.
    let leftover = avoid.limit_accel_neu_cm(
        Vector3f::new(0.0, 0.0, 0.0),
        Vector3f::new(10.0, 0.0, 0.0),
        0.1,
        0,
    );
    assert!(!leftover.skipped);
    assert!(!leftover.limited);
    almost_neu(leftover.modified_vel_neu_cms, 10.0, 0.0, 0.0);
    almost_neu(avoid.prev_avoid_vel_neu_cms(), 10.0, 0.0, 0.0);
}

#[test]
fn limits_active_matches_upstream_window() {
    let avoid = Avoid::new();
    // last_limit_time is 0. now=0 → 0 < 500, so active (upstream boot quirk).
    assert!(avoid.limits_active(0));
    assert!(avoid.limits_active(ACTIVE_LIMIT_TIMEOUT_MS - 1));
    assert!(!avoid.limits_active(ACTIVE_LIMIT_TIMEOUT_MS));
}

#[test]
fn adjust_velocity_wires_limit_accel_after_timeout() {
    let mut avoid = Avoid::new();
    avoid.set_behavior(BEHAVIOR_STOP);
    // Zero backup caps so STOP's zeroed NE is not overwritten before the limiter.
    avoid.set_backup_speed_max_ne_ms(0.0);
    avoid.set_backup_speed_max_u_ms(0.0);
    let leftover = avoid.adjust_velocity(
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        250.0,
        0.0,
        250.0,
        0.01,
        AdjustVelocityContext {
            proximity: obstacle_ahead(50.0),
            now_ms: ACCEL_TIMEOUT_MS + 1,
            ..AdjustVelocityContext::default()
        },
    );
    // Proximity STOP zeros the body-frame NE; limiter then pulls 500 → 497.
    assert!(leftover.proximity_stopped || leftover.desired_vel_neu_cms.x < 500.0);
    assert!(leftover.accel_limited);
    almost_neu(leftover.desired_vel_neu_cms, 497.0, 0.0, 0.0);
    assert!(avoid.limits_active(ACCEL_TIMEOUT_MS + 1));
    assert_eq!(avoid.last_limit_time_ms(), ACCEL_TIMEOUT_MS + 1);
}

#[test]
fn adjust_velocity_disabled_does_not_touch_limit_state() {
    let mut avoid = Avoid::from_params(ap_avoidance::DISABLED, 0.75);
    let leftover = avoid.adjust_velocity(
        Vector3f::new(500.0, 0.0, 0.0),
        0.0,
        250.0,
        0.0,
        250.0,
        0.01,
        AdjustVelocityContext {
            now_ms: 1_000,
            ..AdjustVelocityContext::default()
        },
    );
    almost_neu(leftover.desired_vel_neu_cms, 500.0, 0.0, 0.0);
    assert!(!leftover.accel_limited);
    assert!(is_zero(avoid.last_limit_time_ms() as f32));
    almost_neu(avoid.prev_avoid_vel_neu_cms(), 0.0, 0.0, 0.0);
}
