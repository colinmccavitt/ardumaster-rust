//! Fence-aware `adjust_velocity_z` leftover and the compiled-out
//! climb-rate identity PosHold already documents.
//!
//! Tracked as **COP-026**. Proximity / horizontal `adjust_velocity` stay
//! later.

use ap_avoidance::{
    get_avoidance_adjusted_climbrate_ms, AdjustVelocityZContext, Avoid, ACCEL_CMSS_MAX,
    AVOID_DEFAULT, BACKUP_SPEED_MAX_U_MS_DEFAULT, DISABLED, STOP_AT_BEACON_FENCE, STOP_AT_FENCE,
    USE_PROXIMITY_SENSOR,
};
use ap_fence::{TYPE_ALT_MAX, TYPE_ALT_MIN};
use ap_math::control::sqrt_controller;
use ap_math::scalar::{is_equal, safe_sqrt};

fn almost(a: f32, b: f32) {
    assert!(is_equal(a, b), "{a} != {b}");
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
fn enable_bits_match_upstream() {
    assert_eq!(DISABLED, 0);
    assert_eq!(STOP_AT_FENCE, 1);
    assert_eq!(USE_PROXIMITY_SENSOR, 2);
    assert_eq!(STOP_AT_BEACON_FENCE, 4);
    assert_eq!(AVOID_DEFAULT, STOP_AT_FENCE | USE_PROXIMITY_SENSOR);
    assert_eq!(ACCEL_CMSS_MAX, 100.0);
    assert_eq!(BACKUP_SPEED_MAX_U_MS_DEFAULT, 0.75);

    let on = Avoid::new();
    assert!(on.enabled());
    assert_eq!(on.enabled_bits(), AVOID_DEFAULT);
    almost(on.backup_speed_max_u_ms(), BACKUP_SPEED_MAX_U_MS_DEFAULT);

    let off = Avoid::from_params(DISABLED, 0.0);
    assert!(!off.enabled());
}

#[test]
fn get_max_speed_linear_and_sqrt() {
    almost(
        Avoid::get_max_speed(0.0, 100.0, 200.0, 0.01),
        safe_sqrt(40_000.0),
    );
    almost(
        Avoid::get_max_speed(2.0, 100.0, 50.0, 0.01),
        sqrt_controller(50.0, 2.0, 100.0, 0.01),
    );
}

#[test]
fn compiled_out_climbrate_is_identity() {
    let avoid = Avoid::new();
    let ctx = alt_max_ctx(99.0, 100.0);
    let rate = get_avoidance_adjusted_climbrate_ms(false, &avoid, 0.0, 1.0, 2.5, 0.01, ctx);
    almost(rate, 2.5);
}

#[test]
fn disabled_or_level_climb_is_identity() {
    let off = Avoid::from_params(DISABLED, BACKUP_SPEED_MAX_U_MS_DEFAULT);
    let ctx = alt_max_ctx(99.0, 100.0);
    let leftover = off.adjust_velocity_z(0.0, 250.0, 400.0, 0.01, ctx);
    almost(leftover.climb_rate_cms, 400.0);
    almost(leftover.backup_speed_cms, 0.0);
    almost(leftover.climb_rate_applied_cms, 400.0);
    assert!(!leftover.limit_max_alt);

    let on = Avoid::new();
    let level = on.adjust_velocity_z(0.0, 250.0, 0.0, 0.01, ctx);
    almost(level.climb_rate_applied_cms, 0.0);
    almost(level.backup_speed_cms, 0.0);
}

#[test]
fn approaching_alt_max_limits_climb_to_stopping_speed() {
    let avoid = Avoid::new();
    // 10 m below the safe ceiling, climbing at 5 m/s. Accel is capped at 100 cm/s/s.
    let leftover = avoid.adjust_velocity_z(0.0, 250.0, 500.0, 0.01, alt_max_ctx(90.0, 100.0));
    assert!(leftover.limit_max_alt);
    let max_cms = Avoid::get_max_speed(0.0, ACCEL_CMSS_MAX, 10.0 * 100.0, 0.01);
    almost(leftover.climb_rate_cms, max_cms);
    almost(leftover.backup_speed_cms, 0.0);
    almost(leftover.climb_rate_applied_cms, max_cms);
}

#[test]
fn breached_alt_max_zeros_climb_and_backs_down() {
    let avoid = Avoid::new();
    // 0.5 m above the safe ceiling, still commanding a climb.
    let leftover = avoid.adjust_velocity_z(0.0, 250.0, 200.0, 0.01, alt_max_ctx(100.5, 100.0));
    almost(leftover.climb_rate_cms, 0.0);
    let raw_backup = -Avoid::get_max_speed(0.0, ACCEL_CMSS_MAX, 50.0, 0.01);
    let capped = raw_backup.max(-BACKUP_SPEED_MAX_U_MS_DEFAULT * 100.0);
    almost(leftover.backup_speed_cms, capped);
    almost(leftover.climb_rate_applied_cms, capped);
}

#[test]
fn approaching_alt_min_limits_descent() {
    let avoid = Avoid::new();
    let leftover = avoid.adjust_velocity_z(0.0, 250.0, -500.0, 0.01, alt_min_ctx(30.0, 20.0));
    assert!(leftover.limit_min_alt);
    let max_down = Avoid::get_max_speed(0.0, ACCEL_CMSS_MAX, 10.0 * 100.0, 0.01);
    almost(leftover.climb_rate_cms, -max_down);
    almost(leftover.climb_rate_applied_cms, -max_down);
}

#[test]
fn breached_alt_min_zeros_descent_and_backs_up() {
    let avoid = Avoid::new();
    let leftover = avoid.adjust_velocity_z(0.0, 250.0, -200.0, 0.01, alt_min_ctx(19.5, 20.0));
    almost(leftover.climb_rate_cms, 0.0);
    let raw_backup = Avoid::get_max_speed(0.0, ACCEL_CMSS_MAX, 50.0, 0.01);
    let capped = raw_backup.min(BACKUP_SPEED_MAX_U_MS_DEFAULT * 100.0);
    almost(leftover.backup_speed_cms, capped);
    almost(leftover.climb_rate_applied_cms, capped);
}

#[test]
fn stop_at_fence_off_ignores_alt_limits() {
    let avoid = Avoid::from_params(USE_PROXIMITY_SENSOR, BACKUP_SPEED_MAX_U_MS_DEFAULT);
    let leftover = avoid.adjust_velocity_z(0.0, 250.0, 500.0, 0.01, alt_max_ctx(90.0, 100.0));
    almost(leftover.climb_rate_applied_cms, 500.0);
    assert!(!leftover.limit_max_alt);
}

#[test]
fn ahrs_hgt_ctrl_limit_can_tighten_the_ceiling() {
    let avoid = Avoid::new();
    let ctx = AdjustVelocityZContext {
        fence_present: true,
        fence_enabled: TYPE_ALT_MAX,
        alt_max_u_m: Some(50.0),
        safe_alt_max_m: 100.0,
        // 2 m of headroom vs the fence's 50 m.
        hgt_ctrl_limit_m: Some(10.0),
        curr_alt_d_m: Some(-8.0),
        ..AdjustVelocityZContext::default()
    };
    let leftover = avoid.adjust_velocity_z(0.0, 250.0, 500.0, 0.01, ctx);
    let max_cms = Avoid::get_max_speed(0.0, ACCEL_CMSS_MAX, 2.0 * 100.0, 0.01);
    almost(leftover.climb_rate_applied_cms, max_cms);
}

#[test]
fn compiled_in_wrapper_converts_ms_through_adjust_velocity_z() {
    let avoid = Avoid::new();
    let ctx = alt_max_ctx(90.0, 100.0);
    let rate = get_avoidance_adjusted_climbrate_ms(true, &avoid, 0.0, 2.5, 5.0, 0.01, ctx);
    let leftover = avoid.adjust_velocity_z(0.0, 250.0, 500.0, 0.01, ctx);
    almost(rate, leftover.climb_rate_applied_cms * 0.01);
}
