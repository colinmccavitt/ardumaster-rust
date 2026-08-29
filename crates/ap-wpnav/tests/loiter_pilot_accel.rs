//! AC_Loiter pilot-accel leftover.

use ap_math::scalar::{cd_to_rad, rad_to_cd, wrap_pi, GRAVITY_MSS};
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;
use ap_wpnav::{InitTargetContext, Loiter, LoiterOption, PilotAccelContext, UpdateLoiterContext};

fn lean_xy(roll: f32, pitch: f32, yaw: f32) -> Vector2f {
    let (sin_roll, cos_roll) = (roll.sin(), roll.cos());
    let (sin_pitch, cos_pitch) = (pitch.sin(), pitch.cos());
    let (sin_yaw, cos_yaw) = (yaw.sin(), yaw.cos());
    let divisor = (cos_roll * cos_pitch).max(0.1);
    Vector2f::new(
        GRAVITY_MSS * (-cos_yaw * sin_pitch * cos_roll - sin_yaw * sin_roll) / divisor,
        GRAVITY_MSS * (-sin_yaw * sin_pitch * cos_roll + cos_yaw * sin_roll) / divisor,
    )
}

fn almost(a: f32, b: f32) {
    let d = (a - b).abs();
    assert!(d <= 1e-5, "expected {b}, got {a} (delta {d})");
}

fn almost_vec(got: Vector2f, expected: Vector2f) {
    almost(got.x, expected.x);
    almost(got.y, expected.y);
}

fn shaping_off_no_turn() -> (Loiter, PilotAccelContext) {
    let mut loiter = Loiter::new();
    loiter.set_options(0);
    (loiter, PilotAccelContext::default())
}

#[test]
fn zero_stick_does_not_reset_brake_timer() {
    let (mut loiter, mut ctx) = shaping_off_no_turn();
    ctx.now_ms = 1_500;
    loiter.set_pilot_desired_acceleration_rad(0.0, 0.0, ctx);
    assert_eq!(loiter.brake_timer_ms(), 0);
    almost_vec(
        loiter.get_pilot_desired_acceleration_ne_mss(),
        Vector2f::zero(),
    );
    almost_vec(loiter.predicted_accel_ne_mss(), Vector2f::zero());
    almost_vec(loiter.predicted_euler_rate(), Vector2f::zero());
}

#[test]
fn lean_writes_accel_and_resets_brake_timer() {
    let (mut loiter, mut ctx) = shaping_off_no_turn();
    ctx.now_ms = 250;
    ctx.yaw_rad = 0.0;
    loiter.set_pilot_desired_acceleration_rad(0.2, -0.1, ctx);

    let desired = lean_xy(0.2, -0.1, 0.0);
    almost_vec(loiter.desired_accel_ne_mss(), desired);
    almost_vec(loiter.get_pilot_desired_acceleration_ne_mss(), desired);
    assert_eq!(loiter.brake_timer_ms(), 250);
    assert!(!desired.is_zero());

    let expected_rate = Vector2f::new(4.5 * wrap_pi(0.2), 4.5 * wrap_pi(-0.1));
    almost_vec(loiter.predicted_euler_rate(), expected_rate);
    almost_vec(loiter.predicted_euler_angle_rad(), expected_rate * ctx.dt_s);

    let predicted = lean_xy(
        loiter.predicted_euler_angle_rad().x,
        loiter.predicted_euler_angle_rad().y,
        0.0,
    );
    almost_vec(loiter.predicted_accel_ne_mss(), predicted);
}

#[test]
fn cd_wrapper_matches_radian_path() {
    let (mut a, ctx) = shaping_off_no_turn();
    let (mut b, _) = shaping_off_no_turn();
    a.set_pilot_desired_acceleration_rad(0.15, -0.08, ctx);
    b.set_pilot_desired_acceleration_cd(rad_to_cd(0.15), rad_to_cd(-0.08), ctx);
    almost_vec(a.desired_accel_ne_mss(), b.desired_accel_ne_mss());
    almost_vec(a.predicted_euler_angle_rad(), b.predicted_euler_angle_rad());
    almost(cd_to_rad(rad_to_cd(0.15)), 0.15);
}

#[test]
fn coordinated_turn_adds_yaw_rate_feed_forward() {
    let mut loiter = Loiter::new();
    assert!(loiter.loiter_option_is_set(LoiterOption::CoordinatedTurnEnabled));
    let ctx = PilotAccelContext {
        vel_desired_ned_ms: Vector3f::new(2.0, -1.0, 0.0),
        target_ang_vel_z_rads: 0.5,
        ..PilotAccelContext::default()
    };
    loiter.set_pilot_desired_acceleration_rad(0.0, 0.0, ctx);
    let turn = Vector2f::new(-(-1.0) * 0.5, 2.0 * 0.5);
    almost_vec(loiter.desired_accel_ne_mss(), turn);
    almost_vec(loiter.predicted_accel_ne_mss(), turn);
    assert_eq!(loiter.brake_timer_ms(), 0);
}

#[test]
fn clear_centres_sticks_without_touching_brake_timer() {
    let (mut loiter, mut ctx) = shaping_off_no_turn();
    ctx.now_ms = 400;
    loiter.set_pilot_desired_acceleration_rad(0.25, 0.0, ctx);
    assert_eq!(loiter.brake_timer_ms(), 400);

    ctx.now_ms = 800;
    loiter.clear_pilot_desired_acceleration(ctx);
    almost_vec(
        loiter.get_pilot_desired_acceleration_ne_mss(),
        Vector2f::zero(),
    );
    assert_eq!(loiter.brake_timer_ms(), 400);
}

#[test]
fn stick_input_delays_braking_on_the_next_update() {
    let (mut loiter, mut ctx) = shaping_off_no_turn();
    loiter.init_target(InitTargetContext {
        lean_angle_max_rad: 0.5,
        ..InitTargetContext::default()
    });
    ctx.now_ms = 100;
    loiter.set_pilot_desired_acceleration_rad(0.2, 0.0, ctx);

    let leftover = loiter.update(UpdateLoiterContext {
        now_ms: 100,
        dt_s: 0.01,
        vel_desired_ne_ms: Vector2f::new(5.0, 0.0),
        avoidance_on: false,
        ..UpdateLoiterContext::default()
    });
    almost(loiter.brake_accel_mss(), 0.0);
    assert!(leftover.vel_desired_ne_ms.x > 4.8);
}

#[test]
fn get_angle_max_cd_matches_radian_path() {
    let loiter = Loiter::new();
    almost(loiter.get_angle_max_cd(0.6, 0.3), rad_to_cd(loiter.get_angle_max_rad(0.6, 0.3)));
}
