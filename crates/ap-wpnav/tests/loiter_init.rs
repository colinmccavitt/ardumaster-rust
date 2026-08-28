//! AC_Loiter init / update leftover.

use ap_math::control::{angle_rad_to_accel_mss, sqrt_controller};
use ap_math::scalar::{constrain_value, radians, GRAVITY_MSS};
use ap_math::vector2::Vector2f;
use ap_wpnav::{
    InitTargetContext, Loiter, LoiterOption, UpdateLoiterContext, LOITER_ACCEL_MAX_DEFAULT_MSS,
    LOITER_BRAKE_ACCEL_DEFAULT_MSS, LOITER_BRAKE_JERK_DEFAULT_MSSS,
    LOITER_BRAKE_START_DELAY_DEFAULT_S, LOITER_POS_CORRECTION_MAX_M, LOITER_SPEED_DEFAULT_MS,
    LOITER_SPEED_MIN_MS, LOITER_VEL_CORRECTION_MAX_MS,
};

fn almost(a: f32, b: f32) {
    let d = (a - b).abs();
    assert!(d <= 1e-5, "expected {b}, got {a} (delta {d})");
}

fn almost_vec(got: Vector2f, expected: Vector2f) {
    almost(got.x, expected.x);
    almost(got.y, expected.y);
}

#[test]
fn constructor_records_groupinfo_defaults() {
    let loiter = Loiter::new();
    almost(loiter.speed_max_ne_ms(), LOITER_SPEED_DEFAULT_MS);
    almost(loiter.accel_max_ne_mss(), LOITER_ACCEL_MAX_DEFAULT_MSS);
    almost(loiter.angle_max_deg(), 0.0);
    assert!(loiter.loiter_option_is_set(LoiterOption::CoordinatedTurnEnabled));
    almost_vec(loiter.desired_accel_ne_mss(), Vector2f::zero());
    almost_vec(loiter.predicted_accel_ne_mss(), Vector2f::zero());
    almost(loiter.brake_accel_mss(), 0.0);
}

#[test]
fn init_target_m_zeros_state_and_records_pos_control_leftover() {
    let mut loiter = Loiter::new();
    loiter.set_accel_max_ne_mss(20.0);
    let pos = Vector2f::new(4.0, -1.5);
    let leftover = loiter.init_target_m(
        pos,
        InitTargetContext {
            lean_angle_max_rad: 0.4,
            accel_target_ne_mss: Vector2f::new(3.0, 1.0),
            roll_rad: 0.2,
            pitch_rad: -0.1,
        },
    );

    almost(leftover.correction_speed_ms, LOITER_VEL_CORRECTION_MAX_MS);
    almost(leftover.correction_accel_mss, GRAVITY_MSS * 0.4_f32.tan());
    almost(leftover.pos_error_max_m, LOITER_POS_CORRECTION_MAX_M);
    assert!(leftover.need_ne_init_controller_stopping_point);
    assert!(!leftover.need_ne_relax_velocity_controller);
    assert_eq!(leftover.pos_desired_ne_m, Some(pos));
    almost_vec(loiter.predicted_accel_ne_mss(), Vector2f::zero());
    almost_vec(loiter.desired_accel_ne_mss(), Vector2f::zero());
    almost_vec(loiter.predicted_euler_angle_rad(), Vector2f::zero());
    almost(loiter.brake_accel_mss(), 0.0);
    almost(loiter.accel_max_ne_mss(), leftover.correction_accel_mss);
}

#[test]
fn init_target_copies_pos_control_leftovers() {
    let mut loiter = Loiter::new();
    let leftover = loiter.init_target(InitTargetContext {
        lean_angle_max_rad: 0.6,
        accel_target_ne_mss: Vector2f::new(1.25, -0.5),
        roll_rad: 0.08,
        pitch_rad: -0.03,
    });

    almost(leftover.correction_speed_ms, LOITER_VEL_CORRECTION_MAX_MS);
    almost(leftover.pos_error_max_m, LOITER_POS_CORRECTION_MAX_M);
    assert!(!leftover.need_ne_init_controller_stopping_point);
    assert!(leftover.need_ne_relax_velocity_controller);
    assert!(leftover.pos_desired_ne_m.is_none());
    almost_vec(loiter.predicted_accel_ne_mss(), Vector2f::new(1.25, -0.5));
    almost_vec(
        loiter.predicted_euler_angle_rad(),
        Vector2f::new(0.08, -0.03),
    );
    almost_vec(loiter.predicted_euler_rate(), Vector2f::zero());
    almost_vec(loiter.predicted_euler_accel(), Vector2f::zero());
    almost(loiter.brake_accel_mss(), 0.0);
}

#[test]
fn sanity_check_params_floors_speed_and_clamps_accel() {
    let mut loiter = Loiter::new();
    loiter.set_speed_max_ne_ms(0.01);
    loiter.set_accel_max_ne_mss(40.0);
    loiter.sanity_check_params(0.3);
    almost(loiter.speed_max_ne_ms(), LOITER_SPEED_MIN_MS);
    almost(loiter.accel_max_ne_mss(), GRAVITY_MSS * 0.3_f32.tan());
}

#[test]
fn get_angle_max_rad_defaults_to_two_thirds() {
    let loiter = Loiter::new();
    almost(loiter.get_angle_max_rad(0.6, 0.3), 0.2);
    almost(loiter.get_angle_max_rad(0.3, 0.9), 0.2);
}

#[test]
fn get_angle_max_rad_uses_configured_min_psc() {
    let mut loiter = Loiter::new();
    loiter.set_angle_max_deg(20.0);
    almost(loiter.get_angle_max_rad(1.0, 1.0), radians(20.0));
    almost(loiter.get_angle_max_rad(1.0, 0.1), 0.1);
}

#[test]
fn set_speed_max_floors_at_min() {
    let mut loiter = Loiter::new();
    loiter.set_speed_max_ne_ms(0.05);
    almost(loiter.speed_max_ne_ms(), LOITER_SPEED_MIN_MS);
    loiter.set_speed_max_ne_ms(8.0);
    almost(loiter.speed_max_ne_ms(), 8.0);
}

#[test]
fn update_records_velocity_and_ne_controller_leftovers() {
    let mut loiter = Loiter::new();
    let leftover = loiter.update(UpdateLoiterContext::default());
    assert!(leftover.need_calc_desired_velocity);
    assert!(leftover.need_ne_update_controller);
    assert!(leftover.need_set_pos_vel_accel_ne);
    assert!(leftover.need_avoidance_adjust_velocity);
    assert!(loiter.soften_for_landing());
}

#[test]
fn update_negative_dt_skips_set_pos_vel_accel() {
    let mut loiter = Loiter::new();
    let leftover = loiter.update(UpdateLoiterContext {
        dt_s: -0.01,
        avoidance_on: true,
        ..UpdateLoiterContext::default()
    });
    assert!(leftover.need_calc_desired_velocity);
    assert!(leftover.need_ne_update_controller);
    assert!(!leftover.need_set_pos_vel_accel_ne);
    assert!(!leftover.need_avoidance_adjust_velocity);
}

#[test]
fn update_stationary_holds_the_seat() {
    let mut loiter = Loiter::new();
    let pos = Vector2f::new(10.0, -4.0);
    loiter.init_target_m(pos, InitTargetContext::default());
    let leftover = loiter.update(UpdateLoiterContext {
        pos_desired_ne_m: pos,
        vel_desired_ne_ms: Vector2f::zero(),
        avoidance_on: false,
        ..UpdateLoiterContext::default()
    });
    almost_vec(leftover.pos_desired_ne_m, pos);
    almost_vec(leftover.vel_desired_ne_ms, Vector2f::zero());
    almost_vec(leftover.accel_desired_ne_mss, Vector2f::zero());
    assert!(!leftover.need_avoidance_adjust_velocity);
}

#[test]
fn update_brakes_after_delay() {
    let mut loiter = Loiter::new();
    loiter.init_target(InitTargetContext {
        lean_angle_max_rad: 0.5,
        ..InitTargetContext::default()
    });

    let leftover = loiter.update(UpdateLoiterContext {
        now_ms: 2_000,
        dt_s: 0.01,
        ekf_gnd_spd_limit_ms: 50.0,
        vel_desired_ne_ms: Vector2f::new(5.0, 0.0),
        pos_desired_ne_m: Vector2f::zero(),
        vel_pid_kp: 1.0,
        attitude_lean_angle_max_rad: 0.5,
        pos_lean_angle_max_rad: 0.5,
        avoidance_on: false,
    });

    let gnd = LOITER_SPEED_DEFAULT_MS.min(50.0).max(LOITER_SPEED_MIN_MS);
    let angle_max = loiter.get_angle_max_rad(0.5, 0.5);
    let pilot_accel = angle_rad_to_accel_mss(angle_max);
    let drag = pilot_accel * 5.0 / gnd;
    let brake_cmd = constrain_value(
        sqrt_controller(5.0, 0.5, LOITER_BRAKE_JERK_DEFAULT_MSSS, 0.01),
        0.0,
        LOITER_BRAKE_ACCEL_DEFAULT_MSS,
    );
    let brake = constrain_value(
        brake_cmd,
        -LOITER_BRAKE_JERK_DEFAULT_MSSS * 0.01,
        LOITER_BRAKE_JERK_DEFAULT_MSSS * 0.01,
    );
    let speed = (5.0 - (drag + brake) * 0.01).max(0.0);
    almost(loiter.brake_accel_mss(), brake);
    almost(leftover.vel_desired_ne_ms.x, speed);
    almost(leftover.vel_desired_ne_ms.y, 0.0);
    almost(leftover.accel_desired_ne_mss.x, -brake);
    almost(leftover.pos_desired_ne_m.x, speed * 0.01);
    almost(leftover.pos_desired_ne_m.y, 0.0);
    let _ = LOITER_BRAKE_START_DELAY_DEFAULT_S;
}
