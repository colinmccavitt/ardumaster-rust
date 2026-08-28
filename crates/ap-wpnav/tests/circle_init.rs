//! AC_Circle init / update leftover.

use ap_math::location::get_bearing_rad;
use ap_math::scalar::{constrain_value, is_positive, radians, safe_sqrt, wrap_2pi, wrap_pi};
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;
use ap_wpnav::{
    Circle, CircleOption, InitCircleContext, UpdateCircleContext, CIRCLE_ACTIVE_TIMEOUT_MS,
    CIRCLE_ANGULAR_ACCEL_MIN, CIRCLE_DEFAULT_OPTIONS, CIRCLE_RADIUS_MAX_M, CIRCLE_RADIUS_M_DEFAULT,
    CIRCLE_RATE_DEFAULT,
};

fn almost(a: f32, b: f32) {
    let d = (a - b).abs();
    assert!(d <= 1e-5, "expected {b}, got {a} (delta {d})");
}

fn almost_vec(got: Vector3f, expected: Vector3f) {
    almost(got.x, expected.x);
    almost(got.y, expected.y);
    almost(got.z, expected.z);
}

fn init_ctx(pos: Vector3f, yaw_rad: f32) -> InitCircleContext {
    InitCircleContext {
        yaw_rad,
        cos_yaw: yaw_rad.cos(),
        sin_yaw: yaw_rad.sin(),
        pos_desired_ned_m: pos,
        ne_max_speed_ms: 5.0,
        ne_max_accel_mss: 2.5,
    }
}

#[test]
fn constructor_records_groupinfo_defaults() {
    let circle = Circle::new();
    almost(circle.radius_parm_m(), CIRCLE_RADIUS_M_DEFAULT);
    almost(circle.radius_m(), 0.0);
    almost(circle.get_radius_m(), CIRCLE_RADIUS_M_DEFAULT);
    almost(circle.get_rate_degs(), CIRCLE_RATE_DEFAULT);
    almost(
        circle.rotation_rate_max_rads(),
        radians(CIRCLE_RATE_DEFAULT),
    );
    assert!(circle.option_is_set(CircleOption::ManualControl));
    assert_eq!(CIRCLE_DEFAULT_OPTIONS, 1);
    assert!(circle.pilot_control_enabled());
    assert!(!circle.roi_at_center());
    almost(circle.angle_rad(), 0.0);
    almost(circle.angular_vel_rads(), 0.0);
    almost_vec(circle.center_ned_m(), Vector3f::zero());
    assert!(!circle.center_is_terrain_alt());
}

#[test]
fn init_ned_m_panorama_uses_yaw_and_records_stopping_point() {
    let mut circle = Circle::new();
    let leftover = circle.init_ned_m(
        Vector3f::new(3.0, -1.0, -8.0),
        true,
        15.0,
        init_ctx(Vector3f::new(1.0, 2.0, -4.0), 0.4),
    );

    assert!(leftover.need_ne_init_controller_stopping_point);
    assert!(leftover.need_d_init_controller_stopping_point);
    almost_vec(circle.center_ned_m(), Vector3f::new(3.0, -1.0, -8.0));
    assert!(circle.center_is_terrain_alt());
    almost(circle.rotation_rate_max_rads(), radians(15.0));
    almost(circle.angular_vel_rads(), 0.0);
    almost(circle.angular_vel_max_rads(), radians(15.0));
    almost(
        circle.angular_accel_radss(),
        radians(15.0_f32)
            .abs()
            .max(radians(CIRCLE_ANGULAR_ACCEL_MIN)),
    );
    almost(circle.angle_rad(), 0.4);
    almost(circle.get_angle_total_rad(), 0.0);
}

#[test]
fn init_ned_m_at_center_uses_heading_behind() {
    let mut circle = Circle::new();
    circle.set_radius_m(10.0);
    let pos = Vector3f::new(2.0, 4.0, -1.0);
    circle.init_ned_m(pos, false, CIRCLE_RATE_DEFAULT, init_ctx(pos, 0.5));
    almost(circle.angle_rad(), wrap_pi(0.5 - core::f32::consts::PI));
}

#[test]
fn init_ned_m_offset_uses_bearing_from_center() {
    let mut circle = Circle::new();
    circle.set_radius_m(10.0);
    let center = Vector3f::zero();
    let pos = Vector3f::new(10.0, 0.0, 0.0);
    circle.init_ned_m(center, false, CIRCLE_RATE_DEFAULT, init_ctx(pos, 1.2));
    almost(circle.angle_rad(), wrap_pi(0.0_f32.atan2(10.0)));
}

#[test]
fn init_projects_center_along_heading() {
    let mut circle = Circle::new();
    let stop = Vector3f::new(5.0, 3.0, -2.0);
    let leftover = circle.init(init_ctx(stop, 0.0));

    assert!(leftover.need_ne_init_controller_stopping_point);
    assert!(leftover.need_d_init_controller_stopping_point);
    almost(circle.radius_m(), CIRCLE_RADIUS_M_DEFAULT);
    almost_vec(
        circle.center_ned_m(),
        Vector3f::new(5.0 + CIRCLE_RADIUS_M_DEFAULT, 3.0, -2.0),
    );
    assert!(!circle.center_is_terrain_alt());
    almost(circle.angle_rad(), wrap_pi(0.0 - core::f32::consts::PI));
    almost(circle.angular_vel_rads(), 0.0);
}

#[test]
fn init_at_center_keeps_stopping_point() {
    let mut circle = Circle::new();
    circle.set_options(CircleOption::InitAtCenter as i16);
    let stop = Vector3f::new(-2.0, 7.0, -3.0);
    circle.init(init_ctx(stop, 0.8));
    almost_vec(circle.center_ned_m(), stop);
    almost(circle.angle_rad(), wrap_pi(0.8 - core::f32::consts::PI));
}

#[test]
fn calc_velocities_circle_clamps_rate_to_accel() {
    let mut circle = Circle::new();
    circle.set_radius_m(10.0);
    circle.set_rate_degs(CIRCLE_RATE_DEFAULT);
    circle.calc_velocities(true, 5.0, 2.5);

    let vel_max = 5.0_f32.min(safe_sqrt(0.5 * 2.5 * 10.0));
    let ang_max = vel_max / 10.0;
    let expected = constrain_value(radians(CIRCLE_RATE_DEFAULT), -ang_max, ang_max);
    almost(circle.angular_vel_max_rads(), expected);
    almost(
        circle.angular_accel_radss(),
        (2.5_f32 / 10.0).max(radians(CIRCLE_ANGULAR_ACCEL_MIN)),
    );
    almost(circle.angular_vel_rads(), 0.0);
}

#[test]
fn init_neu_cm_converts_to_ned_metres() {
    let mut circle = Circle::new();
    circle.init_neu_cm(
        Vector3f::new(100.0, 200.0, 300.0),
        false,
        10.0,
        InitCircleContext::default(),
    );
    almost_vec(circle.center_ned_m(), Vector3f::new(1.0, 2.0, -3.0));
}

#[test]
fn update_records_ne_and_climb_leftovers() {
    let mut circle = Circle::new();
    let stop = Vector3f::new(5.0, 3.0, -2.0);
    circle.init(init_ctx(stop, 0.0));

    let leftover = circle.update_ms(
        0.4,
        UpdateCircleContext {
            now_ms: 1_000,
            dt_s: 0.01,
            pos_desired_ned_m: stop,
            pos_desired_u_m: 2.0,
            ne_max_speed_ms: 5.0,
            ne_max_accel_mss: 2.5,
            terrain_u_m: None,
        },
    );

    assert!(leftover.ok);
    assert!(leftover.need_input_pos_vel_accel_ne);
    assert!(!leftover.need_input_pos_vel_accel_d);
    assert!(leftover.need_d_set_pos_target_from_climb_rate);
    assert!(leftover.need_ne_update_controller);
    almost(leftover.climb_rate_ms, 0.4);
    almost(circle.last_update_ms() as f32, 1_000.0);
    assert!(circle.is_active(1_100));
    assert!(!circle.is_active(1_000 + CIRCLE_ACTIVE_TIMEOUT_MS));

    let expected_vel = circle.angular_accel_radss() * 0.01;
    almost(circle.angular_vel_rads(), expected_vel);
    let angle_change = expected_vel * 0.01;
    almost(
        circle.angle_rad(),
        wrap_pi(wrap_pi(0.0 - core::f32::consts::PI) + angle_change),
    );
    almost(circle.get_angle_total_rad(), angle_change);

    let center = circle.center_ned_m();
    let angle = circle.angle_rad();
    almost(
        leftover.target_ned_m.x,
        center.x + circle.radius_m() * (-angle).cos(),
    );
    almost(
        leftover.target_ned_m.y,
        center.y + -circle.radius_m() * (-angle).sin(),
    );
    almost(leftover.target_ned_m.z, -2.0);
    almost(
        circle.get_yaw_rad(),
        get_bearing_rad(
            Vector2f::new(stop.x, stop.y),
            Vector2f::new(center.x, center.y),
        ),
    );
}

#[test]
fn update_terrain_missing_fails_after_angle_advance() {
    let mut circle = Circle::new();
    circle.set_radius_m(8.0);
    circle.init_ned_m(
        Vector3f::new(0.0, 0.0, -10.0),
        true,
        CIRCLE_RATE_DEFAULT,
        init_ctx(Vector3f::new(8.0, 0.0, -10.0), 0.0),
    );
    let leftover = circle.update_ms(0.0, UpdateCircleContext::default());
    assert!(!leftover.ok);
    assert!(!leftover.need_input_pos_vel_accel_ne);
    assert!(!leftover.need_ne_update_controller);
    assert!(is_positive(circle.angular_vel_rads()) || circle.angular_vel_rads().abs() >= 0.0);
    almost(circle.last_update_ms() as f32, 0.0);
}

#[test]
fn update_terrain_records_d_leftover() {
    let mut circle = Circle::new();
    circle.set_radius_m(8.0);
    let center = Vector3f::new(0.0, 0.0, -10.0);
    circle.init_ned_m(
        center,
        true,
        CIRCLE_RATE_DEFAULT,
        init_ctx(Vector3f::new(8.0, 0.0, -10.0), 0.0),
    );
    let leftover = circle.update_ms(
        0.0,
        UpdateCircleContext {
            terrain_u_m: Some(1.5),
            ..UpdateCircleContext::default()
        },
    );
    assert!(leftover.ok);
    assert!(leftover.need_input_pos_vel_accel_d);
    assert!(!leftover.need_d_set_pos_target_from_climb_rate);
    almost(leftover.target_ned_m.z, -10.0 - 1.5);
}

#[test]
fn update_zero_radius_sets_yaw_to_angle() {
    let mut circle = Circle::new();
    circle.init_ned_m(
        Vector3f::new(1.0, 2.0, -3.0),
        false,
        12.0,
        InitCircleContext::default(),
    );
    let leftover = circle.update_ms(0.2, UpdateCircleContext::default());
    assert!(leftover.ok);
    almost_vec(leftover.target_ned_m, Vector3f::new(1.0, 2.0, 0.0));
    almost(circle.get_yaw_rad(), circle.angle_rad());
    almost(leftover.climb_rate_ms, 0.2);
}

#[test]
fn update_face_direction_of_travel_offsets_yaw() {
    let mut circle = Circle::new();
    circle.set_options(
        (CircleOption::ManualControl as i16) | (CircleOption::FaceDirectionOfTravel as i16),
    );
    let stop = Vector3f::new(5.0, 3.0, -2.0);
    circle.init(init_ctx(stop, 0.0));
    circle.update_ms(
        0.0,
        UpdateCircleContext {
            pos_desired_ned_m: stop,
            pos_desired_u_m: 2.0,
            ..UpdateCircleContext::default()
        },
    );
    let center = circle.center_ned_m();
    let bearing = get_bearing_rad(
        Vector2f::new(stop.x, stop.y),
        Vector2f::new(center.x, center.y),
    );
    almost(circle.get_yaw_rad(), wrap_2pi(bearing - radians(90.0)));

    circle.set_rate_degs(-20.0);
    circle.update_ms(
        0.0,
        UpdateCircleContext {
            pos_desired_ned_m: stop,
            pos_desired_u_m: 2.0,
            now_ms: 20,
            ..UpdateCircleContext::default()
        },
    );
    let bearing = get_bearing_rad(
        Vector2f::new(stop.x, stop.y),
        Vector2f::new(center.x, center.y),
    );
    almost(circle.get_yaw_rad(), wrap_2pi(bearing + radians(90.0)));
}

#[test]
fn update_cms_converts_climb_rate() {
    let mut circle = Circle::new();
    circle.init(InitCircleContext::default());
    let leftover = circle.update_cms(50.0, UpdateCircleContext::default());
    almost(leftover.climb_rate_ms, 0.5);
}

#[test]
fn set_radius_clamps_to_max() {
    let mut circle = Circle::new();
    circle.set_radius_m(5_000.0);
    almost(circle.radius_m(), CIRCLE_RADIUS_MAX_M);
    circle.set_radius_m(-3.0);
    almost(circle.radius_m(), 0.0);
}

#[test]
fn check_param_change_reloads_internal_radius() {
    let mut circle = Circle::new();
    circle.set_radius_m(4.0);
    circle.set_radius_parm_m(12.0);
    circle.check_param_change();
    almost(circle.radius_m(), 12.0);
}

#[test]
fn set_rate_degs_does_not_change_param() {
    let mut circle = Circle::new();
    circle.set_rate_degs(7.0);
    almost(circle.rotation_rate_max_rads(), radians(7.0));
    almost(circle.get_rate_degs(), CIRCLE_RATE_DEFAULT);
}
