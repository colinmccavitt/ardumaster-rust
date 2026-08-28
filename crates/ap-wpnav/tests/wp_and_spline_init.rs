//! `wp_and_spline_init_m` — the 4.7 enable / init surface.

use ap_math::scalar::GRAVITY_MSS;
use ap_math::vector3::Vector3f;
use ap_wpnav::{
    AttitudeJerkLimits, WpNav, WPNAV_ACCELERATION_MS, WPNAV_ACTIVE_TIMEOUT_MS, WP_ACC_Z_DEFAULT,
    WP_JERK_DEFAULT, WP_RADIUS_M_DEFAULT, WP_RADIUS_M_MIN, WP_SPD_DEFAULT, WP_SPD_DOWN_DEFAULT,
    WP_SPD_MIN, WP_SPD_UP_DEFAULT,
};

fn almost(a: f32, b: f32) {
    let d = (a - b).abs();
    assert!(d <= 1e-5, "expected {b}, got {a} (delta {d})");
}

#[test]
fn constructor_records_groupinfo_defaults() {
    let nav = WpNav::new();
    almost(nav.default_speed_ne_ms(), WP_SPD_DEFAULT);
    almost(nav.default_speed_up_ms(), WP_SPD_UP_DEFAULT);
    almost(nav.default_speed_down_ms(), WP_SPD_DOWN_DEFAULT);
    almost(nav.wp_acceleration_mss(), WPNAV_ACCELERATION_MS);
    almost(nav.accel_d_mss(), WP_ACC_Z_DEFAULT);
    almost(nav.wp_radius_m(), WP_RADIUS_M_DEFAULT);
    assert!(!nav.flags().reached_destination);
    assert!(!nav.flags().fast_waypoint);
    assert!(!nav.scurve_legs_inited());
}

#[test]
fn init_seats_origin_and_destination_on_the_stopping_point() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(12.0, -3.5, 4.0);
    nav.wp_and_spline_init_m(5.0, stop, 1_000, AttitudeJerkLimits::default());

    assert_eq!(nav.wp_destination_ned_m(), stop);
    assert_eq!(nav.wp_origin_ned_m(), stop);
    almost(nav.desired_speed_ne_ms(), 5.0);
    assert!(!nav.check_wp_speed_change());
    assert!(nav.flags().reached_destination);
    assert!(!nav.flags().fast_waypoint);
    assert!(!nav.origin_and_destination_are_terrain_alt());
    assert!(!nav.this_leg_is_spline());
    assert!(!nav.paused());
    almost(nav.track_dt_scalar(), 1.0);
    almost(nav.offset_vel_ms(), 5.0);
    almost(nav.offset_accel_mss(), 0.0);
    assert!(nav.scurve_legs_inited());
    assert!(nav.pos_control_stopping_point_inited());
}

#[test]
fn zero_speed_uses_wp_spd_and_watches_for_changes() {
    let mut nav = WpNav::new();
    nav.wp_and_spline_init_m(0.0, Vector3f::zero(), 0, AttitudeJerkLimits::default());
    almost(nav.desired_speed_ne_ms(), WP_SPD_DEFAULT);
    assert!(nav.check_wp_speed_change());
    almost(nav.offset_vel_ms(), WP_SPD_DEFAULT);
}

#[test]
fn clamps_radius_and_speed_floors() {
    let mut nav = WpNav::new();
    nav.set_wp_radius_m(0.01);
    nav.set_wp_speed_ms(0.0);
    nav.wp_and_spline_init_m(0.0, Vector3f::zero(), 0, AttitudeJerkLimits::default());
    almost(nav.wp_radius_m(), WP_RADIUS_M_MIN);
    almost(nav.default_speed_ne_ms(), WP_SPD_MIN);
    almost(nav.desired_speed_ne_ms(), WP_SPD_MIN);
}

#[test]
fn records_pos_control_speed_and_accel() {
    let mut nav = WpNav::new();
    nav.wp_and_spline_init_m(3.0, Vector3f::zero(), 0, AttitudeJerkLimits::default());
    let lim = nav.pos_speed_accel();
    almost(lim.ne_speed_ms, 3.0);
    almost(lim.ne_accel_mss, WPNAV_ACCELERATION_MS);
    almost(lim.speed_down_ms, WP_SPD_DOWN_DEFAULT);
    almost(lim.speed_up_ms, WP_SPD_UP_DEFAULT);
    almost(lim.accel_d_mss, WP_ACC_Z_DEFAULT);
}

#[test]
fn unset_jerk_falls_back_to_horizontal_accel() {
    let mut nav = WpNav::new();
    nav.set_wp_jerk_msss(0.0);
    nav.wp_and_spline_init_m(0.0, Vector3f::zero(), 0, AttitudeJerkLimits::default());
    almost(nav.scurve_jerk_max_msss(), WPNAV_ACCELERATION_MS);
}

#[test]
fn zero_attitude_rates_use_wp_jerk_and_half_snap() {
    let mut nav = WpNav::new();
    nav.wp_and_spline_init_m(0.0, Vector3f::zero(), 0, AttitudeJerkLimits::default());
    almost(nav.scurve_jerk_max_msss(), WP_JERK_DEFAULT);
    let expected_snap = (WP_JERK_DEFAULT * core::f32::consts::PI) / (2.0 * 0.1) * 0.5;
    almost(nav.scurve_snap_max_mssss(), expected_snap);
}

#[test]
fn attitude_rate_caps_jerk_below_wp_jerk() {
    let mut nav = WpNav::new();
    let attitude = AttitudeJerkLimits {
        ang_vel_roll_max_rads: 0.05,
        ang_vel_pitch_max_rads: 0.08,
        accel_roll_max_radss: 0.0,
        accel_pitch_max_radss: 0.0,
        input_tc: 0.2,
    };
    nav.wp_and_spline_init_m(0.0, Vector3f::zero(), 0, attitude);
    let jerk = 0.05 * GRAVITY_MSS;
    almost(nav.scurve_jerk_max_msss(), jerk);
    let snap = (jerk * core::f32::consts::PI) / (2.0 * 0.2) * 0.5;
    almost(nav.scurve_snap_max_mssss(), snap);
}

#[test]
fn is_active_for_two_hundred_milliseconds_after_init() {
    let mut nav = WpNav::new();
    nav.wp_and_spline_init_m(0.0, Vector3f::zero(), 1_000, AttitudeJerkLimits::default());
    assert!(nav.is_active(1_000));
    assert!(nav.is_active(1_000 + WPNAV_ACTIVE_TIMEOUT_MS - 1));
    assert!(!nav.is_active(1_000 + WPNAV_ACTIVE_TIMEOUT_MS));
}

#[test]
fn zero_accel_param_falls_back_to_wpnav_acceleration() {
    let mut nav = WpNav::new();
    nav.set_wp_accel_mss(0.0);
    almost(nav.wp_acceleration_mss(), WPNAV_ACCELERATION_MS);
    nav.wp_and_spline_init_m(1.0, Vector3f::zero(), 0, AttitudeJerkLimits::default());
    almost(nav.pos_speed_accel().ne_accel_mss, WPNAV_ACCELERATION_MS);
}
