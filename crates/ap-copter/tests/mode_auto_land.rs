//! `ModeAuto::land_start` leftover, upstream `ArduCopter/mode_auto.cpp`.

use ap_copter::mode_auto::{auto_land_start, AutoLandStartView, AutoSubMode};

fn bits(v: f32) -> u32 {
    v.to_bits()
}

#[test]
fn ready_forwards_wpnav_limits_and_parks_in_land() {
    let out = auto_land_start(&AutoLandStartView::ready());
    assert_eq!(bits(out.ne_speed_ms), bits(5.0));
    assert_eq!(bits(out.ne_accel_mss), bits(1.0));
    assert!(!out.init_ne);
    assert_eq!(bits(out.d_speed_down_ms), bits(1.5));
    assert_eq!(bits(out.d_speed_up_ms), bits(2.5));
    assert_eq!(bits(out.d_accel_mss), bits(2.5));
    assert!(!out.init_d);
    assert!(out.yaw_hold);
    assert!(out.deploy_landing_gear);
    assert!(!out.land_repo_active);
    assert!(!out.prec_land_active);
    assert_eq!(out.submode, AutoSubMode::Land);
}

#[test]
fn idle_ne_inits_without_a_position_ok_gate() {
    let mut view = AutoLandStartView::ready();
    view.ne_is_active = false;
    let out = auto_land_start(&view);
    assert!(out.init_ne);
    assert!(!out.init_d);
    assert_eq!(out.submode, AutoSubMode::Land);
}

#[test]
fn idle_d_inits() {
    let mut view = AutoLandStartView::ready();
    view.d_is_active = false;
    let out = auto_land_start(&view);
    assert!(!out.init_ne);
    assert!(out.init_d);
}

#[test]
fn both_idle_inits_both() {
    let mut view = AutoLandStartView::ready();
    view.ne_is_active = false;
    view.d_is_active = false;
    let out = auto_land_start(&view);
    assert!(out.init_ne);
    assert!(out.init_d);
    assert!(out.yaw_hold);
    assert_eq!(out.submode, AutoSubMode::Land);
}

#[test]
fn landing_gear_compiled_out_skips_deploy() {
    let mut view = AutoLandStartView::ready();
    view.landing_gear = false;
    let out = auto_land_start(&view);
    assert!(!out.deploy_landing_gear);
    assert_eq!(out.submode, AutoSubMode::Land);
}

#[test]
fn repo_and_prec_land_are_always_cleared() {
    let mut view = AutoLandStartView::ready();
    view.ne_is_active = false;
    view.d_is_active = false;
    view.landing_gear = false;
    let out = auto_land_start(&view);
    assert!(!out.land_repo_active);
    assert!(!out.prec_land_active);
}

#[test]
fn wpnav_limits_are_forwarded_unchanged() {
    let view = AutoLandStartView {
        ne_is_active: true,
        d_is_active: true,
        speed_ne_ms: 8.25,
        wp_accel_mss: 3.5,
        speed_down_ms: 0.75,
        speed_up_ms: 4.0,
        accel_d_mss: 1.25,
        landing_gear: true,
    };
    let out = auto_land_start(&view);
    assert_eq!(bits(out.ne_speed_ms), bits(8.25));
    assert_eq!(bits(out.ne_accel_mss), bits(3.5));
    assert_eq!(bits(out.d_speed_down_ms), bits(0.75));
    assert_eq!(bits(out.d_speed_up_ms), bits(4.0));
    assert_eq!(bits(out.d_accel_mss), bits(1.25));
}
