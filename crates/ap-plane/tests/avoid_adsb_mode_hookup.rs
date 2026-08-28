//! AVOID_ADSB mode hookup for guided-enter loiter navigate.

use ap_plane::avoid_adsb_mode_hookup::{avoid_adsb_mode_nav_tick, AvoidAdsbModeNavInputs};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};

fn adsb_features() -> BuildFeatures {
    BuildFeatures {
        adsb: true,
        ..BuildFeatures::default()
    }
}

fn avoid_inp() -> AvoidAdsbModeNavInputs {
    AvoidAdsbModeNavInputs {
        control_mode: ModeNumber::AvoidAdsb.as_number(),
        features: adsb_features(),
        mode_just_entered: true,
        wp_loiter_rad_m: 200,
    }
}

#[test]
fn avoid_adsb_mode_nav_starts_on_enter() {
    let out = avoid_adsb_mode_nav_tick(&avoid_inp());
    assert!(out.applied);
    assert!(out.started);
    assert!(out.allow_loiter);
    assert!(out.clear_throttle_passthru);
    assert!(out.direction_set);
    assert!(!out.loiter_ccw);
    // navigate always calls update_loiter(0); WP_LOITER_RAD is the default.
    assert_eq!(out.loiter_radius_m, 0);
}

#[test]
fn avoid_adsb_mode_nav_resumes_when_already_in_avoid() {
    let mut inp = avoid_inp();
    inp.mode_just_entered = false;
    let out = avoid_adsb_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(!out.started);
    assert!(out.allow_loiter);
    assert!(!out.clear_throttle_passthru);
    assert!(out.direction_set);
    assert!(!out.loiter_ccw);
    assert_eq!(out.loiter_radius_m, 0);
}

#[test]
fn avoid_adsb_mode_nav_skips_other_modes() {
    let mut inp = avoid_inp();
    inp.control_mode = ModeNumber::Guided.as_number();
    let out = avoid_adsb_mode_nav_tick(&inp);
    assert!(!out.applied);
    assert!(!out.started);
    assert!(!out.allow_loiter);
    assert!(!out.direction_set);
    assert!(!out.clear_throttle_passthru);
    assert_eq!(out.loiter_radius_m, 0);
}

#[test]
fn avoid_adsb_mode_nav_skips_without_adsb_feature() {
    let mut inp = avoid_inp();
    inp.features = BuildFeatures::default();
    let out = avoid_adsb_mode_nav_tick(&inp);
    assert!(!out.applied);
    assert!(!out.started);
    assert!(!out.allow_loiter);
    assert!(!out.direction_set);
    assert!(!out.clear_throttle_passthru);
}

#[test]
fn avoid_adsb_mode_nav_decodes_ccw_from_wp_loiter_rad() {
    let mut inp = avoid_inp();
    inp.wp_loiter_rad_m = -150;
    let out = avoid_adsb_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(out.started);
    assert!(out.direction_set);
    assert!(out.loiter_ccw);
    assert_eq!(out.loiter_radius_m, 0);
}

#[test]
fn avoid_adsb_mode_nav_zero_wp_radius_leaves_direction() {
    let mut inp = avoid_inp();
    inp.wp_loiter_rad_m = 0;
    let out = avoid_adsb_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(!out.direction_set);
    assert!(!out.loiter_ccw);
    assert_eq!(out.loiter_radius_m, 0);
}

#[test]
fn main_loop_starts_avoid_adsb_loiter() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.features.adsb = true;
    vehicle.mode.control_mode = ModeNumber::AvoidAdsb.as_number();
    vehicle.wp_loiter_rad_m = -180;
    vehicle.guided_active_radius_m = 250;
    vehicle.guided_throttle_passthru = true;

    vehicle.update_control_mode();

    assert!(vehicle.avoid_adsb_mode_nav_applied);
    assert!(vehicle.avoid_adsb_mode_started);
    assert!(vehicle.avoid_adsb_mode_loiter_allowed);
    assert!(vehicle.avoid_adsb_loiter_ccw);
    assert_eq!(vehicle.avoid_adsb_loiter_radius_m, 0);
    assert_eq!(vehicle.guided_active_radius_m, 0);
    assert!(!vehicle.guided_throttle_passthru);
    assert!(!vehicle.guided_mode_nav_applied);
    assert!(!vehicle.loiter_mode_nav_applied);
    assert!(!vehicle.rtl_mode_nav_applied);
    assert!(!vehicle.auto_mode_mission_applied);
}

#[test]
fn main_loop_resumes_avoid_adsb_without_reenter() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.features.adsb = true;
    vehicle.mode.control_mode = ModeNumber::AvoidAdsb.as_number();
    vehicle.tracked_control_mode = ModeNumber::AvoidAdsb.as_number();
    vehicle.wp_loiter_rad_m = 90;
    vehicle.guided_throttle_passthru = true;
    vehicle.guided_active_radius_m = 90;

    vehicle.update_control_mode();

    assert!(vehicle.avoid_adsb_mode_nav_applied);
    assert!(!vehicle.avoid_adsb_mode_started);
    assert!(vehicle.avoid_adsb_mode_loiter_allowed);
    assert_eq!(vehicle.avoid_adsb_loiter_radius_m, 0);
    assert!(!vehicle.avoid_adsb_loiter_ccw);
    assert!(vehicle.guided_throttle_passthru);
    assert_eq!(vehicle.guided_active_radius_m, 90);
}

#[test]
fn main_loop_mode_14_without_adsb_uses_guided() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.features.adsb = false;
    vehicle.mode.control_mode = ModeNumber::AvoidAdsb.as_number();
    vehicle.wp_loiter_rad_m = 200;

    vehicle.update_control_mode();

    assert!(!vehicle.avoid_adsb_mode_nav_applied);
    assert!(vehicle.guided_mode_nav_applied);
    assert!(vehicle.guided_mode_started);
}
