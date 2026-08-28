//! AUTOLAND mode hookup for enter gates and climb/loiter/land navigate.

use ap_plane::autoland_mode_hookup::{
    autoland_mode_nav_tick, AutolandModeNavInputs, AUTOLAND_WP_ALT_DEFAULT_M,
    AUTOLAND_WP_DIST_DEFAULT_M, STAGE_CLIMB, STAGE_LANDING, STAGE_LOITER,
};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};

fn autoland_features() -> BuildFeatures {
    BuildFeatures {
        autoland: true,
        ..BuildFeatures::default()
    }
}

fn autoland_inp() -> AutolandModeNavInputs {
    AutolandModeNavInputs {
        control_mode: ModeNumber::Autoland.as_number(),
        features: autoland_features(),
        mode_just_entered: true,
        is_flying: true,
        takeoff_direction_initialized: true,
        quadplane_available: false,
        landing_is_deepstall: false,
        terrain_alt_min_m: 0,
        need_climb: false,
        current_stage: STAGE_LOITER,
        climb_complete: false,
        loiter_to_alt_complete: false,
        wp_alt_m: AUTOLAND_WP_ALT_DEFAULT_M,
        wp_dist_m: AUTOLAND_WP_DIST_DEFAULT_M,
    }
}

#[test]
fn autoland_mode_nav_enters_loiter_when_already_high() {
    let out = autoland_mode_nav_tick(&autoland_inp());
    assert!(out.applied);
    assert!(out.started);
    assert!(!out.refused);
    assert_eq!(out.stage, STAGE_LOITER);
    assert!(out.allow_loiter);
    assert!(!out.allow_land);
    assert!(!out.apply_level_roll);
    assert!(out.next_wp_crosstrack);
    assert_eq!(out.wp_alt_m, AUTOLAND_WP_ALT_DEFAULT_M);
    assert_eq!(out.wp_dist_m, AUTOLAND_WP_DIST_DEFAULT_M);
}

#[test]
fn autoland_mode_nav_enters_climb_when_below_terrain_min() {
    let mut inp = autoland_inp();
    inp.terrain_alt_min_m = 30;
    inp.need_climb = true;
    let out = autoland_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(out.started);
    assert_eq!(out.stage, STAGE_CLIMB);
    assert!(out.allow_loiter);
    assert!(!out.allow_land);
    assert!(out.apply_level_roll);
}

#[test]
fn autoland_mode_nav_deepstall_skips_to_landing() {
    let mut inp = autoland_inp();
    inp.landing_is_deepstall = true;
    inp.terrain_alt_min_m = 30;
    inp.need_climb = true;
    let out = autoland_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(out.started);
    assert_eq!(out.stage, STAGE_LANDING);
    assert!(!out.allow_loiter);
    assert!(out.allow_land);
    assert!(!out.apply_level_roll);
}

#[test]
fn autoland_mode_nav_refuses_when_not_flying() {
    let mut inp = autoland_inp();
    inp.is_flying = false;
    let out = autoland_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(!out.started);
    assert!(out.refused);
    assert!(!out.allow_loiter);
    assert!(!out.allow_land);
}

#[test]
fn autoland_mode_nav_refuses_without_takeoff_direction() {
    let mut inp = autoland_inp();
    inp.takeoff_direction_initialized = false;
    let out = autoland_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(!out.started);
    assert!(out.refused);
}

#[test]
fn autoland_mode_nav_refuses_quadplane() {
    let mut inp = autoland_inp();
    inp.quadplane_available = true;
    let out = autoland_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(!out.started);
    assert!(out.refused);
}

#[test]
fn autoland_mode_nav_skips_without_autoland_feature() {
    let mut inp = autoland_inp();
    inp.features = BuildFeatures::default();
    let out = autoland_mode_nav_tick(&inp);
    assert!(!out.applied);
    assert!(!out.started);
    assert!(!out.refused);
    assert_eq!(out.wp_alt_m, 0);
    assert_eq!(out.wp_dist_m, 0);
}

#[test]
fn autoland_mode_nav_skips_other_modes() {
    let mut inp = autoland_inp();
    inp.control_mode = ModeNumber::Takeoff.as_number();
    let out = autoland_mode_nav_tick(&inp);
    assert!(!out.applied);
    assert!(!out.started);
    assert!(!out.allow_loiter);
    assert!(!out.allow_land);
}

#[test]
fn autoland_mode_nav_advances_climb_to_loiter() {
    let mut inp = autoland_inp();
    inp.mode_just_entered = false;
    inp.current_stage = STAGE_CLIMB;
    inp.climb_complete = true;
    let out = autoland_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(!out.started);
    assert_eq!(out.stage, STAGE_LOITER);
    assert!(out.allow_loiter);
    assert!(out.next_wp_crosstrack);
    assert!(!out.apply_level_roll);
}

#[test]
fn autoland_mode_nav_advances_loiter_to_landing() {
    let mut inp = autoland_inp();
    inp.mode_just_entered = false;
    inp.current_stage = STAGE_LOITER;
    inp.loiter_to_alt_complete = true;
    let out = autoland_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(!out.started);
    assert_eq!(out.stage, STAGE_LANDING);
    assert!(!out.allow_loiter);
    assert!(out.allow_land);
    assert!(!out.next_wp_crosstrack);
}

#[test]
fn main_loop_starts_autoland_loiter() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.features.autoland = true;
    vehicle.mode.control_mode = ModeNumber::Autoland.as_number();
    vehicle.is_flying = true;
    vehicle.takeoff_direction_initialized = true;
    vehicle.autoland_wp_alt_m = 70;
    vehicle.autoland_wp_dist_m = 350;

    vehicle.update_control_mode();

    assert!(vehicle.autoland_mode_nav_applied);
    assert!(vehicle.autoland_mode_started);
    assert!(!vehicle.autoland_mode_refused);
    assert_eq!(vehicle.autoland_stage, STAGE_LOITER);
    assert!(vehicle.autoland_mode_loiter_allowed);
    assert!(!vehicle.autoland_mode_land_allowed);
    assert!(!vehicle.autoland_apply_level_roll);
    assert!(vehicle.autoland_next_wp_crosstrack);
    assert_eq!(vehicle.autoland_wp_alt_m, 70);
    assert_eq!(vehicle.autoland_wp_dist_m, 350);
    assert!(!vehicle.takeoff_mode_nav_applied);
    assert!(!vehicle.guided_mode_nav_applied);
    assert!(!vehicle.loiter_mode_nav_applied);
    assert!(!vehicle.rtl_mode_nav_applied);
    assert!(!vehicle.auto_mode_mission_applied);
}

#[test]
fn main_loop_refuses_autoland_when_not_flying() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.features.autoland = true;
    vehicle.mode.control_mode = ModeNumber::Autoland.as_number();
    vehicle.is_flying = false;
    vehicle.takeoff_direction_initialized = true;

    vehicle.update_control_mode();

    assert!(vehicle.autoland_mode_nav_applied);
    assert!(!vehicle.autoland_mode_started);
    assert!(vehicle.autoland_mode_refused);
    assert!(!vehicle.autoland_mode_loiter_allowed);
    assert!(!vehicle.autoland_mode_land_allowed);
}

#[test]
fn main_loop_resumes_autoland_climb_then_lands() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.features.autoland = true;
    vehicle.mode.control_mode = ModeNumber::Autoland.as_number();
    vehicle.tracked_control_mode = ModeNumber::Autoland.as_number();
    vehicle.is_flying = true;
    vehicle.takeoff_direction_initialized = true;
    vehicle.autoland_stage = STAGE_CLIMB;
    vehicle.autoland_climb_complete = true;

    vehicle.update_control_mode();

    assert!(vehicle.autoland_mode_nav_applied);
    assert!(!vehicle.autoland_mode_started);
    assert_eq!(vehicle.autoland_stage, STAGE_LOITER);
    assert!(vehicle.autoland_mode_loiter_allowed);
    assert!(vehicle.autoland_next_wp_crosstrack);
    assert!(!vehicle.autoland_apply_level_roll);
}
