//! `ModeAuto::takeoff_start` leftover, upstream `ArduCopter/mode_auto.cpp`.

use ap_copter::mode_auto::{auto_takeoff_start, AutoSubMode, AutoTakeoffStartView};

fn bits(v: f32) -> u32 {
    v.to_bits()
}

#[test]
fn uninitialised_current_loc_is_a_flow_of_control_refuse() {
    let out = auto_takeoff_start(&AutoTakeoffStartView::uninitialised());
    assert!(!out.ok);
    assert!(out.flow_of_control_error);
    assert!(!out.missing_terrain_data);
    assert_eq!(bits(out.alt_target_m), bits(0.0));
    assert!(!out.alt_target_terrain);
    assert!(!out.yaw_hold);
    assert!(!out.d_init_controller);
    assert!(!out.auto_takeoff_start);
    assert_eq!(out.submode, None);
}

#[test]
fn origin_frame_airborne_uses_converted_alt_and_parks_in_takeoff() {
    let out = auto_takeoff_start(&AutoTakeoffStartView::origin_airborne());
    assert!(out.ok);
    assert!(!out.flow_of_control_error);
    assert!(!out.missing_terrain_data);
    assert_eq!(bits(out.alt_target_m), bits(10.0));
    assert!(!out.alt_target_terrain);
    assert!(out.yaw_hold);
    assert!(out.d_init_controller);
    assert!(out.auto_takeoff_start);
    assert_eq!(out.submode, Some(AutoSubMode::Takeoff));
}

#[test]
fn landed_floors_the_target_one_metre_above_current() {
    let mut view = AutoTakeoffStartView::landed();
    view.origin_alt_m = Some(0.2);
    let out = auto_takeoff_start(&view);
    assert!(out.ok);
    assert_eq!(bits(out.alt_target_m), bits(1.0));
    assert_eq!(out.submode, Some(AutoSubMode::Takeoff));
}

#[test]
fn landed_dest_above_the_floor_is_kept() {
    let out = auto_takeoff_start(&AutoTakeoffStartView::landed());
    assert!(out.ok);
    assert_eq!(bits(out.alt_target_m), bits(10.0));
}

#[test]
fn airborne_floors_at_current_without_the_extra_metre() {
    let mut view = AutoTakeoffStartView::origin_airborne();
    view.current_alt_m = 12.0;
    view.origin_alt_m = Some(8.0);
    let out = auto_takeoff_start(&view);
    assert_eq!(bits(out.alt_target_m), bits(12.0));
    assert!(!out.alt_target_terrain);
}

#[test]
fn terrain_dest_with_offset_converts_to_alt_above_terrain() {
    let view = AutoTakeoffStartView {
        current_loc_initialised: true,
        dest_alt_frame_terrain: true,
        terrain_u_m: Some(50.0),
        current_alt_m: 52.0,
        dest_alt_cm: 1_500,
        origin_alt_m: None,
        land_complete: false,
    };
    let out = auto_takeoff_start(&view);
    assert!(out.ok);
    assert!(!out.missing_terrain_data);
    // current becomes 52 - 50 = 2; dest 15 m; floor is 2.
    assert_eq!(bits(out.alt_target_m), bits(15.0));
    assert!(out.alt_target_terrain);
    assert_eq!(out.submode, Some(AutoSubMode::Takeoff));
}

#[test]
fn terrain_dest_floors_against_terrain_relative_current() {
    let view = AutoTakeoffStartView {
        current_loc_initialised: true,
        dest_alt_frame_terrain: true,
        terrain_u_m: Some(50.0),
        current_alt_m: 52.0,
        dest_alt_cm: 50,
        origin_alt_m: None,
        land_complete: false,
    };
    let out = auto_takeoff_start(&view);
    // dest 0.5 m, terrain-relative current 2.0 → floor 2.0
    assert_eq!(bits(out.alt_target_m), bits(2.0));
    assert!(out.alt_target_terrain);
}

#[test]
fn terrain_dest_without_terrain_logs_and_falls_back() {
    let view = AutoTakeoffStartView {
        current_loc_initialised: true,
        dest_alt_frame_terrain: true,
        terrain_u_m: None,
        current_alt_m: 2.0,
        dest_alt_cm: 1_000,
        origin_alt_m: None,
        land_complete: false,
    };
    let out = auto_takeoff_start(&view);
    assert!(out.ok);
    assert!(out.missing_terrain_data);
    assert_eq!(bits(out.alt_target_m), bits(12.0));
    assert!(!out.alt_target_terrain);
    assert!(out.auto_takeoff_start);
}

#[test]
fn origin_conversion_failure_also_falls_back() {
    let mut view = AutoTakeoffStartView::origin_airborne();
    view.origin_alt_m = None;
    let out = auto_takeoff_start(&view);
    assert!(out.missing_terrain_data);
    assert_eq!(bits(out.alt_target_m), bits(12.0));
    assert!(!out.alt_target_terrain);
}

#[test]
fn terrain_available_is_ignored_unless_dest_is_terrain_frame() {
    let mut view = AutoTakeoffStartView::origin_airborne();
    view.terrain_u_m = Some(50.0);
    let out = auto_takeoff_start(&view);
    assert_eq!(bits(out.alt_target_m), bits(10.0));
    assert!(!out.alt_target_terrain);
    assert!(!out.missing_terrain_data);
}
