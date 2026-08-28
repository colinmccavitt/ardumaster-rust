//! Plane hookup for NAV_WAYPOINT verify-distance / reached-wp.

use ap_math::location::Location;
use ap_math::Ftype;
use ap_mission::{MavFrame, MissionCommand, FIRST_REAL_COMMAND};
use ap_plane::nav_waypoint_verify_hookup::{
    nav_waypoint_verify_tick, NavWaypointVerifyInputs,
};

fn origin() -> Location {
    Location::new(-35_000_000, 149_000_000)
}

fn north_of(base: Location, metres: Ftype) -> Location {
    let mut loc = base;
    loc.offset(metres, Ftype::from(0));
    loc
}

fn waypoint_at(loc: Location) -> MissionCommand {
    MissionCommand::waypoint(FIRST_REAL_COMMAND, MavFrame::Global, loc.lat, loc.lng, loc.alt)
}

#[test]
fn plane_marks_nav_waypoint_reached_inside_wp_radius() {
    let prev = origin();
    let next = north_of(prev, 200.0);
    let current = north_of(prev, 160.0);
    let out = nav_waypoint_verify_tick(&NavWaypointVerifyInputs {
        current_loc: current,
        prev_wp: prev,
        cmd: waypoint_at(next),
        wp_radius_m: 90.0,
    });
    assert!(out.applied);
    assert!(out.reached);
}

#[test]
fn plane_holds_nav_waypoint_until_radius_or_fly_past() {
    let prev = origin();
    let next = north_of(prev, 400.0);
    let current = north_of(prev, 200.0);
    let out = nav_waypoint_verify_tick(&NavWaypointVerifyInputs {
        current_loc: current,
        prev_wp: prev,
        cmd: waypoint_at(next),
        wp_radius_m: 90.0,
    });
    assert!(out.applied);
    assert!(!out.reached);
}

#[test]
fn plane_ignores_a_non_waypoint_command() {
    let out = nav_waypoint_verify_tick(&NavWaypointVerifyInputs::default());
    assert!(!out.applied);
    assert!(!out.reached);
}
