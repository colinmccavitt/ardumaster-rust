//! NAV_WAYPOINT verify-distance / reached-wp (upstream `verify_nav_wp`).

use ap_math::location::Location;
use ap_math::Ftype;
use ap_mission::{verify_nav_wp, VerifyNavWpInputs, WP_RADIUS_DEFAULT_M};

fn origin() -> Location {
    Location::new(-35_000_000, 149_000_000)
}

fn north_of(base: Location, metres: Ftype) -> Location {
    let mut loc = base;
    loc.offset(metres, Ftype::from(0));
    loc
}

#[test]
fn reached_when_distance_is_within_wp_radius() {
    let prev = origin();
    let next = north_of(prev, 200.0);
    let current = north_of(prev, 180.0);
    assert!(
        verify_nav_wp(&VerifyNavWpInputs {
            current_loc: current,
            next_wp: next,
            prev_wp: prev,
            wp_radius_m: WP_RADIUS_DEFAULT_M,
        }),
        "20 m from the waypoint is inside WP_RADIUS 90"
    );
}

#[test]
fn not_reached_when_short_of_wp_radius_and_not_past_the_line() {
    let prev = origin();
    let next = north_of(prev, 400.0);
    let current = north_of(prev, 200.0);
    assert!(
        !verify_nav_wp(&VerifyNavWpInputs {
            current_loc: current,
            next_wp: next,
            prev_wp: prev,
            wp_radius_m: 90.0,
        }),
        "200 m short of a 400 m leg is outside WP_RADIUS and not past the line"
    );
}

#[test]
fn reached_when_flown_past_the_finish_line() {
    let prev = origin();
    let next = north_of(prev, 200.0);
    let current = north_of(prev, 280.0);
    assert!(
        verify_nav_wp(&VerifyNavWpInputs {
            current_loc: current,
            next_wp: next,
            prev_wp: prev,
            wp_radius_m: 30.0,
        }),
        "80 m past the waypoint completes even when outside a tight radius"
    );
}

#[test]
fn on_top_of_the_waypoint_is_reached() {
    let prev = origin();
    let next = north_of(prev, 150.0);
    assert!(verify_nav_wp(&VerifyNavWpInputs {
        current_loc: next,
        next_wp: next,
        prev_wp: prev,
        wp_radius_m: 90.0,
    }));
}
