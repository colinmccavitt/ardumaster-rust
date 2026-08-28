//! Remaining AC_WPNav leftovers: next dest, Location wrappers, terrain,
//! stopping-point conversions, and `force_stop_at_next_wp`.

use ap_math::location::{AltContext, AltFrame, Location};
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;
use ap_wpnav::{
    AttitudeJerkLimits, GetTerrainContext, GetVectorNedContext, SetWpDestinationContext,
    TerrainSource, WpNav,
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

fn ctx_at(now_ms: u32, stopping_point_ned_m: Vector3f) -> SetWpDestinationContext {
    SetWpDestinationContext {
        now_ms,
        attitude: AttitudeJerkLimits::default(),
        stopping_point_ned_m,
        terrain_d_m: None,
    }
}

fn origin_loc() -> Location {
    Location::new(35_0000_000, -1_100_000_000)
}

fn vec_ctx_origin_alt() -> GetVectorNedContext {
    GetVectorNedContext {
        origin: origin_loc(),
        alt: AltContext {
            origin_alt_cm: Some(0),
            ..AltContext::default()
        },
    }
}

#[test]
fn next_dest_preloads_fast_waypoint_and_scurve_leftover() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(1.0, 2.0, 0.0);
    nav.wp_and_spline_init_m(5.0, stop, 0, AttitudeJerkLimits::default());

    let dest = Vector3f::new(11.0, 2.0, 0.0);
    assert!(nav.set_wp_destination_ned_m(dest, false, 0.0, ctx_at(0, stop)));
    assert!(!nav.flags().fast_waypoint);
    assert!(!nav.scurve_next_leg_calculated());

    let next = Vector3f::new(20.0, 8.0, 1.0);
    assert!(nav.set_wp_destination_next_ned_m(next, false, 0.4));

    assert!(nav.flags().fast_waypoint);
    assert!(nav.scurve_next_leg_calculated());
    almost(nav.last_next_arc_rad(), 0.4);
    assert!(!nav.next_leg_is_spline());
    assert!(!nav.this_leg_is_spline());
    assert!(!nav.need_this_leg_dest_speed_max());
    assert_eq!(nav.next_destination_ned_m(), next);
    assert_eq!(nav.wp_destination_ned_m(), dest);
    let limits = nav.update_track_with_speed_accel_limits();
    assert!(limits.need_this_scurve_speed_max);
    assert!(!limits.need_this_spline_speed_accel);
    assert!(limits.need_next_scurve_speed_max);
    assert!(!limits.need_next_spline_speed_accel);
}

#[test]
fn next_dest_after_spline_records_speed_handoff() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, 0.0);
    nav.wp_and_spline_init_m(4.0, stop, 1_000, AttitudeJerkLimits::default());

    let dest = Vector3f::new(10.0, 0.0, 0.0);
    let look = Vector3f::new(18.0, 6.0, 0.0);
    assert!(nav.set_spline_destination_ned_m(dest, false, look, false, true, ctx_at(1_000, stop)));
    assert!(nav.this_leg_is_spline());

    let next = Vector3f::new(28.0, 6.0, 0.0);
    assert!(nav.set_wp_destination_next_ned_m(next, false, 0.0));

    assert!(nav.need_this_leg_dest_speed_max());
    assert!(nav.scurve_next_leg_calculated());
    assert!(!nav.next_leg_is_spline());
    assert!(nav.flags().fast_waypoint);
    assert_eq!(nav.next_destination_ned_m(), next);
}

#[test]
fn mismatched_next_terrain_skips_without_changing_state() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, -2.0);
    nav.wp_and_spline_init_m(3.0, stop, 0, AttitudeJerkLimits::default());
    let dest = Vector3f::new(6.0, 0.0, -2.0);
    assert!(nav.set_wp_destination_ned_m(dest, false, 0.0, ctx_at(0, stop)));

    assert!(nav.set_wp_destination_next_ned_m(Vector3f::new(12.0, 0.0, -1.0), true, 0.2));

    assert!(!nav.flags().fast_waypoint);
    assert!(!nav.scurve_next_leg_calculated());
    almost(nav.last_next_arc_rad(), 0.0);
    assert_eq!(nav.next_destination_ned_m(), Vector3f::zero());
    assert_eq!(nav.wp_destination_ned_m(), dest);
}

#[test]
fn force_stop_clears_fast_and_records_scurve_leftover() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, 0.0);
    nav.wp_and_spline_init_m(4.0, stop, 0, AttitudeJerkLimits::default());
    assert!(nav.set_wp_destination_ned_m(
        Vector3f::new(8.0, 0.0, 0.0),
        false,
        0.0,
        ctx_at(0, stop)
    ));
    assert!(!nav.force_stop_at_next_wp());

    assert!(nav.set_wp_destination_next_ned_m(Vector3f::new(16.0, 0.0, 0.0), false, 0.0));
    assert!(nav.flags().fast_waypoint);
    assert!(nav.scurve_next_leg_calculated());

    assert!(nav.force_stop_at_next_wp());
    assert!(!nav.flags().fast_waypoint);
    assert!(nav.need_this_leg_dest_speed_max_zero());
    assert!(nav.need_next_scurve_init());
    assert!(!nav.scurve_next_leg_calculated());
    // C++ does not clear the stored next destination.
    assert_eq!(nav.next_destination_ned_m(), Vector3f::new(16.0, 0.0, 0.0));
}

#[test]
fn force_stop_after_spline_next_skips_next_scurve_init() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, 0.0);
    nav.wp_and_spline_init_m(4.0, stop, 0, AttitudeJerkLimits::default());
    assert!(nav.set_wp_destination_ned_m(
        Vector3f::new(8.0, 0.0, 0.0),
        false,
        0.0,
        ctx_at(0, stop)
    ));
    assert!(nav.set_spline_destination_next_ned_m(
        Vector3f::new(16.0, 4.0, 0.0),
        false,
        Vector3f::new(24.0, 4.0, 0.0),
        false,
        false
    ));
    assert!(nav.next_leg_is_spline());
    assert!(nav.flags().fast_waypoint);

    assert!(nav.force_stop_at_next_wp());
    assert!(!nav.flags().fast_waypoint);
    assert!(nav.need_this_leg_dest_speed_max_zero());
    assert!(!nav.need_next_scurve_init());
    assert!(nav.next_leg_is_spline());
}

#[test]
fn stopping_point_wrappers_convert_poscontrol_leftover() {
    let leftover = Vector3f::new(3.0, -1.5, 0.25);
    almost_vec(WpNav::get_wp_stopping_point_ned_m(leftover), leftover);

    let ne = WpNav::get_wp_stopping_point_ne_m(leftover);
    almost(ne.x, 3.0);
    almost(ne.y, -1.5);

    let ne_cm = WpNav::get_wp_stopping_point_ne_cm(leftover);
    almost(ne_cm.x, 300.0);
    almost(ne_cm.y, -150.0);

    let neu_cm = WpNav::get_wp_stopping_point_neu_cm(leftover);
    almost(neu_cm.x, 300.0);
    almost(neu_cm.y, -150.0);
    almost(neu_cm.z, -25.0);
    let _unused: Vector2f = ne;
}

#[test]
fn terrain_source_prefers_rangefinder_then_database() {
    let mut nav = WpNav::new();
    assert_eq!(nav.get_terrain_source(false), TerrainSource::Unavailable);
    assert_eq!(
        nav.get_terrain_source(true),
        TerrainSource::FromTerrainDatabase
    );
    assert!(nav.rangefinder_used());
    assert!(!nav.rangefinder_used_and_healthy());

    nav.set_rangefinder_terrain_u_cm(true, true, 250.0);
    assert_eq!(nav.get_terrain_source(true), TerrainSource::FromRangefinder);
    almost(
        nav.get_terrain_u_m(GetTerrainContext::default()).unwrap(),
        2.5,
    );
    almost(
        nav.get_terrain_d_m(GetTerrainContext::default()).unwrap(),
        -2.5,
    );
    assert!(nav.rangefinder_used_and_healthy());

    nav.set_rangefinder_terrain_u_m(true, false, 4.0);
    assert!(nav.get_terrain_u_m(GetTerrainContext::default()).is_none());

    nav.set_rangefinder_use(false);
    let db = GetTerrainContext {
        terrain_database_enabled: true,
        terrain_database_u_m: Some(7.0),
    };
    assert_eq!(
        nav.get_terrain_source(true),
        TerrainSource::FromTerrainDatabase
    );
    almost(nav.get_terrain_u_m(db).unwrap(), 7.0);
    almost(nav.get_terrain_d_m(db).unwrap(), -7.0);
    assert!(nav
        .get_terrain_u_m(GetTerrainContext {
            terrain_database_enabled: true,
            terrain_database_u_m: None,
        })
        .is_none());

    nav.set_terrain_margin_m(0.01);
    almost(nav.terrain_margin_m(), 0.1);
    nav.set_terrain_margin_m(12.0);
    almost(nav.terrain_margin_m(), 12.0);
}

#[test]
fn get_vector_ned_m_origin_and_terrain_frames() {
    let origin = origin_loc();
    let dest = Location::new_with_alt(origin.lat + 1_000, origin.lng, 500, AltFrame::AboveOrigin);
    let ctx = vec_ctx_origin_alt();
    let (ned, is_terrain) = WpNav::get_vector_ned_m(dest, ctx).expect("vector");
    assert!(!is_terrain);
    almost(ned.y, 0.0);
    almost(ned.z, -5.0);
    assert!(ned.x > 10.0);

    let terr = Location::new_with_alt(origin.lat, origin.lng, 800, AltFrame::AboveTerrain);
    let (ned_t, is_terrain_t) = WpNav::get_vector_ned_m(terr, ctx).expect("terrain vector");
    assert!(is_terrain_t);
    almost(ned_t.x, 0.0);
    almost(ned_t.y, 0.0);
    almost(ned_t.z, -8.0);

    let unset = GetVectorNedContext::default();
    assert!(WpNav::get_vector_ned_m(dest, unset).is_none());
}

#[test]
fn location_wrappers_set_and_read_destination() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, 0.0);
    nav.wp_and_spline_init_m(4.0, stop, 0, AttitudeJerkLimits::default());

    let origin = origin_loc();
    let dest = Location::new_with_alt(origin.lat + 2_000, origin.lng, 300, AltFrame::AboveOrigin);
    let vec_ctx = vec_ctx_origin_alt();
    assert!(nav.set_wp_destination_loc(dest, 0.15, vec_ctx, ctx_at(0, stop)));
    assert!(!nav.origin_and_destination_are_terrain_alt());
    almost(nav.last_arc_rad(), 0.15);
    almost(nav.wp_destination_ned_m().z, -3.0);

    let next = Location::new_with_alt(origin.lat + 4_000, origin.lng, 300, AltFrame::AboveOrigin);
    assert!(nav.set_wp_destination_next_loc(next, 0.05, vec_ctx));
    assert!(nav.flags().fast_waypoint);
    assert!(nav.scurve_next_leg_calculated());
    almost(nav.last_next_arc_rad(), 0.05);
    almost(nav.next_destination_ned_m().z, -3.0);

    let got = nav.get_wp_destination_loc(origin).expect("dest loc");
    assert_eq!(got.alt_frame(), AltFrame::AboveOrigin);
    assert_eq!(got.alt, 300);
    assert_eq!(got.lng, origin.lng);
    assert!(got.lat > origin.lat);

    assert!(nav.get_wp_destination_loc(Location::new(0, 0)).is_none());
}

#[test]
fn spline_location_wrappers_reuse_ned_setters() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, 0.0);
    nav.wp_and_spline_init_m(4.0, stop, 1_000, AttitudeJerkLimits::default());

    let origin = origin_loc();
    let dest = Location::new_with_alt(origin.lat + 1_000, origin.lng, 0, AltFrame::AboveOrigin);
    let next = Location::new_with_alt(origin.lat + 2_000, origin.lng, 0, AltFrame::AboveOrigin);
    let vec_ctx = vec_ctx_origin_alt();
    assert!(nav.set_spline_destination_loc(dest, next, true, vec_ctx, ctx_at(1_000, stop)));
    assert!(nav.this_leg_is_spline());
    assert!(nav.spline_this_leg_set());

    let next_next =
        Location::new_with_alt(origin.lat + 3_000, origin.lng, 0, AltFrame::AboveOrigin);
    assert!(nav.set_spline_destination_next_loc(next, next_next, false, vec_ctx));
    assert!(nav.next_leg_is_spline());
    assert!(nav.spline_next_leg_set());
    assert!(nav.flags().fast_waypoint);
}

#[test]
fn corner_accel_defaults_to_twice_horizontal() {
    let mut nav = WpNav::new();
    almost(
        nav.corner_acceleration_mss(),
        2.0 * nav.wp_acceleration_mss(),
    );
    nav.set_wp_accel_c_mss(3.5);
    almost(nav.corner_acceleration_mss(), 3.5);
}
