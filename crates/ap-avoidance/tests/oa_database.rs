//! OA database leftover. Tracked as **COP-026**.
//!
//! Vertical BendyRuler and lean-angle avoidance in non-GPS modes stay
//! later leftovers.

use ap_avoidance::{
    Database, OaDbImportance, OaDbOutputLevel, OaDbSource, QueuePushContext,
    BEAM_WIDTH_DEFAULT_DEG, DATABASE_MAX, DISTANCE_FROM_HOME_M, DIST_MAX_DEFAULT_M,
    MIN_ALT_DEFAULT_M, OUTPUT_DEFAULT, PROCESS_QUEUE_CAP, QUEUE_MAX, QUEUE_SIZE_DEFAULT,
    RADIUS_MIN_DEFAULT_M, REFRESH_MS, SIZE_DEFAULT, TIMEOUT_SECONDS_DEFAULT,
};
use ap_math::scalar::radians;
use ap_math::vector3::Vector3f;

fn origin_pos() -> Vector3f {
    Vector3f::new(10.0, 0.0, 0.0)
}

fn push_ok(db: &mut Database, pos: Vector3f, now_ms: u32, distance_m: f32, radius_m: f32) {
    db.queue_push_radius(
        pos,
        now_ms,
        distance_m,
        radius_m,
        OaDbSource::Proximity,
        0,
        &QueuePushContext::default(),
    );
}

#[test]
fn defaults_match_upstream() {
    let db = Database::new();
    assert!(db.healthy());
    assert_eq!(db.database_size(), SIZE_DEFAULT);
    assert_eq!(db.queue_size(), QUEUE_SIZE_DEFAULT);
    assert_eq!(db.expiry_seconds(), TIMEOUT_SECONDS_DEFAULT);
    assert_eq!(db.output_level(), OaDbOutputLevel::High);
    assert_eq!(OUTPUT_DEFAULT, 1);
    assert_eq!(db.database_count(), 0);
    assert_eq!(DATABASE_MAX, 100);
    assert_eq!(QUEUE_MAX, 80);
    assert_eq!(PROCESS_QUEUE_CAP, 100);
    assert_eq!(DISTANCE_FROM_HOME_M, 3.0);
    assert!((RADIUS_MIN_DEFAULT_M - 0.01).abs() < f32::EPSILON);
    assert!((DIST_MAX_DEFAULT_M - 0.0).abs() < f32::EPSILON);
    assert!((MIN_ALT_DEFAULT_M - 0.0).abs() < f32::EPSILON);
    let expected = radians(BEAM_WIDTH_DEFAULT_DEG).tan();
    assert!((db.dist_to_radius_scalar() - expected).abs() < 1e-6);
}

#[test]
fn unhealthy_when_size_zero() {
    let mut db = Database::new();
    db.set_database_size_param(0);
    db.init();
    assert!(!db.healthy());
    push_ok(&mut db, origin_pos(), 1_000, 5.0, 1.0);
    db.update(1_000);
    assert_eq!(db.database_count(), 0);

    let mut q = Database::new();
    q.set_queue_size_param(0);
    q.init();
    assert!(!q.healthy());
}

#[test]
fn queue_push_and_update_adds_item() {
    let mut db = Database::new();
    let pos = origin_pos();
    push_ok(&mut db, pos, 1_000, 8.0, 1.5);
    assert_eq!(db.queue_len(), 1);
    assert_eq!(db.database_count(), 0);
    db.update(1_000);
    assert_eq!(db.queue_len(), 0);
    assert_eq!(db.database_count(), 1);
    let item = db.get_item(0).expect("stored");
    assert!((item.pos_neu_m.x - pos.x).abs() < f32::EPSILON);
    assert!((item.radius_m - 1.5).abs() < f32::EPSILON);
    assert_eq!(item.source, OaDbSource::Proximity);
    assert_eq!(item.importance, OaDbImportance::Normal);
    assert_eq!(item.send_to_gcs, 0);
}

#[test]
fn radius_min_is_applied() {
    let mut db = Database::new();
    db.set_radius_min_m(2.0);
    db.queue_push_radius(
        origin_pos(),
        10,
        5.0,
        0.2,
        OaDbSource::Proximity,
        0,
        &QueuePushContext::default(),
    );
    db.update(10);
    let item = db.get_item(0).expect("stored");
    assert!((item.radius_m - 2.0).abs() < f32::EPSILON);
}

#[test]
fn dist_max_rejects_far_closest_point() {
    let mut db = Database::new();
    db.set_dist_max_m(5.0);
    // closest = 10 - 1 = 9 > 5
    push_ok(&mut db, origin_pos(), 10, 10.0, 1.0);
    db.update(10);
    assert_eq!(db.database_count(), 0);
}

#[test]
fn dist_max_allows_close_after_radius() {
    let mut db = Database::new();
    db.set_dist_max_m(5.0);
    // closest = 10 - 6 = 4 <= 5
    push_ok(&mut db, origin_pos(), 10, 10.0, 6.0);
    db.update(10);
    assert_eq!(db.database_count(), 1);
}

#[test]
fn dist_max_zero_disables_the_limit() {
    let mut db = Database::new();
    db.set_dist_max_m(0.0);
    push_ok(&mut db, Vector3f::new(1_000.0, 0.0, 0.0), 10, 1_000.0, 1.0);
    db.update(10);
    assert_eq!(db.database_count(), 1);
}

#[test]
fn min_alt_rejects_low_near_home() {
    let mut db = Database::new();
    db.set_min_alt_m(2.0);
    db.queue_push_radius(
        origin_pos(),
        10,
        5.0,
        1.0,
        OaDbSource::Proximity,
        0,
        &QueuePushContext::at_home(1.0),
    );
    db.update(10);
    assert_eq!(db.database_count(), 0);
}

#[test]
fn min_alt_allows_when_high_enough() {
    let mut db = Database::new();
    db.set_min_alt_m(2.0);
    db.queue_push_radius(
        origin_pos(),
        10,
        5.0,
        1.0,
        OaDbSource::Proximity,
        0,
        &QueuePushContext::at_home(2.5),
    );
    db.update(10);
    assert_eq!(db.database_count(), 1);
}

#[test]
fn min_alt_allows_when_far_from_home() {
    let mut db = Database::new();
    db.set_min_alt_m(2.0);
    db.queue_push_radius(
        origin_pos(),
        10,
        5.0,
        1.0,
        OaDbSource::Proximity,
        0,
        &QueuePushContext::away_from_home(4.0, 0.0, 0.1),
    );
    db.update(10);
    assert_eq!(db.database_count(), 1);
}

#[test]
fn min_alt_rejects_unknown_home() {
    let mut db = Database::new();
    db.set_min_alt_m(2.0);
    db.queue_push_radius(
        origin_pos(),
        10,
        5.0,
        1.0,
        OaDbSource::Proximity,
        0,
        &QueuePushContext {
            home_ned_valid: false,
            pos_ned_home_m: Vector3f::new(0.0, 0.0, -10.0),
        },
    );
    db.update(10);
    assert_eq!(db.database_count(), 0);
}

#[test]
fn min_alt_zero_disables_check() {
    let mut db = Database::new();
    db.set_min_alt_m(0.0);
    db.queue_push_radius(
        origin_pos(),
        10,
        5.0,
        1.0,
        OaDbSource::Proximity,
        0,
        &QueuePushContext {
            home_ned_valid: false,
            pos_ned_home_m: Vector3f::zero(),
        },
    );
    db.update(10);
    assert_eq!(db.database_count(), 1);
}

#[test]
fn proximity_match_refreshes_same_object() {
    let mut db = Database::new();
    let pos = origin_pos();
    push_ok(&mut db, pos, 1_000, 8.0, 1.5);
    db.update(1_000);
    push_ok(&mut db, pos + Vector3f::new(0.4, 0.0, 0.0), 2_000, 8.0, 2.0);
    db.update(2_000);
    assert_eq!(db.database_count(), 1);
    let item = db.get_item(0).expect("refreshed");
    assert_eq!(item.timestamp_ms, 2_000);
    assert!((item.radius_m - 2.0).abs() < f32::EPSILON);
    // Proximity refresh does not move the stored position.
    assert!((item.pos_neu_m.x - pos.x).abs() < f32::EPSILON);
}

#[test]
fn proximity_far_is_new_item() {
    let mut db = Database::new();
    push_ok(&mut db, Vector3f::new(0.0, 0.0, 0.0), 10, 5.0, 1.0);
    push_ok(&mut db, Vector3f::new(5.0, 0.0, 0.0), 10, 5.0, 1.0);
    db.update(10);
    assert_eq!(db.database_count(), 2);
}

#[test]
fn ais_match_by_id_updates_position() {
    let mut db = Database::new();
    db.queue_push_radius(
        Vector3f::new(1.0, 0.0, 0.0),
        1_000,
        20.0,
        8.0,
        OaDbSource::Ais,
        42,
        &QueuePushContext::default(),
    );
    db.update(1_000);
    db.queue_push_radius(
        Vector3f::new(40.0, 10.0, 0.0),
        2_000,
        20.0,
        9.0,
        OaDbSource::Ais,
        42,
        &QueuePushContext::default(),
    );
    db.update(2_000);
    assert_eq!(db.database_count(), 1);
    let item = db.get_item(0).expect("ais");
    assert!((item.pos_neu_m.x - 40.0).abs() < f32::EPSILON);
    assert!((item.pos_neu_m.y - 10.0).abs() < f32::EPSILON);
    assert!((item.radius_m - 9.0).abs() < f32::EPSILON);
}

#[test]
fn ais_different_id_is_new() {
    let mut db = Database::new();
    db.queue_push_radius(
        Vector3f::new(1.0, 0.0, 0.0),
        10,
        20.0,
        8.0,
        OaDbSource::Ais,
        1,
        &QueuePushContext::default(),
    );
    db.queue_push_radius(
        Vector3f::new(1.0, 0.0, 0.0),
        10,
        20.0,
        8.0,
        OaDbSource::Ais,
        2,
        &QueuePushContext::default(),
    );
    db.update(10);
    assert_eq!(db.database_count(), 2);
}

#[test]
fn different_source_does_not_match() {
    let mut db = Database::new();
    let pos = origin_pos();
    db.queue_push_radius(
        pos,
        10,
        5.0,
        2.0,
        OaDbSource::Proximity,
        7,
        &QueuePushContext::default(),
    );
    db.queue_push_radius(
        pos,
        10,
        5.0,
        2.0,
        OaDbSource::Ais,
        7,
        &QueuePushContext::default(),
    );
    db.update(10);
    assert_eq!(db.database_count(), 2);
}

#[test]
fn refresh_skips_when_same_radius_and_fresh() {
    let mut db = Database::new();
    let pos = origin_pos();
    push_ok(&mut db, pos, 1_000, 8.0, 1.5);
    db.update(1_000);
    push_ok(&mut db, pos, 1_000 + REFRESH_MS - 1, 8.0, 1.5);
    db.update(1_000 + REFRESH_MS - 1);
    let item = db.get_item(0).expect("held");
    assert_eq!(item.timestamp_ms, 1_000);
}

#[test]
fn refresh_updates_when_500ms_elapsed() {
    let mut db = Database::new();
    let pos = origin_pos();
    push_ok(&mut db, pos, 1_000, 8.0, 1.5);
    db.update(1_000);
    push_ok(&mut db, pos, 1_000 + REFRESH_MS, 8.0, 1.5);
    db.update(1_000 + REFRESH_MS);
    let item = db.get_item(0).expect("refreshed");
    assert_eq!(item.timestamp_ms, 1_000 + REFRESH_MS);
}

#[test]
fn expiry_removes_stale() {
    let mut db = Database::new();
    db.set_expiry_seconds(10);
    push_ok(&mut db, origin_pos(), 0, 5.0, 1.0);
    db.update(0);
    assert_eq!(db.database_count(), 1);
    db.update(10_000);
    assert_eq!(db.database_count(), 1);
    db.update(10_001);
    assert_eq!(db.database_count(), 0);
}

#[test]
fn expiry_zero_never_expires() {
    let mut db = Database::new();
    db.set_expiry_seconds(0);
    push_ok(&mut db, origin_pos(), 0, 5.0, 1.0);
    db.update(0);
    db.update(u32::MAX);
    assert_eq!(db.database_count(), 1);
}

#[test]
fn beam_width_sets_radius() {
    let mut db = Database::new();
    db.queue_push(
        origin_pos(),
        10,
        10.0,
        OaDbSource::Proximity,
        0,
        &QueuePushContext::default(),
    );
    db.update(10);
    let item = db.get_item(0).expect("beam");
    let expected = (10.0 * db.dist_to_radius_scalar()).max(RADIUS_MIN_DEFAULT_M);
    assert!((item.radius_m - expected).abs() < 1e-5);
}

#[test]
fn send_to_gcs_flags_by_output_level() {
    let mut db = Database::new();
    db.set_output_level(OaDbOutputLevel::None);
    assert_eq!(db.send_to_gcs_flags(OaDbImportance::High), 0);
    db.set_output_level(OaDbOutputLevel::High);
    assert_eq!(db.send_to_gcs_flags(OaDbImportance::High), 0xFF);
    assert_eq!(db.send_to_gcs_flags(OaDbImportance::Normal), 0);
    assert_eq!(db.send_to_gcs_flags(OaDbImportance::Low), 0);
    db.set_output_level(OaDbOutputLevel::HighAndNormal);
    assert_eq!(db.send_to_gcs_flags(OaDbImportance::Normal), 0xFF);
    assert_eq!(db.send_to_gcs_flags(OaDbImportance::Low), 0);
    db.set_output_level(OaDbOutputLevel::All);
    assert_eq!(db.send_to_gcs_flags(OaDbImportance::Low), 0xFF);
}

#[test]
fn full_database_drops_new() {
    let mut db = Database::new();
    db.set_database_size_param(2);
    db.init();
    push_ok(&mut db, Vector3f::new(0.0, 0.0, 0.0), 10, 5.0, 0.4);
    push_ok(&mut db, Vector3f::new(10.0, 0.0, 0.0), 10, 5.0, 0.4);
    push_ok(&mut db, Vector3f::new(20.0, 0.0, 0.0), 10, 5.0, 0.4);
    db.update(10);
    assert_eq!(db.database_count(), 2);
    assert!(db.get_item(0).is_some());
    assert!(db.get_item(1).is_some());
    assert!(db.get_item(2).is_none());
}

#[test]
fn process_queue_reports_more_work() {
    let mut db = Database::new();
    db.set_queue_size_param(4);
    db.init();
    for i in 0_u16..4 {
        push_ok(
            &mut db,
            Vector3f::new(f32::from(i) * 10.0, 0.0, 0.0),
            10,
            5.0,
            0.4,
        );
    }
    assert_eq!(db.queue_len(), 4);
    // Cap is 100, so one call drains the leftover queue.
    assert!(!db.process_queue());
    assert_eq!(db.database_count(), 4);
}

#[test]
fn fill_bendy_items_roundtrips_pos_and_radius() {
    let mut db = Database::new();
    push_ok(&mut db, Vector3f::new(3.0, 4.0, 1.0), 10, 6.0, 2.5);
    db.update(10);
    let slots = db.fill_bendy_items();
    let first = slots.first().copied().flatten().expect("bendy");
    assert!((first.pos_neu_m.x - 3.0).abs() < f32::EPSILON);
    assert!((first.pos_neu_m.y - 4.0).abs() < f32::EPSILON);
    assert!((first.radius_m - 2.5).abs() < f32::EPSILON);
    assert!(slots.get(1).copied().flatten().is_none());
}

#[test]
fn output_from_param_matches_upstream_enum() {
    assert_eq!(OaDbOutputLevel::from_param(0), OaDbOutputLevel::None);
    assert_eq!(OaDbOutputLevel::from_param(1), OaDbOutputLevel::High);
    assert_eq!(
        OaDbOutputLevel::from_param(2),
        OaDbOutputLevel::HighAndNormal
    );
    assert_eq!(OaDbOutputLevel::from_param(3), OaDbOutputLevel::All);
    assert_eq!(OaDbOutputLevel::from_param(99), OaDbOutputLevel::High);
}
