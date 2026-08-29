//! `AC_PrecLand_IRLock` + `AC_PrecLand_SITL_Gazebo` leftover.
//!
//! Tracked as **COP-028**. Both backends share one `update` body.
//! SITL (`AP::sitl()`) and the retry state machine stay later.

use ap_math::scalar::is_equal;
use ap_math::vector3::Vector3f;
use ap_precland::{
    IrlockBackend, IrlockSample, PrecLand, PrecLandParams, Type, VectorFrame, LOS_MEAS_TIMEOUT_MS,
    REMAINING,
};

fn almost(a: f32, b: f32) {
    assert!(is_equal(a, b), "{a} != {b}");
}

fn almost_vec(got: Vector3f, expect: Vector3f) {
    almost(got.x, expect.x);
    almost(got.y, expect.y);
    almost(got.z, expect.z);
}

fn target_sample(last_update_ms: u32) -> IrlockSample {
    IrlockSample {
        healthy: true,
        last_update_ms,
        pos_x: 0.3,
        pos_y: -0.4,
        pos_z: 1.0,
    }
}

fn expect_unit() -> Vector3f {
    let mut v = Vector3f::new(-(-0.4), 0.3, 1.0);
    let length = v.length();
    v /= length;
    v
}

fn irlock_inited() -> PrecLand {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled: true,
        sensor_type: Type::Irlock,
        ..PrecLandParams::default()
    });
    let leftover = plnd.init(400);
    assert!(!leftover.skipped);
    assert_eq!(leftover.backend, Some(Type::Irlock));
    assert_eq!(leftover.irlock_bus, Some(-1));
    assert!(!plnd.healthy());
    plnd
}

#[test]
fn irlock_sample_num_targets_follows_healthy() {
    let mut sample = target_sample(10);
    assert_eq!(sample.num_targets(), 1);
    sample.healthy = false;
    assert_eq!(sample.num_targets(), 0);
    assert!(sample.unit_vector_body().is_none());
}

#[test]
fn irlock_sample_unit_vector_body_matches_upstream() {
    let sample = target_sample(10);
    let vec = sample.unit_vector_body().expect("healthy");
    almost_vec(vec, expect_unit());
}

#[test]
fn irlock_init_leaves_healthy_false() {
    let mut backend = IrlockBackend::new();
    assert!(!backend.healthy());
    backend.init();
    assert!(!backend.healthy());
    assert!(backend.get_los_meas().is_none());
    almost(backend.distance_to_target(), 0.0);
}

#[test]
fn irlock_update_writes_body_frd_unit_vector() {
    let mut backend = IrlockBackend::new();
    backend.update(target_sample(4_200), 4_200);
    assert!(backend.healthy());

    let (vec, frame) = backend.get_los_meas().expect("valid LOS");
    assert_eq!(frame, VectorFrame::BodyFrd);
    almost_vec(vec, expect_unit());
    assert_eq!(backend.los_meas_time_ms(), 4_200);
    almost(backend.distance_to_target(), 0.0);
}

#[test]
fn irlock_update_ignores_same_timestamp() {
    let mut backend = IrlockBackend::new();
    backend.update(target_sample(100), 100);
    let first = backend.get_los_meas().expect("first");

    let mut moved = target_sample(100);
    moved.pos_x = 9.0;
    backend.update(moved, 150);
    let second = backend.get_los_meas().expect("same timestamp");
    almost_vec(second.0, first.0);
    assert_eq!(backend.los_meas_time_ms(), 100);
}

#[test]
fn irlock_update_skips_when_unhealthy() {
    let mut backend = IrlockBackend::new();
    let mut sample = target_sample(50);
    sample.healthy = false;
    backend.update(sample, 50);
    assert!(!backend.healthy());
    assert!(backend.get_los_meas().is_none());
}

#[test]
fn irlock_update_expires_stale_los() {
    let mut backend = IrlockBackend::new();
    backend.update(target_sample(1_000), 1_000);
    assert!(backend.get_los_meas().is_some());

    let mut later = target_sample(1_000);
    later.healthy = false;
    backend.update(later, 1_000 + LOS_MEAS_TIMEOUT_MS);
    assert!(
        backend.get_los_meas().is_some(),
        "valid while now - time_ms == 1000"
    );

    backend.update(later, 1_000 + LOS_MEAS_TIMEOUT_MS + 1);
    assert!(backend.get_los_meas().is_none());
}

#[test]
fn frontend_update_without_sample_stays_leftover() {
    let mut plnd = irlock_inited();
    let leftover = plnd.update(100.0, true, 41);
    assert!(leftover.need_backend_update);
    assert!(!leftover.backend_updated);
    assert!(plnd.backend_los_meas().is_none());
}

#[test]
fn frontend_update_with_irlock_writes_los() {
    let mut plnd = irlock_inited();
    let leftover = plnd.update_with_irlock(100.0, true, 4_200, target_sample(4_200));
    assert!(!leftover.skipped);
    assert!(!leftover.need_backend_update);
    assert!(leftover.backend_updated);
    assert!(plnd.healthy());

    let (vec, frame) = plnd
        .backend_los_meas()
        .expect("frontend should expose IRLock LOS");
    assert_eq!(frame, VectorFrame::BodyFrd);
    almost_vec(vec, expect_unit());
    almost(plnd.distance_to_target(), 0.0);

    let sample = plnd.backend_los_sample().expect("sample");
    assert_eq!(sample.time_ms, 4_200);
    almost(sample.distance_to_target_m, 0.0);
}

#[test]
fn frontend_update_does_not_run_irlock_when_disabled() {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled: false,
        sensor_type: Type::Irlock,
        ..PrecLandParams::default()
    });
    let _ = plnd.init(400);
    let leftover = plnd.update_with_irlock(100.0, true, 50, target_sample(50));
    assert!(!leftover.need_backend_update);
    assert!(!leftover.backend_updated);
    assert!(
        plnd.backend_los_meas().is_none(),
        "disabled update must not write LOS"
    );
    assert!(!plnd.healthy());
}

#[test]
fn sitl_gazebo_shares_irlock_update() {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled: true,
        sensor_type: Type::SitlGazebo,
        bus: 2,
        ..PrecLandParams::default()
    });
    let init = plnd.init(400);
    assert_eq!(init.backend, Some(Type::SitlGazebo));
    assert_eq!(init.irlock_bus, Some(2));
    assert!(!plnd.healthy());

    let leftover = plnd.update_with_irlock(80.0, true, 9, target_sample(9));
    assert!(!leftover.need_backend_update);
    assert!(leftover.backend_updated);
    assert!(plnd.healthy());
    let (_vec, frame) = plnd.backend_los_meas().expect("gazebo LOS");
    assert_eq!(frame, VectorFrame::BodyFrd);
}

#[test]
fn sitl_update_stays_leftover() {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled: true,
        sensor_type: Type::Sitl,
        ..PrecLandParams::default()
    });
    let _ = plnd.init(400);
    let leftover = plnd.update(100.0, true, 41);
    assert!(leftover.need_backend_update);
    assert!(!leftover.backend_updated);

    let leftover = plnd.update_with_irlock(100.0, true, 41, target_sample(41));
    assert!(
        leftover.need_backend_update,
        "SITL is not the IRLock path"
    );
    assert!(!leftover.backend_updated);
    assert!(plnd.backend_los_meas().is_none());
}

#[test]
fn retrieve_los_meas_reads_irlock_sample() {
    let mut plnd = irlock_inited();
    let _ = plnd.update_with_irlock(100.0, true, 3_000, target_sample(3_000));
    let sample = plnd.backend_los_sample();
    let (vec, frame) = plnd
        .retrieve_los_meas(sample)
        .expect("new backend measurement");
    assert_eq!(frame, VectorFrame::BodyFrd);
    almost_vec(vec, expect_unit());
    assert_eq!(plnd.last_backend_los_meas_ms(), 3_000);

    assert!(
        plnd.retrieve_los_meas(plnd.backend_los_sample()).is_none(),
        "same timestamp is not a new measurement"
    );
}

#[test]
fn leftover_catalog_drops_irlock_and_gazebo_update() {
    assert!(
        REMAINING.len() > 8,
        "IRLock slice must not claim the 1,133-loc ticket is done"
    );
    assert!(!REMAINING.contains(&"AC_PrecLand_IRLock::update"));
    assert!(!REMAINING.contains(&"AC_PrecLand_SITL_Gazebo::update"));
    assert!(REMAINING.contains(&"AC_PrecLand_IRLock::init(irlock)"));
    assert!(REMAINING.contains(&"AC_PrecLand_SITL_Gazebo::init(irlock)"));
    assert!(REMAINING.contains(&"AC_PrecLand_SITL::update"));
    assert!(REMAINING.contains(&"AC_PrecLand::Write_Precland"));
    assert!(REMAINING.contains(&"AC_PrecLand_StateMachine::update"));
}
