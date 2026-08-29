//! `AC_PrecLand_SITL` leftover.
//!
//! Tracked as **COP-028**. ADR-0004 forbids `AP::sitl()`, so the
//! vehicle injects a [`SitlSample`]. IRLock / SITL-Gazebo `init`,
//! `Write_Precland`, the inertial ring, and the retry state machine
//! stay later.

use ap_math::matrix3::Matrix3f;
use ap_math::rotations_gen::{rotate_inverse, Rotation};
use ap_math::scalar::is_equal;
use ap_math::vector3::Vector3f;
use ap_precland::{
    PrecLand, PrecLandParams, SitlBackend, SitlSample, Type, VectorFrame, LOS_MEAS_TIMEOUT_MS,
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

/// Target 1 m north and 3 m down of the vehicle, identity DCM.
fn target_sample(last_update_ms: u32) -> SitlSample {
    SitlSample {
        healthy: true,
        last_update_ms,
        target_position: Vector3f::new(1.0, 0.0, 3.0),
        enable_target_distance: false,
        body_to_ned: Matrix3f::identity(),
    }
}

/// Upstream `_los_meas.vec_unit` for [`target_sample`] at Pitch270.
///
/// `body_to_ned.mul_transpose(-position)` with identity DCM is
/// `(-1, 0, -3)`, then `/= length()`.
fn expect_unit() -> Vector3f {
    let mut v = Vector3f::new(-1.0, 0.0, -3.0);
    let length = v.length();
    v /= length;
    v
}

fn sitl_inited() -> PrecLand {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled: true,
        sensor_type: Type::Sitl,
        ..PrecLandParams::default()
    });
    let leftover = plnd.init(400);
    assert!(!leftover.skipped);
    assert_eq!(leftover.backend, Some(Type::Sitl));
    assert!(leftover.need_sitl);
    assert!(!plnd.healthy());
    plnd
}

#[test]
fn sitl_init_leaves_healthy_false() {
    let mut backend = SitlBackend::new();
    assert!(!backend.healthy());
    backend.init();
    assert!(!backend.healthy());
    assert!(backend.get_los_meas().is_none());
    almost(backend.distance_to_target(), 0.0);
}

#[test]
fn sitl_update_writes_body_frd_unit_vector() {
    let mut backend = SitlBackend::new();
    backend.update(target_sample(4_200), 4_200, Rotation::Pitch270);
    assert!(backend.healthy());

    let (vec, frame) = backend.get_los_meas().expect("valid LOS");
    assert_eq!(frame, VectorFrame::BodyFrd);
    almost_vec(vec, expect_unit());
    assert_eq!(backend.los_meas_time_ms(), 4_200);
    almost(backend.distance_to_target(), 0.0);
}

#[test]
fn sitl_update_enable_target_distance_uses_vec_length() {
    let mut backend = SitlBackend::new();
    let mut sample = target_sample(10);
    sample.enable_target_distance = true;
    backend.update(sample, 10, Rotation::Pitch270);

    let expect_len = Vector3f::new(-1.0, 0.0, -3.0).length();
    almost(backend.distance_to_target(), expect_len);
    let (vec, _) = backend.get_los_meas().expect("valid LOS");
    almost_vec(vec, expect_unit());
}

#[test]
fn sitl_update_clears_los_on_same_timestamp() {
    let mut backend = SitlBackend::new();
    backend.update(target_sample(100), 100, Rotation::Pitch270);
    assert!(backend.get_los_meas().is_some());

    let mut moved = target_sample(100);
    moved.target_position = Vector3f::new(9.0, 0.0, 1.0);
    backend.update(moved, 150, Rotation::Pitch270);
    assert!(
        backend.get_los_meas().is_none(),
        "SITL else-branch clears valid on a repeated timestamp"
    );
    assert_eq!(backend.los_meas_time_ms(), 100);
}

#[test]
fn sitl_update_clears_los_when_unhealthy() {
    let mut backend = SitlBackend::new();
    backend.update(target_sample(50), 50, Rotation::Pitch270);
    assert!(backend.get_los_meas().is_some());

    let mut sample = target_sample(80);
    sample.healthy = false;
    backend.update(sample, 80, Rotation::Pitch270);
    assert!(!backend.healthy());
    assert!(backend.get_los_meas().is_none());
}

#[test]
fn sitl_update_expires_stale_los() {
    let mut backend = SitlBackend::new();
    backend.update(target_sample(1_000), 1_000, Rotation::Pitch270);
    assert!(backend.get_los_meas().is_some());

    // Same timestamp clears valid before the shared timeout AND.
    let later = target_sample(1_000);
    backend.update(later, 1_000 + LOS_MEAS_TIMEOUT_MS, Rotation::Pitch270);
    assert!(backend.get_los_meas().is_none());
}

#[test]
fn sitl_update_expires_new_sample_older_than_timeout() {
    let mut backend = SitlBackend::new();
    let mut sample = target_sample(5_000);
    backend.update(sample, 5_000 + LOS_MEAS_TIMEOUT_MS + 1, Rotation::Pitch270);
    assert!(
        backend.get_los_meas().is_none(),
        "new LOS older than 1000 ms is expired"
    );

    sample.last_update_ms = 6_000;
    backend.update(sample, 6_000 + LOS_MEAS_TIMEOUT_MS, Rotation::Pitch270);
    assert!(
        backend.get_los_meas().is_some(),
        "valid while now - time_ms == 1000"
    );

    sample.last_update_ms = 7_000;
    backend.update(sample, 7_000 + LOS_MEAS_TIMEOUT_MS + 1, Rotation::Pitch270);
    assert!(backend.get_los_meas().is_none());
}

#[test]
fn sitl_update_rotates_when_orient_is_not_pitch270() {
    let mut pitch270 = SitlBackend::new();
    pitch270.update(target_sample(20), 20, Rotation::Pitch270);
    let (plain, _) = pitch270.get_los_meas().expect("pitch270");

    let mut other = SitlBackend::new();
    other.update(target_sample(20), 20, Rotation::None);
    let (rotated, _) = other.get_los_meas().expect("none");

    let mut expect = expect_unit();
    let _ = rotate_inverse(&mut expect, Rotation::None);
    let _ = rotate_inverse(&mut expect, Rotation::Pitch90);
    almost_vec(rotated, expect);
    assert!(
        !is_equal(plain.x, rotated.x) || !is_equal(plain.y, rotated.y) || !is_equal(plain.z, rotated.z),
        "non-Pitch270 must change the body vector"
    );
}

#[test]
fn frontend_update_without_sample_stays_leftover() {
    let mut plnd = sitl_inited();
    let leftover = plnd.update(100.0, true, 41);
    assert!(leftover.need_backend_update);
    assert!(!leftover.backend_updated);
    assert!(plnd.backend_los_meas().is_none());
}

#[test]
fn frontend_update_with_sitl_writes_los() {
    let mut plnd = sitl_inited();
    let leftover = plnd.update_with_sitl(100.0, true, 4_200, target_sample(4_200));
    assert!(!leftover.skipped);
    assert!(!leftover.need_backend_update);
    assert!(leftover.backend_updated);
    assert!(plnd.healthy());

    let (vec, frame) = plnd
        .backend_los_meas()
        .expect("frontend should expose SITL LOS");
    assert_eq!(frame, VectorFrame::BodyFrd);
    almost_vec(vec, expect_unit());
    almost(plnd.distance_to_target(), 0.0);

    let sample = plnd.backend_los_sample().expect("sample");
    assert_eq!(sample.time_ms, 4_200);
    almost(sample.distance_to_target_m, 0.0);
}

#[test]
fn frontend_update_does_not_run_sitl_when_disabled() {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled: false,
        sensor_type: Type::Sitl,
        ..PrecLandParams::default()
    });
    let leftover_init = plnd.init(400);
    assert!(leftover_init.need_sitl);
    let leftover = plnd.update_with_sitl(100.0, true, 50, target_sample(50));
    assert!(!leftover.need_backend_update);
    assert!(!leftover.backend_updated);
    assert!(
        plnd.backend_los_meas().is_none(),
        "disabled update must not write LOS"
    );
    assert!(!plnd.healthy());
}

#[test]
fn frontend_update_does_not_run_sitl_on_irlock_sample() {
    let mut plnd = sitl_inited();
    // Type::Sitl ignores an IRLock snapshot.
    let leftover = plnd.update_with_irlock(
        100.0,
        true,
        41,
        ap_precland::IrlockSample {
            healthy: true,
            last_update_ms: 41,
            pos_x: 0.3,
            pos_y: -0.4,
            pos_z: 1.0,
        },
    );
    assert!(leftover.need_backend_update);
    assert!(!leftover.backend_updated);
    assert!(plnd.backend_los_meas().is_none());
}

#[test]
fn retrieve_los_meas_reads_sitl_sample() {
    let mut plnd = sitl_inited();
    let _ = plnd.update_with_sitl(100.0, true, 3_000, target_sample(3_000));
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
fn leftover_catalog_drops_sitl_update() {
    assert!(
        REMAINING.len() >= 8,
        "SITL slice must not claim the 1,133-loc ticket is done"
    );
    assert!(!REMAINING.contains(&"AC_PrecLand_SITL::update"));
    assert!(REMAINING.contains(&"AC_PrecLand_SITL::init(AP::sitl)"));
    assert!(REMAINING.contains(&"AC_PrecLand_IRLock::init(irlock)"));
    assert!(REMAINING.contains(&"AC_PrecLand_SITL_Gazebo::init(irlock)"));
    assert!(!REMAINING.contains(&"AC_PrecLand::Write_Precland"));
    assert!(!REMAINING.contains(&"inertial_data_frame_s"));
    assert!(REMAINING.contains(&"AC_PrecLand_StateMachine::update"));
}
