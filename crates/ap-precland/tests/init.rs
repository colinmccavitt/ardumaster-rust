//! `AC_PrecLand::init` leftover.
//!
//! Tracked as **COP-028**. Estimator, `update`, and the retry state
//! machine stay later.

use ap_math::rotations_gen::Rotation;
use ap_math::scalar::is_equal;
use ap_math::vector3::Vector3f;
use ap_precland::{
    EstimatorType, PrecLand, PrecLandParams, TargetState, Type, VectorFrame, LAG_S_DEFAULT,
    LAG_S_MAX, LAG_S_MIN, OPTION_DISABLED, OPTION_FAST_DESCEND, OPTION_MOVING_TARGET,
    OPTION_PRECLAND_AFTER_REPOSITION, ORIENT_DEFAULT_COPTER, REMAINING, XY_MAX_DIST_DESC_M_DEFAULT,
};

fn almost(a: f32, b: f32) {
    assert!(is_equal(a, b), "{a} != {b}");
}

fn almost_vec(got: Vector3f, want: Vector3f) {
    assert!(
        is_equal(got.x, want.x) && is_equal(got.y, want.y) && is_equal(got.z, want.z),
        "({} {} {}) != ({} {} {})",
        got.x,
        got.y,
        got.z,
        want.x,
        want.y,
        want.z
    );
}

#[test]
fn discriminants_match_upstream() {
    assert_eq!(Type::None as u8, 0);
    assert_eq!(Type::Mavlink as u8, 1);
    assert_eq!(Type::Irlock as u8, 2);
    assert_eq!(Type::SitlGazebo as u8, 3);
    assert_eq!(Type::Sitl as u8, 4);
    assert_eq!(EstimatorType::RawSensor as u8, 0);
    assert_eq!(EstimatorType::KalmanFilter as u8, 1);
    assert_eq!(TargetState::NeverSeen as u8, 0);
    assert_eq!(TargetState::OutOfRange as u8, 1);
    assert_eq!(TargetState::RecentlyLost as u8, 2);
    assert_eq!(TargetState::Found as u8, 3);
    assert_eq!(VectorFrame::BodyFrd as u8, 0);
    assert_eq!(VectorFrame::LocalFrd as u8, 1);
    assert_eq!(OPTION_DISABLED, 0);
    assert_eq!(OPTION_MOVING_TARGET, 1);
    assert_eq!(OPTION_PRECLAND_AFTER_REPOSITION, 2);
    assert_eq!(OPTION_FAST_DESCEND, 4);
    almost(LAG_S_DEFAULT, 0.02);
    almost(LAG_S_MIN, 0.02);
    almost(LAG_S_MAX, 0.25);
    almost(XY_MAX_DIST_DESC_M_DEFAULT, 2.5);
    assert_eq!(ORIENT_DEFAULT_COPTER, Rotation::Pitch270);
    assert_eq!(ORIENT_DEFAULT_COPTER as u8, 25);
}

#[test]
fn none_allocates_history_but_no_backend() {
    let mut plnd = PrecLand::new();
    assert!(!plnd.enabled());
    assert_eq!(plnd.sensor_type(), Type::None);

    let leftover = plnd.init(400);
    assert!(!leftover.skipped);
    assert_eq!(leftover.backend, None);
    assert_eq!(leftover.inertial_buffer_size, 8);
    assert_eq!(leftover.irlock_bus, None);
    assert!(!leftover.need_sitl);

    assert_eq!(plnd.backend(), None);
    assert!(!plnd.healthy());
    assert_eq!(plnd.target_state(), TargetState::NeverSeen);
    assert!(plnd.inertial_history_ready());
    assert_eq!(plnd.inertial_buffer_size(), 8);
    almost(plnd.lag_s(), LAG_S_DEFAULT);
    // Type::NONE returns before the approach-vector write.
    almost_vec(plnd.approach_vector_body(), Vector3f::zero());
}

#[test]
fn none_can_init_again_after_type_change() {
    let mut plnd = PrecLand::new();
    let first = plnd.init(400);
    assert!(!first.skipped);
    assert_eq!(first.backend, None);

    plnd.set_sensor_type(Type::Mavlink);
    let second = plnd.init(400);
    assert!(!second.skipped);
    assert_eq!(second.backend, Some(Type::Mavlink));
    assert!(plnd.healthy());
    almost_vec(plnd.approach_vector_body(), Vector3f::new(0.0, 0.0, 1.0));
}

#[test]
fn mavlink_init_sets_healthy_and_skips_second_call() {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled: true,
        sensor_type: Type::Mavlink,
        ..PrecLandParams::default()
    });
    let leftover = plnd.init(400);
    assert!(!leftover.skipped);
    assert_eq!(leftover.backend, Some(Type::Mavlink));
    assert_eq!(leftover.irlock_bus, None);
    assert!(!leftover.need_sitl);
    assert!(plnd.healthy());
    assert_eq!(plnd.target_state(), TargetState::NeverSeen);
    // Pitch270 of (1,0,0) is (0,0,1).
    almost_vec(plnd.approach_vector_body(), Vector3f::new(0.0, 0.0, 1.0));

    plnd.set_lag_s(0.25);
    plnd.set_sensor_type(Type::Irlock);
    let again = plnd.init(50);
    assert!(again.skipped);
    assert_eq!(again.backend, Some(Type::Mavlink));
    // A live init at 50 Hz with lag 0.25 would size the ring to 13.
    // Skip leaves the first-init size. LAG is `@RebootRequired`.
    assert_eq!(again.inertial_buffer_size, 8);
    assert_eq!(again.irlock_bus, None);
    almost(plnd.lag_s(), 0.25);
    assert_eq!(plnd.backend(), Some(Type::Mavlink));
}

#[test]
fn lag_is_constrained_and_sizes_the_ring() {
    let mut low = PrecLand::from_params(PrecLandParams {
        sensor_type: Type::Mavlink,
        lag_s: 0.001,
        ..PrecLandParams::default()
    });
    let leftover = low.init(400);
    almost(low.lag_s(), LAG_S_MIN);
    assert_eq!(leftover.inertial_buffer_size, 8);

    let mut high = PrecLand::from_params(PrecLandParams {
        sensor_type: Type::Mavlink,
        lag_s: 1.0,
        ..PrecLandParams::default()
    });
    let leftover = high.init(400);
    almost(high.lag_s(), LAG_S_MAX);
    assert_eq!(leftover.inertial_buffer_size, 100);

    let mut slow = PrecLand::from_params(PrecLandParams {
        sensor_type: Type::Mavlink,
        ..PrecLandParams::default()
    });
    let leftover = slow.init(1);
    assert_eq!(leftover.inertial_buffer_size, 1);
}

#[test]
fn irlock_and_gazebo_leave_healthy_false() {
    let mut irlock = PrecLand::from_params(PrecLandParams {
        sensor_type: Type::Irlock,
        bus: 1,
        ..PrecLandParams::default()
    });
    let leftover = irlock.init(400);
    assert_eq!(leftover.backend, Some(Type::Irlock));
    assert_eq!(leftover.irlock_bus, Some(1));
    assert!(!leftover.need_sitl);
    assert!(!irlock.healthy());

    let mut gazebo = PrecLand::from_params(PrecLandParams {
        sensor_type: Type::SitlGazebo,
        bus: 0,
        ..PrecLandParams::default()
    });
    let leftover = gazebo.init(400);
    assert_eq!(leftover.backend, Some(Type::SitlGazebo));
    assert_eq!(leftover.irlock_bus, Some(0));
    assert!(!gazebo.healthy());
}

#[test]
fn sitl_init_asks_for_the_sitl_singleton() {
    let mut sitl = PrecLand::from_params(PrecLandParams {
        sensor_type: Type::Sitl,
        ..PrecLandParams::default()
    });
    let leftover = sitl.init(400);
    assert_eq!(leftover.backend, Some(Type::Sitl));
    assert!(leftover.need_sitl);
    assert_eq!(leftover.irlock_bus, None);
    assert!(!sitl.healthy());
}

#[test]
fn orient_none_leaves_approach_forward() {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        sensor_type: Type::Mavlink,
        orient: Rotation::None,
        ..PrecLandParams::default()
    });
    let leftover = plnd.init(400);
    assert!(!leftover.skipped);
    almost_vec(plnd.approach_vector_body(), Vector3f::new(1.0, 0.0, 0.0));
}

#[test]
fn leftover_catalog_is_not_empty() {
    assert!(
        REMAINING.len() > 10,
        "first stub must not claim the 1,133-loc ticket is done"
    );
    assert!(REMAINING.contains(&"AC_PrecLand::update"));
    assert!(REMAINING.contains(&"AC_PrecLand_StateMachine::update"));
    assert!(REMAINING.contains(&"PosVelEKF"));
}
