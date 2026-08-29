//! Inertial history ring leftover.
//!
//! Tracked as **COP-028**. Driver `init` and `AC_PrecLand_StateMachine`
//! stay later.

use ap_math::scalar::is_equal;
use ap_math::vector3::Vector3f;
use ap_precland::{
    InertialHistory, InertialSample, PrecLand, PrecLandParams, Type, INERTIAL_HISTORY_MAX,
    REMAINING,
};

fn almost(a: f32, b: f32) {
    assert!(is_equal(a, b), "{a} != {b}");
}

fn frame(dt: f32, vn: f32, valid: bool) -> InertialSample {
    InertialSample {
        corrected_vehicle_delta_velocity_ned: Vector3f::new(vn, 0.0, 0.0),
        inertial_nav_velocity: Vector3f::new(vn, 0.0, 0.0),
        inertial_nav_velocity_valid: valid,
        dt,
        ..InertialSample::default()
    }
}

fn mavlink_inited() -> PrecLand {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled: true,
        sensor_type: Type::Mavlink,
        ..PrecLandParams::default()
    });
    let leftover = plnd.init(400);
    assert!(!leftover.skipped);
    assert_eq!(leftover.inertial_buffer_size, 8);
    assert_eq!(plnd.inertial_history().size(), 8);
    plnd
}

#[test]
fn object_array_push_pop_and_index() {
    let mut ring = InertialHistory::new(3);
    assert_eq!(ring.size(), 3);
    assert_eq!(ring.available(), 0);
    assert_eq!(ring.space(), 3);
    assert!(ring.is_empty());
    assert!(ring.delayed().is_none());
    assert!(ring.newest().is_none());

    assert!(ring.push(frame(0.01, 1.0, true)));
    assert!(ring.push(frame(0.01, 2.0, true)));
    assert!(ring.push(frame(0.01, 3.0, true)));
    assert_eq!(ring.available(), 3);
    assert_eq!(ring.space(), 0);
    almost(ring.get(0).unwrap().inertial_nav_velocity.x, 1.0);
    almost(ring.get(1).unwrap().inertial_nav_velocity.x, 2.0);
    almost(ring.get(2).unwrap().inertial_nav_velocity.x, 3.0);
    assert!(ring.get(3).is_none());
    assert!(!ring.push(frame(0.01, 99.0, true)));

    assert!(ring.pop());
    almost(ring.delayed().unwrap().inertial_nav_velocity.x, 2.0);
    assert_eq!(ring.available(), 2);
}

#[test]
fn push_force_drops_oldest() {
    let mut ring = InertialHistory::new(2);
    assert!(ring.push_force(frame(0.01, 1.0, true)));
    assert!(ring.push_force(frame(0.01, 2.0, true)));
    assert!(ring.push_force(frame(0.01, 3.0, true)));
    assert_eq!(ring.available(), 2);
    almost(ring.delayed().unwrap().inertial_nav_velocity.x, 2.0);
    almost(ring.newest().unwrap().inertial_nav_velocity.x, 3.0);

    let later: Vec<f32> = ring.later().map(|f| f.inertial_nav_velocity.x).collect();
    assert_eq!(later.len(), 1);
    almost(later[0], 3.0);
}

#[test]
fn wrap_around_preserves_order() {
    let mut ring = InertialHistory::new(3);
    for i in 1..=8 {
        assert!(ring.push_force(frame(0.002_5, i as f32, true)));
    }
    assert_eq!(ring.available(), 3);
    almost(ring.get(0).unwrap().inertial_nav_velocity.x, 6.0);
    almost(ring.get(1).unwrap().inertial_nav_velocity.x, 7.0);
    almost(ring.get(2).unwrap().inertial_nav_velocity.x, 8.0);
}

#[test]
fn unallocated_ring_rejects_push() {
    let mut ring = InertialHistory::default();
    assert_eq!(ring.size(), 0);
    assert!(!ring.push(frame(0.01, 1.0, true)));
    assert!(!ring.push_force(frame(0.01, 1.0, true)));
    assert!(!ring.pop());
    assert!(ring.get(0).is_none());
}

#[test]
fn any_inertial_nav_invalid_walks_the_ring() {
    let mut ring = InertialHistory::new(4);
    assert!(!ring.any_inertial_nav_invalid());
    ring.push_force(frame(0.01, 1.0, true));
    ring.push_force(frame(0.01, 2.0, false));
    ring.push_force(frame(0.01, 3.0, true));
    assert!(ring.any_inertial_nav_invalid());
    ring.clear();
    ring.push_force(frame(0.01, 1.0, true));
    assert!(!ring.any_inertial_nav_invalid());
}

#[test]
fn max_clamps_to_no_std_cap() {
    let ring = InertialHistory::new(u16::MAX);
    assert_eq!(ring.size(), INERTIAL_HISTORY_MAX as u16);
}

#[test]
fn init_allocates_sized_ring() {
    let mut plnd = PrecLand::new();
    assert_eq!(plnd.inertial_history().size(), 0);
    assert!(!plnd.inertial_history_ready());

    let leftover = plnd.init(400);
    assert_eq!(leftover.inertial_buffer_size, 8);
    assert!(plnd.inertial_history_ready());
    assert_eq!(plnd.inertial_history().size(), 8);
    assert_eq!(plnd.inertial_history().available(), 0);
}

#[test]
fn update_without_frame_leaves_ahrs_leftover() {
    let mut plnd = mavlink_inited();
    let leftover = plnd.update(100.0, true, 0);
    assert!(leftover.need_inertial_push);
    assert_eq!(plnd.inertial_history().available(), 0);
}

#[test]
fn update_with_inertial_push_forces_newest() {
    let mut plnd = mavlink_inited();
    let leftover = plnd.update_with_inertial(100.0, true, 0, frame(0.002_5, 1.5, true));
    assert!(!leftover.need_inertial_push);
    assert_eq!(plnd.inertial_history().available(), 1);
    almost(
        plnd.inertial_delayed().unwrap().inertial_nav_velocity.x,
        1.5,
    );
    assert!(!plnd.any_inertial_nav_invalid());

    for i in 2..=10 {
        let _ = plnd.update_with_inertial(100.0, true, 0, frame(0.002_5, i as f32, true));
    }
    // 8-slot ring (0.02 * 400); push_force dropped the oldest two.
    assert_eq!(plnd.inertial_history().available(), 8);
    almost(
        plnd.inertial_delayed().unwrap().inertial_nav_velocity.x,
        3.0,
    );
    almost(plnd.inertial_newest().unwrap().inertial_nav_velocity.x, 10.0);
    let later: Vec<f32> = plnd
        .inertial_history()
        .later()
        .map(|f| f.inertial_nav_velocity.x)
        .collect();
    assert_eq!(later.len(), 7);
    almost(later[0], 4.0);
    almost(later[6], 10.0);
}

#[test]
fn leftover_catalog_drops_ring() {
    assert!(
        REMAINING.is_empty(),
        "driver init and StateMachine stay later"
    );
    assert!(!REMAINING.contains(&"inertial_data_frame_s"));
    assert!(!REMAINING.contains(&"ObjectArray<inertial_data_frame_s>"));
    assert!(!REMAINING.contains(&"AC_PrecLand::Write_Precland"));
    assert!(!REMAINING.contains(&"AC_PrecLand_IRLock::init(irlock)"));
    assert!(!REMAINING.contains(&"AC_PrecLand_SITL::init(AP::sitl)"));
    assert!(!REMAINING.contains(&"AC_PrecLand_StateMachine::update"));
}
