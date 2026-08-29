//! `AC_PrecLand_Backend` + `AC_PrecLand_MAVLink` leftover.
//!
//! Tracked as **COP-028**. SITL and the retry state machine stay later.
//! IRLock / SITL-Gazebo `update` live in `tests/irlock.rs`.

use ap_math::scalar::is_equal;
use ap_math::vector3::Vector3f;
use ap_precland::{
    Backend, HandleMsgLeftover, LandingTargetMsg, MavlinkBackend, PrecLand, PrecLandParams, Type,
    VectorFrame, LOS_MEAS_TIMEOUT_MS, MAV_FRAME_BODY_FRD, MAV_FRAME_LOCAL_FRD, REMAINING,
};

fn almost(a: f32, b: f32) {
    assert!(is_equal(a, b), "{a} != {b}");
}

fn almost_vec(got: Vector3f, expect: Vector3f) {
    almost(got.x, expect.x);
    almost(got.y, expect.y);
    almost(got.z, expect.z);
}

fn mavlink_inited() -> PrecLand {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled: true,
        sensor_type: Type::Mavlink,
        ..PrecLandParams::default()
    });
    let leftover = plnd.init(400);
    assert!(!leftover.skipped);
    assert_eq!(leftover.backend, Some(Type::Mavlink));
    plnd
}

fn body_position_packet() -> LandingTargetMsg {
    LandingTargetMsg {
        frame: MAV_FRAME_BODY_FRD,
        position_valid: 1,
        distance: 2.0,
        x: 0.4,
        y: -0.6,
        z: 2.0,
        angle_x: 0.0,
        angle_y: 0.0,
    }
}

#[test]
fn backend_getters_empty_until_valid() {
    let backend = Backend::new();
    assert!(backend.get_los_meas().is_none());
    assert_eq!(backend.los_meas_time_ms(), 0);
    almost(backend.distance_to_target(), 0.0);
    assert!(backend.los_sample().is_none());
}

#[test]
fn backend_default_handle_msg_is_noop() {
    let mut backend = Backend::new();
    backend.handle_msg(body_position_packet(), 1_000);
    assert!(backend.get_los_meas().is_none());
}

#[test]
fn mavlink_init_sets_healthy() {
    let mut mav = MavlinkBackend::new();
    assert!(!mav.healthy());
    mav.init();
    assert!(mav.healthy());
}

#[test]
fn mavlink_handle_msg_position_valid_body_frd() {
    let mut mav = MavlinkBackend::new();
    let leftover = mav.handle_msg(body_position_packet(), 4_200);
    assert!(leftover.accepted);
    assert!(!leftover.need_gcs_wrong_frame);
    assert!(!leftover.rejected_non_positive_distance);

    let (vec, frame) = mav.get_los_meas().expect("valid LOS");
    assert_eq!(frame, VectorFrame::BodyFrd);
    almost_vec(vec, Vector3f::new(0.2, -0.3, 1.0));
    almost(mav.distance_to_target(), 2.0);
    assert_eq!(mav.los_meas_time_ms(), 4_200);
}

#[test]
fn mavlink_handle_msg_position_valid_local_frd() {
    let mut mav = MavlinkBackend::new();
    let mut packet = body_position_packet();
    packet.frame = MAV_FRAME_LOCAL_FRD;
    let leftover = mav.handle_msg(packet, 9);
    assert!(leftover.accepted);
    let (_vec, frame) = mav.get_los_meas().expect("valid LOS");
    assert_eq!(frame, VectorFrame::LocalFrd);
}

#[test]
fn mavlink_handle_msg_rejects_non_positive_distance() {
    let mut mav = MavlinkBackend::new();
    let mut packet = body_position_packet();
    packet.distance = 0.0;
    let leftover = mav.handle_msg(packet, 50);
    assert!(!leftover.accepted);
    assert!(leftover.rejected_non_positive_distance);
    assert!(mav.get_los_meas().is_none());
    almost(mav.distance_to_target(), 0.0);
}

#[test]
fn mavlink_handle_msg_angle_path() {
    let mut mav = MavlinkBackend::new();
    let packet = LandingTargetMsg {
        frame: MAV_FRAME_BODY_FRD,
        position_valid: 0,
        distance: 3.5,
        x: 0.0,
        y: 0.0,
        z: 0.0,
        angle_x: 0.2,
        angle_y: -0.1,
    };
    let leftover = mav.handle_msg(packet, 77);
    assert!(leftover.accepted);

    let expect = {
        let mut v = Vector3f::new(-libm::tanf(-0.1), libm::tanf(0.2), 1.0);
        let length = v.length();
        v /= length;
        v
    };
    let (vec, frame) = mav.get_los_meas().expect("valid LOS");
    assert_eq!(frame, VectorFrame::BodyFrd);
    almost_vec(vec, expect);
    almost(mav.distance_to_target(), 3.5);
}

#[test]
fn mavlink_handle_msg_clamps_negative_distance_on_angle_path() {
    let mut mav = MavlinkBackend::new();
    let packet = LandingTargetMsg {
        frame: MAV_FRAME_BODY_FRD,
        position_valid: 0,
        distance: -1.5,
        x: 0.0,
        y: 0.0,
        z: 0.0,
        angle_x: 0.0,
        angle_y: 0.0,
    };
    let leftover = mav.handle_msg(packet, 1);
    assert!(leftover.accepted);
    almost(mav.distance_to_target(), 0.0);
}

#[test]
fn mavlink_wrong_frame_gcs_once() {
    let mut mav = MavlinkBackend::new();
    let packet = LandingTargetMsg {
        frame: 1,
        position_valid: 1,
        distance: 1.0,
        x: 0.0,
        y: 0.0,
        z: 1.0,
        angle_x: 0.0,
        angle_y: 0.0,
    };
    let first = mav.handle_msg(packet, 10);
    assert!(!first.accepted);
    assert!(first.need_gcs_wrong_frame);
    assert!(mav.wrong_frame_msg_sent());

    let second = mav.handle_msg(packet, 11);
    assert!(!second.accepted);
    assert!(!second.need_gcs_wrong_frame);
    assert!(mav.get_los_meas().is_none());
}

#[test]
fn mavlink_update_expires_stale_los() {
    let mut mav = MavlinkBackend::new();
    let leftover = mav.handle_msg(body_position_packet(), 1_000);
    assert!(leftover.accepted);
    assert!(mav.get_los_meas().is_some());

    mav.update(1_000 + LOS_MEAS_TIMEOUT_MS);
    assert!(
        mav.get_los_meas().is_some(),
        "valid while now - time_ms == 1000"
    );

    mav.update(1_000 + LOS_MEAS_TIMEOUT_MS + 1);
    assert!(mav.get_los_meas().is_none());
    almost(mav.distance_to_target(), 2.0);
}

#[test]
fn frontend_handle_msg_runs_mavlink() {
    let mut plnd = mavlink_inited();
    let packet = body_position_packet();
    let leftover = plnd.handle_msg(packet, 2_222);
    assert!(!leftover.skipped);
    assert!(!leftover.need_backend_handle_msg);
    let mav = leftover.mavlink.expect("MAVLink leftover");
    assert!(mav.accepted);

    let (vec, frame) = plnd
        .backend_los_meas()
        .expect("frontend should expose backend LOS");
    assert_eq!(frame, VectorFrame::BodyFrd);
    almost_vec(vec, Vector3f::new(0.2, -0.3, 1.0));
    almost(plnd.distance_to_target(), 2.0);

    let sample = plnd.backend_los_sample().expect("sample");
    assert_eq!(sample.time_ms, 2_222);
    almost(sample.distance_to_target_m, 2.0);
}

#[test]
fn frontend_handle_msg_skips_without_backend() {
    let mut plnd = PrecLand::new();
    let leftover = plnd.handle_msg(body_position_packet(), 1);
    assert_eq!(
        leftover,
        HandleMsgLeftover {
            skipped: true,
            need_backend_handle_msg: false,
            timestamp_ms: 1,
            packet: body_position_packet(),
            mavlink: None,
        }
    );
    assert!(plnd.backend_los_meas().is_none());
}

#[test]
fn frontend_update_runs_mavlink_when_enabled() {
    let mut plnd = mavlink_inited();
    let _ = plnd.handle_msg(body_position_packet(), 100);
    assert!(plnd.backend_los_meas().is_some());

    let leftover = plnd.update(150.0, true, 100 + LOS_MEAS_TIMEOUT_MS + 1);
    assert!(!leftover.skipped);
    assert!(!leftover.need_backend_update);
    assert!(leftover.backend_updated);
    assert!(plnd.backend_los_meas().is_none());
}

#[test]
fn frontend_update_does_not_run_mavlink_when_disabled() {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled: false,
        sensor_type: Type::Mavlink,
        ..PrecLandParams::default()
    });
    let _ = plnd.init(400);
    let _ = plnd.handle_msg(body_position_packet(), 100);
    let leftover = plnd.update(150.0, true, 100 + LOS_MEAS_TIMEOUT_MS + 1);
    assert!(!leftover.need_backend_update);
    assert!(!leftover.backend_updated);
    assert!(
        plnd.backend_los_meas().is_some(),
        "disabled update must not expire LOS"
    );
}

#[test]
fn irlock_update_stays_leftover() {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled: true,
        sensor_type: Type::Irlock,
        ..PrecLandParams::default()
    });
    let _ = plnd.init(400);
    let leftover = plnd.update(100.0, true, 41);
    assert!(leftover.need_backend_update);
    assert!(!leftover.backend_updated);
}

#[test]
fn leftover_catalog_drops_backend_and_mavlink() {
    assert!(
        REMAINING.len() > 8,
        "backend slice must not claim the 1,133-loc ticket is done"
    );
    assert!(!REMAINING.contains(&"AC_PrecLand_Backend::update"));
    assert!(!REMAINING.contains(&"AC_PrecLand_Backend::handle_msg"));
    assert!(!REMAINING.contains(&"AC_PrecLand_MAVLink::update"));
    assert!(!REMAINING.contains(&"AC_PrecLand_MAVLink::handle_msg"));
    assert!(REMAINING.contains(&"AC_PrecLand::Write_Precland"));
    assert!(!REMAINING.contains(&"AC_PrecLand_IRLock::update"));
    assert!(!REMAINING.contains(&"AC_PrecLand_SITL_Gazebo::update"));
    assert!(!REMAINING.contains(&"AC_PrecLand_SITL::update"));
    assert!(REMAINING.contains(&"AC_PrecLand_SITL::init(AP::sitl)"));
    assert!(REMAINING.contains(&"AC_PrecLand_StateMachine::update"));
}

#[test]
fn retrieve_los_meas_reads_mavlink_sample() {
    let mut plnd = mavlink_inited();
    let _ = plnd.handle_msg(body_position_packet(), 3_000);
    let sample = plnd.backend_los_sample();
    let (vec, frame) = plnd
        .retrieve_los_meas(sample)
        .expect("new backend measurement");
    assert_eq!(frame, VectorFrame::BodyFrd);
    almost_vec(vec, Vector3f::new(0.2, -0.3, 1.0));
    assert_eq!(plnd.last_backend_los_meas_ms(), 3_000);

    assert!(
        plnd.retrieve_los_meas(plnd.backend_los_sample()).is_none(),
        "same timestamp is not a new measurement"
    );
}
