//! `AC_PrecLand::update` / `handle_msg` leftover.
//!
//! Tracked as **COP-028**. Estimator, inertial history, SITL, and the
//! retry state machine stay later. MAVLink `update` / `handle_msg`
//! and IRLock / SITL-Gazebo `update` run in this crate.

use ap_math::scalar::is_equal;
use ap_precland::{
    HandleMsgLeftover, LandingTargetMsg, PrecLand, PrecLandParams, Type, UpdateLeftover,
    LOG_INTERVAL_MS, REMAINING,
};

fn almost(a: f32, b: f32) {
    assert!(is_equal(a, b), "{a} != {b}");
}

fn mavlink_inited(enabled: bool) -> PrecLand {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled,
        sensor_type: Type::Mavlink,
        ..PrecLandParams::default()
    });
    let leftover = plnd.init(400);
    assert!(!leftover.skipped);
    assert_eq!(leftover.backend, Some(Type::Mavlink));
    plnd
}

#[test]
fn update_skips_before_init() {
    let mut plnd = PrecLand::new();
    let leftover = plnd.update(150.0, true, 41);
    assert!(leftover.skipped);
    almost(leftover.rangefinder_alt_m, 0.0);
    assert!(!leftover.need_inertial_push);
    assert!(!leftover.need_backend_update);
    assert!(!leftover.backend_updated);
    assert!(!leftover.need_run_estimator);
    assert!(!leftover.need_check_target_status);
    assert!(!leftover.need_write_precland);
    assert_eq!(plnd.last_log_ms(), 0);
}

#[test]
fn update_skips_when_type_is_none() {
    let mut plnd = PrecLand::new();
    let init = plnd.init(400);
    assert!(plnd.inertial_history_ready());
    assert_eq!(init.backend, None);

    let leftover = plnd.update(250.0, true, 41);
    assert!(leftover.skipped);
    assert!(!leftover.need_inertial_push);
    assert!(!leftover.need_check_target_status);
}

#[test]
fn update_converts_cm_and_gates_on_enabled() {
    let mut disabled = mavlink_inited(false);
    let leftover = disabled.update(150.0, true, 0);
    assert!(!leftover.skipped);
    almost(leftover.rangefinder_alt_m, 1.5);
    assert!(leftover.rangefinder_alt_valid);
    assert!(leftover.need_inertial_push);
    assert!(!leftover.need_backend_update);
    assert!(!leftover.backend_updated);
    assert!(!leftover.need_run_estimator);
    assert!(leftover.need_check_target_status);
    // `0 - 0 > 40` is false. First tick does not log.
    assert!(!leftover.need_write_precland);

    let mut enabled = mavlink_inited(true);
    let leftover = enabled.update(250.0, false, 0);
    assert!(!leftover.skipped);
    almost(leftover.rangefinder_alt_m, 2.5);
    assert!(!leftover.rangefinder_alt_valid);
    assert!(!leftover.need_backend_update);
    assert!(leftover.backend_updated);
    assert!(leftover.need_run_estimator);
    assert!(leftover.need_check_target_status);
}

#[test]
fn update_set_enabled_opens_the_estimator_gate() {
    let mut plnd = mavlink_inited(false);
    let first = plnd.update(100.0, true, 0);
    assert!(!first.need_backend_update);
    assert!(!first.backend_updated);
    plnd.set_enabled(true);
    let second = plnd.update(100.0, true, 0);
    assert!(!second.need_backend_update);
    assert!(second.backend_updated);
    assert!(second.need_run_estimator);
}

#[test]
fn update_logs_at_25hz() {
    let mut plnd = mavlink_inited(true);
    let first = plnd.update(100.0, true, LOG_INTERVAL_MS);
    assert!(!first.need_write_precland);
    assert_eq!(plnd.last_log_ms(), 0);

    let second = plnd.update(100.0, true, LOG_INTERVAL_MS + 1);
    assert!(second.need_write_precland);
    assert_eq!(plnd.last_log_ms(), LOG_INTERVAL_MS + 1);

    let third = plnd.update(100.0, true, LOG_INTERVAL_MS + 1 + LOG_INTERVAL_MS);
    assert!(!third.need_write_precland);
    assert_eq!(plnd.last_log_ms(), LOG_INTERVAL_MS + 1);

    let fourth = plnd.update(100.0, true, LOG_INTERVAL_MS + 1 + LOG_INTERVAL_MS + 1);
    assert!(fourth.need_write_precland);
    assert_eq!(
        plnd.last_log_ms(),
        LOG_INTERVAL_MS + 1 + LOG_INTERVAL_MS + 1
    );
}

#[test]
fn update_skip_does_not_advance_log_clock() {
    let mut plnd = PrecLand::new();
    let leftover = plnd.update(100.0, true, 1_000);
    assert!(leftover.skipped);
    assert_eq!(plnd.last_log_ms(), 0);
}

#[test]
fn handle_msg_skips_without_backend() {
    let mut plnd = PrecLand::new();
    let packet = LandingTargetMsg {
        frame: 12,
        position_valid: 1,
        distance: 3.0,
        x: 0.1,
        y: 0.2,
        z: 1.0,
        angle_x: 0.05,
        angle_y: -0.04,
    };
    let leftover = plnd.handle_msg(packet, 1_234);
    assert_eq!(
        leftover,
        HandleMsgLeftover {
            skipped: true,
            need_backend_handle_msg: false,
            timestamp_ms: 1_234,
            packet,
            mavlink: None,
        }
    );
}

#[test]
fn handle_msg_dispatches_when_backend_exists() {
    let mut plnd = mavlink_inited(true);
    let packet = LandingTargetMsg {
        frame: 20,
        position_valid: 0,
        distance: 0.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
        angle_x: 0.1,
        angle_y: 0.2,
    };
    let leftover = plnd.handle_msg(packet, 9_001);
    assert!(!leftover.skipped);
    assert!(!leftover.need_backend_handle_msg);
    assert_eq!(leftover.timestamp_ms, 9_001);
    assert_eq!(leftover.packet, packet);
    let mav = leftover.mavlink.expect("MAVLink leftover");
    assert!(mav.accepted);
    assert!(plnd.backend_los_meas().is_some());
}

#[test]
fn leftover_catalog_drops_update_and_handle_msg() {
    assert!(
        REMAINING.len() > 10,
        "update slice must not claim the 1,133-loc ticket is done"
    );
    assert!(!REMAINING.contains(&"AC_PrecLand::update"));
    assert!(!REMAINING.contains(&"AC_PrecLand::handle_msg"));
    assert!(!REMAINING.contains(&"AC_PrecLand::run_estimator"));
    assert!(!REMAINING.contains(&"AC_PrecLand::check_ekf_init_timeout"));
    assert!(!REMAINING.contains(&"AC_PrecLand::construct_pos_meas_using_rangefinder"));
    assert!(!REMAINING.contains(&"AC_PrecLand::retrieve_los_meas"));
    assert!(!REMAINING.contains(&"AC_PrecLand::run_output_prediction"));
    assert!(!REMAINING.contains(&"AC_PrecLand::check_target_status"));
    assert!(REMAINING.contains(&"AC_PrecLand::Write_Precland"));
    assert!(!REMAINING.contains(&"AC_PrecLand_Backend::update"));
    assert!(!REMAINING.contains(&"AC_PrecLand_MAVLink::handle_msg"));
    assert!(!REMAINING.contains(&"AC_PrecLand_IRLock::update"));
    assert!(!REMAINING.contains(&"AC_PrecLand_SITL::update"));
    assert!(REMAINING.contains(&"AC_PrecLand_SITL::init(AP::sitl)"));
    assert!(REMAINING.contains(&"AC_PrecLand_StateMachine::update"));
    assert!(!REMAINING.contains(&"PosVelEKF"));
}

#[test]
fn update_leftover_partial_eq() {
    let leftover = UpdateLeftover {
        skipped: false,
        rangefinder_alt_m: 1.0,
        rangefinder_alt_valid: true,
        need_inertial_push: true,
        need_backend_update: true,
        backend_updated: false,
        need_run_estimator: true,
        need_check_target_status: true,
        need_write_precland: false,
    };
    assert_eq!(leftover, leftover);
}
