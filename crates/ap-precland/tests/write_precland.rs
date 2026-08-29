//! `AC_PrecLand::Write_Precland` leftover.
//!
//! Tracked as **COP-028**. `AP::logger().WriteBlock` stays a logger
//! leftover. COP-028 leftovers are closed.

use ap_math::scalar::is_equal;
use ap_math::vector3::Vector3f;
use ap_precland::{
    EstimatorInput, EstimatorType, EstimatorWorld, InertialSample, LosSample, PrecLand,
    PrecLandParams, Type, VectorFrame, LOG_INTERVAL_MS, REMAINING,
};

fn almost(a: f32, b: f32) {
    assert!(is_equal(a, b), "{a} != {b}");
}

fn mavlink(enabled: bool) -> PrecLand {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled,
        sensor_type: Type::Mavlink,
        estimator_type: EstimatorType::KalmanFilter,
        ..PrecLandParams::default()
    });
    let leftover = plnd.init(400);
    assert!(!leftover.skipped);
    plnd
}

#[test]
fn write_precland_skips_when_disabled() {
    let mut plnd = mavlink(false);
    let leftover = plnd.write_precland(1_000, 1_000_000);
    assert!(leftover.skipped);
    assert!(leftover.packet.is_none());
    assert!(!leftover.need_write_block);
}

#[test]
fn write_precland_packs_zeros_when_no_target() {
    let mut plnd = mavlink(true);
    assert!(plnd.healthy());
    let leftover = plnd.write_precland(50, 50_000);
    assert!(!leftover.skipped);
    assert!(leftover.need_write_block);
    let pkt = leftover.packet.expect("packed");
    assert_eq!(pkt.time_us, 50_000);
    assert_eq!(pkt.healthy, 1);
    assert_eq!(pkt.target_acquired, 0);
    almost(pkt.pos_x, 0.0);
    almost(pkt.pos_y, 0.0);
    almost(pkt.vel_x, 0.0);
    almost(pkt.vel_y, 0.0);
    almost(pkt.meas_x, 0.0);
    almost(pkt.meas_y, 0.0);
    almost(pkt.meas_z, 0.0);
    assert_eq!(pkt.last_meas, 0);
    assert_eq!(pkt.ekf_outcount, 0);
    assert_eq!(pkt.estimator, EstimatorType::KalmanFilter as u8);
}

#[test]
fn write_precland_records_retrieve_los_last_meas() {
    let mut plnd = mavlink(true);
    let los = LosSample {
        time_ms: 4_200,
        vec_unit: Vector3f::new(0.0, 0.0, 1.0),
        frame: VectorFrame::BodyFrd,
        distance_to_target_m: 0.0,
    };
    assert!(plnd.retrieve_los_meas(Some(los)).is_some());
    let leftover = plnd.write_precland(4_200, 4_200_000);
    let pkt = leftover.packet.expect("packed");
    assert_eq!(pkt.last_meas, 4_200);
    assert_eq!(pkt.healthy, 1);
}

#[test]
fn write_precland_matches_getters_after_estimator() {
    let mut plnd = PrecLand::from_params(PrecLandParams {
        enabled: true,
        sensor_type: Type::Mavlink,
        estimator_type: EstimatorType::RawSensor,
        ..PrecLandParams::default()
    });
    let _ = plnd.init(400);

    let delayed = InertialSample {
        inertial_nav_velocity: Vector3f::new(0.5, -0.25, 0.0),
        inertial_nav_velocity_valid: true,
        dt: 0.002_5,
        ..InertialSample::default()
    };
    let los = LosSample {
        time_ms: 100,
        vec_unit: Vector3f::new(0.0, 0.0, 1.0),
        frame: VectorFrame::BodyFrd,
        distance_to_target_m: 2.0,
    };
    let input = EstimatorInput {
        rangefinder_alt_m: 2.0,
        rangefinder_alt_valid: true,
        now_ms: 100,
        delayed,
        any_inertial_nav_invalid: false,
        los: Some(los),
        world: EstimatorWorld::default(),
    };
    let _ = plnd.run_estimator(input);

    let packed = plnd.write_precland(100, 100_000);
    assert!(!packed.skipped);
    let pkt = packed.packet.expect("packed");
    assert_eq!(pkt.estimator, EstimatorType::RawSensor as u8);
    assert_eq!(
        pkt.target_acquired,
        u8::from(plnd.estimator_target_acquired())
    );
    let meas = plnd.get_target_position_measurement_ned_m();
    almost(pkt.meas_x, meas.x);
    almost(pkt.meas_y, meas.y);
    almost(pkt.meas_z, meas.z);
    let pos = plnd
        .get_target_position_relative_ne_m(100)
        .unwrap_or_default();
    almost(pkt.pos_x, pos.x);
    almost(pkt.pos_y, pos.y);
}

#[test]
fn update_cadence_runs_write_precland() {
    let mut plnd = mavlink(true);
    let first = plnd.update(100.0, true, LOG_INTERVAL_MS);
    assert!(!first.need_write_precland);
    assert!(first.write_precland.is_none());

    let second = plnd.update(100.0, true, LOG_INTERVAL_MS + 1);
    assert!(second.need_write_precland);
    let write = second.write_precland.expect("cadence packed");
    assert!(!write.skipped);
    assert!(write.need_write_block);
    let pkt = write.packet.expect("packet");
    assert_eq!(pkt.time_us, u64::from(LOG_INTERVAL_MS + 1) * 1000);
}

#[test]
fn update_cadence_skips_write_when_disabled() {
    let mut plnd = mavlink(false);
    let leftover = plnd.update(100.0, true, LOG_INTERVAL_MS + 1);
    assert!(leftover.need_write_precland);
    let write = leftover.write_precland.expect("cadence ran");
    assert!(write.skipped);
    assert!(!write.need_write_block);
}

#[test]
fn leftover_catalog_drops_write_precland() {
    assert!(!REMAINING.contains(&"AC_PrecLand::Write_Precland"));
    assert!(!REMAINING.contains(&"AC_PrecLand_StateMachine::update"));
}
