//! DO_CHANGE_SPEED / airspeed-groundspeed-throttle mission command.

use ap_math::scalar::is_equal;
use ap_mission::{
    do_change_speed, do_change_speed_cmd, is_do_change_speed, speed_content, DoChangeSpeedInputs,
    Mission, AIRSPEED_MAX_DEFAULT_MS, AIRSPEED_MIN_DEFAULT_MS, CHANGE_SPEED_RESET_MS,
    FIRST_REAL_COMMAND, MAV_CMD_DO_CHANGE_SPEED, MAV_CMD_NAV_WAYPOINT, NEW_AIRSPEED_CM_NONE,
    SPEED_TYPE_AIRSPEED, SPEED_TYPE_CLIMB_SPEED, SPEED_TYPE_DESCENT_SPEED, SPEED_TYPE_GROUNDSPEED,
};

#[test]
fn command_id_is_mav_cmd_do_change_speed() {
    let cmd = do_change_speed_cmd(FIRST_REAL_COMMAND);
    assert_eq!(MAV_CMD_DO_CHANGE_SPEED, 178);
    assert_eq!(cmd.command, MAV_CMD_DO_CHANGE_SPEED);
    assert!(is_do_change_speed(cmd.command));
    assert!(!is_do_change_speed(MAV_CMD_NAV_WAYPOINT));
    assert_eq!(cmd.seq, 1);
}

#[test]
fn speed_content_packs_type_target_and_throttle() {
    let speed = speed_content(SPEED_TYPE_AIRSPEED, 12.0, 60.0);
    assert_eq!(speed.speed_type, SPEED_TYPE_AIRSPEED);
    assert!(is_equal(speed.target_ms, 12.0));
    assert!(is_equal(speed.throttle_pct, 60.0));
    let ground = speed_content(SPEED_TYPE_GROUNDSPEED, 8.0, 0.0);
    assert_eq!(ground.speed_type, SPEED_TYPE_GROUNDSPEED);
    assert!(is_equal(ground.target_ms, 8.0));
}

#[test]
fn do_change_speed_cmd_round_trips_through_mission_storage() {
    let mut mission = Mission::new();
    assert!(mission.add_cmd(ap_mission::MissionCommand::waypoint(
        0,
        ap_mission::MavFrame::Global,
        1,
        2,
        3,
    )));
    assert!(mission.add_cmd(do_change_speed_cmd(99)));
    let stored = mission.read_cmd(1).expect("seq 1 written");
    assert_eq!(stored.seq, 1);
    assert_eq!(stored.command, MAV_CMD_DO_CHANGE_SPEED);
    assert!(is_do_change_speed(stored.command));
}

#[test]
fn do_change_speed_sets_airspeed_in_range() {
    let out = do_change_speed(&DoChangeSpeedInputs {
        speed_type: SPEED_TYPE_AIRSPEED,
        target_ms: 12.0,
        throttle_pct: 80.0,
        ..DoChangeSpeedInputs::default()
    });
    assert!(out.applied);
    assert!(out.set_airspeed);
    assert_eq!(
        out.new_airspeed_cm, 1200,
        "in-range airspeed writes target * 100 and returns before throttle"
    );
    assert!(
        !out.set_throttle,
        "accepted airspeed does not also set throttle"
    );
    assert!(!out.set_groundspeed);
}

#[test]
fn do_change_speed_reset_restores_default_airspeed() {
    let out = do_change_speed(&DoChangeSpeedInputs {
        speed_type: SPEED_TYPE_AIRSPEED,
        target_ms: CHANGE_SPEED_RESET_MS,
        new_airspeed_cm: 1500,
        ..DoChangeSpeedInputs::default()
    });
    assert!(out.applied);
    assert!(out.set_airspeed);
    assert_eq!(
        out.new_airspeed_cm, NEW_AIRSPEED_CM_NONE,
        "-2 m/s clears the AUTO/GUIDED scratch so cruise param is used again"
    );
}

#[test]
fn do_change_speed_rejects_airspeed_outside_min_max() {
    let below = do_change_speed(&DoChangeSpeedInputs {
        speed_type: SPEED_TYPE_AIRSPEED,
        target_ms: AIRSPEED_MIN_DEFAULT_MS - 1.0,
        throttle_pct: 0.0,
        new_airspeed_cm: 777,
        ..DoChangeSpeedInputs::default()
    });
    assert!(
        !below.applied,
        "below AIRSPEED_MIN is a no-op without throttle"
    );
    assert!(!below.set_airspeed);
    assert_eq!(below.new_airspeed_cm, 777);

    let above = do_change_speed(&DoChangeSpeedInputs {
        speed_type: SPEED_TYPE_AIRSPEED,
        target_ms: AIRSPEED_MAX_DEFAULT_MS + 1.0,
        throttle_pct: 0.0,
        new_airspeed_cm: 777,
        ..DoChangeSpeedInputs::default()
    });
    assert!(
        !above.applied,
        "above AIRSPEED_MAX is a no-op without throttle"
    );
    assert_eq!(above.new_airspeed_cm, 777);
}

#[test]
fn do_change_speed_out_of_range_airspeed_falls_through_to_throttle() {
    let out = do_change_speed(&DoChangeSpeedInputs {
        speed_type: SPEED_TYPE_AIRSPEED,
        target_ms: 40.0,
        throttle_pct: 55.0,
        throttle_cruise: 45.0,
        ..DoChangeSpeedInputs::default()
    });
    assert!(out.applied);
    assert!(!out.set_airspeed);
    assert!(out.set_throttle);
    assert!(is_equal(out.throttle_cruise, 55.0));
}

#[test]
fn do_change_speed_sets_groundspeed() {
    let out = do_change_speed(&DoChangeSpeedInputs {
        speed_type: SPEED_TYPE_GROUNDSPEED,
        target_ms: 7.5,
        throttle_pct: 90.0,
        min_groundspeed_ms: 0.0,
        ..DoChangeSpeedInputs::default()
    });
    assert!(out.applied);
    assert!(out.set_groundspeed);
    assert!(is_equal(out.min_groundspeed_ms, 7.5));
    assert!(
        !out.set_throttle,
        "groundspeed returns before the throttle fallback"
    );
    assert!(!out.set_airspeed);
}

#[test]
fn do_change_speed_climb_type_uses_throttle_only() {
    let climb = do_change_speed(&DoChangeSpeedInputs {
        speed_type: SPEED_TYPE_CLIMB_SPEED,
        target_ms: 12.0,
        throttle_pct: 70.0,
        ..DoChangeSpeedInputs::default()
    });
    assert!(climb.applied);
    assert!(!climb.set_airspeed);
    assert!(!climb.set_groundspeed);
    assert!(climb.set_throttle);
    assert!(is_equal(climb.throttle_cruise, 70.0));

    let descent = do_change_speed(&DoChangeSpeedInputs {
        speed_type: SPEED_TYPE_DESCENT_SPEED,
        target_ms: 12.0,
        throttle_pct: 0.0,
        ..DoChangeSpeedInputs::default()
    });
    assert!(
        !descent.applied,
        "ignored climb/descent types with no throttle are a no-op"
    );
}

#[test]
fn do_change_speed_rejects_throttle_outside_percent() {
    let zero = do_change_speed(&DoChangeSpeedInputs {
        speed_type: SPEED_TYPE_CLIMB_SPEED,
        throttle_pct: 0.0,
        throttle_cruise: 45.0,
        ..DoChangeSpeedInputs::default()
    });
    assert!(!zero.applied, "throttle 0 means no change");
    assert!(is_equal(zero.throttle_cruise, 45.0));

    let over = do_change_speed(&DoChangeSpeedInputs {
        speed_type: SPEED_TYPE_CLIMB_SPEED,
        throttle_pct: 101.0,
        throttle_cruise: 45.0,
        ..DoChangeSpeedInputs::default()
    });
    assert!(!over.applied, "throttle > 100 is rejected");
    assert!(is_equal(over.throttle_cruise, 45.0));
}
