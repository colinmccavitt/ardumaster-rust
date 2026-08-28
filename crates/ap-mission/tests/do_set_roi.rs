//! DO_SET_ROI / point-camera-at-location mission command.

use ap_math::location::Location;
use ap_mission::{
    do_set_roi, do_set_roi_cmd, is_do_set_roi, roi_content, roi_location_set, DoSetRoiInputs,
    Mission, FIRST_REAL_COMMAND, MAV_CMD_DO_SET_ROI, MAV_CMD_NAV_WAYPOINT,
    MAV_MOUNT_MODE_GPS_POINT, MAV_MOUNT_MODE_NEUTRAL,
};

fn roi_lla() -> Location {
    Location::new_with_alt(
        -35_363_261,
        149_165_237,
        58_400,
        ap_math::location::AltFrame::Absolute,
    )
}

#[test]
fn command_id_is_mav_cmd_do_set_roi() {
    let cmd = do_set_roi_cmd(FIRST_REAL_COMMAND);
    assert_eq!(MAV_CMD_DO_SET_ROI, 201);
    assert_eq!(cmd.command, MAV_CMD_DO_SET_ROI);
    assert!(is_do_set_roi(cmd.command));
    assert!(!is_do_set_roi(MAV_CMD_NAV_WAYPOINT));
    assert_eq!(cmd.seq, 1);
}

#[test]
fn roi_content_packs_location() {
    let loc = roi_lla();
    let packed = roi_content(loc);
    assert_eq!(packed.location, loc);
    assert_eq!(packed.location.lat, -35_363_261);
    assert!(roi_location_set(&packed.location));
}

#[test]
fn do_set_roi_cmd_round_trips_through_mission_storage() {
    let mut mission = Mission::new();
    assert!(mission.add_cmd(ap_mission::MissionCommand::waypoint(
        0,
        ap_mission::MavFrame::Global,
        1,
        2,
        3,
    )));
    assert!(mission.add_cmd(do_set_roi_cmd(99)));
    let stored = mission.read_cmd(1).expect("seq 1 written");
    assert_eq!(stored.seq, 1);
    assert_eq!(stored.command, MAV_CMD_DO_SET_ROI);
    assert!(is_do_set_roi(stored.command));
}

#[test]
fn do_set_roi_points_mount_at_initialised_location() {
    let loc = roi_lla();
    assert!(roi_location_set(&loc));
    let out = do_set_roi(&DoSetRoiInputs {
        location: loc,
        mount_mode: MAV_MOUNT_MODE_NEUTRAL,
        default_mode: MAV_MOUNT_MODE_NEUTRAL,
        roi: Location::new(0, 0),
    });
    assert!(out.applied, "initialised LLA calls set_roi_target");
    assert!(!out.cleared);
    assert_eq!(out.roi, loc);
    assert_eq!(
        out.mount_mode, MAV_MOUNT_MODE_GPS_POINT,
        "set_roi_target switches the mount to GPS-point tracking"
    );
}

#[test]
fn do_set_roi_clears_gps_point_when_location_unset() {
    let previous = roi_lla();
    assert!(!roi_location_set(&Location::new(0, 0)));
    let out = do_set_roi(&DoSetRoiInputs {
        location: Location::new(0, 0),
        mount_mode: MAV_MOUNT_MODE_GPS_POINT,
        default_mode: MAV_MOUNT_MODE_NEUTRAL,
        roi: previous,
    });
    assert!(!out.applied, "unset 0,0 does not call set_roi_target");
    assert!(
        out.cleared,
        "GPS-point tracking is switched off via set_mode_to_default"
    );
    assert_eq!(
        out.roi, previous,
        "clearing mode leaves the stored ROI target alone"
    );
    assert_eq!(out.mount_mode, MAV_MOUNT_MODE_NEUTRAL);
}

#[test]
fn do_set_roi_leaves_non_gps_point_when_location_unset() {
    let previous = roi_lla();
    let out = do_set_roi(&DoSetRoiInputs {
        location: Location::new(0, 0),
        mount_mode: MAV_MOUNT_MODE_NEUTRAL,
        default_mode: MAV_MOUNT_MODE_NEUTRAL,
        roi: previous,
    });
    assert!(!out.applied);
    assert!(
        !out.cleared,
        "unset LLA is a no-op unless the mount is already GPS-point"
    );
    assert_eq!(out.roi, previous);
    assert_eq!(out.mount_mode, MAV_MOUNT_MODE_NEUTRAL);
}

#[test]
fn do_set_roi_replaces_an_existing_roi_target() {
    let previous = Location::new_with_alt(
        -35_280_000,
        149_120_000,
        60_000,
        ap_math::location::AltFrame::Absolute,
    );
    let loc = roi_lla();
    let out = do_set_roi(&DoSetRoiInputs {
        location: loc,
        mount_mode: MAV_MOUNT_MODE_GPS_POINT,
        default_mode: MAV_MOUNT_MODE_NEUTRAL,
        roi: previous,
    });
    assert!(out.applied);
    assert!(!out.cleared);
    assert_eq!(out.roi, loc);
    assert_ne!(out.roi, previous);
    assert_eq!(out.mount_mode, MAV_MOUNT_MODE_GPS_POINT);
}
