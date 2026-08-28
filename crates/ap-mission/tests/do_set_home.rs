//! DO_SET_HOME / current-or-specified-LLA mission command.

use ap_math::location::Location;
use ap_mission::{
    do_set_home, do_set_home_cmd, home_content, is_do_set_home, set_home_location_valid,
    set_home_use_current, DoSetHomeInputs, Mission, FIRST_REAL_COMMAND, MAV_CMD_DO_SET_HOME,
    MAV_CMD_NAV_WAYPOINT, SET_HOME_USE_CURRENT,
};

fn specified_lla() -> Location {
    Location::new_with_alt(
        -35_363_261,
        149_165_237,
        58_400,
        ap_math::location::AltFrame::Absolute,
    )
}

fn current_lla() -> Location {
    Location::new_with_alt(
        -35_280_000,
        149_120_000,
        60_000,
        ap_math::location::AltFrame::Absolute,
    )
}

#[test]
fn command_id_is_mav_cmd_do_set_home() {
    let cmd = do_set_home_cmd(FIRST_REAL_COMMAND);
    assert_eq!(MAV_CMD_DO_SET_HOME, 179);
    assert_eq!(cmd.command, MAV_CMD_DO_SET_HOME);
    assert!(is_do_set_home(cmd.command));
    assert!(!is_do_set_home(MAV_CMD_NAV_WAYPOINT));
    assert_eq!(cmd.seq, 1);
}

#[test]
fn home_content_packs_p1_and_location() {
    let loc = specified_lla();
    let current = home_content(SET_HOME_USE_CURRENT, loc);
    assert_eq!(current.p1, 1);
    assert_eq!(current.location, loc);
    let specified = home_content(0, loc);
    assert_eq!(specified.p1, 0);
    assert_eq!(specified.location.lat, -35_363_261);
}

#[test]
fn do_set_home_cmd_round_trips_through_mission_storage() {
    let mut mission = Mission::new();
    assert!(mission.add_cmd(ap_mission::MissionCommand::waypoint(
        0,
        ap_mission::MavFrame::Global,
        1,
        2,
        3,
    )));
    assert!(mission.add_cmd(do_set_home_cmd(99)));
    let stored = mission.read_cmd(1).expect("seq 1 written");
    assert_eq!(stored.seq, 1);
    assert_eq!(stored.command, MAV_CMD_DO_SET_HOME);
    assert!(is_do_set_home(stored.command));
}

#[test]
fn do_set_home_uses_current_when_p1_is_1_and_gps_ok() {
    let current = current_lla();
    let specified = specified_lla();
    assert!(set_home_use_current(SET_HOME_USE_CURRENT, true));
    let out = do_set_home(&DoSetHomeInputs {
        p1: SET_HOME_USE_CURRENT,
        specified,
        current,
        gps_ok_3d: true,
        home: Location::new(0, 0),
    });
    assert!(out.applied);
    assert!(
        out.used_current,
        "p1==1 with a 3D fix writes gps.location()"
    );
    assert_eq!(out.home, current);
    assert_ne!(out.home, specified);
}

#[test]
fn do_set_home_uses_specified_lla_when_p1_is_0() {
    let current = current_lla();
    let specified = specified_lla();
    assert!(!set_home_use_current(0, true));
    let out = do_set_home(&DoSetHomeInputs {
        p1: 0,
        specified,
        current,
        gps_ok_3d: true,
        ..DoSetHomeInputs::default()
    });
    assert!(out.applied);
    assert!(!out.used_current, "p1==0 always uses the item LLA");
    assert_eq!(out.home, specified);
}

#[test]
fn do_set_home_falls_back_to_specified_without_3d_fix() {
    let current = current_lla();
    let specified = specified_lla();
    assert!(!set_home_use_current(SET_HOME_USE_CURRENT, false));
    let out = do_set_home(&DoSetHomeInputs {
        p1: SET_HOME_USE_CURRENT,
        specified,
        current,
        gps_ok_3d: false,
        ..DoSetHomeInputs::default()
    });
    assert!(out.applied);
    assert!(
        !out.used_current,
        "p1==1 without GPS_OK_FIX_3D falls through to the specified LLA"
    );
    assert_eq!(out.home, specified);
}

#[test]
fn do_set_home_rejects_uninitialised_location() {
    assert!(!set_home_location_valid(&Location::new(0, 0)));
    let previous = specified_lla();
    let out = do_set_home(&DoSetHomeInputs {
        p1: 0,
        specified: Location::new(0, 0),
        current: Location::new(0, 0),
        gps_ok_3d: false,
        home: previous,
    });
    assert!(!out.applied, "unset 0,0 is a silent set_home failure");
    assert!(!out.used_current);
    assert_eq!(
        out.home, previous,
        "failed apply leaves AHRS home unchanged"
    );
}

#[test]
fn do_set_home_rejects_out_of_range_latlng() {
    let previous = specified_lla();
    let bad = Location::new(91 * 10_000_000, 0);
    assert!(!set_home_location_valid(&bad));
    let out = do_set_home(&DoSetHomeInputs {
        p1: SET_HOME_USE_CURRENT,
        specified: specified_lla(),
        current: bad,
        gps_ok_3d: true,
        home: previous,
    });
    assert!(out.used_current, "current path was selected");
    assert!(!out.applied, "lat > 90 deg is a silent set_home failure");
    assert_eq!(out.home, previous);
}
