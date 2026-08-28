//! NAV_LAND / land-to-waypoint mission command (do + verify).

use ap_math::location::{AltFrame, Location};
use ap_mission::{
    do_land, is_nav_land, land_abort_altitude_cm, land_cmd, land_verify_height_m, verify_land,
    DoLandInputs, MavFrame, VerifyLandInputs, FIRST_REAL_COMMAND, LAND_ABORT_ALT_DEFAULT_CM,
    LAND_ABORT_PITCH_DEFAULT_CD, MAV_CMD_NAV_LAND, MAV_CMD_NAV_WAYPOINT,
};

fn here() -> Location {
    Location::new_with_alt(-35_363_261, 149_165_237, 10_000, AltFrame::Absolute)
}

fn pad() -> Location {
    Location::new_with_alt(-35_362_000, 149_166_000, 0, AltFrame::Absolute)
}

#[test]
fn command_id_is_mav_cmd_nav_land() {
    let cmd = land_cmd(
        FIRST_REAL_COMMAND,
        MavFrame::Global,
        -35_362_000,
        149_166_000,
        0,
    );
    assert_eq!(MAV_CMD_NAV_LAND, 21);
    assert_eq!(cmd.command, MAV_CMD_NAV_LAND);
    assert!(is_nav_land(cmd.command));
    assert!(!is_nav_land(MAV_CMD_NAV_WAYPOINT));
    assert_eq!(cmd.seq, 1);
    assert_eq!(cmd.location.lat, -35_362_000);
    assert_eq!(cmd.location.lng, 149_166_000);
    assert_eq!(cmd.location.alt, 0);
}

#[test]
fn do_land_uses_command_location() {
    let cmd_loc = pad();
    let out = do_land(&DoLandInputs {
        cmd_loc,
        cmd_p1: 40,
        takeoff_altitude_rel_cm: 0,
        takeoff_pitch_cd: 0,
        abort_landing: false,
    });
    assert_eq!(out.next_wp.lat, cmd_loc.lat);
    assert_eq!(out.next_wp.lng, cmd_loc.lng);
    assert_eq!(out.next_wp.alt, 0);
    assert!(!out.leave_abort);
}

#[test]
fn do_land_uses_p1_abort_altitude_in_metres() {
    let out = do_land(&DoLandInputs {
        cmd_loc: pad(),
        cmd_p1: 40,
        takeoff_altitude_rel_cm: 1_200,
        takeoff_pitch_cd: 1_500,
        abort_landing: false,
    });
    assert_eq!(out.takeoff_altitude_rel_cm, 4_000, "p1 metres * 100");
    assert_eq!(
        land_abort_altitude_cm(40, 1_200),
        4_000,
        "cmd.p1 wins over the last takeoff"
    );
    assert_eq!(out.takeoff_pitch_cd, 1_500, "existing pitch is kept");
}

#[test]
fn do_land_defaults_abort_alt_and_pitch_when_unset() {
    let out = do_land(&DoLandInputs {
        cmd_loc: pad(),
        cmd_p1: 0,
        takeoff_altitude_rel_cm: 0,
        takeoff_pitch_cd: 0,
        abort_landing: false,
    });
    assert_eq!(out.takeoff_altitude_rel_cm, LAND_ABORT_ALT_DEFAULT_CM);
    assert_eq!(out.takeoff_pitch_cd, LAND_ABORT_PITCH_DEFAULT_CD);
}

#[test]
fn do_land_keeps_last_takeoff_alt_when_p1_zero() {
    let out = do_land(&DoLandInputs {
        cmd_loc: pad(),
        cmd_p1: 0,
        takeoff_altitude_rel_cm: 5_000,
        takeoff_pitch_cd: 800,
        abort_landing: false,
    });
    assert_eq!(out.takeoff_altitude_rel_cm, 5_000);
    assert_eq!(out.takeoff_pitch_cd, 800);
}

#[test]
fn do_land_leaves_sticky_abort_stage() {
    let out = do_land(&DoLandInputs {
        cmd_loc: pad(),
        cmd_p1: 30,
        takeoff_altitude_rel_cm: 0,
        takeoff_pitch_cd: 0,
        abort_landing: true,
    });
    assert!(
        out.leave_abort,
        "do_land must set_flight_stage(LAND) when starting from ABORT_LANDING"
    );
    assert_eq!(out.takeoff_altitude_rel_cm, 3_000);
}

#[test]
fn verify_land_incomplete_until_landing_library_completes() {
    let inbound = verify_land(&VerifyLandInputs {
        abort_landing: false,
        height_above_target_m: 12.0,
        terrain_correction_m: 2.0,
        landing_complete: false,
    });
    assert!(!inbound.complete, "approach is still in progress");
    assert!(!inbound.abort_path);
    assert!(
        (inbound.height_m - 10.0).abs() < 1e-6,
        "terrain correction is subtracted (got {})",
        inbound.height_m
    );
    assert!((land_verify_height_m(12.0, 2.0) - 10.0).abs() < 1e-6);

    let done = verify_land(&VerifyLandInputs {
        abort_landing: false,
        height_above_target_m: 0.4,
        terrain_correction_m: 0.0,
        landing_complete: true,
    });
    assert!(done.complete, "landing.verify_land said the item is done");
    assert!(!done.abort_path);
}

#[test]
fn verify_abort_path_never_completes_the_item() {
    let aborting = verify_land(&VerifyLandInputs {
        abort_landing: true,
        height_above_target_m: here().alt as f32 * 0.01,
        terrain_correction_m: 0.0,
        landing_complete: true,
    });
    assert!(
        aborting.abort_path,
        "ABORT_LANDING uses verify_abort_landing"
    );
    assert!(
        !aborting.complete,
        "upstream abort verify always returns false so the mission index is left alone"
    );
}
