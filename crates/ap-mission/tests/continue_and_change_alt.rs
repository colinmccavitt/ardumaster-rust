//! NAV_CONTINUE_AND_CHANGE_ALT / continue-on-heading while changing altitude.

use ap_math::location::{AltContext, AltFrame, Location};
use ap_math::Ftype;
use ap_mission::{
    continue_and_change_alt_cmd, continue_and_change_alt_reached, do_continue_and_change_alt,
    is_nav_continue_and_change_alt, verify_continue_and_change_alt, DoContinueAndChangeAltInputs,
    MavFrame, VerifyContinueAndChangeAltInputs, CHANGE_ALT_CLIMB, CHANGE_ALT_DESCEND,
    CHANGE_ALT_NEUTRAL, CONTINUE_AND_CHANGE_ALT_BAND_CM, CONTINUE_AND_CHANGE_ALT_EXTEND_M,
    CONTINUE_AND_CHANGE_ALT_OFFSET_M, FIRST_REAL_COMMAND, HOLD_COURSE_NONE,
    MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT, MAV_CMD_NAV_WAYPOINT,
};

fn here() -> Location {
    Location::new_with_alt(-35_363_261, 149_165_237, 10_000, AltFrame::Absolute)
}

fn ahead() -> Location {
    Location::new_with_alt(-35_362_000, 149_166_000, 10_000, AltFrame::Absolute)
}

#[test]
fn command_id_is_mav_cmd_nav_continue_and_change_alt() {
    let cmd = continue_and_change_alt_cmd(
        FIRST_REAL_COMMAND,
        MavFrame::Global,
        -35_362_000,
        149_166_000,
        15_000,
    );
    assert_eq!(MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT, 30);
    assert_eq!(cmd.command, MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT);
    assert!(is_nav_continue_and_change_alt(cmd.command));
    assert!(!is_nav_continue_and_change_alt(MAV_CMD_NAV_WAYPOINT));
    assert_eq!(cmd.seq, 1);
    assert_eq!(cmd.location.lat, -35_362_000);
    assert_eq!(cmd.location.lng, 149_166_000);
    assert_eq!(cmd.location.alt, 15_000);
}

#[test]
fn do_uses_waypoint_bearing_when_prev_next_differ() {
    let cmd_loc = Location::new_with_alt(0, 0, 15_000, AltFrame::Absolute);
    let next = ahead();
    let out = do_continue_and_change_alt(&DoContinueAndChangeAltInputs {
        prev_wp: here(),
        next_wp: next,
        cmd_loc,
        cmd_p1: 1,
        gps_ok: true,
        gps_ground_course_deg: 90.0,
        yaw_cd: 18_000,
        alt_ctx: AltContext::default(),
    });
    assert_eq!(
        out.hold_course_cd, HOLD_COURSE_NONE,
        "usual mission-bearing case"
    );
    assert_eq!(
        out.next_wp.lat, next.lat,
        "lat/lng stay on the mission line"
    );
    assert_eq!(out.next_wp.lng, next.lng);
    assert_eq!(out.next_wp.alt, 15_000);
    assert_eq!(out.next_wp.alt_frame(), AltFrame::Absolute);
    assert_eq!(out.condition_value, CHANGE_ALT_CLIMB);
}

#[test]
fn do_projects_gps_course_when_prev_next_same() {
    let start = here();
    let cmd_loc = Location::new_with_alt(0, 0, 18_000, AltFrame::Absolute);
    let out = do_continue_and_change_alt(&DoContinueAndChangeAltInputs {
        prev_wp: start,
        next_wp: start,
        cmd_loc,
        cmd_p1: 2,
        gps_ok: true,
        gps_ground_course_deg: 90.0,
        yaw_cd: 0,
        alt_ctx: AltContext::default(),
    });
    assert_eq!(
        out.hold_course_cd, HOLD_COURSE_NONE,
        "GPS course is still waypoint steering"
    );
    assert_eq!(out.condition_value, CHANGE_ALT_DESCEND);
    assert_eq!(out.next_wp.alt, 18_000);
    let pushed = start.get_distance(out.next_wp);
    assert!(
        (pushed - Ftype::from(CONTINUE_AND_CHANGE_ALT_OFFSET_M)).abs() < Ftype::from(15.0_f32),
        "GPS projection pushes next_WP ~1 km (got {pushed})"
    );
    assert_ne!(
        out.next_wp.lng, start.lng,
        "90 deg ground course is east, so longitude must move"
    );
}

#[test]
fn do_holds_yaw_when_prev_next_same_and_no_gps() {
    let start = here();
    let cmd_loc = Location::new_with_alt(0, 0, 12_000, AltFrame::Absolute);
    let out = do_continue_and_change_alt(&DoContinueAndChangeAltInputs {
        prev_wp: start,
        next_wp: start,
        cmd_loc,
        cmd_p1: 0,
        gps_ok: false,
        gps_ground_course_deg: 0.0,
        yaw_cd: 0,
        alt_ctx: AltContext::default(),
    });
    assert_eq!(
        out.hold_course_cd, 0,
        "wrap_360_cd(yaw_sensor) when GPS is down"
    );
    assert_eq!(out.condition_value, CHANGE_ALT_NEUTRAL);
    let pushed = start.get_distance(out.next_wp);
    assert!(
        (pushed - Ftype::from(CONTINUE_AND_CHANGE_ALT_OFFSET_M)).abs() < Ftype::from(15.0_f32),
        "yaw projection pushes next_WP ~1 km (got {pushed})"
    );
}

#[test]
fn do_copies_terrain_alt_without_converting() {
    let next = ahead();
    let cmd_loc = Location::new_with_alt(0, 0, 4_000, AltFrame::AboveTerrain);
    let out = do_continue_and_change_alt(&DoContinueAndChangeAltInputs {
        prev_wp: here(),
        next_wp: next,
        cmd_loc,
        cmd_p1: 1,
        gps_ok: false,
        gps_ground_course_deg: 0.0,
        yaw_cd: 0,
        alt_ctx: AltContext::default(),
    });
    assert_eq!(out.next_wp.alt, 4_000);
    assert_eq!(out.next_wp.alt_frame(), AltFrame::AboveTerrain);
}

#[test]
fn verify_incomplete_until_altitude_goal() {
    let prev = here();
    let next = Location::new_with_alt(-35_362_000, 149_166_000, 15_000, AltFrame::Absolute);
    let climbing = verify_continue_and_change_alt(&VerifyContinueAndChangeAltInputs {
        prev_wp: prev,
        next_wp: next,
        current_loc: prev,
        hold_course_cd: HOLD_COURSE_NONE,
        condition_value: CHANGE_ALT_CLIMB,
        current_alt_cm: 10_000,
    });
    assert!(
        !climbing.complete,
        "50 m below a climb target is outside the 5 m band"
    );
    assert!(!climbing.heading_hold);
    assert!(!continue_and_change_alt_reached(
        CHANGE_ALT_CLIMB,
        10_000,
        15_000
    ));

    let at_alt = verify_continue_and_change_alt(&VerifyContinueAndChangeAltInputs {
        prev_wp: prev,
        next_wp: next,
        current_loc: prev,
        hold_course_cd: HOLD_COURSE_NONE,
        condition_value: CHANGE_ALT_CLIMB,
        current_alt_cm: 15_000,
    });
    assert!(at_alt.complete, "climb completes at or above the target");

    let descending = verify_continue_and_change_alt(&VerifyContinueAndChangeAltInputs {
        prev_wp: prev,
        next_wp: next,
        current_loc: prev,
        hold_course_cd: HOLD_COURSE_NONE,
        condition_value: CHANGE_ALT_DESCEND,
        current_alt_cm: 20_000,
    });
    assert!(!descending.complete, "still above a descend target");

    let descended = verify_continue_and_change_alt(&VerifyContinueAndChangeAltInputs {
        prev_wp: prev,
        next_wp: next,
        current_loc: prev,
        hold_course_cd: HOLD_COURSE_NONE,
        condition_value: CHANGE_ALT_DESCEND,
        current_alt_cm: 15_000,
    });
    assert!(descended.complete);

    let in_band = verify_continue_and_change_alt(&VerifyContinueAndChangeAltInputs {
        prev_wp: prev,
        next_wp: next,
        current_loc: prev,
        hold_course_cd: HOLD_COURSE_NONE,
        condition_value: CHANGE_ALT_NEUTRAL,
        current_alt_cm: 15_000 - (CONTINUE_AND_CHANGE_ALT_BAND_CM - 1),
    });
    assert!(
        in_band.complete,
        "neutral hint completes inside the 5 m band"
    );
}

#[test]
fn verify_heading_hold_when_waypoints_coincide() {
    let start = here();
    let out = verify_continue_and_change_alt(&VerifyContinueAndChangeAltInputs {
        prev_wp: start,
        next_wp: start,
        current_loc: start,
        hold_course_cd: 9_000,
        condition_value: CHANGE_ALT_NEUTRAL,
        current_alt_cm: start.alt,
    });
    assert!(
        out.heading_hold,
        "coincident WPs + hold_course use heading hold"
    );
    assert!(out.complete, "already at the stored altitude");
    assert_eq!(
        out.next_wp.lat, start.lat,
        "heading-hold does not extend next_WP"
    );
    assert_eq!(out.next_wp.lng, start.lng);
}

#[test]
fn verify_extends_next_wp_when_closer_than_200m() {
    let prev = here();
    let next = Location::new_with_alt(prev.lat + 500, prev.lng, 12_000, AltFrame::Absolute);
    let before = prev.get_distance(next);
    assert!(
        before < Ftype::from(200.0_f32),
        "fixture must start inside the 200 m extend threshold (got {before})"
    );
    let out = verify_continue_and_change_alt(&VerifyContinueAndChangeAltInputs {
        prev_wp: prev,
        next_wp: next,
        current_loc: prev,
        hold_course_cd: HOLD_COURSE_NONE,
        condition_value: CHANGE_ALT_NEUTRAL,
        current_alt_cm: 8_000,
    });
    assert!(!out.heading_hold);
    assert!(!out.complete);
    let after = prev.get_distance(out.next_wp);
    assert!(
        (after - (before + Ftype::from(CONTINUE_AND_CHANGE_ALT_EXTEND_M))).abs()
            < Ftype::from(20.0_f32),
        "verify pushes another 300 m down the line (before={before}, after={after})"
    );
}
