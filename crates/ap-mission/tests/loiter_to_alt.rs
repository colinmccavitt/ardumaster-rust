//! NAV_LOITER_TO_ALT / climb-then-loiter-to-alt command (do + verify).

use ap_math::location::Location;
use ap_mission::{
    do_loiter_to_alt, is_nav_loiter_to_alt, loiter_to_alt_cmd, loiter_to_alt_reached,
    verify_loiter_to_alt, DoLoiterToAltInputs, MavFrame, VerifyLoiterToAltInputs,
    FIRST_REAL_COMMAND, LOITER_TO_ALT_BAND_CM, MAV_CMD_NAV_LOITER_TO_ALT, MAV_CMD_NAV_WAYPOINT,
};

fn here() -> Location {
    Location::new_with_alt(
        -35_363_261,
        149_165_237,
        10_000,
        ap_math::location::AltFrame::AboveHome,
    )
}

#[test]
fn command_id_is_mav_cmd_nav_loiter_to_alt() {
    let cmd = loiter_to_alt_cmd(
        FIRST_REAL_COMMAND,
        MavFrame::GlobalRelativeAlt,
        -35_362_000,
        149_166_000,
        15_000,
    );
    assert_eq!(MAV_CMD_NAV_LOITER_TO_ALT, 31);
    assert_eq!(cmd.command, MAV_CMD_NAV_LOITER_TO_ALT);
    assert!(is_nav_loiter_to_alt(cmd.command));
    assert!(!is_nav_loiter_to_alt(MAV_CMD_NAV_WAYPOINT));
    assert_eq!(cmd.seq, 1);
    assert_eq!(cmd.location.lat, -35_362_000);
    assert_eq!(cmd.location.lng, 149_166_000);
    assert_eq!(cmd.location.alt, 15_000);
}

#[test]
fn do_loiter_to_alt_uses_command_location_and_cw_default() {
    let cmd_loc = Location::new_with_alt(
        -35_361_000,
        149_167_000,
        15_000,
        ap_math::location::AltFrame::AboveHome,
    );
    let out = do_loiter_to_alt(&DoLoiterToAltInputs {
        current_loc: here(),
        cmd_loc,
        cmd_p1: 80,
    });
    assert_eq!(out.next_wp.lat, -35_361_000);
    assert_eq!(out.next_wp.lng, 149_167_000);
    assert_eq!(out.next_wp.alt, 15_000);
    assert_eq!(out.loiter_direction, 1, "loiter_ccw unset is clockwise");
    assert_eq!(out.loiter_radius_m, 80);
    assert_eq!(out.condition_value, 0, "alt never reached at start");
}

#[test]
fn do_loiter_to_alt_decodes_ccw_from_location_flag() {
    let mut cmd_loc = Location::new(-35_361_000, 149_167_000);
    cmd_loc.loiter_ccw = true;
    let out = do_loiter_to_alt(&DoLoiterToAltInputs {
        current_loc: here(),
        cmd_loc,
        cmd_p1: 60,
    });
    assert_eq!(out.loiter_direction, -1);
    assert!(out.next_wp.loiter_ccw);
}

#[test]
fn do_loiter_to_alt_sanitizes_zero_latlng_to_current() {
    let cmd_loc = Location::new_with_alt(0, 0, 18_000, ap_math::location::AltFrame::AboveHome);
    let current = here();
    let out = do_loiter_to_alt(&DoLoiterToAltInputs {
        current_loc: current,
        cmd_loc,
        cmd_p1: 90,
    });
    assert_eq!(out.next_wp.lat, current.lat);
    assert_eq!(out.next_wp.lng, current.lng);
    assert_eq!(out.next_wp.alt, 18_000, "alt comes from the command");
}

#[test]
fn verify_loiter_to_alt_incomplete_until_climb_then_circle() {
    let target_alt_cm = 15_000;
    let inbound = verify_loiter_to_alt(&VerifyLoiterToAltInputs {
        cmd_p1: 80,
        current_alt_cm: 10_000,
        target_alt_cm,
        reached_loiter_target: false,
        sum_cd: 2,
        unable_to_achieve_target_alt: false,
        condition_value: 0,
    });
    assert!(
        !inbound.complete,
        "must reach the loiter before checking altitude"
    );
    assert_eq!(inbound.loiter_radius_m, 80);
    assert_eq!(inbound.condition_value, 0);

    let climbing = verify_loiter_to_alt(&VerifyLoiterToAltInputs {
        cmd_p1: 80,
        current_alt_cm: 12_000,
        target_alt_cm,
        reached_loiter_target: true,
        sum_cd: 4_000,
        unable_to_achieve_target_alt: false,
        condition_value: 0,
    });
    assert!(
        !climbing.complete,
        "still climbing: 30 m below target is outside the 5 m band"
    );
    assert!(!loiter_to_alt_reached(12_000, target_alt_cm, true));
    assert_eq!(climbing.condition_value, 0);

    let not_yet_circling = verify_loiter_to_alt(&VerifyLoiterToAltInputs {
        cmd_p1: 80,
        current_alt_cm: target_alt_cm,
        target_alt_cm,
        reached_loiter_target: true,
        sum_cd: 1,
        unable_to_achieve_target_alt: false,
        condition_value: 0,
    });
    assert!(
        !not_yet_circling.complete,
        "sum_cd must exceed 1 (upstream uses labs(sum_cd) > 1)"
    );

    let at_alt = verify_loiter_to_alt(&VerifyLoiterToAltInputs {
        cmd_p1: 80,
        current_alt_cm: target_alt_cm - (LOITER_TO_ALT_BAND_CM - 1),
        target_alt_cm,
        reached_loiter_target: true,
        sum_cd: 2,
        unable_to_achieve_target_alt: false,
        condition_value: 0,
    });
    assert!(at_alt.complete, "inside the 5 m band while circling");
    assert_eq!(at_alt.condition_value, 1);
    assert_eq!(at_alt.loiter_radius_m, 80);
}

#[test]
fn verify_loiter_to_alt_stuck_completes_primary_goal() {
    let done = verify_loiter_to_alt(&VerifyLoiterToAltInputs {
        cmd_p1: 100,
        current_alt_cm: 10_000,
        target_alt_cm: 20_000,
        reached_loiter_target: true,
        sum_cd: 3 * 36_000,
        unable_to_achieve_target_alt: true,
        condition_value: 0,
    });
    assert!(done.complete, "unable_to_achieve_target_alt ends the climb");
    assert_eq!(done.condition_value, 1);
    assert_eq!(done.loiter_radius_m, 100);
}
