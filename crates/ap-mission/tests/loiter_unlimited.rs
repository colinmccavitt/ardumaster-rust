//! NAV_LOITER_UNLIM / loiter-unlimited command (do + verify).

use ap_math::location::Location;
use ap_mission::{
    do_loiter_unlimited, is_nav_loiter_unlim, loiter_unlimited_cmd, verify_loiter_unlim,
    DoLoiterUnlimitedInputs, MavFrame, VerifyLoiterUnlimInputs, FIRST_REAL_COMMAND,
    MAV_CMD_NAV_LOITER_UNLIM, MAV_CMD_NAV_WAYPOINT,
};

fn here() -> Location {
    Location::new_with_alt(-35_363_261, 149_165_237, 10_000, ap_math::location::AltFrame::AboveHome)
}

#[test]
fn command_id_is_mav_cmd_nav_loiter_unlim() {
    let cmd = loiter_unlimited_cmd(
        FIRST_REAL_COMMAND,
        MavFrame::GlobalRelativeAlt,
        -35_362_000,
        149_166_000,
        12_000,
    );
    assert_eq!(MAV_CMD_NAV_LOITER_UNLIM, 17);
    assert_eq!(cmd.command, MAV_CMD_NAV_LOITER_UNLIM);
    assert!(is_nav_loiter_unlim(cmd.command));
    assert!(!is_nav_loiter_unlim(MAV_CMD_NAV_WAYPOINT));
    assert_eq!(cmd.seq, 1);
    assert_eq!(cmd.location.lat, -35_362_000);
    assert_eq!(cmd.location.lng, 149_166_000);
    assert_eq!(cmd.location.alt, 12_000);
}

#[test]
fn do_loiter_unlimited_uses_command_location_and_cw_default() {
    let cmd_loc = Location::new_with_alt(
        -35_361_000,
        149_167_000,
        15_000,
        ap_math::location::AltFrame::AboveHome,
    );
    let out = do_loiter_unlimited(&DoLoiterUnlimitedInputs {
        current_loc: here(),
        cmd_loc,
    });
    assert_eq!(out.next_wp.lat, -35_361_000);
    assert_eq!(out.next_wp.lng, 149_167_000);
    assert_eq!(out.next_wp.alt, 15_000);
    assert_eq!(out.loiter_direction, 1, "loiter_ccw unset is clockwise");
}

#[test]
fn do_loiter_unlimited_decodes_ccw_from_location_flag() {
    let mut cmd_loc = Location::new(-35_361_000, 149_167_000);
    cmd_loc.loiter_ccw = true;
    let out = do_loiter_unlimited(&DoLoiterUnlimitedInputs {
        current_loc: here(),
        cmd_loc,
    });
    assert_eq!(out.loiter_direction, -1);
    assert!(out.next_wp.loiter_ccw);
}

#[test]
fn do_loiter_unlimited_sanitizes_zero_latlng_to_current() {
    let cmd_loc = Location::new_with_alt(0, 0, 8_000, ap_math::location::AltFrame::AboveHome);
    let current = here();
    let out = do_loiter_unlimited(&DoLoiterUnlimitedInputs {
        current_loc: current,
        cmd_loc,
    });
    assert_eq!(out.next_wp.lat, current.lat);
    assert_eq!(out.next_wp.lng, current.lng);
    assert_eq!(out.next_wp.alt, 8_000, "alt comes from the command");
}

#[test]
fn verify_loiter_unlim_never_completes() {
    let out = verify_loiter_unlim(&VerifyLoiterUnlimInputs { cmd_p1: 150 });
    assert!(!out.complete, "unlimited loiter is never a reached waypoint");
    assert_eq!(out.loiter_radius_m, 150);
}

#[test]
fn verify_loiter_unlim_zero_p1_means_default_radius() {
    let out = verify_loiter_unlim(&VerifyLoiterUnlimInputs { cmd_p1: 0 });
    assert!(!out.complete);
    assert_eq!(
        out.loiter_radius_m, 0,
        "update_loiter(0) substitutes WP_LOITER_RAD later"
    );
}
