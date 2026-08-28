//! NAV_LOITER_TIME / timed-loiter command (do + verify).

use ap_math::location::Location;
use ap_mission::{
    do_loiter_time, is_nav_loiter_time, loiter_time_cmd, loiter_time_max_ms, verify_loiter_time,
    DoLoiterTimeInputs, MavFrame, VerifyLoiterTimeInputs, FIRST_REAL_COMMAND,
    MAV_CMD_NAV_LOITER_TIME, MAV_CMD_NAV_WAYPOINT,
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
fn command_id_is_mav_cmd_nav_loiter_time() {
    let cmd = loiter_time_cmd(
        FIRST_REAL_COMMAND,
        MavFrame::GlobalRelativeAlt,
        -35_362_000,
        149_166_000,
        12_000,
    );
    assert_eq!(MAV_CMD_NAV_LOITER_TIME, 19);
    assert_eq!(cmd.command, MAV_CMD_NAV_LOITER_TIME);
    assert!(is_nav_loiter_time(cmd.command));
    assert!(!is_nav_loiter_time(MAV_CMD_NAV_WAYPOINT));
    assert_eq!(cmd.seq, 1);
    assert_eq!(cmd.location.lat, -35_362_000);
    assert_eq!(cmd.location.lng, 149_166_000);
    assert_eq!(cmd.location.alt, 12_000);
}

#[test]
fn do_loiter_time_uses_command_location_and_cw_default() {
    let cmd_loc = Location::new_with_alt(
        -35_361_000,
        149_167_000,
        15_000,
        ap_math::location::AltFrame::AboveHome,
    );
    let out = do_loiter_time(&DoLoiterTimeInputs {
        current_loc: here(),
        cmd_loc,
        cmd_p1: 45,
    });
    assert_eq!(out.next_wp.lat, -35_361_000);
    assert_eq!(out.next_wp.lng, 149_167_000);
    assert_eq!(out.next_wp.alt, 15_000);
    assert_eq!(out.loiter_direction, 1, "loiter_ccw unset is clockwise");
    assert_eq!(out.time_max_ms, 45_000);
    assert_eq!(out.condition_value, 1);
}

#[test]
fn do_loiter_time_decodes_ccw_from_location_flag() {
    let mut cmd_loc = Location::new(-35_361_000, 149_167_000);
    cmd_loc.loiter_ccw = true;
    let out = do_loiter_time(&DoLoiterTimeInputs {
        current_loc: here(),
        cmd_loc,
        cmd_p1: 20,
    });
    assert_eq!(out.loiter_direction, -1);
    assert!(out.next_wp.loiter_ccw);
}

#[test]
fn do_loiter_time_sanitizes_zero_latlng_to_current() {
    let cmd_loc = Location::new_with_alt(0, 0, 8_000, ap_math::location::AltFrame::AboveHome);
    let current = here();
    let out = do_loiter_time(&DoLoiterTimeInputs {
        current_loc: current,
        cmd_loc,
        cmd_p1: 10,
    });
    assert_eq!(out.next_wp.lat, current.lat);
    assert_eq!(out.next_wp.lng, current.lng);
    assert_eq!(out.next_wp.alt, 8_000, "alt comes from the command");
}

#[test]
fn do_loiter_time_p1_seconds_become_milliseconds() {
    assert_eq!(loiter_time_max_ms(0), 0);
    assert_eq!(loiter_time_max_ms(1), 1_000);
    assert_eq!(loiter_time_max_ms(90), 90_000);
}

#[test]
fn verify_loiter_time_incomplete_until_hold_elapses() {
    let time_max_ms = loiter_time_max_ms(30);

    let inbound = verify_loiter_time(&VerifyLoiterTimeInputs {
        now_ms: 5_000,
        start_time_ms: 0,
        time_max_ms,
        reached_loiter_target: false,
        sum_cd: 100,
        condition_value: 1,
    });
    assert!(
        !inbound.complete,
        "must reach the loiter before starting the timer"
    );
    assert_eq!(inbound.start_time_ms, 0);
    assert_eq!(inbound.loiter_radius_m, 0);
    assert_eq!(inbound.condition_value, 1);

    let reached_but_no_orbit = verify_loiter_time(&VerifyLoiterTimeInputs {
        now_ms: 5_000,
        start_time_ms: 0,
        time_max_ms,
        reached_loiter_target: true,
        sum_cd: 1,
        condition_value: 1,
    });
    assert!(
        !reached_but_no_orbit.complete,
        "sum_cd must be > 1 (upstream uses >)"
    );
    assert_eq!(reached_but_no_orbit.start_time_ms, 0);

    let started = verify_loiter_time(&VerifyLoiterTimeInputs {
        now_ms: 5_000,
        start_time_ms: 0,
        time_max_ms,
        reached_loiter_target: true,
        sum_cd: 2,
        condition_value: 1,
    });
    assert!(!started.complete, "timer starts; hold has not elapsed");
    assert_eq!(started.start_time_ms, 5_000);

    let holding = verify_loiter_time(&VerifyLoiterTimeInputs {
        now_ms: 5_000 + time_max_ms,
        start_time_ms: 5_000,
        time_max_ms,
        reached_loiter_target: true,
        sum_cd: 2,
        condition_value: 1,
    });
    assert!(
        !holding.complete,
        "elapsed must exceed time_max_ms (upstream uses >)"
    );
    assert_eq!(holding.condition_value, 1);

    let done = verify_loiter_time(&VerifyLoiterTimeInputs {
        now_ms: 5_000 + time_max_ms + 1,
        start_time_ms: 5_000,
        time_max_ms,
        reached_loiter_target: true,
        sum_cd: 2,
        condition_value: 1,
    });
    assert!(done.complete);
    assert_eq!(done.condition_value, 0);
    assert_eq!(done.loiter_radius_m, 0);
    assert_eq!(done.start_time_ms, 5_000);
}

#[test]
fn verify_loiter_time_always_uses_default_radius() {
    let out = verify_loiter_time(&VerifyLoiterTimeInputs {
        now_ms: 1_000,
        start_time_ms: 0,
        time_max_ms: 10_000,
        reached_loiter_target: false,
        sum_cd: 0,
        condition_value: 1,
    });
    assert!(!out.complete);
    assert_eq!(
        out.loiter_radius_m, 0,
        "update_loiter(0) substitutes WP_LOITER_RAD later"
    );
}
