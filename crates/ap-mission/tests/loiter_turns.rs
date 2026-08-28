//! NAV_LOITER_TURNS / loiter-turns command (do + verify).

use ap_math::location::Location;
use ap_mission::{
    do_loiter_turns, is_nav_loiter_turns, loiter_turns_cmd, loiter_turns_radius_m,
    loiter_turns_total_cd, pack_loiter_turns_p1, verify_loiter_turns, DoLoiterTurnsInputs,
    MavFrame, VerifyLoiterTurnsInputs, FIRST_REAL_COMMAND, LOITER_TURNS_CD_PER_ORBIT,
    LOITER_TURNS_FRACTIONAL_BIT, LOITER_TURNS_RADIUS_X10_BIT, MAV_CMD_NAV_LOITER_TURNS,
    MAV_CMD_NAV_WAYPOINT,
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
fn command_id_is_mav_cmd_nav_loiter_turns() {
    let cmd = loiter_turns_cmd(
        FIRST_REAL_COMMAND,
        MavFrame::GlobalRelativeAlt,
        -35_362_000,
        149_166_000,
        12_000,
    );
    assert_eq!(MAV_CMD_NAV_LOITER_TURNS, 18);
    assert_eq!(cmd.command, MAV_CMD_NAV_LOITER_TURNS);
    assert!(is_nav_loiter_turns(cmd.command));
    assert!(!is_nav_loiter_turns(MAV_CMD_NAV_WAYPOINT));
    assert_eq!(cmd.seq, 1);
    assert_eq!(cmd.location.lat, -35_362_000);
    assert_eq!(cmd.location.lng, 149_166_000);
    assert_eq!(cmd.location.alt, 12_000);
}

#[test]
fn do_loiter_turns_uses_command_location_and_cw_default() {
    let cmd_loc = Location::new_with_alt(
        -35_361_000,
        149_167_000,
        15_000,
        ap_math::location::AltFrame::AboveHome,
    );
    let out = do_loiter_turns(&DoLoiterTurnsInputs {
        current_loc: here(),
        cmd_loc,
        cmd_p1: pack_loiter_turns_p1(3, 80),
        type_specific_bits: 0,
    });
    assert_eq!(out.next_wp.lat, -35_361_000);
    assert_eq!(out.next_wp.lng, 149_167_000);
    assert_eq!(out.next_wp.alt, 15_000);
    assert_eq!(out.loiter_direction, 1, "loiter_ccw unset is clockwise");
    assert_eq!(out.total_cd, 3 * LOITER_TURNS_CD_PER_ORBIT);
    assert_eq!(out.condition_value, 1);
}

#[test]
fn do_loiter_turns_decodes_ccw_from_location_flag() {
    let mut cmd_loc = Location::new(-35_361_000, 149_167_000);
    cmd_loc.loiter_ccw = true;
    let out = do_loiter_turns(&DoLoiterTurnsInputs {
        current_loc: here(),
        cmd_loc,
        cmd_p1: pack_loiter_turns_p1(1, 60),
        type_specific_bits: 0,
    });
    assert_eq!(out.loiter_direction, -1);
    assert!(out.next_wp.loiter_ccw);
}

#[test]
fn do_loiter_turns_sanitizes_zero_latlng_to_current() {
    let cmd_loc = Location::new_with_alt(0, 0, 8_000, ap_math::location::AltFrame::AboveHome);
    let current = here();
    let out = do_loiter_turns(&DoLoiterTurnsInputs {
        current_loc: current,
        cmd_loc,
        cmd_p1: pack_loiter_turns_p1(2, 90),
        type_specific_bits: 0,
    });
    assert_eq!(out.next_wp.lat, current.lat);
    assert_eq!(out.next_wp.lng, current.lng);
    assert_eq!(out.next_wp.alt, 8_000, "alt comes from the command");
}

#[test]
fn do_loiter_turns_fractional_turns_are_256ths() {
    // 0.5 orbit stored as 128 with bit 1 set: 128 * 36000 / 256 = 18000.
    let p1 = pack_loiter_turns_p1(128, 50);
    assert_eq!(
        loiter_turns_total_cd(p1, LOITER_TURNS_FRACTIONAL_BIT),
        18_000
    );
    let out = do_loiter_turns(&DoLoiterTurnsInputs {
        current_loc: here(),
        cmd_loc: Location::new(-35_361_000, 149_167_000),
        cmd_p1: p1,
        type_specific_bits: LOITER_TURNS_FRACTIONAL_BIT,
    });
    assert_eq!(out.total_cd, 18_000);
}

#[test]
fn verify_loiter_turns_incomplete_until_orbits_done() {
    let p1 = pack_loiter_turns_p1(2, 120);
    let total_cd = loiter_turns_total_cd(p1, 0);
    let inbound = verify_loiter_turns(&VerifyLoiterTurnsInputs {
        cmd_p1: p1,
        type_specific_bits: 0,
        reached_loiter_target: false,
        sum_cd: total_cd + 100,
        total_cd,
        condition_value: 1,
    });
    assert!(
        !inbound.complete,
        "must reach the loiter before counting orbits"
    );
    assert_eq!(inbound.loiter_radius_m, 120);
    assert_eq!(inbound.condition_value, 1);

    let circling = verify_loiter_turns(&VerifyLoiterTurnsInputs {
        cmd_p1: p1,
        type_specific_bits: 0,
        reached_loiter_target: true,
        sum_cd: total_cd,
        total_cd,
        condition_value: 1,
    });
    assert!(
        !circling.complete,
        "sum_cd must exceed total_cd (upstream uses >)"
    );

    let done = verify_loiter_turns(&VerifyLoiterTurnsInputs {
        cmd_p1: p1,
        type_specific_bits: 0,
        reached_loiter_target: true,
        sum_cd: total_cd + 1,
        total_cd,
        condition_value: 1,
    });
    assert!(done.complete);
    assert_eq!(done.condition_value, 0);
    assert_eq!(done.loiter_radius_m, 120);
}

#[test]
fn verify_loiter_turns_radius_x10_from_type_specific_bit() {
    let p1 = pack_loiter_turns_p1(1, 40);
    assert_eq!(loiter_turns_radius_m(p1, 0), 40);
    assert_eq!(loiter_turns_radius_m(p1, LOITER_TURNS_RADIUS_X10_BIT), 400);
    let out = verify_loiter_turns(&VerifyLoiterTurnsInputs {
        cmd_p1: p1,
        type_specific_bits: LOITER_TURNS_RADIUS_X10_BIT,
        reached_loiter_target: false,
        sum_cd: 0,
        total_cd: LOITER_TURNS_CD_PER_ORBIT,
        condition_value: 1,
    });
    assert!(!out.complete);
    assert_eq!(out.loiter_radius_m, 400);
}
