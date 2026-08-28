//! ARMING_MIS_ITEMS / mission-item pre-arm check.

use ap_arming::mission_items::{
    mis_item_required, mission_items_named_check, mission_starts_with_takeoff, FIRST_REAL_COMMAND,
    MAV_CMD_NAV_TAKEOFF, MAV_CMD_NAV_WAYPOINT, MISSION_CHECK_NAME, ARMING_MIS_ITEMS_DEFAULT,
    MIS_ITEM_CHECK_LAND, MIS_ITEM_CHECK_TAKEOFF,
};
use ap_arming::{Arming, Check, PreArmOutcome};

#[test]
fn plane_default_mis_items_requires_nothing() {
    assert_eq!(ARMING_MIS_ITEMS_DEFAULT, 0);
    assert!(!mis_item_required(ARMING_MIS_ITEMS_DEFAULT, MIS_ITEM_CHECK_TAKEOFF));
    assert!(!mis_item_required(ARMING_MIS_ITEMS_DEFAULT, MIS_ITEM_CHECK_LAND));
}

#[test]
fn takeoff_bit_is_upstream_bit_three() {
    assert_eq!(MIS_ITEM_CHECK_TAKEOFF, 1 << 3);
    assert!(mis_item_required(MIS_ITEM_CHECK_TAKEOFF, MIS_ITEM_CHECK_TAKEOFF));
    assert!(!mis_item_required(MIS_ITEM_CHECK_TAKEOFF, MIS_ITEM_CHECK_LAND));
}

#[test]
fn empty_mission_fails() {
    assert!(!mission_starts_with_takeoff(0, MAV_CMD_NAV_TAKEOFF));
    assert!(!mission_starts_with_takeoff(FIRST_REAL_COMMAND, MAV_CMD_NAV_TAKEOFF));
    let named = mission_items_named_check(0, MAV_CMD_NAV_TAKEOFF);
    assert_eq!(named.check, Check::Mission);
    assert_eq!(named.name, MISSION_CHECK_NAME);
    assert!(!named.ok);
}

#[test]
fn first_flown_waypoint_fails_the_mission_named_check() {
    assert!(!mission_starts_with_takeoff(2, MAV_CMD_NAV_WAYPOINT));
    let named = mission_items_named_check(2, MAV_CMD_NAV_WAYPOINT);
    assert_eq!(named.check, Check::Mission);
    assert!(!named.ok);
}

#[test]
fn first_flown_takeoff_passes() {
    assert!(mission_starts_with_takeoff(2, MAV_CMD_NAV_TAKEOFF));
    let named = mission_items_named_check(2, MAV_CMD_NAV_TAKEOFF);
    assert_eq!(named.check, Check::Mission);
    assert_eq!(named.name, MISSION_CHECK_NAME);
    assert!(named.ok);
}

#[test]
fn registry_refuses_when_mission_is_empty() {
    let arming = Arming::new();
    let named = mission_items_named_check(0, MAV_CMD_NAV_TAKEOFF);
    assert_eq!(
        arming.pre_arm_checks(&[named]),
        PreArmOutcome::Refused {
            check: Check::Mission,
            name: MISSION_CHECK_NAME,
        }
    );
}

#[test]
fn registry_refuses_when_first_flown_item_is_not_takeoff() {
    let arming = Arming::new();
    let named = mission_items_named_check(3, MAV_CMD_NAV_WAYPOINT);
    assert_eq!(
        arming.pre_arm_checks(&[named]),
        PreArmOutcome::Refused {
            check: Check::Mission,
            name: MISSION_CHECK_NAME,
        }
    );
}

#[test]
fn registry_allows_when_first_flown_item_is_takeoff() {
    let arming = Arming::new();
    let named = mission_items_named_check(2, MAV_CMD_NAV_TAKEOFF);
    assert_eq!(arming.pre_arm_checks(&[named]), PreArmOutcome::Allowed);
}

#[test]
fn skipping_mission_lets_an_empty_plan_through() {
    let arming = Arming {
        checks_to_skip: Check::Mission.as_u32(),
        ..Arming::new()
    };
    let named = mission_items_named_check(0, MAV_CMD_NAV_WAYPOINT);
    assert_eq!(arming.pre_arm_checks(&[named]), PreArmOutcome::Allowed);
}
