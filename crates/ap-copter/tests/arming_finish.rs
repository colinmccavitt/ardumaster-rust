//! The last COP-020 slices: winch, terrain database, arm/disarm guards.

use ap_copter::arming::{
    arm_entry, disarm_guard, terrain_database_required, winch_check, ArmEntry, ArmingMethod,
    DisarmRefusal, TerrainSource,
};

#[test]
fn winch_check_passes_when_checks_are_disabled() {
    assert!(winch_check(false, true, false));
}

#[test]
fn winch_check_passes_when_no_winch_is_fitted() {
    assert!(winch_check(true, false, false));
}

#[test]
fn winch_check_delegates_to_the_winch_when_present() {
    assert!(winch_check(true, true, true));
    assert!(!winch_check(true, true, false));
}

#[test]
fn rangefinder_terrain_never_requires_the_database() {
    assert!(!terrain_database_required(
        TerrainSource::Rangefinder,
        true,
        true,
    ));
}

#[test]
fn database_terrain_with_rtl_above_terrain_requires_the_database() {
    assert!(terrain_database_required(
        TerrainSource::Database,
        true,
        false,
    ));
}

#[test]
fn database_terrain_without_rtl_above_terrain_defers_to_shared() {
    assert!(terrain_database_required(
        TerrainSource::Database,
        false,
        true,
    ));
    assert!(!terrain_database_required(
        TerrainSource::Database,
        false,
        false,
    ));
}

#[test]
fn disarm_is_allowed_when_already_disarmed() {
    assert_eq!(
        disarm_guard(false, true, ArmingMethod::GroundStation, false, false),
        None,
    );
}

#[test]
fn gcs_disarm_is_refused_in_flight() {
    assert_eq!(
        disarm_guard(
            true,
            true,
            ArmingMethod::GroundStation,
            false,
            false,
        ),
        Some(DisarmRefusal::FlyingViaGroundStation),
    );
}

#[test]
fn gcs_disarm_is_allowed_on_the_ground() {
    assert_eq!(
        disarm_guard(
            true,
            true,
            ArmingMethod::GroundStation,
            true,
            false,
        ),
        None,
    );
}

#[test]
fn rudder_disarm_is_refused_in_auto_throttle_while_flying() {
    assert_eq!(
        disarm_guard(true, true, ArmingMethod::Pilot, false, false),
        Some(DisarmRefusal::FlyingViaRudder),
    );
}

#[test]
fn rudder_disarm_is_allowed_in_manual_throttle_while_flying() {
    assert_eq!(
        disarm_guard(true, true, ArmingMethod::Pilot, false, true),
        None,
    );
}

#[test]
fn arm_entry_refuses_reentrancy() {
    assert_eq!(arm_entry(true, false), ArmEntry::Reentrant);
}

#[test]
fn arm_entry_short_circuits_when_already_armed() {
    assert_eq!(arm_entry(false, true), ArmEntry::AlreadyArmed);
}

#[test]
fn arm_entry_proceeds_otherwise() {
    assert_eq!(arm_entry(false, false), ArmEntry::Proceed);
}
