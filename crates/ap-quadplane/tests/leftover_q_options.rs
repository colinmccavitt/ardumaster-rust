//! Leftover Q_OPTIONS bits — leftover_option_is_set on QuadPlane.

use ap_quadplane::quadplane_completeness::{
    LeftoverFailsafeMode, LeftoverQOption, ARMING_DELAY_MS,
};
use ap_quadplane::vtol_mode::{
    MAV_CMD_NAV_LAND, MAV_CMD_NAV_TAKEOFF, MAV_CMD_NAV_VTOL_LAND, MAV_CMD_NAV_VTOL_TAKEOFF,
    MAV_CMD_NAV_WAYPOINT,
};
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

#[test]
fn leftover_option_is_set_reads_quadplane_options() {
    let mut qp = QuadPlane::new();
    assert_eq!(qp.options(), 0);
    assert!(!qp.leftover_option_is_set(LeftoverQOption::LevelTransition));
    assert!(!qp.leftover_option_is_set(LeftoverQOption::FsQrtl));
    assert!(!qp.leftover_option_is_set(LeftoverQOption::DelayArming));
    assert!(!qp.leftover_option_is_set(LeftoverQOption::ThrLandingControl));
    assert!(!qp.leftover_option_is_set(LeftoverQOption::FsRtl));

    qp.set_options(
        LeftoverQOption::LevelTransition.as_i32()
            | LeftoverQOption::FsQrtl.as_i32()
            | LeftoverQOption::DelayArming.as_i32()
            | LeftoverQOption::ThrLandingControl.as_i32(),
    );
    assert!(qp.leftover_option_is_set(LeftoverQOption::LevelTransition));
    assert!(qp.leftover_option_is_set(LeftoverQOption::FsQrtl));
    assert!(qp.leftover_option_is_set(LeftoverQOption::DelayArming));
    assert!(qp.leftover_option_is_set(LeftoverQOption::ThrLandingControl));
    assert!(!qp.leftover_option_is_set(LeftoverQOption::FsRtl));
    assert!(!qp.leftover_option_is_set(LeftoverQOption::AllowFwTakeoff));
}

#[test]
fn leftover_level_transition_helpers() {
    let mut qp = available_qp();
    assert!(!qp.leftover_level_transition());
    assert!(!qp.leftover_level_transition_limits_climb(false));
    assert!(!qp.leftover_level_transition_limits_roll(true, true));

    qp.set_options(LeftoverQOption::LevelTransition.as_i32());
    assert!(qp.leftover_level_transition());
    assert!(qp.leftover_level_transition_limits_climb(false));
    assert!(!qp.leftover_level_transition_limits_climb(true));
    assert!(qp.leftover_level_transition_limits_roll(true, true));
    assert!(!qp.leftover_level_transition_limits_roll(false, true));
    assert!(!qp.leftover_level_transition_limits_roll(true, false));
}

#[test]
fn leftover_allow_fw_takeoff_and_land_gate_is_vtol() {
    let mut qp = available_qp();
    assert!(!qp.leftover_allow_fw_takeoff());
    assert!(!qp.leftover_allow_fw_land());
    assert!(qp.is_vtol_takeoff(MAV_CMD_NAV_VTOL_TAKEOFF));
    assert!(qp.is_vtol_takeoff(MAV_CMD_NAV_TAKEOFF));
    assert!(qp.is_vtol_land(MAV_CMD_NAV_VTOL_LAND));
    assert!(qp.is_vtol_land(MAV_CMD_NAV_LAND));
    assert!(!qp.is_vtol_takeoff(MAV_CMD_NAV_WAYPOINT));

    qp.set_options(
        LeftoverQOption::AllowFwTakeoff.as_i32() | LeftoverQOption::AllowFwLand.as_i32(),
    );
    assert!(qp.leftover_allow_fw_takeoff());
    assert!(qp.leftover_allow_fw_land());
    assert!(qp.is_vtol_takeoff(MAV_CMD_NAV_VTOL_TAKEOFF));
    assert!(!qp.is_vtol_takeoff(MAV_CMD_NAV_TAKEOFF));
    assert!(qp.is_vtol_land(MAV_CMD_NAV_VTOL_LAND));
    assert!(!qp.is_vtol_land(MAV_CMD_NAV_LAND));
}

#[test]
fn leftover_delay_arming_and_thr_landing_and_failsafe() {
    let mut qp = available_qp();
    assert_eq!(ARMING_DELAY_MS, 2000);
    assert!(!qp.leftover_motors_delay_arming(true));
    assert!(!qp.leftover_thr_landing_control());
    assert_eq!(qp.leftover_q_failsafe_mode(), LeftoverFailsafeMode::Qland);

    qp.set_options(LeftoverQOption::DelayArming.as_i32());
    assert!(qp.leftover_motors_delay_arming(true));
    assert!(!qp.leftover_motors_delay_arming(false));

    qp.set_options(LeftoverQOption::DisarmedTilt.as_i32());
    assert!(qp.leftover_motors_delay_arming(true));

    qp.set_options(LeftoverQOption::ThrLandingControl.as_i32());
    assert!(qp.leftover_thr_landing_control());

    qp.set_options(LeftoverQOption::FsQrtl.as_i32());
    assert_eq!(qp.leftover_q_failsafe_mode(), LeftoverFailsafeMode::Qrtl);

    qp.set_options(LeftoverQOption::FsRtl.as_i32());
    assert_eq!(qp.leftover_q_failsafe_mode(), LeftoverFailsafeMode::Rtl);

    qp.set_options(LeftoverQOption::FsRtl.as_i32() | LeftoverQOption::FsQrtl.as_i32());
    assert_eq!(qp.leftover_q_failsafe_mode(), LeftoverFailsafeMode::Rtl);
}

#[test]
fn leftover_helpers_do_not_rewrite_setup_or_logging() {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    assert!(qp.available());
    assert_eq!(qp.logging().qtun_writes(), 0);
    assert!(!qp.leftover_option_is_set(LeftoverQOption::FsQrtl));
}
