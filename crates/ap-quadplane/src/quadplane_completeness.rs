//! VT-001 QuadPlane completeness: surfaces already on main vs leftover.
//!
//! Catalogs the `ArduPlane/quadplane.cpp` / `.h` port. Items marked
//! [`PortStatus::OnMain`] landed in earlier VT-001 slices and must not
//! be redone. [`PortStatus::ThisSlice`] is leftover land-sequence
//! predicates (`in_vtol_land_approach` / descent / final / sequence /
//! poscontrol / airbrake). [`PortStatus::Remaining`] are leftover
//! `quadplane.cpp` / `.h` surfaces not yet stubbed (motors/hold,
//! guided/QRTL, thrust-loss, TECS leftovers).
//!
//! This module does not rewrite [`crate::air_mode`], [`crate::auto_vtol`],
//! [`crate::landing`], [`crate::logging`],
//! [`crate::mode_q`], [`crate::motor_test`], [`crate::poscontrol`],
//! [`crate::tailsitter`], [`crate::throttle`], [`crate::transition`],
//! [`crate::transition_fsm`], [`crate::vtol_mode`], or
//! [`crate::weathervane`].

use crate::poscontrol::PositionControlState;

/// Whether a catalog row is already hooked up or left for later work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortStatus {
    /// Present on `main` before this slice.
    OnMain,
    /// Added by this VT-001 slice (this table + leftover helpers).
    ThisSlice,
    /// Leftover `quadplane.cpp` / `.h` surface, not yet stubbed.
    Remaining,
}

/// One QuadPlane surface in the completeness table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QuadPlanePortItem {
    /// Surface name.
    pub name: &'static str,
    /// Hooked up on main / this slice, or remaining.
    pub status: PortStatus,
    /// Short note (upstream symbol or why remaining).
    pub note: &'static str,
}

/// Completeness table: ported QuadPlane stubs vs leftover `quadplane.cpp` / `.h`.
///
/// On-main rows match the VT-001 slices already landed. Remaining rows
/// are leftover behavioral surfaces — this ticket is not table-only.
pub const QUADPLANE_COMPLETENESS: &[QuadPlanePortItem] = &[
    QuadPlanePortItem {
        name: "setup / Q_FRAME_CLASS",
        status: PortStatus::OnMain,
        note: "lib.rs QuadPlane::setup / classify_frame / available / enabled",
    },
    QuadPlanePortItem {
        name: "mode_enter / poscontrol",
        status: PortStatus::OnMain,
        note: "poscontrol.rs mode_enter / init_throttle_wait / QPOS_*",
    },
    QuadPlanePortItem {
        name: "vtol_mode",
        status: PortStatus::OnMain,
        note: "vtol_mode.rs in_vtol_mode / in_vtol_auto",
    },
    QuadPlanePortItem {
        name: "air_mode",
        status: PortStatus::OnMain,
        note: "air_mode.rs air_mode_active / update / option_is_set (bits 9, 13)",
    },
    QuadPlanePortItem {
        name: "weathervane",
        status: PortStatus::OnMain,
        note: "weathervane.rs get_weathervane_yaw_rate_cds / assist-handoff",
    },
    QuadPlanePortItem {
        name: "throttle mix / tilt-wait",
        status: PortStatus::OnMain,
        note: "throttle.rs update_throttle_mix / tilt_fwd_complete",
    },
    QuadPlanePortItem {
        name: "motor_test",
        status: PortStatus::OnMain,
        note: "motor_test.rs mavlink_motor_test_start / output / stop",
    },
    QuadPlanePortItem {
        name: "landing-detect / do_user_takeoff",
        status: PortStatus::OnMain,
        note: "landing.rs should_relax / land_detector / check_land_* / do_user_takeoff",
    },
    QuadPlanePortItem {
        name: "completeness table",
        status: PortStatus::OnMain,
        note: "this catalog + leftover API contract helpers",
    },
    QuadPlanePortItem {
        name: "leftover Q_OPTIONS bits",
        status: PortStatus::OnMain,
        note: "leftover_option_is_set / LEVEL_TRANSITION / ALLOW_FW_* / FS_QRTL / FS_RTL / DELAY_ARMING / THR_LANDING_CONTROL",
    },
    QuadPlanePortItem {
        name: "assisted-flight latch extras",
        status: PortStatus::OnMain,
        note: "force_fw_control_recovery / in_spin_recovery QTUN bits / leftover_show_vtol_view / leftover_use_multicopter_control",
    },
    QuadPlanePortItem {
        name: "logging",
        status: PortStatus::OnMain,
        note: "logging.rs Log_Write_QControl_Tuning / log_QPOS / Log_Write_AttRate",
    },
    QuadPlanePortItem {
        name: "position / takeoff / waypoint controllers",
        status: PortStatus::OnMain,
        note: "position_controller.rs vtol_position_controller / takeoff_controller / waypoint_controller",
    },
    QuadPlanePortItem {
        name: "land-sequence predicates",
        status: PortStatus::ThisSlice,
        note: "land_sequence.rs in_vtol_land_approach / descent / final / sequence / poscontrol / airbrake",
    },
    QuadPlanePortItem {
        name: "AUTO mission VTOL",
        status: PortStatus::OnMain,
        note: "auto_vtol.rs do_vtol_takeoff / do_vtol_land / verify_* / control_auto",
    },
    QuadPlanePortItem {
        name: "motors_output / hold / set_armed",
        status: PortStatus::Remaining,
        note: "motors_output / hold_hover / hold_stabilize / set_armed (not stubbed)",
    },
    QuadPlanePortItem {
        name: "guided / QRTL / RTL_MODE",
        status: PortStatus::Remaining,
        note: "guided_start / guided_update / RTL_MODE NONE..QRTL_ALWAYS (not stubbed)",
    },
    QuadPlanePortItem {
        name: "thrust-loss / ESC-cal / takeoff-failure",
        status: PortStatus::Remaining,
        note: "thrust_loss_check / run_esc_calibration / takeoff_failure_scalar (not stubbed)",
    },
    QuadPlanePortItem {
        name: "TECS / stick-mix / stopping-distance leftovers",
        status: PortStatus::Remaining,
        note: "should_disable_TECS / allow_stick_mixing / stopping_distance_m (not stubbed)",
    },
];

/// Leftover `Q_OPTIONS` bits not yet in [`crate::air_mode::QOption`].
///
/// Bits 9 and 13 (`AIRMODE_UNUSED`, `DISABLE_GROUND_EFFECT_COMP`) are
/// on main. Bit 14 lives on mode_q, bit 18 on tailsitter, bit 19 on
/// the SLT FSM — still not this leftover table's job to rewrite.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(i32)]
pub enum LeftoverQOption {
    /// Bit 0, upstream `Option::LEVEL_TRANSITION`.
    LevelTransition = 1 << 0,
    /// Bit 1, upstream `Option::ALLOW_FW_TAKEOFF`.
    AllowFwTakeoff = 1 << 1,
    /// Bit 2, upstream `Option::ALLOW_FW_LAND`.
    AllowFwLand = 1 << 2,
    /// Bit 3, upstream `Option::RESPECT_TAKEOFF_FRAME`.
    RespectTakeoffFrame = 1 << 3,
    /// Bit 4, upstream `Option::MISSION_LAND_FW_APPROACH`.
    MissionLandFwApproach = 1 << 4,
    /// Bit 5, upstream `Option::FS_QRTL`.
    FsQrtl = 1 << 5,
    /// Bit 6, upstream `Option::IDLE_GOV_MANUAL`.
    IdleGovManual = 1 << 6,
    /// Bit 8, upstream `Option::TAILSIT_Q_ASSIST_MOTORS_ONLY`.
    TailsitQAssistMotorsOnly = 1 << 8,
    /// Bit 10, upstream `Option::DISARMED_TILT`.
    DisarmedTilt = 1 << 10,
    /// Bit 11, upstream `Option::DELAY_ARMING`.
    DelayArming = 1 << 11,
    /// Bit 15, upstream `Option::THR_LANDING_CONTROL`.
    ThrLandingControl = 1 << 15,
    /// Bit 16, upstream `Option::DISABLE_APPROACH`.
    DisableApproach = 1 << 16,
    /// Bit 17, upstream `Option::REPOSITION_LANDING`.
    RepositionLanding = 1 << 17,
    /// Bit 20, upstream `Option::FS_RTL`.
    FsRtl = 1 << 20,
    /// Bit 21, upstream `Option::DISARMED_TILT_UP`.
    DisarmedTiltUp = 1 << 21,
    /// Bit 22, upstream `Option::SCALE_FF_ANGLE_P`.
    ScaleFfAngleP = 1 << 22,
}

impl LeftoverQOption {
    /// Upstream discriminant.
    #[must_use]
    pub const fn as_i32(self) -> i32 {
        self as i32
    }
}

/// Upstream `option_is_set` for a leftover bit.
#[must_use]
pub const fn leftover_option_is_set(options: i32, option: LeftoverQOption) -> bool {
    (options & option.as_i32()) != 0
}

/// Leftover `Q_OPTIONS` FS_RTL / FS_QRTL pick after a Q-mode RC failsafe.
///
/// Upstream `events.cpp` `rc_failsafe_*_on_event`: `FS_RTL` wins over
/// `FS_QRTL`; neither bit falls through to QLAND.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LeftoverFailsafeMode {
    /// Neither bit — QLAND.
    Qland,
    /// `FS_QRTL` without `FS_RTL`.
    Qrtl,
    /// `FS_RTL` (wins when both bits are set).
    Rtl,
}

use crate::QuadPlane;

impl QuadPlane {
    /// Upstream `QuadPlane::option_is_set` for leftover `Q_OPTIONS` bits.
    ///
    /// Bits 9 and 13 stay on [`crate::air_mode::QOption`]. This is the
    /// leftover table: LEVEL_TRANSITION, ALLOW_FW_*, FS_QRTL / FS_RTL,
    /// DELAY_ARMING, THR_LANDING_CONTROL, and the rest of
    /// [`LeftoverQOption`].
    #[must_use]
    pub const fn leftover_option_is_set(&self, option: LeftoverQOption) -> bool {
        leftover_option_is_set(self.options, option)
    }

    /// `Q_OPTIONS` `LEVEL_TRANSITION`.
    #[must_use]
    pub const fn leftover_level_transition(&self) -> bool {
        self.leftover_option_is_set(LeftoverQOption::LevelTransition)
    }

    /// SLT airspeed-wait climb clamp: `LEVEL_TRANSITION` and not tiltrotor.
    ///
    /// Upstream `MIN(assist_climb_rate_cms(), 0)` in
    /// `SLT_Transition::update`. `hold_hover` is a later leftover row.
    #[must_use]
    pub const fn leftover_level_transition_limits_climb(&self, tiltrotor_enabled: bool) -> bool {
        self.leftover_level_transition() && !tiltrotor_enabled
    }

    /// `SLT_Transition::set_FW_roll_limit` — assist + AIRSPEED_WAIT/TIMER
    /// + `LEVEL_TRANSITION`.
    #[must_use]
    pub const fn leftover_level_transition_limits_roll(
        &self,
        assisted_flight: bool,
        in_airspeed_or_timer: bool,
    ) -> bool {
        assisted_flight && in_airspeed_or_timer && self.leftover_level_transition()
    }

    /// `Q_OPTIONS` `ALLOW_FW_TAKEOFF`.
    #[must_use]
    pub const fn leftover_allow_fw_takeoff(&self) -> bool {
        self.leftover_option_is_set(LeftoverQOption::AllowFwTakeoff)
    }

    /// `Q_OPTIONS` `ALLOW_FW_LAND`.
    #[must_use]
    pub const fn leftover_allow_fw_land(&self) -> bool {
        self.leftover_option_is_set(LeftoverQOption::AllowFwLand)
    }

    /// `Q_OPTIONS` `THR_LANDING_CONTROL`.
    ///
    /// `landing_descent_rate_ms` throttle scaling is a later leftover.
    #[must_use]
    pub const fn leftover_thr_landing_control(&self) -> bool {
        self.leftover_option_is_set(LeftoverQOption::ThrLandingControl)
    }

    /// `motors_output` arming-delay gate.
    ///
    /// `DELAY_ARMING` or `DISARMED_TILT` plus `arming.get_delay_arming()`
    /// keeps the spool at `SHUT_DOWN`. `motors_output` itself is a later
    /// leftover row.
    #[must_use]
    pub const fn leftover_motors_delay_arming(&self, arming_delay_active: bool) -> bool {
        arming_delay_active
            && (self.leftover_option_is_set(LeftoverQOption::DelayArming)
                || self.leftover_option_is_set(LeftoverQOption::DisarmedTilt))
    }

    /// Leftover `Q_OPTIONS` FS_RTL / FS_QRTL pick.
    ///
    /// `FS_RTL` wins over `FS_QRTL`.
    #[must_use]
    pub const fn leftover_q_failsafe_mode(&self) -> LeftoverFailsafeMode {
        if self.leftover_option_is_set(LeftoverQOption::FsRtl) {
            LeftoverFailsafeMode::Rtl
        } else if self.leftover_option_is_set(LeftoverQOption::FsQrtl) {
            LeftoverFailsafeMode::Qrtl
        } else {
            LeftoverFailsafeMode::Qland
        }
    }

    /// Upstream `bool force_fw_control_recovery`.
    #[must_use]
    pub const fn leftover_force_fw_control_recovery(&self) -> bool {
        self.force_fw_control_recovery
    }

    /// Upstream `bool in_spin_recovery`.
    #[must_use]
    pub const fn leftover_in_spin_recovery(&self) -> bool {
        self.in_spin_recovery
    }

    /// Latch leftover `force_fw_control_recovery`.
    pub fn leftover_set_force_fw_control_recovery(&mut self, force_fw_control_recovery: bool) {
        self.force_fw_control_recovery = force_fw_control_recovery;
    }

    /// Latch leftover `in_spin_recovery`.
    pub fn leftover_set_in_spin_recovery(&mut self, in_spin_recovery: bool) {
        self.in_spin_recovery = in_spin_recovery;
    }

    /// `mode_enter` leftover: both recovery latches clear.
    ///
    /// Upstream `QuadPlane::mode_enter` writes
    /// `force_fw_control_recovery = false; in_spin_recovery = false`.
    /// [`QuadPlane::mode_enter`] itself stays on the poscontrol slice.
    pub fn leftover_clear_recovery_latches(&mut self) {
        self.force_fw_control_recovery = false;
        self.in_spin_recovery = false;
    }

    /// QTUN `assist` bits 5/6 from the leftover latches.
    ///
    /// Upstream `Log_Write_QControl_Tuning` ORs `fw_force` /
    /// `spin_recovery` from these members. The logging slice still
    /// packs the same bits from [`crate::logging::QTunView`].
    #[must_use]
    pub const fn leftover_qtun_assist_latch_flags(&self) -> u8 {
        leftover_qtun_assist_latch_flags(self.force_fw_control_recovery, self.in_spin_recovery)
    }

    /// Upstream `QuadPlane::show_vtol_view` leftover recovery gate.
    ///
    /// `available() && transition->show_vtol_view() &&
    /// !force_fw_control_recovery`. The transition half is a later
    /// leftover; pass it in.
    #[must_use]
    pub const fn leftover_show_vtol_view(&self, transition_show: bool) -> bool {
        self.available() && transition_show && !self.force_fw_control_recovery
    }

    /// `multicopter_attitude_rate_update` leftover recovery gate.
    ///
    /// Upstream `in_vtol_mode() && !tailsitter.in_vtol_transition() &&
    /// !force_fw_control_recovery`.
    #[must_use]
    pub const fn leftover_use_multicopter_control(
        &self,
        in_vtol_mode: bool,
        tailsitter_in_vtol_transition: bool,
    ) -> bool {
        in_vtol_mode && !tailsitter_in_vtol_transition && !self.force_fw_control_recovery
    }
}

/// Pack leftover QTUN `fw_force` / `spin_recovery` bits.
#[must_use]
pub const fn leftover_qtun_assist_latch_flags(force_fw: bool, spin_recovery: bool) -> u8 {
    qtun_assist_flags(false, false, false, false, false, force_fw, spin_recovery)
}

/// `Q_RTL_MODE` / `QuadPlane::RTL_MODE`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(i8)]
pub enum RtlMode {
    /// `RTL_MODE::NONE`.
    None = 0,
    /// `RTL_MODE::SWITCH_QRTL`.
    SwitchQrtl = 1,
    /// `RTL_MODE::VTOL_APPROACH_QRTL`.
    VtolApproachQrtl = 2,
    /// `RTL_MODE::QRTL_ALWAYS`.
    QrtlAlways = 3,
}

impl RtlMode {
    /// Inverse of the upstream discriminant.
    #[must_use]
    pub const fn from_i8(value: i8) -> Option<Self> {
        match value {
            0 => Some(Self::None),
            1 => Some(Self::SwitchQrtl),
            2 => Some(Self::VtolApproachQrtl),
            3 => Some(Self::QrtlAlways),
            _ => None,
        }
    }

    /// Upstream discriminant.
    #[must_use]
    pub const fn as_i8(self) -> i8 {
        self as i8
    }
}

/// QTUN write period, upstream `now - last_qtun_log_ms > 40`.
pub const QTUN_PERIOD_MS: u32 = 40;

/// Arming spool delay, upstream `AP_ARMING_DELAY_MS` (used when
/// `DELAY_ARMING` or `DISARMED_TILT` is set).
pub const ARMING_DELAY_MS: u32 = 2000;

/// Leftover logger message names from `quadplane.cpp`.
pub const LOG_MESSAGES: &[&str] = &["QTUN", "QPOS", "QBRK", "FWDT"];

/// `log_QPOS` field names, upstream `WriteStreaming("QPOS", ...)`.
pub const QPOS_FIELDS: &[&str] = &["TimeUS", "State", "Dist", "TSpd", "TAcc", "OShoot"];

/// `log_QControl_Tuning` / `LOG_QTUN_MSG` field names (after TimeUS).
pub const QTUN_FIELDS: &[&str] = &[
    "throttle_in",
    "angle_boost",
    "throttle_out",
    "throttle_hover",
    "desired_alt",
    "inav_alt",
    "baro_alt",
    "target_climb_rate",
    "climb_rate",
    "throttle_mix",
    "transition_state",
    "assist",
];

/// QTUN `assist` bit 0, upstream `log_assistance_flags::in_assisted_flight`.
pub const QTUN_ASSIST_IN_ASSISTED_FLIGHT: u8 = 1 << 0;
/// QTUN `assist` bit 1, upstream `log_assistance_flags::forced`.
pub const QTUN_ASSIST_FORCED: u8 = 1 << 1;
/// QTUN `assist` bit 2, upstream `log_assistance_flags::speed`.
pub const QTUN_ASSIST_SPEED: u8 = 1 << 2;
/// QTUN `assist` bit 3, upstream `log_assistance_flags::alt`.
pub const QTUN_ASSIST_ALT: u8 = 1 << 3;
/// QTUN `assist` bit 4, upstream `log_assistance_flags::angle`.
pub const QTUN_ASSIST_ANGLE: u8 = 1 << 4;
/// QTUN `assist` bit 5, leftover `force_fw_control_recovery`.
pub const QTUN_ASSIST_FW_FORCE: u8 = 1 << 5;
/// QTUN `assist` bit 6, leftover `in_spin_recovery`.
pub const QTUN_ASSIST_SPIN_RECOVERY: u8 = 1 << 6;

/// Pack leftover QTUN `assist` flags. Upstream `Log_Write_QControl_Tuning`.
#[must_use]
pub const fn qtun_assist_flags(
    assisted_flight: bool,
    force_assist: bool,
    speed_assist: bool,
    alt_assist: bool,
    angle_assist: bool,
    fw_force_recovery: bool,
    spin_recovery: bool,
) -> u8 {
    let mut flags = 0u8;
    if assisted_flight {
        flags |= QTUN_ASSIST_IN_ASSISTED_FLIGHT;
    }
    if force_assist {
        flags |= QTUN_ASSIST_FORCED;
    }
    if speed_assist {
        flags |= QTUN_ASSIST_SPEED;
    }
    if alt_assist {
        flags |= QTUN_ASSIST_ALT;
    }
    if angle_assist {
        flags |= QTUN_ASSIST_ANGLE;
    }
    if fw_force_recovery {
        flags |= QTUN_ASSIST_FW_FORCE;
    }
    if spin_recovery {
        flags |= QTUN_ASSIST_SPIN_RECOVERY;
    }
    flags
}

/// `in_vtol_land_descent` poscontrol states.
///
/// Upstream `QPOS_LAND_DESCEND` / `QPOS_LAND_FINAL` / `QPOS_LAND_ABORT`.
#[must_use]
pub const fn land_descent_state(state: PositionControlState) -> bool {
    matches!(
        state,
        PositionControlState::LandDescend
            | PositionControlState::LandFinal
            | PositionControlState::LandAbort
    )
}

/// `in_vtol_land_final` — descent and `QPOS_LAND_FINAL`.
#[must_use]
pub const fn land_final_state(in_descent: bool, state: PositionControlState) -> bool {
    in_descent && matches!(state, PositionControlState::LandFinal)
}

/// AUTO `in_vtol_land_approach` poscontrol states.
///
/// Upstream `QPOS_APPROACH` / `AIRBRAKE` / `POSITION1` / `POSITION2`.
#[must_use]
pub const fn land_approach_state(state: PositionControlState) -> bool {
    matches!(
        state,
        PositionControlState::Approach
            | PositionControlState::Airbrake
            | PositionControlState::Position1
            | PositionControlState::Position2
    )
}

/// QRTL approach: `poscontrol.get_state() <= QPOS_POSITION2`.
#[must_use]
pub const fn qrtl_approach_state(state: PositionControlState) -> bool {
    (state as u8) <= PositionControlState::Position2 as u8
}

/// `in_vtol_airbrake` — `QPOS_AIRBRAKE`.
#[must_use]
pub const fn airbrake_state(state: PositionControlState) -> bool {
    matches!(state, PositionControlState::Airbrake)
}

/// `in_vtol_land_poscontrol` — `poscontrol.get_state() >= QPOS_POSITION1`.
#[must_use]
pub const fn land_poscontrol_state(state: PositionControlState) -> bool {
    (state as u8) >= PositionControlState::Position1 as u8
}

/// `in_vtol_land_sequence` OR. Upstream
/// `qrtl || in_vtol_land_approach() || in_vtol_land_descent() ||
/// in_vtol_land_final()`.
#[must_use]
pub const fn land_sequence(qrtl: bool, approach: bool, descent: bool, land_final: bool) -> bool {
    qrtl || approach || descent || land_final
}

/// Rows already hooked up on `main` (must not be redone).
#[must_use]
pub fn on_main_items() -> impl Iterator<Item = &'static QuadPlanePortItem> {
    QUADPLANE_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::OnMain)
}

/// Rows added by this slice.
#[must_use]
pub fn this_slice_items() -> impl Iterator<Item = &'static QuadPlanePortItem> {
    QUADPLANE_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::ThisSlice)
}

/// Leftover `quadplane.cpp` / `.h` surfaces not yet stubbed.
#[must_use]
pub fn remaining_items() -> impl Iterator<Item = &'static QuadPlanePortItem> {
    QUADPLANE_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::Remaining)
}

/// Count rows in each status bucket.
#[must_use]
pub fn completeness_counts() -> (usize, usize, usize) {
    let mut on_main = 0;
    let mut this_slice = 0;
    let mut remaining = 0;
    for item in QUADPLANE_COMPLETENESS {
        match item.status {
            PortStatus::OnMain => on_main += 1,
            PortStatus::ThisSlice => this_slice += 1,
            PortStatus::Remaining => remaining += 1,
        }
    }
    (on_main, this_slice, remaining)
}

/// True when `name` is listed with `status`.
#[must_use]
pub fn completeness_has(name: &str, status: PortStatus) -> bool {
    QUADPLANE_COMPLETENESS
        .iter()
        .any(|item| item.name == name && item.status == status)
}

/// True when every name in the table appears once.
#[must_use]
pub fn completeness_unique_names() -> bool {
    for (i, item) in QUADPLANE_COMPLETENESS.iter().enumerate() {
        for other in QUADPLANE_COMPLETENESS.iter().skip(i + 1) {
            if item.name == other.name {
                return false;
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_covers_main_surfaces_and_leftover_api() {
        assert!(completeness_unique_names());
        let (on_main, this_slice, remaining) = completeness_counts();
        assert_eq!(on_main, 14);
        assert_eq!(this_slice, 1);
        assert_eq!(remaining, 4);
        assert!(completeness_has(
            "setup / Q_FRAME_CLASS",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "landing-detect / do_user_takeoff",
            PortStatus::OnMain
        ));
        assert!(completeness_has("completeness table", PortStatus::OnMain));
        assert!(completeness_has(
            "leftover Q_OPTIONS bits",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "assisted-flight latch extras",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "position / takeoff / waypoint controllers",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "land-sequence predicates",
            PortStatus::ThisSlice
        ));
        assert!(completeness_has("logging", PortStatus::OnMain));
        assert!(completeness_has("AUTO mission VTOL", PortStatus::OnMain));
        assert_eq!(on_main_items().count(), 14);
        assert_eq!(this_slice_items().count(), 1);
        assert_eq!(remaining_items().count(), 4);
    }

    #[test]
    fn remaining_does_not_repeat_hooked_surfaces() {
        for item in remaining_items() {
            assert!(
                !completeness_has(item.name, PortStatus::OnMain),
                "{} listed remaining but already on main",
                item.name
            );
            assert!(
                !completeness_has(item.name, PortStatus::ThisSlice),
                "{} listed remaining but added this slice",
                item.name
            );
        }
    }

    #[test]
    fn leftover_q_options_bits_match_upstream() {
        assert_eq!(LeftoverQOption::LevelTransition.as_i32(), 1 << 0);
        assert_eq!(LeftoverQOption::AllowFwTakeoff.as_i32(), 1 << 1);
        assert_eq!(LeftoverQOption::AllowFwLand.as_i32(), 1 << 2);
        assert_eq!(LeftoverQOption::FsQrtl.as_i32(), 1 << 5);
        assert_eq!(LeftoverQOption::DelayArming.as_i32(), 1 << 11);
        assert_eq!(LeftoverQOption::ThrLandingControl.as_i32(), 1 << 15);
        assert_eq!(LeftoverQOption::DisableApproach.as_i32(), 1 << 16);
        assert_eq!(LeftoverQOption::FsRtl.as_i32(), 1 << 20);
        assert!(!leftover_option_is_set(0, LeftoverQOption::FsQrtl));
        assert!(leftover_option_is_set(
            LeftoverQOption::FsQrtl.as_i32(),
            LeftoverQOption::FsQrtl
        ));
        assert!(!leftover_option_is_set(
            LeftoverQOption::FsQrtl.as_i32(),
            LeftoverQOption::FsRtl
        ));
    }

    #[test]
    fn leftover_rtl_mode_and_arming_delay() {
        assert_eq!(RtlMode::from_i8(0), Some(RtlMode::None));
        assert_eq!(RtlMode::from_i8(1), Some(RtlMode::SwitchQrtl));
        assert_eq!(RtlMode::from_i8(2), Some(RtlMode::VtolApproachQrtl));
        assert_eq!(RtlMode::from_i8(3), Some(RtlMode::QrtlAlways));
        assert_eq!(RtlMode::from_i8(4), None);
        assert_eq!(RtlMode::QrtlAlways.as_i8(), 3);
        assert_eq!(ARMING_DELAY_MS, 2000);
        assert_eq!(QTUN_PERIOD_MS, 40);
    }

    #[test]
    fn leftover_qtun_assist_flags_and_log_names() {
        assert_eq!(
            qtun_assist_flags(false, false, false, false, false, false, false),
            0
        );
        assert_eq!(
            qtun_assist_flags(true, false, false, false, false, false, false),
            QTUN_ASSIST_IN_ASSISTED_FLIGHT
        );
        assert_eq!(
            qtun_assist_flags(true, false, false, false, false, true, true),
            QTUN_ASSIST_IN_ASSISTED_FLIGHT | QTUN_ASSIST_FW_FORCE | QTUN_ASSIST_SPIN_RECOVERY
        );
        assert_eq!(LOG_MESSAGES, &["QTUN", "QPOS", "QBRK", "FWDT"]);
        assert_eq!(
            QPOS_FIELDS,
            &["TimeUS", "State", "Dist", "TSpd", "TAcc", "OShoot"]
        );
        assert_eq!(QTUN_FIELDS.len(), 12);
    }

    #[test]
    fn leftover_assist_latch_extras_pack_and_gate() {
        let mut qp = crate::QuadPlane::with_enable(1);
        assert!(qp.setup());
        assert!(!qp.leftover_force_fw_control_recovery());
        assert!(!qp.leftover_in_spin_recovery());
        assert_eq!(qp.leftover_qtun_assist_latch_flags(), 0);
        assert_eq!(leftover_qtun_assist_latch_flags(false, false), 0);
        assert_eq!(
            leftover_qtun_assist_latch_flags(true, true),
            QTUN_ASSIST_FW_FORCE | QTUN_ASSIST_SPIN_RECOVERY
        );
        qp.leftover_set_force_fw_control_recovery(true);
        qp.leftover_set_in_spin_recovery(true);
        assert!(qp.leftover_force_fw_control_recovery());
        assert!(qp.leftover_in_spin_recovery());
        assert_eq!(
            qp.leftover_qtun_assist_latch_flags(),
            QTUN_ASSIST_FW_FORCE | QTUN_ASSIST_SPIN_RECOVERY
        );
        assert!(qp.leftover_show_vtol_view(true) == false);
        assert!(qp.leftover_use_multicopter_control(true, false) == false);
        qp.leftover_clear_recovery_latches();
        assert!(!qp.leftover_force_fw_control_recovery());
        assert!(!qp.leftover_in_spin_recovery());
        assert!(qp.leftover_show_vtol_view(true));
        assert!(qp.leftover_use_multicopter_control(true, false));
        assert!(!qp.leftover_use_multicopter_control(true, true));
        assert!(!qp.leftover_show_vtol_view(false));
    }

    #[test]
    fn leftover_land_sequence_predicates() {
        assert!(land_descent_state(PositionControlState::LandDescend));
        assert!(land_descent_state(PositionControlState::LandFinal));
        assert!(land_descent_state(PositionControlState::LandAbort));
        assert!(!land_descent_state(PositionControlState::Position2));
        assert!(land_final_state(true, PositionControlState::LandFinal));
        assert!(!land_final_state(false, PositionControlState::LandFinal));
        assert!(land_approach_state(PositionControlState::Approach));
        assert!(land_approach_state(PositionControlState::Position2));
        assert!(!land_approach_state(PositionControlState::LandDescend));
        assert!(qrtl_approach_state(PositionControlState::None));
        assert!(qrtl_approach_state(PositionControlState::Position2));
        assert!(!qrtl_approach_state(PositionControlState::LandDescend));
        assert!(airbrake_state(PositionControlState::Airbrake));
        assert!(land_poscontrol_state(PositionControlState::Position1));
        assert!(!land_poscontrol_state(PositionControlState::Airbrake));
        assert!(land_sequence(true, false, false, false));
        assert!(land_sequence(false, true, false, false));
        assert!(!land_sequence(false, false, false, false));
    }
}
