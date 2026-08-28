//! VT-001 QuadPlane completeness: surfaces already on main vs leftover.
//!
//! Catalogs the `ArduPlane/quadplane.cpp` / `.h` port. Items marked
//! [`PortStatus::OnMain`] landed in earlier VT-001 slices and must not
//! be redone. [`PortStatus::ThisSlice`] is leftover thrust_loss_check /
//! run_esc_calibration / takeoff_failure_scalar. [`PortStatus::Remaining`]
//! are leftover `quadplane.cpp` / `.h` surfaces not yet stubbed
//! (TECS leftovers).
//!
//! This module does not rewrite [`crate::air_mode`], [`crate::auto_vtol`],
//! [`crate::landing`], [`crate::land_sequence`], [`crate::logging`],
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
        status: PortStatus::OnMain,
        note: "land_sequence.rs in_vtol_land_approach / descent / final / sequence / poscontrol / airbrake",
    },
    QuadPlanePortItem {
        name: "AUTO mission VTOL",
        status: PortStatus::OnMain,
        note: "auto_vtol.rs do_vtol_takeoff / do_vtol_land / verify_* / control_auto",
    },
    QuadPlanePortItem {
        name: "motors_output / hold / set_armed",
        status: PortStatus::OnMain,
        note: "motors_output.rs motors_output / hold_hover / hold_stabilize / set_armed",
    },
    QuadPlanePortItem {
        name: "guided / QRTL / RTL_MODE",
        status: PortStatus::OnMain,
        note: "guided.rs guided_start / guided_update / RTL_MODE NONE..QRTL_ALWAYS / guided_mode_enabled",
    },
    QuadPlanePortItem {
        name: "thrust-loss / ESC-cal / takeoff-failure",
        status: PortStatus::ThisSlice,
        note: "thrust_loss.rs thrust_loss_check / run_esc_calibration / takeoff_failure_scalar",
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
    /// `SLT_Transition::update`. `hold_hover` is stubbed on
    /// [`crate::motors_output`].
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
    /// keeps the spool at `SHUT_DOWN`. Used by [`crate::motors_output`].
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

/// `motors_output` thrust-loss inactive window, ms.
pub const MOTORS_INACTIVE_MS: u32 = 100;

/// Rate-controller relax window, ms (`now - last_att_control_ms > 100`).
pub const ATT_CONTROL_RELAX_MS: u32 = 100;

/// `motors->get_throttle()` floor that counts as active.
pub const MOTORS_ACTIVE_THROTTLE: f32 = 0.01;

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

/// `hold_stabilize` ground-idle gate: `throttle_in <= 0 && !air_mode`.
#[must_use]
pub const fn hold_stabilize_ground_idle(throttle_in: f32, air_mode_active: bool) -> bool {
    throttle_in <= 0.0 && !air_mode_active
}

/// Angle-boost in `hold_stabilize`: off for tailsitter + assist.
#[must_use]
pub const fn hold_stabilize_should_boost(tailsitter_enabled: bool, assisted_flight: bool) -> bool {
    !(tailsitter_enabled && assisted_flight)
}

/// Tailsitter VTOL-transition skip: `in_vtol_transition && !assisted`.
#[must_use]
pub const fn motors_output_skip_tailsitter_transition(
    tailsitter_in_vtol_transition: bool,
    assisted_flight: bool,
) -> bool {
    tailsitter_in_vtol_transition && !assisted_flight
}

/// `(now - last_motors_active_ms) > 100`.
#[must_use]
pub const fn motors_inactive(now_ms: u32, last_motors_active_ms: u32) -> bool {
    now_ms.wrapping_sub(last_motors_active_ms) > MOTORS_INACTIVE_MS
}

/// `motors->get_throttle() > 0.01 || tiltrotor.motors_active()`.
#[must_use]
pub const fn motors_were_active(motors_throttle: f32, tiltrotor_motors_active: bool) -> bool {
    motors_throttle > MOTORS_ACTIVE_THROTTLE || tiltrotor_motors_active
}

/// Rate-controller inactive relax: `(now - last_att_control_ms) > 100`.
#[must_use]
pub const fn att_control_relax_stale(now_ms: u32, last_att_control_ms: u32) -> bool {
    now_ms.wrapping_sub(last_att_control_ms) > ATT_CONTROL_RELAX_MS
}

/// `hold_hover` climb demand: `target_climb_rate_cms * 0.01`.
#[must_use]
pub const fn climb_rate_ms_from_cms(target_climb_rate_cms: f32) -> f32 {
    target_climb_rate_cms * 0.01
}

/// `poscontrol.slow_descent`: from_alt > to_alt (absolute, else raw).
#[must_use]
pub const fn guided_slow_descent(from_alt_cm: i32, to_alt_cm: i32) -> bool {
    from_alt_cm > to_alt_cm
}

/// `guided_update` climb path: GUIDED + `guided_takeoff` + current < next.
#[must_use]
pub const fn guided_update_climbing(
    in_guided: bool,
    guided_takeoff: bool,
    current_alt_cm: i32,
    next_wp_alt_cm: i32,
) -> bool {
    in_guided && guided_takeoff && current_alt_cm < next_wp_alt_cm
}

/// Upstream `QuadPlane::guided_mode_enabled` gates.
#[must_use]
pub const fn guided_mode_enabled(
    available: bool,
    in_guided: bool,
    in_auto: bool,
    auto_loiter_turns: bool,
    guided_mode: i8,
) -> bool {
    if !available {
        return false;
    }
    if !in_guided && !in_auto {
        return false;
    }
    if in_auto && auto_loiter_turns {
        return false;
    }
    guided_mode != 0
}

/// ModeRTL `_enter` switches to QRTL when `Q_RTL_MODE == QRTL_ALWAYS`.
#[must_use]
pub const fn rtl_mode_qrtl_always(mode: RtlMode) -> bool {
    matches!(mode, RtlMode::QrtlAlways)
}

/// ModeRTL VTOL-landing: `SWITCH_QRTL` or `VTOL_APPROACH_QRTL`.
#[must_use]
pub const fn rtl_mode_vtol_landing(mode: RtlMode) -> bool {
    matches!(mode, RtlMode::SwitchQrtl | RtlMode::VtolApproachQrtl)
}

/// Leftover `Q_THRST_LOSS_OPT` bits, upstream `ThrustLoss::Option`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(i32)]
pub enum ThrustLossOption {
    /// Bit 0, upstream `ThrustLoss::Option::DISABLED`.
    Disabled = 1 << 0,
    /// Bit 1, upstream `ThrustLoss::Option::VTOL_ONLY`.
    VtolOnly = 1 << 1,
}

impl ThrustLossOption {
    /// Upstream discriminant.
    #[must_use]
    pub const fn as_i32(self) -> i32 {
        self as i32
    }
}

/// `sq(radians(15.0))` — leftover tilt-target reject.
pub const THRUST_LOSS_TILT_LIMIT_DEG: f32 = 15.0;

/// `sq(radians(15.0))`.
pub const THRUST_LOSS_TILT_LIMIT_RAD_SQ: f32 = {
    let r = 15.0 * 3.141_592_7 / 180.0;
    r * r
};

/// Leftover attitude-error reject, degrees.
pub const THRUST_LOSS_ANGLE_ERROR_DEG: f32 = 30.0;

/// Leftover throttle floor (`get_throttle_in() < 0.25`).
pub const THRUST_LOSS_THROTTLE_MIN: f32 = 0.25;

/// Leftover throttle-saturation gate (`get_throttle_in() < 0.9`).
pub const THRUST_LOSS_THROTTLE_SAT: f32 = 0.9;

/// Floor on leftover `takeoff_time_limit_ms`, upstream `MAX(..., 5000)`.
pub const TAKEOFF_FAILURE_TIME_LIMIT_MIN_MS: u32 = 5000;

/// Upstream `ThrustLoss::option_is_set`.
#[must_use]
pub const fn thrust_loss_option_is_set(options: i32, option: ThrustLossOption) -> bool {
    options & option.as_i32() != 0
}

/// `DISABLED` bit — check is off in every mode.
#[must_use]
pub const fn thrust_loss_disabled(options: i32) -> bool {
    thrust_loss_option_is_set(options, ThrustLossOption::Disabled)
}

/// `VTOL_ONLY` while not in a Q* / VTOL-auto mode.
#[must_use]
pub const fn thrust_loss_vtol_only_skip(options: i32, in_vtol_mode: bool) -> bool {
    thrust_loss_option_is_set(options, ThrustLossOption::VtolOnly) && !in_vtol_mode
}

/// Already boosting, disarmed, not flying, or spool not unlimited.
#[must_use]
pub const fn thrust_loss_already_engaged_or_idle(
    thrust_boost: bool,
    armed: bool,
    is_flying: bool,
    spool_unlimited: bool,
) -> bool {
    thrust_boost || !armed || !is_flying || !spool_unlimited
}

/// Target tilt `xy().length_squared() > sq(radians(15))`.
#[must_use]
pub const fn thrust_loss_tilt_too_steep(att_target_xy_rad_len_sq: f32) -> bool {
    att_target_xy_rad_len_sq > THRUST_LOSS_TILT_LIMIT_RAD_SQ
}

/// Throttle below 90% and not already upper-limited.
#[must_use]
pub const fn thrust_loss_throttle_not_saturated(throttle_in: f32, throttle_upper: bool) -> bool {
    throttle_in < THRUST_LOSS_THROTTLE_SAT && !throttle_upper
}

/// Throttle below 25% — reject (avoids low-command false positives).
#[must_use]
pub const fn thrust_loss_throttle_too_low(throttle_in: f32) -> bool {
    throttle_in < THRUST_LOSS_THROTTLE_MIN
}

/// No NED vel, or `vel_NED.z` is not positive (not descending).
#[must_use]
pub const fn thrust_loss_not_descending(have_vel_ned: bool, vel_ned_z: f32) -> bool {
    !have_vel_ned || vel_ned_z <= 0.0
}

/// Attitude error at or above 30 deg — aircraft already lost control.
#[must_use]
pub const fn thrust_loss_attitude_lost(att_error_deg: f32) -> bool {
    att_error_deg >= THRUST_LOSS_ANGLE_ERROR_DEG
}

/// Leftover `run_esc_calibration` passthrough (0 when disarmed).
#[must_use]
pub const fn esc_cal_passthrough(mode: i8, armed: bool, throttle_input: f32) -> f32 {
    if !armed {
        return 0.0;
    }
    match mode {
        1 => throttle_input * 0.01,
        2 => 1.0,
        _ => 0.0,
    }
}

/// `is_positive(takeoff_failure_scalar)`.
#[must_use]
pub const fn takeoff_failure_scalar_armed(scalar: f32) -> bool {
    scalar > 0.0
}

/// `is_positive(scalar) && elapsed > takeoff_time_limit_ms`.
#[must_use]
pub const fn takeoff_failure_timed_out(scalar: f32, elapsed_ms: u32, limit_ms: u32) -> bool {
    takeoff_failure_scalar_armed(scalar) && elapsed_ms > limit_ms
}

/// `MAX(travel_time_s * takeoff_failure_scalar * 1000, 5000)`.
#[must_use]
pub const fn takeoff_failure_time_limit_ms(travel_time_s: f32, scalar: f32) -> u32 {
    let scaled = travel_time_s * scalar * 1000.0;
    if scaled > TAKEOFF_FAILURE_TIME_LIMIT_MIN_MS as f32 {
        scaled as u32
    } else {
        TAKEOFF_FAILURE_TIME_LIMIT_MIN_MS
    }
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
        assert_eq!(on_main, 17);
        assert_eq!(this_slice, 1);
        assert_eq!(remaining, 1);
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
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "motors_output / hold / set_armed",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "guided / QRTL / RTL_MODE",
            PortStatus::OnMain
        ));
        assert!(completeness_has(
            "thrust-loss / ESC-cal / takeoff-failure",
            PortStatus::ThisSlice
        ));
        assert!(completeness_has("logging", PortStatus::OnMain));
        assert!(completeness_has("AUTO mission VTOL", PortStatus::OnMain));
        assert_eq!(on_main_items().count(), 17);
        assert_eq!(this_slice_items().count(), 1);
        assert_eq!(remaining_items().count(), 1);
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

    #[test]
    fn leftover_thrust_loss_esc_cal_and_takeoff_failure_helpers() {
        assert!(thrust_loss_disabled(ThrustLossOption::Disabled.as_i32()));
        assert!(!thrust_loss_disabled(0));
        assert!(thrust_loss_vtol_only_skip(
            ThrustLossOption::VtolOnly.as_i32(),
            false
        ));
        assert!(!thrust_loss_vtol_only_skip(
            ThrustLossOption::VtolOnly.as_i32(),
            true
        ));
        assert!(thrust_loss_already_engaged_or_idle(true, true, true, true));
        assert!(thrust_loss_tilt_too_steep(1.0));
        assert!(!thrust_loss_tilt_too_steep(0.0));
        assert!(thrust_loss_throttle_not_saturated(0.5, false));
        assert!(thrust_loss_throttle_too_low(0.2));
        assert!(thrust_loss_not_descending(false, 1.0));
        assert!(thrust_loss_attitude_lost(THRUST_LOSS_ANGLE_ERROR_DEG));
        assert_eq!((esc_cal_passthrough(1, true, 50.0) * 100.0) as i32, 50);
        assert_eq!(esc_cal_passthrough(2, true, 0.0) as i32, 1);
        assert!(!takeoff_failure_scalar_armed(0.0));
        assert!(takeoff_failure_scalar_armed(1.0));
        assert_eq!(takeoff_failure_time_limit_ms(1.0, 0.0), 5000);
        assert!(takeoff_failure_timed_out(1.0, 5001, 5000));
        assert!(!takeoff_failure_timed_out(0.0, 5001, 5000));
    }

    #[test]
    fn leftover_guided_start_update_and_rtl_mode_helpers() {
        assert!(guided_slow_descent(20000, 15000));
        assert!(!guided_slow_descent(15000, 20000));
        assert!(guided_update_climbing(true, true, 8000, 12000));
        assert!(!guided_update_climbing(false, true, 8000, 12000));
        assert!(!guided_update_climbing(true, false, 8000, 12000));
        assert!(!guided_update_climbing(true, true, 12000, 12000));
        assert!(guided_mode_enabled(true, true, false, false, 1));
        assert!(guided_mode_enabled(true, false, true, false, 1));
        assert!(!guided_mode_enabled(false, true, false, false, 1));
        assert!(!guided_mode_enabled(true, false, true, true, 1));
        assert!(!guided_mode_enabled(true, true, false, false, 0));
        assert!(rtl_mode_qrtl_always(RtlMode::QrtlAlways));
        assert!(!rtl_mode_qrtl_always(RtlMode::SwitchQrtl));
        assert!(rtl_mode_vtol_landing(RtlMode::SwitchQrtl));
        assert!(rtl_mode_vtol_landing(RtlMode::VtolApproachQrtl));
        assert!(!rtl_mode_vtol_landing(RtlMode::QrtlAlways));
        assert!(!rtl_mode_vtol_landing(RtlMode::None));
    }

    #[test]
    fn leftover_motors_output_hold_and_inactive_helpers() {
        assert!(hold_stabilize_ground_idle(0.0, false));
        assert!(!hold_stabilize_ground_idle(0.1, false));
        assert!(!hold_stabilize_ground_idle(0.0, true));
        assert!(hold_stabilize_should_boost(false, false));
        assert!(!hold_stabilize_should_boost(true, true));
        assert!(motors_output_skip_tailsitter_transition(true, false));
        assert!(!motors_output_skip_tailsitter_transition(true, true));
        assert!(motors_inactive(101, 0));
        assert!(!motors_inactive(100, 0));
        assert!(att_control_relax_stale(101, 0));
        assert!(motors_were_active(0.02, false));
        assert!(!motors_were_active(0.01, false));
        assert!(motors_were_active(0.0, true));
        assert_eq!(climb_rate_ms_from_cms(100.0) as i32, 1);
        assert_eq!(MOTORS_INACTIVE_MS, 100);
        assert_eq!(ATT_CONTROL_RELAX_MS, 100);
    }
}
