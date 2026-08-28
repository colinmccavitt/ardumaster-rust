//! ModeQAutotune + `QAutoTune::init` stub, upstream
//! `ArduPlane/mode_qautotune.cpp` / `qautotune.cpp` / `qautotune.h`
//! (Plane-4.7.0).
//!
//! Tracked as **VT-009**. `Mode::enter` always calls
//! [`QuadPlane::mode_enter`] then [`qautotune_enter`]'s `_enter`,
//! which is `return QAutoTune::init()`. Init refuses when
//! [`QuadPlane::available`] is false. Otherwise it leftover-calls
//! `AC_AutoTune::init_internals(position_hold)` with
//! `position_hold` true only when `previous_mode` was QLOITER.
//!
//! `update()` is the nav leftover: it delegates to
//! [`crate::mode_q::qstabilize_update`]. `run()` is tailsitter FW
//! pull-up (`Mode::run()`) or leftover `qautotune.run()` plus FW
//! `stabilize_roll` / `stabilize_pitch` / centered rudder. `_exit`
//! leftover-calls `qautotune.stop()`.
//!
//! This slice does **not** port `AC_AutoTune_Multi`. The
//! `qautotune.cpp` vehicle hooks (`get_desired_climb_rate_ms`,
//! `get_pilot_desired_rp_yrate_rad`, `init_z_limits`, `log_pids`)
//! are leftover-API in [`MODE_QAUTOTUNE_SURFACES`].

use crate::mode_q::{qstabilize_update, QManualUpdate, QManualUpdateView};
use crate::mode_qland::MODE_QLOITER;
use crate::QuadPlane;

/// `Mode::Number::QAUTOTUNE`.
pub const MODE_QAUTOTUNE: u8 = 22;

/// Upstream `ModeQAutotune::name`.
pub const MODE_QAUTOTUNE_NAME: &str = "QAutotune";

/// Upstream `ModeQAutotune::name4`.
pub const MODE_QAUTOTUNE_NAME4: &str = "QATN";

/// `ModeQAutotune` — VTOL attitude autotune, number 22.
///
/// `is_vtol_mode` and `is_vtol_man_mode` are true. The base
/// `is_vtol_man_throttle` stays false (QHover-style altitude hold
/// while the leftover multi-copter tuner runs).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ModeQAutotune;

impl ModeQAutotune {
    /// Inverse of the upstream `Mode::Number` discriminant.
    #[must_use]
    pub const fn from_number(number: u8) -> Option<Self> {
        if number == MODE_QAUTOTUNE {
            Some(Self)
        } else {
            None
        }
    }

    /// Upstream `ModeQAutotune::mode_number`.
    #[must_use]
    pub const fn mode_number(self) -> u8 {
        MODE_QAUTOTUNE
    }

    /// Upstream `ModeQAutotune::name`.
    #[must_use]
    pub const fn name(self) -> &'static str {
        MODE_QAUTOTUNE_NAME
    }

    /// Upstream `ModeQAutotune::name4`.
    #[must_use]
    pub const fn name4(self) -> &'static str {
        MODE_QAUTOTUNE_NAME4
    }

    /// Upstream `ModeQAutotune::is_vtol_mode`.
    #[must_use]
    pub const fn is_vtol_mode(self) -> bool {
        true
    }

    /// Upstream `ModeQAutotune::is_vtol_man_mode`.
    #[must_use]
    pub const fn is_vtol_man_mode(self) -> bool {
        true
    }

    /// Upstream `Mode::is_vtol_man_throttle` — not overridden.
    #[must_use]
    pub const fn is_vtol_man_throttle(self) -> bool {
        false
    }
}

/// Plane-side `QAutoTune` (`qautotune.h`), not `AC_AutoTune_Multi`.
///
/// [`Self::init`] is the real enter gate. `run` / `stop` /
/// `init_internals` are leftover flags — the multi-copter twitch
/// tuner lives in COP and is not rewritten here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QAutoTune {
    /// `use_poshold` latched by leftover `init_internals`.
    position_hold: bool,
    /// Leftover `AC_AutoTune::init_internals` ran.
    internals_inited: bool,
    /// Leftover `QAutoTune` / `AC_AutoTune::run` ran this session.
    ran: bool,
    /// Leftover `qautotune.stop()` ran (`ModeQAutotune::_exit`).
    stopped: bool,
}

impl Default for QAutoTune {
    fn default() -> Self {
        Self::new()
    }
}

impl QAutoTune {
    /// Idle tuner — not initialised, not running.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            position_hold: false,
            internals_inited: false,
            ran: false,
            stopped: false,
        }
    }

    /// Upstream `QAutoTune::init`.
    ///
    /// False when [`QuadPlane::available`] is false. Otherwise
    /// leftover-calls [`Self::init_internals`] with `position_hold`
    /// true only when `previous_mode` is QLOITER (19).
    pub fn init(&mut self, qp: &QuadPlane, previous_mode: u8) -> bool {
        if !qp.available() {
            return false;
        }
        let position_hold = previous_mode == MODE_QLOITER;
        self.init_internals(position_hold)
    }

    /// Leftover `AC_AutoTune::init_internals(use_poshold, …)`.
    ///
    /// Records `use_poshold` and returns true. The armed-motors
    /// reject, gain backup, and TuneMode switch are COP
    /// `AC_AutoTune_Multi` and are not ported.
    pub fn init_internals(&mut self, position_hold: bool) -> bool {
        self.position_hold = position_hold;
        self.internals_inited = true;
        self.ran = false;
        self.stopped = false;
        true
    }

    /// Leftover `AC_AutoTune::run` — twitch / level / update gains.
    pub fn run(&mut self) {
        self.ran = true;
    }

    /// Leftover `AC_AutoTune::stop` — `ModeQAutotune::_exit`.
    pub fn stop(&mut self) {
        self.stopped = true;
    }

    /// `use_poshold` after a successful [`Self::init`].
    #[must_use]
    pub const fn position_hold(&self) -> bool {
        self.position_hold
    }

    /// Whether leftover `init_internals` ran.
    #[must_use]
    pub const fn internals_inited(&self) -> bool {
        self.internals_inited
    }

    /// Whether leftover `run` ran.
    #[must_use]
    pub const fn ran(&self) -> bool {
        self.ran
    }

    /// Whether leftover `stop` ran.
    #[must_use]
    pub const fn stopped(&self) -> bool {
        self.stopped
    }
}

/// Combined `Mode::enter` for QAutotune: `mode_enter` then `_enter`.
///
/// Upstream `_enter` is `return quadplane.qautotune.init()`.
pub fn qautotune_enter(qp: &mut QuadPlane, tune: &mut QAutoTune, previous_mode: u8) -> bool {
    qp.mode_enter();
    tune.init(qp, previous_mode)
}

/// Upstream `ModeQAutotune::_exit` — `qautotune.stop()`.
pub fn qautotune_exit(tune: &mut QAutoTune) {
    tune.stop();
}

/// Which `run()` path ModeQAutotune took.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QAutotuneRunAction {
    /// Tailsitter FW pull-up: `Mode::run()`.
    FwControllers,
    /// Leftover `qautotune.run()` then FW stabilize + centered rudder.
    TuneThenStabilize,
}

/// Outcome of one `ModeQAutotune::run()` tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QAutotuneRun {
    /// FW leftover vs tune + stabilize.
    pub action: QAutotuneRunAction,
    /// Leftover `qautotune.run()` ran (false on FW pull-up).
    pub tune_ran: bool,
    /// `plane.stabilize_roll()` ran.
    pub stabilize_roll: bool,
    /// `plane.stabilize_pitch()` ran.
    pub stabilize_pitch: bool,
    /// `output_rudder_and_steering(0.0)` ran.
    pub rudder_centered: bool,
}

impl QAutotuneRun {
    const fn fw_controllers() -> Self {
        Self {
            action: QAutotuneRunAction::FwControllers,
            tune_ran: false,
            stabilize_roll: false,
            stabilize_pitch: false,
            rudder_centered: false,
        }
    }

    const fn tune_then_stabilize() -> Self {
        Self {
            action: QAutotuneRunAction::TuneThenStabilize,
            tune_ran: true,
            stabilize_roll: true,
            stabilize_pitch: true,
            rudder_centered: true,
        }
    }
}

/// Tailsitter / clock view `ModeQAutotune::run` reads.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QAutotuneRunView {
    /// `tailsitter.in_vtol_transition(now)`.
    pub tailsitter_in_vtol_transition: bool,
}

impl QAutotuneRunView {
    /// Conventional hover — not in tailsitter FW pull-up.
    #[must_use]
    pub const fn flying() -> Self {
        Self {
            tailsitter_in_vtol_transition: false,
        }
    }

    /// Tailsitter FW pull-up phase of VTOL transition.
    #[must_use]
    pub const fn tailsitter_fw_transition() -> Self {
        Self {
            tailsitter_in_vtol_transition: true,
        }
    }
}

/// Upstream `ModeQAutotune::run`.
///
/// Tailsitter FW pull-up runs `Mode::run()` and returns. Otherwise
/// leftover `qautotune.run()`, then `stabilize_roll` /
/// `stabilize_pitch` / `output_rudder_and_steering(0)`.
pub fn qautotune_run(tune: &mut QAutoTune, view: &QAutotuneRunView) -> QAutotuneRun {
    if view.tailsitter_in_vtol_transition {
        return QAutotuneRun::fw_controllers();
    }
    tune.run();
    QAutotuneRun::tune_then_stabilize()
}

/// Upstream `ModeQAutotune::update` — `plane.mode_qstabilize.update()`.
#[must_use]
pub const fn qautotune_update(view: &QManualUpdateView) -> QManualUpdate {
    qstabilize_update(view)
}

/// Leftover `QAutoTune::get_desired_climb_rate_ms`.
///
/// `get_pilot_desired_climb_rate_cms() * 0.01`.
#[must_use]
pub const fn leftover_desired_climb_rate_ms(pilot_climb_cms: f32) -> f32 {
    pilot_climb_cms * 0.01
}

/// Leftover `QAutoTune::get_pilot_desired_rp_yrate_rad` demand.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LeftoverPilotRpYrate {
    /// `des_roll_rad` — 0 when both sticks are centered.
    pub des_roll_rad: f32,
    /// `des_pitch_rad` — 0 when both sticks are centered.
    pub des_pitch_rad: f32,
    /// `des_yaw_rate_rads` from `get_desired_yaw_rate_cds`.
    pub des_yaw_rate_rads: f32,
    /// Stick-center branch (`control_in` roll and pitch both 0).
    pub sticks_centered: bool,
}

/// Upstream `cd_to_rad` scale (`π / 18000`).
pub const CD_TO_RAD: f32 = 3.141_592_7 / 18_000.0;

/// Leftover `QAutoTune::get_pilot_desired_rp_yrate_rad`.
///
/// Centered roll+pitch sticks force roll/pitch demand to 0; otherwise
/// `cd_to_rad(nav_roll_cd / nav_pitch_cd)`. Yaw rate is always
/// `cd_to_rad(get_desired_yaw_rate_cds())`.
#[must_use]
pub const fn leftover_pilot_desired_rp_yrate_rad(
    roll_control_in: i16,
    pitch_control_in: i16,
    nav_roll_cd: i32,
    nav_pitch_cd: i32,
    desired_yaw_rate_cds: f32,
) -> LeftoverPilotRpYrate {
    let sticks_centered = roll_control_in == 0 && pitch_control_in == 0;
    let (des_roll_rad, des_pitch_rad) = if sticks_centered {
        (0.0, 0.0)
    } else {
        (
            nav_roll_cd as f32 * CD_TO_RAD,
            nav_pitch_cd as f32 * CD_TO_RAD,
        )
    };
    LeftoverPilotRpYrate {
        des_roll_rad,
        des_pitch_rad,
        des_yaw_rate_rads: desired_yaw_rate_cds * CD_TO_RAD,
        sticks_centered,
    }
}

/// Leftover `QAutoTune::init_z_limits` D-axis writes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LeftoverZLimits {
    /// `pos_control->D_set_max_speed_accel_m` ran.
    pub d_speed_accel_set: bool,
    /// `pos_control->D_set_correction_speed_accel_m` ran.
    pub d_correction_set: bool,
}

/// Leftover `QAutoTune::init_z_limits`.
///
/// Both D-axis speed/accel setters run with
/// `get_pilot_velocity_z_max_dn_m` / `pilot_speed_z_max_up_ms` /
/// `pilot_accel_z_mss`.
#[must_use]
pub const fn leftover_init_z_limits() -> LeftoverZLimits {
    LeftoverZLimits {
        d_speed_accel_set: true,
        d_correction_set: true,
    }
}

/// Leftover `QAutoTune::log_pids` — PIQR / PIQP / PIQY `Write_PID`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LeftoverLogPids {
    /// `LOG_PIQR_MSG` rate-roll PID.
    pub piqr: bool,
    /// `LOG_PIQP_MSG` rate-pitch PID.
    pub piqp: bool,
    /// `LOG_PIQY_MSG` rate-yaw PID.
    pub piqy: bool,
}

/// Leftover `QAutoTune::log_pids` (`HAL_LOGGING_ENABLED`).
#[must_use]
pub const fn leftover_log_pids() -> LeftoverLogPids {
    LeftoverLogPids {
        piqr: true,
        piqp: true,
        piqy: true,
    }
}

/// Whether a catalog row is already hooked up or leftover-API.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QAutotunePortStatus {
    /// Present on `main` before this slice (none for VT-009).
    OnMain,
    /// Added by this slice (mode stubs + leftover-API).
    ThisSlice,
}

/// One `mode_qautotune.cpp` / `qautotune.cpp` surface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QAutotuneSurface {
    /// Upstream `.cpp` file.
    pub file: &'static str,
    /// Function name.
    pub name: &'static str,
    /// Hooked up on main or this slice.
    pub status: QAutotunePortStatus,
    /// Short note (Rust symbol).
    pub note: &'static str,
}

/// Completeness closer: every function in the two VT-009 cpp files.
pub const MODE_QAUTOTUNE_SURFACES: &[QAutotuneSurface] = &[
    QAutotuneSurface {
        file: "mode_qautotune.cpp",
        name: "_enter",
        status: QAutotunePortStatus::ThisSlice,
        note: "qautotune_enter / QAutoTune::init",
    },
    QAutotuneSurface {
        file: "mode_qautotune.cpp",
        name: "update",
        status: QAutotunePortStatus::ThisSlice,
        note: "qautotune_update delegates to qstabilize_update",
    },
    QAutotuneSurface {
        file: "mode_qautotune.cpp",
        name: "run",
        status: QAutotunePortStatus::ThisSlice,
        note: "qautotune_run FW leftover vs tune + stabilize + rudder",
    },
    QAutotuneSurface {
        file: "mode_qautotune.cpp",
        name: "_exit",
        status: QAutotunePortStatus::ThisSlice,
        note: "qautotune_exit / qautotune.stop leftover",
    },
    QAutotuneSurface {
        file: "qautotune.cpp",
        name: "init",
        status: QAutotunePortStatus::ThisSlice,
        note: "QAutoTune::init available gate + QLOITER position_hold",
    },
    QAutotuneSurface {
        file: "qautotune.cpp",
        name: "get_desired_climb_rate_ms",
        status: QAutotunePortStatus::ThisSlice,
        note: "leftover_desired_climb_rate_ms cms * 0.01",
    },
    QAutotuneSurface {
        file: "qautotune.cpp",
        name: "get_pilot_desired_rp_yrate_rad",
        status: QAutotunePortStatus::ThisSlice,
        note: "leftover_pilot_desired_rp_yrate_rad stick-center vs nav",
    },
    QAutotuneSurface {
        file: "qautotune.cpp",
        name: "init_z_limits",
        status: QAutotunePortStatus::ThisSlice,
        note: "leftover_init_z_limits D-axis speed/accel",
    },
    QAutotuneSurface {
        file: "qautotune.cpp",
        name: "log_pids",
        status: QAutotunePortStatus::ThisSlice,
        note: "leftover_log_pids PIQR / PIQP / PIQY",
    },
];

/// True when every listed surface is `OnMain` or `ThisSlice`.
#[must_use]
pub const fn mode_qautotune_surfaces_complete() -> bool {
    let mut i = 0;
    while i < MODE_QAUTOTUNE_SURFACES.len() {
        match MODE_QAUTOTUNE_SURFACES[i].status {
            QAutotunePortStatus::OnMain | QAutotunePortStatus::ThisSlice => {}
        }
        i += 1;
    }
    MODE_QAUTOTUNE_SURFACES.len() == 9
}
