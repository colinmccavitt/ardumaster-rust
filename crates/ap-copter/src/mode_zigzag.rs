//! `ModeZigZag` init leftover, upstream `ArduCopter/mode_zigzag.cpp`.
//!
//! Tracked as **COP-024**. ZigZag's horizontal hold is [`ap_wpnav::Loiter`]
//! (COP-011); this file does not rewrite it. What it still owns is the
//! ModeZigZag leftover those leftovers do not decide: convert the pilot
//! through AC_Loiter's angle max, seat `init_target`, then reset the
//! A/B / auto machine so a re-entry cannot resume a half-flown grid.
//!
//! # `init` always succeeds and always forgets A/B
//!
//! `ModeZigZag::init` never reads `ignore_checks`. It is ModeLoiter's
//! seating leftover plus `init_auto`: `stage` is `STORING_POINTS`, both
//! destinations are zeroed, `is_auto` is false, `auto_stage` is
//! `MANUAL`, `line_count` is 0, and `is_suspended` is false. A mode
//! change mid-grid must not keep flying the previous pair.
//!
//! The vertical controller is initialised only when it is inactive — a
//! mode change mid-climb must not reset a controller that is already
//! tracking — then writes the same pilot speed / accel limits to both
//! the max and the correction setters.

use crate::stick_nav::pilot_desired_lean_angles_rad;
use ap_wpnav::{InitTargetContext, InitTargetLeftover, Loiter};

/// `Mode::Number::ZIGZAG`.
pub const MODE_NUMBER_ZIGZAG: u8 = 24;

/// `ModeZigZag` capability flags from `mode.h`.
///
/// These are not computed. They are the leftover catalog of what the
/// class reports to `set_mode` and the arming checks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ZigZagModeFlags {
    /// `mode_number()`.
    pub mode_number: u8,
    /// `requires_position()`. ZigZag flies a stored A/B pair.
    pub requires_position: bool,
    /// `has_manual_throttle()`. False: the D controller owns throttle.
    pub has_manual_throttle: bool,
    /// `allows_arming(...)`.
    pub allows_arming: bool,
    /// `is_autopilot()`. True: the auto grid is an autopilot path.
    pub is_autopilot: bool,
}

/// Upstream `ModeZigZag` flags.
#[must_use]
pub const fn zigzag_mode_flags() -> ZigZagModeFlags {
    ZigZagModeFlags {
        mode_number: MODE_NUMBER_ZIGZAG,
        requires_position: true,
        has_manual_throttle: false,
        allows_arming: true,
        is_autopilot: true,
    }
}

/// Upstream `ModeZigZag::has_user_takeoff`.
///
/// Unlike Sport / FlowHold, ZigZag always allows a user takeoff —
/// `must_navigate` is unread. The mode can climb while flying the grid.
#[must_use]
pub const fn zigzag_has_user_takeoff(_must_navigate: bool) -> bool {
    true
}

/// `ModeZigZag::ZigZagState` after `init`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZigZagStage {
    /// `STORING_POINTS` — pilot has manual control while A and B are saved.
    StoringPoints,
    /// `AUTO` — flying the stored pair / sideways legs.
    Auto,
    /// `MANUAL_REGAIN` — switch in the middle, pilot has control again.
    ManualRegain,
}

/// `ModeZigZag::AutoState` after `init_auto`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZigZagAutoStage {
    /// `MANUAL` — not in ZigZag Auto.
    Manual,
    /// `AB_MOVING` — flying A→B or B→A.
    AbMoving,
    /// `SIDEWAYS` — flying the sideways offset.
    Sideways,
}

/// What `ModeZigZag::init` reads.
#[derive(Debug, Clone, Copy)]
pub struct ZigZagInitView {
    /// `channel_roll->norm_input_dz()`.
    pub roll_in_norm: f32,
    /// `channel_pitch->norm_input_dz()`.
    pub pitch_in_norm: f32,
    /// `rc().has_valid_input()`.
    pub has_valid_input: bool,
    /// `_attitude_control.lean_angle_max_rad()` for [`Loiter::get_angle_max_rad`].
    pub attitude_lean_angle_max_rad: f32,
    /// `_pos_control.get_lean_angle_max_rad()` for [`Loiter::get_angle_max_rad`].
    pub pos_lean_angle_max_rad: f32,
    /// `attitude_control->get_althold_lean_angle_max_rad()`.
    pub althold_lean_angle_max_rad: f32,
    /// `pos_control->D_is_active()`.
    pub d_is_active: bool,
    /// `get_pilot_speed_dn_ms()`.
    pub speed_dn_ms: f32,
    /// `get_pilot_speed_up_ms()`.
    pub speed_up_ms: f32,
    /// `get_pilot_accel_D_mss()`.
    pub accel_d_mss: f32,
    /// Context [`Loiter::init_target`] reads.
    pub init_target_ctx: InitTargetContext,
}

impl ZigZagInitView {
    /// Pilot defaults, D controller already running, sticks centred.
    #[must_use]
    pub fn typical() -> Self {
        Self {
            roll_in_norm: 0.0,
            pitch_in_norm: 0.0,
            has_valid_input: true,
            attitude_lean_angle_max_rad: 0.523_598_8,
            pos_lean_angle_max_rad: 0.523_598_8,
            althold_lean_angle_max_rad: 0.523_598_8,
            d_is_active: true,
            speed_dn_ms: 2.5,
            speed_up_ms: 2.5,
            accel_d_mss: 2.5,
            init_target_ctx: InitTargetContext::default(),
        }
    }
}

/// Leftover of one `ModeZigZag::init`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ZigZagInit {
    /// Euler roll demand passed to `set_pilot_desired_acceleration_rad`.
    pub target_roll_rad: f32,
    /// Euler pitch demand passed to `set_pilot_desired_acceleration_rad`.
    pub target_pitch_rad: f32,
    /// Always true: `update_simple_mode` runs before the conversion.
    pub update_simple_mode: bool,
    /// Always true. Pilot-accel shaping stays on COP-011; this records the call.
    pub set_pilot_desired_acceleration: bool,
    /// [`Loiter::init_target`] leftover.
    pub init_target: InitTargetLeftover,
    /// `D_init_controller()` — only when the controller was inactive.
    pub init_d_controller: bool,
    /// Speed / accel written to both limit setters.
    pub speed_dn_ms: f32,
    /// Climb speed written to both limit setters.
    pub speed_up_ms: f32,
    /// Vertical accel written to both limit setters.
    pub accel_d_mss: f32,
    /// Always true: `D_set_max_speed_accel_m`.
    pub set_max_speed_accel: bool,
    /// Always true: `D_set_correction_speed_accel_m`, same three numbers.
    pub set_correction_speed_accel: bool,
    /// `stage` after `init`. Always [`ZigZagStage::StoringPoints`].
    pub stage: ZigZagStage,
    /// `dest_A_ne_m.zero()` ran.
    pub dest_a_cleared: bool,
    /// `dest_B_ne_m.zero()` ran.
    pub dest_b_cleared: bool,
    /// `init_auto()` leftover of `is_auto`.
    pub is_auto: bool,
    /// `init_auto()` leftover of `auto_stage`.
    pub auto_stage: ZigZagAutoStage,
    /// `init_auto()` leftover of `line_count`.
    pub line_count: u16,
    /// `init_auto()` leftover of `is_suspended`.
    pub is_suspended: bool,
    /// Always true. `ignore_checks` is unread.
    pub ok: bool,
}

/// Upstream `ModeZigZag::init`. Always succeeds; `ignore_checks` is unread.
///
/// Converts the pilot through [`Loiter::get_angle_max_rad`], records the
/// pilot-accel leftover, seats AC_Loiter with [`Loiter::init_target`],
/// then runs `init_auto`. The D controller is initialised only when it
/// is not already active.
#[must_use]
pub fn zigzag_init(_ignore_checks: bool, loiter: &mut Loiter, view: &ZigZagInitView) -> ZigZagInit {
    let angle_max_rad = loiter.get_angle_max_rad(
        view.attitude_lean_angle_max_rad,
        view.pos_lean_angle_max_rad,
    );
    let (target_roll_rad, target_pitch_rad) = pilot_desired_lean_angles_rad(
        view.roll_in_norm,
        view.pitch_in_norm,
        angle_max_rad,
        view.althold_lean_angle_max_rad,
        view.has_valid_input,
    );
    let init_target = loiter.init_target(view.init_target_ctx);
    ZigZagInit {
        target_roll_rad,
        target_pitch_rad,
        update_simple_mode: true,
        set_pilot_desired_acceleration: true,
        init_target,
        init_d_controller: !view.d_is_active,
        speed_dn_ms: view.speed_dn_ms,
        speed_up_ms: view.speed_up_ms,
        accel_d_mss: view.accel_d_mss,
        set_max_speed_accel: true,
        set_correction_speed_accel: true,
        stage: ZigZagStage::StoringPoints,
        dest_a_cleared: true,
        dest_b_cleared: true,
        is_auto: false,
        auto_stage: ZigZagAutoStage::Manual,
        line_count: 0,
        is_suspended: false,
        ok: true,
    }
}
