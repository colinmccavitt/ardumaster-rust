//! `ModeZigZag` init / run leftover, upstream `ArduCopter/mode_zigzag.cpp`.
//!
//! Tracked as **COP-024**. ZigZag's horizontal hold is [`ap_wpnav::Loiter`]
//! (COP-011); this file does not rewrite it. What it still owns is the
//! ModeZigZag leftover those leftovers do not decide: convert the pilot
//! through AC_Loiter's angle max, seat `init_target`, then reset the
//! A/B / auto machine so a re-entry cannot resume a half-flown grid.
//!
//! # `run` is AUTO or the loiter leftover, never both unless dropping
//!
//! `ModeZigZag::run` writes the D max limits, clamps direction / line
//! count, then either flies `auto_control` or hands back to the pilot.
//! A same-tick drop from AUTO to MANUAL_REGAIN still runs
//! `manual_control` afterwards — that is why a terrain-failed wpnav
//! tick is not a hover in AUTO. wpnav / dest-calc leftovers stay on
//! COP-012; this records the call and the fallback.
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

use crate::alt_hold::{alt_hold_state, AltHoldInputs, AltHoldModeState};
use crate::mode_althold::AltHoldVertical;
use crate::mode_loiter::LoiterNavAction;
use crate::mode_stabilize::RateIReset;
use crate::pilot_input::pilot_desired_yaw_rate_rads;
use crate::stick_nav::pilot_desired_lean_angles_rad;
use ap_math::scalar::constrain_value;
use ap_motors::spool::{DesiredSpoolState, SpoolState};
use ap_wpnav::{
    InitTargetContext, InitTargetLeftover, Loiter, UpdateLoiterContext, UpdateLoiterLeftover,
};

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

/// `ZIGZAG_WP_RADIUS_M`. Destination is reached only inside this radius.
pub const ZIGZAG_WP_RADIUS_M: f32 = 3.0;

/// `ZIGZAG_LINE_INFINITY`. `_line_num` of `-1` flies the grid forever.
pub const ZIGZAG_LINE_INFINITY: i16 = -1;

/// `ModeZigZag::Destination`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZigZagDestination {
    /// Destination A.
    A,
    /// Destination B.
    B,
}

/// `ModeZigZag::Direction` after the `run` clamp.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZigZagDirection {
    /// `FORWARD`.
    Forward = 0,
    /// `RIGHT`.
    Right = 1,
    /// `BACKWARD`.
    Backward = 2,
    /// `LEFT`.
    Left = 3,
}

impl ZigZagDirection {
    /// Clamp `_direction` to `0..=3`, then map. Upstream `constrain_int16`.
    #[must_use]
    pub const fn from_param(direction: i16) -> Self {
        match direction {
            1 => Self::Right,
            2 => Self::Backward,
            3 => Self::Left,
            _ => Self::Forward,
        }
    }
}

/// What the AUTO branch of `ModeZigZag::run` decided to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZigZagRunAction {
    /// Stage was not AUTO, or AUTO already handed back to the pilot.
    None,
    /// Still en-route: `auto_control()`.
    AutoControl,
    /// `return_to_manual_control(maintain_target)`.
    ReturnToManual { maintain_target: bool },
    /// Reached a sideways dest with lines remaining: move to the other of A/B.
    SaveOrMoveToOther,
    /// Reached an A/B dest with lines remaining: `spray(false)` then `move_to_side()`.
    MoveToSide,
    /// Line count exhausted: `init_auto()` then `return_to_manual_control(true)`.
    InitAutoThenManual,
}

/// Attitude call ZigZag `manual_control` issues.
///
/// Unlike ModeLoiter (always thrust-vector), ZigZag uses Euler roll/pitch
/// plus yaw-rate on MotorStopped / Takeoff / Flying, and the thrust-vector
/// heading-rate call only on the two landed states.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZigZagAttitude {
    /// `input_euler_angle_roll_pitch_euler_rate_yaw_rad`.
    EulerRateYaw,
    /// `input_thrust_vector_rate_heading_rads`.
    ThrustVector,
}

/// Upstream `Mode::is_disarmed_or_landed`.
#[must_use]
pub const fn zigzag_is_disarmed_or_landed(
    armed: bool,
    auto_armed: bool,
    land_complete: bool,
) -> bool {
    !armed || !auto_armed || land_complete
}

/// What `ModeZigZag::reached_destination` reads.
#[derive(Debug, Clone, Copy)]
pub struct ZigZagReachedView {
    /// `wp_nav->reached_wp_destination()`.
    pub wp_reached: bool,
    /// `wp_nav->get_wp_distance_to_destination_m()`.
    pub wp_distance_m: f32,
    /// `reach_wp_time_ms` before this tick. `0` means not started.
    pub reach_wp_time_ms: u32,
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
    /// `_wp_delay_s`.
    pub wp_delay_s: i16,
}

/// Leftover of one `ModeZigZag::reached_destination`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ZigZagReached {
    /// True only when wpnav agrees, the radius is tight, and the delay elapsed.
    pub reached: bool,
    /// `reach_wp_time_ms` after this tick. Unchanged when the radius is not met.
    pub reach_wp_time_ms: u32,
}

/// Upstream `ModeZigZag::reached_destination`.
///
/// wpnav's own "reached" flag is not enough — the vehicle must also be
/// inside [`ZIGZAG_WP_RADIUS_M`]. The first passing tick stamps
/// `reach_wp_time_ms`; later ticks wait `_wp_delay_s` (clamped to `0..=127`)
/// seconds before returning true.
#[must_use]
pub fn zigzag_reached_destination(view: &ZigZagReachedView) -> ZigZagReached {
    if !view.wp_reached || view.wp_distance_m > ZIGZAG_WP_RADIUS_M {
        return ZigZagReached {
            reached: false,
            reach_wp_time_ms: view.reach_wp_time_ms,
        };
    }
    let reach_wp_time_ms = if view.reach_wp_time_ms == 0 {
        view.now_ms
    } else {
        view.reach_wp_time_ms
    };
    let delay_ms = (view.wp_delay_s.clamp(0, 127) as u32).saturating_mul(1000);
    ZigZagReached {
        reached: view.now_ms.wrapping_sub(reach_wp_time_ms) >= delay_ms,
        reach_wp_time_ms,
    }
}

/// Leftover of one `ModeZigZag::return_to_manual_control`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ZigZagReturnToManual {
    /// `stage == AUTO` so the helper actually ran.
    pub applied: bool,
    /// Stage after the helper. [`ZigZagStage::ManualRegain`] when applied.
    pub stage: ZigZagStage,
    /// Always false when applied (`is_auto = false`).
    pub is_auto: bool,
    /// `spray(false)` — compiled out unless the sprayer leftover exists.
    pub spray_off: bool,
    /// `loiter_nav->clear_pilot_desired_acceleration()`.
    pub clear_pilot_desired_acceleration: bool,
    /// `init_target_m(wp_dest.xy())` when `maintain_target`, else `init_target()`.
    pub init_target: Option<InitTargetLeftover>,
    /// `maintain_target` argument.
    pub maintain_target: bool,
}

/// Upstream `ModeZigZag::return_to_manual_control`.
///
/// A no-op unless `stage` is AUTO. The applied path always sprays off,
/// clears the pilot-accel leftover, reseats AC_Loiter (at the waypoint
/// when `maintain_target`, otherwise at the current target), and clears
/// `is_auto` so a later `run_auto` cannot resume a half-cleared grid.
#[must_use]
pub fn zigzag_return_to_manual(
    loiter: &mut Loiter,
    maintain_target: bool,
    stage: ZigZagStage,
    is_auto: bool,
    wp_dest_ne_m: (f32, f32),
    init_target_ctx: InitTargetContext,
) -> ZigZagReturnToManual {
    if stage != ZigZagStage::Auto {
        return ZigZagReturnToManual {
            applied: false,
            stage,
            is_auto,
            spray_off: false,
            clear_pilot_desired_acceleration: false,
            init_target: None,
            maintain_target,
        };
    }
    let init_target = if maintain_target {
        loiter.init_target_m(
            ap_math::vector2::Vector2f::new(wp_dest_ne_m.0, wp_dest_ne_m.1),
            init_target_ctx,
        )
    } else {
        loiter.init_target(init_target_ctx)
    };
    ZigZagReturnToManual {
        applied: true,
        stage: ZigZagStage::ManualRegain,
        is_auto: false,
        spray_off: true,
        clear_pilot_desired_acceleration: true,
        init_target: Some(init_target),
        maintain_target,
    }
}

/// Leftover of one `ModeZigZag::auto_control`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ZigZagAutoControl {
    /// Pilot yaw-rate demand, rad/s.
    pub target_yaw_rate_rads: f32,
    /// Always `THROTTLE_UNLIMITED`.
    pub desired_spool: DesiredSpoolState,
    /// `wp_nav->update_wpnav()` leftover of success. Terrain failure is `false`.
    pub wpnav_ok: bool,
    /// Always true: `D_update_controller`.
    pub update_d_controller: bool,
    /// Always true: Euler roll/pitch from wpnav, yaw-rate from the pilot.
    pub input_euler_angle: bool,
    /// `return_to_manual_control(false)` because wpnav failed.
    pub return_to_manual: bool,
}

/// Upstream `ModeZigZag::auto_control`.
///
/// wpnav owns the roll/pitch leftover (COP-012); this records the call
/// and the terrain-failure fallback. A failed `update_wpnav` drops the
/// vehicle back to the pilot on the same tick.
#[must_use]
pub fn zigzag_auto_control(view: &ZigZagRunView) -> ZigZagAutoControl {
    let target_yaw_rate_rads = pilot_desired_yaw_rate_rads(
        view.yaw_in_norm,
        view.yaw_rate_degs,
        view.yaw_expo,
        view.has_valid_input,
    );
    ZigZagAutoControl {
        target_yaw_rate_rads,
        desired_spool: DesiredSpoolState::ThrottleUnlimited,
        wpnav_ok: view.wpnav_ok,
        update_d_controller: true,
        input_euler_angle: true,
        return_to_manual: !view.wpnav_ok,
    }
}

/// Manual leftover of one `ModeZigZag::manual_control` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ZigZagManual {
    /// What the altitude-hold machine returned.
    pub state: AltHoldModeState,
    /// Spool command from the machine, if any.
    pub desired_spool: Option<DesiredSpoolState>,
    /// Euler roll demand passed to `set_pilot_desired_acceleration_rad`.
    pub target_roll_rad: f32,
    /// Euler pitch demand passed to `set_pilot_desired_acceleration_rad`.
    pub target_pitch_rad: f32,
    /// Yaw-rate demand, rad/s.
    pub target_yaw_rate_rads: f32,
    /// Climb rate after the speed clamp (and, on takeoff / flying, after
    /// the avoidance identity).
    pub target_climb_rate_ms: f32,
    /// Always true: `update_simple_mode` runs before the conversion.
    pub update_simple_mode: bool,
    /// Always true. Pilot-accel shaping stays on COP-011.
    pub set_pilot_desired_acceleration: bool,
    /// `loiter_nav->soften_for_landing` when `land_complete_maybe`.
    pub soften_for_landing: bool,
    /// Rate-controller I-term reset.
    pub reset_rate_i: RateIReset,
    /// `reset_yaw_target_and_rate` this iteration.
    pub reset_yaw_target_and_rate: bool,
    /// The `reset_rate` argument. ZigZag uses the default `true` on
    /// MotorStopped and LandedGroundIdle.
    pub reset_yaw_rate: bool,
    /// Vertical-controller leftover.
    pub vertical: AltHoldVertical,
    /// Whether this tick re-seated or ticked AC_Loiter.
    pub nav: LoiterNavAction,
    /// [`Loiter::init_target`] leftover when [`LoiterNavAction::InitTarget`].
    pub init_target: Option<InitTargetLeftover>,
    /// [`Loiter::update`] leftover when [`LoiterNavAction::Update`].
    pub update: Option<UpdateLoiterLeftover>,
    /// `takeoff.start_m` should run (Takeoff and the helper is idle).
    pub start_takeoff: bool,
    /// Altitude handed to `takeoff.start_m`, clamped to `[0, 10]`.
    pub takeoff_start_alt_m: f32,
    /// Attitude call for this state.
    pub attitude: ZigZagAttitude,
    /// Always true: `pos_control->D_update_controller()` after the switch.
    pub update_d_controller: bool,
}

/// Upstream `ModeZigZag::manual_control`.
///
/// Same altitude-hold machine as ModeLoiter, including the
/// `soften_for_landing` leftover. Attitude is Euler on the flying /
/// stopped / takeoff branches and thrust-vector only when landed.
/// Avoidance and surface-tracking are compiled out — the climb rate
/// passes through unchanged, which is the path this leftover records.
#[must_use]
pub fn zigzag_manual_control(loiter: &mut Loiter, view: &ZigZagRunView) -> ZigZagManual {
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
    let target_yaw_rate_rads = pilot_desired_yaw_rate_rads(
        view.yaw_in_norm,
        view.yaw_rate_degs,
        view.yaw_expo,
        view.has_valid_input,
    );
    let target_climb_rate_ms = constrain_value(
        view.target_climb_rate_ms,
        -view.speed_dn_ms,
        view.speed_up_ms,
    );
    let soften_for_landing = if view.land_complete_maybe {
        loiter.soften_for_landing()
    } else {
        false
    };
    let decision = alt_hold_state(&AltHoldInputs {
        armed: view.armed,
        spool_state: view.spool_state,
        takeoff_running: view.takeoff_running,
        auto_armed: view.auto_armed,
        land_complete: view.land_complete,
        using_interlock: view.using_interlock,
        target_climb_rate_ms,
    });
    let (reset_rate_i, reset_yaw, reset_yaw_rate, vertical, start_takeoff, nav, attitude) =
        match decision.state {
            AltHoldModeState::MotorStopped => (
                RateIReset::Hard,
                true,
                true,
                AltHoldVertical::Relax,
                false,
                LoiterNavAction::InitTarget,
                ZigZagAttitude::EulerRateYaw,
            ),
            AltHoldModeState::LandedGroundIdle => (
                RateIReset::Smooth,
                true,
                true,
                AltHoldVertical::Relax,
                false,
                LoiterNavAction::InitTarget,
                ZigZagAttitude::ThrustVector,
            ),
            AltHoldModeState::LandedPreTakeoff => (
                RateIReset::Smooth,
                false,
                false,
                AltHoldVertical::Relax,
                false,
                LoiterNavAction::InitTarget,
                ZigZagAttitude::ThrustVector,
            ),
            AltHoldModeState::Takeoff => (
                RateIReset::None,
                false,
                false,
                AltHoldVertical::Takeoff,
                !view.takeoff_running,
                LoiterNavAction::Update,
                ZigZagAttitude::EulerRateYaw,
            ),
            AltHoldModeState::Flying => (
                RateIReset::None,
                false,
                false,
                AltHoldVertical::ClimbRate,
                false,
                LoiterNavAction::Update,
                ZigZagAttitude::EulerRateYaw,
            ),
        };
    let (init_target, update) = match nav {
        LoiterNavAction::InitTarget => (Some(loiter.init_target(view.init_target_ctx)), None),
        LoiterNavAction::Update => (None, Some(loiter.update(view.update_ctx))),
    };
    ZigZagManual {
        state: decision.state,
        desired_spool: decision.desired_spool,
        target_roll_rad,
        target_pitch_rad,
        target_yaw_rate_rads,
        target_climb_rate_ms,
        update_simple_mode: true,
        set_pilot_desired_acceleration: true,
        soften_for_landing,
        reset_rate_i,
        reset_yaw_target_and_rate: reset_yaw,
        reset_yaw_rate,
        vertical,
        nav,
        init_target,
        update,
        start_takeoff,
        takeoff_start_alt_m: constrain_value(view.takeoff_alt_m, 0.0, 10.0),
        attitude,
        update_d_controller: true,
    }
}

/// Pilot / vehicle / grid view `ModeZigZag::run` reads.
#[derive(Debug, Clone, Copy)]
pub struct ZigZagRunView {
    /// `stage` before this tick.
    pub stage: ZigZagStage,
    /// `is_auto`.
    pub is_auto: bool,
    /// `auto_stage`.
    pub auto_stage: ZigZagAutoStage,
    /// `line_count`.
    pub line_count: u16,
    /// `_line_num` before the clamp.
    pub line_num: i16,
    /// `_direction` before the clamp.
    pub direction: i16,
    /// `ab_dest_stored`.
    pub ab_dest_stored: ZigZagDestination,
    /// `motors->get_interlock()`.
    pub interlock: bool,
    /// `wp_nav->update_wpnav()` leftover of success.
    pub wpnav_ok: bool,
    /// `wp_nav->get_wp_destination_NED_m().xy()` for `maintain_target`.
    pub wp_dest_ne_m: (f32, f32),
    /// Inputs [`zigzag_reached_destination`] reads.
    pub reached: ZigZagReachedView,
    /// `channel_roll->norm_input_dz()`.
    pub roll_in_norm: f32,
    /// `channel_pitch->norm_input_dz()`.
    pub pitch_in_norm: f32,
    /// `channel_yaw->norm_input_dz()`.
    pub yaw_in_norm: f32,
    /// `rc().has_valid_input()`.
    pub has_valid_input: bool,
    /// `_attitude_control.lean_angle_max_rad()` for [`Loiter::get_angle_max_rad`].
    pub attitude_lean_angle_max_rad: f32,
    /// `_pos_control.get_lean_angle_max_rad()` for [`Loiter::get_angle_max_rad`].
    pub pos_lean_angle_max_rad: f32,
    /// `attitude_control->get_althold_lean_angle_max_rad()`.
    pub althold_lean_angle_max_rad: f32,
    /// Pilot yaw command-model rate, deg/s.
    pub yaw_rate_degs: f32,
    /// Pilot yaw command-model expo.
    pub yaw_expo: f32,
    /// Already-converted climb rate, m/s, up positive.
    pub target_climb_rate_ms: f32,
    /// `get_pilot_speed_dn_ms()`.
    pub speed_dn_ms: f32,
    /// `get_pilot_speed_up_ms()`.
    pub speed_up_ms: f32,
    /// `get_pilot_accel_D_mss()`. Written every run tick.
    pub accel_d_mss: f32,
    /// `motors->armed()`.
    pub armed: bool,
    /// `motors->get_spool_state()`.
    pub spool_state: SpoolState,
    /// `takeoff.running()`.
    pub takeoff_running: bool,
    /// `copter.ap.auto_armed`.
    pub auto_armed: bool,
    /// `copter.ap.land_complete`.
    pub land_complete: bool,
    /// `copter.ap.land_complete_maybe`.
    pub land_complete_maybe: bool,
    /// `copter.ap.using_interlock`.
    pub using_interlock: bool,
    /// `g2.pilot_takeoff_alt_m`.
    pub takeoff_alt_m: f32,
    /// Context [`Loiter::init_target`] reads.
    pub init_target_ctx: InitTargetContext,
    /// Context [`Loiter::update`] reads.
    pub update_ctx: UpdateLoiterContext,
}

impl ZigZagRunView {
    /// Armed, auto-armed, airborne, storing points, motors unlimited.
    #[must_use]
    pub const fn storing_points() -> Self {
        Self {
            stage: ZigZagStage::StoringPoints,
            is_auto: false,
            auto_stage: ZigZagAutoStage::Manual,
            line_count: 0,
            line_num: 0,
            direction: 0,
            ab_dest_stored: ZigZagDestination::A,
            interlock: true,
            wpnav_ok: true,
            wp_dest_ne_m: (0.0, 0.0),
            reached: ZigZagReachedView {
                wp_reached: false,
                wp_distance_m: 10.0,
                reach_wp_time_ms: 0,
                now_ms: 1_000,
                wp_delay_s: 0,
            },
            roll_in_norm: 0.0,
            pitch_in_norm: 0.0,
            yaw_in_norm: 0.0,
            has_valid_input: true,
            attitude_lean_angle_max_rad: 0.523_598_8,
            pos_lean_angle_max_rad: 0.523_598_8,
            althold_lean_angle_max_rad: 0.523_598_8,
            yaw_rate_degs: 200.0,
            yaw_expo: 0.0,
            target_climb_rate_ms: 0.0,
            speed_dn_ms: 2.5,
            speed_up_ms: 2.5,
            accel_d_mss: 2.5,
            armed: true,
            spool_state: SpoolState::ThrottleUnlimited,
            takeoff_running: false,
            auto_armed: true,
            land_complete: false,
            land_complete_maybe: false,
            using_interlock: false,
            takeoff_alt_m: 2.5,
            init_target_ctx: InitTargetContext {
                lean_angle_max_rad: 0.523_598_8,
                accel_target_ne_mss: ap_math::vector2::Vector2f { x: 0.0, y: 0.0 },
                roll_rad: 0.0,
                pitch_rad: 0.0,
            },
            update_ctx: UpdateLoiterContext {
                now_ms: 0,
                dt_s: 0.01,
                ekf_gnd_spd_limit_ms: 50.0,
                vel_desired_ne_ms: ap_math::vector2::Vector2f { x: 0.0, y: 0.0 },
                pos_desired_ne_m: ap_math::vector2::Vector2f { x: 0.0, y: 0.0 },
                vel_pid_kp: 1.0,
                attitude_lean_angle_max_rad: 0.523_598_8,
                pos_lean_angle_max_rad: 0.523_598_8,
                avoidance_on: true,
            },
        }
    }

    /// Flying the A/B pair, not yet at the waypoint.
    #[must_use]
    pub const fn auto_enroute() -> Self {
        let mut view = Self::storing_points();
        view.stage = ZigZagStage::Auto;
        view.is_auto = true;
        view.auto_stage = ZigZagAutoStage::AbMoving;
        view.line_num = 4;
        view.line_count = 1;
        view.ab_dest_stored = ZigZagDestination::A;
        view
    }
}

/// Leftover of one `ModeZigZag::run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ZigZagRun {
    /// Always true: `D_set_max_speed_accel_m` at the top of `run`.
    pub set_max_speed_accel: bool,
    /// `_direction` after `constrain_int16(..., 0, 3)`.
    pub direction: ZigZagDirection,
    /// `_line_num` after `constrain_int16(..., ZIGZAG_LINE_INFINITY, INT16_MAX)`.
    pub line_num: i16,
    /// Stage after the AUTO branch (and any same-tick return-to-manual).
    pub stage: ZigZagStage,
    /// `is_auto` after any `init_auto` / return-to-manual.
    pub is_auto: bool,
    /// What the AUTO branch decided.
    pub action: ZigZagRunAction,
    /// `AP_Notify::events.waypoint_complete = 1`.
    pub waypoint_complete: bool,
    /// `spray(false)` on the sideways-advance and return-to-manual paths.
    pub spray_off: bool,
    /// Other of A/B when [`ZigZagRunAction::SaveOrMoveToOther`].
    pub move_dest: Option<ZigZagDestination>,
    /// `reached_destination` leftover when stage was AUTO and we were armed.
    pub reached: Option<ZigZagReached>,
    /// `return_to_manual_control` leftover, if that helper ran.
    pub return_to_manual: Option<ZigZagReturnToManual>,
    /// `auto_control` leftover, if that helper ran.
    pub auto_control: Option<ZigZagAutoControl>,
    /// `manual_control` leftover. Runs whenever the (possibly updated)
    /// stage is STORING_POINTS or MANUAL_REGAIN — including the same
    /// tick that just dropped out of AUTO.
    pub manual: Option<ZigZagManual>,
}

/// Upstream `ModeZigZag::run`.
///
/// Writes the D max limits, clamps direction / line count, then either
/// flies `auto_control` or hands back to the pilot. A same-tick drop
/// from AUTO to MANUAL_REGAIN still runs `manual_control` afterwards —
/// that is why a terrain-failed wpnav tick is not a hover in AUTO.
#[must_use]
pub fn zigzag_run(loiter: &mut Loiter, view: &ZigZagRunView) -> ZigZagRun {
    let direction = ZigZagDirection::from_param(view.direction.clamp(0, 3));
    let line_num = view.line_num.clamp(ZIGZAG_LINE_INFINITY, i16::MAX);

    let mut stage = view.stage;
    let mut is_auto = view.is_auto;
    let mut action = ZigZagRunAction::None;
    let mut waypoint_complete = false;
    let mut spray_off = false;
    let mut move_dest = None;
    let mut reached = None;
    let mut return_to_manual = None;
    let mut auto_control = None;

    if stage == ZigZagStage::Auto {
        if zigzag_is_disarmed_or_landed(view.armed, view.auto_armed, view.land_complete)
            || !view.interlock
        {
            action = ZigZagRunAction::ReturnToManual {
                maintain_target: false,
            };
            let rtm = zigzag_return_to_manual(
                loiter,
                false,
                stage,
                is_auto,
                view.wp_dest_ne_m,
                view.init_target_ctx,
            );
            stage = rtm.stage;
            is_auto = rtm.is_auto;
            spray_off = rtm.spray_off;
            return_to_manual = Some(rtm);
        } else {
            let hit = zigzag_reached_destination(&view.reached);
            reached = Some(hit);
            if hit.reached {
                waypoint_complete = true;
                if view.is_auto {
                    if line_num == ZIGZAG_LINE_INFINITY
                        || (view.line_count as i32) < i32::from(line_num)
                    {
                        if view.auto_stage == ZigZagAutoStage::Sideways {
                            action = ZigZagRunAction::SaveOrMoveToOther;
                            move_dest = Some(match view.ab_dest_stored {
                                ZigZagDestination::A => ZigZagDestination::B,
                                ZigZagDestination::B => ZigZagDestination::A,
                            });
                        } else {
                            spray_off = true;
                            action = ZigZagRunAction::MoveToSide;
                        }
                    } else {
                        action = ZigZagRunAction::InitAutoThenManual;
                        is_auto = false;
                        let rtm = zigzag_return_to_manual(
                            loiter,
                            true,
                            stage,
                            is_auto,
                            view.wp_dest_ne_m,
                            view.init_target_ctx,
                        );
                        stage = rtm.stage;
                        is_auto = rtm.is_auto;
                        spray_off = rtm.spray_off;
                        return_to_manual = Some(rtm);
                    }
                } else {
                    action = ZigZagRunAction::ReturnToManual {
                        maintain_target: true,
                    };
                    let rtm = zigzag_return_to_manual(
                        loiter,
                        true,
                        stage,
                        is_auto,
                        view.wp_dest_ne_m,
                        view.init_target_ctx,
                    );
                    stage = rtm.stage;
                    is_auto = rtm.is_auto;
                    spray_off = rtm.spray_off;
                    return_to_manual = Some(rtm);
                }
            } else {
                action = ZigZagRunAction::AutoControl;
                let ac = zigzag_auto_control(view);
                if ac.return_to_manual {
                    let rtm = zigzag_return_to_manual(
                        loiter,
                        false,
                        stage,
                        is_auto,
                        view.wp_dest_ne_m,
                        view.init_target_ctx,
                    );
                    stage = rtm.stage;
                    is_auto = rtm.is_auto;
                    spray_off = rtm.spray_off;
                    return_to_manual = Some(rtm);
                }
                auto_control = Some(ac);
            }
        }
    }

    let manual = if stage == ZigZagStage::StoringPoints || stage == ZigZagStage::ManualRegain {
        Some(zigzag_manual_control(loiter, view))
    } else {
        None
    };

    ZigZagRun {
        set_max_speed_accel: true,
        direction,
        line_num,
        stage,
        is_auto,
        action,
        waypoint_complete,
        spray_off,
        move_dest,
        reached,
        return_to_manual,
        auto_control,
        manual,
    }
}
