//! QLOITER / QLAND / LoiterAltQLand `_enter` plus QLAND `run()`,
//! upstream `ArduPlane/mode_qloiter.cpp` / `mode_qland.cpp` /
//! `mode_LoiterAltQLand.cpp` (Plane-4.7.0).
//!
//! Tracked as **VT-005**. This slice is [`qland_run`]:
//! `ModeQLand::run` always calls `ModeQLoiter::run`, and the
//! QLAND leftover inside that run is descent-rate, land-final,
//! and [`QuadPlane::check_land_complete`]. `Mode::enter` always calls
//! [`QuadPlane::mode_enter`] then the mode's `_enter`. QLoiter
//! initialises loiter_nav, latches the D-axis speed / accel limits,
//! calls [`QuadPlane::init_throttle_wait`], records
//! `last_loiter_ms`, and clears the precland timestamp.
//!
//! QLand calls QLoiter `_enter` (not `enter` — `mode_enter` is not
//! run a second time), then forces `throttle_wait = false`,
//! `setup_target_position()`, `poscontrol` to `QPOS_LAND_DESCEND`,
//! snapshots AGL into `last_land_final_agl_m`, and zeros the land
//! detector timers.
//!
//! LoiterAltQLand is a fixed-wing loiter that hands off to QLand.
//! Already in a VTOL mode (`previous_mode->is_vtol_mode()` or
//! `quadplane.in_vtol_mode()`) is `set_mode(QLAND,
//! ModeReason::LOITER_ALT_IN_VTOL)`. Otherwise it runs
//! `ModeLoiter::_enter`, retargets altitude to `Q_RTL_ALT`, and
//! [`switch_qland`] may immediately enter QLand
//! (`ModeReason::LOITER_ALT_REACHED_QLAND`).

use crate::air_mode::QOption;
use crate::landing::{
    LandCompleteResult, LandCompleteView, LandDetectView, LandFinalView, RelaxView,
};
use crate::poscontrol::PositionControlState;
use crate::QuadPlane;

/// `Mode::Number::QLOITER`.
pub const MODE_QLOITER: u8 = 19;
/// `Mode::Number::QLAND`.
pub const MODE_QLAND: u8 = 20;
/// `Mode::Number::LOITER_ALT_QLAND`.
pub const MODE_LOITER_ALT_QLAND: u8 = 25;

/// `ModeReason::LOITER_ALT_REACHED_QLAND`.
pub const MODE_REASON_LOITER_ALT_REACHED_QLAND: u8 = 46;
/// `ModeReason::LOITER_ALT_IN_VTOL`.
pub const MODE_REASON_LOITER_ALT_IN_VTOL: u8 = 47;

/// Default `Q_RTL_ALT`, upstream `AP_GROUPINFO("RTL_ALT", 35, QuadPlane, qrtl_alt_m, 15)`.
pub const Q_RTL_ALT_DEFAULT_M: f32 = 15.0;

/// The three modes this slice ports `_enter` for.
///
/// Discriminants match `Mode::Number`. QLoiter and QLand are
/// `is_vtol_mode`. Only QLoiter is `is_vtol_man_mode` (pilot lean
/// into the loiter controller). LoiterAltQLand is a `ModeLoiter`
/// subclass — fixed-wing until it hands off.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum QLandFamily {
    /// `ModeQLoiter` — position-hold VTOL, altitude-hold throttle.
    Loiter = 19,
    /// `ModeQLand` — VTOL descent; `_enter` seeds QLoiter then land-descend.
    Land = 20,
    /// `ModeLoiterAltQLand` — FW loiter down to `Q_RTL_ALT`, then QLand.
    LoiterAltQLand = 25,
}

impl QLandFamily {
    /// Inverse of the upstream `Mode::Number` discriminant.
    #[must_use]
    pub const fn from_number(number: u8) -> Option<Self> {
        match number {
            MODE_QLOITER => Some(Self::Loiter),
            MODE_QLAND => Some(Self::Land),
            MODE_LOITER_ALT_QLAND => Some(Self::LoiterAltQLand),
            _ => None,
        }
    }

    /// Upstream `Mode::mode_number`.
    #[must_use]
    pub const fn mode_number(self) -> u8 {
        self as u8
    }

    /// Upstream `Mode::is_vtol_mode`.
    ///
    /// QLoiter / QLand override this to true. LoiterAltQLand keeps
    /// the `ModeLoiter` base `false`.
    #[must_use]
    pub const fn is_vtol_mode(self) -> bool {
        matches!(self, Self::Loiter | Self::Land)
    }

    /// Upstream `Mode::is_vtol_man_mode`.
    ///
    /// QLoiter overrides this to true (pilot lean). QLand does not
    /// — it is an automatic descent. LoiterAltQLand is FW loiter.
    #[must_use]
    pub const fn is_vtol_man_mode(self) -> bool {
        matches!(self, Self::Loiter)
    }

    /// Upstream `Mode::is_vtol_man_throttle`.
    ///
    /// None of these three override the base `false`. QLoiter uses
    /// `init_throttle_wait`; QLand forces `throttle_wait = false`.
    #[must_use]
    pub const fn is_vtol_man_throttle(self) -> bool {
        false
    }

    /// Upstream `ModeQLand::_pre_arm_checks` — always `false`.
    ///
    /// QLand refuses arming in this mode. The other two keep the
    /// base checks (not rewritten here).
    #[must_use]
    pub const fn qland_pre_arm_refuses(self) -> bool {
        matches!(self, Self::Land)
    }
}

/// Pilot / flying / clock view QLoiter `_enter` reads.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QLoiterEnterView {
    /// `get_throttle_input()`, the stick `init_throttle_wait` reads.
    pub throttle_input: i16,
    /// `plane.is_flying()`.
    pub is_flying: bool,
    /// `AP_HAL::millis()` copied onto `quadplane.last_loiter_ms`.
    pub now_ms: u32,
}

impl QLoiterEnterView {
    /// Stick + flying + clock as `ModeQLoiter::_enter` would read them.
    #[must_use]
    pub const fn new(throttle_input: i16, is_flying: bool, now_ms: u32) -> Self {
        Self {
            throttle_input,
            is_flying,
            now_ms,
        }
    }

    /// Parked on the ground at idle throttle — `throttle_wait` becomes true.
    #[must_use]
    pub const fn parked_idle() -> Self {
        Self {
            throttle_input: 0,
            is_flying: false,
            now_ms: 1_000,
        }
    }
}

/// Side effects QLoiter `_enter` records besides `throttle_wait`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QLoiterEnterState {
    /// `loiter_nav->clear_pilot_desired_acceleration()`.
    pub loiter_accel_cleared: bool,
    /// `loiter_nav->init_target()`.
    pub loiter_target_inited: bool,
    /// `pos_control->D_set_max_speed_accel_m` ran.
    pub d_speed_accel_set: bool,
    /// `pos_control->D_set_correction_speed_accel_m` ran.
    pub d_correction_set: bool,
    /// `quadplane.last_loiter_ms` after `_enter`.
    pub last_loiter_ms: u32,
    /// `ModeQLoiter::last_target_loc_set_ms` after `_enter` (always 0).
    pub last_target_loc_set_ms: u32,
}

impl Default for QLoiterEnterState {
    fn default() -> Self {
        Self::new()
    }
}

impl QLoiterEnterState {
    /// Nothing latched yet — before `_enter`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            loiter_accel_cleared: false,
            loiter_target_inited: false,
            d_speed_accel_set: false,
            d_correction_set: false,
            last_loiter_ms: 0,
            last_target_loc_set_ms: 0,
        }
    }
}

/// Apply `ModeQLoiter::_enter` without `Mode::enter`'s `mode_enter`.
///
/// QLand calls this (not [`qloiter_enter`]) so `mode_enter` is not
/// run twice on a QLand switch.
fn qloiter_enter_body(qp: &mut QuadPlane, view: QLoiterEnterView, state: &mut QLoiterEnterState) {
    state.loiter_accel_cleared = true;
    state.loiter_target_inited = true;
    state.d_speed_accel_set = true;
    state.d_correction_set = true;
    qp.init_throttle_wait(view.throttle_input, view.is_flying);
    state.last_loiter_ms = view.now_ms;
    state.last_target_loc_set_ms = 0;
}

/// Combined `Mode::enter` for QLoiter: `mode_enter` then `_enter`.
///
/// Always returns true.
pub fn qloiter_enter(
    qp: &mut QuadPlane,
    view: QLoiterEnterView,
    state: &mut QLoiterEnterState,
) -> bool {
    qp.mode_enter();
    qloiter_enter_body(qp, view, state);
    true
}

/// AGL / loiter view QLand `_enter` reads on top of QLoiter's.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QLandEnterView {
    /// Nested QLoiter `_enter` view (throttle / flying / clock).
    pub loiter: QLoiterEnterView,
    /// `plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING)`.
    pub height_above_ground_m: f32,
}

impl QLandEnterView {
    /// Nested loiter view plus the AGL snapshot QLand latches.
    #[must_use]
    pub const fn new(loiter: QLoiterEnterView, height_above_ground_m: f32) -> Self {
        Self {
            loiter,
            height_above_ground_m,
        }
    }

    /// Parked idle, 10 m AGL — a typical QLand switch from hover.
    #[must_use]
    pub const fn parked_idle() -> Self {
        Self {
            loiter: QLoiterEnterView::parked_idle(),
            height_above_ground_m: 10.0,
        }
    }
}

/// Side effects QLand `_enter` records besides poscontrol / throttle_wait.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QLandEnterState {
    /// Nested QLoiter `_enter` latch.
    pub qloiter: QLoiterEnterState,
    /// `quadplane.setup_target_position()` ran.
    pub target_position_setup: bool,
    /// `last_land_final_agl_m` snapshot from relative ground altitude.
    pub last_land_final_agl_m: f32,
    /// `landing_detect.lower_limit_start_ms` / `land_start_ms` zeroed.
    pub land_detect_cleared: bool,
    /// `landing_gear.deploy_for_landing()` ran.
    pub landing_gear_deployed: bool,
}

impl Default for QLandEnterState {
    fn default() -> Self {
        Self::new()
    }
}

impl QLandEnterState {
    /// Nothing latched yet — before `_enter`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            qloiter: QLoiterEnterState::new(),
            target_position_setup: false,
            last_land_final_agl_m: 0.0,
            land_detect_cleared: false,
            landing_gear_deployed: false,
        }
    }
}

/// Combined `Mode::enter` for QLand: `mode_enter` then `_enter`.
///
/// Upstream `ModeQLand::_enter` calls `mode_qloiter._enter()` (the
/// body, not `Mode::enter`), then clears `throttle_wait`, sets up
/// the target, and moves poscontrol to `QPOS_LAND_DESCEND`. Always
/// returns true.
pub fn qland_enter(qp: &mut QuadPlane, view: QLandEnterView, state: &mut QLandEnterState) -> bool {
    qp.mode_enter();
    qloiter_enter_body(qp, view.loiter, &mut state.qloiter);
    qp.set_throttle_wait(false);
    state.target_position_setup = true;
    qp.poscontrol_mut()
        .set_state(PositionControlState::LandDescend);
    state.last_land_final_agl_m = view.height_above_ground_m;
    state.land_detect_cleared = true;
    state.landing_gear_deployed = true;
    true
}

/// Which location LoiterAltQLand seeds the loiter from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoiterAltSeed {
    /// `already_in_a_loiter` — reuse `plane.next_WP_loc`.
    NextWp,
    /// Otherwise use `plane.current_loc`.
    CurrentLoc,
}

/// Guided-request altitude frame after `handle_guided_request`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GuidedAltFrame {
    /// `Location::AltFrame::ABOVE_TERRAIN` when terrain is enabled in QLAND.
    AboveTerrain,
    /// `Location::AltFrame::ABOVE_HOME` otherwise.
    AboveHome,
}

/// Why LoiterAltQLand `_enter` finished the way it did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoiterAltQLandAction {
    /// Already VTOL: `set_mode(QLAND, LOITER_ALT_IN_VTOL)`.
    HandoffInVtol,
    /// FW loiter, then `switch_qland` entered QLand.
    HandoffReachedQland,
    /// FW loiter; still above the QLand altitude.
    StayLoiter,
}

/// Plane / nav view LoiterAltQLand `_enter` reads.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LoiterAltQLandEnterView {
    /// `plane.previous_mode->is_vtol_mode()`.
    pub previous_is_vtol: bool,
    /// `plane.quadplane.in_vtol_mode()`.
    pub in_vtol_mode: bool,
    /// `plane.nav_controller->reached_loiter_target()`.
    pub reached_loiter_target: bool,
    /// `plane.nav_controller->data_is_stale()`.
    pub nav_data_stale: bool,
    /// `current_loc.get_height_above(next_WP_loc, dist)` after the
    /// guided retarget. `None` is a failed height read.
    pub height_above_next_wp_m: Option<f32>,
    /// `quadplane.qrtl_alt_m` (`Q_RTL_ALT`).
    pub qrtl_alt_m: f32,
    /// `plane.terrain_enabled_in_mode(Mode::Number::QLAND)`.
    pub terrain_enabled_in_qland: bool,
    /// Nested QLand `_enter` view if this path hands off.
    pub qland: QLandEnterView,
}

impl LoiterAltQLandEnterView {
    /// Fixed-wing, not yet at the loiter, default `Q_RTL_ALT`.
    #[must_use]
    pub const fn fw_above() -> Self {
        Self {
            previous_is_vtol: false,
            in_vtol_mode: false,
            reached_loiter_target: false,
            nav_data_stale: false,
            height_above_next_wp_m: Some(20.0),
            qrtl_alt_m: Q_RTL_ALT_DEFAULT_M,
            terrain_enabled_in_qland: false,
            qland: QLandEnterView::parked_idle(),
        }
    }
}

/// Outcome of [`loiter_alt_qland_enter`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LoiterAltQLandEnter {
    /// VTOL handoff / reached-QLand / stay in FW loiter.
    pub action: LoiterAltQLandAction,
    /// `ModeLoiter::_enter` ran (the FW path only).
    pub loiter_enter: bool,
    /// `handle_guided_request` ran (the FW path only).
    pub guided_request: bool,
    /// Which location seeded the loiter / guided WP.
    pub seed: Option<LoiterAltSeed>,
    /// Altitude written by `handle_guided_request`.
    pub guided_alt_m: f32,
    /// Frame of that altitude.
    pub guided_frame: GuidedAltFrame,
    /// `ModeReason` number if `set_mode(QLAND, …)` ran.
    pub mode_reason: Option<u8>,
    /// Nested QLand `_enter` latch when a handoff ran.
    pub qland: Option<QLandEnterState>,
}

impl LoiterAltQLandEnter {
    const fn stay(seed: LoiterAltSeed, alt_m: f32, frame: GuidedAltFrame) -> Self {
        Self {
            action: LoiterAltQLandAction::StayLoiter,
            loiter_enter: true,
            guided_request: true,
            seed: Some(seed),
            guided_alt_m: alt_m,
            guided_frame: frame,
            mode_reason: None,
            qland: None,
        }
    }
}

/// `already_in_a_loiter` — reuse `next_WP_loc` only when the loiter
/// target is reached and the nav data is fresh.
#[must_use]
pub const fn already_in_a_loiter(reached: bool, nav_data_stale: bool) -> bool {
    reached && !nav_data_stale
}

/// Upstream `ModeLoiterAltQLand::switch_qland` predicate.
///
/// QLand when height-above fails or is negative, **and** the loiter
/// target is reached. Stale nav data is not consulted here.
#[must_use]
pub const fn switch_qland(height_above_next_wp_m: Option<f32>, reached: bool) -> bool {
    let fail_or_negative = match height_above_next_wp_m {
        None => true,
        Some(dist) => dist < 0.0,
    };
    fail_or_negative && reached
}

/// Combined `Mode::enter` for LoiterAltQLand: `mode_enter` then `_enter`.
///
/// A VTOL handoff (or `switch_qland`) runs [`qland_enter`], matching
/// `set_mode(plane.mode_qland, …)` which calls `Mode::enter` again.
/// Always returns true.
pub fn loiter_alt_qland_enter(
    qp: &mut QuadPlane,
    view: LoiterAltQLandEnterView,
) -> LoiterAltQLandEnter {
    qp.mode_enter();
    if view.previous_is_vtol || view.in_vtol_mode {
        let mut qland = QLandEnterState::new();
        let _ok = qland_enter(qp, view.qland, &mut qland);
        return LoiterAltQLandEnter {
            action: LoiterAltQLandAction::HandoffInVtol,
            loiter_enter: false,
            guided_request: false,
            seed: None,
            guided_alt_m: 0.0,
            guided_frame: GuidedAltFrame::AboveHome,
            mode_reason: Some(MODE_REASON_LOITER_ALT_IN_VTOL),
            qland: Some(qland),
        };
    }

    let seed = if already_in_a_loiter(view.reached_loiter_target, view.nav_data_stale) {
        LoiterAltSeed::NextWp
    } else {
        LoiterAltSeed::CurrentLoc
    };
    let frame = if view.terrain_enabled_in_qland {
        GuidedAltFrame::AboveTerrain
    } else {
        GuidedAltFrame::AboveHome
    };

    if switch_qland(view.height_above_next_wp_m, view.reached_loiter_target) {
        let mut qland = QLandEnterState::new();
        let _ok = qland_enter(qp, view.qland, &mut qland);
        return LoiterAltQLandEnter {
            action: LoiterAltQLandAction::HandoffReachedQland,
            loiter_enter: true,
            guided_request: true,
            seed: Some(seed),
            guided_alt_m: view.qrtl_alt_m,
            guided_frame: frame,
            mode_reason: Some(MODE_REASON_LOITER_ALT_REACHED_QLAND),
            qland: Some(qland),
        };
    }

    LoiterAltQLandEnter::stay(seed, view.qrtl_alt_m, frame)
}

/// Default `Q_LAND_FINAL_SPD`, upstream `AP_GROUPINFO("LAND_FINAL_SPD", 26, QuadPlane, land_final_speed_ms, 0.5)`.
pub const Q_LAND_FINAL_SPD_DEFAULT_MS: f32 = 0.5;

/// Default `Q_WP_SPD_DN` / `wp_nav->get_default_speed_down_ms()`.
///
/// Upstream `AC_WPNav` `WP_SPD_DOWN_DEFAULT` is 1.5 m/s.
pub const Q_WP_SPD_DN_DEFAULT_MS: f32 = 1.5;

/// Extra metres above `Q_LAND_FINAL_ALT` where the descent interpolate
/// reaches the waypoint down-speed.
///
/// Upstream `land_final_alt_m + 6`.
pub const LAND_DESCENT_INTERP_SPAN_M: f32 = 6.0;

/// Which path `ModeQLand::run` (via `ModeQLoiter::run`) took this tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QLandRunAction {
    /// `assist.check_VTOL_recovery()` — QHover recovery.
    VtolRecovery,
    /// Tailsitter FW pull-up: `Mode::run()`.
    FwControllers,
    /// QLoiter `throttle_wait` leftover (QLAND `_enter` normally clears this).
    ThrottleWait,
    /// QLAND descent / land-final / land-complete leftover.
    Descend,
}

/// Outcome of one [`qland_run`] tick.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QLandRun {
    /// Recovery / FW / wait / descent.
    pub action: QLandRunAction,
    /// `ModeQLand::run` always calls `mode_qloiter.run()`.
    pub used_qloiter: bool,
    /// `check_land_final` promoted poscontrol to `QPOS_LAND_FINAL`.
    pub switched_land_final: bool,
    /// `setup_target_position()` after that switch.
    pub target_position_setup: bool,
    /// `landing_descent_rate_ms` this tick (positive is down).
    pub descent_rate_ms: f32,
    /// `D_set_pos_target_from_climb_rate_ms(-descent_rate, descent_rate>0)`.
    pub climb_rate_target_ms: f32,
    /// `ahrs.set_touchdown_expected(true)` in LAND_FINAL without
    /// `DISABLE_GROUND_EFFECT_COMP`.
    pub touchdown_expected: bool,
    /// `check_land_complete` result (idle on early-outs).
    pub land_complete: LandCompleteResult,
}

impl QLandRun {
    const fn early(action: QLandRunAction) -> Self {
        Self {
            action,
            used_qloiter: true,
            switched_land_final: false,
            target_position_setup: false,
            descent_rate_ms: 0.0,
            climb_rate_target_ms: 0.0,
            touchdown_expected: false,
            land_complete: LandCompleteResult::idle(),
        }
    }
}

/// Plane / nav view [`qland_run`] reads after QLoiter's common control.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QLandRunView {
    /// `quadplane.assist.check_VTOL_recovery()`.
    pub vtol_recovery: bool,
    /// `tailsitter.in_vtol_transition(now)`.
    pub tailsitter_in_vtol_transition: bool,
    /// `AP_HAL::millis()` (override-descent window; unused in this stub).
    pub now_ms: u32,
    /// `plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING)`.
    pub height_above_ground_m: f32,
    /// `wp_nav->get_default_speed_down_ms()`.
    pub wp_speed_down_ms: f32,
    /// `quadplane.land_final_speed_ms` (`Q_LAND_FINAL_SPD`).
    pub land_final_speed_ms: f32,
    /// Motors / height snapshot `check_land_final` / `check_land_complete` share.
    pub detect: LandDetectView,
    /// `plane.in_auto_mission_id(MAV_CMD_NAV_PAYLOAD_PLACE)`.
    pub payload_place: bool,
    /// `control_mode == mode_auto`.
    pub in_auto: bool,
    /// `mission.continue_after_land()`.
    pub continue_after_land: bool,
}

impl QLandRunView {
    /// Mid-descent tick: 12.5 m AGL, default speeds, motors flying.
    #[must_use]
    pub const fn descending() -> Self {
        Self {
            vtol_recovery: false,
            tailsitter_in_vtol_transition: false,
            now_ms: 1_000,
            height_above_ground_m: 12.5,
            wp_speed_down_ms: Q_WP_SPD_DN_DEFAULT_MS,
            land_final_speed_ms: Q_LAND_FINAL_SPD_DEFAULT_MS,
            detect: LandDetectView {
                relax: RelaxView::flying(),
                height_m: 12.5,
            },
            payload_place: false,
            in_auto: false,
            continue_after_land: false,
        }
    }

    /// Below `Q_LAND_FINAL_ALT`, still flying (land-final switch, not complete).
    #[must_use]
    pub const fn below_final() -> Self {
        Self {
            vtol_recovery: false,
            tailsitter_in_vtol_transition: false,
            now_ms: 1_000,
            height_above_ground_m: 3.0,
            wp_speed_down_ms: Q_WP_SPD_DN_DEFAULT_MS,
            land_final_speed_ms: Q_LAND_FINAL_SPD_DEFAULT_MS,
            detect: LandDetectView {
                relax: RelaxView::flying(),
                height_m: 3.0,
            },
            payload_place: false,
            in_auto: false,
            continue_after_land: false,
        }
    }

    /// Settled detector tick used for land-complete.
    #[must_use]
    pub const fn settled_complete(now_ms: u32, height_m: f32) -> Self {
        Self {
            vtol_recovery: false,
            tailsitter_in_vtol_transition: false,
            now_ms,
            height_above_ground_m: height_m,
            wp_speed_down_ms: Q_WP_SPD_DN_DEFAULT_MS,
            land_final_speed_ms: Q_LAND_FINAL_SPD_DEFAULT_MS,
            detect: LandDetectView::settled(now_ms, height_m),
            payload_place: false,
            in_auto: false,
            continue_after_land: false,
        }
    }
}

/// Upstream `AP_Math::linear_interpolate` (low/high output vs var).
#[must_use]
pub const fn linear_interpolate(
    low_out: f32,
    high_out: f32,
    var: f32,
    var_low: f32,
    var_high: f32,
) -> f32 {
    if var <= var_low {
        return low_out;
    }
    if var >= var_high {
        return high_out;
    }
    let span = var_high - var_low;
    low_out + (var - var_low) * (high_out - low_out) / span
}

/// `poscontrol.get_state() < QPOS_LAND_FINAL` (discriminant order).
#[must_use]
pub const fn pos_before_land_final(state: PositionControlState) -> bool {
    (state as u8) < (PositionControlState::LandFinal as u8)
}

/// Upstream `QuadPlane::landing_descent_rate_ms` (rate / land-final clamp).
///
/// Override-descent and `THR_LANDING_CONTROL` are later leftovers.
/// Pilot repositioning (`pilot_correction_active`) stops a positive
/// descent (`MIN(0, ret)`).
#[must_use]
pub fn landing_descent_rate_ms(qp: &QuadPlane, view: &QLandRunView) -> f32 {
    let mut height_m = view.height_above_ground_m;
    if qp.poscontrol().state() == PositionControlState::LandFinal
        && height_m > qp.land_final_alt_m()
    {
        height_m = qp.land_final_alt_m();
    }
    let mut ret_ms = linear_interpolate(
        view.land_final_speed_ms,
        view.wp_speed_down_ms,
        height_m,
        qp.land_final_alt_m(),
        qp.land_final_alt_m() + LAND_DESCENT_INTERP_SPAN_M,
    );
    if qp.poscontrol().pilot_correction_active() && ret_ms > 0.0 {
        ret_ms = 0.0;
    }
    ret_ms
}

/// Combined `ModeQLand::run`: always `mode_qloiter.run()`, then the
/// QLAND leftover (descent / land-final / land-complete).
///
/// QLoiter recovery, tailsitter FW pull-up, and `throttle_wait` return
/// before that leftover, matching the early-outs in
/// `ModeQLoiter::run`.
pub fn qland_run(qp: &mut QuadPlane, view: QLandRunView) -> QLandRun {
    if view.vtol_recovery {
        return QLandRun::early(QLandRunAction::VtolRecovery);
    }
    if view.tailsitter_in_vtol_transition {
        return QLandRun::early(QLandRunAction::FwControllers);
    }
    if qp.throttle_wait() {
        return QLandRun::early(QLandRunAction::ThrottleWait);
    }

    let mut switched_land_final = false;
    let mut target_position_setup = false;
    if pos_before_land_final(qp.poscontrol().state())
        && qp.check_land_final(LandFinalView {
            detect: view.detect,
            height_above_ground_m: view.height_above_ground_m,
        })
    {
        qp.poscontrol_mut()
            .set_state(PositionControlState::LandFinal);
        switched_land_final = true;
        target_position_setup = true;
    }

    let descent_rate_ms = landing_descent_rate_ms(qp, &view);
    let touchdown_expected = qp.poscontrol().state() == PositionControlState::LandFinal
        && !qp.option_is_set(QOption::DisableGroundEffectComp);
    let land_complete = qp.check_land_complete(LandCompleteView {
        detect: view.detect,
        payload_place: view.payload_place,
        in_auto: view.in_auto,
        continue_after_land: view.continue_after_land,
    });

    QLandRun {
        action: QLandRunAction::Descend,
        used_qloiter: true,
        switched_land_final,
        target_position_setup,
        descent_rate_ms,
        climb_rate_target_ms: -descent_rate_ms,
        touchdown_expected,
        land_complete,
    }
}
