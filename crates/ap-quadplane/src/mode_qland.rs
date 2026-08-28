//! QLOITER / QLAND / LoiterAltQLand `_enter` stub, upstream
//! `ArduPlane/mode_qloiter.cpp` / `mode_qland.cpp` /
//! `mode_LoiterAltQLand.cpp` (Plane-4.7.0).
//!
//! Tracked as **VT-005**. `Mode::enter` always calls
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
