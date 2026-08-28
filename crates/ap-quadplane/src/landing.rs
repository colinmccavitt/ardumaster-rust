//! QuadPlane landing-detect and GUIDED user-takeoff, upstream
//! `QuadPlane::should_relax` / `land_detector` / `check_land_complete`
//! / `check_land_final` / `do_user_takeoff` (Plane-4.7.0 `quadplane.cpp`).
//!
//! Tracked as **VT-001**. The leftover `landing_detect` block times
//! motors-at-lower-limit (`should_relax`), then requires a stable
//! inertial height for `timeout_ms` (`land_detector`). Complete / final
//! land use that detector; GUIDED [`QuadPlane::do_user_takeoff`] is the
//! MAVLink user-takeoff leftover. This is not a rewrite of motor-test
//! or throttle mix.

use crate::air_mode::QOption;
use crate::poscontrol::PositionControlState;
use crate::QuadPlane;

/// Default `Q_LAND_ALTCHG`, upstream `AP_GROUPINFO("LAND_ALTCHG", ..., 0.2)`.
pub const Q_LAND_ALTCHG_DEFAULT_M: f32 = 0.2;

/// Default `Q_LAND_FINAL_ALT`, upstream `AP_GROUPINFO("LAND_FINAL_ALT", ..., 6)`.
pub const Q_LAND_FINAL_ALT_DEFAULT_M: f32 = 6.0;

/// `should_relax` dwell, upstream `> 1000` ms at lower limit.
pub const LAND_RELAX_MS: u32 = 1000;

/// `check_land_complete` detector timeout, upstream `land_detector(4000)`.
pub const LAND_COMPLETE_TIMEOUT_MS: u32 = 4000;

/// Extra lower-limit dwell on top of the detector timeout.
///
/// Upstream `(now - lower_limit_start_ms) < (timeout_ms+1000)`.
pub const LAND_LOWER_LIMIT_EXTRA_MS: u32 = 1000;

/// `check_land_final` detector timeout, upstream `land_detector(6000)`.
pub const LAND_FINAL_TIMEOUT_MS: u32 = 6000;

/// Max AGL change that still allows the height-based land-final switch.
///
/// Upstream `const float max_change_m = 5`.
pub const LAND_FINAL_MAX_CHANGE_M: f32 = 5.0;

/// Throttle treated as "at lower limit", upstream `get_throttle() < 0.01f`.
pub const LAND_THROTTLE_LOWER_EPS: f32 = 0.01;

/// `Q_OPTIONS` bit 13, upstream `QuadPlane::Option::DISABLE_GROUND_EFFECT_COMP`.
pub const Q_OPTIONS_DISABLE_GROUND_EFFECT_COMP: i32 = 1 << 13;

/// Motors / mix inputs [`QuadPlane::should_relax`] reads each tick.
///
/// This crate does not own `AP_Motors` or `AC_AttitudeControl`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RelaxView {
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
    /// `motors->get_throttle()` (0..1).
    pub throttle: f32,
    /// `motors->limit.throttle_lower`.
    pub throttle_lower: bool,
    /// `attitude_control->is_throttle_mix_min()`.
    pub throttle_mix_min: bool,
}

impl RelaxView {
    /// Motors demanding thrust, not at the lower limit.
    #[must_use]
    pub const fn flying() -> Self {
        Self {
            now_ms: 0,
            throttle: 0.5,
            throttle_lower: false,
            throttle_mix_min: false,
        }
    }

    /// Motors at the lower limit with min mix (landed-looking).
    #[must_use]
    pub const fn lower_limit(now_ms: u32) -> Self {
        Self {
            now_ms,
            throttle: 0.0,
            throttle_lower: true,
            throttle_mix_min: true,
        }
    }
}

/// Inputs for [`QuadPlane::land_detector`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LandDetectView {
    /// Motors / mix snapshot (also drives [`QuadPlane::should_relax`]).
    pub relax: RelaxView,
    /// `inertial_nav.get_position_z_up_cm() * 0.01`.
    pub height_m: f32,
}

impl LandDetectView {
    /// Lower-limit motors at a fixed height.
    #[must_use]
    pub const fn settled(now_ms: u32, height_m: f32) -> Self {
        Self {
            relax: RelaxView::lower_limit(now_ms),
            height_m,
        }
    }
}

/// Inputs for [`QuadPlane::check_land_complete`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LandCompleteView {
    /// Detector snapshot (`should_relax` + height).
    pub detect: LandDetectView,
    /// `plane.in_auto_mission_id(MAV_CMD_NAV_PAYLOAD_PLACE)`.
    pub payload_place: bool,
    /// `control_mode == mode_auto`.
    pub in_auto: bool,
    /// `mission.continue_after_land()`.
    pub continue_after_land: bool,
}

impl LandCompleteView {
    /// Final-land detector tick, not payload-place, not AUTO continue.
    #[must_use]
    pub const fn qland(now_ms: u32, height_m: f32) -> Self {
        Self {
            detect: LandDetectView::settled(now_ms, height_m),
            payload_place: false,
            in_auto: false,
            continue_after_land: false,
        }
    }
}

/// Side-effects of [`QuadPlane::check_land_complete`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LandCompleteResult {
    /// The C++ method returned `true`.
    pub complete: bool,
    /// `poscontrol` was moved to `QPOS_LAND_COMPLETE`.
    pub state_complete: bool,
    /// Payload-place shut motors down and returned `false`.
    pub spool_shutdown: bool,
    /// Disarm-on-land (not AUTO + `continue_after_land`).
    pub disarm: bool,
}

impl LandCompleteResult {
    /// No land-complete action.
    #[must_use]
    pub const fn idle() -> Self {
        Self {
            complete: false,
            state_complete: false,
            spool_shutdown: false,
            disarm: false,
        }
    }
}

/// Inputs for [`QuadPlane::check_land_final`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LandFinalView {
    /// Detector snapshot used when AGL is not yet stable / below final.
    pub detect: LandDetectView,
    /// `plane.relative_ground_altitude(RangeFinderUse::TAKEOFF_LANDING)`.
    pub height_above_ground_m: f32,
}

/// Inputs for [`QuadPlane::do_user_takeoff`].
///
/// Plane owns mode / arming / `is_flying()`; this crate does not.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UserTakeoffView {
    /// `control_mode == mode_guided`.
    pub in_guided: bool,
    /// `arming.is_armed_and_safety_off()`.
    pub armed_and_safety_off: bool,
    /// `QuadPlane` / Plane `is_flying()`.
    pub is_flying: bool,
    /// Requested climb (`takeoff_altitude`), metres.
    pub takeoff_altitude_m: f32,
}

impl UserTakeoffView {
    /// Armed GUIDED, still on the ground.
    #[must_use]
    pub const fn armed_guided(takeoff_altitude_m: f32) -> Self {
        Self {
            in_guided: true,
            armed_and_safety_off: true,
            is_flying: false,
            takeoff_altitude_m,
        }
    }
}

/// Outcome of [`QuadPlane::do_user_takeoff`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UserTakeoffResult {
    /// The C++ method returned `true`.
    pub accepted: bool,
    /// Write `auto_state.vtol_loiter = true` on accept.
    pub vtol_loiter: bool,
    /// `ahrs.set_takeoff_expected(true)` unless `DISABLE_GROUND_EFFECT_COMP`.
    pub takeoff_expected: bool,
    /// Climb copied onto `next_WP_loc` (`offset_up_m`).
    pub climb_m: f32,
}

impl UserTakeoffResult {
    /// Rejected takeoff — no AUTO / AHRS writes.
    #[must_use]
    pub const fn rejected() -> Self {
        Self {
            accepted: false,
            vtol_loiter: false,
            takeoff_expected: false,
            climb_m: 0.0,
        }
    }
}

/// Upstream `QuadPlane::landing_detect` block.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LandingDetect {
    /// `lower_limit_start_ms` — 0 means not at the lower limit.
    lower_limit_start_ms: u32,
    /// `land_start_ms` — 0 means the altitude dwell has not started.
    land_start_ms: u32,
    /// `vpos_start_m` latched when `land_start_ms` is armed.
    vpos_start_m: f32,
    /// `Q_LAND_ALTCHG` / `detect_alt_change_m`.
    detect_alt_change_m: f32,
}

impl LandingDetect {
    /// Zero timers, default `Q_LAND_ALTCHG`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            lower_limit_start_ms: 0,
            land_start_ms: 0,
            vpos_start_m: 0.0,
            detect_alt_change_m: Q_LAND_ALTCHG_DEFAULT_M,
        }
    }

    /// `landing_detect.lower_limit_start_ms`.
    #[must_use]
    pub const fn lower_limit_start_ms(&self) -> u32 {
        self.lower_limit_start_ms
    }

    /// `landing_detect.land_start_ms`.
    #[must_use]
    pub const fn land_start_ms(&self) -> u32 {
        self.land_start_ms
    }

    /// `landing_detect.vpos_start_m`.
    #[must_use]
    pub const fn vpos_start_m(&self) -> f32 {
        self.vpos_start_m
    }

    /// `Q_LAND_ALTCHG`.
    #[must_use]
    pub const fn detect_alt_change_m(&self) -> f32 {
        self.detect_alt_change_m
    }

    /// Write `Q_LAND_ALTCHG` (tests / parameter poke).
    pub fn set_detect_alt_change_m(&mut self, detect_alt_change_m: f32) {
        self.detect_alt_change_m = detect_alt_change_m;
    }

    /// Zero land / lower-limit timers (`do_vtol_land` / `QPOS_LAND_FINAL`).
    pub fn clear_land_timers(&mut self) {
        self.lower_limit_start_ms = 0;
        self.land_start_ms = 0;
    }
}

impl Default for LandingDetect {
    fn default() -> Self {
        Self::new()
    }
}

impl QuadPlane {
    /// Upstream `landing_detect` block.
    #[must_use]
    pub const fn landing_detect(&self) -> &LandingDetect {
        &self.landing_detect
    }

    /// Mutable `landing_detect` (parameter poke / tests).
    pub fn landing_detect_mut(&mut self) -> &mut LandingDetect {
        &mut self.landing_detect
    }

    /// `Q_LAND_FINAL_ALT`.
    #[must_use]
    pub const fn land_final_alt_m(&self) -> f32 {
        self.land_final_alt_m
    }

    /// Write `Q_LAND_FINAL_ALT`.
    pub fn set_land_final_alt_m(&mut self, land_final_alt_m: f32) {
        self.land_final_alt_m = land_final_alt_m;
    }

    /// `last_land_final_agl_m` (height-glitch filter).
    #[must_use]
    pub const fn last_land_final_agl_m(&self) -> f32 {
        self.last_land_final_agl_m
    }

    /// `guided_takeoff` latch set by [`Self::do_user_takeoff`].
    #[must_use]
    pub const fn guided_takeoff(&self) -> bool {
        self.guided_takeoff
    }

    /// Write `guided_takeoff` (tests / mode-enter leftover).
    pub fn set_guided_takeoff(&mut self, guided_takeoff: bool) {
        self.guided_takeoff = guided_takeoff;
    }

    /// Upstream `QuadPlane::should_relax`.
    ///
    /// Clears both land timers when the motors are not at the lower
    /// limit. Otherwise latches `lower_limit_start_ms` and returns
    /// true after [`LAND_RELAX_MS`].
    pub fn should_relax(&mut self, view: RelaxView) -> bool {
        let mut motor_at_lower_limit = view.throttle_lower && view.throttle_mix_min;
        if view.throttle < LAND_THROTTLE_LOWER_EPS {
            motor_at_lower_limit = true;
        }
        if !motor_at_lower_limit {
            self.landing_detect.lower_limit_start_ms = 0;
            self.landing_detect.land_start_ms = 0;
            return false;
        }
        if self.landing_detect.lower_limit_start_ms == 0 {
            self.landing_detect.lower_limit_start_ms = view.now_ms;
        }
        view.now_ms
            .wrapping_sub(self.landing_detect.lower_limit_start_ms)
            > LAND_RELAX_MS
    }

    /// Upstream `QuadPlane::land_detector`.
    ///
    /// Requires [`Self::should_relax`] and no pilot correction. Height
    /// must stay inside `Q_LAND_ALTCHG` for `timeout_ms`, and the
    /// motors must have been at the lower limit for
    /// `timeout_ms + `[`LAND_LOWER_LIMIT_EXTRA_MS`].
    pub fn land_detector(&mut self, view: LandDetectView, timeout_ms: u32) -> bool {
        let might_be_landed =
            self.should_relax(view.relax) && !self.poscontrol.pilot_correction_active();
        if !might_be_landed {
            self.landing_detect.land_start_ms = 0;
            return false;
        }
        if self.landing_detect.land_start_ms == 0 {
            self.landing_detect.land_start_ms = view.relax.now_ms;
            self.landing_detect.vpos_start_m = view.height_m;
        }
        if abs_f32(view.height_m - self.landing_detect.vpos_start_m)
            > self.landing_detect.detect_alt_change_m
        {
            self.landing_detect.land_start_ms = 0;
            return false;
        }
        if view
            .relax
            .now_ms
            .wrapping_sub(self.landing_detect.land_start_ms)
            < timeout_ms
            || view
                .relax
                .now_ms
                .wrapping_sub(self.landing_detect.lower_limit_start_ms)
                < timeout_ms.saturating_add(LAND_LOWER_LIMIT_EXTRA_MS)
        {
            return false;
        }
        true
    }

    /// Upstream `QuadPlane::check_land_complete`.
    ///
    /// Only runs in `QPOS_LAND_FINAL`. A successful detector moves
    /// poscontrol to `QPOS_LAND_COMPLETE`. Payload-place shuts the
    /// motors down and returns false; otherwise disarm unless AUTO is
    /// set to continue after land.
    pub fn check_land_complete(&mut self, view: LandCompleteView) -> LandCompleteResult {
        if self.poscontrol.state() != PositionControlState::LandFinal {
            return LandCompleteResult::idle();
        }
        if !self.land_detector(view.detect, LAND_COMPLETE_TIMEOUT_MS) {
            return LandCompleteResult::idle();
        }
        self.poscontrol
            .set_state(PositionControlState::LandComplete);
        if view.payload_place {
            return LandCompleteResult {
                complete: false,
                state_complete: true,
                spool_shutdown: true,
                disarm: false,
            };
        }
        let disarm = !view.in_auto || !view.continue_after_land;
        LandCompleteResult {
            complete: true,
            state_complete: true,
            spool_shutdown: false,
            disarm,
        }
    }

    /// Upstream `QuadPlane::check_land_final`.
    ///
    /// Switches to land-final when AGL is below `Q_LAND_FINAL_ALT` and
    /// within [`LAND_FINAL_MAX_CHANGE_M`] of the previous sample.
    /// Otherwise updates `last_land_final_agl_m` and falls through to
    /// [`Self::land_detector`] at [`LAND_FINAL_TIMEOUT_MS`].
    pub fn check_land_final(&mut self, view: LandFinalView) -> bool {
        let height_above_ground_m = view.height_above_ground_m;
        if height_above_ground_m < self.land_final_alt_m
            && abs_f32(height_above_ground_m - self.last_land_final_agl_m) < LAND_FINAL_MAX_CHANGE_M
        {
            return true;
        }
        self.last_land_final_agl_m = height_above_ground_m;
        self.land_detector(view.detect, LAND_FINAL_TIMEOUT_MS)
    }

    /// Upstream `QuadPlane::do_user_takeoff`.
    ///
    /// GUIDED + armed + not flying only. Accept sets `guided_takeoff`,
    /// clears `guided_wait_takeoff`, and asks the caller to write
    /// `auto_state.vtol_loiter` / `ahrs.set_takeoff_expected`.
    pub fn do_user_takeoff(&mut self, view: UserTakeoffView) -> UserTakeoffResult {
        if !view.in_guided || !view.armed_and_safety_off || view.is_flying {
            return UserTakeoffResult::rejected();
        }
        self.guided_takeoff = true;
        self.guided_wait_takeoff = false;
        let takeoff_expected = !self.option_is_set(QOption::DisableGroundEffectComp);
        UserTakeoffResult {
            accepted: true,
            vtol_loiter: true,
            takeoff_expected,
            climb_m: view.takeoff_altitude_m,
        }
    }
}

const fn abs_f32(v: f32) -> f32 {
    if v < 0.0 {
        -v
    } else {
        v
    }
}
