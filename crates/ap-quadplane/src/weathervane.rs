//! QuadPlane weathervane and assist-handoff stub, upstream
//! `QuadPlane::get_weathervane_yaw_rate_cds` /
//! `QuadPlane::get_desired_yaw_rate_cds` and the
//! `SLT_Transition::update` latch that copies
//! `VTOL_Assist::should_assist` into `assisted_flight`
//! (Plane-4.7.0 `quadplane.cpp` / `AC_WeatherVane`).
//!
//! Tracked as **VT-001**. The `Q_WVANE_*` object lives on QuadPlane
//! (`AC_WeatherVane *weathervane`, allocated in `setup()`). VT-002 owns
//! the assist *decision*; this slice is the QuadPlane-side handoff
//! when that decision becomes active. The VT-003 FSM owns
//! `transition->allow_weathervane()`.

use crate::QuadPlane;

/// Plane `Q_WVANE_ENABLE` default, upstream `WVANE_PARAM_ENABLED` 1.
pub const WVANE_ENABLE_DEFAULT: i8 = 1;

/// Plane `Q_WVANE_GAIN` default, upstream `WVANE_PARAM_GAIN_DEFAULT` 0.
pub const WVANE_GAIN_DEFAULT: f32 = 0.0;

/// `Q_WVANE_ANG_MIN` default (deg), upstream `AP_GROUPINFO("ANG_MIN", ..., 1.0)`.
pub const WVANE_ANG_MIN_DEFAULT: f32 = 1.0;

/// `Q_WVANE_HGT_MIN` default (m).
pub const WVANE_HGT_MIN_DEFAULT: f32 = 0.0;

/// `Q_WVANE_TAKEOFF` / `Q_WVANE_LAND` default (no override).
pub const WVANE_DIR_OVERRIDE_DEFAULT: i8 = -1;

/// `Q_PLT_Y_RATE` default (deg/s), upstream `AC_CommandModel{100.0, ...}`.
pub const PILOT_YAW_RATE_DPS_DEFAULT: f32 = 100.0;

/// 2 s dwell before `AC_WeatherVane::get_yaw_out` produces a yaw.
pub const WVANE_ACTIVATE_MS: u32 = 2000;

/// Stale-run reset, upstream `now - last_check_ms > 250`.
pub const WVANE_STALE_MS: u32 = 250;

/// Scale from weathervane output to a fraction of the pilot rate.
pub const WVANE_OUTPUT_SCALE: f32 = 1.0 / 45.0;

/// Direction the airframe yaws into wind, upstream `AC_WeatherVane::Direction`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(i8)]
pub enum WeatherVaneDirection {
    /// `-1` — only during takeoff / landing (see TAKEOFF / LAND).
    TakeoffOrLandOnly = -1,
    /// `0` — disabled.
    Off = 0,
    /// `1` — nose into wind.
    NoseIn = 1,
    /// `2` — nose or tail, whichever is closer.
    NoseOrTailIn = 2,
    /// `3` — side into wind (copter tailsitters).
    SideIn = 3,
    /// `4` — tail into wind.
    TailIn = 4,
}

impl WeatherVaneDirection {
    /// Inverse of the upstream discriminant.
    #[must_use]
    pub const fn from_i8(value: i8) -> Option<Self> {
        match value {
            -1 => Some(Self::TakeoffOrLandOnly),
            0 => Some(Self::Off),
            1 => Some(Self::NoseIn),
            2 => Some(Self::NoseOrTailIn),
            3 => Some(Self::SideIn),
            4 => Some(Self::TailIn),
            _ => None,
        }
    }

    /// Upstream discriminant.
    #[must_use]
    pub const fn as_i8(self) -> i8 {
        self as i8
    }
}

/// `Q_WVANE_OPTIONS` bits, upstream `AC_WeatherVane::Options`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(i16)]
pub enum WeatherVaneOption {
    /// Bit 0: use pitch when nose- or tail-in.
    PitchEnable = 1 << 0,
}

/// Inputs [`QuadPlane::get_weathervane_yaw_rate_cds`] reads from Plane.
///
/// This crate does not own `transition`, motors, `pos_control`, or
/// the current flight mode.
#[derive(Clone, Copy, Debug)]
pub struct WeathervaneSample {
    /// `QuadPlane::in_vtol_mode()`.
    pub in_vtol_mode: bool,
    /// `transition->allow_weathervane()` (VT-003 / tailsitter override).
    pub allow_weathervane: bool,
    /// `motors->armed()`.
    pub motors_armed: bool,
    /// `motors->get_desired_spool_state() == THROTTLE_UNLIMITED`.
    pub throttle_unlimited: bool,
    /// `control_mode == mode_qstabilize`.
    pub qstabilize: bool,
    /// `control_mode == mode_qautotune`.
    pub qautotune: bool,
    /// `control_mode == mode_qhover`.
    pub qhover: bool,
    /// `QuadPlane::should_relax()`.
    pub should_relax: bool,
    /// `channel_rudder->get_control_in()`. Pilot yaw overrides weathervane.
    pub pilot_yaw: i16,
    /// Height used by `Q_WVANE_HGT_MIN` (m AGL / above home).
    pub height_m: f32,
    /// `pos_control->get_roll_cd()`.
    pub roll_cd: f32,
    /// `pos_control->get_pitch_cd()`.
    pub pitch_cd: f32,
    /// `in_vtol_auto() && is_vtol_takeoff(nav_cmd)`.
    pub is_takeoff: bool,
    /// `in_vtol_land_sequence()`.
    pub is_landing: bool,
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
}

impl Default for WeathervaneSample {
    fn default() -> Self {
        Self::new()
    }
}

impl WeathervaneSample {
    /// VTOL position-control, armed, unlimited spool — the open gate.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            in_vtol_mode: true,
            allow_weathervane: true,
            motors_armed: true,
            throttle_unlimited: true,
            qstabilize: false,
            qautotune: false,
            qhover: false,
            should_relax: false,
            pilot_yaw: 0,
            height_m: 10.0,
            roll_cd: 0.0,
            pitch_cd: 0.0,
            is_takeoff: false,
            is_landing: false,
            // 0 is the unset sentinel for `first_activate_ms`.
            now_ms: 1,
        }
    }

    /// Fixed-wing / not VTOL — weathervane must reset and return 0.
    #[must_use]
    pub const fn fixed_wing() -> Self {
        let mut s = Self::new();
        s.in_vtol_mode = false;
        s
    }
}

/// `AC_WeatherVane` parameter object, allocated by [`QuadPlane::setup`].
#[derive(Clone, Copy, Debug)]
pub struct WeatherVane {
    direction: i8,
    gain: f32,
    min_dz_ang_deg: f32,
    min_height: f32,
    takeoff_direction: i8,
    landing_direction: i8,
    options: i16,
    last_output: f32,
    first_activate_ms: u32,
    last_check_ms: u32,
    allowed: bool,
}

impl Default for WeatherVane {
    fn default() -> Self {
        Self::new()
    }
}

impl WeatherVane {
    /// Plane `Q_WVANE_*` defaults (`ENABLE` 1, `GAIN` 0, `ANG_MIN` 1).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            direction: WVANE_ENABLE_DEFAULT,
            gain: WVANE_GAIN_DEFAULT,
            min_dz_ang_deg: WVANE_ANG_MIN_DEFAULT,
            min_height: WVANE_HGT_MIN_DEFAULT,
            takeoff_direction: WVANE_DIR_OVERRIDE_DEFAULT,
            landing_direction: WVANE_DIR_OVERRIDE_DEFAULT,
            options: 0,
            last_output: 0.0,
            first_activate_ms: 0,
            last_check_ms: 0,
            allowed: true,
        }
    }

    /// `Q_WVANE_ENABLE`.
    #[must_use]
    pub const fn direction(&self) -> i8 {
        self.direction
    }

    /// Write `Q_WVANE_ENABLE`.
    pub fn set_direction(&mut self, direction: i8) {
        self.direction = direction;
    }

    /// `Q_WVANE_GAIN`.
    #[must_use]
    pub const fn gain(&self) -> f32 {
        self.gain
    }

    /// Write `Q_WVANE_GAIN`.
    pub fn set_gain(&mut self, gain: f32) {
        self.gain = gain;
    }

    /// `Q_WVANE_ANG_MIN` (deg).
    #[must_use]
    pub const fn min_dz_ang_deg(&self) -> f32 {
        self.min_dz_ang_deg
    }

    /// Write `Q_WVANE_ANG_MIN`.
    pub fn set_min_dz_ang_deg(&mut self, min_dz_ang_deg: f32) {
        self.min_dz_ang_deg = min_dz_ang_deg;
    }

    /// `Q_WVANE_HGT_MIN` (m).
    #[must_use]
    pub const fn min_height(&self) -> f32 {
        self.min_height
    }

    /// Write `Q_WVANE_HGT_MIN`.
    pub fn set_min_height(&mut self, min_height: f32) {
        self.min_height = min_height;
    }

    /// `Q_WVANE_TAKEOFF`.
    #[must_use]
    pub const fn takeoff_direction(&self) -> i8 {
        self.takeoff_direction
    }

    /// Write `Q_WVANE_TAKEOFF`.
    pub fn set_takeoff_direction(&mut self, takeoff_direction: i8) {
        self.takeoff_direction = takeoff_direction;
    }

    /// `Q_WVANE_LAND`.
    #[must_use]
    pub const fn landing_direction(&self) -> i8 {
        self.landing_direction
    }

    /// Write `Q_WVANE_LAND`.
    pub fn set_landing_direction(&mut self, landing_direction: i8) {
        self.landing_direction = landing_direction;
    }

    /// `Q_WVANE_OPTIONS`.
    #[must_use]
    pub const fn options(&self) -> i16 {
        self.options
    }

    /// Write `Q_WVANE_OPTIONS`.
    pub fn set_options(&mut self, options: i16) {
        self.options = options;
    }

    /// Whether something other than the parameter may weathervane.
    #[must_use]
    pub const fn allowed(&self) -> bool {
        self.allowed
    }

    /// Upstream `AC_WeatherVane::allow_weathervaning`.
    pub fn allow_weathervaning(&mut self, allow: bool) {
        self.allowed = allow;
    }

    /// Last slewed output (centideg-scale, before the QuadPlane rate map).
    #[must_use]
    pub const fn last_output(&self) -> f32 {
        self.last_output
    }

    /// Upstream `AC_WeatherVane::reset`.
    pub fn reset(&mut self, now_ms: u32) {
        self.last_output = 0.0;
        self.first_activate_ms = 0;
        self.last_check_ms = now_ms;
    }

    /// Upstream `AC_WeatherVane::get_yaw_out`.
    ///
    /// Returns `Some(yaw_output)` when the controller is active after
    /// the 2 s dwell. `None` means the caller must treat the rate as 0
    /// (and the controller has been reset when a gate failed).
    pub fn get_yaw_out(
        &mut self,
        pilot_yaw: i16,
        height_m: f32,
        roll_cd: f32,
        pitch_cd: f32,
        is_takeoff: bool,
        is_landing: bool,
        now_ms: u32,
    ) -> Option<f32> {
        let mut dir = self.direction;
        if dir == WeatherVaneDirection::Off as i8
            || !self.allowed
            || pilot_yaw != 0
            || self.gain <= 0.0
        {
            self.reset(now_ms);
            return None;
        }
        if is_takeoff && self.takeoff_direction >= 0 {
            dir = self.takeoff_direction;
        }
        if is_landing && self.landing_direction >= 0 {
            dir = self.landing_direction;
        }
        if dir == WeatherVaneDirection::Off as i8
            || dir == WeatherVaneDirection::TakeoffOrLandOnly as i8
        {
            self.reset(now_ms);
            return None;
        }
        if self.min_height > 0.0 && height_m <= self.min_height {
            self.reset(now_ms);
            return None;
        }

        if now_ms.wrapping_sub(self.last_check_ms) > WVANE_STALE_MS && self.last_check_ms != 0 {
            // Not run recently — restart the 2 s buffer.
            self.reset(now_ms);
        }
        self.last_check_ms = now_ms;

        if self.first_activate_ms == 0 {
            self.first_activate_ms = now_ms;
        }
        if now_ms.wrapping_sub(self.first_activate_ms) < WVANE_ACTIVATE_MS {
            return None;
        }

        let deadzone_cdeg = self.min_dz_ang_deg * 100.0;
        let pitch_enable = (self.options & WeatherVaneOption::PitchEnable as i16) != 0;
        let output = match WeatherVaneDirection::from_i8(dir) {
            Some(WeatherVaneDirection::NoseIn) => {
                let raw = if pitch_enable && pitch_cd - deadzone_cdeg > 0.0 {
                    abs_f32(roll_cd) + (pitch_cd - deadzone_cdeg)
                } else {
                    max_f32(abs_f32(roll_cd) - deadzone_cdeg, 0.0)
                };
                if roll_cd < 0.0 {
                    -raw
                } else {
                    raw
                }
            }
            Some(WeatherVaneDirection::NoseOrTailIn) => {
                let raw = max_f32(abs_f32(roll_cd) - deadzone_cdeg, 0.0);
                if (roll_cd < 0.0) != (pitch_cd > 0.0) {
                    -raw
                } else {
                    raw
                }
            }
            Some(WeatherVaneDirection::SideIn) => {
                let raw = max_f32(abs_f32(pitch_cd) - deadzone_cdeg, 0.0);
                if (pitch_cd > 0.0) != (roll_cd > 0.0) {
                    -raw
                } else {
                    raw
                }
            }
            Some(WeatherVaneDirection::TailIn) => {
                let raw = if pitch_enable && pitch_cd + deadzone_cdeg < 0.0 {
                    abs_f32(roll_cd) - (pitch_cd + deadzone_cdeg)
                } else {
                    max_f32(abs_f32(roll_cd) - deadzone_cdeg, 0.0)
                };
                if roll_cd > 0.0 {
                    -raw
                } else {
                    raw
                }
            }
            Some(WeatherVaneDirection::Off | WeatherVaneDirection::TakeoffOrLandOnly) | None => {
                self.reset(now_ms);
                return None;
            }
        };

        self.last_output = 0.98 * self.last_output + 0.02 * output * self.gain;
        Some(self.last_output)
    }
}

impl QuadPlane {
    /// Whether [`Self::setup`] constructed the weathervane object.
    ///
    /// Upstream `weathervane != nullptr` after a successful setup.
    #[must_use]
    pub const fn weathervane_inited(&self) -> bool {
        self.weathervane_inited
    }

    /// The `Q_WVANE_*` object.
    #[must_use]
    pub const fn weathervane(&self) -> &WeatherVane {
        &self.weathervane
    }

    /// Mutable `Q_WVANE_*` object (parameter poke / tests).
    pub fn weathervane_mut(&mut self) -> &mut WeatherVane {
        &mut self.weathervane
    }

    /// Upstream `QuadPlane::get_weathervane_yaw_rate_cds`.
    ///
    /// Zero (and a controller reset) unless we are in a VTOL position
    /// mode, the transition allows weathervane, motors are armed and
    /// unlimited, and the mode is not QStabilize / QAutotune / QHover
    /// / relax. Otherwise maps `get_yaw_out` through
    /// `constrain(out/45, ±100) * Q_PLT_Y_RATE * 0.5`.
    pub fn get_weathervane_yaw_rate_cds(&mut self, sample: &WeathervaneSample) -> f32 {
        if !self.weathervane_inited
            || !sample.in_vtol_mode
            || !sample.allow_weathervane
            || !sample.motors_armed
            || !sample.throttle_unlimited
            || sample.qstabilize
            || sample.qautotune
            || sample.qhover
            || sample.should_relax
        {
            self.weathervane.reset(sample.now_ms);
            return 0.0;
        }
        match self.weathervane.get_yaw_out(
            sample.pilot_yaw,
            sample.height_m,
            sample.roll_cd,
            sample.pitch_cd,
            sample.is_takeoff,
            sample.is_landing,
            sample.now_ms,
        ) {
            Some(wv_output) => {
                constrain_f32(wv_output * WVANE_OUTPUT_SCALE, -100.0, 100.0)
                    * PILOT_YAW_RATE_DPS_DEFAULT
                    * 0.5
            }
            None => 0.0,
        }
    }

    /// Upstream `QuadPlane::get_desired_yaw_rate_cds`.
    ///
    /// Adds coordinated-turn yaw while [`Self::assisted_flight`], then
    /// pilot yaw, then weathervane when `should_weathervane` is set.
    /// `desired_auto_yaw_rate_cds` and `get_pilot_input_yaw_rate_cds`
    /// live on Plane; the caller passes those rates.
    pub fn get_desired_yaw_rate_cds(
        &mut self,
        should_weathervane: bool,
        pilot_yaw_rate_cds: f32,
        auto_yaw_rate_cds: f32,
        sample: &WeathervaneSample,
    ) -> f32 {
        let mut yaw_cds = 0.0;
        if self.assisted_flight {
            yaw_cds += auto_yaw_rate_cds;
        }
        yaw_cds += pilot_yaw_rate_cds;
        if should_weathervane {
            yaw_cds += self.get_weathervane_yaw_rate_cds(sample);
        }
        yaw_cds
    }

    /// Handoff of `VTOL_Assist::should_assist` into `assisted_flight`.
    ///
    /// Upstream `SLT_Transition::update`:
    /// `if (assist.should_assist(...)) assisted_flight = true; else
    /// assisted_flight = false`. VT-002 owns the decision; this is
    /// the QuadPlane latch that becomes active when assist does.
    ///
    /// Returns [`Self::in_assisted_flight`] after the write.
    pub fn apply_assist_handoff(&mut self, should_assist: bool) -> bool {
        self.assisted_flight = should_assist;
        self.in_assisted_flight()
    }
}

const fn abs_f32(v: f32) -> f32 {
    if v < 0.0 {
        -v
    } else {
        v
    }
}

const fn max_f32(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}

const fn constrain_f32(v: f32, min: f32, max: f32) -> f32 {
    if v < min {
        min
    } else if v > max {
        max
    } else {
        v
    }
}
