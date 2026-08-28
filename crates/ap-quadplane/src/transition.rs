//! Tailsitter transition pitch / throttle ramp stub.
//!
//! Tracked as **VT-007**. Upstream `Tailsitter_Transition` ramps pitch
//! between hover and forward-flight, and `Tailsitter::output` picks the
//! throttle that rides along with that pitch change.
//!
//! # Pitch
//!
//! VTOL → FW (`ANGLE_WAIT_FW`) subtracts `Q_TAILSIT_RAT_FW` (deg/s) from
//! the starting pitch. FW → VTOL (`set_FW_roll_pitch` while
//! `in_vtol_transition`) adds `Q_TAILSIT_RAT_VT`. Both clamp to
//! ±[`PITCH_CD_LIMIT`]. The angle that ends the wait is
//! [`Q_TAILSIT_ANGLE`](TRANSITION_ANGLE_FW_DEFAULT) for FW, and
//! [`get_transition_angle_vtol`](TransitionRamp::get_transition_angle_vtol)
//! for VTOL (`Q_TAILSIT_ANG_VT`, or ANGLE when that param is zero).
//!
//! The `* 0.1` in upstream is `(deg/s) * (ms) * 0.1 = centidegrees`.
//!
//! # Throttle
//!
//! During FW → VTOL, a non-negative `Q_TAILSIT_THR_VT` is used as a
//! 0..1 demand (`MIN(THR_VT * 0.01, 1)`). `-1` (the default) takes the
//! greater of hover throttle and cruise. During VTOL → FW the demand is
//! `MAX(hover, current)`.
//!
//! `Q_TAILSIT_GSCMAX` (older name `THSCMX`) is the ceiling of the
//! hover/throttle gain scaler. The ATT_THR slew (`posTC` / `negTC`) and
//! the rest of `speed_scaling` are later slices.

/// Default `Q_TAILSIT_ANGLE`, upstream `AP_GROUPINFO("ANGLE", ..., 45)`.
pub const TRANSITION_ANGLE_FW_DEFAULT: i8 = 45;

/// Default `Q_TAILSIT_ANG_VT`, upstream `AP_GROUPINFO("ANG_VT", ..., 0)`.
pub const TRANSITION_ANGLE_VTOL_DEFAULT: i8 = 0;

/// Default `Q_TAILSIT_RAT_FW` (deg/s), upstream `AP_GROUPINFO("RAT_FW", ..., 50)`.
pub const TRANSITION_RATE_FW_DEFAULT: f32 = 50.0;

/// Default `Q_TAILSIT_RAT_VT` (deg/s), upstream `AP_GROUPINFO("RAT_VT", ..., 50)`.
pub const TRANSITION_RATE_VTOL_DEFAULT: f32 = 50.0;

/// Default `Q_TAILSIT_THR_VT` (%). `-1` means hover (MAX cruise).
pub const TRANSITION_THROTTLE_VTOL_DEFAULT: f32 = -1.0;

/// Default `Q_TAILSIT_GSCMAX` (older name `THSCMX`).
pub const THROTTLE_SCALE_MAX_DEFAULT: f32 = 2.0;

/// Default `Q_TAILSIT_GSCMIN`.
pub const GAIN_SCALING_MIN_DEFAULT: f32 = 0.4;

/// Upstream `constrain_float(..., -8500, 8500)` on the pitch demand.
pub const PITCH_CD_LIMIT: i32 = 8500;

/// Which way the airframe is pitching through the transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransitionKind {
    /// VTOL → fixed-wing. Upstream `State::ANGLE_WAIT_FW`.
    ToFw,
    /// Fixed-wing → VTOL. Upstream `State::ANGLE_WAIT_VTOL`.
    ToVtol,
}

/// Pitch / throttle ramp parameters, upstream `Tailsitter` transition fields.
#[derive(Debug, Clone, Copy)]
pub struct TransitionRamp {
    angle_fw: i8,
    angle_vtol: i8,
    rate_fw: f32,
    rate_vtol: f32,
    throttle_vtol: f32,
    throttle_scale_max: f32,
    gain_scaling_min: f32,
}

impl Default for TransitionRamp {
    fn default() -> Self {
        Self::new()
    }
}

impl TransitionRamp {
    /// `AP_GROUPINFO` defaults for ANGLE / ANG_VT / RAT_* / THR_VT / GSCMAX.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            angle_fw: TRANSITION_ANGLE_FW_DEFAULT,
            angle_vtol: TRANSITION_ANGLE_VTOL_DEFAULT,
            rate_fw: TRANSITION_RATE_FW_DEFAULT,
            rate_vtol: TRANSITION_RATE_VTOL_DEFAULT,
            throttle_vtol: TRANSITION_THROTTLE_VTOL_DEFAULT,
            throttle_scale_max: THROTTLE_SCALE_MAX_DEFAULT,
            gain_scaling_min: GAIN_SCALING_MIN_DEFAULT,
        }
    }

    /// `Q_TAILSIT_ANGLE` (deg).
    #[must_use]
    pub const fn angle_fw(&self) -> i8 {
        self.angle_fw
    }

    /// `Q_TAILSIT_ANG_VT` (deg). Zero means "use ANGLE".
    #[must_use]
    pub const fn angle_vtol(&self) -> i8 {
        self.angle_vtol
    }

    /// `Q_TAILSIT_RAT_FW` (deg/s).
    #[must_use]
    pub const fn rate_fw(&self) -> f32 {
        self.rate_fw
    }

    /// `Q_TAILSIT_RAT_VT` (deg/s).
    #[must_use]
    pub const fn rate_vtol(&self) -> f32 {
        self.rate_vtol
    }

    /// `Q_TAILSIT_THR_VT` (%).
    #[must_use]
    pub const fn throttle_vtol(&self) -> f32 {
        self.throttle_vtol
    }

    /// `Q_TAILSIT_GSCMAX` / older `THSCMX`.
    #[must_use]
    pub const fn throttle_scale_max(&self) -> f32 {
        self.throttle_scale_max
    }

    /// `Q_TAILSIT_GSCMIN`.
    #[must_use]
    pub const fn gain_scaling_min(&self) -> f32 {
        self.gain_scaling_min
    }

    /// Poke `Q_TAILSIT_ANGLE`.
    pub fn set_angle_fw(&mut self, angle: i8) {
        self.angle_fw = angle;
    }

    /// Poke `Q_TAILSIT_ANG_VT`.
    pub fn set_angle_vtol(&mut self, angle: i8) {
        self.angle_vtol = angle;
    }

    /// Poke `Q_TAILSIT_RAT_FW`.
    pub fn set_rate_fw(&mut self, rate: f32) {
        self.rate_fw = rate;
    }

    /// Poke `Q_TAILSIT_RAT_VT`.
    pub fn set_rate_vtol(&mut self, rate: f32) {
        self.rate_vtol = rate;
    }

    /// Poke `Q_TAILSIT_THR_VT`.
    pub fn set_throttle_vtol(&mut self, throttle: f32) {
        self.throttle_vtol = throttle;
    }

    /// Poke `Q_TAILSIT_GSCMAX`.
    pub fn set_throttle_scale_max(&mut self, max: f32) {
        self.throttle_scale_max = max;
    }

    /// Poke `Q_TAILSIT_GSCMIN`.
    pub fn set_gain_scaling_min(&mut self, min: f32) {
        self.gain_scaling_min = min;
    }

    /// Upstream `Tailsitter::get_transition_angle_vtol`.
    ///
    /// Returns `Q_TAILSIT_ANG_VT` when that param is non-zero, otherwise
    /// `Q_TAILSIT_ANGLE`.
    #[must_use]
    pub const fn get_transition_angle_vtol(&self) -> i8 {
        if self.angle_vtol == 0 {
            self.angle_fw
        } else {
            self.angle_vtol
        }
    }

    /// Commanded pitch (centidegrees) after `dt_ms` of transition.
    ///
    /// VTOL → FW: `initial - rate_fw * dt * 0.1` (sign flipped when
    /// inverted). FW → VTOL: `initial + rate_vtol * dt * 0.1`.
    /// Clamped to ±[`PITCH_CD_LIMIT`].
    #[must_use]
    pub fn pitch_cd(
        &self,
        kind: TransitionKind,
        initial_pitch_cd: f32,
        dt_ms: u32,
        inverted: bool,
    ) -> i32 {
        let raw = match kind {
            TransitionKind::ToFw => {
                let sign = if inverted { -1.0 } else { 1.0 };
                initial_pitch_cd - pitch_delta_cd(self.rate_fw, dt_ms) * sign
            }
            TransitionKind::ToVtol => initial_pitch_cd + pitch_delta_cd(self.rate_vtol, dt_ms),
        };
        constrain_pitch_cd(raw)
    }

    /// Angle half of `transition_fw_complete` /
    /// `transition_vtol_complete`: `|pitch_cd| > angle * 100`.
    ///
    /// Roll-error and the 1.5× timeout are later slices.
    #[must_use]
    pub fn angle_complete(&self, kind: TransitionKind, pitch_cd: i32) -> bool {
        let angle = match kind {
            TransitionKind::ToFw => self.angle_fw,
            TransitionKind::ToVtol => self.get_transition_angle_vtol(),
        };
        pitch_cd.unsigned_abs() > u32::from(angle.unsigned_abs()) * 100
    }

    /// Throttle demand (0..1) that rides with the pitch ramp.
    ///
    /// - [`TransitionKind::ToVtol`]: `Q_TAILSIT_THR_VT` when
    ///   `!is_negative`, else `MAX(hover, cruise_pct * 0.01)`.
    /// - [`TransitionKind::ToFw`]: `MAX(hover, current)`.
    ///
    /// `actuator_to_thrust` / battery expo are not applied here.
    #[must_use]
    pub fn throttle(&self, kind: TransitionKind, hover: f32, cruise_pct: f32, current: f32) -> f32 {
        match kind {
            TransitionKind::ToVtol => {
                if is_negative(self.throttle_vtol) {
                    hover.max(cruise_pct * 0.01)
                } else {
                    (self.throttle_vtol * 0.01).min(1.0)
                }
            }
            TransitionKind::ToFw => hover.max(current),
        }
    }

    /// Hover/throttle gain scaler, upstream `speed_scaling` throttle path.
    ///
    /// `throttle_scale_max` when throttle is not positive; otherwise
    /// `constrain(hover / throttle, GSCMIN, GSCMAX)`.
    #[must_use]
    pub fn throttle_scaler(&self, hover: f32, throttle: f32) -> f32 {
        if is_positive(throttle) {
            constrain_f32(
                hover / throttle,
                self.gain_scaling_min,
                self.throttle_scale_max,
            )
        } else {
            self.throttle_scale_max
        }
    }
}

/// `(deg/s) * (ms) * 0.1` → centidegrees. Upstream comment on `update`.
fn pitch_delta_cd(rate_deg_s: f32, dt_ms: u32) -> f32 {
    rate_deg_s * dt_ms as f32 * 0.1
}

fn constrain_pitch_cd(v: f32) -> i32 {
    // C++ float-to-int32 truncates toward zero after constrain_float.
    constrain_f32(v, -PITCH_CD_LIMIT as f32, PITCH_CD_LIMIT as f32) as i32
}

fn constrain_f32(v: f32, min: f32, max: f32) -> f32 {
    v.clamp(min, max)
}

/// Upstream `is_negative` for the float params.
fn is_negative(v: f32) -> bool {
    v < 0.0
}

/// Upstream `is_positive` for the float params.
fn is_positive(v: f32) -> bool {
    v > 0.0
}
