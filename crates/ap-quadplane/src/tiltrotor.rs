//! Tiltrotor enable and type stub, upstream `ArduPlane/tiltrotor.*`.
//!
//! Tracked as **VT-008**. This slice is the gate that decides whether the
//! tiltrotor object is live, and which tilt type it is flying.
//!
//! # When it is enabled
//!
//! Upstream `Tiltrotor::enabled` is `(enable > 0) && setup_complete`.
//! `setup()` fills `enable` from the old heuristic when `Q_TILT_ENABLE`
//! was never written: a QuadPlane whose `Q_TILT_MASK` is non-zero, or
//! whose `Q_TILT_TYPE` is [`TiltType::Bicopter`], is a tiltrotor, so
//! enable is saved as 1. An explicit zero stays off. `setup_complete`
//! is set only when enable ends up positive — calling `setup` on a
//! disabled object does not make `enabled()` true.
//!
//! The comment on `tiltrotor.cpp` says the block is "Enabled by setting
//! `Q_TILT_MASK` to a non-zero value". Bicopter is the other auto-enable
//! path because those airframes do not use a tilt mask.
//!
//! # The four tilt types
//!
//! Upstream `Q_TILT_TYPE` (`Tiltrotor::type`):
//!
//! - [`TiltType::Continuous`] — rotors tilt to any angle on demand.
//! - [`TiltType::Binary`] — retract-style servo, fully forward or fully up.
//! - [`TiltType::VectoredYaw`] — tilt motors vector thrust for yaw in hover.
//! - [`TiltType::Bicopter`] — must use tailsitter frame class (10).
//!
//! [`Tiltrotor::tilt_type`] is `Some` only when the object is live and
//! the stored discriminant is one of those four.
//!
//! # Tilt angle and slew
//!
//! Upstream `current_tilt` is a 0..1 proportion (0 = rotors up / hover,
//! 1 = fully forward). [`Tiltrotor::tilt_angle`] is that as degrees
//! (`current_tilt * 90`), matching the TILT log field. [`Tiltrotor::slew`]
//! walks `current_tilt` toward a target at `Q_TILT_RATE_UP` /
//! `Q_TILT_RATE_DN` (`max_rate_up_dps` / `max_rate_down_dps`). Rate-down
//! of zero uses the up rate. `up` in [`Tiltrotor::tilt_max_change`] is
//! `newtilt < current_tilt` — decreasing tilt is hover-ward.
//!
//! # Vectored yaw and flap mix
//!
//! [`Tiltrotor::vectoring_hover`] is the armed VTOL half of
//! `Tiltrotor::vectoring`: `base_output` from `Q_TILT_YAW_ANGLE` /
//! `Q_TILT_FIX_ANGLE`, then left/right offset from motors yaw+roll
//! with the hover throttle scaler. [`Tiltrotor::vectoring_fw`] is the
//! `tilt_over_max_angle` / elevon half (`Q_TILT_FIX_GAIN`).
//! [`Tiltrotor::get_forward_flight_tilt`] is the `Q_TILT_WING_FLAP`
//! mix (`k_flap_auto` 0..100).
//!
//! The 90 DPS fast-tilt override (manual / unstabilised FW), flap-range
//! rate argument on `tilt_max_change`, `continuous_update` /
//! `binary_update`, tilt-compensate, bicopter output, and
//! `Tiltrotor_Transition` leftover live in later slices.

/// Default `Q_TILT_ENABLE`, upstream `AP_GROUPINFO_FLAGS("ENABLE", 1, Tiltrotor, enable, 0)`.
pub const TILT_ENABLE_DEFAULT: i8 = 0;

/// Default `Q_TILT_MASK`, upstream `AP_GROUPINFO("MASK", 2, Tiltrotor, tilt_mask, 0)`.
pub const TILT_MASK_DEFAULT: i16 = 0;

/// Default `Q_TILT_TYPE`, upstream `AP_GROUPINFO("TYPE", 5, Tiltrotor, type, TILT_TYPE_CONTINUOUS)`.
pub const TILT_TYPE_DEFAULT: i8 = 0;

/// Default `Q_TILT_RATE_UP`, upstream `AP_GROUPINFO("RATE_UP", 3, Tiltrotor, max_rate_up_dps, 40)`.
pub const TILT_RATE_UP_DPS_DEFAULT: i16 = 40;

/// Default `Q_TILT_RATE_DN`, upstream `AP_GROUPINFO("RATE_DN", 6, Tiltrotor, max_rate_down_dps, 0)`.
///
/// Zero means "use [`TILT_RATE_UP_DPS_DEFAULT`]" in [`Tiltrotor::tilt_max_change`].
pub const TILT_RATE_DN_DPS_DEFAULT: i16 = 0;

/// Default `Q_TILT_MAX`, upstream `AP_GROUPINFO("MAX", 4, Tiltrotor, max_angle_deg, 45)`.
///
/// Beyond this angle [`Tiltrotor::tilt_over_max_angle`] is true and
/// vectored-yaw mix yields to fixed-wing tilt.
pub const TILT_MAX_ANGLE_DEG_DEFAULT: i8 = 45;

/// Default `Q_TILT_YAW_ANGLE`, upstream `AP_GROUPINFO("YAW_ANGLE", 7, Tiltrotor, tilt_yaw_angle, 0)`.
///
/// VTOL tilt-servo angle at minimum output (fully back). Non-zero plus
/// [`TiltType::VectoredYaw`] is what gives hover yaw authority.
pub const TILT_YAW_ANGLE_DEG_DEFAULT: f32 = 0.0;

/// Default `Q_TILT_FIX_ANGLE`, upstream `AP_GROUPINFO("FIX_ANGLE", 8, Tiltrotor, fixed_angle, 0)`.
pub const TILT_FIXED_ANGLE_DEG_DEFAULT: f32 = 0.0;

/// Default `Q_TILT_FIX_GAIN`, upstream `AP_GROUPINFO("FIX_GAIN", 9, Tiltrotor, fixed_gain, 0)`.
pub const TILT_FIXED_GAIN_DEFAULT: f32 = 0.0;

/// Default `Q_TILT_WING_FLAP`, upstream `AP_GROUPINFO("WING_FLAP", 10, Tiltrotor, flap_angle_deg, 0)`.
///
/// [`Tiltrotor::get_fully_forward_tilt`] is `1 - flap/90`.
/// [`Tiltrotor::get_forward_flight_tilt`] scales that by `k_flap_auto`.
pub const TILT_FLAP_ANGLE_DEG_DEFAULT: f32 = 0.0;

/// Vectored tilt-servo outputs, upstream `SRV_Channels::set_output_scaled`
/// range 0..1000 (`k_tiltMotorLeft` / `Right` / `Rear` / `RearLeft` /
/// `RearRight`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VectoredTiltOut {
    /// `k_tiltMotorLeft`.
    pub left: f32,
    /// `k_tiltMotorRight`.
    pub right: f32,
    /// `k_tiltMotorRear`.
    pub rear: f32,
    /// `k_tiltMotorRearLeft`.
    pub rear_left: f32,
    /// `k_tiltMotorRearRight`.
    pub rear_right: f32,
    /// Upstream `motors->limit.yaw` after the mix.
    pub yaw_limited: bool,
}

/// Types of tilt mechanisms, upstream `Tiltrotor::TILT_TYPE_*`.
///
/// Discriminants match the `@Values` on `Q_TILT_TYPE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i8)]
pub enum TiltType {
    /// `TILT_TYPE_CONTINUOUS` — rotors tilt to any angle on demand.
    Continuous = 0,
    /// `TILT_TYPE_BINARY` — retract-style, fully forward or fully up.
    Binary = 1,
    /// `TILT_TYPE_VECTORED_YAW` — tilt motors control yaw in hover.
    VectoredYaw = 2,
    /// `TILT_TYPE_BICOPTER` — must use tailsitter frame class (10).
    Bicopter = 3,
}

impl TiltType {
    /// Inverse of the upstream discriminant.
    #[must_use]
    pub const fn from_i8(value: i8) -> Option<Self> {
        match value {
            0 => Some(Self::Continuous),
            1 => Some(Self::Binary),
            2 => Some(Self::VectoredYaw),
            3 => Some(Self::Bicopter),
            _ => None,
        }
    }

    /// Upstream discriminant.
    #[must_use]
    pub const fn as_i8(self) -> i8 {
        self as i8
    }
}

/// What `Tiltrotor::setup` reads off QuadPlane and the tiltrotor params.
#[derive(Debug, Clone, Copy)]
pub struct TiltrotorConfig {
    /// `Q_TILT_ENABLE` when the parameter has been written.
    ///
    /// `None` means unconfigured: `setup` applies the mask / bicopter
    /// heuristic.
    pub enable: Option<i8>,
    /// `Q_TILT_MASK`, upstream `tilt_mask`.
    pub tilt_mask: i16,
    /// `Q_TILT_TYPE`, upstream `Tiltrotor::type`.
    pub tilt_type: i8,
}

impl TiltrotorConfig {
    /// A disabled, unconfigured tiltrotor (zero mask, continuous type).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            enable: None,
            tilt_mask: TILT_MASK_DEFAULT,
            tilt_type: TILT_TYPE_DEFAULT,
        }
    }

    /// Unconfigured enable with a non-zero tilt mask (auto-enables).
    #[must_use]
    pub const fn with_tilt_mask(tilt_mask: i16) -> Self {
        Self {
            enable: None,
            tilt_mask,
            tilt_type: TILT_TYPE_DEFAULT,
        }
    }

    /// Unconfigured enable as a bicopter (auto-enables, no tilt mask).
    #[must_use]
    pub const fn bicopter() -> Self {
        Self {
            enable: None,
            tilt_mask: TILT_MASK_DEFAULT,
            tilt_type: TiltType::Bicopter as i8,
        }
    }
}

impl Default for TiltrotorConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// The tiltrotor object, upstream `class Tiltrotor`.
#[derive(Debug, Clone, Copy)]
pub struct Tiltrotor {
    enable: i8,
    setup_complete: bool,
    tilt_mask: i16,
    tilt_type: i8,
    max_rate_up_dps: i16,
    max_rate_down_dps: i16,
    max_angle_deg: i8,
    tilt_yaw_angle: f32,
    fixed_angle: f32,
    fixed_gain: f32,
    flap_angle_deg: f32,
    current_tilt: f32,
    angle_achieved: bool,
}

impl Tiltrotor {
    /// Run upstream `Tiltrotor::setup` and return the resulting object.
    ///
    /// Does not persist parameters (`set_and_save`); the caller owns that.
    /// Servo assignment, thrust-compensation callback, and
    /// `Tiltrotor_Transition` allocation are later slices.
    #[must_use]
    pub fn setup(cfg: TiltrotorConfig) -> Self {
        let mut enable = cfg.enable.unwrap_or(TILT_ENABLE_DEFAULT);
        if cfg.enable.is_none() && (cfg.tilt_mask != 0 || cfg.tilt_type == TiltType::Bicopter as i8)
        {
            enable = 1;
        }

        // Upstream returns early when `enable <= 0`, leaving
        // `setup_complete` false.
        let setup_complete = enable > 0;
        Self {
            enable,
            setup_complete,
            tilt_mask: cfg.tilt_mask,
            tilt_type: cfg.tilt_type,
            max_rate_up_dps: TILT_RATE_UP_DPS_DEFAULT,
            max_rate_down_dps: TILT_RATE_DN_DPS_DEFAULT,
            max_angle_deg: TILT_MAX_ANGLE_DEG_DEFAULT,
            tilt_yaw_angle: TILT_YAW_ANGLE_DEG_DEFAULT,
            fixed_angle: TILT_FIXED_ANGLE_DEG_DEFAULT,
            fixed_gain: TILT_FIXED_GAIN_DEFAULT,
            flap_angle_deg: TILT_FLAP_ANGLE_DEG_DEFAULT,
            current_tilt: 0.0,
            angle_achieved: false,
        }
    }

    /// Current `Q_TILT_ENABLE` after setup.
    #[must_use]
    pub const fn enable(&self) -> i8 {
        self.enable
    }

    /// Upstream `Tiltrotor::enabled` — `(enable > 0) && setup_complete`.
    #[must_use]
    pub const fn enabled(&self) -> bool {
        self.enable > 0 && self.setup_complete
    }

    /// Current `Q_TILT_MASK` after setup.
    #[must_use]
    pub const fn tilt_mask(&self) -> i16 {
        self.tilt_mask
    }

    /// Current `Q_TILT_TYPE` discriminant after setup.
    #[must_use]
    pub const fn tilt_type_raw(&self) -> i8 {
        self.tilt_type
    }

    /// Decoded tilt type when the object is live.
    ///
    /// `None` when disabled or the stored discriminant is not one of the
    /// four upstream `TILT_TYPE_*` values.
    #[must_use]
    pub const fn tilt_type(&self) -> Option<TiltType> {
        if !self.enabled() {
            return None;
        }
        TiltType::from_i8(self.tilt_type)
    }

    /// Upstream `Tiltrotor::is_vectored` — `enabled() && _is_vectored`.
    ///
    /// `_is_vectored` is set in `setup` when the tilt mask is non-zero
    /// and the type is [`TiltType::VectoredYaw`].
    #[must_use]
    pub const fn is_vectored(&self) -> bool {
        self.enabled() && self.tilt_mask != 0 && self.tilt_type == TiltType::VectoredYaw as i8
    }

    /// Current `Q_TILT_RATE_UP` after setup, degrees per second.
    #[must_use]
    pub const fn max_rate_up_dps(&self) -> i16 {
        self.max_rate_up_dps
    }

    /// Write `Q_TILT_RATE_UP`.
    pub fn set_max_rate_up_dps(&mut self, max_rate_up_dps: i16) {
        self.max_rate_up_dps = max_rate_up_dps;
    }

    /// Current `Q_TILT_RATE_DN` after setup, degrees per second.
    #[must_use]
    pub const fn max_rate_down_dps(&self) -> i16 {
        self.max_rate_down_dps
    }

    /// Write `Q_TILT_RATE_DN`. Zero means "use the up rate".
    pub fn set_max_rate_down_dps(&mut self, max_rate_down_dps: i16) {
        self.max_rate_down_dps = max_rate_down_dps;
    }

    /// Current tilt proportion, upstream `Tiltrotor::current_tilt`.
    ///
    /// `0` is rotors up (hover), `1` is fully forward.
    #[must_use]
    pub const fn current_tilt(&self) -> f32 {
        self.current_tilt
    }

    /// Current tilt in degrees from vertical, `current_tilt * 90`.
    ///
    /// Matches the TILT field `write_log` stores (`current_tilt * 90.0`).
    #[must_use]
    pub const fn tilt_angle(&self) -> f32 {
        self.current_tilt * 90.0
    }

    /// Upstream `bool angle_achieved` after the last [`Self::slew`].
    #[must_use]
    pub const fn angle_achieved(&self) -> bool {
        self.angle_achieved
    }

    /// Upstream `Tiltrotor::tilt_angle_achieved`.
    ///
    /// True when disabled, when the type is not continuous, or when the
    /// last slew reached its target. Slow rates can leave continuous
    /// tilts lagging, so this is not the same as [`Self::fully_fwd`].
    #[must_use]
    pub const fn tilt_angle_achieved(&self) -> bool {
        !self.enabled() || self.tilt_type != TiltType::Continuous as i8 || self.angle_achieved
    }

    /// Current `Q_TILT_MAX` after setup, degrees.
    #[must_use]
    pub const fn max_angle_deg(&self) -> i8 {
        self.max_angle_deg
    }

    /// Write `Q_TILT_MAX`.
    pub fn set_max_angle_deg(&mut self, max_angle_deg: i8) {
        self.max_angle_deg = max_angle_deg;
    }

    /// Current `Q_TILT_YAW_ANGLE` after setup, degrees.
    #[must_use]
    pub const fn tilt_yaw_angle(&self) -> f32 {
        self.tilt_yaw_angle
    }

    /// Write `Q_TILT_YAW_ANGLE`.
    pub fn set_tilt_yaw_angle(&mut self, tilt_yaw_angle: f32) {
        self.tilt_yaw_angle = tilt_yaw_angle;
    }

    /// Current `Q_TILT_FIX_ANGLE` after setup, degrees.
    #[must_use]
    pub const fn fixed_angle(&self) -> f32 {
        self.fixed_angle
    }

    /// Write `Q_TILT_FIX_ANGLE`.
    pub fn set_fixed_angle(&mut self, fixed_angle: f32) {
        self.fixed_angle = fixed_angle;
    }

    /// Current `Q_TILT_FIX_GAIN` after setup.
    #[must_use]
    pub const fn fixed_gain(&self) -> f32 {
        self.fixed_gain
    }

    /// Write `Q_TILT_FIX_GAIN`.
    pub fn set_fixed_gain(&mut self, fixed_gain: f32) {
        self.fixed_gain = fixed_gain;
    }

    /// Current `Q_TILT_WING_FLAP` after setup, degrees.
    #[must_use]
    pub const fn flap_angle_deg(&self) -> f32 {
        self.flap_angle_deg
    }

    /// Write `Q_TILT_WING_FLAP`.
    pub fn set_flap_angle_deg(&mut self, flap_angle_deg: f32) {
        self.flap_angle_deg = flap_angle_deg;
    }

    /// Upstream `Tiltrotor::get_fully_forward_tilt`.
    ///
    /// `1 - flap_angle_deg/90`. Default flap is zero, so this is `1`.
    #[must_use]
    pub const fn get_fully_forward_tilt(&self) -> f32 {
        1.0 - (self.flap_angle_deg * (1.0 / 90.0))
    }

    /// Upstream `Tiltrotor::get_forward_flight_tilt`.
    ///
    /// `1 - (flap_angle_deg/90) * flap_auto * 0.01`. `flap_auto` is the
    /// slew-limited `k_flap_auto` scaled output (0..100).
    #[must_use]
    pub const fn get_forward_flight_tilt(&self, flap_auto: f32) -> f32 {
        1.0 - ((self.flap_angle_deg * (1.0 / 90.0)) * flap_auto * 0.01)
    }

    /// Upstream `Tiltrotor::tilt_over_max_angle`.
    ///
    /// True when `current_tilt` is past `min(Q_TILT_MAX/90,
    /// get_forward_flight_tilt(flap_auto))`. `flap_auto` 0 (no flap
    /// demand) leaves the bound at `Q_TILT_MAX/90`.
    #[must_use]
    pub const fn tilt_over_max_angle(&self, flap_auto: f32) -> bool {
        let tilt_threshold = (self.max_angle_deg as f32) * (1.0 / 90.0);
        let fwd = self.get_forward_flight_tilt(flap_auto);
        let limit = if tilt_threshold < fwd {
            tilt_threshold
        } else {
            fwd
        };
        self.current_tilt > limit
    }

    /// Total tilt travel, upstream `90 + tilt_yaw_angle + fixed_angle`.
    #[must_use]
    pub const fn total_angle_deg(&self) -> f32 {
        90.0 + self.tilt_yaw_angle + self.fixed_angle
    }

    /// Output (0..1) that points motors straight up,
    /// `tilt_yaw_angle / total_angle`.
    #[must_use]
    pub const fn zero_out(&self) -> f32 {
        let total = self.total_angle_deg();
        if total <= 0.0 {
            0.0
        } else {
            self.tilt_yaw_angle / total
        }
    }

    /// Forward-flight tilt limit as a 0..1 output,
    /// `fixed_angle / total_angle`.
    #[must_use]
    pub const fn fixed_tilt_limit(&self) -> f32 {
        let total = self.total_angle_deg();
        if total <= 0.0 {
            0.0
        } else {
            self.fixed_angle / total
        }
    }

    /// Base tilt output from `current_tilt` before yaw / roll mix.
    ///
    /// `zero_out + current_tilt * (level_out - zero_out)` where
    /// `level_out = 1 - fixed_tilt_limit`.
    #[must_use]
    pub const fn base_output(&self) -> f32 {
        let zero = self.zero_out();
        let level = 1.0 - self.fixed_tilt_limit();
        zero + (self.current_tilt * (level - zero))
    }

    /// Armed VTOL half of `Tiltrotor::vectoring`.
    ///
    /// `yaw_out` / `roll_out` are `motors->get_yaw()+get_yaw_ff()` and
    /// roll equivalents (`-1..1`). `throttle` / `hover_throttle` are
    /// `get_throttle_out` / `get_throttle_hover`. Servo writes are a
    /// later slice; this returns the 0..1000 scaled values.
    #[must_use]
    pub fn vectoring_hover(
        &self,
        yaw_out: f32,
        roll_out: f32,
        throttle: f32,
        hover_throttle: f32,
    ) -> VectoredTiltOut {
        let base = self.base_output();
        let yaw_range = self.zero_out();
        let throttle_scaler = if throttle > 0.0 {
            constrain_f32(hover_throttle / throttle, 0.5, 2.0)
        } else {
            2.0
        };
        let tilt_rad = self.current_tilt * core::f32::consts::FRAC_PI_2;
        let sin_tilt = libm::sinf(tilt_rad);
        let cos_tilt = libm::cosf(tilt_rad);
        let avg_roll_factor = 0.5;
        let mut tilt_scale =
            throttle_scaler * yaw_out * cos_tilt + avg_roll_factor * roll_out * sin_tilt;
        let mut yaw_limited = false;
        if abs_f32(tilt_scale) > 1.0 {
            tilt_scale = constrain_f32(tilt_scale, -1.0, 1.0);
            yaw_limited = true;
        }
        let tilt_offset = tilt_scale * yaw_range;
        let mut left_tilt = base + tilt_offset;
        let mut right_tilt = base - tilt_offset;
        if ((left_tilt > 1.0) || (left_tilt < 0.0)) && ((right_tilt > 1.0) || (right_tilt < 0.0)) {
            yaw_limited = true;
        }
        left_tilt = constrain_f32(left_tilt, 0.0, 1.0) * 1000.0;
        right_tilt = constrain_f32(right_tilt, 0.0, 1.0) * 1000.0;
        let rear = constrain_f32(base, 0.0, 1.0) * 1000.0;
        VectoredTiltOut {
            left: left_tilt,
            right: right_tilt,
            rear,
            rear_left: left_tilt,
            rear_right: right_tilt,
            yaw_limited,
        }
    }

    /// Fixed-wing / `tilt_over_max_angle` half of `Tiltrotor::vectoring`.
    ///
    /// `elevon_left` / `elevon_right` / `elevator` are the 0..±4500
    /// scaled surface outputs. `scaler` is 1 in MANUAL, otherwise
    /// `FW_vector_throttle_scaling() / get_speed_scaler()`.
    #[must_use]
    pub const fn vectoring_fw(
        &self,
        elevon_left: f32,
        elevon_right: f32,
        elevator: f32,
        scaler: f32,
    ) -> VectoredTiltOut {
        let base = self.base_output();
        let gain = self.fixed_gain * self.fixed_tilt_limit() * scaler;
        let right = gain * elevon_right * (1.0 / 4500.0);
        let left = gain * elevon_left * (1.0 / 4500.0);
        let mid = gain * elevator * (1.0 / 4500.0);
        VectoredTiltOut {
            left: constrain_f32(base - right, 0.0, 1.0) * 1000.0,
            right: constrain_f32(base - left, 0.0, 1.0) * 1000.0,
            rear: constrain_f32(base + mid, 0.0, 1.0) * 1000.0,
            rear_left: constrain_f32(base + left, 0.0, 1.0) * 1000.0,
            rear_right: constrain_f32(base + right, 0.0, 1.0) * 1000.0,
            yaw_limited: false,
        }
    }

    /// Upstream `Tiltrotor::fully_fwd`.
    #[must_use]
    pub const fn fully_fwd(&self) -> bool {
        self.enabled() && self.tilt_mask != 0 && self.current_tilt >= self.get_fully_forward_tilt()
    }

    /// Upstream `Tiltrotor::fully_up`.
    #[must_use]
    pub const fn fully_up(&self) -> bool {
        self.enabled() && self.tilt_mask != 0 && self.current_tilt <= 0.0
    }

    /// Maximum tilt-proportion change this tick, upstream `tilt_max_change`.
    ///
    /// `up` is hover-ward (`newtilt < current_tilt`). `dt_s` is
    /// `plane.G_Dt`. The 90 DPS fast-tilt override and flap-range
    /// argument are later slices.
    #[must_use]
    pub const fn tilt_max_change(&self, up: bool, dt_s: f32) -> f32 {
        let rate = if up || self.max_rate_down_dps <= 0 {
            self.max_rate_up_dps as f32
        } else {
            self.max_rate_down_dps as f32
        };
        let dt = if dt_s < 0.0 { 0.0 } else { dt_s };
        rate * dt * (1.0 / 90.0)
    }

    /// Slew `current_tilt` toward `newtilt`, upstream `Tiltrotor::slew`.
    ///
    /// `newtilt` is 0..1. `dt_s` is `plane.G_Dt`. Servo output
    /// (`k_motor_tilt`) is a later slice.
    pub fn slew(&mut self, newtilt: f32, dt_s: f32) {
        let up = newtilt < self.current_tilt;
        let max_change = self.tilt_max_change(up, dt_s);
        self.current_tilt = constrain_f32(
            newtilt,
            self.current_tilt - max_change,
            self.current_tilt + max_change,
        );
        self.angle_achieved = is_equal_f32(newtilt, self.current_tilt);
    }
}

const fn abs_f32(v: f32) -> f32 {
    if v < 0.0 {
        -v
    } else {
        v
    }
}

const fn constrain_f32(v: f32, lo: f32, hi: f32) -> f32 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

const fn is_equal_f32(a: f32, b: f32) -> bool {
    abs_f32(a - b) < f32::EPSILON
}
