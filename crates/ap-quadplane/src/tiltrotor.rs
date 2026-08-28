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
//! # Leftover update / compensate / transition
//!
//! This closer stubs the remaining `tiltrotor.cpp` / `.h` public API:
//! [`Tiltrotor::update`] / [`Tiltrotor::continuous_update`] /
//! [`Tiltrotor::binary_update`] / [`Tiltrotor::binary_slew`],
//! [`Tiltrotor::tilt_compensate`] / [`Tiltrotor::tilt_compensate_angle`],
//! [`Tiltrotor::bicopter_output`], [`Tiltrotor::write_log`],
//! [`Tiltrotor::get_forward_throttle`], and [`TiltrotorTransition`].
//! Live SRV / motors / logger writes stay with the caller.
//!
//! [`Tiltrotor::tilt_max_change_ex`] adds the leftover
//! `in_flap_range` argument and the 90 DPS fast-tilt override.

use crate::transition_fsm::TransitionState;

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

/// Minimum fast-tilt rate (deg/s) in MANUAL or unstabilised FW.
///
/// Upstream `tilt_max_change` `MAX(rate, 90)`.
pub const TILT_FAST_TILT_DPS: f32 = 90.0;

/// Upstream `SERVO_MAX` used by `Tiltrotor::bicopter_output`.
pub const TILT_SERVO_MAX: f32 = 4500.0;

/// Upstream `LOG_TILT_MSG` name.
pub const LOG_TILT_NAME: &str = "TILT";

/// Upstream `LOG_TILT_MSG` fields (`"TimeUS,Tilt,FL,FR"`).
pub const LOG_TILT_FIELDS: &str = "TimeUS,Tilt,FL,FR";

/// Upstream `LOG_TILT_MSG` units (`"sddd"`).
pub const LOG_TILT_UNITS: &str = "sddd";

/// Upstream `LOG_TILT_MSG` multipliers (`"F---"`).
pub const LOG_TILT_MULTS: &str = "F---";

/// Relock `transition_yaw_cd` when the last set is older than this.
pub const TRANSITION_YAW_LOCK_MS: u32 = 100;

/// Coordinated-turn integrate starts when `|nav_roll_cd|` exceeds this.
pub const COORD_TURN_ROLL_CD_MIN: i32 = 1000;

const TILT_GRAVITY_MSS: f32 = 9.806_65;

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

/// Flight-mode snapshot for `Tiltrotor::update` / `continuous_update` /
/// `binary_update`.
#[derive(Debug, Clone, Copy)]
pub struct TiltUpdateIn {
    /// `quadplane.in_vtol_mode()`.
    pub in_vtol_mode: bool,
    /// `plane.arming.is_armed_and_safety_off()`.
    pub armed: bool,
    /// `quadplane.assisted_flight`.
    pub assisted_flight: bool,
    /// `plane.control_mode == &plane.mode_manual`.
    pub manual_mode: bool,
    /// `quadplane.option_is_set(Option::DISARMED_TILT_UP)`.
    pub disarmed_tilt_up: bool,
    /// `plane.control_mode == &plane.mode_qautotune`.
    pub qautotune: bool,
    /// QACRO / QSTABILIZE / QHOVER.
    pub qacro_qstabilize_qhover: bool,
    /// `quadplane.rc_fwd_thr_ch != nullptr`.
    pub has_rc_fwd_thr: bool,
    /// `get_vfwd_method() == ActiveFwdThr::NEW`.
    pub using_new_vfwd: bool,
    /// `quadplane.is_flying_vtol()`.
    pub flying_vtol: bool,
    /// `quadplane.forward_throttle_pct()` (0..100).
    pub forward_throttle_pct: f32,
    /// `SRV_Channels::get_output_scaled(k_throttle)`.
    pub fw_throttle: f32,
    /// `MAX(plane.aparm.throttle_min, 0)`.
    pub throttle_min: f32,
    /// `motors->get_throttle()`.
    pub motors_throttle: f32,
    /// `quadplane.motor_test.running`.
    pub motor_test_running: bool,
    /// Slew-limited `k_flap_auto` (0..100).
    pub flap_auto: f32,
    /// `plane.G_Dt`.
    pub dt_s: f32,
    /// `transition->transition_state`.
    pub transition_state: TransitionState,
}

impl TiltUpdateIn {
    /// Zeroed FW / disarmed snapshot (`dt_s` 0.1, `AIRSPEED_WAIT`).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            in_vtol_mode: false,
            armed: false,
            assisted_flight: false,
            manual_mode: false,
            disarmed_tilt_up: false,
            qautotune: false,
            qacro_qstabilize_qhover: false,
            has_rc_fwd_thr: false,
            using_new_vfwd: false,
            flying_vtol: false,
            forward_throttle_pct: 0.0,
            fw_throttle: 0.0,
            throttle_min: 0.0,
            motors_throttle: 0.0,
            motor_test_running: false,
            flap_auto: 0.0,
            dt_s: 0.1,
            transition_state: TransitionState::AirspeedWait,
        }
    }
}

impl Default for TiltUpdateIn {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of `update` / `continuous_update` / `binary_update`.
///
/// Live `SRV_Channels` / `motors->output_motor_mask` writes stay with
/// the caller; this reports the scaled values those writes would use.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TiltUpdateOut {
    /// `_motors_active` after the tick.
    pub motors_active: bool,
    /// `current_throttle` after the tick.
    pub current_throttle: f32,
    /// `current_tilt` after the tick.
    pub current_tilt: f32,
    /// True when `output_motor_mask` would run.
    pub ran_motor_mask: bool,
    /// Throttle argument to `output_motor_mask`.
    pub motor_mask_throttle: f32,
    /// Motor bitmask argument (`0` when throttle is zero).
    pub motor_mask: i16,
    /// `k_motor_tilt` 0..1000 after `binary_slew`; `None` on continuous.
    pub motor_tilt_scaled: Option<f32>,
    /// True when `update` would call `vectoring()`.
    pub ran_vectoring: bool,
}

/// Inputs for `Tiltrotor::bicopter_output`.
#[derive(Debug, Clone, Copy)]
pub struct BicopterIn {
    /// `quadplane.motor_test.running`.
    pub motor_test_running: bool,
    /// `quadplane.in_vtol_mode()`.
    pub in_vtol_mode: bool,
    /// `quadplane.assisted_flight`.
    pub assisted_flight: bool,
    /// Current `k_tiltMotorLeft` scaled output.
    pub tilt_left: f32,
    /// Current `k_tiltMotorRight` scaled output.
    pub tilt_right: f32,
}

/// Result of `Tiltrotor::bicopter_output`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BicopterOut {
    /// `k_tiltMotorLeft` after the mix.
    pub left: f32,
    /// `k_tiltMotorRight` after the mix.
    pub right: f32,
    /// False when type is not bicopter or motor-test is running.
    pub applied: bool,
    /// `quadplane.motors_output(true)` (assisted) vs `false`.
    pub motors_output_assisted: bool,
}

/// `Tiltrotor::write_log` packet (no timestamp / logger backend).
#[derive(Debug, Clone, Copy)]
pub struct TiltrotorLog {
    /// `current_tilt * 90` (degrees from vertical).
    pub current_tilt_deg: f32,
    /// Front-left servo angle, or NaN when type is not vectored-yaw.
    pub front_left_tilt: f32,
    /// Front-right servo angle, or NaN when type is not vectored-yaw.
    pub front_right_tilt: f32,
    /// True when FL/FR are computed (vectored-yaw type).
    pub sides_valid: bool,
}

/// `Tiltrotor_Transition` leftover (no heap `NEW_NOTHROW`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TiltrotorTransition {
    vectored: bool,
    in_vtol_mode: bool,
    transition_state: TransitionState,
    transition_yaw_cd: f32,
}

impl TiltrotorTransition {
    /// Snapshot used by the four leftover `Tiltrotor_Transition` methods.
    #[must_use]
    pub const fn new(
        vectored: bool,
        in_vtol_mode: bool,
        transition_state: TransitionState,
        transition_yaw_cd: f32,
    ) -> Self {
        Self {
            vectored,
            in_vtol_mode,
            transition_state,
            transition_yaw_cd,
        }
    }

    /// Build from a live [`Tiltrotor`].
    #[must_use]
    pub const fn from_tiltrotor(
        tiltrotor: &Tiltrotor,
        in_vtol_mode: bool,
        transition_state: TransitionState,
    ) -> Self {
        Self::new(
            tiltrotor.is_vectored(),
            in_vtol_mode,
            transition_state,
            tiltrotor.transition_yaw_cd,
        )
    }

    /// Upstream `use_multirotor_control_in_fwd_transition`.
    #[must_use]
    pub const fn use_multirotor_control_in_fwd_transition(&self) -> bool {
        if !self.vectored {
            return false;
        }
        matches!(
            self.transition_state,
            TransitionState::AirspeedWait | TransitionState::Timer
        )
    }

    /// Upstream `Tiltrotor_Transition::update_yaw_target`.
    ///
    /// Copies [`Self::transition_yaw_cd`] when multirotor-in-fwd is live.
    pub const fn update_yaw_target(&self, yaw_target_cd: &mut f32) -> bool {
        if !self.use_multirotor_control_in_fwd_transition() {
            return false;
        }
        *yaw_target_cd = self.transition_yaw_cd;
        true
    }

    /// Upstream `Tiltrotor_Transition::show_vtol_view`.
    #[must_use]
    pub const fn show_vtol_view(&self) -> bool {
        if self.in_vtol_mode {
            return true;
        }
        self.vectored
            && matches!(
                self.transition_state,
                TransitionState::AirspeedWait | TransitionState::Timer
            )
    }

    /// Upstream `Tiltrotor_Transition::allow_vfwd`.
    ///
    /// `thrust_boost` is `motors->get_thrust_boost()`. `lost_motor_tilting`
    /// is `tiltrotor.is_motor_tilting(lost_motor)`. `lost_roll_factor`
    /// is `motors->get_roll_factor(lost_motor)`.
    #[must_use]
    pub const fn allow_vfwd(
        &self,
        thrust_boost: bool,
        lost_motor_tilting: bool,
        lost_roll_factor: f32,
    ) -> bool {
        if !self.vectored {
            return true;
        }
        if !thrust_boost {
            return true;
        }
        if !lost_motor_tilting {
            return true;
        }
        is_zero_f32(lost_roll_factor)
    }
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
    current_throttle: f32,
    motors_active: bool,
    have_fw_motor: bool,
    have_vtol_motor: bool,
    transition_yaw_cd: f32,
    transition_yaw_set_ms: u32,
}

impl Tiltrotor {
    /// Run upstream `Tiltrotor::setup` and return the resulting object.
    ///
    /// Does not persist parameters (`set_and_save`); the caller owns that.
    /// Servo assignment, thrust-compensation callback, and
    /// `Tiltrotor_Transition` heap allocation stay with the caller.
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
            current_throttle: 0.0,
            motors_active: false,
            have_fw_motor: false,
            have_vtol_motor: false,
            transition_yaw_cd: 0.0,
            transition_yaw_set_ms: 0,
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
    /// `plane.G_Dt`. Fast-tilt / flap-range live on
    /// [`Self::tilt_max_change_ex`].
    #[must_use]
    pub const fn tilt_max_change(&self, up: bool, dt_s: f32) -> f32 {
        self.tilt_max_change_ex(up, false, false, dt_s)
    }

    /// Slew `current_tilt` toward `newtilt`, upstream `Tiltrotor::slew`.
    ///
    /// `newtilt` is 0..1. `dt_s` is `plane.G_Dt`. Servo output
    /// (`k_motor_tilt`) stays with the caller.
    pub fn slew(&mut self, newtilt: f32, dt_s: f32) {
        self.slew_with(newtilt, dt_s, false);
    }

    /// Upstream `Tiltrotor::is_motor_tilting`.
    #[must_use]
    pub const fn is_motor_tilting(&self, motor: u8) -> bool {
        if motor >= 16 {
            return false;
        }
        (self.tilt_mask as u16) & (1u16 << motor) != 0
    }

    /// Upstream `Tiltrotor::has_fw_motor`.
    #[must_use]
    pub const fn has_fw_motor(&self) -> bool {
        self.have_fw_motor
    }

    /// Record a configured fixed-wing throttle channel (setup leftover).
    pub fn set_have_fw_motor(&mut self, have_fw_motor: bool) {
        self.have_fw_motor = have_fw_motor;
    }

    /// Upstream `Tiltrotor::has_vtol_motor`.
    #[must_use]
    pub const fn has_vtol_motor(&self) -> bool {
        self.have_vtol_motor
    }

    /// Record a permanent VTOL motor (setup leftover).
    pub fn set_have_vtol_motor(&mut self, have_vtol_motor: bool) {
        self.have_vtol_motor = have_vtol_motor;
    }

    /// Upstream `Tiltrotor::motors_active` — `enabled() && _motors_active`.
    #[must_use]
    pub const fn motors_active(&self) -> bool {
        self.enabled() && self.motors_active
    }

    /// Upstream `Tiltrotor::current_throttle`.
    #[must_use]
    pub const fn current_throttle(&self) -> f32 {
        self.current_throttle
    }

    /// Last `transition_yaw_cd` written by [`Self::update_yaw_target`].
    #[must_use]
    pub const fn transition_yaw_cd(&self) -> f32 {
        self.transition_yaw_cd
    }

    /// Snapshot of leftover [`TiltrotorTransition`] methods.
    #[must_use]
    pub const fn transition_view(
        &self,
        in_vtol_mode: bool,
        transition_state: TransitionState,
    ) -> TiltrotorTransition {
        TiltrotorTransition::from_tiltrotor(self, in_vtol_mode, transition_state)
    }

    /// Maximum tilt-proportion change this tick, leftover `in_flap_range`
    /// / 90 DPS fast-tilt override.
    ///
    /// `up` is hover-ward. Fast tilt applies only when the type is not
    /// binary, the move is not hover-ward, and the target is not in the
    /// flap range.
    #[must_use]
    pub const fn tilt_max_change_ex(
        &self,
        up: bool,
        in_flap_range: bool,
        fast_tilt: bool,
        dt_s: f32,
    ) -> f32 {
        let mut rate = if up || self.max_rate_down_dps <= 0 {
            self.max_rate_up_dps as f32
        } else {
            self.max_rate_down_dps as f32
        };
        if self.tilt_type != TiltType::Binary as i8 && !up && !in_flap_range && fast_tilt && rate < TILT_FAST_TILT_DPS
        {
            rate = TILT_FAST_TILT_DPS;
        }
        let dt = if dt_s < 0.0 { 0.0 } else { dt_s };
        rate * dt * (1.0 / 90.0)
    }

    /// Slew `current_tilt` toward `newtilt` with an optional fast-tilt rate.
    pub fn slew_with(&mut self, newtilt: f32, dt_s: f32, fast_tilt: bool) {
        let up = newtilt < self.current_tilt;
        let in_flap = newtilt > self.get_fully_forward_tilt();
        let max_change = self.tilt_max_change_ex(up, in_flap, fast_tilt, dt_s);
        self.current_tilt = constrain_f32(
            newtilt,
            self.current_tilt - max_change,
            self.current_tilt + max_change,
        );
        self.angle_achieved = is_equal_f32(newtilt, self.current_tilt);
    }

    /// Upstream `Tiltrotor::binary_slew`.
    ///
    /// Servo output is binary (0 or 1000). `current_tilt` is rate-limited.
    pub fn binary_slew(&mut self, forward: bool, dt_s: f32) -> f32 {
        let scaled = if forward { 1000.0 } else { 0.0 };
        let max_change = self.tilt_max_change(!forward, dt_s);
        if forward {
            self.current_tilt = constrain_f32(self.current_tilt + max_change, 0.0, 1.0);
        } else {
            self.current_tilt = constrain_f32(self.current_tilt - max_change, 0.0, 1.0);
        }
        scaled
    }

    /// Upstream `Tiltrotor::binary_update`.
    pub fn binary_update(&mut self, inp: TiltUpdateIn) -> TiltUpdateOut {
        self.motors_active = true;
        let mut ran_motor_mask = false;
        let mut motor_mask_throttle = 0.0;
        let mut motor_mask = 0i16;
        let motor_tilt_scaled = if !inp.in_vtol_mode {
            let scaled = self.binary_slew(true, inp.dt_s);
            let new_throttle = inp.fw_throttle * 0.01;
            if self.current_tilt >= 1.0 {
                ran_motor_mask = true;
                motor_mask_throttle = new_throttle;
                motor_mask = if is_zero_f32(new_throttle) {
                    0
                } else {
                    self.tilt_mask
                };
            }
            Some(scaled)
        } else {
            Some(self.binary_slew(false, inp.dt_s))
        };
        self.snapshot_out(ran_motor_mask, motor_mask_throttle, motor_mask, motor_tilt_scaled, false)
    }

    /// Upstream `Tiltrotor::continuous_update`.
    pub fn continuous_update(&mut self, inp: TiltUpdateIn) -> TiltUpdateOut {
        self.motors_active = false;
        let fast = inp.manual_mode || (inp.armed && !inp.in_vtol_mode && !inp.assisted_flight);
        let fwd = self.get_forward_flight_tilt(inp.flap_auto);

        if !inp.in_vtol_mode && (!inp.armed || !inp.assisted_flight) {
            let disarmed_tilt_up = !inp.armed && !inp.manual_mode && inp.disarmed_tilt_up;
            self.slew_with(if disarmed_tilt_up { 0.0 } else { fwd }, inp.dt_s, fast);
            let max_change = self.tilt_max_change_ex(false, false, fast, inp.dt_s);
            let new_throttle = constrain_f32(inp.fw_throttle * 0.01, 0.0, 1.0);
            if self.current_tilt < self.get_fully_forward_tilt() {
                self.current_throttle = constrain_f32(
                    new_throttle,
                    self.current_throttle - max_change,
                    self.current_throttle + max_change,
                );
            } else {
                self.current_throttle = new_throttle;
            }
            if !inp.armed {
                self.current_throttle = 0.0;
            } else {
                self.motors_active = true;
            }
            let mut ran_motor_mask = false;
            let mut motor_mask_throttle = 0.0;
            let mut motor_mask = 0i16;
            if !inp.motor_test_running {
                ran_motor_mask = true;
                motor_mask_throttle = self.current_throttle;
                motor_mask = if is_zero_f32(self.current_throttle) {
                    0
                } else {
                    self.tilt_mask
                };
            }
            return self.snapshot_out(ran_motor_mask, motor_mask_throttle, motor_mask, None, false);
        }

        let max_change = self.tilt_max_change(inp.motors_throttle < self.current_throttle, inp.dt_s);
        self.current_throttle = constrain_f32(
            inp.motors_throttle,
            self.current_throttle - max_change,
            self.current_throttle + max_change,
        );

        if inp.qautotune {
            self.slew_with(0.0, inp.dt_s, fast);
            return self.snapshot_out(false, 0.0, 0, None, false);
        }

        if !inp.assisted_flight && inp.using_new_vfwd && inp.flying_vtol {
            let fwd_g_demand = 0.01 * inp.forward_throttle_pct;
            let atan_deg = rad_to_deg(libm::atanf(fwd_g_demand));
            let fwd_tilt_deg = if atan_deg < self.max_angle_deg as f32 {
                atan_deg
            } else {
                self.max_angle_deg as f32
            };
            let target = min_f32(fwd_tilt_deg * (1.0 / 90.0), fwd);
            self.slew_with(target, inp.dt_s, fast);
            return self.snapshot_out(false, 0.0, 0, None, false);
        }

        if !inp.assisted_flight && inp.qacro_qstabilize_qhover {
            if !inp.has_rc_fwd_thr {
                self.slew_with(0.0, inp.dt_s, fast);
            } else {
                let settilt = 0.01 * inp.forward_throttle_pct;
                let target = min_f32(settilt * (self.max_angle_deg as f32) * (1.0 / 90.0), fwd);
                self.slew_with(target, inp.dt_s, fast);
            }
            return self.snapshot_out(false, 0.0, 0, None, false);
        }

        if inp.assisted_flight
            && matches!(
                inp.transition_state,
                TransitionState::Timer | TransitionState::Done
            )
        {
            self.slew_with(fwd, inp.dt_s, fast);
        } else {
            let thr_min = if inp.throttle_min > 0.0 {
                inp.throttle_min
            } else {
                0.0
            };
            let settilt = constrain_f32((inp.fw_throttle - thr_min) * 0.02, 0.0, 1.0);
            let target = min_f32(settilt * (self.max_angle_deg as f32) * (1.0 / 90.0), fwd);
            self.slew_with(target, inp.dt_s, fast);
        }
        self.snapshot_out(false, 0.0, 0, None, false)
    }

    /// Upstream `Tiltrotor::update`.
    pub fn update(&mut self, inp: TiltUpdateIn) -> TiltUpdateOut {
        if !self.enabled() || self.tilt_mask == 0 {
            return self.snapshot_out(false, 0.0, 0, None, false);
        }
        let mut out = if self.tilt_type == TiltType::Binary as i8 {
            self.binary_update(inp)
        } else {
            self.continuous_update(inp)
        };
        out.ran_vectoring = self.tilt_type == TiltType::VectoredYaw as i8;
        out
    }

    /// Upstream `Tiltrotor::tilt_compensate_angle`.
    ///
    /// `roll_factors` / `yaw_out` stand in for `motors->get_roll_factor`
    /// and `get_yaw()+get_yaw_ff()`. Missing roll factors read as 0.
    pub fn tilt_compensate_angle(
        &self,
        thrust: &mut [f32],
        non_tilted_mul: f32,
        tilted_mul: f32,
        yaw_out: f32,
        roll_factors: &[f32],
    ) {
        let mut tilt_total = 0.0;
        let mut tilt_count = 0u8;
        let mut i = 0usize;
        while i < thrust.len() {
            let Some(slot) = thrust.get_mut(i) else {
                break;
            };
            if !self.is_motor_tilting(i as u8) {
                *slot *= non_tilted_mul;
            } else {
                *slot *= tilted_mul;
                tilt_total += *slot;
                tilt_count = tilt_count.saturating_add(1);
            }
            i += 1;
        }
        if tilt_count == 0 {
            return;
        }
        let sin_tilt = libm::sinf(self.current_tilt * core::f32::consts::FRAC_PI_2);
        let yaw_gain = libm::sinf(deg_to_rad(self.tilt_yaw_angle));
        let avg_tilt_thrust = tilt_total / (tilt_count as f32);
        let mut largest_tilted = 0.0;
        i = 0;
        while i < thrust.len() {
            if self.is_motor_tilting(i as u8) {
                let Some(slot) = thrust.get_mut(i) else {
                    break;
                };
                *slot = self.current_tilt * avg_tilt_thrust + *slot * (1.0 - self.current_tilt);
                let roll = match roll_factors.get(i) {
                    Some(v) => *v,
                    None => 0.0,
                };
                *slot += roll * yaw_out * sin_tilt * yaw_gain;
                if *slot > largest_tilted {
                    largest_tilted = *slot;
                }
            }
            i += 1;
        }
        if largest_tilted > 1.0 {
            let scale = 1.0 / largest_tilted;
            i = 0;
            while i < thrust.len() {
                if let Some(slot) = thrust.get_mut(i) {
                    *slot *= scale;
                }
                i += 1;
            }
        }
    }

    /// Upstream `Tiltrotor::tilt_compensate`.
    pub fn tilt_compensate(
        &self,
        thrust: &mut [f32],
        in_vtol_mode: bool,
        yaw_out: f32,
        roll_factors: &[f32],
    ) {
        if self.current_tilt <= 0.0 {
            return;
        }
        if in_vtol_mode {
            let tilt_factor = libm::cosf(self.current_tilt * core::f32::consts::FRAC_PI_2);
            self.tilt_compensate_angle(thrust, tilt_factor, 1.0, yaw_out, roll_factors);
        } else {
            let tilt_for_inv = if self.current_tilt > 0.98 {
                0.98
            } else {
                self.current_tilt
            };
            let inv_tilt_factor = 1.0 / libm::cosf(tilt_for_inv * core::f32::consts::FRAC_PI_2);
            self.tilt_compensate_angle(thrust, 1.0, inv_tilt_factor, yaw_out, roll_factors);
        }
    }

    /// Upstream `Tiltrotor::bicopter_output`.
    #[must_use]
    pub fn bicopter_output(&self, inp: BicopterIn) -> BicopterOut {
        if self.tilt_type != TiltType::Bicopter as i8 || inp.motor_test_running {
            return BicopterOut {
                left: inp.tilt_left,
                right: inp.tilt_right,
                applied: false,
                motors_output_assisted: false,
            };
        }
        if !inp.in_vtol_mode && self.fully_fwd() {
            return BicopterOut {
                left: -TILT_SERVO_MAX,
                right: -TILT_SERVO_MAX,
                applied: true,
                motors_output_assisted: false,
            };
        }
        let mut tilt_left = inp.tilt_left;
        let mut tilt_right = inp.tilt_right;
        if tilt_left < 0.0 {
            tilt_left *= self.tilt_yaw_angle * (1.0 / 90.0);
        }
        if tilt_right < 0.0 {
            tilt_right *= self.tilt_yaw_angle * (1.0 / 90.0);
        }
        let scaling = libm::cosf(self.current_tilt * core::f32::consts::FRAC_PI_2);
        tilt_left *= scaling;
        tilt_right *= scaling;
        tilt_left = constrain_f32(
            -(self.current_tilt * TILT_SERVO_MAX) + tilt_left,
            -TILT_SERVO_MAX,
            TILT_SERVO_MAX,
        );
        tilt_right = constrain_f32(
            -(self.current_tilt * TILT_SERVO_MAX) + tilt_right,
            -TILT_SERVO_MAX,
            TILT_SERVO_MAX,
        );
        BicopterOut {
            left: tilt_left,
            right: tilt_right,
            applied: true,
            motors_output_assisted: inp.assisted_flight,
        }
    }

    /// Upstream `Tiltrotor::write_log`.
    ///
    /// `left_servo` / `right_servo` are `k_tiltMotorLeft` / `Right`
    /// scaled outputs. Returns `None` when disabled.
    #[must_use]
    pub fn write_log(&self, left_servo: f32, right_servo: f32) -> Option<TiltrotorLog> {
        if !self.enabled() {
            return None;
        }
        if self.tilt_type != TiltType::VectoredYaw as i8 {
            return Some(TiltrotorLog {
                current_tilt_deg: self.current_tilt * 90.0,
                front_left_tilt: f32::NAN,
                front_right_tilt: f32::NAN,
                sides_valid: false,
            });
        }
        let scale = self.total_angle_deg() * 0.001;
        Some(TiltrotorLog {
            current_tilt_deg: self.current_tilt * 90.0,
            front_left_tilt: left_servo * scale - self.tilt_yaw_angle,
            front_right_tilt: right_servo * scale - self.tilt_yaw_angle,
            sides_valid: true,
        })
    }

    /// Upstream `Tiltrotor::get_forward_throttle`.
    ///
    /// `spin_min` / `spin_max` are `thr_lin` bounds. `motor_actuators`
    /// are `(motor_index, thrust_to_actuator)` pairs; only tilting
    /// motors are averaged.
    #[must_use]
    pub fn get_forward_throttle(
        &self,
        spin_min: f32,
        spin_max: f32,
        motor_actuators: &[(u8, f32)],
    ) -> Option<f32> {
        if !self.enabled() || self.tilt_mask == 0 || self.tilt_type != TiltType::VectoredYaw as i8 {
            return None;
        }
        let range = spin_max - spin_min;
        if range <= 0.0 {
            return None;
        }
        let mut sum = 0.0;
        let mut n = 0u8;
        let mut i = 0usize;
        while i < motor_actuators.len() {
            let Some(&(motor, act)) = motor_actuators.get(i) else {
                break;
            };
            if self.is_motor_tilting(motor) {
                sum += (act - spin_min) / range;
                n = n.saturating_add(1);
            }
            i += 1;
        }
        if n == 0 {
            None
        } else {
            Some(sum / (n as f32))
        }
    }

    /// Upstream `Tiltrotor::update_yaw_target`.
    ///
    /// `now_ms` is `AP_HAL::millis()`. `pilot_yaw_rate_cds` is
    /// `get_pilot_input_yaw_rate_cds()`. `yaw_sensor_cd` is
    /// `ahrs.yaw_sensor`. `airspeed` is EAS when available.
    /// `nav_roll_cd` / `airspeed_min` are the coordinated-turn inputs.
    pub fn update_yaw_target(
        &mut self,
        now_ms: u32,
        pilot_yaw_rate_cds: f32,
        yaw_sensor_cd: f32,
        airspeed: Option<f32>,
        nav_roll_cd: i32,
        airspeed_min: f32,
    ) {
        let elapsed = now_ms.wrapping_sub(self.transition_yaw_set_ms);
        if elapsed > TRANSITION_YAW_LOCK_MS || !is_zero_f32(pilot_yaw_rate_cds) {
            self.transition_yaw_cd = yaw_sensor_cd;
        }
        if let Some(aspeed) = airspeed {
            let roll_abs = if nav_roll_cd < 0 {
                -nav_roll_cd
            } else {
                nav_roll_cd
            };
            if roll_abs > COORD_TURN_ROLL_CD_MIN {
                let dt = elapsed as f32 * 0.001;
                let amin = if airspeed_min > 5.0 { airspeed_min } else { 5.0 };
                let v = if aspeed > amin { aspeed } else { amin };
                let yaw_rate_cds = fixedwing_turn_rate_dps(nav_roll_cd as f32 * 0.01, v) * 100.0;
                self.transition_yaw_cd += yaw_rate_cds * dt;
            }
        }
        self.transition_yaw_set_ms = now_ms;
    }

    fn snapshot_out(
        &self,
        ran_motor_mask: bool,
        motor_mask_throttle: f32,
        motor_mask: i16,
        motor_tilt_scaled: Option<f32>,
        ran_vectoring: bool,
    ) -> TiltUpdateOut {
        TiltUpdateOut {
            motors_active: self.motors_active,
            current_throttle: self.current_throttle,
            current_tilt: self.current_tilt,
            ran_motor_mask,
            motor_mask_throttle,
            motor_mask,
            motor_tilt_scaled,
            ran_vectoring,
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

const fn is_zero_f32(v: f32) -> bool {
    abs_f32(v) < f32::EPSILON
}

const fn min_f32(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}

const fn deg_to_rad(d: f32) -> f32 {
    d * (core::f32::consts::PI / 180.0)
}

const fn rad_to_deg(r: f32) -> f32 {
    r * (180.0 / core::f32::consts::PI)
}

fn fixedwing_turn_rate_dps(bank_angle_deg: f32, airspeed: f32) -> f32 {
    let bank = constrain_f32(bank_angle_deg, -80.0, 80.0);
    let v = if airspeed > 1.0 { airspeed } else { 1.0 };
    rad_to_deg(TILT_GRAVITY_MSS * libm::tanf(deg_to_rad(bank)) / v)
}
