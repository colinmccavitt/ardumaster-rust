//! Tailsitter enable and input-type stub, upstream `ArduPlane/tailsitter.*`.
//!
//! Tracked as **VT-007**. This slice is the gate that decides whether the
//! tailsitter object is live, and which of the two airframe input types it
//! is flying.
//!
//! # When it is enabled
//!
//! Upstream `Tailsitter::enabled` is `(enable > 0) && setup_complete`.
//! `setup()` fills `enable` from the old heuristic when `Q_TAILSIT_ENABLE`
//! was never written: a QuadPlane whose `Q_FRAME_CLASS` is
//! [`MOTOR_FRAME_TAILSITTER`] (or whose `Q_TAILSIT_MOTMX` motor mask is
//! non-zero) is a tailsitter, so enable is saved as 1. An explicit zero
//! stays off. `setup_complete` is set only when enable ends up positive —
//! calling `setup` on a disabled object does not make `enabled()` true.
//!
//! Bicopter tiltrotors are excluded from that heuristic upstream
//! (`tiltrotor.type != TILT_TYPE_BICOPTER`). That check lives with the
//! tiltrotor port; this stub treats the tiltrotor as not a bicopter.
//!
//! # The two input types
//!
//! Duo-motor tailsitters take attitude through one of two paths:
//!
//! - [`InputType::VectoredYaw`] — tilt motors vector thrust for yaw (and
//!   pitch). Upstream `_is_vectored`: tailsitter frame, non-zero
//!   `Q_TAILSIT_VHGAIN`, and a tilt-motor servo assigned.
//! - [`InputType::ControlSurfaces`] — elevator / aileron / rudder, or
//!   elevon / V-tail. Upstream `is_control_surface_tailsitter`: tailsitter
//!   frame, and either zero hover gain or no left tilt motor.
//!
//! The vectored-yaw tilt mix is [`VectoredYawMix`] (`Q_TAILSIT_VHGAIN` /
//! `Q_TAILSIT_VFGAIN`). Hover stick remapping is [`Tailsitter::check_input`]
//! (`Q_TAILSIT_INPUT` PlaneMode / BodyFrameRoll).

/// `Q_FRAME_CLASS` value that selects a duo-motor tailsitter, upstream
/// `AP_Motors::MOTOR_FRAME_TAILSITTER`.
pub const MOTOR_FRAME_TAILSITTER: u8 = 10;

/// Default `Q_TAILSIT_ENABLE`, upstream `AP_GROUPINFO_FLAGS("ENABLE", ...)`.
pub const TAILSIT_ENABLE_DEFAULT: i8 = 0;

/// Default `Q_TAILSIT_VHGAIN`, upstream `AP_GROUPINFO("VHGAIN", ..., 0.5)`.
pub const VECTORED_HOVER_GAIN_DEFAULT: f32 = 0.5;

/// Default `Q_TAILSIT_INPUT`, upstream `AP_GROUPINFO("INPUT", ..., 0)`.
pub const TAILSIT_INPUT_DEFAULT: i8 = 0;

/// Bit 0 of `Q_TAILSIT_INPUT` — PlaneMode stick swap.
///
/// Upstream `TAILSITTER_INPUT_PLANE = (1U<<0)`.
pub const TAILSITTER_INPUT_PLANE: u8 = 1 << 0;

/// Bit 1 of `Q_TAILSIT_INPUT` — body-frame roll when flying level.
///
/// Upstream `TAILSITTER_INPUT_BF_ROLL = (1U<<1)`.
pub const TAILSITTER_INPUT_BF_ROLL: u8 = 1 << 1;

/// Upstream `FLT_EPSILON` as used by `is_zero` in `AP_Math`.
const FLT_EPSILON: f32 = 1.192_092_90e-7;

fn is_zero(v: f32) -> bool {
    v.abs() < FLT_EPSILON
}

/// How a duo-motor tailsitter takes yaw / pitch input.
///
/// These are the airframe paths, not the `Q_TAILSIT_INPUT` stick bitmask
/// ([`TailsitInput`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputType {
    /// Tilt motors vector thrust for yaw. Upstream `_is_vectored`.
    VectoredYaw,
    /// Flying surfaces only. Upstream `is_control_surface_tailsitter`.
    ControlSurfaces,
}

/// Hover stick convention, upstream `Q_TAILSIT_INPUT`.
///
/// Bit 0 is PlaneMode, bit 1 is BodyFrameRoll. [`Tailsitter::check_input`]
/// remaps `control_in` only when PlaneMode is set and the tailsitter is
/// [`Tailsitter::active`]. BodyFrameRoll does not swap the sticks; it
/// selects body-frame roll in the attitude controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TailsitInput {
    /// Bitmask 0. Roll stick = earth-frame roll, yaw stick = earth-frame yaw.
    Multicopters,
    /// Bit 0 only. `check_input` swaps: roll' = yaw, yaw' = -roll.
    PlaneMode,
    /// Bit 1 only. No `check_input` swap; attitude uses body-frame roll.
    BodyFrameRoll,
    /// Both bits. `check_input` swap plus pitch-rotated body-frame roll.
    PlaneModeBodyFrameRoll,
}

/// What `Tailsitter::setup` reads off QuadPlane and the tailsitter params.
#[derive(Debug, Clone, Copy)]
pub struct TailsitterConfig {
    /// `Q_TAILSIT_ENABLE` when the parameter has been written.
    ///
    /// `None` means unconfigured: `setup` applies the frame-class heuristic.
    pub enable: Option<i8>,
    /// `Q_FRAME_CLASS`, upstream `quadplane.frame_class`.
    pub frame_class: u8,
    /// `Q_TAILSIT_MOTMX`, upstream `motor_mask`. Non-zero is a copter tailsitter.
    pub motor_mask: u16,
    /// `Q_TAILSIT_VHGAIN`, upstream `vectored_hover_gain`.
    pub vectored_hover_gain: f32,
    /// `SRV_Channel::k_tiltMotorLeft` assigned.
    pub tilt_motor_left: bool,
    /// `SRV_Channel::k_tiltMotorRight` assigned.
    pub tilt_motor_right: bool,
    /// `Q_TAILSIT_INPUT`, upstream `AP_Int8 input_type`.
    pub input: i8,
}

impl TailsitterConfig {
    /// A disabled, unconfigured tailsitter on a non-tailsitter frame.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            enable: None,
            frame_class: 0,
            motor_mask: 0,
            vectored_hover_gain: VECTORED_HOVER_GAIN_DEFAULT,
            tilt_motor_left: false,
            tilt_motor_right: false,
            input: TAILSIT_INPUT_DEFAULT,
        }
    }

    /// Unconfigured enable on a duo-motor tailsitter frame.
    #[must_use]
    pub const fn tailsitter_frame() -> Self {
        Self {
            enable: None,
            frame_class: MOTOR_FRAME_TAILSITTER,
            motor_mask: 0,
            vectored_hover_gain: VECTORED_HOVER_GAIN_DEFAULT,
            tilt_motor_left: false,
            tilt_motor_right: false,
            input: TAILSIT_INPUT_DEFAULT,
        }
    }
}

impl Default for TailsitterConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// The tailsitter object, upstream `class Tailsitter`.
#[derive(Debug, Clone, Copy)]
pub struct Tailsitter {
    enable: i8,
    setup_complete: bool,
    frame_class: u8,
    vectored_hover_gain: f32,
    tilt_motor_left: bool,
    tilt_motor_right: bool,
    input: i8,
}

impl Tailsitter {
    /// Run upstream `Tailsitter::setup` and return the resulting object.
    ///
    /// Does not persist parameters (`set_and_save`); the caller owns that.
    #[must_use]
    pub fn setup(cfg: TailsitterConfig) -> Self {
        let mut enable = cfg.enable.unwrap_or(TAILSIT_ENABLE_DEFAULT);
        if cfg.enable.is_none()
            && (cfg.frame_class == MOTOR_FRAME_TAILSITTER || cfg.motor_mask != 0)
        {
            enable = 1;
        }

        let setup_complete = enable > 0;
        Self {
            enable,
            setup_complete,
            frame_class: cfg.frame_class,
            vectored_hover_gain: cfg.vectored_hover_gain,
            tilt_motor_left: cfg.tilt_motor_left,
            tilt_motor_right: cfg.tilt_motor_right,
            input: cfg.input,
        }
    }

    /// Current `Q_TAILSIT_ENABLE` after setup.
    #[must_use]
    pub const fn enable(&self) -> i8 {
        self.enable
    }

    /// Upstream `Tailsitter::enabled` — `(enable > 0) && setup_complete`.
    #[must_use]
    pub const fn enabled(&self) -> bool {
        self.enable > 0 && self.setup_complete
    }

    /// Upstream `_is_vectored`.
    ///
    /// True when the frame is a tailsitter, hover gain is non-zero, and
    /// either tilt motor is assigned.
    #[must_use]
    pub fn is_vectored(&self) -> bool {
        self.frame_class == MOTOR_FRAME_TAILSITTER
            && !is_zero(self.vectored_hover_gain)
            && (self.tilt_motor_left || self.tilt_motor_right)
    }

    /// Upstream `Tailsitter::is_control_surface_tailsitter`.
    ///
    /// True when the frame is a tailsitter and either hover gain is zero
    /// or the *left* tilt motor is unassigned. The left-only check is
    /// upstream's, not a simplification.
    #[must_use]
    pub fn is_control_surface_tailsitter(&self) -> bool {
        self.frame_class == MOTOR_FRAME_TAILSITTER
            && (is_zero(self.vectored_hover_gain) || !self.tilt_motor_left)
    }

    /// Which airframe input path this tailsitter is flying, if it is live.
    ///
    /// Vectored yaw wins when both predicates can be true (gain set, only
    /// the right tilt motor assigned). A copter tailsitter enabled via
    /// motor mask on a non-tailsitter frame has neither path.
    #[must_use]
    pub fn input_type(&self) -> Option<InputType> {
        if !self.enabled() {
            return None;
        }
        if self.is_vectored() {
            Some(InputType::VectoredYaw)
        } else if self.is_control_surface_tailsitter() {
            Some(InputType::ControlSurfaces)
        } else {
            None
        }
    }

    /// Current `Q_TAILSIT_INPUT` bitmask.
    #[must_use]
    pub const fn input(&self) -> i8 {
        self.input
    }

    /// Bit 0 of `Q_TAILSIT_INPUT`. Upstream `TAILSITTER_INPUT_PLANE`.
    #[must_use]
    pub const fn plane_mode(&self) -> bool {
        (self.input as u8) & TAILSITTER_INPUT_PLANE != 0
    }

    /// Bit 1 of `Q_TAILSIT_INPUT`. Upstream `TAILSITTER_INPUT_BF_ROLL`.
    #[must_use]
    pub const fn body_frame_roll(&self) -> bool {
        (self.input as u8) & TAILSITTER_INPUT_BF_ROLL != 0
    }

    /// Decode `Q_TAILSIT_INPUT` into a hover-stick convention.
    #[must_use]
    pub const fn tailsit_input(&self) -> TailsitInput {
        match (self.plane_mode(), self.body_frame_roll()) {
            (false, false) => TailsitInput::Multicopters,
            (true, false) => TailsitInput::PlaneMode,
            (false, true) => TailsitInput::BodyFrameRoll,
            (true, true) => TailsitInput::PlaneModeBodyFrameRoll,
        }
    }

    /// Upstream `Tailsitter::active`.
    ///
    /// True when enabled and either in a VTOL mode or in the
    /// `ANGLE_WAIT_FW` fixed-wing transition.
    #[must_use]
    pub const fn active(&self, in_vtol_mode: bool, angle_wait_fw: bool) -> bool {
        self.enabled() && (in_vtol_mode || angle_wait_fw)
    }

    /// Upstream `Tailsitter::check_input`.
    ///
    /// When active and PlaneMode, swap the roll and rudder `control_in`
    /// values so the roll stick commands earth-frame yaw and the rudder
    /// commands earth-frame roll: roll becomes yaw, yaw becomes -roll.
    /// BodyFrameRoll does not change this swap.
    #[must_use]
    pub fn check_input(
        &self,
        roll: i16,
        yaw: i16,
        in_vtol_mode: bool,
        angle_wait_fw: bool,
    ) -> (i16, i16) {
        if self.active(in_vtol_mode, angle_wait_fw) && self.plane_mode() {
            let remapped_yaw = i16::try_from(-i32::from(roll)).unwrap_or(i16::MIN);
            (yaw, remapped_yaw)
        } else {
            (roll, yaw)
        }
    }
}

/// Default `Q_TAILSIT_VFGAIN`, upstream `AP_GROUPINFO("VFGAIN", ..., 0)`.
pub const VECTORED_FORWARD_GAIN_DEFAULT: f32 = 0.0;

/// Default `Q_TAILSIT_VHPOW`, upstream `AP_GROUPINFO("VHPOW", ..., 2.5)`.
pub const VECTORED_HOVER_POWER_DEFAULT: f32 = 2.5;

/// Upstream `SERVO_MAX` / `SERVO_OUTPUT_RANGE` for tilt motors and surfaces.
pub const SERVO_MAX: f32 = 4500.0;

/// Vectored-yaw tilt-motor mix, upstream `Tailsitter::output` + motors tilt.
///
/// Tracked as **VT-007**. Duo-motor tailsitters vector thrust for yaw and
/// pitch by tilting the motors. `AP_MotorsTailsitter` writes
/// `tilt_left = pitch - yaw`, `tilt_right = pitch + yaw` (each -1..1,
/// then `* SERVO_MAX`). `Tailsitter::output` then applies the gains:
///
/// - Hover / VTOL: no tilt unless [`Q_TAILSIT_VHGAIN`](VECTORED_HOVER_GAIN_DEFAULT)
///   is positive; then `extra_elevator + tilt * VHGAIN` (assist path also
///   multiplies the hover/throttle scaler). Extra elevator is a power-law
///   of the pitch error (`Q_TAILSIT_VHPOW`) so the motors can point up
///   for takeoff without integrator windup.
/// - Forward flight: no tilt unless `Q_TAILSIT_VFGAIN` is positive; then
///   `(elevator ± aileron) * VFGAIN * scaler`.
///
/// This is the tailsitter-specific mix, not a rewrite of ap-motors mixing.
#[derive(Debug, Clone, Copy)]
pub struct VectoredYawMix {
    hover_gain: f32,
    forward_gain: f32,
    hover_power: f32,
}

impl Default for VectoredYawMix {
    fn default() -> Self {
        Self::new()
    }
}

impl VectoredYawMix {
    /// `AP_GROUPINFO` defaults for VHGAIN / VFGAIN / VHPOW.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            hover_gain: VECTORED_HOVER_GAIN_DEFAULT,
            forward_gain: VECTORED_FORWARD_GAIN_DEFAULT,
            hover_power: VECTORED_HOVER_POWER_DEFAULT,
        }
    }

    /// `Q_TAILSIT_VHGAIN`.
    #[must_use]
    pub const fn hover_gain(&self) -> f32 {
        self.hover_gain
    }

    /// `Q_TAILSIT_VFGAIN`.
    #[must_use]
    pub const fn forward_gain(&self) -> f32 {
        self.forward_gain
    }

    /// `Q_TAILSIT_VHPOW`.
    #[must_use]
    pub const fn hover_power(&self) -> f32 {
        self.hover_power
    }

    /// Poke `Q_TAILSIT_VHGAIN`.
    pub fn set_hover_gain(&mut self, gain: f32) {
        self.hover_gain = gain;
    }

    /// Poke `Q_TAILSIT_VFGAIN`.
    pub fn set_forward_gain(&mut self, gain: f32) {
        self.forward_gain = gain;
    }

    /// Poke `Q_TAILSIT_VHPOW`.
    pub fn set_hover_power(&mut self, power: f32) {
        self.hover_power = power;
    }

    /// Hover / VTOL tilt from motors pitch and yaw in `-1..1`.
    ///
    /// `tilt_left = (pitch - yaw) * SERVO_MAX * VHGAIN`,
    /// `tilt_right = (pitch + yaw) * SERVO_MAX * VHGAIN`.
    /// Zero (or negative) VHGAIN produces zero tilt.
    #[must_use]
    pub fn hover_tilt(&self, pitch: f32, yaw: f32) -> (f32, f32) {
        self.mix_hover(pitch, yaw, 0.0, 1.0)
    }

    /// Hover mix with extra elevator and the assist throttle scaler.
    ///
    /// Upstream assist path is `tilt * VHGAIN * throttle_scaler` with no
    /// extra elevator. The main VTOL path is
    /// `extra_elevator + tilt * VHGAIN` (scaler 1).
    #[must_use]
    pub fn mix_hover(
        &self,
        pitch: f32,
        yaw: f32,
        extra_elevator: f32,
        throttle_scaler: f32,
    ) -> (f32, f32) {
        if !is_positive(self.hover_gain) {
            return (0.0, 0.0);
        }
        let left = (pitch - yaw) * SERVO_MAX;
        let right = (pitch + yaw) * SERVO_MAX;
        (
            extra_elevator + left * self.hover_gain * throttle_scaler,
            extra_elevator + right * self.hover_gain * throttle_scaler,
        )
    }

    /// Forward-flight tilt from already-scaled elevator / aileron.
    ///
    /// `tilt_left = (elevator + aileron) * VFGAIN * scaler`,
    /// `tilt_right = (elevator - aileron) * VFGAIN * scaler`.
    /// Zero (or negative) VFGAIN produces zero tilt.
    #[must_use]
    pub fn mix_forward(&self, elevator: f32, aileron: f32, scaler: f32) -> (f32, f32) {
        if !is_positive(self.forward_gain) {
            return (0.0, 0.0);
        }
        (
            (elevator + aileron) * self.forward_gain * scaler,
            (elevator - aileron) * self.forward_gain * scaler,
        )
    }

    /// Extra elevator from the halved pitch error, upstream `Tailsitter::output`.
    ///
    /// `extra_pitch = constrain(pitch_error_cd, ±SERVO_MAX) / SERVO_MAX`,
    /// then `sign * |extra_pitch|^VHPOW * SERVO_MAX` when the error is
    /// non-zero and the airframe is in a VTOL mode. Zero VHGAIN skips it.
    #[must_use]
    pub fn extra_elevator(&self, pitch_error_cd: f32, in_vtol_mode: bool) -> f32 {
        if !is_positive(self.hover_gain) {
            return 0.0;
        }
        let extra_pitch = constrain_f32(pitch_error_cd, -SERVO_MAX, SERVO_MAX) / SERVO_MAX;
        if is_zero(extra_pitch) || !in_vtol_mode {
            return 0.0;
        }
        let extra_sign = if extra_pitch > 0.0 { 1.0 } else { -1.0 };
        extra_sign * libm::powf(extra_pitch.abs(), self.hover_power) * SERVO_MAX
    }
}

fn is_positive(v: f32) -> bool {
    v > 0.0
}

fn constrain_f32(v: f32, min: f32, max: f32) -> f32 {
    v.clamp(min, max)
}
