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
//! (`Q_TAILSIT_INPUT` PlaneMode / BodyFrameRoll). Forward-flight lift-motor
//! hold is [`Tailsitter::output_kind`] / [`mask_motor_actuator`]
//! (`Q_TAILSIT_MOTMX` / `output_motor_mask`).
//! Control-surface speed scaling is [`GainScaling`] (`Q_TAILSIT_GSCMSK`).
//! Pitch-relax after a hover-gain vectored setup is [`Tailsitter::relax_pitch`].
//! Post-transition pitch-forward / pitch-down rate-limit is [`PitchLimit`]
//! (`set_VTOL_roll_pitch_limit` / the `fw_limit_*` leftover of
//! `set_FW_roll_pitch`). [`in_vtol_transition`] is the 1 s
//! `last_vtol_mode_ms` window used by that leftover and by
//! [`PitchLimit::allow_stick_mixing`].

/// `Q_FRAME_CLASS` value that selects a duo-motor tailsitter, upstream
/// `AP_Motors::MOTOR_FRAME_TAILSITTER`.
pub const MOTOR_FRAME_TAILSITTER: u8 = 10;

/// Default `Q_TAILSIT_ENABLE`, upstream `AP_GROUPINFO_FLAGS("ENABLE", ...)`.
pub const TAILSIT_ENABLE_DEFAULT: i8 = 0;

/// Default `Q_TAILSIT_VHGAIN`, upstream `AP_GROUPINFO("VHGAIN", ..., 0.5)`.
pub const VECTORED_HOVER_GAIN_DEFAULT: f32 = 0.5;

/// Default `Q_TAILSIT_INPUT`, upstream `AP_GROUPINFO("INPUT", ..., 0)`.
pub const TAILSIT_INPUT_DEFAULT: i8 = 0;

/// Default `Q_TAILSIT_MOTMX`, upstream `AP_GROUPINFO("MOTMX", ..., 0)`.
pub const TAILSIT_MOTMX_DEFAULT: u16 = 0;

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

/// Which path `Tailsitter::output` takes this cycle.
///
/// Copter tailsitters keep selected motors spinning in forward flight
/// via `Q_TAILSIT_MOTMX` and `AP_MotorsMulticopter::output_motor_mask`.
/// Duo-motor tailsitters typically leave the mask at 0; the call still
/// happens, it just writes no motors. The vectored tilt mix after that
/// call is [`VectoredYawMix`], not this enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputKind {
    /// Early return: `!enabled() || motor_test || !quadplane.initialised`.
    Silent,
    /// FW / VTOL-transition and not assisted: `output_motor_mask`.
    MotorMask,
    /// Active VTOL, or assisted flight: copter `motors_output`.
    Copter,
}

/// Inputs `Tailsitter::output` reads besides its own params.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OutputContext {
    /// `quadplane.initialised`.
    pub initialised: bool,
    /// `quadplane.motor_test.running`.
    pub motor_test: bool,
    /// `quadplane.in_vtol_mode()`.
    pub in_vtol_mode: bool,
    /// `transition_state == ANGLE_WAIT_FW` — [`Tailsitter::active`].
    pub angle_wait_fw: bool,
    /// `Tailsitter::in_vtol_transition`.
    pub in_vtol_transition: bool,
    /// `quadplane.assisted_flight`.
    pub assisted_flight: bool,
}

impl OutputContext {
    /// Initialised QuadPlane, no motor test, FW cruise, not assisted.
    #[must_use]
    pub const fn fw_cruise() -> Self {
        Self {
            initialised: true,
            motor_test: false,
            in_vtol_mode: false,
            angle_wait_fw: false,
            in_vtol_transition: false,
            assisted_flight: false,
        }
    }

    /// Initialised QuadPlane in a Q* hover, not transitioning.
    #[must_use]
    pub const fn vtol_hover() -> Self {
        Self {
            initialised: true,
            motor_test: false,
            in_vtol_mode: true,
            angle_wait_fw: false,
            in_vtol_transition: false,
            assisted_flight: false,
        }
    }
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
    motor_mask: u16,
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
            motor_mask: cfg.motor_mask,
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

    /// Current `Q_TAILSIT_MOTMX` after setup.
    #[must_use]
    pub const fn motor_mask(&self) -> u16 {
        self.motor_mask
    }

    /// Upstream `Tailsitter::output` path this cycle.
    ///
    /// Silent when disabled, QuadPlane is not initialised, or a motor
    /// test is running (the test must not be overwritten). MotorMask
    /// when not [`Tailsitter::active`] or in the FW→VTOL transition,
    /// unless assisted flight has already taken the copter path.
    /// Copter otherwise (Q* hover, or assisted FW).
    #[must_use]
    pub const fn output_kind(&self, ctx: OutputContext) -> OutputKind {
        if !self.enabled() || !ctx.initialised || ctx.motor_test {
            return OutputKind::Silent;
        }
        if (!self.active(ctx.in_vtol_mode, ctx.angle_wait_fw) || ctx.in_vtol_transition)
            && !ctx.assisted_flight
        {
            return OutputKind::MotorMask;
        }
        OutputKind::Copter
    }

    /// Whether `output` calls `motors->output_min` before the path.
    ///
    /// Disarm / emergency-stop still take [`OutputKind`]; they do not
    /// become Silent. Upstream writes min, then continues.
    #[must_use]
    pub const fn output_min_first(soft_armed: bool, emergency_stop: bool) -> bool {
        !soft_armed || emergency_stop
    }

    /// Upstream `Tailsitter::relax_pitch`.
    ///
    /// Vectored belly-sitters keep pitch tight so the motors stay
    /// pointed and the props do not hit the ground. Everyone else
    /// relaxes. After a FW to VTOL rate-limit has started
    /// (`vtol_limit_start_ms != 0`) pitch is always relaxed, even on
    /// a vectored airframe. Hover-gain leftover: `_is_vectored` is
    /// the same predicate as [`Self::is_vectored`] (`Q_TAILSIT_VHGAIN`
    /// plus a tilt motor).
    #[must_use]
    pub fn relax_pitch(&self, vtol_limit_start_ms: u32) -> bool {
        !self.enabled() || !self.is_vectored() || vtol_limit_start_ms != 0
    }
}

/// Whether motor `i` stays live in FW under `Q_TAILSIT_MOTMX`.
///
/// Upstream `output_motor_mask` walks `AP_MOTORS_MAX_NUM_MOTORS` and
/// writes only motors that are enabled *and* have the mask bit set.
/// This stub is the mask-bit half (`mask & (1U << i)`).
#[must_use]
pub const fn motor_in_fw_mask(mask: u16, motor: u8) -> bool {
    motor < 16 && (mask & (1u16 << motor)) != 0
}

/// Actuator for one motor on the FW mask path.
///
/// Upstream `AP_MotorsMulticopter::output_motor_mask`: motors not in
/// the mask are not written (`None`). Motors in the mask get
/// `thrust + roll_factor * rudder_dt * 0.5` when armed and interlocked,
/// otherwise zero throttle. Copter-frame roll is plane-frame yaw.
#[must_use]
pub fn mask_motor_actuator(
    mask: u16,
    motor: u8,
    thrust: f32,
    roll_factor: f32,
    rudder_dt: f32,
    armed_interlock: bool,
) -> Option<f32> {
    if !motor_in_fw_mask(mask, motor) {
        return None;
    }
    if armed_interlock {
        Some(thrust + roll_factor * rudder_dt * 0.5)
    } else {
        Some(0.0)
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

/// Bit 0 of `Q_TAILSIT_GSCMSK` — scale gains with throttle.
///
/// Upstream `TAILSITTER_GSCL_THROTTLE`.
pub const TAILSITTER_GSCL_THROTTLE: u16 = 1 << 0;

/// Bit 1 of `Q_TAILSIT_GSCMSK` — reduce gain at high throttle / tilt.
///
/// Upstream `TAILSITTER_GSCL_ATT_THR`.
pub const TAILSITTER_GSCL_ATT_THR: u16 = 1 << 1;

/// Bit 2 of `Q_TAILSIT_GSCMSK` — disk-theory velocity scaling.
///
/// Upstream `TAILSITTER_GSCL_DISK_THEORY`. Requires a
/// positive `Q_TAILSIT_DSKLD`.
pub const TAILSITTER_GSCL_DISK_THEORY: u16 = 1 << 2;

/// Bit 3 of `Q_TAILSIT_GSCMSK` — scale with air density.
///
/// Upstream `TAILSITTER_GSCL_ALTITUDE`.
pub const TAILSITTER_GSCL_ALTITUDE: u16 = 1 << 3;

/// Default `Q_TAILSIT_GSCMSK`, upstream `TAILSITTER_GSCL_THROTTLE`.
pub const GAIN_SCALING_MASK_DEFAULT: u16 = TAILSITTER_GSCL_THROTTLE;

/// Default `Q_TAILSIT_DSKLD`, upstream `AP_GROUPINFO("DSKLD", ..., 0)`.
pub const DISK_LOADING_DEFAULT: f32 = 0.0;

/// Sea-level air density used by the disk-theory path.
///
/// Upstream `SSL_AIR_DENSITY` in `AP_Math/definitions.h`.
pub const SSL_AIR_DENSITY: f32 = 1.225;

/// Standard gravity used by the disk-theory path.
///
/// Upstream `GRAVITY_MSS`.
pub const GRAVITY_MSS: f32 = 9.806_65;

/// `cosf(0.125 * pi)` — tilt angle where ATT_THR starts ramping down.
///
/// Upstream `constexpr float c_trans_angle = 0.9238795`.
pub const ATT_THR_C_TRANS_ANGLE: f32 = 0.923_879_5;

/// ATT_THR positive slew time-constant, seconds. Upstream `posTC`.
pub const ATT_THR_POS_TC: f32 = 2.0;

/// ATT_THR negative slew time-constant, seconds. Upstream `negTC`.
pub const ATT_THR_NEG_TC: f32 = 1.0;

/// Which `speed_scaling` branch produces `spd_scaler`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GainScalePath {
    /// ATT_THR bit set. Tilt + high-throttle attenuation, then slew.
    AttThr,
    /// DISK_THEORY bit, positive `Q_TAILSIT_DSKLD`, and an airspeed estimate.
    DiskTheory,
    /// DISK_THEORY bit and positive DSKLD, but no airspeed — throttle fallback.
    DiskTheoryFallback,
    /// THROTTLE bit set. The GROUPINFO default.
    Throttle,
    /// No producing bit. `spd_scaler` stays 1.
    Unity,
}

/// Inputs `Tailsitter::speed_scaling` reads besides its own params.
#[derive(Debug, Clone, Copy)]
pub struct SpeedScaleInput {
    /// `motors->get_throttle_hover()`.
    pub hover: f32,
    /// `motors->get_throttle_out()`.
    pub throttle: f32,
    /// `ahrs_view->get_rotation_body_to_ned().c.z`.
    pub c_tilt: f32,
    /// `quadplane.ahrs.airspeed_EAS` succeeded.
    pub have_airspeed: bool,
    /// EAS from that call; unused when [`Self::have_airspeed`] is false.
    pub airspeed: f32,
    /// `ahrs.get_air_density_ratio()`. 1.0 at sea level.
    pub density_ratio: f32,
    /// `plane.G_Dt`, seconds.
    pub dt_s: f32,
}

impl SpeedScaleInput {
    /// Hover 0.4, mid throttle, level tilt, sea-level, 400 Hz loop.
    #[must_use]
    pub const fn hover_level() -> Self {
        Self {
            hover: 0.4,
            throttle: 0.4,
            c_tilt: 1.0,
            have_airspeed: false,
            airspeed: 0.0,
            density_ratio: 1.0,
            dt_s: 0.002_5,
        }
    }
}

/// `spd_scaler` / `throttle_scaler` from one `speed_scaling` cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpeedScaleOutput {
    /// Which branch produced `speed_scaler` before the altitude multiply.
    pub path: GainScalePath,
    /// Hover/throttle ratio, used on the tilt motors and the THROTTLE path.
    pub throttle_scaler: f32,
    /// Applied to aileron / elevator / rudder (and logged as `speed_scaler`).
    pub speed_scaler: f32,
}

/// Control-surface speed scaling, upstream `Tailsitter::speed_scaling`.
///
/// Tracked as **VT-007**. `Q_TAILSIT_GSCMSK` selects the method:
///
/// - Throttle (default): `spd_scaler = constrain(hover/throttle, GSCMIN, GSCMAX)`.
/// - ATT_THR: reduce throws at large tilt (`c.z` below
///   [`ATT_THR_C_TRANS_ANGLE`]) and high throttle, then slew with
///   [`ATT_THR_POS_TC`] / [`ATT_THR_NEG_TC`]. If the result is still >= 1
///   and the Throttle bit is also set, take `MAX(throttle_scaler, 1)`.
/// - Disk theory: `Ue^2` from disk loading; no airspeed falls back to
///   the throttle scaler. Positive DSKLD is required.
/// - Altitude: after any of the above, divide by the air-density ratio.
///
/// Tilt motors always take `throttle_scaler`; flying surfaces take
/// `spd_scaler`. The hover/throttle formula itself already lives on
/// [`crate::transition::TransitionRamp::throttle_scaler`]; this object
/// is the GSCMASK dispatch the ramp stub deferred.
#[derive(Debug, Clone, Copy)]
pub struct GainScaling {
    mask: u16,
    throttle_scale_max: f32,
    gain_scaling_min: f32,
    disk_loading: f32,
    last_spd_scaler: f32,
}

impl Default for GainScaling {
    fn default() -> Self {
        Self::new()
    }
}

impl GainScaling {
    /// `AP_GROUPINFO` defaults for GSCMSK / GSCMAX / GSCMIN / DSKLD.
    ///
    /// `last_spd_scaler` starts at 1, matching the field initializer.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            mask: GAIN_SCALING_MASK_DEFAULT,
            throttle_scale_max: crate::transition::THROTTLE_SCALE_MAX_DEFAULT,
            gain_scaling_min: crate::transition::GAIN_SCALING_MIN_DEFAULT,
            disk_loading: DISK_LOADING_DEFAULT,
            last_spd_scaler: 1.0,
        }
    }

    /// Current `Q_TAILSIT_GSCMSK`.
    #[must_use]
    pub const fn mask(&self) -> u16 {
        self.mask
    }

    /// Poke `Q_TAILSIT_GSCMSK`.
    pub fn set_mask(&mut self, mask: u16) {
        self.mask = mask;
    }

    /// `Q_TAILSIT_GSCMAX`.
    #[must_use]
    pub const fn throttle_scale_max(&self) -> f32 {
        self.throttle_scale_max
    }

    /// `Q_TAILSIT_GSCMIN`.
    #[must_use]
    pub const fn gain_scaling_min(&self) -> f32 {
        self.gain_scaling_min
    }

    /// `Q_TAILSIT_DSKLD`.
    #[must_use]
    pub const fn disk_loading(&self) -> f32 {
        self.disk_loading
    }

    /// Poke `Q_TAILSIT_DSKLD`.
    pub fn set_disk_loading(&mut self, disk_loading: f32) {
        self.disk_loading = disk_loading;
    }

    /// Last slewed `spd_scaler`, upstream `last_spd_scaler`.
    #[must_use]
    pub const fn last_spd_scaler(&self) -> f32 {
        self.last_spd_scaler
    }

    /// Which branch `speed_scaling` would take for this mask / DSKLD.
    #[must_use]
    pub fn path(&self, have_airspeed: bool) -> GainScalePath {
        if self.mask & TAILSITTER_GSCL_ATT_THR != 0 {
            GainScalePath::AttThr
        } else if self.mask & TAILSITTER_GSCL_DISK_THEORY != 0 && is_positive(self.disk_loading) {
            if have_airspeed {
                GainScalePath::DiskTheory
            } else {
                GainScalePath::DiskTheoryFallback
            }
        } else if self.mask & TAILSITTER_GSCL_THROTTLE != 0 {
            GainScalePath::Throttle
        } else {
            GainScalePath::Unity
        }
    }

    /// Hover/throttle scaler, same formula as
    /// [`crate::transition::TransitionRamp::throttle_scaler`].
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

    /// ATT_THR tilt + high-throttle attenuation, before the slew.
    ///
    /// Level (`c.z = 1`) and throttle at or below `1.25 * hover` leaves
    /// the scaler at 1. Tilt past [`ATT_THR_C_TRANS_ANGLE`] ramps toward
    /// GSCMIN; throttle above the (possibly reduced) threshold multiplies
    /// a further attenuation.
    #[must_use]
    pub fn att_thr_pre_slew(&self, hover: f32, throttle: f32, c_tilt: f32) -> f32 {
        let min_scale = self.gain_scaling_min;
        let mut tthr = 1.25 * hover;
        let mut spd_scaler = 1.0;
        if c_tilt < ATT_THR_C_TRANS_ANGLE {
            let alpha = (1.0 - min_scale) / ATT_THR_C_TRANS_ANGLE;
            let beta = 1.0 - alpha * ATT_THR_C_TRANS_ANGLE;
            spd_scaler = constrain_f32(beta + alpha * c_tilt, min_scale, 1.0);
            tthr = 0.5 * hover;
        }
        if throttle > tthr {
            let throttle_atten = 1.0 - (throttle - tthr) / (1.0 - tthr);
            spd_scaler *= throttle_atten;
            spd_scaler = constrain_f32(spd_scaler, min_scale, 1.0);
        }
        spd_scaler
    }

    /// Limit the ATT_THR scaler to +/- `G_Dt / TC` of [`Self::last_spd_scaler`].
    pub fn slew(&mut self, target: f32, dt_s: f32) -> f32 {
        let posdelta = dt_s / ATT_THR_POS_TC;
        let negdelta = dt_s / ATT_THR_NEG_TC;
        let spd = constrain_f32(
            target,
            self.last_spd_scaler - negdelta,
            self.last_spd_scaler + posdelta,
        );
        self.last_spd_scaler = spd;
        spd
    }

    /// After ATT_THR slew: if `spd >= 1` and the Throttle bit is set,
    /// `MAX(throttle_scaler, 1)`.
    #[must_use]
    pub fn att_thr_maybe_throttle(&self, spd_scaler: f32, hover: f32, throttle: f32) -> f32 {
        if spd_scaler >= 1.0 && self.mask & TAILSITTER_GSCL_THROTTLE != 0 {
            self.throttle_scaler(hover, throttle).max(1.0)
        } else {
            spd_scaler
        }
    }

    /// Disk-theory `spd_scaler` when airspeed is known.
    ///
    /// `Ue^2_hover / Ue^2` with `Ue^2 = ((t/t_h) * DSKLD * g) / (0.5 rho) + U0^2`.
    /// Altitude bit uses sea-level density for the hover case.
    #[must_use]
    pub fn disk_theory_scaler(
        &self,
        hover: f32,
        throttle: f32,
        airspeed: f32,
        density_ratio: f32,
    ) -> f32 {
        let rho = SSL_AIR_DENSITY * density_ratio;
        let hover_rho = if self.mask & TAILSITTER_GSCL_ALTITUDE != 0 {
            SSL_AIR_DENSITY
        } else {
            rho
        };
        let sq_hover_outflow = (self.disk_loading * GRAVITY_MSS) / (0.5 * hover_rho);
        let u0 = if airspeed > 0.0 { airspeed } else { 0.0 };
        let thrust_term = if is_positive(hover) {
            (throttle / hover) * self.disk_loading * GRAVITY_MSS / (0.5 * rho)
        } else {
            0.0
        };
        let sq_outflow = thrust_term + u0 * u0;
        if is_positive(sq_outflow) {
            constrain_f32(
                sq_hover_outflow / sq_outflow,
                self.gain_scaling_min,
                self.throttle_scale_max,
            )
        } else {
            self.throttle_scale_max
        }
    }

    /// Altitude bit: divide by the air-density ratio.
    #[must_use]
    pub fn apply_altitude(&self, spd_scaler: f32, density_ratio: f32) -> f32 {
        if self.mask & TAILSITTER_GSCL_ALTITUDE != 0 {
            spd_scaler / density_ratio
        } else {
            spd_scaler
        }
    }

    /// One `speed_scaling` cycle. Updates [`Self::last_spd_scaler`] on ATT_THR.
    pub fn scale(&mut self, inp: &SpeedScaleInput) -> SpeedScaleOutput {
        let throttle_scaler = self.throttle_scaler(inp.hover, inp.throttle);
        let path = self.path(inp.have_airspeed);
        let mut speed_scaler = match path {
            GainScalePath::AttThr => {
                let pre = self.att_thr_pre_slew(inp.hover, inp.throttle, inp.c_tilt);
                let slewed = self.slew(pre, inp.dt_s);
                self.att_thr_maybe_throttle(slewed, inp.hover, inp.throttle)
            }
            GainScalePath::DiskTheory => {
                self.disk_theory_scaler(inp.hover, inp.throttle, inp.airspeed, inp.density_ratio)
            }
            GainScalePath::DiskTheoryFallback | GainScalePath::Throttle => throttle_scaler,
            GainScalePath::Unity => 1.0,
        };
        speed_scaler = self.apply_altitude(speed_scaler, inp.density_ratio);
        SpeedScaleOutput {
            path,
            throttle_scaler,
            speed_scaler,
        }
    }

    /// Flying-surface throw after `speed_scaling`. Tilt motors use
    /// [`Self::scale_tilt`] instead.
    #[must_use]
    pub const fn scale_surface(value: f32, speed_scaler: f32) -> f32 {
        value * speed_scaler
    }

    /// Tilt-motor throw after `speed_scaling` — always `throttle_scaler`.
    #[must_use]
    pub const fn scale_tilt(value: f32, throttle_scaler: f32) -> f32 {
        value * throttle_scaler
    }
}

/// Milliseconds after leaving FW before `in_vtol_transition(now)`
/// stops treating the airframe as “just came out of forward flight”.
///
/// Upstream `Tailsitter::in_vtol_transition`: `(now - last_vtol_mode_ms) > 1000`.
pub const LAST_VTOL_MODE_MS: u32 = 1000;

/// Upstream `Tailsitter::in_vtol_transition`.
///
/// False when the tailsitter is off or we are not in a Q* mode.
/// True while `transition_state == ANGLE_WAIT_VTOL`. When `now_ms` is
/// non-zero, also true if more than [`LAST_VTOL_MODE_MS`] has passed
/// since `last_vtol_mode_ms` — we have only just come out of forward
/// flight. `allow_stick_mixing` calls this with `now == 0`, so that
/// window is skipped there.
#[must_use]
pub const fn in_vtol_transition(
    enabled: bool,
    in_vtol_mode: bool,
    angle_wait_vtol: bool,
    now_ms: u32,
    last_vtol_mode_ms: u32,
) -> bool {
    if !enabled || !in_vtol_mode {
        return false;
    }
    if angle_wait_vtol {
        return true;
    }
    if now_ms != 0 && now_ms.wrapping_sub(last_vtol_mode_ms) > LAST_VTOL_MODE_MS {
        return true;
    }
    false
}

/// Post-transition pitch rate-limit, upstream
/// `Tailsitter_Transition::set_VTOL_roll_pitch_limit` and the
/// `fw_limit_*` leftover of `set_FW_roll_pitch`.
///
/// After FW → VTOL completes, [`Self::start_vtol`] stamps
/// `vtol_limit_start_ms` and pitch is not allowed to walk toward 0
/// faster than `Q_TAILSIT_RAT_VT`. After VTOL → FW completes,
/// [`Self::start_fw`] ramps pitch down from the completion attitude
/// at `Q_TAILSIT_RAT_FW` and never limits past 0 or to a smaller
/// (more nose-down) demand than the FW controller already asked for.
///
/// Stick mixing is blocked while [`in_vtol_transition`] is true
/// (nose-up) or while the FW pitch-down leftover is still active.
#[derive(Debug, Clone, Copy)]
pub struct PitchLimit {
    vtol_limit_start_ms: u32,
    vtol_limit_initial_pitch: f32,
    fw_limit_start_ms: u32,
    fw_limit_initial_pitch: f32,
    rate_vtol: f32,
    rate_fw: f32,
}

impl Default for PitchLimit {
    fn default() -> Self {
        Self::new()
    }
}

impl PitchLimit {
    /// Inactive limits at the default `Q_TAILSIT_RAT_VT` / `RAT_FW`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            vtol_limit_start_ms: 0,
            vtol_limit_initial_pitch: 0.0,
            fw_limit_start_ms: 0,
            fw_limit_initial_pitch: 0.0,
            rate_vtol: crate::transition::TRANSITION_RATE_VTOL_DEFAULT,
            rate_fw: crate::transition::TRANSITION_RATE_FW_DEFAULT,
        }
    }

    /// `vtol_limit_start_ms`. Zero means the VTOL leftover is idle.
    #[must_use]
    pub const fn vtol_limit_start_ms(&self) -> u32 {
        self.vtol_limit_start_ms
    }

    /// `fw_limit_start_ms`. Zero means the FW leftover is idle.
    #[must_use]
    pub const fn fw_limit_start_ms(&self) -> u32 {
        self.fw_limit_start_ms
    }

    /// `Q_TAILSIT_RAT_VT` (deg/s) used by the VTOL leftover.
    #[must_use]
    pub const fn rate_vtol(&self) -> f32 {
        self.rate_vtol
    }

    /// `Q_TAILSIT_RAT_FW` (deg/s) used by the FW leftover.
    #[must_use]
    pub const fn rate_fw(&self) -> f32 {
        self.rate_fw
    }

    /// Poke `Q_TAILSIT_RAT_VT`.
    pub fn set_rate_vtol(&mut self, rate: f32) {
        self.rate_vtol = rate;
    }

    /// Poke `Q_TAILSIT_RAT_FW`.
    pub fn set_rate_fw(&mut self, rate: f32) {
        self.rate_fw = rate;
    }

    /// Armed FW → VTOL complete: start the VTOL pitch-forward leftover.
    ///
    /// Upstream `VTOL_update` writes `vtol_limit_initial_pitch =
    /// ahrs_view->pitch_sensor` with no clamp.
    pub fn start_vtol(&mut self, now_ms: u32, initial_pitch_cd: f32) {
        self.vtol_limit_start_ms = now_ms;
        self.vtol_limit_initial_pitch = initial_pitch_cd;
    }

    /// Armed VTOL → FW complete: start the FW pitch-down leftover.
    ///
    /// Upstream `update` clamps the starting pitch to ±[`crate::transition::PITCH_CD_LIMIT`].
    pub fn start_fw(&mut self, now_ms: u32, initial_pitch_cd: f32) {
        self.fw_limit_start_ms = now_ms;
        let limit = crate::transition::PITCH_CD_LIMIT as f32;
        self.fw_limit_initial_pitch = constrain_f32(initial_pitch_cd, -limit, limit);
    }

    /// Upstream `Tailsitter_Transition::set_VTOL_roll_pitch_limit`.
    ///
    /// Returns `true` when the demand was limited (roll forced to 0,
    /// pitch held at the leftover). Returns `false` and clears
    /// `vtol_limit_start_ms` when the leftover has passed 0 or the
    /// demanded pitch is already beyond the leftover (more toward 0
    /// than the leftover still wants).
    pub fn set_vtol_roll_pitch_limit(
        &mut self,
        nav_roll_cd: &mut i32,
        nav_pitch_cd: &mut i32,
        now_ms: u32,
    ) -> bool {
        if self.vtol_limit_start_ms == 0 {
            return false;
        }
        let pitch_change_cd =
            now_ms.wrapping_sub(self.vtol_limit_start_ms) as f32 * self.rate_vtol * 0.1;
        if pitch_change_cd > self.vtol_limit_initial_pitch.abs() {
            self.vtol_limit_start_ms = 0;
            return false;
        }
        if self.vtol_limit_initial_pitch < 0.0 {
            let pitch_limit = self.vtol_limit_initial_pitch + pitch_change_cd;
            if (*nav_pitch_cd as f32) > pitch_limit {
                *nav_pitch_cd = pitch_limit as i32;
                *nav_roll_cd = 0;
                return true;
            }
        } else {
            let pitch_limit = self.vtol_limit_initial_pitch - pitch_change_cd;
            if (*nav_pitch_cd as f32) < pitch_limit {
                *nav_pitch_cd = pitch_limit as i32;
                *nav_roll_cd = 0;
                return true;
            }
        }
        self.vtol_limit_start_ms = 0;
        false
    }

    /// Upstream `set_FW_roll_pitch` leftover when `transition_state == DONE`.
    ///
    /// Returns `true` when the leftover still holds pitch. Clears
    /// `fw_limit_start_ms` when the leftover has reached 0 or the
    /// demanded pitch is already at or above the leftover (never
    /// limit to a smaller pitch angle).
    pub fn apply_fw_pitch_down_limit(
        &mut self,
        nav_roll_cd: &mut i32,
        nav_pitch_cd: &mut i32,
        now_ms: u32,
    ) -> bool {
        if self.fw_limit_start_ms == 0 {
            return false;
        }
        let pitch_limit_cd = self.fw_limit_initial_pitch
            - now_ms.wrapping_sub(self.fw_limit_start_ms) as f32 * self.rate_fw * 0.1;
        if pitch_limit_cd <= 0.0 || (*nav_pitch_cd as f32) >= pitch_limit_cd {
            self.fw_limit_start_ms = 0;
            false
        } else {
            *nav_pitch_cd = pitch_limit_cd as i32;
            *nav_roll_cd = 0;
            true
        }
    }

    /// Upstream `Tailsitter_Transition::allow_stick_mixing`.
    ///
    /// False while pitching up into VTOL (`in_vtol_transition()` with
    /// the default `now == 0`) or while levelling off in FW
    /// (`transition_state == DONE` and `fw_limit_start_ms != 0`).
    #[must_use]
    pub const fn allow_stick_mixing(
        &self,
        in_vtol_transition: bool,
        transition_done: bool,
    ) -> bool {
        if in_vtol_transition {
            return false;
        }
        if transition_done && self.fw_limit_start_ms != 0 {
            return false;
        }
        true
    }
}
