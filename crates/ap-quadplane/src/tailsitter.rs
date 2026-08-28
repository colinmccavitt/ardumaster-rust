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
//!
//! The `Tailsitter_Transition` FSM is [`TailsitterTransition`]
//! (`ANGLE_WAIT_FW` / `ANGLE_WAIT_VTOL` / `DONE`). The leftover
//! complete predicates — disarmed, roll-error, 1.5× timeout, and
//! the vectored zero-throttle VTOL shortcut — live there; the
//! pitch-angle half stays on [`crate::transition::TransitionRamp`].
//! `update` / `VTOL_update` / `show_vtol_view` /
//! `get_mav_vtol_state` / `restart` / `force_transition_complete`
//! / [`Tailsitter::is_in_fw_flight`] are that slice.
//!
//! Copter-path surface mix is [`CopterOutputMix`] (`Q_TAILSIT_VT_R_P`
//! / `VT_P_P` / `VT_Y_P`, then elevon / V-tail with pitch-priority
//! headroom). Servo assignment leftover from `setup` is
//! [`SurfaceAssign`]. [`Tailsitter::write_log`] emits the TSIT
//! packet. `Q_TAILSIT_ENABLE == 2` leftover is
//! [`Tailsitter::enable_always_setup`].

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

/// `Q_TAILSIT_ENABLE == 2` — force Qassist / no control surfaces.
///
/// Upstream `@Values: 0:Disable, 1:Enable, 2:Enable Always`.
pub const TAILSIT_ENABLE_ALWAYS: i8 = 2;

/// Upstream `QuadPlane::Option::ONLY_ARM_IN_QMODE_OR_AUTO` (`1<<18`).
///
/// `setup` ORs this into `Q_OPTIONS` when enable is
/// [`TAILSIT_ENABLE_ALWAYS`].
pub const Q_OPTIONS_ONLY_ARM_IN_QMODE_OR_AUTO: i32 = 1 << 18;

/// Tailsitter `defaults_table` `MIXING_GAIN` (plane param default is 0.5).
pub const TAILSITTER_MIXING_GAIN_DEFAULT: f32 = 1.0;

/// Plane `MIXING_GAIN` `GSCALAR` default.
pub const PLANE_MIXING_GAIN_DEFAULT: f32 = 0.5;

/// Plane `MIXING_OFFSET` `GSCALAR` default.
pub const MIXING_OFFSET_DEFAULT: i16 = 0;

/// Default `Q_TAILSIT_VT_R_P`, upstream `AP_GROUPINFO("VT_R_P", ..., 1)`.
pub const VTOL_ROLL_SCALE_DEFAULT: f32 = 1.0;

/// Default `Q_TAILSIT_VT_P_P`, upstream `AP_GROUPINFO("VT_P_P", ..., 1)`.
pub const VTOL_PITCH_SCALE_DEFAULT: f32 = 1.0;

/// Default `Q_TAILSIT_VT_Y_P`, upstream `AP_GROUPINFO("VT_Y_P", ..., 1)`.
pub const VTOL_YAW_SCALE_DEFAULT: f32 = 1.0;

/// Tailsitter `defaults_table` `Q_TRANSITION_MS` (QuadPlane GROUPINFO is 5000).
pub const TAILSITTER_TRANSITION_MS_DEFAULT: u32 = 2000;

/// Servo function assignments leftover from `Tailsitter::setup`.
///
/// Upstream `_have_elevator` / `_have_aileron` / `_have_rudder` /
/// `_have_elevon` / `_have_v_tail`. Used by [`CopterOutputMix`] to
/// decide which saturation flags can trip `motors->limit`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SurfaceAssign {
    /// `SRV_Channel::k_elevator` assigned.
    pub elevator: bool,
    /// `SRV_Channel::k_aileron` assigned.
    pub aileron: bool,
    /// `SRV_Channel::k_rudder` assigned.
    pub rudder: bool,
    /// `k_elevon_left` or `k_elevon_right` assigned.
    pub elevon: bool,
    /// `k_vtail_left` or `k_vtail_right` assigned.
    pub v_tail: bool,
}

impl SurfaceAssign {
    /// No flying surfaces assigned.
    pub const NONE: Self = Self {
        elevator: false,
        aileron: false,
        rudder: false,
        elevon: false,
        v_tail: false,
    };

    /// Elevator, aileron, and rudder assigned (no elevon / V-tail).
    pub const CONVENTIONAL: Self = Self {
        elevator: true,
        aileron: true,
        rudder: true,
        elevon: false,
        v_tail: false,
    };

    /// Elevons and V-tail assigned (no dedicated elevator / aileron / rudder).
    pub const ELEVON_VTAIL: Self = Self {
        elevator: false,
        aileron: false,
        rudder: false,
        elevon: true,
        v_tail: true,
    };
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
    /// Servo function assignments leftover from `Tailsitter::setup`.
    pub surfaces: SurfaceAssign,
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
            surfaces: SurfaceAssign::NONE,
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
            surfaces: SurfaceAssign::NONE,
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
    surfaces: SurfaceAssign,
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
            surfaces: cfg.surfaces,
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

    /// Servo function assignments leftover from `Tailsitter::setup`.
    #[must_use]
    pub const fn surface_assign(&self) -> SurfaceAssign {
        self.surfaces
    }

    /// Upstream `Tailsitter::write_log`.
    ///
    /// Disabled tailsitters emit nothing. The three scalers are the
    /// `log_data` fields `speed_scaling` recorded on the last cycle.
    #[must_use]
    pub const fn write_log(
        &self,
        time_us: u64,
        throttle_scaler: f32,
        speed_scaler: f32,
        min_throttle: f32,
    ) -> Option<TsitLog> {
        if !self.enabled() {
            return None;
        }
        Some(TsitLog {
            time_us,
            throttle_scaler,
            speed_scaler,
            min_throttle,
        })
    }

    /// `Q_TAILSIT_ENABLE == 2` leftover from `Tailsitter::setup`.
    ///
    /// Forces Qassist, latches `AirMode::ASSISTED_FLIGHT_ONLY`, and
    /// ORs `Q_OPTIONS` with [`Q_OPTIONS_ONLY_ARM_IN_QMODE_OR_AUTO`].
    /// `None` when enable is not 2.
    #[must_use]
    pub const fn enable_always_setup(&self) -> Option<EnableAlwaysSetup> {
        if self.enable == TAILSIT_ENABLE_ALWAYS && self.setup_complete {
            Some(EnableAlwaysSetup {
                force_assist: true,
                air_mode: crate::air_mode::AirMode::AssistedFlightOnly,
                only_arm_option: Q_OPTIONS_ONLY_ARM_IN_QMODE_OR_AUTO,
            })
        } else {
            None
        }
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

/// Minimum `|roll|` that ends a tailsitter transition, in centidegrees.
///
/// Upstream `MAX(4500, plane.roll_limit_cd + 500)` floor.
pub const ROLL_ERROR_FLOOR_CD: i32 = 4500;

/// Added to `roll_limit_cd` before the [`ROLL_ERROR_FLOOR_CD`] max.
pub const ROLL_ERROR_MARGIN_CD: i32 = 500;

/// `transition_*_complete` timeout scale: `1.5 * 1000 ms`.
///
/// Upstream `((angle ± initial_pitch*0.01) / rate) * 1500`.
pub const TRANSITION_TIMEOUT_SCALE: f32 = 1500.0;

/// Vectored VTOL complete: `get_pilot_throttle() < 0.05`.
pub const VTOL_ZERO_THROTTLE: f32 = 0.05;

/// Vectored VTOL complete: `ahrs.groundspeed() < 1.0` m/s.
pub const VTOL_ZERO_GROUNDSPEED_MS: f32 = 1.0;

/// Inverted-flight roll fold, upstream `18000 - labs(roll_sensor)`.
pub const INVERTED_ROLL_CD: i32 = 18000;

/// Why [`TailsitterTransition::transition_fw_complete`] /
/// [`TailsitterTransition::transition_vtol_complete`] returned true.
///
/// The pitch-angle half is already on
/// [`crate::transition::TransitionRamp::angle_complete`]. This is the
/// leftover: disarmed, roll-error, 1.5× timeout, and the vectored
/// zero-throttle VTOL shortcut.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CompleteReason {
    /// `!arming.is_armed_and_safety_off()` — instant, no GCS text.
    Disarmed,
    /// `|pitch| > transition_angle * 100`.
    Pitch,
    /// `|roll|` (or inverted fold) past [`roll_error_limit_cd`].
    RollError,
    /// Elapsed time past the 1.5× rate-limit budget.
    Timeout,
    /// Vectored, pilot throttle under [`VTOL_ZERO_THROTTLE`],
    /// groundspeed under [`VTOL_ZERO_GROUNDSPEED_MS`].
    ZeroThrottle,
}

/// `Tailsitter_Transition::State`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TailsitterTransitionState {
    /// VTOL → FW pitch-down wait. Upstream `ANGLE_WAIT_FW = 0`.
    AngleWaitFw = 0,
    /// FW → VTOL pitch-up wait. Upstream `ANGLE_WAIT_VTOL = 1`.
    AngleWaitVtol = 1,
    /// Transition finished. Upstream `DONE = 2`.
    Done = 2,
}

/// Attitude / arming sample the complete predicates read.
#[derive(Debug, Clone, Copy)]
pub struct TransitionCompleteSample {
    /// `plane.arming.is_armed_and_safety_off()`.
    pub armed_and_safety_off: bool,
    /// `ahrs.pitch_sensor` (centidegrees).
    pub pitch_cd: i32,
    /// `ahrs.roll_sensor` (centidegrees).
    pub roll_cd: i32,
    /// `plane.roll_limit_cd`.
    pub roll_limit_cd: i32,
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
    /// `_is_vectored` — VTOL zero-throttle shortcut only.
    pub is_vectored: bool,
    /// `quadplane.get_pilot_throttle()` (0..1).
    pub pilot_throttle: f32,
    /// `ahrs.groundspeed()` (m/s).
    pub groundspeed_ms: f32,
    /// `plane.fly_inverted()`.
    pub fly_inverted: bool,
}

impl TransitionCompleteSample {
    /// Armed, level, 45° roll limit, moving, mid throttle.
    #[must_use]
    pub const fn armed_level() -> Self {
        Self {
            armed_and_safety_off: true,
            pitch_cd: 0,
            roll_cd: 0,
            roll_limit_cd: 4500,
            now_ms: 0,
            is_vectored: false,
            pilot_throttle: 0.5,
            groundspeed_ms: 10.0,
            fly_inverted: false,
        }
    }
}

/// Result of [`TailsitterTransition::update`] (FW-mode cycle).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TransitionFwUpdate {
    /// `TECS_controller.use_synthetic_airspeed()` this cycle.
    ///
    /// Latched from state *before* a same-cycle complete.
    pub use_synthetic_airspeed: bool,
    /// Forced true while still in `ANGLE_WAIT_FW`.
    pub assisted_flight: bool,
    /// Commanded pitch while waiting. `None` when not ramping.
    pub nav_pitch_cd: Option<i32>,
    /// Commanded roll while waiting. `None` when not ramping.
    pub nav_roll_cd: Option<i32>,
    /// `MAX(hover, current)` while waiting. `None` when not ramping.
    pub throttle: Option<f32>,
    /// Armed complete this cycle — start the FW pitch-down leftover.
    pub start_fw_limit: bool,
    /// Why we completed, if we did.
    pub completed: Option<CompleteReason>,
}

/// Result of [`TailsitterTransition::vtol_update`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransitionVtolUpdate {
    /// Still in `ANGLE_WAIT_VTOL` (complete returned false).
    pub still_waiting: bool,
    /// Assistance while the nose-up wait is running.
    pub assisted_flight: bool,
    /// Armed complete this cycle — start the VTOL pitch-forward leftover.
    pub start_vtol_limit: bool,
    /// Why we completed the VTOL wait, if we did.
    pub completed: Option<CompleteReason>,
}

/// `Tailsitter_Transition` state machine, upstream `tailsitter.cpp`.
///
/// Holds `ANGLE_WAIT_FW` / `ANGLE_WAIT_VTOL` / `DONE`, the transition
/// timestamps, and the leftover complete predicates (roll-error,
/// 1.5× timeout, disarmed, vectored zero-throttle). The pitch / throttle
/// *ramp* stays on [`crate::transition::TransitionRamp`]; this object
/// owns the FSM that *calls* those ramps.
///
/// After QuadPlane `setup`, upstream `force_transition_complete()` so
/// [`Self::new`] starts in [`TailsitterTransitionState::Done`].
#[derive(Debug, Clone, Copy)]
pub struct TailsitterTransition {
    state: TailsitterTransitionState,
    ramp: crate::transition::TransitionRamp,
    vtol_transition_start_ms: u32,
    vtol_transition_initial_pitch: f32,
    fw_transition_start_ms: u32,
    fw_transition_initial_pitch: f32,
    last_vtol_mode_ms: u32,
    vtol_limit_start_ms: u32,
    fw_limit_start_ms: u32,
}

impl Default for TailsitterTransition {
    fn default() -> Self {
        Self::new()
    }
}

impl TailsitterTransition {
    /// Post-`setup` object: `force_transition_complete` has run.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            state: TailsitterTransitionState::Done,
            ramp: crate::transition::TransitionRamp::new(),
            vtol_transition_start_ms: 0,
            vtol_transition_initial_pitch: 0.0,
            fw_transition_start_ms: 0,
            fw_transition_initial_pitch: 0.0,
            last_vtol_mode_ms: 0,
            vtol_limit_start_ms: 0,
            fw_limit_start_ms: 0,
        }
    }

    /// Current `transition_state`.
    #[must_use]
    pub const fn state(&self) -> TailsitterTransitionState {
        self.state
    }

    /// Upstream `complete()` — `transition_state == DONE`.
    #[must_use]
    pub const fn complete(&self) -> bool {
        matches!(self.state, TailsitterTransitionState::Done)
    }

    /// Upstream `get_log_transition_state`.
    #[must_use]
    pub const fn get_log_transition_state(&self) -> u8 {
        self.state as u8
    }

    /// Upstream `active_frwd` — `transition_state == ANGLE_WAIT_FW`.
    #[must_use]
    pub const fn active_frwd(&self) -> bool {
        matches!(self.state, TailsitterTransitionState::AngleWaitFw)
    }

    /// Pitch / throttle ramp parameters this FSM uses.
    #[must_use]
    pub const fn ramp(&self) -> crate::transition::TransitionRamp {
        self.ramp
    }

    /// Mutable ramp (tests poke `Q_TAILSIT_ANGLE` / `RAT_*`).
    #[must_use]
    pub fn ramp_mut(&mut self) -> &mut crate::transition::TransitionRamp {
        &mut self.ramp
    }

    /// `vtol_transition_start_ms`.
    #[must_use]
    pub const fn vtol_transition_start_ms(&self) -> u32 {
        self.vtol_transition_start_ms
    }

    /// `fw_transition_start_ms`.
    #[must_use]
    pub const fn fw_transition_start_ms(&self) -> u32 {
        self.fw_transition_start_ms
    }

    /// `last_vtol_mode_ms`.
    #[must_use]
    pub const fn last_vtol_mode_ms(&self) -> u32 {
        self.last_vtol_mode_ms
    }

    /// `vtol_limit_start_ms` stamped by an armed [`Self::vtol_update`] complete.
    #[must_use]
    pub const fn vtol_limit_start_ms(&self) -> u32 {
        self.vtol_limit_start_ms
    }

    /// `fw_limit_start_ms` stamped by an armed [`Self::update`] complete.
    #[must_use]
    pub const fn fw_limit_start_ms(&self) -> u32 {
        self.fw_limit_start_ms
    }

    /// Upstream `Tailsitter::is_in_fw_flight`.
    ///
    /// `enabled && !in_vtol_mode && transition_state == DONE`.
    #[must_use]
    pub const fn is_in_fw_flight(&self, enabled: bool, in_vtol_mode: bool) -> bool {
        enabled && !in_vtol_mode && self.complete()
    }

    /// Upstream `Tailsitter_Transition::show_vtol_view`.
    ///
    /// VTOL mode is hidden while still pitching up (`ANGLE_WAIT_VTOL`).
    /// FW mode still shows the VTOL view while pitching down
    /// (`ANGLE_WAIT_FW`).
    #[must_use]
    pub const fn show_vtol_view(&self, in_vtol_mode: bool) -> bool {
        if in_vtol_mode && matches!(self.state, TailsitterTransitionState::AngleWaitVtol) {
            return false;
        }
        if !in_vtol_mode && matches!(self.state, TailsitterTransitionState::AngleWaitFw) {
            return true;
        }
        in_vtol_mode
    }

    /// Upstream `Tailsitter_Transition::get_mav_vtol_state`.
    #[must_use]
    pub const fn get_mav_vtol_state(&self, in_vtol_mode: bool) -> crate::air_mode::MavVtolState {
        match self.state {
            TailsitterTransitionState::AngleWaitVtol => {
                crate::air_mode::MavVtolState::TransitionToMc
            }
            TailsitterTransitionState::Done => crate::air_mode::MavVtolState::Fw,
            TailsitterTransitionState::AngleWaitFw => {
                if in_vtol_mode {
                    crate::air_mode::MavVtolState::Mc
                } else {
                    crate::air_mode::MavVtolState::TransitionToFw
                }
            }
        }
    }

    /// Upstream `Tailsitter_Transition::allow_weathervane`.
    ///
    /// `in_vtol_transition` is the `now == 0` form (ANGLE_WAIT_VTOL
    /// only). Weathervane waits until the VTOL leftover has also
    /// cleared (`vtol_limit_start_ms == 0`).
    #[must_use]
    pub const fn allow_weathervane(&self, in_vtol_transition: bool) -> bool {
        !in_vtol_transition && self.vtol_limit_start_ms == 0
    }

    /// Upstream `Tailsitter_Transition::restart`.
    ///
    /// `attitude_target_pitch_cd` is
    /// `attitude_control->get_attitude_target_quat().get_euler_pitch()
    /// * degrees(100)` — already in centidegrees, then clamped to
    /// ±[`crate::transition::PITCH_CD_LIMIT`].
    pub fn restart(&mut self, now_ms: u32, attitude_target_pitch_cd: f32) {
        self.state = TailsitterTransitionState::AngleWaitFw;
        self.fw_transition_start_ms = now_ms;
        let limit = crate::transition::PITCH_CD_LIMIT as f32;
        self.fw_transition_initial_pitch = constrain_f32(attitude_target_pitch_cd, -limit, limit);
    }

    /// Upstream `Tailsitter_Transition::force_transition_complete`.
    ///
    /// `nav_pitch_cd` is clamped to ±[`crate::transition::PITCH_CD_LIMIT`]
    /// and stored as the next VTOL-transition start. Clears
    /// `fw_limit_start_ms`.
    pub fn force_transition_complete(&mut self, now_ms: u32, nav_pitch_cd: i32) {
        self.state = TailsitterTransitionState::Done;
        self.vtol_transition_start_ms = now_ms;
        let limit = crate::transition::PITCH_CD_LIMIT as f32;
        self.vtol_transition_initial_pitch = constrain_f32(nav_pitch_cd as f32, -limit, limit);
        self.fw_limit_start_ms = 0;
    }

    /// Upstream `Tailsitter::transition_fw_complete`.
    ///
    /// Order: disarmed, pitch, roll-error, 1.5× timeout. `None` is
    /// still waiting.
    #[must_use]
    pub fn transition_fw_complete(
        &self,
        sample: &TransitionCompleteSample,
    ) -> Option<CompleteReason> {
        if !sample.armed_and_safety_off {
            return Some(CompleteReason::Disarmed);
        }
        if self
            .ramp
            .angle_complete(crate::transition::TransitionKind::ToFw, sample.pitch_cd)
        {
            return Some(CompleteReason::Pitch);
        }
        if roll_past_error(sample.roll_cd, sample.roll_limit_cd, false) {
            return Some(CompleteReason::RollError);
        }
        if elapsed_past_timeout(
            sample.now_ms,
            self.fw_transition_start_ms,
            fw_timeout_ms(
                self.ramp.angle_fw(),
                self.fw_transition_initial_pitch,
                self.ramp.rate_fw(),
            ),
        ) {
            return Some(CompleteReason::Timeout);
        }
        None
    }

    /// Upstream `Tailsitter::transition_vtol_complete`.
    ///
    /// Order: disarmed, vectored zero-throttle, pitch (`ANG_VT`
    /// fallback), inverted roll-error, 1.5× timeout. `None` is still
    /// waiting.
    #[must_use]
    pub fn transition_vtol_complete(
        &self,
        sample: &TransitionCompleteSample,
    ) -> Option<CompleteReason> {
        if !sample.armed_and_safety_off {
            return Some(CompleteReason::Disarmed);
        }
        if sample.is_vectored
            && sample.pilot_throttle < VTOL_ZERO_THROTTLE
            && sample.groundspeed_ms < VTOL_ZERO_GROUNDSPEED_MS
        {
            return Some(CompleteReason::ZeroThrottle);
        }
        if self
            .ramp
            .angle_complete(crate::transition::TransitionKind::ToVtol, sample.pitch_cd)
        {
            return Some(CompleteReason::Pitch);
        }
        if roll_past_error(sample.roll_cd, sample.roll_limit_cd, sample.fly_inverted) {
            return Some(CompleteReason::RollError);
        }
        if elapsed_past_timeout(
            sample.now_ms,
            self.vtol_transition_start_ms,
            vtol_timeout_ms(
                self.ramp.get_transition_angle_vtol(),
                self.vtol_transition_initial_pitch,
                self.ramp.rate_vtol(),
            ),
        ) {
            return Some(CompleteReason::Timeout);
        }
        None
    }

    /// Upstream `Tailsitter_Transition::update` (fixed-wing mode).
    ///
    /// `ANGLE_WAIT_FW` ramps pitch down via
    /// [`crate::transition::TransitionRamp::pitch_cd`] and holds
    /// throttle at `MAX(hover, current)`. Completing while armed
    /// stamps `fw_limit_start_ms`.
    pub fn update(
        &mut self,
        sample: &TransitionCompleteSample,
        inverted: bool,
        hover: f32,
        current_throttle: f32,
    ) -> TransitionFwUpdate {
        let use_synthetic_airspeed = !matches!(self.state, TailsitterTransitionState::Done);
        if !matches!(self.state, TailsitterTransitionState::AngleWaitFw) {
            return TransitionFwUpdate {
                use_synthetic_airspeed,
                assisted_flight: false,
                nav_pitch_cd: None,
                nav_roll_cd: None,
                throttle: None,
                start_fw_limit: false,
                completed: None,
            };
        }
        if let Some(reason) = self.transition_fw_complete(sample) {
            self.state = TailsitterTransitionState::Done;
            let start_fw_limit = sample.armed_and_safety_off;
            if start_fw_limit {
                self.fw_limit_start_ms = sample.now_ms;
            }
            return TransitionFwUpdate {
                use_synthetic_airspeed,
                assisted_flight: false,
                nav_pitch_cd: None,
                nav_roll_cd: None,
                throttle: None,
                start_fw_limit,
                completed: Some(reason),
            };
        }
        let dt = sample.now_ms.wrapping_sub(self.fw_transition_start_ms);
        let nav_pitch_cd = self.ramp.pitch_cd(
            crate::transition::TransitionKind::ToFw,
            self.fw_transition_initial_pitch,
            dt,
            inverted,
        );
        let throttle = self.ramp.throttle(
            crate::transition::TransitionKind::ToFw,
            hover,
            0.0,
            current_throttle,
        );
        TransitionFwUpdate {
            use_synthetic_airspeed,
            assisted_flight: true,
            nav_pitch_cd: Some(nav_pitch_cd),
            nav_roll_cd: Some(0),
            throttle: Some(throttle),
            start_fw_limit: false,
            completed: None,
        }
    }

    /// Upstream `Tailsitter_Transition::VTOL_update`.
    ///
    /// More than [`LAST_VTOL_MODE_MS`] since the last VTOL cycle
    /// enters `ANGLE_WAIT_VTOL`. Completing while armed stamps
    /// `vtol_limit_start_ms`. Either way (except an incomplete wait)
    /// [`Self::restart`] sets up the next FW transition.
    ///
    /// `attitude_target_pitch_cd` is forwarded to [`Self::restart`].
    pub fn vtol_update(
        &mut self,
        sample: &TransitionCompleteSample,
        attitude_target_pitch_cd: f32,
    ) -> TransitionVtolUpdate {
        let now = sample.now_ms;
        if now.wrapping_sub(self.last_vtol_mode_ms) > LAST_VTOL_MODE_MS {
            self.state = TailsitterTransitionState::AngleWaitVtol;
        }
        self.last_vtol_mode_ms = now;

        if matches!(self.state, TailsitterTransitionState::AngleWaitVtol) {
            if let Some(reason) = self.transition_vtol_complete(sample) {
                let start_vtol_limit = sample.armed_and_safety_off;
                if start_vtol_limit {
                    self.vtol_limit_start_ms = now;
                }
                self.restart(now, attitude_target_pitch_cd);
                return TransitionVtolUpdate {
                    still_waiting: false,
                    assisted_flight: true,
                    start_vtol_limit,
                    completed: Some(reason),
                };
            }
            return TransitionVtolUpdate {
                still_waiting: true,
                assisted_flight: true,
                start_vtol_limit: false,
                completed: None,
            };
        }
        self.restart(now, attitude_target_pitch_cd);
        TransitionVtolUpdate {
            still_waiting: false,
            assisted_flight: false,
            start_vtol_limit: false,
            completed: None,
        }
    }

    /// Upstream `set_FW_roll_pitch` nose-up half (`in_vtol_transition`).
    ///
    /// The `DONE` / `fw_limit_*` leftover stays on [`PitchLimit`]. When
    /// not in the VTOL transition and state is `DONE`, this only
    /// restamps `vtol_transition_start_ms` / initial pitch so the next
    /// nose-up starts from the current demand.
    pub fn set_fw_roll_pitch(
        &mut self,
        nav_pitch_cd: &mut i32,
        nav_roll_cd: &mut i32,
        now_ms: u32,
        in_vtol_transition: bool,
    ) {
        if in_vtol_transition {
            let dt = now_ms.wrapping_sub(self.vtol_transition_start_ms);
            *nav_pitch_cd = self.ramp.pitch_cd(
                crate::transition::TransitionKind::ToVtol,
                self.vtol_transition_initial_pitch,
                dt,
                false,
            );
            *nav_roll_cd = 0;
        } else if matches!(self.state, TailsitterTransitionState::Done) {
            self.vtol_transition_start_ms = now_ms;
            let limit = crate::transition::PITCH_CD_LIMIT as f32;
            self.vtol_transition_initial_pitch = constrain_f32(*nav_pitch_cd as f32, -limit, limit);
        }
    }
}

impl Tailsitter {
    /// Upstream `Tailsitter::is_in_fw_flight`.
    ///
    /// `enabled && !in_vtol_mode && transition_state == DONE`.
    #[must_use]
    pub const fn is_in_fw_flight(&self, in_vtol_mode: bool, transition_done: bool) -> bool {
        self.enabled() && !in_vtol_mode && transition_done
    }
}

/// Upstream `MAX(4500, roll_limit_cd + 500)`.
#[must_use]
pub const fn roll_error_limit_cd(roll_limit_cd: i32) -> i32 {
    let extra = roll_limit_cd.saturating_add(ROLL_ERROR_MARGIN_CD);
    if extra > ROLL_ERROR_FLOOR_CD {
        extra
    } else {
        ROLL_ERROR_FLOOR_CD
    }
}

fn roll_abs_cd(roll_cd: i32, fly_inverted: bool) -> i32 {
    let abs_roll = roll_cd.unsigned_abs() as i32;
    if fly_inverted {
        INVERTED_ROLL_CD - abs_roll
    } else {
        abs_roll
    }
}

fn roll_past_error(roll_cd: i32, roll_limit_cd: i32, fly_inverted: bool) -> bool {
    roll_abs_cd(roll_cd, fly_inverted) > roll_error_limit_cd(roll_limit_cd)
}

fn fw_timeout_ms(angle_fw: i8, initial_pitch_cd: f32, rate_fw: f32) -> f32 {
    (f32::from(angle_fw) + initial_pitch_cd * 0.01) / rate_fw * TRANSITION_TIMEOUT_SCALE
}

fn vtol_timeout_ms(angle_vtol: i8, initial_pitch_cd: f32, rate_vtol: f32) -> f32 {
    (f32::from(angle_vtol) - initial_pitch_cd * 0.01) / rate_vtol * TRANSITION_TIMEOUT_SCALE
}

fn elapsed_past_timeout(now_ms: u32, start_ms: u32, timeout_ms: f32) -> bool {
    now_ms.wrapping_sub(start_ms) as f32 > timeout_ms
}

/// Unconfigured `Q_TAILSIT_RAT_FW` leftover from `Tailsitter::setup`.
///
/// When `transition_rate_fw` was never written, setup saves
/// `transition_angle_fw / (quadplane.transition_time_ms / 2000)`.
/// Tailsitter `defaults_table` sets `Q_TRANSITION_MS` to
/// [`TAILSITTER_TRANSITION_MS_DEFAULT`] (2000), so the default angle
/// of 45 deg becomes 45 deg/s. The GROUPINFO default for `RAT_FW` is
/// still [`crate::transition::TRANSITION_RATE_FW_DEFAULT`] (50) when
/// the parameter *is* configured.
#[must_use]
pub const fn unconfigured_transition_rate_fw(angle_fw: i8, transition_time_ms: u32) -> f32 {
    (angle_fw as f32) / (transition_time_ms as f32 / 2000.0)
}

/// TSIT log payload, upstream `log_tailsitter` without `LOG_PACKET_HEADER`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TsitLog {
    /// `AP_HAL` microsecond timestamp.
    pub time_us: u64,
    /// Last `speed_scaling` hover/throttle scaler.
    pub throttle_scaler: f32,
    /// Last `speed_scaling` `spd_scaler` applied to flying surfaces.
    pub speed_scaler: f32,
    /// Last `disk_loading_min_throttle` pushed to `AP_MotorsTailsitter`.
    pub min_throttle: f32,
}

/// `Q_TAILSIT_ENABLE == 2` side-effects from `Tailsitter::setup`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EnableAlwaysSetup {
    /// `quadplane.assist.set_state(VTOL_Assist::STATE::FORCE_ENABLED)`.
    pub force_assist: bool,
    /// `quadplane.air_mode = AirMode::ASSISTED_FLIGHT_ONLY`.
    pub air_mode: crate::air_mode::AirMode,
    /// Bit ORed into `Q_OPTIONS`.
    pub only_arm_option: i32,
}

/// Motors attitude that `Tailsitter::output` pulls onto flying surfaces.
///
/// Values are the AP_Motors `-1..1` outputs plus feedforward.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorAttitude {
    /// `motors->get_yaw()`.
    pub yaw: f32,
    /// `motors->get_yaw_ff()`.
    pub yaw_ff: f32,
    /// `motors->get_pitch()`.
    pub pitch: f32,
    /// `motors->get_pitch_ff()`.
    pub pitch_ff: f32,
    /// `motors->get_roll()`.
    pub roll: f32,
    /// `motors->get_roll_ff()`.
    pub roll_ff: f32,
}

impl MotorAttitude {
    /// Zero demand, no feedforward.
    #[must_use]
    pub const fn zero() -> Self {
        Self {
            yaw: 0.0,
            yaw_ff: 0.0,
            pitch: 0.0,
            pitch_ff: 0.0,
            roll: 0.0,
            roll_ff: 0.0,
        }
    }
}

/// Scaled aileron / elevator / rudder before elevon / V-tail mix.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CopterSurfaces {
    /// `k_aileron` — VTOL yaw, negated.
    pub aileron: f32,
    /// `k_elevator` — VTOL pitch.
    pub elevator: f32,
    /// `k_rudder` — VTOL roll.
    pub rudder: f32,
}

/// Elevon / V-tail outputs after pitch-priority headroom.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElevonVtail {
    /// `k_elevon_left` = elevator_mix - aileron_mix.
    pub elevon_left: f32,
    /// `k_elevon_right` = elevator_mix + aileron_mix.
    pub elevon_right: f32,
    /// `k_vtail_left` = elevator_mix + rudder_mix.
    pub vtail_left: f32,
    /// `k_vtail_right` = elevator_mix - rudder_mix.
    pub vtail_right: f32,
}

/// `motors->limit` flags written at the end of `Tailsitter::output`.
///
/// Upstream only sets these `true`; it never clears a flag that
/// `motors_output` already raised.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MotorLimits {
    /// `motors->limit.roll`.
    pub roll: bool,
    /// `motors->limit.pitch`.
    pub pitch: bool,
    /// `motors->limit.yaw`.
    pub yaw: bool,
}

/// Combined copter-path mix result.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CopterOutput {
    /// Aileron / elevator / rudder after `VTOL_*_scale`.
    pub surfaces: CopterSurfaces,
    /// Elevon / V-tail after pitch-priority headroom.
    pub elevon_vtail: ElevonVtail,
    /// Flags that would be written to `motors->limit`.
    pub limits: MotorLimits,
}

/// Copter-path output mix, leftover `Tailsitter::output` after `motors_output`.
///
/// Tracked as **VT-007**. After the copter rate controller has run,
/// tailsitter copies motors pitch / roll / yaw onto plane surfaces
/// with an axis swap (`aileron = -(yaw+yaw_ff)*SERVO_MAX*VT_Y_P`,
/// `elevator = (pitch+pitch_ff)*SERVO_MAX*VT_P_P`,
/// `rudder = (roll+roll_ff)*SERVO_MAX*VT_R_P`), then mixes elevon
/// and V-tail giving pitch full priority: any headroom left under
/// [`SERVO_MAX`] is shared with aileron / rudder, otherwise those
/// axes are zeroed. Saturation of a dedicated surface, a tilt motor
/// on a vectored airframe, or a clipped elevon / V-tail sets
/// `motors->limit`.
///
/// This is not a rewrite of ap-motors mixing or of [`VectoredYawMix`].
#[derive(Debug, Clone, Copy)]
pub struct CopterOutputMix {
    roll_scale: f32,
    pitch_scale: f32,
    yaw_scale: f32,
    mixing_gain: f32,
    mixing_offset: i16,
    surfaces: SurfaceAssign,
}

impl Default for CopterOutputMix {
    fn default() -> Self {
        Self::new()
    }
}

impl CopterOutputMix {
    /// GROUPINFO / tailsitter-table defaults.
    ///
    /// `VT_*_P` are 1. `MIXING_GAIN` is the tailsitter table value
    /// ([`TAILSITTER_MIXING_GAIN_DEFAULT`]), not the plane 0.5.
    /// Surfaces start unassigned ([`SurfaceAssign::NONE`]).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            roll_scale: VTOL_ROLL_SCALE_DEFAULT,
            pitch_scale: VTOL_PITCH_SCALE_DEFAULT,
            yaw_scale: VTOL_YAW_SCALE_DEFAULT,
            mixing_gain: TAILSITTER_MIXING_GAIN_DEFAULT,
            mixing_offset: MIXING_OFFSET_DEFAULT,
            surfaces: SurfaceAssign::NONE,
        }
    }

    /// Conventional elevator / aileron / rudder assignment.
    #[must_use]
    pub const fn conventional() -> Self {
        Self {
            roll_scale: VTOL_ROLL_SCALE_DEFAULT,
            pitch_scale: VTOL_PITCH_SCALE_DEFAULT,
            yaw_scale: VTOL_YAW_SCALE_DEFAULT,
            mixing_gain: TAILSITTER_MIXING_GAIN_DEFAULT,
            mixing_offset: MIXING_OFFSET_DEFAULT,
            surfaces: SurfaceAssign::CONVENTIONAL,
        }
    }

    /// Elevon + V-tail assignment, no dedicated surfaces.
    #[must_use]
    pub const fn elevon_vtail() -> Self {
        Self {
            roll_scale: VTOL_ROLL_SCALE_DEFAULT,
            pitch_scale: VTOL_PITCH_SCALE_DEFAULT,
            yaw_scale: VTOL_YAW_SCALE_DEFAULT,
            mixing_gain: TAILSITTER_MIXING_GAIN_DEFAULT,
            mixing_offset: MIXING_OFFSET_DEFAULT,
            surfaces: SurfaceAssign::ELEVON_VTAIL,
        }
    }

    /// `Q_TAILSIT_VT_R_P`.
    #[must_use]
    pub const fn roll_scale(&self) -> f32 {
        self.roll_scale
    }

    /// `Q_TAILSIT_VT_P_P`.
    #[must_use]
    pub const fn pitch_scale(&self) -> f32 {
        self.pitch_scale
    }

    /// `Q_TAILSIT_VT_Y_P`.
    #[must_use]
    pub const fn yaw_scale(&self) -> f32 {
        self.yaw_scale
    }

    /// `MIXING_GAIN` used by the elevon / V-tail mix.
    #[must_use]
    pub const fn mixing_gain(&self) -> f32 {
        self.mixing_gain
    }

    /// `MIXING_OFFSET` used by the elevon / V-tail mix.
    #[must_use]
    pub const fn mixing_offset(&self) -> i16 {
        self.mixing_offset
    }

    /// Servo assignment leftover.
    #[must_use]
    pub const fn surface_assign(&self) -> SurfaceAssign {
        self.surfaces
    }

    /// Poke `Q_TAILSIT_VT_R_P`.
    pub fn set_roll_scale(&mut self, scale: f32) {
        self.roll_scale = scale;
    }

    /// Poke `Q_TAILSIT_VT_P_P`.
    pub fn set_pitch_scale(&mut self, scale: f32) {
        self.pitch_scale = scale;
    }

    /// Poke `Q_TAILSIT_VT_Y_P`.
    pub fn set_yaw_scale(&mut self, scale: f32) {
        self.yaw_scale = scale;
    }

    /// Poke `MIXING_GAIN`.
    pub fn set_mixing_gain(&mut self, gain: f32) {
        self.mixing_gain = gain;
    }

    /// Poke `MIXING_OFFSET`.
    pub fn set_mixing_offset(&mut self, offset: i16) {
        self.mixing_offset = offset;
    }

    /// Poke the setup leftover surface flags.
    pub fn set_surface_assign(&mut self, surfaces: SurfaceAssign) {
        self.surfaces = surfaces;
    }

    /// Pull copter outputs onto aileron / elevator / rudder.
    ///
    /// Upstream:
    /// `aileron  = (yaw+yaw_ff) * -SERVO_MAX * VTOL_yaw_scale`
    /// `elevator = (pitch+pitch_ff) * SERVO_MAX * VTOL_pitch_scale`
    /// `rudder   = (roll+roll_ff) * SERVO_MAX * VTOL_roll_scale`
    #[must_use]
    pub fn surfaces(&self, att: MotorAttitude) -> CopterSurfaces {
        CopterSurfaces {
            aileron: (att.yaw + att.yaw_ff) * -SERVO_MAX * self.yaw_scale,
            elevator: (att.pitch + att.pitch_ff) * SERVO_MAX * self.pitch_scale,
            rudder: (att.roll + att.roll_ff) * SERVO_MAX * self.roll_scale,
        }
    }

    /// Saturation of dedicated surfaces and vectored tilt motors.
    ///
    /// `tilt_lim` is only considered when the airframe is vectored.
    /// A true tilt flag raises both pitch and yaw limits.
    #[must_use]
    pub fn surface_limits(
        &self,
        s: CopterSurfaces,
        tilt_left: f32,
        tilt_right: f32,
        is_vectored: bool,
    ) -> MotorLimits {
        let tilt_lim =
            is_vectored && (tilt_left.abs() >= SERVO_MAX || tilt_right.abs() >= SERVO_MAX);
        let roll_lim = self.surfaces.rudder && s.rudder.abs() >= SERVO_MAX;
        let pitch_lim = self.surfaces.elevator && s.elevator.abs() >= SERVO_MAX;
        let yaw_lim = self.surfaces.aileron && s.aileron.abs() >= SERVO_MAX;
        MotorLimits {
            roll: roll_lim,
            pitch: pitch_lim || tilt_lim,
            yaw: yaw_lim || tilt_lim,
        }
    }

    /// Elevon / V-tail mix with pitch-priority headroom.
    ///
    /// Updates `limits` the way the C++ function ORs
    /// `_have_elevon` / `_have_v_tail` into yaw / pitch / roll.
    pub fn mix_elevon_vtail(&self, s: CopterSurfaces, limits: &mut MotorLimits) -> ElevonVtail {
        let offset = f32::from(self.mixing_offset);
        let elevator_mix = s.elevator * (100.0 - offset) * 0.01 * self.mixing_gain;
        let mut aileron_mix = s.aileron * (100.0 + offset) * 0.01 * self.mixing_gain;
        let mut rudder_mix = s.rudder * (100.0 + offset) * 0.01 * self.mixing_gain;
        let headroom = SERVO_MAX - elevator_mix.abs();
        if is_positive(headroom) {
            if aileron_mix.abs() > headroom {
                aileron_mix *= headroom / aileron_mix.abs();
                limits.yaw |= self.surfaces.elevon;
            }
            if rudder_mix.abs() > headroom {
                rudder_mix *= headroom / rudder_mix.abs();
                limits.roll |= self.surfaces.v_tail;
            }
        } else {
            aileron_mix = 0.0;
            rudder_mix = 0.0;
            limits.yaw |= self.surfaces.elevon;
            limits.pitch |= self.surfaces.elevon || self.surfaces.v_tail;
            limits.roll |= self.surfaces.v_tail;
        }
        ElevonVtail {
            elevon_left: elevator_mix - aileron_mix,
            elevon_right: elevator_mix + aileron_mix,
            vtail_left: elevator_mix + rudder_mix,
            vtail_right: elevator_mix - rudder_mix,
        }
    }

    /// Full leftover copter path: surfaces, elevon / V-tail, limits.
    #[must_use]
    pub fn mix(
        &self,
        att: MotorAttitude,
        tilt_left: f32,
        tilt_right: f32,
        is_vectored: bool,
    ) -> CopterOutput {
        let surfaces = self.surfaces(att);
        let mut limits = self.surface_limits(surfaces, tilt_left, tilt_right, is_vectored);
        let elevon_vtail = self.mix_elevon_vtail(surfaces, &mut limits);
        CopterOutput {
            surfaces,
            elevon_vtail,
            limits,
        }
    }
}
