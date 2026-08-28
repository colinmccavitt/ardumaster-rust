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
//! Stick remapping (`Q_TAILSIT_INPUT` PlaneMode / BodyFrameRoll) and the
//! vectored mix itself are later slices.

/// `Q_FRAME_CLASS` value that selects a duo-motor tailsitter, upstream
/// `AP_Motors::MOTOR_FRAME_TAILSITTER`.
pub const MOTOR_FRAME_TAILSITTER: u8 = 10;

/// Default `Q_TAILSIT_ENABLE`, upstream `AP_GROUPINFO_FLAGS("ENABLE", ...)`.
pub const TAILSIT_ENABLE_DEFAULT: i8 = 0;

/// Default `Q_TAILSIT_VHGAIN`, upstream `AP_GROUPINFO("VHGAIN", ..., 0.5)`.
pub const VECTORED_HOVER_GAIN_DEFAULT: f32 = 0.5;

/// Upstream `FLT_EPSILON` as used by `is_zero` in `AP_Math`.
const FLT_EPSILON: f32 = 1.192_092_90e-7;

fn is_zero(v: f32) -> bool {
    v.abs() < FLT_EPSILON
}

/// How a duo-motor tailsitter takes yaw / pitch input.
///
/// These are the airframe paths, not the `Q_TAILSIT_INPUT` stick bitmask.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputType {
    /// Tilt motors vector thrust for yaw. Upstream `_is_vectored`.
    VectoredYaw,
    /// Flying surfaces only. Upstream `is_control_surface_tailsitter`.
    ControlSurfaces,
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
}
