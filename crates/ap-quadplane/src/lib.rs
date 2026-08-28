//! QuadPlane / VTOL support, upstream `ArduPlane/quadplane.*` (Plane-4.7.0).
//!
//! Tracked as **VT-001**. `setup` / `available` live on [`QuadPlane`].
//! [`QuadPlane::in_vtol_mode`] / [`QuadPlane::in_vtol_auto`] are true
//! when the current flight mode is a Q* VTOL mode, or AUTO is flying a
//! VTOL nav command.
//!
//! This slice: [`QuadPlane::mode_enter`] resets poscontrol / lean-angle
//! when Plane changes mode, and [`Self::setup`] constructs the
//! attitude_control / pos_control stubs (COP controllers live in
//! `ap-control`). [`QuadPlane::init_throttle_wait`] is the QHover /
//! QLoiter enter hook.
//!
//! Upstream:
//! - `enabled()` is `return enable != 0` (`Q_ENABLE`, `AP_Int8 enable`).
//! - `setup()` returns `true` if already `initialised`; returns `false`
//!   when `enable` is zero; otherwise allocates motors and the COP
//!   controllers and ends with `initialised = true`.
//! - `available()` is `return initialised`.
//! - `mode_enter()` is called on every mode change; when available it
//!   sets `pos_control->set_lean_angle_max_cd(0)` and always returns
//!   poscontrol to `QPOS_NONE`.

#![no_std]

pub mod air_mode;
pub mod poscontrol;
pub mod tailsitter;
pub mod transition;
pub mod transition_fsm;
pub mod vtol_mode;

/// Default `Q_ENABLE`, upstream `AP_GROUPINFO_FLAGS("ENABLE", 1, QuadPlane, enable, 0, ...)`.
pub const Q_ENABLE_DEFAULT: i8 = 0;

/// The QuadPlane object, upstream `class QuadPlane`.
#[derive(Clone, Copy, Debug)]
pub struct QuadPlane {
    /// `Q_ENABLE`, upstream `AP_Int8 enable`.
    enable: i8,
    /// Upstream `bool initialised` — set true at the end of [`Self::setup`].
    initialised: bool,
    /// Lift-motor object constructed by [`Self::setup`].
    ///
    /// Upstream `AP_MotorsMulticopter *motors`. Frame-class allocation
    /// (`AP_MotorsMatrix` / `AP_MotorsTri` / `AP_MotorsTailsitter`) is a
    /// later slice; this flag is the non-null pointer after motors-init.
    motors_inited: bool,
    /// Attitude-control object constructed by [`Self::setup`].
    ///
    /// Upstream `AC_AttitudeControl_Multi *attitude_control`. The
    /// controller lives in COP; this flag is the non-null pointer.
    attitude_control_inited: bool,
    /// Position-control object constructed by [`Self::setup`].
    ///
    /// Upstream `AC_PosControl *pos_control`. The controller lives in
    /// COP; this flag is the non-null pointer.
    pos_control_inited: bool,
    /// Last `pos_control->set_lean_angle_max_cd` this stub recorded.
    lean_angle_max_cd: i32,
    /// Upstream `PosControlState poscontrol`.
    poscontrol: poscontrol::PosControlState,
    /// Upstream `bool throttle_wait`.
    throttle_wait: bool,
    /// Upstream `bool guided_wait_takeoff`.
    guided_wait_takeoff: bool,
    /// Upstream `bool guided_wait_takeoff_on_mode_enter`.
    guided_wait_takeoff_on_mode_enter: bool,
    /// `Q_OPTIONS`, upstream `AP_Int32 options`.
    options: i32,
    /// Air-mode latch, upstream `AirMode air_mode`.
    air_mode: air_mode::AirMode,
    /// Upstream `bool assisted_flight`.
    assisted_flight: bool,
}

impl QuadPlane {
    /// A disabled QuadPlane (`Q_ENABLE` 0), matching the parameter default.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            enable: Q_ENABLE_DEFAULT,
            initialised: false,
            motors_inited: false,
            attitude_control_inited: false,
            pos_control_inited: false,
            lean_angle_max_cd: 0,
            poscontrol: poscontrol::PosControlState::new(),
            throttle_wait: false,
            guided_wait_takeoff: false,
            guided_wait_takeoff_on_mode_enter: false,
            options: air_mode::Q_OPTIONS_DEFAULT,
            air_mode: air_mode::AirMode::Off,
            assisted_flight: false,
        }
    }

    /// Construct with an explicit `Q_ENABLE` value.
    ///
    /// Does not run [`Self::setup`]; [`Self::available`] stays false
    /// until setup succeeds.
    #[must_use]
    pub const fn with_enable(enable: i8) -> Self {
        Self {
            enable,
            initialised: false,
            motors_inited: false,
            attitude_control_inited: false,
            pos_control_inited: false,
            lean_angle_max_cd: 0,
            poscontrol: poscontrol::PosControlState::new(),
            throttle_wait: false,
            guided_wait_takeoff: false,
            guided_wait_takeoff_on_mode_enter: false,
            options: air_mode::Q_OPTIONS_DEFAULT,
            air_mode: air_mode::AirMode::Off,
            assisted_flight: false,
        }
    }

    /// Write `Q_ENABLE`.
    ///
    /// Does not clear [`Self::initialised`]; upstream `available()` is
    /// only that flag, not a re-check of `enable`.
    pub fn set_enable(&mut self, enable: i8) {
        self.enable = enable;
    }

    /// Current `Q_ENABLE` value.
    #[must_use]
    pub const fn enable(&self) -> i8 {
        self.enable
    }

    /// Upstream `QuadPlane::enabled` — `return enable != 0`.
    #[must_use]
    pub const fn enabled(&self) -> bool {
        self.enable != 0
    }

    /// Upstream `bool initialised`.
    #[must_use]
    pub const fn initialised(&self) -> bool {
        self.initialised
    }

    /// Whether [`Self::setup`] constructed the lift-motor object.
    ///
    /// Upstream `motors != nullptr` after a successful setup.
    #[must_use]
    pub const fn motors_inited(&self) -> bool {
        self.motors_inited
    }

    /// Upstream `QuadPlane::setup`.
    ///
    /// When already initialised, returns `true`. When `Q_ENABLE == 0`,
    /// returns `false` and leaves motors and the COP controllers
    /// unallocated. Otherwise runs the motors-init stub, constructs
    /// the attitude_control / pos_control stubs, and sets `initialised`.
    ///
    /// Soft-armed rejection, the memory check, and frame-class
    /// construction are later slices.
    pub fn setup(&mut self) -> bool {
        if self.initialised {
            return true;
        }
        if !self.enabled() {
            return false;
        }
        // Upstream `motors = NEW_NOTHROW AP_MotorsMatrix(rc_speed)` (default
        // frame class). A null allocation is `allocation_error("motors")`.
        self.motors_inited = true;
        // Upstream `attitude_control = NEW_NOTHROW AC_AttitudeControl_TS(...)`
        // then `pos_control = NEW_NOTHROW AC_PosControl(...)`. The objects
        // live in COP; these flags are the non-null pointers.
        self.attitude_control_inited = true;
        self.pos_control_inited = true;
        self.initialised = true;
        true
    }

    /// Upstream `QuadPlane::available` — `return initialised`.
    #[must_use]
    pub const fn available(&self) -> bool {
        self.initialised
    }
}

impl Default for QuadPlane {
    fn default() -> Self {
        Self::new()
    }
}
