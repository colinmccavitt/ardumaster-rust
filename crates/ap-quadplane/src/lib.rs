//! QuadPlane / VTOL support, upstream `ArduPlane/quadplane.*` (Plane-4.7.0).
//!
//! Tracked as **VT-001**. `setup` / `available` live on [`QuadPlane`].
//! [`QuadPlane::in_vtol_mode`] / [`QuadPlane::in_vtol_auto`] are true
//! when the current flight mode is a Q* VTOL mode, or AUTO is flying a
//! VTOL nav command.
//!
//! AUTO mission VTOL already landed. This slice:
//! [`logging`] stubs leftover QTUN / QPOS / AttRate
//! (`Log_Write_QControl_Tuning` / `log_QPOS` / `Log_Write_AttRate`
//! plus the `update()` 25 Hz gate). It does not rewrite setup,
//! air-mode, landing, auto_vtol, motor-test, tailsitter completeness,
//! or the leftover catalog.
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
pub mod auto_vtol;
pub mod completeness;
pub mod landing;
pub mod logging;
pub mod mode_q;
pub mod mode_qautotune;
pub mod mode_qland;
pub mod mode_qrtl;
pub mod motor_test;
pub mod poscontrol;
pub mod quadplane_completeness;
pub mod tailsitter;
pub mod throttle;
pub mod tiltrotor;
pub mod transition;
pub mod transition_fsm;
pub mod vtol_mode;
pub mod weathervane;

pub use completeness::{PortStatus, TailsitterPortItem, TAILSITTER_COMPLETENESS};

/// Default `Q_ENABLE`, upstream `AP_GROUPINFO_FLAGS("ENABLE", 1, QuadPlane, enable, 0, ...)`.
pub const Q_ENABLE_DEFAULT: i8 = 0;

/// Default `Q_FRAME_CLASS`, upstream `AP_GROUPINFO("FRAME_CLASS", 46, QuadPlane, frame_class, 1)`.
pub const Q_FRAME_CLASS_DEFAULT: u8 = 1;

/// Default `Q_FRAME_TYPE`, upstream `AP_GROUPINFO("FRAME_TYPE", 31, QuadPlane, frame_type, 1)`.
pub const Q_FRAME_TYPE_DEFAULT: u8 = 1;

/// Default `Q_TILT_ENABLE`, upstream `AP_GROUPINFO_FLAGS("ENABLE", 1, Tiltrotor, enable, 0)`.
pub const Q_TILT_ENABLE_DEFAULT: i8 = 0;

/// `Q_FRAME_CLASS` / `AP_Motors::motor_frame_class`.
///
/// Only the QuadPlane-supported values pass [`QuadPlane::frame_class_supported`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MotorFrameClass {
    /// `MOTOR_FRAME_UNDEFINED`.
    Undefined = 0,
    /// `MOTOR_FRAME_QUAD`.
    Quad = 1,
    /// `MOTOR_FRAME_HEXA`.
    Hexa = 2,
    /// `MOTOR_FRAME_OCTA`.
    Octa = 3,
    /// `MOTOR_FRAME_OCTAQUAD`.
    OctaQuad = 4,
    /// `MOTOR_FRAME_Y6`.
    Y6 = 5,
    /// `MOTOR_FRAME_HELI` — not a QuadPlane class.
    Heli = 6,
    /// `MOTOR_FRAME_TRI`.
    Tri = 7,
    /// `MOTOR_FRAME_SINGLE` — not a QuadPlane class.
    Single = 8,
    /// `MOTOR_FRAME_COAX` — not a QuadPlane class.
    Coax = 9,
    /// `MOTOR_FRAME_TAILSITTER` — duo-motor tailsitter.
    Tailsitter = 10,
    /// `MOTOR_FRAME_HELI_DUAL` — not a QuadPlane class.
    HeliDual = 11,
    /// `MOTOR_FRAME_DODECAHEXA` — not a QuadPlane class.
    DodecaHexa = 12,
    /// `MOTOR_FRAME_HELI_QUAD` — not a QuadPlane class.
    HeliQuad = 13,
    /// `MOTOR_FRAME_DECA`.
    Deca = 14,
    /// `MOTOR_FRAME_SCRIPTING_MATRIX`.
    ScriptingMatrix = 15,
    /// `MOTOR_FRAME_6DOF_SCRIPTING` — not a QuadPlane class.
    SixDofScripting = 16,
    /// `MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX`.
    DynamicScriptingMatrix = 17,
}

impl MotorFrameClass {
    /// Inverse of the upstream discriminant.
    #[must_use]
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Undefined),
            1 => Some(Self::Quad),
            2 => Some(Self::Hexa),
            3 => Some(Self::Octa),
            4 => Some(Self::OctaQuad),
            5 => Some(Self::Y6),
            6 => Some(Self::Heli),
            7 => Some(Self::Tri),
            8 => Some(Self::Single),
            9 => Some(Self::Coax),
            10 => Some(Self::Tailsitter),
            11 => Some(Self::HeliDual),
            12 => Some(Self::DodecaHexa),
            13 => Some(Self::HeliQuad),
            14 => Some(Self::Deca),
            15 => Some(Self::ScriptingMatrix),
            16 => Some(Self::SixDofScripting),
            17 => Some(Self::DynamicScriptingMatrix),
            _ => None,
        }
    }

    /// Upstream discriminant.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// Motors object `QuadPlane::setup` would `NEW_NOTHROW`.
///
/// This is the class selection only — not a rewrite of ap-motors mixing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotorsKind {
    /// `AP_MotorsMatrix` — default multicopter / tiltrotor lift motors.
    Matrix,
    /// `AP_MotorsTri`.
    Tri,
    /// `AP_MotorsTailsitter` — duo-motor tailsitter (`Q_FRAME_CLASS` 10).
    Tailsitter,
}

/// VTOL airframe family selected at setup.
///
/// Tiltrotor is `Q_TILT_ENABLE`, not a `Q_FRAME_CLASS` value. Tailsitter
/// is `Q_FRAME_CLASS == TAILSITTER` or `Q_TAILSIT_ENABLE > 0`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VtolAirframe {
    /// Conventional multicopter lift (SLT).
    Multicopter,
    /// Duo-motor / tailsitter.
    Tailsitter,
    /// Tiltrotor on a multicopter frame class.
    Tiltrotor,
}

/// Result of [`QuadPlane::classify_frame`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameSetup {
    /// Tailsitter / tiltrotor / multicopter.
    pub airframe: VtolAirframe,
    /// Motors class `setup` would allocate.
    pub motors_kind: MotorsKind,
}

/// The QuadPlane object, upstream `class QuadPlane`.
#[derive(Clone, Copy, Debug)]
pub struct QuadPlane {
    /// `Q_ENABLE`, upstream `AP_Int8 enable`.
    enable: i8,
    /// `Q_FRAME_CLASS`, upstream `AP_Enum<AP_Motors::motor_frame_class> frame_class`.
    frame_class: u8,
    /// `Q_FRAME_TYPE`, upstream `AP_Enum<AP_Motors::motor_frame_type> frame_type`.
    frame_type: u8,
    /// `Q_TAILSIT_ENABLE`, upstream `Tailsitter::enable`.
    tailsit_enable: i8,
    /// `Q_TILT_ENABLE`, upstream `Tiltrotor::enable`.
    tilt_enable: i8,
    /// Upstream `bool initialised` — set true at the end of [`Self::setup`].
    initialised: bool,
    /// Lift-motor object constructed by [`Self::setup`].
    ///
    /// Upstream `AP_MotorsMulticopter *motors`. The concrete class is
    /// [`Self::motors_kind`] (`AP_MotorsMatrix` / `AP_MotorsTri` /
    /// `AP_MotorsTailsitter`).
    motors_inited: bool,
    /// Motors class selected by the last successful [`Self::setup`].
    motors_kind: MotorsKind,
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
    /// Weathervane object constructed by [`Self::setup`].
    ///
    /// Upstream `AC_WeatherVane *weathervane`.
    weathervane_inited: bool,
    /// Upstream `AC_WeatherVane *weathervane` (embedded, not a heap pointer).
    weathervane: weathervane::WeatherVane,
    /// Upstream `motor_test` start / output / stop block.
    motor_test: motor_test::MotorTest,
    /// `motors->armed()` latch written by motor-test start / stop.
    motors_armed: bool,
    /// Upstream `landing_detect` block (`Q_LAND_ALTCHG` + timers).
    landing_detect: landing::LandingDetect,
    /// `Q_LAND_FINAL_ALT` / `land_final_alt_m`.
    land_final_alt_m: f32,
    /// `last_land_final_agl_m` height-glitch filter.
    last_land_final_agl_m: f32,
    /// Upstream `bool guided_takeoff`.
    guided_takeoff: bool,
    /// AUTO mission VTOL leftover (`do_vtol_*` / `verify_*` / `control_auto`).
    auto_vtol: auto_vtol::AutoVtol,
    /// Leftover QTUN / QPOS / AttRate logger block.
    logging: logging::QLogging,
}

impl QuadPlane {
    /// A disabled QuadPlane (`Q_ENABLE` 0), matching the parameter default.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            enable: Q_ENABLE_DEFAULT,
            frame_class: Q_FRAME_CLASS_DEFAULT,
            frame_type: Q_FRAME_TYPE_DEFAULT,
            tailsit_enable: tailsitter::TAILSIT_ENABLE_DEFAULT,
            tilt_enable: Q_TILT_ENABLE_DEFAULT,
            initialised: false,
            motors_inited: false,
            motors_kind: MotorsKind::Matrix,
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
            weathervane_inited: false,
            weathervane: weathervane::WeatherVane::new(),
            motor_test: motor_test::MotorTest::new(),
            motors_armed: false,
            landing_detect: landing::LandingDetect::new(),
            land_final_alt_m: landing::Q_LAND_FINAL_ALT_DEFAULT_M,
            last_land_final_agl_m: 0.0,
            guided_takeoff: false,
            auto_vtol: auto_vtol::AutoVtol::new(),
            logging: logging::QLogging::new(),
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
            frame_class: Q_FRAME_CLASS_DEFAULT,
            frame_type: Q_FRAME_TYPE_DEFAULT,
            tailsit_enable: tailsitter::TAILSIT_ENABLE_DEFAULT,
            tilt_enable: Q_TILT_ENABLE_DEFAULT,
            initialised: false,
            motors_inited: false,
            motors_kind: MotorsKind::Matrix,
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
            weathervane_inited: false,
            weathervane: weathervane::WeatherVane::new(),
            motor_test: motor_test::MotorTest::new(),
            motors_armed: false,
            landing_detect: landing::LandingDetect::new(),
            land_final_alt_m: landing::Q_LAND_FINAL_ALT_DEFAULT_M,
            last_land_final_agl_m: 0.0,
            guided_takeoff: false,
            auto_vtol: auto_vtol::AutoVtol::new(),
            logging: logging::QLogging::new(),
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

    /// Write `Q_FRAME_CLASS`.
    ///
    /// Has no effect after a successful [`Self::setup`]; upstream does
    /// not re-allocate motors on a parameter write.
    pub fn set_frame_class(&mut self, frame_class: u8) {
        self.frame_class = frame_class;
    }

    /// Current `Q_FRAME_CLASS` value.
    #[must_use]
    pub const fn frame_class(&self) -> u8 {
        self.frame_class
    }

    /// Write `Q_FRAME_TYPE`.
    pub fn set_frame_type(&mut self, frame_type: u8) {
        self.frame_type = frame_type;
    }

    /// Current `Q_FRAME_TYPE` value.
    #[must_use]
    pub const fn frame_type(&self) -> u8 {
        self.frame_type
    }

    /// Write `Q_TAILSIT_ENABLE`.
    pub fn set_tailsit_enable(&mut self, tailsit_enable: i8) {
        self.tailsit_enable = tailsit_enable;
    }

    /// Current `Q_TAILSIT_ENABLE` value.
    #[must_use]
    pub const fn tailsit_enable(&self) -> i8 {
        self.tailsit_enable
    }

    /// Write `Q_TILT_ENABLE`.
    pub fn set_tilt_enable(&mut self, tilt_enable: i8) {
        self.tilt_enable = tilt_enable;
    }

    /// Current `Q_TILT_ENABLE` value.
    #[must_use]
    pub const fn tilt_enable(&self) -> i8 {
        self.tilt_enable
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

    /// Motors class selected by the last successful [`Self::setup`].
    ///
    /// `None` until motors are allocated.
    #[must_use]
    pub const fn motors_kind(&self) -> Option<MotorsKind> {
        if self.motors_inited {
            Some(self.motors_kind)
        } else {
            None
        }
    }

    /// Airframe family after a successful [`Self::setup`].
    ///
    /// `None` until motors are allocated. Uses the same table as
    /// [`Self::classify_frame`].
    #[must_use]
    pub const fn vtol_airframe(&self) -> Option<VtolAirframe> {
        if !self.motors_inited {
            return None;
        }
        match Self::classify_frame(self.frame_class, self.tailsit_enable, self.tilt_enable) {
            Some(sel) => Some(sel.airframe),
            None => None,
        }
    }

    /// Whether `Q_FRAME_CLASS` is one of the QuadPlane `setup()` cases.
    ///
    /// Unsupported values (`HELI`, `SINGLE`, `COAX`, …) are a
    /// `config_error` upstream.
    #[must_use]
    pub const fn frame_class_supported(frame_class: u8) -> bool {
        matches!(frame_class, 1 | 2 | 3 | 4 | 5 | 7 | 10 | 14 | 15 | 17)
    }

    /// Motors object for a supported `Q_FRAME_CLASS`.
    #[must_use]
    pub const fn motors_kind_for(frame_class: u8) -> MotorsKind {
        match frame_class {
            7 => MotorsKind::Tri,
            10 => MotorsKind::Tailsitter,
            _ => MotorsKind::Matrix,
        }
    }

    /// Frame-class / tilt-enable selection, upstream `QuadPlane::setup`.
    ///
    /// `None` is an unsupported `Q_FRAME_CLASS`, or the tailsitter +
    /// tiltrotor config error (`set TAILSIT_ENABLE 0 or TILT_ENABLE 0`).
    /// Tiltrotor is `Q_TILT_ENABLE`, not a frame-class value.
    #[must_use]
    pub const fn classify_frame(
        frame_class: u8,
        tailsit_enable: i8,
        tilt_enable: i8,
    ) -> Option<FrameSetup> {
        if !Self::frame_class_supported(frame_class) {
            return None;
        }
        let tailsitter = frame_class == MotorFrameClass::Tailsitter as u8 || tailsit_enable > 0;
        if tailsitter && tilt_enable > 0 {
            return None;
        }
        let airframe = if tailsitter {
            VtolAirframe::Tailsitter
        } else if tilt_enable > 0 {
            VtolAirframe::Tiltrotor
        } else {
            VtolAirframe::Multicopter
        };
        Some(FrameSetup {
            airframe,
            motors_kind: Self::motors_kind_for(frame_class),
        })
    }

    /// Upstream `QuadPlane::setup`.
    ///
    /// When already initialised, returns `true`. When `Q_ENABLE == 0`,
    /// returns `false` and leaves motors and the COP controllers
    /// unallocated. Unsupported `Q_FRAME_CLASS` and a tailsitter +
    /// tiltrotor pair also return `false`. Otherwise runs the
    /// frame-class motors-init stub, constructs the attitude_control /
    /// pos_control / weathervane stubs, and sets `initialised`.
    ///
    /// Soft-armed rejection and the memory check are later slices.
    pub fn setup(&mut self) -> bool {
        if self.initialised {
            return true;
        }
        if !self.enabled() {
            return false;
        }
        let Some(sel) =
            Self::classify_frame(self.frame_class, self.tailsit_enable, self.tilt_enable)
        else {
            return false;
        };
        // Upstream `motors = NEW_NOTHROW AP_MotorsMatrix` / `AP_MotorsTri`
        // / `AP_MotorsTailsitter` from `Q_FRAME_CLASS`. A null allocation
        // is `allocation_error("motors")`.
        self.motors_kind = sel.motors_kind;
        self.motors_inited = true;
        // Upstream `attitude_control = NEW_NOTHROW AC_AttitudeControl_TS(...)`
        // then `pos_control = NEW_NOTHROW AC_PosControl(...)`. The objects
        // live in COP; these flags are the non-null pointers.
        self.attitude_control_inited = true;
        self.pos_control_inited = true;
        // Upstream `weathervane = NEW_NOTHROW AC_WeatherVane()`.
        self.weathervane_inited = true;
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
