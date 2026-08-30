//! Simple / SuperSimple leftover, upstream `ArduCopter/Copter.cpp`.
//!
//! Tracked as **COP-012**. [`init_simple_bearing`], [`update_simple_mode`],
//! and [`update_super_simple_bearing`] are the Copter.cpp leftovers after
//! the logging / 3 Hz / 1 Hz slice. They are not scheduled callbacks —
//! modes call [`update_simple_mode`] from `run()`, arming and
//! `SIMPLE_HEADING_RESET` call [`init_simple_bearing`], and
//! `run_nav_updates` / `set_simple_mode` call
//! [`update_super_simple_bearing`]. `set_simple_mode` itself stays on
//! `AP_State.cpp`.

use ap_math::scalar::{radians, wrap_2pi, wrap_pi};

use crate::vehicle_loop::{should_log, MASK_LOG_ANY};

pub use crate::aux_fn::SimpleMode;

/// `SUPER_SIMPLE_RADIUS_M` from Copter `config.h`.
///
/// `Copter.h` still comments a 20 m radius; the compiled default is 10 m.
pub const SUPER_SIMPLE_RADIUS_M: f32 = 10.0;

/// SuperSimple only rewrites the heading after this many degrees of change.
///
/// Upstream `radians(5.0)` in `update_super_simple_bearing`.
pub const SUPER_SIMPLE_BEARING_THRESH_DEG: f32 = 5.0;

/// Inputs to `Copter::init_simple_bearing`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InitSimpleBearingInputs {
    /// `ahrs.cos_yaw()`.
    pub ahrs_cos_yaw: f32,
    /// `ahrs.sin_yaw()`.
    pub ahrs_sin_yaw: f32,
    /// `ahrs.get_yaw_rad()`.
    pub ahrs_yaw_rad: f32,
    /// `ahrs.yaw_sensor` — the value `Log_Write_Data` would store.
    pub ahrs_yaw_sensor: i32,
    /// `LOG_BITMASK`.
    pub log_bitmask: u32,
}

/// What `Copter::init_simple_bearing` wrote onto the vehicle.
///
/// SuperSimple's last-bearing is yaw + 180°, but its cos/sin copy the
/// *simple* heading, not `cos/sin` of that opposite bearing. A port that
/// seeded SuperSimple from the opposite heading would rotate sticks 180°
/// the first time SuperSimple ran, before `update_super_simple_bearing`
/// had a chance to publish the home-relative pair.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InitSimpleBearingLeftover {
    /// `simple_cos_yaw = ahrs.cos_yaw()`.
    pub simple_cos_yaw: f32,
    /// `simple_sin_yaw = ahrs.sin_yaw()`.
    pub simple_sin_yaw: f32,
    /// `wrap_2PI(ahrs.get_yaw_rad() + radians(180.0))`.
    pub super_simple_last_bearing_rad: f32,
    /// Copied from [`Self::simple_cos_yaw`], not from the +180° bearing.
    pub super_simple_cos_yaw: f32,
    /// Copied from [`Self::simple_sin_yaw`].
    pub super_simple_sin_yaw: f32,
    /// `Log_Write_Data(INIT_SIMPLE_BEARING, ahrs.yaw_sensor)` — `MASK_LOG_ANY`.
    pub log_init_simple_bearing: bool,
    /// The yaw-sensor value that log line would carry.
    pub logged_yaw_sensor: i32,
}

/// `Copter::init_simple_bearing`.
#[must_use]
pub fn init_simple_bearing(inputs: InitSimpleBearingInputs) -> InitSimpleBearingLeftover {
    let simple_cos_yaw = inputs.ahrs_cos_yaw;
    let simple_sin_yaw = inputs.ahrs_sin_yaw;
    InitSimpleBearingLeftover {
        simple_cos_yaw,
        simple_sin_yaw,
        super_simple_last_bearing_rad: wrap_2pi(inputs.ahrs_yaw_rad + radians(180.0)),
        super_simple_cos_yaw: simple_cos_yaw,
        super_simple_sin_yaw: simple_sin_yaw,
        log_init_simple_bearing: should_log(inputs.log_bitmask, MASK_LOG_ANY),
        logged_yaw_sensor: inputs.ahrs_yaw_sensor,
    }
}

/// Inputs to `Copter::update_simple_mode`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UpdateSimpleModeInputs {
    /// Current `Copter::simple_mode`.
    pub simple_mode: SimpleMode,
    /// `ap.new_radio_frame`.
    pub new_radio_frame: bool,
    /// `rc().has_valid_input()`.
    pub has_valid_input: bool,
    /// `channel_roll->get_control_in()`.
    pub roll_control_in: i16,
    /// `channel_pitch->get_control_in()`.
    pub pitch_control_in: i16,
    /// `simple_cos_yaw`.
    pub simple_cos_yaw: f32,
    /// `simple_sin_yaw`.
    pub simple_sin_yaw: f32,
    /// `super_simple_cos_yaw`.
    pub super_simple_cos_yaw: f32,
    /// `super_simple_sin_yaw`.
    pub super_simple_sin_yaw: f32,
    /// `ahrs.cos_yaw()`.
    pub ahrs_cos_yaw: f32,
    /// `ahrs.sin_yaw()`.
    pub ahrs_sin_yaw: f32,
}

/// What `Copter::update_simple_mode` asked the RC channels / `ap` to do.
///
/// The frame-consume write sits *before* the valid-input refuse. A port
/// that returned on bad RC without clearing `new_radio_frame` would keep
/// rotating the same bind-time PWM every later tick once valid input
/// arrived. NONE / no-new-frame return without touching the flag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UpdateSimpleModeLeftover {
    /// `ap.new_radio_frame = false` ran.
    pub consumed_radio_frame: bool,
    /// Stick rotation ran and the channels were written.
    pub rotated: bool,
    /// `channel_roll` after this call (unchanged when not rotated).
    pub roll_control_in: i16,
    /// `channel_pitch` after this call (unchanged when not rotated).
    pub pitch_control_in: i16,
}

/// `Copter::update_simple_mode`.
///
/// SIMPLE rotates by the arming heading; SuperSimple rotates by the
/// home-relative heading. Both then rotate from north-facing into the
/// vehicle's current yaw. Folding those two rotations into one matrix
/// would still match one heading, but would hide that the first pair
/// is captured at arm and the second is live AHRS.
#[must_use]
pub fn update_simple_mode(inputs: UpdateSimpleModeInputs) -> UpdateSimpleModeLeftover {
    if inputs.simple_mode == SimpleMode::None || !inputs.new_radio_frame {
        return UpdateSimpleModeLeftover {
            consumed_radio_frame: false,
            rotated: false,
            roll_control_in: inputs.roll_control_in,
            pitch_control_in: inputs.pitch_control_in,
        };
    }

    if !inputs.has_valid_input {
        return UpdateSimpleModeLeftover {
            consumed_radio_frame: true,
            rotated: false,
            roll_control_in: inputs.roll_control_in,
            pitch_control_in: inputs.pitch_control_in,
        };
    }

    let roll = f32::from(inputs.roll_control_in);
    let pitch = f32::from(inputs.pitch_control_in);
    let (cos_yaw, sin_yaw) = if inputs.simple_mode == SimpleMode::Simple {
        (inputs.simple_cos_yaw, inputs.simple_sin_yaw)
    } else {
        (inputs.super_simple_cos_yaw, inputs.super_simple_sin_yaw)
    };
    let rollx = roll * cos_yaw - pitch * sin_yaw;
    let pitchx = roll * sin_yaw + pitch * cos_yaw;
    UpdateSimpleModeLeftover {
        consumed_radio_frame: true,
        rotated: true,
        roll_control_in: control_in_from_f32(
            rollx * inputs.ahrs_cos_yaw + pitchx * inputs.ahrs_sin_yaw,
        ),
        pitch_control_in: control_in_from_f32(
            -rollx * inputs.ahrs_sin_yaw + pitchx * inputs.ahrs_cos_yaw,
        ),
    }
}

/// Inputs to `Copter::update_super_simple_bearing`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UpdateSuperSimpleBearingInputs {
    /// `force_update` — `set_simple_mode(SUPERSIMPLE)` passes `true`.
    pub force_update: bool,
    /// Current `Copter::simple_mode`.
    pub simple_mode: SimpleMode,
    /// `home_distance_m()`.
    pub home_distance_m: f32,
    /// `home_bearing_rad()`.
    pub home_bearing_rad: f32,
    /// `super_simple_last_bearing_rad` before this call.
    pub super_simple_last_bearing_rad: f32,
    /// `super_simple_cos_yaw` before this call (echoed on refuse).
    pub super_simple_cos_yaw: f32,
    /// `super_simple_sin_yaw` before this call (echoed on refuse).
    pub super_simple_sin_yaw: f32,
}

/// What `Copter::update_super_simple_bearing` wrote onto the vehicle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UpdateSuperSimpleBearingLeftover {
    /// Cos/sin / last-bearing were rewritten.
    pub updated: bool,
    /// `super_simple_last_bearing_rad` after this call.
    pub super_simple_last_bearing_rad: f32,
    /// `super_simple_cos_yaw` after this call.
    pub super_simple_cos_yaw: f32,
    /// `super_simple_sin_yaw` after this call.
    pub super_simple_sin_yaw: f32,
}

/// `Copter::update_super_simple_bearing`.
///
/// Without `force_update` the leftover refuses unless SuperSimple is
/// on *and* the vehicle is outside [`SUPER_SIMPLE_RADIUS_M`]. The 5°
/// deadband then sits on `wrap_PI(last - home)` — a port that compared
/// unwrapped bearings would keep rewriting while the vehicle circled
/// through ±π.
#[must_use]
pub fn update_super_simple_bearing(
    inputs: UpdateSuperSimpleBearingInputs,
) -> UpdateSuperSimpleBearingLeftover {
    if !inputs.force_update {
        if inputs.simple_mode != SimpleMode::SuperSimple {
            return echo_super_simple(inputs);
        }
        if inputs.home_distance_m < SUPER_SIMPLE_RADIUS_M {
            return echo_super_simple(inputs);
        }
    }

    if libm::fabsf(wrap_pi(
        inputs.super_simple_last_bearing_rad - inputs.home_bearing_rad,
    )) < radians(SUPER_SIMPLE_BEARING_THRESH_DEG)
    {
        return echo_super_simple(inputs);
    }

    let last = inputs.home_bearing_rad;
    let angle_rad = last + radians(180.0);
    UpdateSuperSimpleBearingLeftover {
        updated: true,
        super_simple_last_bearing_rad: last,
        super_simple_cos_yaw: libm::cosf(angle_rad),
        super_simple_sin_yaw: libm::sinf(angle_rad),
    }
}

fn echo_super_simple(inputs: UpdateSuperSimpleBearingInputs) -> UpdateSuperSimpleBearingLeftover {
    UpdateSuperSimpleBearingLeftover {
        updated: false,
        super_simple_last_bearing_rad: inputs.super_simple_last_bearing_rad,
        super_simple_cos_yaw: inputs.super_simple_cos_yaw,
        super_simple_sin_yaw: inputs.super_simple_sin_yaw,
    }
}

/// `RC_Channel::set_control_in` takes `int16_t`; the rotation is float.
///
/// Upstream converts by assigning the product into `int16_t`, which
/// truncates toward zero for in-range stick values.
#[allow(
    clippy::cast_possible_truncation,
    reason = "reproduces upstream float -> int16_t assignment of rotated control_in"
)]
fn control_in_from_f32(value: f32) -> i16 {
    value as i16
}
