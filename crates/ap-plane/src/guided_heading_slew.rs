//! GUIDED offboard heading-slew PID (`g2.guidedHeading`).
//!
//! Ports Plane-4.7.0 `ModeGuided::update` real lines 48-71, gated
//! `#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED`. The outer three-way roll
//! selector that decides whether this branch applies lives in
//! [`crate::guided_mode_hookup`]; this module only computes that branch's
//! `nav_roll_cd` and the `target_heading_*` state it writes.
//!
//! `g2.guidedHeading` is `AC_PID` (not `AC_PID_Basic`) — `Parameters.h:568`
//! `AC_PID guidedHeading{5000,0,0,0,10,5,5,5,0}` — so this reuses
//! [`ap_pid::AcPid::update_error`]. `update_load_factor()` stays the
//! caller's job.

use ap_math::scalar::{constrain_int32, degrees, wrap_pi, Real, GRAVITY_MSS};
use ap_pid::{AcPid, PidGains};

/// Upstream `guided_heading_type_t` (`ArduPlane/defines.h:176`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GuidedHeadingType {
    /// `GUIDED_HEADING_NONE` — no heading track. The caller (FW-043) already
    /// gates this branch off when the type is none; if this function is still
    /// invoked, the groundspeed-course error path is used, matching
    /// upstream's `== GUIDED_HEADING_HEADING` / else.
    None = 0,
    /// `GUIDED_HEADING_COG` — hold ground track.
    Cog = 1,
    /// `GUIDED_HEADING_HEADING` — hold a heading (yaw).
    Heading = 2,
}

/// Default `g2.guidedHeading` gains from `Parameters.h:568`.
///
/// Constructor args are `p, i, d, ff, imax, filt_T_hz, filt_E_hz, filt_D_hz,
/// srmax`. `srtau` / `dff` take the `AC_PID` constructor defaults (1.0 / 0.0).
#[must_use]
pub fn guided_heading_pid_gains() -> PidGains {
    PidGains {
        p: 5000.0,
        i: 0.0,
        d: 0.0,
        ff: 0.0,
        imax: 10.0,
        filt_t_hz: 5.0,
        filt_e_hz: 5.0,
        filt_d_hz: 5.0,
        srmax: 0.0,
        ..PidGains::default()
    }
}

/// A fresh `g2.guidedHeading` controller with the Parameters.h defaults.
#[must_use]
pub fn guided_heading_pid() -> AcPid {
    AcPid::new(guided_heading_pid_gains())
}

/// Inputs for the heading-slew PID (real lines 48-71).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GuidedHeadingSlewInputs {
    /// `AP_HAL::millis()` this tick (`tnow`).
    pub now_ms: u32,
    /// `guided_state.target_heading_time_ms` before this tick.
    pub target_heading_time_ms: u32,
    /// `guided_state.target_heading`, radians in `(-pi, pi]`.
    pub target_heading: f32,
    /// `guided_state.target_heading_type`.
    pub target_heading_type: GuidedHeadingType,
    /// `ahrs.get_yaw_rad()`.
    pub yaw_rad: f32,
    /// `ahrs.groundspeed_vector().x` (north, m/s).
    pub groundspeed_x: f32,
    /// `ahrs.groundspeed_vector().y` (east, m/s).
    pub groundspeed_y: f32,
    /// `guided_state.target_heading_accel_limit`, m/s².
    pub target_heading_accel_limit: f32,
    /// `plane.roll_limit_cd`.
    pub roll_limit_cd: i32,
    /// `guided_state.target_heading_limit` from the previous tick — fed into
    /// `update_error` this tick, then replaced by this tick's saturation.
    pub target_heading_limit: bool,
}

/// Result of the heading-slew PID — the values FW-043 already threads as
/// `heading_slew_nav_roll_cd` plus the two `guided_state` fields this
/// branch writes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GuidedHeadingSlewOutput {
    /// `constrain_int32(desired, -bank_limit, bank_limit)`.
    pub nav_roll_cd: i32,
    /// `fabsf(desired) >= bank_limit` after this tick.
    pub target_heading_limit: bool,
    /// `tnow`, stored back into `guided_state.target_heading_time_ms`.
    pub target_heading_time_ms: u32,
}

/// Run one heading-slew PID step and return the constrained `nav_roll_cd`.
///
/// `guided_heading` is `g2.guidedHeading` (`AC_PID`). Does not call
/// `update_load_factor()` — FW-043 already returns that flag for this branch.
#[must_use]
pub fn guided_heading_slew(
    inp: &GuidedHeadingSlewInputs,
    guided_heading: &mut AcPid,
) -> GuidedHeadingSlewOutput {
    // uint32 millis subtraction, then * 1e-3f (real lines 49-51).
    let delta = inp.now_ms.wrapping_sub(inp.target_heading_time_ms) as f32 * 1e-3_f32;
    let target_heading_time_ms = inp.now_ms;

    let error = if inp.target_heading_type == GuidedHeadingType::Heading {
        wrap_pi(inp.target_heading - inp.yaw_rad)
    } else {
        wrap_pi(
            inp.target_heading - f32::atan2(-inp.groundspeed_y, -inp.groundspeed_x)
                + core::f32::consts::PI,
        )
    };

    // degrees(atanf(accel/g)) * 1e2f, then MIN with roll_limit_cd (promotes).
    let mut bank_limit = degrees(f32::atan(inp.target_heading_accel_limit / GRAVITY_MSS)) * 1e2_f32;
    bank_limit = bank_limit.min(inp.roll_limit_cd as f32);

    let desired = guided_heading.update_error(error, delta, inp.target_heading_limit, inp.now_ms);

    let target_heading_limit = desired.abs() >= bank_limit;

    // C++ constrain_int32(float, float, float) converts each arg to int32_t
    // (truncation toward zero) before the integer constrain.
    let nav_roll_cd = constrain_int32(desired as i32, (-bank_limit) as i32, bank_limit as i32);

    GuidedHeadingSlewOutput {
        nav_roll_cd,
        target_heading_limit,
        target_heading_time_ms,
    }
}
