//! Attitude leftover, upstream `ArduCopter/Attitude.cpp`.
//!
//! Tracked as **COP-021**. AutoYaw `get_heading` / `set_mode` and
//! `update_land_detector` already live in their own modules. This is the
//! rest of the ticket: the Copter-level helpers in Attitude.cpp itself.
//!
//! `Mode::get_pilot_desired_lean_angles_rad` and
//! `get_pilot_desired_yaw_rate_rads` are already ported as conversions in
//! [`crate::stick_nav`] and [`crate::pilot_input`] — they live in
//! `mode.cpp`, not here. [`non_takeoff_throttle`] is already the
//! `get_non_takeoff_throttle` leftover, kept next to the land detector
//! that uses it.
//!
//! # The comment says cm/s; the function is metres
//!
//! `get_pilot_desired_climb_rate_ms` still carries a comment from the
//! rename. The arithmetic is metres per second: `PILOT_SPD_UP` /
//! `PILOT_SPD_DN` are already m/s, and the stick scales them directly.
//!
//! # Deadzone is written back
//!
//! The leftover constrains `THR_DZ` to 0..400 and `.set()`s it. A port
//! that only clamped a local copy would leave an out-of-range parameter
//! sitting there for the next consumer (motors.cpp uses the same field).
//!
//! # Toy-mode adjust is compiled out of the leftover
//!
//! `TOY_MODE_ENABLED` plus `g2.toy_mode.enabled()` is the only path that
//! calls `throttle_adjust`. The adjust itself is `toy_mode.cpp` and is
//! not this leftover; when both switches are on, the caller hands in the
//! post-adjust stick.

use ap_math::scalar::{constrain_value, is_zero, radians};

pub use crate::land_detector::non_takeoff_throttle;

/// Default `THR_DZ`, upstream `THR_DZ_DEFAULT`.
pub const THR_DZ_DEFAULT: i16 = 100;

/// Upper bound written back onto `THR_DZ`.
pub const THR_DZ_MAX: i16 = 400;

/// Default `PILOT_SPD_UP`, m/s.
pub const PILOT_SPD_UP_DEFAULT: f32 = 2.5;

/// Vertical speed that still counts as a hover, m/s. Upstream `0.6`.
pub const HOVER_VEL_D_MAX_MS: f32 = 0.6;

/// dt handed to `motors->update_throttle_hover`. The 100 Hz comment.
pub const HOVER_LEARN_DT_S: f32 = 0.01;

/// Descent speed the pilot stick asks for, upstream
/// `Copter::get_pilot_speed_dn_ms`.
///
/// A zero `PILOT_SPD_DN` is not "do not descend": it means "use the climb
/// speed both ways". `fabsf` then makes a negative parameter climb-speed
/// sized rather than a descent the wrong way.
#[must_use]
pub fn pilot_speed_dn_ms(pilot_speed_dn_ms: f32, pilot_speed_up_ms: f32) -> f32 {
    if is_zero(pilot_speed_dn_ms) {
        libm::fabsf(pilot_speed_up_ms)
    } else {
        libm::fabsf(pilot_speed_dn_ms)
    }
}

/// `THR_DZ` after the leftover `.set(constrain_int16(..., 0, 400))`.
#[must_use]
pub fn constrain_throttle_deadzone(deadzone: i16) -> i16 {
    deadzone.clamp(0, THR_DZ_MAX)
}

/// Which deadband arm `get_pilot_desired_climb_rate_ms` took.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClimbBand {
    /// `!rc().has_valid_input()` — rate forced to zero.
    Failsafe,
    /// Below `mid - dz`. Uses [`pilot_speed_dn_ms`].
    Below,
    /// Inside the deadband, including the edges. Rate is zero.
    Deadband,
    /// Above `mid + dz`. Uses `PILOT_SPD_UP`.
    Above,
}

/// Inputs `Copter::get_pilot_desired_climb_rate_ms` reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClimbRateContext {
    /// `rc().has_valid_input()`.
    pub has_valid_input: bool,
    /// `channel_throttle->get_control_in()`, before toy-mode adjust.
    pub throttle_control: f32,
    /// `TOY_MODE_ENABLED`. Not a runtime parameter.
    pub toy_mode_compiled: bool,
    /// `g2.toy_mode.enabled()`.
    pub toy_mode_enabled: bool,
    /// Leftover of `toy_mode.throttle_adjust`. Ignored unless both toy
    /// switches are on.
    pub throttle_after_toy_adjust: f32,
    /// `g.throttle_deadzone` before the leftover `.set()`.
    pub throttle_deadzone: i16,
    /// Leftover of `get_throttle_mid()`.
    pub throttle_mid: f32,
    /// `g2.pilot_speed_dn_ms` before [`pilot_speed_dn_ms`].
    pub pilot_speed_dn_ms: f32,
    /// `g2.pilot_speed_up_ms`.
    pub pilot_speed_up_ms: f32,
}

impl Default for ClimbRateContext {
    fn default() -> Self {
        Self {
            has_valid_input: true,
            throttle_control: 500.0,
            toy_mode_compiled: false,
            toy_mode_enabled: false,
            throttle_after_toy_adjust: 500.0,
            throttle_deadzone: THR_DZ_DEFAULT,
            throttle_mid: 500.0,
            pilot_speed_dn_ms: 0.0,
            pilot_speed_up_ms: PILOT_SPD_UP_DEFAULT,
        }
    }
}

/// What `Copter::get_pilot_desired_climb_rate_ms` stored and asked for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClimbRateLeftover {
    /// The climb rate, m/s, up positive.
    pub rate_ms: f32,
    /// Which deadband arm produced it.
    pub band: ClimbBand,
    /// Leftover of `channel_throttle->get_control_in()`.
    pub need_throttle_in: bool,
    /// Both toy switches were on, so `throttle_adjust` ran.
    pub need_toy_adjust: bool,
    /// Stick after constrain, and after toy-adjust when that ran.
    pub throttle_control: f32,
    /// Deadzone after the leftover `.set()`.
    pub deadzone: i16,
    /// The parameter write ran. False only on the failsafe return.
    pub deadzone_written: bool,
    /// Leftover of `get_throttle_mid()`.
    pub need_throttle_mid: bool,
    /// The below-deadband arm asked for [`pilot_speed_dn_ms`].
    pub need_speed_dn: bool,
    /// The above-deadband arm asked for `PILOT_SPD_UP`.
    pub need_speed_up: bool,
}

/// Pilot throttle to climb rate, leftover of
/// `Copter::get_pilot_desired_climb_rate_ms`.
///
/// The deadband is inclusive: sitting exactly on `mid ± dz` is zero, not
/// a hair of climb. The below-arm divides by `deadband_bottom` (mid − dz),
/// not by mid, so a wide deadzone steepens the descent half.
#[must_use]
pub fn get_pilot_desired_climb_rate_ms(ctx: &ClimbRateContext) -> ClimbRateLeftover {
    if !ctx.has_valid_input {
        return ClimbRateLeftover {
            rate_ms: 0.0,
            band: ClimbBand::Failsafe,
            need_throttle_in: false,
            need_toy_adjust: false,
            throttle_control: 0.0,
            deadzone: ctx.throttle_deadzone,
            deadzone_written: false,
            need_throttle_mid: false,
            need_speed_dn: false,
            need_speed_up: false,
        };
    }

    let need_toy_adjust = ctx.toy_mode_compiled && ctx.toy_mode_enabled;
    let throttle_raw = if need_toy_adjust {
        ctx.throttle_after_toy_adjust
    } else {
        ctx.throttle_control
    };
    let throttle_control = constrain_value(throttle_raw, 0.0, 1000.0);
    let deadzone = constrain_throttle_deadzone(ctx.throttle_deadzone);
    let deadband_top = ctx.throttle_mid + f32::from(deadzone);
    let deadband_bottom = ctx.throttle_mid - f32::from(deadzone);

    let (band, rate_ms, need_speed_dn, need_speed_up) = if throttle_control < deadband_bottom {
        let speed_dn = pilot_speed_dn_ms(ctx.pilot_speed_dn_ms, ctx.pilot_speed_up_ms);
        (
            ClimbBand::Below,
            speed_dn * (throttle_control - deadband_bottom) / deadband_bottom,
            true,
            false,
        )
    } else if throttle_control > deadband_top {
        (
            ClimbBand::Above,
            ctx.pilot_speed_up_ms * (throttle_control - deadband_top) / (1000.0 - deadband_top),
            false,
            true,
        )
    } else {
        (ClimbBand::Deadband, 0.0, false, false)
    };

    ClimbRateLeftover {
        rate_ms,
        band,
        need_throttle_in: true,
        need_toy_adjust,
        throttle_control,
        deadzone,
        deadzone_written: true,
        need_throttle_mid: true,
        need_speed_dn,
        need_speed_up,
    }
}

/// Why `update_throttle_hover` returned without learning.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HoverLearnSkip {
    /// `!motors->armed()`.
    Disarmed,
    /// `ap.land_complete`.
    LandComplete,
    /// `standby_active`.
    Standby,
    /// `flightmode->has_manual_throttle()`.
    ManualThrottle,
    /// `mode_number() == DRIFT`.
    Drift,
    /// `!is_zero(pos_control->get_vel_desired_U_ms())`.
    VerticalDemand,
    /// `!ahrs.get_velocity_D`.
    NoVelocityD,
    /// Armed, level-enough check failed: throttle, descent, or lean.
    NotLevelHover,
}

/// Inputs `Copter::update_throttle_hover` reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HoverLearnContext {
    /// `motors->armed()`.
    pub armed: bool,
    /// `ap.land_complete`.
    pub land_complete: bool,
    /// `standby_active`.
    pub standby_active: bool,
    /// `flightmode->has_manual_throttle()`.
    pub has_manual_throttle: bool,
    /// `flightmode->mode_number() == Mode::Number::DRIFT`.
    pub is_drift: bool,
    /// `pos_control->get_vel_desired_U_ms()`.
    pub vel_desired_u_ms: f32,
    /// `ahrs.get_velocity_D` succeeded.
    pub velocity_d_ok: bool,
    /// Down velocity, m/s. Unread when [`Self::velocity_d_ok`] is false.
    pub vel_d_ms: f32,
    /// `motors->get_throttle()`.
    pub throttle: f32,
    /// `ahrs.get_roll_rad()`.
    pub roll_rad: f32,
    /// `ahrs.get_pitch_rad()`.
    pub pitch_rad: f32,
    /// `attitude_control->get_roll_trim_rad()` — heli hover roll trim.
    pub roll_trim_rad: f32,
    /// `HAL_GYROFFT_ENABLED`. Not a runtime parameter.
    pub gyro_fft_enabled: bool,
    /// `motors->get_throttle_out()`, for the FFT leftover.
    pub throttle_out: f32,
}

impl Default for HoverLearnContext {
    fn default() -> Self {
        Self {
            armed: true,
            land_complete: false,
            standby_active: false,
            has_manual_throttle: false,
            is_drift: false,
            vel_desired_u_ms: 0.0,
            velocity_d_ok: true,
            vel_d_ms: 0.0,
            throttle: 0.5,
            roll_rad: 0.0,
            pitch_rad: 0.0,
            roll_trim_rad: 0.0,
            gyro_fft_enabled: false,
            throttle_out: 0.5,
        }
    }
}

/// What `Copter::update_throttle_hover` asked the vehicle for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HoverLearnLeftover {
    /// Why learning did not run, or `None` when it did.
    pub skip: Option<HoverLearnSkip>,
    /// Reached `get_vel_desired_U_ms`.
    pub need_vel_desired_u: bool,
    /// Reached `ahrs.get_velocity_D`.
    pub need_velocity_d: bool,
    /// Reached `motors->get_throttle`.
    pub need_throttle: bool,
    /// Reached the roll / pitch / trim comparison.
    pub need_attitude: bool,
    /// `motors->update_throttle_hover(0.01)`.
    pub learn: bool,
    /// `HAL_GYROFFT_ENABLED` and we learned, so `update_freq_hover` ran.
    pub learn_gyro_fft: bool,
    /// Leftover of `motors->get_throttle_out` for the FFT call.
    pub need_throttle_out: bool,
}

/// Hover-throttle learning leftover, upstream `Copter::update_throttle_hover`.
///
/// Drift is excluded even though it is not a manual-throttle mode: its
/// throttle assist would corrupt the hover average. A non-zero vertical
/// demand is the same idea — the aircraft is climbing, not hovering.
///
/// `get_velocity_D` failing is a hard skip, unlike the land detector,
/// which treats a failed read as zero. Learning on an unknown descent
/// would pull the hover estimate toward whatever throttle happens to
/// be out while the EKF is lost.
#[must_use]
pub fn update_throttle_hover(ctx: &HoverLearnContext) -> HoverLearnLeftover {
    let idle = HoverLearnLeftover {
        skip: None,
        need_vel_desired_u: false,
        need_velocity_d: false,
        need_throttle: false,
        need_attitude: false,
        learn: false,
        learn_gyro_fft: false,
        need_throttle_out: false,
    };

    if !ctx.armed {
        return HoverLearnLeftover {
            skip: Some(HoverLearnSkip::Disarmed),
            ..idle
        };
    }
    if ctx.land_complete {
        return HoverLearnLeftover {
            skip: Some(HoverLearnSkip::LandComplete),
            ..idle
        };
    }
    if ctx.standby_active {
        return HoverLearnLeftover {
            skip: Some(HoverLearnSkip::Standby),
            ..idle
        };
    }
    if ctx.has_manual_throttle {
        return HoverLearnLeftover {
            skip: Some(HoverLearnSkip::ManualThrottle),
            ..idle
        };
    }
    if ctx.is_drift {
        return HoverLearnLeftover {
            skip: Some(HoverLearnSkip::Drift),
            ..idle
        };
    }

    let need_vel_desired_u = true;
    if !is_zero(ctx.vel_desired_u_ms) {
        return HoverLearnLeftover {
            skip: Some(HoverLearnSkip::VerticalDemand),
            need_vel_desired_u,
            ..idle
        };
    }

    let need_velocity_d = true;
    if !ctx.velocity_d_ok {
        return HoverLearnLeftover {
            skip: Some(HoverLearnSkip::NoVelocityD),
            need_vel_desired_u,
            need_velocity_d,
            ..idle
        };
    }

    let need_throttle = true;
    let need_attitude = true;
    let level = ctx.throttle > 0.0
        && libm::fabsf(ctx.vel_d_ms) < HOVER_VEL_D_MAX_MS
        && libm::fabsf(ctx.roll_rad - ctx.roll_trim_rad) < radians(5.0)
        && libm::fabsf(ctx.pitch_rad) < radians(5.0);

    if !level {
        return HoverLearnLeftover {
            skip: Some(HoverLearnSkip::NotLevelHover),
            need_vel_desired_u,
            need_velocity_d,
            need_throttle,
            need_attitude,
            ..idle
        };
    }

    let learn_gyro_fft = ctx.gyro_fft_enabled;
    HoverLearnLeftover {
        skip: None,
        need_vel_desired_u,
        need_velocity_d,
        need_throttle,
        need_attitude,
        learn: true,
        learn_gyro_fft,
        need_throttle_out: learn_gyro_fft,
    }
}

/// What `Copter::run_rate_controller_main` asked the controllers for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RateControllerMainLeftover {
    /// `AP::scheduler().get_last_loop_time_s()`.
    pub dt_s: f32,
    /// Always: `pos_control->set_dt_s`.
    pub set_pos_control_dt: bool,
    /// Always: `attitude_control->set_dt_s`.
    pub set_attitude_control_dt: bool,
    /// `motors->set_dt_s` — only when the rate thread is off.
    pub set_motors_dt: bool,
    /// `attitude_control->rate_controller_run`.
    pub run_rate_controller: bool,
    /// Always: `rate_controller_target_reset`.
    pub reset_rate_target: bool,
}

/// Rate-controller tick leftover, upstream
/// `Copter::run_rate_controller_main`.
///
/// The rate thread owns `rate_controller_run` and the motors dt when it
/// is on. The target reset still runs on the main thread either way —
/// sysid and other one-shot inputs must not leak into the next iteration
/// just because the rate loop lives elsewhere.
#[must_use]
pub fn run_rate_controller_main(
    last_loop_time_s: f32,
    using_rate_thread: bool,
) -> RateControllerMainLeftover {
    RateControllerMainLeftover {
        dt_s: last_loop_time_s,
        set_pos_control_dt: true,
        set_attitude_control_dt: true,
        set_motors_dt: !using_rate_thread,
        run_rate_controller: !using_rate_thread,
        reset_rate_target: true,
    }
}

/// What `set_accel_throttle_I_from_pilot_throttle` wrote onto the D PID.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AccelThrottleILeftover {
    /// Leftover of `attitude_control->get_throttle_in()`.
    pub need_throttle_in: bool,
    /// Leftover of `motors->get_throttle_hover()`.
    pub need_throttle_hover: bool,
    /// Throttle after constrain to 0..1.
    pub pilot_throttle: f32,
    /// `-(pilot_throttle - hover)` written to the accel-D integrator.
    pub integrator: f32,
}

/// I-term handoff leftover, upstream
/// `Copter::set_accel_throttle_I_from_pilot_throttle`.
///
/// The difference between the last pilot throttle and hover is parked in
/// the vertical accel I so the first autopilot iteration does not jump.
/// The sign is a negation because the D-frame integrator is down-positive
/// while throttle-above-hover is up.
#[must_use]
pub fn set_accel_throttle_i_from_pilot_throttle(
    throttle_in: f32,
    throttle_hover: f32,
) -> AccelThrottleILeftover {
    let pilot_throttle = constrain_value(throttle_in, 0.0, 1.0);
    AccelThrottleILeftover {
        need_throttle_in: true,
        need_throttle_hover: true,
        pilot_throttle,
        integrator: -(pilot_throttle - throttle_hover),
    }
}
