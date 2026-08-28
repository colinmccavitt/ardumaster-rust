//! AC_Loiter init / update leftover, upstream `libraries/AC_WPNav/AC_Loiter`.
//! Tracked as **COP-011**.
//!
//! Copter-4.7 has no separate `enable()`. The first real calls are
//! [`Loiter::init_target_m`] (stationary seat on a NE position) and
//! [`Loiter::init_target`] (re-init from the current PosControl
//! leftover). After that the 400 Hz tick is [`Loiter::update`]:
//! leftover of [`Loiter::calc_desired_velocity`] then
//! `NE_update_controller`.
//!
//! ADR-0004 forbids the AHRS / PosControl / millis singletons, so the
//! caller supplies lean-angle limits, EKF ground-speed, desired
//! pos/vel, and `now_ms`. The PosControl methods
//! `NE_set_correction_speed_accel_m`, `NE_set_pos_error_max_m`,
//! `NE_init_controller_stopping_point`, `NE_relax_velocity_controller`,
//! `set_pos_desired_NE_m`, `set_pos_vel_accel_NE_m`, and
//! `NE_update_controller` stay on COP-009; this records that they
//! must run. Fence/obstacle velocity adjust stays on COP-026.
//!
//! # What this module does not own
//!
//! Pilot-accel shaping (`set_pilot_desired_acceleration_rad`) is a later
//! COP-011 slice. [`crate::circle`] owns AC_Circle init / update.
//! [`crate::wpnav`] is COP-010 and is not rewritten here.

use ap_math::control::{angle_rad_to_accel_mss, sqrt_controller};
use ap_math::scalar::{
    constrain_value, is_negative, is_positive, is_zero, radians, Real, GRAVITY_MSS,
};
use ap_math::vector2::Vector2f;

/// Default horizontal loiter speed, m/s. Upstream `LOITER_SPEED_DEFAULT_MS`
/// (Copter / QuadPlane, not trad heli).
pub const LOITER_SPEED_DEFAULT_MS: f32 = 12.5;
/// Default braking acceleration, m/s². Upstream
/// `LOITER_BRAKE_ACCEL_DEFAULT_MSS` (Copter).
pub const LOITER_BRAKE_ACCEL_DEFAULT_MSS: f32 = 2.5;
/// Default braking jerk, m/s³. Upstream `LOITER_BRAKE_JERK_DEFAULT_MSSS`
/// (Copter).
pub const LOITER_BRAKE_JERK_DEFAULT_MSSS: f32 = 5.0;
/// Minimum allowed horizontal loiter speed, m/s. Upstream
/// `LOITER_SPEED_MIN_MS`.
pub const LOITER_SPEED_MIN_MS: f32 = 0.2;
/// Default correction acceleration, m/s². Upstream
/// `LOITER_ACCEL_MAX_DEFAULT_MSS`.
pub const LOITER_ACCEL_MAX_DEFAULT_MSS: f32 = 5.0;
/// Brake-start delay after sticks centre, seconds. Upstream
/// `LOITER_BRAKE_START_DELAY_DEFAULT_S`.
pub const LOITER_BRAKE_START_DELAY_DEFAULT_S: f32 = 1.0;
/// Correction-speed leftover written to PosControl, m/s. Upstream
/// `LOITER_VEL_CORRECTION_MAX_MS`.
pub const LOITER_VEL_CORRECTION_MAX_MS: f32 = 2.0;
/// Position-error leftover written to PosControl, m. Upstream
/// `LOITER_POS_CORRECTION_MAX_M`.
pub const LOITER_POS_CORRECTION_MAX_M: f32 = 2.0;
/// `is_active` window, milliseconds. Upstream `LOITER_ACTIVE_TIMEOUT_MS`.
pub const LOITER_ACTIVE_TIMEOUT_MS: u32 = 200;
/// Default `LOITER_OPTIONS` bitmask. Upstream `LOITER_DEFAULT_OPTIONS`.
pub const LOITER_DEFAULT_OPTIONS: i8 = 1;

/// Bitfields of `LOITER_OPTIONS`. Upstream `AC_Loiter::LoiterOption`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i8)]
pub enum LoiterOption {
    /// Bit 0 — coordinated-turn feed-forward on pilot accel.
    CoordinatedTurnEnabled = 1,
}

/// Caller-supplied leftovers `init_target` / `init_target_m` read from
/// AttitudeControl and PosControl. ADR-0004 forbids those singletons.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InitTargetContext {
    /// `_attitude_control.lean_angle_max_rad()` for `sanity_check_params`.
    pub lean_angle_max_rad: f32,
    /// `_pos_control.get_accel_target_NED_mss().xy()` — `init_target` only.
    pub accel_target_ne_mss: Vector2f,
    /// `_pos_control.get_roll_rad()` — `init_target` only.
    pub roll_rad: f32,
    /// `_pos_control.get_pitch_rad()` — `init_target` only.
    pub pitch_rad: f32,
}

impl Default for InitTargetContext {
    fn default() -> Self {
        Self {
            lean_angle_max_rad: 0.0,
            accel_target_ne_mss: Vector2f::zero(),
            roll_rad: 0.0,
            pitch_rad: 0.0,
        }
    }
}

/// Leftover of one `init_target` / `init_target_m`. The PosControl
/// setters stay on COP-009; this records the values they would take.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InitTargetLeftover {
    /// `NE_set_correction_speed_accel_m` speed, always
    /// [`LOITER_VEL_CORRECTION_MAX_MS`].
    pub correction_speed_ms: f32,
    /// `NE_set_correction_speed_accel_m` accel after
    /// [`Loiter::sanity_check_params`].
    pub correction_accel_mss: f32,
    /// `NE_set_pos_error_max_m`, always [`LOITER_POS_CORRECTION_MAX_M`].
    pub pos_error_max_m: f32,
    /// `init_target_m` calls `NE_init_controller_stopping_point`.
    pub need_ne_init_controller_stopping_point: bool,
    /// `init_target` calls `NE_relax_velocity_controller`.
    pub need_ne_relax_velocity_controller: bool,
    /// `init_target_m` leftover of `set_pos_desired_NE_m`.
    pub pos_desired_ne_m: Option<Vector2f>,
}

/// Caller-supplied leftovers `update` / `calc_desired_velocity` read from
/// AHRS, PosControl, and HAL. ADR-0004 forbids those singletons.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UpdateLoiterContext {
    /// `AP_HAL::millis` compared against `_brake_timer_ms`.
    pub now_ms: u32,
    /// `_pos_control.get_dt_s()`. Negative dt skips the write leftover.
    pub dt_s: f32,
    /// `AP::ahrs().getControlLimits` ground-speed limit, m/s.
    pub ekf_gnd_spd_limit_ms: f32,
    /// `_pos_control.get_vel_desired_NED_ms().xy()`.
    pub vel_desired_ne_ms: Vector2f,
    /// `_pos_control.get_pos_desired_NED_m().xy()`.
    pub pos_desired_ne_m: Vector2f,
    /// `_pos_control.NE_get_vel_pid().kP()`.
    pub vel_pid_kp: f32,
    /// `_attitude_control.lean_angle_max_rad()`.
    pub attitude_lean_angle_max_rad: f32,
    /// `_pos_control.get_lean_angle_max_rad()`.
    pub pos_lean_angle_max_rad: f32,
    /// `update(avoidance_on)` argument.
    pub avoidance_on: bool,
}

impl Default for UpdateLoiterContext {
    fn default() -> Self {
        Self {
            now_ms: 0,
            dt_s: 0.01,
            ekf_gnd_spd_limit_ms: 50.0,
            vel_desired_ne_ms: Vector2f::zero(),
            pos_desired_ne_m: Vector2f::zero(),
            vel_pid_kp: 1.0,
            attitude_lean_angle_max_rad: 0.5,
            pos_lean_angle_max_rad: 0.5,
            avoidance_on: true,
        }
    }
}

/// Leftover of one `update` tick. `calc_desired_velocity` lives here;
/// `NE_update_controller` and `AC_Avoid::adjust_velocity` stay later.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UpdateLoiterLeftover {
    /// Always true: `update` always calls `calc_desired_velocity`.
    pub need_calc_desired_velocity: bool,
    /// Always true: `NE_update_controller` runs even when dt is invalid.
    pub need_ne_update_controller: bool,
    /// True when dt was valid and `set_pos_vel_accel_NE_m` would run.
    pub need_set_pos_vel_accel_ne: bool,
    /// True when copter avoidance would run (valid dt and `avoidance_on`).
    pub need_avoidance_adjust_velocity: bool,
    /// Position leftover of `set_pos_vel_accel_NE_m` (pre-avoidance).
    pub pos_desired_ne_m: Vector2f,
    /// Velocity leftover of `set_pos_vel_accel_NE_m` (pre-avoidance).
    pub vel_desired_ne_ms: Vector2f,
    /// Acceleration leftover of `set_pos_vel_accel_NE_m`.
    pub accel_desired_ne_mss: Vector2f,
}

/// Horizontal loiter controller. Upstream `AC_Loiter`.
///
/// Construction matches the C++ constructor plus BSS-zeroed internals:
/// GroupInfo defaults and zeroed predicted / desired state. The first
/// real call is [`init_target_m`](Self::init_target_m) or
/// [`init_target`](Self::init_target).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Loiter {
    angle_max_deg: f32,
    speed_max_ne_ms: f32,
    accel_max_ne_mss: f32,
    brake_accel_max_mss: f32,
    brake_jerk_max_msss: f32,
    brake_delay_s: f32,
    options: i8,
    desired_accel_ne_mss: Vector2f,
    predicted_accel_ne_mss: Vector2f,
    predicted_euler_angle_rad: Vector2f,
    predicted_euler_rate: Vector2f,
    predicted_euler_accel: Vector2f,
    brake_timer_ms: u32,
    brake_accel_mss: f32,
}

impl Default for Loiter {
    fn default() -> Self {
        Self::new()
    }
}

impl Loiter {
    /// Construct with Copter GroupInfo defaults. Upstream constructor
    /// plus BSS-zero of the predicted / brake members.
    pub fn new() -> Self {
        Self {
            angle_max_deg: 0.0,
            speed_max_ne_ms: LOITER_SPEED_DEFAULT_MS,
            accel_max_ne_mss: LOITER_ACCEL_MAX_DEFAULT_MSS,
            brake_accel_max_mss: LOITER_BRAKE_ACCEL_DEFAULT_MSS,
            brake_jerk_max_msss: LOITER_BRAKE_JERK_DEFAULT_MSSS,
            brake_delay_s: LOITER_BRAKE_START_DELAY_DEFAULT_S,
            options: LOITER_DEFAULT_OPTIONS,
            desired_accel_ne_mss: Vector2f::zero(),
            predicted_accel_ne_mss: Vector2f::zero(),
            predicted_euler_angle_rad: Vector2f::zero(),
            predicted_euler_rate: Vector2f::zero(),
            predicted_euler_accel: Vector2f::zero(),
            brake_timer_ms: 0,
            brake_accel_mss: 0.0,
        }
    }

    /// `LOIT_ANG_MAX`, degrees. Zero means 2/3 of the PSC lean limit.
    pub fn angle_max_deg(&self) -> f32 {
        self.angle_max_deg
    }

    /// Write `LOIT_ANG_MAX` (tests and a later param slice).
    pub fn set_angle_max_deg(&mut self, angle_max_deg: f32) {
        self.angle_max_deg = angle_max_deg;
    }

    /// `LOIT_SPEED_MS` after the last sanity / setter clamp.
    pub fn speed_max_ne_ms(&self) -> f32 {
        self.speed_max_ne_ms
    }

    /// `LOIT_ACC_MAX_M` after the last sanity clamp.
    pub fn accel_max_ne_mss(&self) -> f32 {
        self.accel_max_ne_mss
    }

    /// Write `LOIT_ACC_MAX_M` so a later [`init_target`](Self::init_target)
    /// can exercise the lean-angle clamp.
    pub fn set_accel_max_ne_mss(&mut self, accel_max_ne_mss: f32) {
        self.accel_max_ne_mss = accel_max_ne_mss;
    }

    /// Current braking acceleration, m/s².
    pub fn brake_accel_mss(&self) -> f32 {
        self.brake_accel_mss
    }

    /// Pilot-requested NE acceleration after the last update leftover.
    pub fn desired_accel_ne_mss(&self) -> Vector2f {
        self.desired_accel_ne_mss
    }

    /// Predicted NE acceleration used by the velocity leftover.
    pub fn predicted_accel_ne_mss(&self) -> Vector2f {
        self.predicted_accel_ne_mss
    }

    /// Predicted roll / pitch leftover of `init_target`.
    pub fn predicted_euler_angle_rad(&self) -> Vector2f {
        self.predicted_euler_angle_rad
    }

    /// Predicted roll / pitch rate leftover of `init_target`.
    pub fn predicted_euler_rate(&self) -> Vector2f {
        self.predicted_euler_rate
    }

    /// Predicted roll / pitch accel leftover of `init_target`.
    pub fn predicted_euler_accel(&self) -> Vector2f {
        self.predicted_euler_accel
    }

    /// `LOITER_OPTIONS` bit 0. Upstream `loiter_option_is_set`.
    pub fn loiter_option_is_set(&self, option: LoiterOption) -> bool {
        (self.options & (option as i8)) != 0
    }

    /// Maximum pilot-commanded lean angle, radians. Upstream
    /// `get_angle_max_rad`.
    pub fn get_angle_max_rad(
        &self,
        attitude_lean_angle_max_rad: f32,
        pos_lean_angle_max_rad: f32,
    ) -> f32 {
        if !is_positive(self.angle_max_deg) {
            attitude_lean_angle_max_rad.min(pos_lean_angle_max_rad) * (2.0 / 3.0)
        } else {
            radians(self.angle_max_deg).min(pos_lean_angle_max_rad)
        }
    }

    /// Floor horizontal loiter speed. Upstream `set_speed_max_NE_ms`.
    pub fn set_speed_max_ne_ms(&mut self, speed_max_ne_ms: f32) {
        self.speed_max_ne_ms = speed_max_ne_ms.max(LOITER_SPEED_MIN_MS);
    }

    /// Soften horizontal gains for landing. Upstream `soften_for_landing`.
    /// PosControl `NE_soften_for_landing` stays on COP-009.
    pub fn soften_for_landing(&self) -> bool {
        true
    }

    /// Stationary loiter seat. Upstream `init_target_m`.
    pub fn init_target_m(
        &mut self,
        position_ne_m: Vector2f,
        ctx: InitTargetContext,
    ) -> InitTargetLeftover {
        self.sanity_check_params(ctx.lean_angle_max_rad);
        self.predicted_accel_ne_mss = Vector2f::zero();
        self.desired_accel_ne_mss = Vector2f::zero();
        self.predicted_euler_angle_rad = Vector2f::zero();
        self.brake_accel_mss = 0.0;
        InitTargetLeftover {
            correction_speed_ms: LOITER_VEL_CORRECTION_MAX_MS,
            correction_accel_mss: self.accel_max_ne_mss,
            pos_error_max_m: LOITER_POS_CORRECTION_MAX_M,
            need_ne_init_controller_stopping_point: true,
            need_ne_relax_velocity_controller: false,
            pos_desired_ne_m: Some(position_ne_m),
        }
    }

    /// Re-init from the current PosControl leftover. Upstream `init_target`.
    pub fn init_target(&mut self, ctx: InitTargetContext) -> InitTargetLeftover {
        self.sanity_check_params(ctx.lean_angle_max_rad);
        self.predicted_accel_ne_mss = ctx.accel_target_ne_mss;
        self.predicted_euler_angle_rad = Vector2f::new(ctx.roll_rad, ctx.pitch_rad);
        self.predicted_euler_rate = Vector2f::zero();
        self.predicted_euler_accel = Vector2f::zero();
        self.brake_accel_mss = 0.0;
        InitTargetLeftover {
            correction_speed_ms: LOITER_VEL_CORRECTION_MAX_MS,
            correction_accel_mss: self.accel_max_ne_mss,
            pos_error_max_m: LOITER_POS_CORRECTION_MAX_M,
            need_ne_init_controller_stopping_point: false,
            need_ne_relax_velocity_controller: true,
            pos_desired_ne_m: None,
        }
    }

    /// One loiter tick. Upstream `update`.
    pub fn update(&mut self, ctx: UpdateLoiterContext) -> UpdateLoiterLeftover {
        let avoidance_on = ctx.avoidance_on;
        let mut leftover = self.calc_desired_velocity(ctx);
        leftover.need_calc_desired_velocity = true;
        leftover.need_ne_update_controller = true;
        leftover.need_avoidance_adjust_velocity =
            leftover.need_set_pos_vel_accel_ne && avoidance_on;
        leftover
    }

    /// Feed-forward velocity leftover. Upstream `calc_desired_velocity`.
    /// Avoidance velocity adjust is recorded, not applied (COP-026).
    pub fn calc_desired_velocity(&mut self, ctx: UpdateLoiterContext) -> UpdateLoiterLeftover {
        let gnd_speed_limit_ms = self
            .speed_max_ne_ms
            .min(ctx.ekf_gnd_spd_limit_ms)
            .max(LOITER_SPEED_MIN_MS);
        let pilot_acceleration_max_mss = angle_rad_to_accel_mss(
            self.get_angle_max_rad(ctx.attitude_lean_angle_max_rad, ctx.pos_lean_angle_max_rad),
        );

        if is_negative(ctx.dt_s) {
            return UpdateLoiterLeftover {
                need_calc_desired_velocity: true,
                need_ne_update_controller: false,
                need_set_pos_vel_accel_ne: false,
                need_avoidance_adjust_velocity: false,
                pos_desired_ne_m: ctx.pos_desired_ne_m,
                vel_desired_ne_ms: ctx.vel_desired_ne_ms,
                accel_desired_ne_mss: self.desired_accel_ne_mss,
            };
        }

        let mut desired_vel_ne_ms = ctx.vel_desired_ne_ms + self.predicted_accel_ne_mss * ctx.dt_s;
        let mut loiter_accel_brake_mss = Vector2f::zero();
        let mut desired_speed_ms = desired_vel_ne_ms.length();
        if !is_zero(desired_speed_ms) {
            let desired_vel_norm = desired_vel_ne_ms / desired_speed_ms;
            let drag_decel_mss = pilot_acceleration_max_mss * desired_speed_ms / gnd_speed_limit_ms;

            let mut loiter_brake_accel_mss = 0.0;
            let elapsed_ms = ctx.now_ms.wrapping_sub(self.brake_timer_ms) as f32;
            if elapsed_ms > self.brake_delay_s.max(ctx.dt_s) * 1000.0 {
                let brake_gain = ctx.vel_pid_kp * 0.5;
                loiter_brake_accel_mss = constrain_value(
                    sqrt_controller(
                        desired_speed_ms,
                        brake_gain,
                        self.brake_jerk_max_msss,
                        ctx.dt_s,
                    ),
                    0.0,
                    self.brake_accel_max_mss,
                );
            }

            self.brake_accel_mss += constrain_value(
                loiter_brake_accel_mss - self.brake_accel_mss,
                -self.brake_jerk_max_msss * ctx.dt_s,
                self.brake_jerk_max_msss * ctx.dt_s,
            );
            loiter_accel_brake_mss = desired_vel_norm * self.brake_accel_mss;
            desired_speed_ms =
                (desired_speed_ms - (drag_decel_mss + self.brake_accel_mss) * ctx.dt_s).max(0.0);
            desired_vel_ne_ms = desired_vel_norm * desired_speed_ms;
        }

        self.desired_accel_ne_mss -= loiter_accel_brake_mss;

        let desired_vel_ms = desired_vel_ne_ms.length();
        if desired_vel_ms > gnd_speed_limit_ms {
            desired_vel_ne_ms = desired_vel_ne_ms * (gnd_speed_limit_ms / desired_vel_ms);
        }

        let pos_desired_ne_m = ctx.pos_desired_ne_m + desired_vel_ne_ms * ctx.dt_s;
        UpdateLoiterLeftover {
            need_calc_desired_velocity: true,
            need_ne_update_controller: false,
            need_set_pos_vel_accel_ne: true,
            need_avoidance_adjust_velocity: false,
            pos_desired_ne_m,
            vel_desired_ne_ms: desired_vel_ne_ms,
            accel_desired_ne_mss: self.desired_accel_ne_mss,
        }
    }

    /// Clamp speed and accel. Upstream `sanity_check_params`.
    pub fn sanity_check_params(&mut self, lean_angle_max_rad: f32) {
        self.speed_max_ne_ms = self.speed_max_ne_ms.max(LOITER_SPEED_MIN_MS);
        self.accel_max_ne_mss = self
            .accel_max_ne_mss
            .min(GRAVITY_MSS * lean_angle_max_rad.tan());
    }
}
