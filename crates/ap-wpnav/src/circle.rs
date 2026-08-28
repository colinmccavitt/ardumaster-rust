//! AC_Circle init / update leftover, upstream `libraries/AC_WPNav/AC_Circle`.
//! Tracked as **COP-011**.
//!
//! Copter-4.7 has no separate `enable()`. The first real calls are
//! [`Circle::init_ned_m`] (explicit NED center) and [`Circle::init`]
//! (center from the current PosControl stopping point, optionally
//! projected one radius along heading). After that the 100 Hz tick is
//! [`Circle::update_ms`]: leftover of `calc_velocities`, the angular
//! ramp, the target seat, then `input_pos_vel_accel_NE_m` /
//! `D_set_pos_target_from_climb_rate_ms` (or the terrain D leftover)
//! and `NE_update_controller`.
//!
//! ADR-0004 forbids the AHRS / PosControl / millis singletons, so the
//! caller supplies yaw, the desired NED seat, PosControl NE speed /
//! accel limits, `dt_s`, and `now_ms`. The PosControl methods
//! `NE_init_controller_stopping_point`, `D_init_controller_stopping_point`,
//! `input_pos_vel_accel_NE_m`, `input_pos_vel_accel_D_m`,
//! `D_set_pos_target_from_climb_rate_ms`, and `NE_update_controller`
//! stay on COP-009; this records that they must run. Terrain-database
//! height is a leftover (`UpdateCircleContext::terrain_u_m`).
//!
//! # What this module does not own
//!
//! `set_center(Location)`, closest-point-on-circle, and parameter
//! conversion stay on a later COP-011 slice. [`crate::loiter`] already
//! owns init / update; its remaining leftover is pilot-accel shaping.

use ap_math::location::get_bearing_rad;
use ap_math::scalar::{
    constrain_value, is_equal, is_positive, is_zero, radians, safe_sqrt, wrap_2pi, wrap_pi, Real,
};
use ap_math::vector3::Vector3f;

/// Default circle radius, metres. Upstream `AC_CIRCLE_RADIUS_M_DEFAULT`.
pub const CIRCLE_RADIUS_M_DEFAULT: f32 = 10.0;
/// Default turn rate, deg/s. Upstream `AC_CIRCLE_RATE_DEFAULT`.
pub const CIRCLE_RATE_DEFAULT: f32 = 20.0;
/// Minimum angular acceleration, deg/s². Upstream
/// `AC_CIRCLE_ANGULAR_ACCEL_MIN`.
pub const CIRCLE_ANGULAR_ACCEL_MIN: f32 = 2.0;
/// Maximum allowed circle radius, metres. Upstream
/// `AC_CIRCLE_RADIUS_MAX_M`.
pub const CIRCLE_RADIUS_MAX_M: f32 = 2000.0;
/// `is_active` window, milliseconds. Upstream hard-codes 200.
pub const CIRCLE_ACTIVE_TIMEOUT_MS: u32 = 200;
/// Default `CIRCLE_OPTIONS` bitmask. Upstream GroupInfo default `1`.
pub const CIRCLE_DEFAULT_OPTIONS: i16 = 1;

/// Bitfields of `CIRCLE_OPTIONS`. Upstream `AC_Circle::CircleOptions`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum CircleOption {
    /// Bit 0 — RC pitch / roll control radius and rate.
    ManualControl = 1 << 0,
    /// Bit 1 — yaw faces the direction of travel.
    FaceDirectionOfTravel = 1 << 1,
    /// Bit 2 — init center at the current position, not one radius ahead.
    InitAtCenter = 1 << 2,
    /// Bit 3 — mount ROI at the circle center.
    RoiAtCenter = 1 << 3,
}

/// Caller-supplied leftovers `init` / `init_ned_m` read from AHRS and
/// PosControl. ADR-0004 forbids those singletons.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InitCircleContext {
    /// `_ahrs.yaw`.
    pub yaw_rad: f32,
    /// `_ahrs.cos_yaw()`.
    pub cos_yaw: f32,
    /// `_ahrs.sin_yaw()`.
    pub sin_yaw: f32,
    /// `_pos_control.get_pos_desired_NED_m()` after the stopping-point
    /// leftover.
    pub pos_desired_ned_m: Vector3f,
    /// `_pos_control.NE_get_max_speed_ms()`.
    pub ne_max_speed_ms: f32,
    /// `_pos_control.NE_get_max_accel_mss()`.
    pub ne_max_accel_mss: f32,
}

impl Default for InitCircleContext {
    fn default() -> Self {
        Self {
            yaw_rad: 0.0,
            cos_yaw: 1.0,
            sin_yaw: 0.0,
            pos_desired_ned_m: Vector3f::zero(),
            ne_max_speed_ms: 5.0,
            ne_max_accel_mss: 2.5,
        }
    }
}

/// Leftover of one `init` / `init_ned_m`. The PosControl stopping-point
/// setters stay on COP-009; this records that they must run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InitCircleLeftover {
    /// Always true: both inits call `NE_init_controller_stopping_point`.
    pub need_ne_init_controller_stopping_point: bool,
    /// Always true: both inits call `D_init_controller_stopping_point`.
    pub need_d_init_controller_stopping_point: bool,
}

/// Caller-supplied leftovers `update_ms` reads from PosControl, HAL, and
/// the terrain source. ADR-0004 forbids those singletons.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UpdateCircleContext {
    /// `AP_HAL::millis` written to `_last_update_ms` on success.
    pub now_ms: u32,
    /// `_pos_control.get_dt_s()`.
    pub dt_s: f32,
    /// `_pos_control.get_pos_desired_NED_m()` used for yaw-to-center.
    pub pos_desired_ned_m: Vector3f,
    /// `_pos_control.get_pos_desired_U_m()` used when the center is not
    /// terrain-relative.
    pub pos_desired_u_m: f32,
    /// `_pos_control.NE_get_max_speed_ms()`.
    pub ne_max_speed_ms: f32,
    /// `_pos_control.NE_get_max_accel_mss()`.
    pub ne_max_accel_mss: f32,
    /// Leftover of `get_terrain_U_m`. Required when the center is
    /// terrain-relative; `None` fails the update.
    pub terrain_u_m: Option<f32>,
}

impl Default for UpdateCircleContext {
    fn default() -> Self {
        Self {
            now_ms: 0,
            dt_s: 0.01,
            pos_desired_ned_m: Vector3f::zero(),
            pos_desired_u_m: 0.0,
            ne_max_speed_ms: 5.0,
            ne_max_accel_mss: 2.5,
            terrain_u_m: None,
        }
    }
}

/// Leftover of one `update_ms` tick. Target construction lives here;
/// PosControl input / NE update stay on COP-009.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UpdateCircleLeftover {
    /// Upstream `update_ms` return: false when terrain is required but
    /// unavailable.
    pub ok: bool,
    /// `input_pos_vel_accel_NE_m` runs on the success path.
    pub need_input_pos_vel_accel_ne: bool,
    /// `input_pos_vel_accel_D_m` runs when the center is terrain-relative.
    pub need_input_pos_vel_accel_d: bool,
    /// `D_set_pos_target_from_climb_rate_ms` runs when the center is
    /// origin-relative.
    pub need_d_set_pos_target_from_climb_rate: bool,
    /// `NE_update_controller` runs on the success path.
    pub need_ne_update_controller: bool,
    /// Position leftover of `input_pos_vel_accel_NE_m` / D input.
    pub target_ned_m: Vector3f,
    /// Climb-rate leftover of `D_set_pos_target_from_climb_rate_ms`.
    pub climb_rate_ms: f32,
}

/// Circle-mode controller. Upstream `AC_Circle`.
///
/// Construction matches the C++ constructor plus BSS-zeroed internals:
/// GroupInfo defaults and `radians(RATE)` on `_rotation_rate_max_rads`.
/// The first real call is [`init`](Self::init) or
/// [`init_ned_m`](Self::init_ned_m).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Circle {
    radius_parm_m: f32,
    rate_parm_degs: f32,
    options: i16,
    center_ned_m: Vector3f,
    radius_m: f32,
    rotation_rate_max_rads: f32,
    yaw_rad: f32,
    angle_rad: f32,
    angle_total_rad: f32,
    angular_vel_rads: f32,
    angular_vel_max_rads: f32,
    angular_accel_radss: f32,
    last_update_ms: u32,
    last_radius_param_m: f32,
    is_terrain_alt: bool,
}

impl Default for Circle {
    fn default() -> Self {
        Self::new()
    }
}

impl Circle {
    /// Construct with Copter GroupInfo defaults. Upstream constructor
    /// plus BSS-zero of the center / angle / terrain members.
    pub fn new() -> Self {
        Self {
            radius_parm_m: CIRCLE_RADIUS_M_DEFAULT,
            rate_parm_degs: CIRCLE_RATE_DEFAULT,
            options: CIRCLE_DEFAULT_OPTIONS,
            center_ned_m: Vector3f::zero(),
            radius_m: 0.0,
            rotation_rate_max_rads: radians(CIRCLE_RATE_DEFAULT),
            yaw_rad: 0.0,
            angle_rad: 0.0,
            angle_total_rad: 0.0,
            angular_vel_rads: 0.0,
            angular_vel_max_rads: 0.0,
            angular_accel_radss: 0.0,
            last_update_ms: 0,
            last_radius_param_m: 0.0,
            is_terrain_alt: false,
        }
    }

    /// `CIRCLE_RADIUS_M` parameter, metres.
    pub fn radius_parm_m(&self) -> f32 {
        self.radius_parm_m
    }

    /// Write `CIRCLE_RADIUS_M` (tests and a later param slice).
    pub fn set_radius_parm_m(&mut self, radius_parm_m: f32) {
        self.radius_parm_m = radius_parm_m;
    }

    /// Flight radius `_radius_m` used by init / update, metres.
    pub fn radius_m(&self) -> f32 {
        self.radius_m
    }

    /// Upstream `get_radius_m`: internal radius, or the parameter when
    /// `_radius_m` is non-positive.
    pub fn get_radius_m(&self) -> f32 {
        if is_positive(self.radius_m) {
            self.radius_m
        } else {
            self.radius_parm_m
        }
    }

    /// Upstream `set_radius_m`. Clamped to [`CIRCLE_RADIUS_MAX_M`].
    pub fn set_radius_m(&mut self, radius_m: f32) {
        self.radius_m = constrain_value(radius_m, 0.0, CIRCLE_RADIUS_MAX_M);
    }

    /// `CIRCLE_RATE` parameter, deg/s. Upstream `get_rate_degs`.
    pub fn get_rate_degs(&self) -> f32 {
        self.rate_parm_degs
    }

    /// Write `CIRCLE_RATE` (tests and a later param slice).
    pub fn set_rate_parm_degs(&mut self, rate_parm_degs: f32) {
        self.rate_parm_degs = rate_parm_degs;
    }

    /// Current angular velocity, deg/s. Upstream `get_rate_current`.
    pub fn get_rate_current(&self) -> f32 {
        ap_math::scalar::degrees(self.angular_vel_rads)
    }

    /// Requested turn rate, rad/s. Upstream `_rotation_rate_max_rads`.
    pub fn rotation_rate_max_rads(&self) -> f32 {
        self.rotation_rate_max_rads
    }

    /// Upstream `set_rate_degs`: writes `_rotation_rate_max_rads` only.
    pub fn set_rate_degs(&mut self, rate_degs: f32) {
        self.rotation_rate_max_rads = radians(rate_degs);
    }

    /// Circle center, NED metres. Upstream `get_center_NED_m`.
    pub fn center_ned_m(&self) -> Vector3f {
        self.center_ned_m
    }

    /// Upstream `set_center_NED_m`.
    pub fn set_center_ned_m(&mut self, center_ned_m: Vector3f, is_terrain_alt: bool) {
        self.center_ned_m = center_ned_m;
        self.is_terrain_alt = is_terrain_alt;
    }

    /// Upstream `center_is_terrain_alt`.
    pub fn center_is_terrain_alt(&self) -> bool {
        self.is_terrain_alt
    }

    /// Desired yaw, radians. Upstream `get_yaw_rad`.
    pub fn get_yaw_rad(&self) -> f32 {
        self.yaw_rad
    }

    /// Current angle around the circle, radians.
    pub fn angle_rad(&self) -> f32 {
        self.angle_rad
    }

    /// Accumulated angle travelled, radians. Upstream `get_angle_total_rad`.
    pub fn get_angle_total_rad(&self) -> f32 {
        self.angle_total_rad
    }

    /// Current angular velocity, rad/s.
    pub fn angular_vel_rads(&self) -> f32 {
        self.angular_vel_rads
    }

    /// Maximum angular velocity after the last `calc_velocities`.
    pub fn angular_vel_max_rads(&self) -> f32 {
        self.angular_vel_max_rads
    }

    /// Angular acceleration limit after the last `calc_velocities`.
    pub fn angular_accel_radss(&self) -> f32 {
        self.angular_accel_radss
    }

    /// Timestamp leftover of the last successful `update_ms`.
    pub fn last_update_ms(&self) -> u32 {
        self.last_update_ms
    }

    /// Write `CIRCLE_OPTIONS` (tests and a later param slice).
    pub fn set_options(&mut self, options: i16) {
        self.options = options;
    }

    /// `CIRCLE_OPTIONS` bit test. Upstream option helpers.
    pub fn option_is_set(&self, option: CircleOption) -> bool {
        (self.options & (option as i16)) != 0
    }

    /// Upstream `pilot_control_enabled`.
    pub fn pilot_control_enabled(&self) -> bool {
        self.option_is_set(CircleOption::ManualControl)
    }

    /// Upstream `roi_at_center`.
    pub fn roi_at_center(&self) -> bool {
        self.option_is_set(CircleOption::RoiAtCenter)
    }

    /// Upstream `is_active`.
    pub fn is_active(&self, now_ms: u32) -> bool {
        now_ms.wrapping_sub(self.last_update_ms) < CIRCLE_ACTIVE_TIMEOUT_MS
    }

    /// Upstream `check_param_change`.
    pub fn check_param_change(&mut self) {
        if !is_equal(self.last_radius_param_m, self.radius_parm_m) {
            self.radius_m = self.radius_parm_m;
            self.last_radius_param_m = self.radius_m;
        }
    }

    /// Explicit NED center. Upstream `init_NED_m`.
    pub fn init_ned_m(
        &mut self,
        center_ned_m: Vector3f,
        is_terrain_alt: bool,
        rate_degs: f32,
        ctx: InitCircleContext,
    ) -> InitCircleLeftover {
        self.center_ned_m = center_ned_m;
        self.is_terrain_alt = is_terrain_alt;
        self.rotation_rate_max_rads = radians(rate_degs);
        self.calc_velocities(true, ctx.ne_max_speed_ms, ctx.ne_max_accel_mss);
        self.init_start_angle(false, ctx.yaw_rad, ctx.pos_desired_ned_m);
        InitCircleLeftover {
            need_ne_init_controller_stopping_point: true,
            need_d_init_controller_stopping_point: true,
        }
    }

    /// Centimetre NEU wrapper. Upstream `init_NEU_cm`.
    pub fn init_neu_cm(
        &mut self,
        center_neu_cm: Vector3f,
        is_terrain_alt: bool,
        rate_degs: f32,
        ctx: InitCircleContext,
    ) -> InitCircleLeftover {
        let center_ned_m = Vector3f::new(
            center_neu_cm.x * 0.01,
            center_neu_cm.y * 0.01,
            -center_neu_cm.z * 0.01,
        );
        self.init_ned_m(center_ned_m, is_terrain_alt, rate_degs, ctx)
    }

    /// Stopping-point init. Upstream `init`.
    pub fn init(&mut self, ctx: InitCircleContext) -> InitCircleLeftover {
        self.radius_m = self.radius_parm_m;
        self.last_radius_param_m = self.radius_m;
        self.rotation_rate_max_rads = radians(self.rate_parm_degs);

        let mut center = ctx.pos_desired_ned_m;
        if !self.option_is_set(CircleOption::InitAtCenter) {
            center.x += self.radius_m * ctx.cos_yaw;
            center.y += self.radius_m * ctx.sin_yaw;
        }
        self.center_ned_m = center;
        self.is_terrain_alt = false;

        self.calc_velocities(true, ctx.ne_max_speed_ms, ctx.ne_max_accel_mss);
        self.init_start_angle(true, ctx.yaw_rad, ctx.pos_desired_ned_m);
        InitCircleLeftover {
            need_ne_init_controller_stopping_point: true,
            need_d_init_controller_stopping_point: true,
        }
    }

    /// One circle tick, climb rate in m/s. Upstream `update_ms`.
    pub fn update_ms(
        &mut self,
        climb_rate_ms: f32,
        ctx: UpdateCircleContext,
    ) -> UpdateCircleLeftover {
        self.calc_velocities(false, ctx.ne_max_speed_ms, ctx.ne_max_accel_mss);

        let dt = ctx.dt_s;
        if self.angular_vel_rads < self.angular_vel_max_rads {
            self.angular_vel_rads += self.angular_accel_radss.abs() * dt;
            self.angular_vel_rads = self.angular_vel_rads.min(self.angular_vel_max_rads);
        }
        if self.angular_vel_rads > self.angular_vel_max_rads {
            self.angular_vel_rads -= self.angular_accel_radss.abs() * dt;
            self.angular_vel_rads = self.angular_vel_rads.max(self.angular_vel_max_rads);
        }

        let angle_change_rad = self.angular_vel_rads * dt;
        self.angle_rad += angle_change_rad;
        self.angle_rad = wrap_pi(self.angle_rad);
        self.angle_total_rad += angle_change_rad;

        if self.is_terrain_alt && ctx.terrain_u_m.is_none() {
            return UpdateCircleLeftover {
                ok: false,
                need_input_pos_vel_accel_ne: false,
                need_input_pos_vel_accel_d: false,
                need_d_set_pos_target_from_climb_rate: false,
                need_ne_update_controller: false,
                target_ned_m: Vector3f::zero(),
                climb_rate_ms,
            };
        }

        let target_d_m = if self.is_terrain_alt {
            self.center_ned_m.z - ctx.terrain_u_m.unwrap_or(0.0)
        } else {
            -ctx.pos_desired_u_m
        };

        let mut target_ned_m = Vector3f::new(self.center_ned_m.x, self.center_ned_m.y, target_d_m);
        if !is_zero(self.radius_m) {
            target_ned_m.x += self.radius_m * (-self.angle_rad).cos();
            target_ned_m.y += -self.radius_m * (-self.angle_rad).sin();
            self.yaw_rad = get_bearing_rad(
                ap_math::vector2::Vector2f::new(ctx.pos_desired_ned_m.x, ctx.pos_desired_ned_m.y),
                ap_math::vector2::Vector2f::new(self.center_ned_m.x, self.center_ned_m.y),
            );
            if self.option_is_set(CircleOption::FaceDirectionOfTravel) {
                self.yaw_rad += if is_positive(self.rotation_rate_max_rads) {
                    -radians(90.0)
                } else {
                    radians(90.0)
                };
                self.yaw_rad = wrap_2pi(self.yaw_rad);
            }
        } else {
            self.yaw_rad = self.angle_rad;
        }

        self.last_update_ms = ctx.now_ms;
        UpdateCircleLeftover {
            ok: true,
            need_input_pos_vel_accel_ne: true,
            need_input_pos_vel_accel_d: self.is_terrain_alt,
            need_d_set_pos_target_from_climb_rate: !self.is_terrain_alt,
            need_ne_update_controller: true,
            target_ned_m,
            climb_rate_ms,
        }
    }

    /// Centimetre-per-second wrapper. Upstream `update_cms`.
    pub fn update_cms(
        &mut self,
        climb_rate_cms: f32,
        ctx: UpdateCircleContext,
    ) -> UpdateCircleLeftover {
        self.update_ms(climb_rate_cms * 0.01, ctx)
    }

    /// Angular limits from radius and rate. Upstream `calc_velocities`.
    pub fn calc_velocities(
        &mut self,
        init_velocity: bool,
        ne_max_speed_ms: f32,
        ne_max_accel_mss: f32,
    ) {
        if self.radius_m <= 0.0 {
            self.angular_vel_max_rads = self.rotation_rate_max_rads;
            self.angular_accel_radss = self
                .angular_vel_max_rads
                .abs()
                .max(radians(CIRCLE_ANGULAR_ACCEL_MIN));
        } else {
            let vel_max_ms = ne_max_speed_ms.min(safe_sqrt(0.5 * ne_max_accel_mss * self.radius_m));
            self.angular_vel_max_rads = vel_max_ms / self.radius_m;
            self.angular_vel_max_rads = constrain_value(
                self.rotation_rate_max_rads,
                -self.angular_vel_max_rads,
                self.angular_vel_max_rads,
            );
            self.angular_accel_radss =
                (ne_max_accel_mss / self.radius_m).max(radians(CIRCLE_ANGULAR_ACCEL_MIN));
        }
        if init_velocity {
            self.angular_vel_rads = 0.0;
        }
    }

    /// Initial angle around the circle. Upstream `init_start_angle`.
    pub fn init_start_angle(
        &mut self,
        use_heading: bool,
        yaw_rad: f32,
        pos_desired_ned_m: Vector3f,
    ) {
        self.angle_total_rad = 0.0;
        if self.radius_m <= 0.0 {
            self.angle_rad = yaw_rad;
            return;
        }
        if use_heading {
            self.angle_rad = wrap_pi(yaw_rad - core::f32::consts::PI);
        } else if is_equal(pos_desired_ned_m.x, self.center_ned_m.x)
            && is_equal(pos_desired_ned_m.y, self.center_ned_m.y)
        {
            self.angle_rad = wrap_pi(yaw_rad - core::f32::consts::PI);
        } else {
            let bearing_rad = (pos_desired_ned_m.y - self.center_ned_m.y)
                .atan2(pos_desired_ned_m.x - self.center_ned_m.x);
            self.angle_rad = wrap_pi(bearing_rad);
        }
    }
}
