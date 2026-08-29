//! `AC_Avoid` enable bits, the fence-aware climb-rate leftover, the
//! horizontal leftover (`limit_velocity_NE` plus proximity-backed STOP),
//! and the full `adjust_velocity` leftover (NE / body proximity + Z +
//! NEU backup mix).
//!
//! Upstream `libraries/AC_Avoidance/AC_Avoid.cpp` (`adjust_velocity`,
//! `adjust_velocity_NED_m`, `adjust_velocity_z`, `limit_velocity_NE`,
//! `adjust_velocity_proximity`) and
//! `ArduCopter/mode.cpp` (`Mode::get_avoidance_adjusted_climbrate_ms`).

use ap_fence::{TYPE_ALT_MAX, TYPE_ALT_MIN};
use ap_math::control::sqrt_controller;
use ap_math::scalar::{constrain_value, is_negative, is_positive, is_zero, safe_sqrt, sq};
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;

/// Avoidance disabled. Upstream `AC_AVOID_DISABLED`.
pub const DISABLED: u8 = 0;
/// Stop at the geofence. Upstream `AC_AVOID_STOP_AT_FENCE`.
pub const STOP_AT_FENCE: u8 = 1;
/// Stop from the proximity sensor. Upstream `AC_AVOID_USE_PROXIMITY_SENSOR`.
pub const USE_PROXIMITY_SENSOR: u8 = 2;
/// Stop at the beacon perimeter. Upstream `AC_AVOID_STOP_AT_BEACON_FENCE`.
pub const STOP_AT_BEACON_FENCE: u8 = 4;
/// Default `AVOID_ENABLE` bitmask. Upstream `AC_AVOID_DEFAULT`.
pub const AVOID_DEFAULT: u8 = STOP_AT_FENCE | USE_PROXIMITY_SENSOR;

/// Maximum avoidance accel, cm/s/s. Upstream `AC_AVOID_ACCEL_CMSS_MAX`.
pub const ACCEL_CMSS_MAX: f32 = 100.0;
/// Default `AVOID_BACKZ_SPD`, m/s. Upstream `AP_GROUPINFO` default.
pub const BACKUP_SPEED_MAX_U_MS_DEFAULT: f32 = 0.75;
/// Default `AVOID_BACKUP_SPD`, m/s. Upstream `AP_GROUPINFO` default.
pub const BACKUP_SPEED_MAX_NE_MS_DEFAULT: f32 = 0.75;
/// Default `AVOID_MARGIN`, m. Upstream `AP_GROUPINFO` default.
pub const MARGIN_M_DEFAULT: f32 = 2.0;
/// Default `AVOID_BACKUP_DZ`, m. Upstream `AP_GROUPINFO` default.
pub const BACKUP_DEADZONE_M_DEFAULT: f32 = 0.10;

/// Slide around the obstacle. Upstream `BEHAVIOR_SLIDE`. Copter default.
pub const BEHAVIOR_SLIDE: u8 = 0;
/// Stop before the obstacle. Upstream `BEHAVIOR_STOP`.
pub const BEHAVIOR_STOP: u8 = 1;

/// Injected leftovers of the fence / AHRS reads inside `adjust_velocity_z`.
///
/// Proximity `get_upward_distance` stays later.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdjustVelocityZContext {
    /// Leftover of `AP::fence()` non-null.
    pub fence_present: bool,
    /// Leftover of `fence->get_enabled_fences()`.
    pub fence_enabled: u8,
    /// Leftover of `get_alt_in_alt_min_frame_m`. `None` skips the floor.
    pub alt_min_u_m: Option<f32>,
    /// Leftover of `fence->get_safe_alt_min_m`.
    pub safe_alt_min_m: f32,
    /// Leftover of `get_alt_in_alt_max_frame_m`. `None` skips the ceiling.
    pub alt_max_u_m: Option<f32>,
    /// Leftover of `fence->get_safe_alt_max_m`.
    pub safe_alt_max_m: f32,
    /// Leftover of `_ahrs.get_hgt_ctrl_limit` (UP, metres).
    pub hgt_ctrl_limit_m: Option<f32>,
    /// Leftover of `_ahrs.get_relative_position_D_origin_float` (DOWN, metres).
    pub curr_alt_d_m: Option<f32>,
}

impl Default for AdjustVelocityZContext {
    fn default() -> Self {
        Self {
            fence_present: false,
            fence_enabled: 0,
            alt_min_u_m: None,
            safe_alt_min_m: 0.0,
            alt_max_u_m: None,
            safe_alt_max_m: 0.0,
            hgt_ctrl_limit_m: None,
            curr_alt_d_m: None,
        }
    }
}

/// Leftover of one `AC_Avoid::adjust_velocity_z` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdjustVelocityZLeftover {
    /// Climb rate after the 5-arg body (backup not yet mixed).
    pub climb_rate_cms: f32,
    /// Backup speed from a vertical breach. Upstream `backup_speed_cms`.
    pub backup_speed_cms: f32,
    /// Climb rate after the 3-arg wrapper mixes backup in.
    pub climb_rate_applied_cms: f32,
    /// Floor limit was armed (`limit_min_alt`).
    pub limit_min_alt: bool,
    /// Ceiling limit was armed (`limit_max_alt`).
    pub limit_max_alt: bool,
}

/// Injected leftovers of `AP::proximity()` / AHRS inside
/// `adjust_velocity_proximity`.
///
/// ADR-0004 forbids those singletons. [`ProximityStopContext::obstacle_neu_cm`]
/// is the leftover of `get_obstacle`. [`ProximityStopContext::intersect_limit_neu_cm`]
/// is the leftover of `closest_point_from_segment_to_obstacle` on the
/// projected stopping-point segment. Both are body-frame NEU centimetres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProximityStopContext {
    /// Leftover of `AP::proximity()` non-null.
    pub proximity_present: bool,
    /// Leftover of `_proximity_alt_enabled`.
    pub proximity_alt_enabled: bool,
    /// Leftover of `get_obstacle_count()`.
    pub obstacle_count: u8,
    /// Leftover of `_ahrs.earth_to_body2D` / `body_to_earth2D` yaw, radians.
    pub yaw_rad: f32,
    /// Leftover of `get_obstacle`. `None` is an invalid reading.
    pub obstacle_neu_cm: Option<Vector3f>,
    /// Leftover of `closest_point_from_segment_to_obstacle`. `None` means
    /// the stopping-point segment does not intersect this obstacle.
    pub intersect_limit_neu_cm: Option<Vector3f>,
}

impl Default for ProximityStopContext {
    fn default() -> Self {
        Self {
            proximity_present: false,
            proximity_alt_enabled: true,
            obstacle_count: 0,
            yaw_rad: 0.0,
            obstacle_neu_cm: None,
            intersect_limit_neu_cm: None,
        }
    }
}

/// Leftover of one `AC_Avoid::adjust_velocity_proximity` call (STOP / SLIDE).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProximityStopLeftover {
    /// Desired NEU velocity after the proximity arm, earth frame, cm/s.
    pub desired_vel_neu_cms: Vector3f,
    /// Backup NEU velocity, earth frame, cm/s. Upstream `backup_vel_neu_cms`.
    pub backup_vel_neu_cms: Vector3f,
    /// Body-frame stopping point plus margin. Zero when desired vel is zero.
    pub stopping_point_plus_margin_neu_cm: Vector3f,
    /// STOP armed and zeroed the body-frame velocity (`limit_distance <= margin`).
    pub stopped: bool,
    /// `limit_velocity_NE` / `limit_velocity_NEU` changed the body-frame velocity.
    pub limited: bool,
}

/// NE leftover of `AC_Avoid::adjust_velocity` with only the proximity arm.
///
/// Fence / beacon / `limit_accel_NEU_cm` / vertical mix stay later.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdjustVelocityNeLeftover {
    /// Desired NEU velocity after proximity + NE backup mix, cm/s.
    pub desired_vel_neu_cms: Vector3f,
    /// Horizontal backup after `AVOID_BACKUP_SPD` length limit, cm/s.
    pub backup_vel_ne_cms: Vector2f,
    /// Upstream `backing_up` for the NE axes.
    pub backing_up: bool,
}

/// Injected leftovers for the full `AC_Avoid::adjust_velocity` leftover.
///
/// ADR-0004 forbids the fence / proximity / AHRS singletons. Fence NE
/// (circle / polygon / beacon) and `limit_accel_NEU_cm` stay later.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdjustVelocityContext {
    /// Leftover of `AP::proximity()` / AHRS yaw inside the proximity arm.
    pub proximity: ProximityStopContext,
    /// Leftover of `AP::fence()` / AHRS height inside `adjust_velocity_z`.
    pub vertical: AdjustVelocityZContext,
}

impl Default for AdjustVelocityContext {
    fn default() -> Self {
        Self {
            proximity: ProximityStopContext::default(),
            vertical: AdjustVelocityZContext::default(),
        }
    }
}

/// Leftover of one `AC_Avoid::adjust_velocity` call.
///
/// Proximity (earth → body → earth) plus the vertical fence tail, then
/// the NE / U backup mix. Circle / polygon / beacon NE stay later.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdjustVelocityLeftover {
    /// Desired NEU velocity after proximity, Z, and backup mix, cm/s.
    pub desired_vel_neu_cms: Vector3f,
    /// Combined backup after `AVOID_BACKUP_SPD` / `AVOID_BACKZ_SPD`, cm/s.
    pub backup_vel_neu_cms: Vector3f,
    /// Upstream `backing_up`.
    pub backing_up: bool,
    /// Proximity STOP zeroed the body-frame velocity.
    pub proximity_stopped: bool,
    /// Proximity `limit_velocity_NEU` changed the body-frame velocity.
    pub proximity_limited: bool,
    /// Vertical floor limit was armed.
    pub limit_min_alt: bool,
    /// Vertical ceiling limit was armed.
    pub limit_max_alt: bool,
}

impl AdjustVelocityLeftover {
    /// Upstream `adjust_velocity_NED_m` output frame, m/s.
    #[must_use]
    pub fn desired_vel_ned_ms(self) -> Vector3f {
        Vector3f::new(
            self.desired_vel_neu_cms.x * 0.01,
            self.desired_vel_neu_cms.y * 0.01,
            -self.desired_vel_neu_cms.z * 0.01,
        )
    }
}

/// `AC_Avoid` enable bitmask, vertical leftover, NE / proximity leftover,
/// and the full `adjust_velocity` leftover.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Avoid {
    /// `AVOID_ENABLE` bitmask. Upstream `_enabled`.
    enabled: u8,
    /// `AVOID_BACKZ_SPD`, m/s. Upstream `_backup_speed_max_u_ms`.
    backup_speed_max_u_ms: f32,
    /// `AVOID_BACKUP_SPD`, m/s. Upstream `_backup_speed_max_ne_ms`.
    backup_speed_max_ne_ms: f32,
    /// `AVOID_MARGIN`, m. Upstream `_margin_m`.
    margin_m: f32,
    /// `AVOID_BEHAVE`. Upstream `_behavior`.
    behavior: u8,
    /// Runtime proximity enable. Upstream `_proximity_enabled`.
    proximity_enabled: bool,
    /// `AVOID_BACKUP_DZ`, m. Upstream `_backup_deadzone_m`.
    backup_deadzone_m: f32,
}

impl Default for Avoid {
    fn default() -> Self {
        Self::new()
    }
}

impl Avoid {
    /// Param defaults: `AC_AVOID_DEFAULT`, Copter SLIDE, and backup 0.75.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            enabled: AVOID_DEFAULT,
            backup_speed_max_u_ms: BACKUP_SPEED_MAX_U_MS_DEFAULT,
            backup_speed_max_ne_ms: BACKUP_SPEED_MAX_NE_MS_DEFAULT,
            margin_m: MARGIN_M_DEFAULT,
            behavior: BEHAVIOR_SLIDE,
            proximity_enabled: true,
            backup_deadzone_m: BACKUP_DEADZONE_M_DEFAULT,
        }
    }

    /// Seed from `AVOID_ENABLE` / `AVOID_BACKZ_SPD`. Other params stay defaults.
    #[must_use]
    pub const fn from_params(enabled: u8, backup_speed_max_u_ms: f32) -> Self {
        Self {
            enabled,
            backup_speed_max_u_ms,
            backup_speed_max_ne_ms: BACKUP_SPEED_MAX_NE_MS_DEFAULT,
            margin_m: MARGIN_M_DEFAULT,
            behavior: BEHAVIOR_SLIDE,
            proximity_enabled: true,
            backup_deadzone_m: BACKUP_DEADZONE_M_DEFAULT,
        }
    }

    /// `enabled()` — `_enabled != AC_AVOID_DISABLED`.
    #[must_use]
    pub const fn enabled(&self) -> bool {
        self.enabled != DISABLED
    }

    /// Raw `AVOID_ENABLE` bitmask.
    #[must_use]
    pub const fn enabled_bits(&self) -> u8 {
        self.enabled
    }

    /// Set `AVOID_ENABLE`.
    pub fn set_enabled(&mut self, bits: u8) {
        self.enabled = bits;
    }

    /// `AVOID_BACKZ_SPD`.
    #[must_use]
    pub const fn backup_speed_max_u_ms(&self) -> f32 {
        self.backup_speed_max_u_ms
    }

    /// Set `AVOID_BACKZ_SPD`.
    pub fn set_backup_speed_max_u_ms(&mut self, speed_ms: f32) {
        self.backup_speed_max_u_ms = speed_ms;
    }

    /// `AVOID_BACKUP_SPD`.
    #[must_use]
    pub const fn backup_speed_max_ne_ms(&self) -> f32 {
        self.backup_speed_max_ne_ms
    }

    /// Set `AVOID_BACKUP_SPD`.
    pub fn set_backup_speed_max_ne_ms(&mut self, speed_ms: f32) {
        self.backup_speed_max_ne_ms = speed_ms;
    }

    /// `AVOID_MARGIN`.
    #[must_use]
    pub const fn margin_m(&self) -> f32 {
        self.margin_m
    }

    /// Set `AVOID_MARGIN`.
    pub fn set_margin_m(&mut self, margin_m: f32) {
        self.margin_m = margin_m;
    }

    /// `AVOID_BEHAVE`.
    #[must_use]
    pub const fn behavior(&self) -> u8 {
        self.behavior
    }

    /// Set `AVOID_BEHAVE`.
    pub fn set_behavior(&mut self, behavior: u8) {
        self.behavior = behavior;
    }

    /// `proximity_avoidance_enable`.
    pub fn proximity_avoidance_enable(&mut self, on_off: bool) {
        self.proximity_enabled = on_off;
    }

    /// `_proximity_enabled && (_enabled & AC_AVOID_USE_PROXIMITY_SENSOR)`.
    #[must_use]
    pub const fn proximity_avoidance_enabled(&self) -> bool {
        self.proximity_enabled && (self.enabled & USE_PROXIMITY_SENSOR) > 0
    }

    /// `AVOID_BACKUP_DZ`.
    #[must_use]
    pub const fn backup_deadzone_m(&self) -> f32 {
        self.backup_deadzone_m
    }

    /// Set `AVOID_BACKUP_DZ`.
    pub fn set_backup_deadzone_m(&mut self, deadzone_m: f32) {
        self.backup_deadzone_m = deadzone_m;
    }

    /// Speed whose stopping distance is exactly `distance`.
    ///
    /// Upstream `AC_Avoid::get_max_speed`. `kP == 0` is the linear
    /// (`safe_sqrt(2 * distance * accel)`) arm; otherwise
    /// [`sqrt_controller`].
    #[must_use]
    pub fn get_max_speed(k_p: f32, accel: f32, distance: f32, dt: f32) -> f32 {
        if is_zero(k_p) {
            safe_sqrt(2.0 * distance * accel)
        } else {
            sqrt_controller(distance, k_p, accel, dt)
        }
    }

    /// Distance required to stop, given current speed.
    ///
    /// Upstream `AC_Avoid::get_stopping_distance` (copied from
    /// `AC_PosControl`). Units follow the caller — Copter uses cm.
    #[must_use]
    pub fn get_stopping_distance(k_p: f32, accel_cmss: f32, speed_cms: f32) -> f32 {
        if accel_cmss <= 0.0 || is_zero(speed_cms) {
            return 0.0;
        }
        if k_p <= 0.0 {
            return 0.5 * sq(speed_cms) / accel_cmss;
        }
        if speed_cms < accel_cmss / k_p {
            speed_cms / k_p
        } else {
            accel_cmss / (2.0 * k_p * k_p) + (speed_cms * speed_cms) / (2.0 * accel_cmss)
        }
    }

    /// Limit the NE component along `limit_direction_ne`.
    ///
    /// Upstream `AC_Avoid::limit_velocity_NE`. `limit_direction_ne` is a
    /// unit vector. Callers that already measured a distance pass that
    /// distance as `limit_distance_cm`.
    #[must_use]
    pub fn limit_velocity_ne(
        k_p: f32,
        accel_cmss: f32,
        desired_vel_ne_cms: Vector2f,
        limit_direction_ne: Vector2f,
        limit_distance_cm: f32,
        dt: f32,
    ) -> Vector2f {
        let max_speed = Self::get_max_speed(k_p, accel_cmss, limit_distance_cm, dt);
        let speed = desired_vel_ne_cms.dot(limit_direction_ne);
        if speed > max_speed {
            desired_vel_ne_cms + limit_direction_ne * (max_speed - speed)
        } else {
            desired_vel_ne_cms
        }
    }

    /// Limit NEU velocity toward an obstacle, leaving a `margin`.
    ///
    /// Upstream `AC_Avoid::limit_velocity_NEU`. Unit-agnostic: vel,
    /// obstacle, margin, and accel must share a base unit.
    #[must_use]
    pub fn limit_velocity_neu(
        k_p: f32,
        accel_cmss: f32,
        desired_vel_neu_cms: Vector3f,
        obstacle_vector_neu: Vector3f,
        margin: f32,
        k_p_z: f32,
        accel_z_cmss: f32,
        dt: f32,
    ) -> Vector3f {
        if desired_vel_neu_cms.is_zero() {
            return desired_vel_neu_cms;
        }
        let Some(vel_dir) = desired_vel_neu_cms.normalized() else {
            return desired_vel_neu_cms;
        };
        let margin_vector_neu = vel_dir * margin;
        let mut out = desired_vel_neu_cms;
        let limit_direction_ne = obstacle_vector_neu.xy();
        if !limit_direction_ne.is_zero() {
            let distance_from_fence_xy =
                (limit_direction_ne.length() - margin_vector_neu.xy().length()).max(0.0);
            if let Some(dir) = limit_direction_ne.normalized() {
                let velocity_ne = Self::limit_velocity_ne(
                    k_p,
                    accel_cmss,
                    Vector2f::new(out.x, out.y),
                    dir,
                    distance_from_fence_xy,
                    dt,
                );
                out.x = velocity_ne.x;
                out.y = velocity_ne.y;
            }
        }

        if is_zero(out.z) || is_zero(obstacle_vector_neu.z) {
            return out;
        }
        if is_positive(out.z) != is_positive(obstacle_vector_neu.z) {
            return out;
        }

        let velocity_original_u = out.z;
        let speed_u = out.z.abs();
        let dist_u = (obstacle_vector_neu.z.abs() - margin_vector_neu.z.abs()).max(0.0);
        out.z = if is_zero(dist_u) {
            0.0
        } else {
            Self::get_max_speed(k_p_z, accel_z_cmss, dist_u, dt).min(speed_u)
        };
        if is_negative(velocity_original_u) {
            out.z = -out.z;
        }
        out
    }

    /// Proximity-backed STOP / SLIDE leftover.
    ///
    /// Upstream `AC_Avoid::adjust_velocity_proximity` for one injected
    /// obstacle. Fence / beacon / OA planner stay later. AHRS 2-D yaw
    /// rotation is the leftover of `earth_to_body2D` / `body_to_earth2D`.
    #[must_use]
    pub fn adjust_velocity_proximity(
        &self,
        k_p: f32,
        accel_cmss: f32,
        desired_vel_neu_cms: Vector3f,
        k_p_z: f32,
        accel_z_cmss: f32,
        dt: f32,
        ctx: ProximityStopContext,
    ) -> ProximityStopLeftover {
        let identity = ProximityStopLeftover {
            desired_vel_neu_cms,
            backup_vel_neu_cms: Vector3f::zero(),
            stopping_point_plus_margin_neu_cm: Vector3f::zero(),
            stopped: false,
            limited: false,
        };
        if !self.proximity_avoidance_enabled() || !ctx.proximity_alt_enabled {
            return identity;
        }
        if !ctx.proximity_present || ctx.obstacle_count == 0 {
            return identity;
        }

        let desired_vel_body_ne = earth_to_body2d(
            Vector2f::new(desired_vel_neu_cms.x, desired_vel_neu_cms.y),
            ctx.yaw_rad,
        );
        let mut safe_vel_neu = Vector3f::new(
            desired_vel_body_ne.x,
            desired_vel_body_ne.y,
            desired_vel_neu_cms.z,
        );
        let safe_vel_orig = safe_vel_neu;
        let margin_cm = (self.margin_m * 100.0).max(0.0);

        let mut stopping_point = Vector3f::zero();
        if !desired_vel_neu_cms.is_zero() {
            let speed_cms = safe_vel_neu.length();
            if !is_zero(speed_cms) {
                let stop = Self::get_stopping_distance(k_p, accel_cmss, speed_cms);
                stopping_point = safe_vel_neu * ((2.0 + margin_cm + stop) / speed_cms);
            }
        }

        let Some(vector_to_obstacle) = ctx.obstacle_neu_cm else {
            return identity;
        };
        let dist_to_boundary_cm = vector_to_obstacle.length();
        if is_zero(dist_to_boundary_cm) {
            return identity;
        }

        let mut backup_body_ne = Vector2f::zero();
        let mut backup_u = 0.0_f32;
        if is_negative(dist_to_boundary_cm - margin_cm) {
            let breach_dist_cm = margin_cm - dist_to_boundary_cm;
            let deadzone_cm = self.backup_deadzone_m.max(0.0) * 100.0;
            if breach_dist_cm > deadzone_cm {
                if let Some(n) = vector_to_obstacle.normalized() {
                    let margin_vector = n * breach_dist_cm;
                    backup_body_ne = backup_velocity_ne(
                        k_p,
                        accel_cmss,
                        margin_vector.xy().length(),
                        vector_to_obstacle.xy(),
                        dt,
                    );
                    backup_u = backup_velocity_u(k_p_z, accel_z_cmss, margin_vector.z, dt);
                }
            }
        }

        let mut stopped = false;
        let mut limited = false;
        if !desired_vel_neu_cms.is_zero() {
            match self.behavior {
                BEHAVIOR_STOP => {
                    if let Some(limit_direction) = ctx.intersect_limit_neu_cm {
                        let limit_distance_cm = limit_direction.length();
                        if is_zero(limit_distance_cm) {
                            return identity;
                        }
                        if limit_distance_cm <= margin_cm {
                            safe_vel_neu = Vector3f::zero();
                            stopped = true;
                        } else {
                            let limited_vel = Self::limit_velocity_neu(
                                k_p,
                                accel_cmss,
                                safe_vel_neu,
                                limit_direction,
                                margin_cm,
                                k_p_z,
                                accel_z_cmss,
                                dt,
                            );
                            limited = limited_vel != safe_vel_neu;
                            safe_vel_neu = limited_vel;
                        }
                    }
                }
                _ => {
                    let limit_distance_cm = vector_to_obstacle.length();
                    if !is_zero(limit_distance_cm) {
                        let limited_vel = Self::limit_velocity_neu(
                            k_p,
                            accel_cmss,
                            safe_vel_neu,
                            vector_to_obstacle,
                            margin_cm,
                            k_p_z,
                            accel_z_cmss,
                            dt,
                        );
                        limited = limited_vel != safe_vel_neu;
                        safe_vel_neu = limited_vel;
                    }
                }
            }
        }

        if safe_vel_neu == safe_vel_orig && backup_body_ne.is_zero() && is_zero(backup_u) {
            return ProximityStopLeftover {
                desired_vel_neu_cms,
                backup_vel_neu_cms: Vector3f::zero(),
                stopping_point_plus_margin_neu_cm: stopping_point,
                stopped: false,
                limited: false,
            };
        }

        let safe_vel_ne =
            body_to_earth2d(Vector2f::new(safe_vel_neu.x, safe_vel_neu.y), ctx.yaw_rad);
        let backup_ne = body_to_earth2d(backup_body_ne, ctx.yaw_rad);
        ProximityStopLeftover {
            desired_vel_neu_cms: Vector3f::new(safe_vel_ne.x, safe_vel_ne.y, safe_vel_neu.z),
            backup_vel_neu_cms: Vector3f::new(backup_ne.x, backup_ne.y, backup_u),
            stopping_point_plus_margin_neu_cm: stopping_point,
            stopped,
            limited,
        }
    }

    /// NE leftover of `AC_Avoid::adjust_velocity` (proximity arm only).
    ///
    /// Disabled is identity. Fence / beacon / `adjust_velocity_z` /
    /// `limit_accel_NEU_cm` stay later leftovers.
    #[must_use]
    pub fn adjust_velocity_ne(
        &self,
        desired_vel_neu_cms: Vector3f,
        k_p: f32,
        accel_cmss: f32,
        k_p_z: f32,
        accel_z_cmss: f32,
        dt: f32,
        ctx: ProximityStopContext,
    ) -> AdjustVelocityNeLeftover {
        if self.enabled == DISABLED {
            return AdjustVelocityNeLeftover {
                desired_vel_neu_cms,
                backup_vel_ne_cms: Vector2f::zero(),
                backing_up: false,
            };
        }

        let accel_limited_cmss = accel_cmss.min(ACCEL_CMSS_MAX);
        let prox = self.adjust_velocity_proximity(
            k_p,
            accel_limited_cmss,
            desired_vel_neu_cms,
            k_p_z,
            accel_z_cmss,
            dt,
            ctx,
        );

        let mut desired = prox.desired_vel_neu_cms;
        let mut backup = prox.backup_vel_neu_cms;
        let mut backing_up = false;
        let backup_speed_max_ne_cms = self.backup_speed_max_ne_ms * 100.0;
        if !backup.xy().is_zero() && is_positive(backup_speed_max_ne_cms) {
            backing_up = true;
            let mut xy = backup.xy();
            xy.limit_length(backup_speed_max_ne_cms);
            backup.x = xy.x;
            backup.y = xy.y;
            if !is_zero(backup.x) {
                desired.x = if is_positive(backup.x) {
                    desired.x.max(backup.x)
                } else {
                    desired.x.min(backup.x)
                };
            }
            if !is_zero(backup.y) {
                desired.y = if is_positive(backup.y) {
                    desired.y.max(backup.y)
                } else {
                    desired.y.min(backup.y)
                };
            }
        }

        AdjustVelocityNeLeftover {
            desired_vel_neu_cms: desired,
            backup_vel_ne_cms: backup.xy(),
            backing_up,
        }
    }

    /// Full leftover of `AC_Avoid::adjust_velocity`.
    ///
    /// Disabled is identity. The proximity arm rotates earth NE through
    /// body (obstacles are body-frame) and back. The fence tail is
    /// [`Avoid::adjust_velocity_z`] only — circle / polygon / beacon NE
    /// and `limit_accel_NEU_cm` stay later leftovers.
    #[must_use]
    pub fn adjust_velocity(
        &self,
        desired_vel_neu_cms: Vector3f,
        k_p: f32,
        accel_cmss: f32,
        k_p_z: f32,
        accel_z_cmss: f32,
        dt: f32,
        ctx: AdjustVelocityContext,
    ) -> AdjustVelocityLeftover {
        if self.enabled == DISABLED {
            return AdjustVelocityLeftover {
                desired_vel_neu_cms,
                backup_vel_neu_cms: Vector3f::zero(),
                backing_up: false,
                proximity_stopped: false,
                proximity_limited: false,
                limit_min_alt: false,
                limit_max_alt: false,
            };
        }

        let accel_limited_cmss = accel_cmss.min(ACCEL_CMSS_MAX);
        let prox = self.adjust_velocity_proximity(
            k_p,
            accel_limited_cmss,
            desired_vel_neu_cms,
            k_p_z,
            accel_z_cmss,
            dt,
            ctx.proximity,
        );
        let mut desired = prox.desired_vel_neu_cms;

        // `adjust_velocity_fence` tail: vertical fence only.
        let z = self.adjust_velocity_z(k_p_z, accel_z_cmss, desired.z, dt, ctx.vertical);
        desired.z = z.climb_rate_cms;

        let mut q1 = Vector2f::zero();
        let mut q2 = Vector2f::zero();
        let mut q3 = Vector2f::zero();
        let mut q4 = Vector2f::zero();
        let mut back_up = 0.0_f32;
        let mut back_down = 0.0_f32;
        Self::find_max_quadrant_velocity_3d(
            prox.backup_vel_neu_cms,
            &mut q1,
            &mut q2,
            &mut q3,
            &mut q4,
            &mut back_up,
            &mut back_down,
        );
        Self::find_max_quadrant_velocity_3d(
            Vector3f::new(0.0, 0.0, z.backup_speed_cms),
            &mut q1,
            &mut q2,
            &mut q3,
            &mut q4,
            &mut back_up,
            &mut back_down,
        );

        // A single NE source keeps the proximity backup vector. Upstream
        // quadrant binning requires both components non-zero, so an
        // axis-aligned leftover would otherwise vanish before fence NE
        // sources exist to combine.
        let backup_ne = if prox.backup_vel_neu_cms.xy().is_zero() {
            q1 + q2 + q3 + q4
        } else {
            prox.backup_vel_neu_cms.xy()
        };
        let mut backup = Vector3f::new(backup_ne.x, backup_ne.y, back_down + back_up);

        let mut backing_up = false;
        let backup_speed_max_ne_cms = self.backup_speed_max_ne_ms * 100.0;
        if !backup.xy().is_zero() && is_positive(backup_speed_max_ne_cms) {
            backing_up = true;
            let mut xy = backup.xy();
            xy.limit_length(backup_speed_max_ne_cms);
            backup.x = xy.x;
            backup.y = xy.y;
            if !is_zero(backup.x) {
                desired.x = if is_positive(backup.x) {
                    desired.x.max(backup.x)
                } else {
                    desired.x.min(backup.x)
                };
            }
            if !is_zero(backup.y) {
                desired.y = if is_positive(backup.y) {
                    desired.y.max(backup.y)
                } else {
                    desired.y.min(backup.y)
                };
            }
        }

        let backup_speed_max_u_cms = self.backup_speed_max_u_ms * 100.0;
        if !is_zero(backup.z) && is_positive(backup_speed_max_u_cms) {
            backing_up = true;
            backup.z = constrain_value(backup.z, -backup_speed_max_u_cms, backup_speed_max_u_cms);
            if !is_zero(backup.z) {
                desired.z = if is_positive(backup.z) {
                    desired.z.max(backup.z)
                } else {
                    desired.z.min(backup.z)
                };
            }
        }

        AdjustVelocityLeftover {
            desired_vel_neu_cms: desired,
            backup_vel_neu_cms: backup,
            backing_up,
            proximity_stopped: prox.stopped,
            proximity_limited: prox.limited,
            limit_min_alt: z.limit_min_alt,
            limit_max_alt: z.limit_max_alt,
        }
    }

    /// Upstream `AC_Avoid::adjust_velocity_NED_m`.
    ///
    /// Converts NED m/s → NEU cm/s, runs [`Avoid::adjust_velocity`], and
    /// leaves the leftover in NEU cm/s ([`AdjustVelocityLeftover::desired_vel_ned_ms`]).
    #[must_use]
    pub fn adjust_velocity_ned_m(
        &self,
        desired_vel_ned_ms: Vector3f,
        k_p: f32,
        accel_mss: f32,
        k_p_z: f32,
        accel_z_mss: f32,
        dt: f32,
        ctx: AdjustVelocityContext,
    ) -> AdjustVelocityLeftover {
        let desired_vel_neu_cms = Vector3f::new(
            desired_vel_ned_ms.x * 100.0,
            desired_vel_ned_ms.y * 100.0,
            -desired_vel_ned_ms.z * 100.0,
        );
        self.adjust_velocity(
            desired_vel_neu_cms,
            k_p,
            accel_mss * 100.0,
            k_p_z,
            accel_z_mss * 100.0,
            dt,
            ctx,
        )
    }

    /// Bin a backup velocity into the matching NE quadrant.
    ///
    /// Upstream `AC_Avoid::find_max_quadrant_velocity`. Axis-aligned
    /// vectors (a zero component) match no quadrant.
    pub fn find_max_quadrant_velocity(
        desired_vel: Vector2f,
        quad1_vel: &mut Vector2f,
        quad2_vel: &mut Vector2f,
        quad3_vel: &mut Vector2f,
        quad4_vel: &mut Vector2f,
    ) {
        if desired_vel.is_zero() {
            return;
        }
        if is_positive(desired_vel.x) && is_positive(desired_vel.y) {
            quad1_vel.x = quad1_vel.x.max(desired_vel.x);
            quad1_vel.y = quad1_vel.y.max(desired_vel.y);
        }
        if is_negative(desired_vel.x) && is_positive(desired_vel.y) {
            quad2_vel.x = quad2_vel.x.min(desired_vel.x);
            quad2_vel.y = quad2_vel.y.max(desired_vel.y);
        }
        if is_negative(desired_vel.x) && is_negative(desired_vel.y) {
            quad3_vel.x = quad3_vel.x.min(desired_vel.x);
            quad3_vel.y = quad3_vel.y.min(desired_vel.y);
        }
        if is_positive(desired_vel.x) && is_negative(desired_vel.y) {
            quad4_vel.x = quad4_vel.x.max(desired_vel.x);
            quad4_vel.y = quad4_vel.y.min(desired_vel.y);
        }
    }

    /// Horizontal quadrants plus max-up / min-down vertical components.
    ///
    /// Upstream `AC_Avoid::find_max_quadrant_velocity_3D`.
    pub fn find_max_quadrant_velocity_3d(
        desired_vel: Vector3f,
        quad1_vel: &mut Vector2f,
        quad2_vel: &mut Vector2f,
        quad3_vel: &mut Vector2f,
        quad4_vel: &mut Vector2f,
        max_z_vel: &mut f32,
        min_z_vel: &mut f32,
    ) {
        Self::find_max_quadrant_velocity(
            desired_vel.xy(),
            quad1_vel,
            quad2_vel,
            quad3_vel,
            quad4_vel,
        );
        if is_positive(desired_vel.z) && desired_vel.z > *max_z_vel {
            *max_z_vel = desired_vel.z;
        }
        if is_negative(desired_vel.z) && desired_vel.z < *min_z_vel {
            *min_z_vel = desired_vel.z;
        }
    }

    /// Fence-aware climb-rate leftover, upstream `AC_Avoid::adjust_velocity_z`.
    ///
    /// The 5-arg body writes `climb_rate_cms` / `backup_speed_cms`. The
    /// 3-arg wrapper then mixes backup into [`AdjustVelocityZLeftover::climb_rate_applied_cms`].
    /// Disabled or a level climb is the identity PosHold already records.
    #[must_use]
    pub fn adjust_velocity_z(
        &self,
        k_p: f32,
        accel_cmss: f32,
        climb_rate_cms: f32,
        dt: f32,
        ctx: AdjustVelocityZContext,
    ) -> AdjustVelocityZLeftover {
        let mut leftover = AdjustVelocityZLeftover {
            climb_rate_cms,
            backup_speed_cms: 0.0,
            climb_rate_applied_cms: climb_rate_cms,
            limit_min_alt: false,
            limit_max_alt: false,
        };

        // `#ifdef AP_AVOID_ENABLE_Z` is always on for Copter.
        if self.enabled == DISABLED || is_zero(climb_rate_cms) {
            return leftover;
        }

        let accel_limited_cmss = accel_cmss.min(ACCEL_CMSS_MAX);
        let mut max_alt_diff_m = 0.0_f32;
        let mut min_alt_diff_m = 0.0_f32;

        if (self.enabled & STOP_AT_FENCE) > 0 && ctx.fence_present {
            if (ctx.fence_enabled & TYPE_ALT_MIN) > 0 {
                if let Some(veh_alt_m) = ctx.alt_min_u_m {
                    min_alt_diff_m = veh_alt_m - ctx.safe_alt_min_m;
                    leftover.limit_min_alt = true;
                }
            }
            if (ctx.fence_enabled & TYPE_ALT_MAX) > 0 {
                if let Some(veh_alt_m) = ctx.alt_max_u_m {
                    max_alt_diff_m = ctx.safe_alt_max_m - veh_alt_m;
                    leftover.limit_max_alt = true;
                }
            }
        }

        if let (Some(alt_limit_m), Some(curr_alt_m)) = (ctx.hgt_ctrl_limit_m, ctx.curr_alt_d_m) {
            let ctrl_alt_diff_m = alt_limit_m + curr_alt_m;
            if !leftover.limit_max_alt || ctrl_alt_diff_m < max_alt_diff_m {
                max_alt_diff_m = ctrl_alt_diff_m;
                leftover.limit_max_alt = true;
            }
        }

        if leftover.limit_max_alt || leftover.limit_min_alt {
            let max_back_spd_cms = self.backup_speed_max_u_ms * 100.0;
            if max_alt_diff_m <= 0.0 && leftover.limit_max_alt {
                leftover.climb_rate_cms = leftover.climb_rate_cms.min(0.0);
                if is_positive(max_back_spd_cms) {
                    leftover.backup_speed_cms =
                        -Self::get_max_speed(k_p, accel_limited_cmss, -max_alt_diff_m * 100.0, dt);
                    leftover.backup_speed_cms = leftover.backup_speed_cms.max(-max_back_spd_cms);
                }
                leftover.climb_rate_applied_cms =
                    apply_backup(leftover.climb_rate_cms, leftover.backup_speed_cms);
                return leftover;
            } else if min_alt_diff_m <= 0.0 && leftover.limit_min_alt {
                leftover.climb_rate_cms = leftover.climb_rate_cms.max(0.0);
                if is_positive(max_back_spd_cms) {
                    leftover.backup_speed_cms =
                        Self::get_max_speed(k_p, accel_limited_cmss, -min_alt_diff_m * 100.0, dt);
                    leftover.backup_speed_cms = leftover.backup_speed_cms.min(max_back_spd_cms);
                }
                leftover.climb_rate_applied_cms =
                    apply_backup(leftover.climb_rate_cms, leftover.backup_speed_cms);
                return leftover;
            }

            if leftover.limit_max_alt {
                let max_alt_max_speed_cms =
                    Self::get_max_speed(k_p, accel_limited_cmss, max_alt_diff_m * 100.0, dt);
                leftover.climb_rate_cms = leftover.climb_rate_cms.min(max_alt_max_speed_cms);
            }
            if leftover.limit_min_alt {
                let max_alt_min_speed =
                    Self::get_max_speed(k_p, accel_limited_cmss, min_alt_diff_m * 100.0, dt);
                leftover.climb_rate_cms = leftover.climb_rate_cms.max(-max_alt_min_speed);
            }
        }

        leftover.climb_rate_applied_cms =
            apply_backup(leftover.climb_rate_cms, leftover.backup_speed_cms);
        leftover
    }
}

/// 3-arg `adjust_velocity_z` tail: mix backup into the climb rate.
fn apply_backup(climb_rate_cms: f32, backup_speed_cms: f32) -> f32 {
    if is_zero(backup_speed_cms) {
        return climb_rate_cms;
    }
    if is_negative(backup_speed_cms) {
        climb_rate_cms.min(backup_speed_cms)
    } else {
        climb_rate_cms.max(backup_speed_cms)
    }
}

/// Leftover of `ahrs.earth_to_body2D`.
fn earth_to_body2d(ef: Vector2f, yaw_rad: f32) -> Vector2f {
    let mut v = ef;
    v.rotate(-yaw_rad);
    v
}

/// Leftover of `ahrs.body_to_earth2D`.
fn body_to_earth2d(bf: Vector2f, yaw_rad: f32) -> Vector2f {
    let mut v = bf;
    v.rotate(yaw_rad);
    v
}

/// Horizontal arm of `calc_backup_velocity_2D` for one obstacle.
fn backup_velocity_ne(
    k_p: f32,
    accel_cmss: f32,
    back_distance_cm: f32,
    limit_direction: Vector2f,
    dt: f32,
) -> Vector2f {
    if limit_direction.is_zero() {
        return Vector2f::zero();
    }
    let Some(dir) = limit_direction.normalized() else {
        return Vector2f::zero();
    };
    let back_speed_cms = Avoid::get_max_speed(k_p, 0.4 * accel_cmss, back_distance_cm.abs(), dt);
    dir * (-back_speed_cms)
}

/// Vertical arm of `calc_backup_velocity_3D` for one obstacle.
fn backup_velocity_u(k_p_z: f32, accel_z_cmss: f32, back_distance_u_cm: f32, dt: f32) -> f32 {
    if is_zero(back_distance_u_cm) {
        return 0.0;
    }
    let mut back_speed_z_cms =
        Avoid::get_max_speed(k_p_z, 0.4 * accel_z_cmss, back_distance_u_cm.abs(), dt);
    if is_positive(back_distance_u_cm) {
        back_speed_z_cms = -back_speed_z_cms;
    }
    back_speed_z_cms
}

/// Copter `Mode::get_avoidance_adjusted_climbrate_ms` leftover.
///
/// `compiled_in == false` is the `#else` arm PosHold / Loiter already
/// document: the climb rate is unchanged. When compiled in, the rate is
/// converted to cm/s, run through the 3-arg `adjust_velocity_z`, and
/// converted back.
#[must_use]
pub fn get_avoidance_adjusted_climbrate_ms(
    compiled_in: bool,
    avoid: &Avoid,
    k_p: f32,
    accel_mss: f32,
    target_rate_ms: f32,
    dt: f32,
    ctx: AdjustVelocityZContext,
) -> f32 {
    if !compiled_in {
        return target_rate_ms;
    }
    let leftover = avoid.adjust_velocity_z(k_p, accel_mss * 100.0, target_rate_ms * 100.0, dt, ctx);
    leftover.climb_rate_applied_cms * 0.01
}
