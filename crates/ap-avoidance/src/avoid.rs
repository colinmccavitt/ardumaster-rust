//! `AC_Avoid` enable bits and the fence-aware climb-rate leftover.
//!
//! Upstream `libraries/AC_Avoidance/AC_Avoid.cpp` (`adjust_velocity_z`)
//! and `ArduCopter/mode.cpp` (`Mode::get_avoidance_adjusted_climbrate_ms`).

use ap_fence::{TYPE_ALT_MAX, TYPE_ALT_MIN};
use ap_math::control::sqrt_controller;
use ap_math::scalar::{is_negative, is_positive, is_zero, safe_sqrt};

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

/// `AC_Avoid` enable bitmask and the vertical leftover.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Avoid {
    /// `AVOID_ENABLE` bitmask. Upstream `_enabled`.
    enabled: u8,
    /// `AVOID_BACKZ_SPD`, m/s. Upstream `_backup_speed_max_u_ms`.
    backup_speed_max_u_ms: f32,
}

impl Default for Avoid {
    fn default() -> Self {
        Self::new()
    }
}

impl Avoid {
    /// Param defaults: `AC_AVOID_DEFAULT` and `AVOID_BACKZ_SPD` 0.75.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            enabled: AVOID_DEFAULT,
            backup_speed_max_u_ms: BACKUP_SPEED_MAX_U_MS_DEFAULT,
        }
    }

    /// Seed from `AVOID_ENABLE` / `AVOID_BACKZ_SPD`.
    #[must_use]
    pub const fn from_params(enabled: u8, backup_speed_max_u_ms: f32) -> Self {
        Self {
            enabled,
            backup_speed_max_u_ms,
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
