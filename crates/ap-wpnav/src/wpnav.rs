//! Constructor, destination set, `update_wpnav`, `advance_wp_target_along_track`, `set_spline_destination_NED_m`, and `set_spline_destination_next_NED_m` leftover.

use ap_math::control::{shape_vel_accel, update_vel_accel};
use ap_math::location::{get_bearing_cd, get_bearing_rad};
use ap_math::scalar::{constrain_value, is_equal, is_positive, is_zero, GRAVITY_MSS};
use ap_math::vector3::Vector3f;

/// Default horizontal acceleration, m/s². Upstream `WPNAV_ACCELERATION_MS`.
pub const WPNAV_ACCELERATION_MS: f32 = 2.5;
/// Default horizontal speed, m/s. Upstream `WP_SPD_DEFAULT`.
pub const WP_SPD_DEFAULT: f32 = 10.0;
/// Minimum horizontal speed, m/s. Upstream `WP_SPD_MIN`.
pub const WP_SPD_MIN: f32 = 0.01;
/// Default waypoint radius, m. Upstream `WP_RADIUS_M_DEFAULT`.
pub const WP_RADIUS_M_DEFAULT: f32 = 2.0;
/// Minimum waypoint radius, m. Upstream `WP_RADIUS_M_MIN`.
pub const WP_RADIUS_M_MIN: f32 = 0.05;
/// Default climb speed, m/s. Upstream `WP_SPD_UP_DEFAULT`.
pub const WP_SPD_UP_DEFAULT: f32 = 2.5;
/// Default descent speed, m/s. Upstream `WP_SPD_DOWN_DEFAULT`.
pub const WP_SPD_DOWN_DEFAULT: f32 = 1.5;
/// Default vertical acceleration, m/s². Upstream `WP_ACC_Z_DEFAULT`.
pub const WP_ACC_Z_DEFAULT: f32 = 1.0;
/// Default waypoint jerk, m/s³. Upstream `WP_JERK` GroupInfo default.
pub const WP_JERK_DEFAULT: f32 = 1.0;
/// Default terrain margin, m. Upstream `TER_MARGIN` GroupInfo default.
pub const TERRAIN_MARGIN_DEFAULT_M: f32 = 10.0;
/// `is_active()` window, milliseconds. Upstream `AC_WPNav::is_active`.
pub const WPNAV_ACTIVE_TIMEOUT_MS: u32 = 200;

/// Attitude limits `calc_scurve_jerk_and_snap` reads from
/// `AC_AttitudeControl`. COP-007 / COP-008 own those getters; we take the
/// values as inputs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AttitudeJerkLimits {
    /// `get_ang_vel_roll_max_rads`.
    pub ang_vel_roll_max_rads: f32,
    /// `get_ang_vel_pitch_max_rads`.
    pub ang_vel_pitch_max_rads: f32,
    /// `get_accel_roll_max_radss`.
    pub accel_roll_max_radss: f32,
    /// `get_accel_pitch_max_radss`.
    pub accel_pitch_max_radss: f32,
    /// `get_input_tc`, seconds. Floor inside the snap formula is 0.1.
    pub input_tc: f32,
}

impl Default for AttitudeJerkLimits {
    fn default() -> Self {
        Self {
            ang_vel_roll_max_rads: 0.0,
            ang_vel_pitch_max_rads: 0.0,
            accel_roll_max_radss: 0.0,
            accel_pitch_max_radss: 0.0,
            input_tc: 0.0,
        }
    }
}

/// Speed and acceleration `wp_and_spline_init_m` programs into
/// `AC_PosControl`. The methods themselves stay on COP-009.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PosControlSpeedAccel {
    /// Horizontal speed written to both max and correction NE limits.
    pub ne_speed_ms: f32,
    /// Horizontal acceleration written to both max and correction NE limits.
    pub ne_accel_mss: f32,
    /// Descent speed (always positive), written to both D max and correction.
    pub speed_down_ms: f32,
    /// Climb speed, written to both D max and correction.
    pub speed_up_ms: f32,
    /// Vertical acceleration, written to both D max and correction.
    pub accel_d_mss: f32,
}

/// Caller-supplied inputs `set_wp_destination_NED_m` reads from
/// PosControl, HAL, and terrain. ADR-0004 forbids those singletons.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SetWpDestinationContext {
    /// `AP_HAL::millis` used by `is_active` and a possible re-init.
    pub now_ms: u32,
    /// Attitude limits re-init forwards to `calc_scurve_jerk_and_snap`.
    pub attitude: AttitudeJerkLimits,
    /// PosControl stopping point used when the previous destination was
    /// interrupted. Same convention as [`WpNav::wp_and_spline_init_m`].
    pub stopping_point_ned_m: Vector3f,
    /// Terrain D offset, metres. Required when `is_terrain_alt` flips.
    pub terrain_d_m: Option<f32>,
}

impl Default for SetWpDestinationContext {
    fn default() -> Self {
        Self {
            now_ms: 0,
            attitude: AttitudeJerkLimits::default(),
            stopping_point_ned_m: Vector3f::zero(),
            terrain_d_m: None,
        }
    }
}

/// Caller-supplied inputs `update_wpnav` reads from PosControl, HAL, and
/// terrain. ADR-0004 forbids those singletons.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UpdateWpNavContext {
    /// `AP_HAL::millis` stamped onto `_wp_last_update_ms`.
    pub now_ms: u32,
    /// `_pos_control.get_dt_s()`, forwarded to the advance leftover.
    pub dt_s: f32,
    /// Terrain D offset, metres. Required when the current dest is
    /// terrain-relative; missing makes `advance_wp_target_along_track`
    /// return false.
    pub terrain_d_m: Option<f32>,
}

impl Default for UpdateWpNavContext {
    fn default() -> Self {
        Self {
            now_ms: 0,
            dt_s: 0.01,
            terrain_d_m: None,
        }
    }
}

/// Leftover of one `update_wpnav` tick. Speed-param detection and the
/// last-update stamp live here. `advance_wp_target_along_track` (S-curve
/// / spline / `shape_vel_accel`) and `NE_update_controller` stay later
/// slices; this records that they must run.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UpdateWpNavLeftover {
    /// True when this tick applied `set_speed_NE_ms` after a `WP_SPD` change.
    pub applied_speed_ne: bool,
    /// True when this tick applied `set_speed_up_ms` after a `WP_SPD_UP` change.
    pub applied_speed_up: bool,
    /// True when this tick applied `set_speed_down_ms` after a `WP_SPD_DN` change.
    pub applied_speed_down: bool,
    /// True when any speed setter asked `update_track_with_speed_accel_limits`.
    pub need_update_track_limits: bool,
    /// Always true: `update_wpnav` always calls `advance_wp_target_along_track`.
    pub need_advance_track: bool,
    /// Always true: `NE_update_controller` runs even when advance fails.
    pub need_ne_update_controller: bool,
    /// C++ return: false only when terrain-alt dest has no terrain offset.
    pub advance_ok: bool,
    /// dt forwarded to the advance leftover.
    pub dt_s: f32,
}

/// Caller-supplied inputs `advance_wp_target_along_track` reads from
/// PosControl. ADR-0004 forbids those singletons. S-curve / spline
/// `advance_target_along_track` stays in `ap-math` (COP-002 / COP-003);
/// the caller reports whether that leftover finished the path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdvanceWpTargetContext {
    /// `_pos_control.get_dt_s()`.
    pub dt_s: f32,
    /// Terrain D offset, metres. Required when the dest is terrain-relative.
    pub terrain_d_m: Option<f32>,
    /// Leftover of `PosControl::terrain_scaler_D_m`. 1.0 when unused.
    pub terrain_scaler: f32,
    /// `PosControl::get_pos_estimate_NED_m`.
    pub pos_estimate_ned_m: Vector3f,
    /// `PosControl::get_pos_offset_NED_m`.
    pub pos_offset_ned_m: Vector3f,
    /// `PosControl::get_vel_desired_NED_ms`.
    pub vel_desired_ned_ms: Vector3f,
    /// `PosControl::get_vel_offset_D_ms`.
    pub vel_offset_d_ms: f32,
    /// `PosControl::get_pos_error_NED_m`.
    pub pos_error_ned_m: Vector3f,
    /// `PosControl::get_vel_estimate_NED_ms`.
    pub vel_estimate_ned_ms: Vector3f,
    /// `PosControl::NE_get_pos_p().kP()`.
    pub pos_p_kp: f32,
    /// `PosControl::get_shaping_jerk_NE_msss`.
    pub shaping_jerk_ne_msss: f32,
    /// Leftover of SCurve / SplineCurve `advance_target_along_track`.
    pub path_finished: bool,
}

impl Default for AdvanceWpTargetContext {
    fn default() -> Self {
        Self {
            dt_s: 0.01,
            terrain_d_m: None,
            terrain_scaler: 1.0,
            pos_estimate_ned_m: Vector3f::zero(),
            pos_offset_ned_m: Vector3f::zero(),
            vel_desired_ned_ms: Vector3f::zero(),
            vel_offset_d_ms: 0.0,
            pos_error_ned_m: Vector3f::zero(),
            vel_estimate_ned_ms: Vector3f::zero(),
            pos_p_kp: 1.0,
            shaping_jerk_ne_msss: 5.0,
            path_finished: false,
        }
    }
}

/// Leftover of one `advance_wp_target_along_track` tick. Track-time
/// shaping and the reached-destination flag live here. S-curve / spline
/// `advance_target_along_track` and `PosControl::set_pos_vel_accel_NED_m`
/// stay later slices.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AdvanceWpTargetLeftover {
    /// C++ return: false only when terrain-alt dest has no terrain offset.
    pub ok: bool,
    /// Always true on success: `set_pos_terrain_target_D_m`.
    pub need_set_pos_terrain_target: bool,
    /// True on success when the current leg is not a spline.
    pub need_scurve_advance: bool,
    /// True on success when the current leg is a spline.
    pub need_spline_advance: bool,
    /// Always true on success: `set_pos_vel_accel_NED_m`.
    pub need_set_pos_vel_accel: bool,
    /// Raw (unfiltered) track-progress scalar, constrained to `[0, 1]`.
    pub raw_track_dt_scalar: f32,
    /// `offset_vel_ms / wp_desired_speed_ne_ms`, or 1 when speed is zero.
    pub vel_dt_scalar: f32,
    /// Filtered `_track_dt_scalar * vel_dt_scalar * dt` forwarded to the
    /// path leftover.
    pub dt_along_track_s: f32,
}

/// The three `wpnav_flags` bits `wp_and_spline_init_m` writes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct WpNavFlags {
    /// True once the current destination has been reached.
    pub reached_destination: bool,
    /// True when the radius may be ignored (fast waypoint).
    pub fast_waypoint: bool,
    /// True once a yaw target has been set. Init does not touch this.
    pub wp_yaw_set: bool,
}

/// Waypoint navigator. Upstream `AC_WPNav`.
///
/// Construction matches the C++ constructor: parameter defaults and the
/// two flags it clears. The first real call is
/// [`wp_and_spline_init_m`](Self::wp_and_spline_init_m).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WpNav {
    wp_speed_ms: f32,
    wp_speed_up_ms: f32,
    wp_speed_down_ms: f32,
    wp_radius_m: f32,
    wp_accel_mss: f32,
    wp_accel_c_mss: f32,
    wp_accel_z_mss: f32,
    wp_jerk_msss: f32,
    terrain_margin_m: f32,

    last_wp_speed_ms: f32,
    last_wp_speed_up_ms: f32,
    last_wp_speed_down_ms: f32,
    check_wp_speed_change: bool,

    wp_desired_speed_ne_ms: f32,
    origin_ned_m: Vector3f,
    destination_ned_m: Vector3f,
    track_dt_scalar: f32,
    offset_vel_ms: f32,
    offset_accel_mss: f32,
    paused: bool,
    is_terrain_alt: bool,
    this_leg_is_spline: bool,
    wp_last_update_ms: u32,
    flags: WpNavFlags,

    scurve_jerk_max_msss: f32,
    scurve_snap_max_mssss: f32,
    scurve_legs_inited: bool,
    pos_control_stopping_point_inited: bool,
    pos_speed_accel: PosControlSpeedAccel,

    next_destination_ned_m: Vector3f,
    next_leg_is_spline: bool,
    scurve_this_leg_calculated: bool,
    pos_terrain_d_m: f32,
    last_arc_rad: f32,
    spline_this_leg_set: bool,
    spline_origin_vel_ned_ms: Vector3f,
    spline_destination_vel_ned_ms: Vector3f,
    spline_next_leg_set: bool,
    spline_next_destination_ned_m: Vector3f,
    spline_next_origin_vel_ned_ms: Vector3f,
    spline_next_destination_vel_ned_ms: Vector3f,
    need_this_leg_dest_speed_max: bool,
}

impl Default for WpNav {
    fn default() -> Self {
        Self::new()
    }
}

impl WpNav {
    /// Construct with the GroupInfo defaults. Upstream `AC_WPNav::AC_WPNav`.
    #[must_use]
    pub fn new() -> Self {
        let wp_speed_ms = WP_SPD_DEFAULT;
        let wp_speed_up_ms = WP_SPD_UP_DEFAULT;
        let wp_speed_down_ms = WP_SPD_DOWN_DEFAULT;
        Self {
            wp_speed_ms,
            wp_speed_up_ms,
            wp_speed_down_ms,
            wp_radius_m: WP_RADIUS_M_DEFAULT,
            wp_accel_mss: WPNAV_ACCELERATION_MS,
            wp_accel_c_mss: 0.0,
            wp_accel_z_mss: WP_ACC_Z_DEFAULT,
            wp_jerk_msss: WP_JERK_DEFAULT,
            terrain_margin_m: TERRAIN_MARGIN_DEFAULT_M,
            last_wp_speed_ms: wp_speed_ms,
            last_wp_speed_up_ms: wp_speed_up_ms,
            last_wp_speed_down_ms: wp_speed_down_ms.abs(),
            check_wp_speed_change: false,
            wp_desired_speed_ne_ms: 0.0,
            origin_ned_m: Vector3f::zero(),
            destination_ned_m: Vector3f::zero(),
            track_dt_scalar: 0.0,
            offset_vel_ms: 0.0,
            offset_accel_mss: 0.0,
            paused: false,
            is_terrain_alt: false,
            this_leg_is_spline: false,
            wp_last_update_ms: 0,
            flags: WpNavFlags::default(),
            scurve_jerk_max_msss: 0.0,
            scurve_snap_max_mssss: 0.0,
            scurve_legs_inited: false,
            pos_control_stopping_point_inited: false,
            pos_speed_accel: PosControlSpeedAccel {
                ne_speed_ms: 0.0,
                ne_accel_mss: 0.0,
                speed_down_ms: 0.0,
                speed_up_ms: 0.0,
                accel_d_mss: 0.0,
            },
            next_destination_ned_m: Vector3f::zero(),
            next_leg_is_spline: false,
            scurve_this_leg_calculated: false,
            pos_terrain_d_m: 0.0,
            last_arc_rad: 0.0,
            spline_this_leg_set: false,
            spline_origin_vel_ned_ms: Vector3f::zero(),
            spline_destination_vel_ned_ms: Vector3f::zero(),
            spline_next_leg_set: false,
            spline_next_destination_ned_m: Vector3f::zero(),
            spline_next_origin_vel_ned_ms: Vector3f::zero(),
            spline_next_destination_vel_ned_ms: Vector3f::zero(),
            need_this_leg_dest_speed_max: false,
        }
    }

    /// Default horizontal speed, m/s. Upstream `get_default_speed_NE_ms`.
    #[must_use]
    pub fn default_speed_ne_ms(&self) -> f32 {
        self.wp_speed_ms
    }

    /// Default climb speed, m/s. Upstream `get_default_speed_up_ms`.
    #[must_use]
    pub fn default_speed_up_ms(&self) -> f32 {
        self.wp_speed_up_ms
    }

    /// Default descent speed, m/s, always positive. Upstream
    /// `get_default_speed_down_ms`.
    #[must_use]
    pub fn default_speed_down_ms(&self) -> f32 {
        libm::fabsf(self.wp_speed_down_ms)
    }

    /// Horizontal acceleration, m/s². Upstream `get_wp_acceleration_mss`.
    #[must_use]
    pub fn wp_acceleration_mss(&self) -> f32 {
        if is_positive(self.wp_accel_mss) {
            self.wp_accel_mss
        } else {
            WPNAV_ACCELERATION_MS
        }
    }

    /// Vertical acceleration, m/s². Upstream `get_accel_D_mss`.
    #[must_use]
    pub fn accel_d_mss(&self) -> f32 {
        self.wp_accel_z_mss
    }

    /// Waypoint radius, m. Upstream `get_wp_radius_m`.
    #[must_use]
    pub fn wp_radius_m(&self) -> f32 {
        self.wp_radius_m
    }

    /// Destination of the current leg, NED metres. Upstream
    /// `get_wp_destination_NED_m`.
    #[must_use]
    pub fn wp_destination_ned_m(&self) -> Vector3f {
        self.destination_ned_m
    }

    /// Origin of the current leg, NED metres. Upstream `get_wp_origin_NED_m`.
    #[must_use]
    pub fn wp_origin_ned_m(&self) -> Vector3f {
        self.origin_ned_m
    }

    /// Destination of the current leg, NEU centimetres. Upstream
    /// `get_wp_destination_NEU_cm`.
    #[must_use]
    pub fn wp_destination_neu_cm(&self) -> Vector3f {
        Vector3f::new(
            self.destination_ned_m.x * 100.0,
            self.destination_ned_m.y * 100.0,
            -self.destination_ned_m.z * 100.0,
        )
    }

    /// Origin of the current leg, NEU centimetres. Upstream
    /// `get_wp_origin_NEU_cm`.
    #[must_use]
    pub fn wp_origin_neu_cm(&self) -> Vector3f {
        Vector3f::new(
            self.origin_ned_m.x * 100.0,
            self.origin_ned_m.y * 100.0,
            -self.origin_ned_m.z * 100.0,
        )
    }

    /// Preloaded next destination, NED metres. Cleared when a new current
    /// destination is set.
    #[must_use]
    pub fn next_destination_ned_m(&self) -> Vector3f {
        self.next_destination_ned_m
    }

    /// True if the current destination has been reached. Upstream
    /// `reached_wp_destination`.
    #[must_use]
    pub fn reached_wp_destination(&self) -> bool {
        self.flags.reached_destination
    }

    /// True if the horizontal (NE) distance is inside the waypoint radius.
    /// Upstream `AC_WPNav::reached_wp_destination_NE`. Z is ignored.
    #[must_use]
    pub fn reached_wp_destination_ne(&self, pos_estimate_ned_m: Vector3f) -> bool {
        self.get_wp_distance_to_destination_m(pos_estimate_ned_m) < self.wp_radius_m
    }

    /// True after `set_wp_destination_NED_m` asked for a new this-leg
    /// `SCurve::calculate_track`. The track object stays in `ap-math`.
    #[must_use]
    pub fn scurve_this_leg_calculated(&self) -> bool {
        self.scurve_this_leg_calculated
    }

    /// Terrain D offset last written to `PosControl::init_pos_terrain_D_m`.
    #[must_use]
    pub fn pos_terrain_d_m(&self) -> f32 {
        self.pos_terrain_d_m
    }

    /// Arc angle last passed to `calculate_track`, radians.
    #[must_use]
    pub fn last_arc_rad(&self) -> f32 {
        self.last_arc_rad
    }

    /// True after `set_spline_destination_NED_m` asked for a new this-leg
    /// `SplineCurve::set_origin_and_destination`. The curve object stays
    /// in `ap-math`.
    #[must_use]
    pub fn spline_this_leg_set(&self) -> bool {
        self.spline_this_leg_set
    }

    /// Origin velocity leftover forwarded to `SplineCurve`, NED m/s.
    #[must_use]
    pub fn spline_origin_vel_ned_ms(&self) -> Vector3f {
        self.spline_origin_vel_ned_ms
    }

    /// Destination velocity leftover forwarded to `SplineCurve`, NED m/s.
    #[must_use]
    pub fn spline_destination_vel_ned_ms(&self) -> Vector3f {
        self.spline_destination_vel_ned_ms
    }

    /// True after `set_spline_destination_next_NED_m` asked for a new
    /// next-leg `SplineCurve::set_origin_and_destination`. The curve
    /// object stays in `ap-math`.
    #[must_use]
    pub fn spline_next_leg_set(&self) -> bool {
        self.spline_next_leg_set
    }

    /// Next-leg destination leftover forwarded to `_spline_next_leg`, NED m.
    #[must_use]
    pub fn spline_next_destination_ned_m(&self) -> Vector3f {
        self.spline_next_destination_ned_m
    }

    /// Next-leg origin velocity leftover forwarded to `_spline_next_leg`, NED m/s.
    #[must_use]
    pub fn spline_next_origin_vel_ned_ms(&self) -> Vector3f {
        self.spline_next_origin_vel_ned_ms
    }

    /// Next-leg destination velocity leftover forwarded to `_spline_next_leg`, NED m/s.
    #[must_use]
    pub fn spline_next_destination_vel_ned_ms(&self) -> Vector3f {
        self.spline_next_destination_vel_ned_ms
    }

    /// Leftover of this-leg `set_destination_speed_max` after a next spline
    /// was preloaded (`_scurve_this_leg` or `_spline_this_leg`).
    #[must_use]
    pub fn need_this_leg_dest_speed_max(&self) -> bool {
        self.need_this_leg_dest_speed_max
    }

    /// True if the preloaded next leg is a spline.
    #[must_use]
    pub fn next_leg_is_spline(&self) -> bool {
        self.next_leg_is_spline
    }

    /// Desired horizontal speed for the current segment, m/s.
    #[must_use]
    pub fn desired_speed_ne_ms(&self) -> f32 {
        self.wp_desired_speed_ne_ms
    }

    /// True when `WP_SPD` should be watched for in-flight changes.
    #[must_use]
    pub fn check_wp_speed_change(&self) -> bool {
        self.check_wp_speed_change
    }

    /// Last recorded `WP_SPD`, m/s. Used to detect in-flight changes.
    #[must_use]
    pub fn last_wp_speed_ms(&self) -> f32 {
        self.last_wp_speed_ms
    }

    /// Last recorded `WP_SPD_UP`, m/s.
    #[must_use]
    pub fn last_wp_speed_up_ms(&self) -> f32 {
        self.last_wp_speed_up_ms
    }

    /// Last recorded `WP_SPD_DN`, m/s.
    #[must_use]
    pub fn last_wp_speed_down_ms(&self) -> f32 {
        self.last_wp_speed_down_ms
    }

    /// The three waypoint flags.
    #[must_use]
    pub fn flags(&self) -> WpNavFlags {
        self.flags
    }

    /// True if the navigator is paused. Upstream `paused()`.
    #[must_use]
    pub fn paused(&self) -> bool {
        self.paused
    }

    /// True if origin/destination Z are terrain-relative.
    #[must_use]
    pub fn origin_and_destination_are_terrain_alt(&self) -> bool {
        self.is_terrain_alt
    }

    /// True if the current leg is a spline.
    #[must_use]
    pub fn this_leg_is_spline(&self) -> bool {
        self.this_leg_is_spline
    }

    /// Track time-step scalar. Init sets this to 1.
    #[must_use]
    pub fn track_dt_scalar(&self) -> f32 {
        self.track_dt_scalar
    }

    /// Filtered horizontal speed used for terrain-margin shaping.
    #[must_use]
    pub fn offset_vel_ms(&self) -> f32 {
        self.offset_vel_ms
    }

    /// Filtered horizontal acceleration used for terrain-margin shaping.
    #[must_use]
    pub fn offset_accel_mss(&self) -> f32 {
        self.offset_accel_mss
    }

    /// Computed S-curve jerk, m/s³.
    #[must_use]
    pub fn scurve_jerk_max_msss(&self) -> f32 {
        self.scurve_jerk_max_msss
    }

    /// Computed S-curve snap, m/s⁴.
    #[must_use]
    pub fn scurve_snap_max_mssss(&self) -> f32 {
        self.scurve_snap_max_mssss
    }

    /// True after init has cleared the three S-curve legs (`SCurve::init`).
    #[must_use]
    pub fn scurve_legs_inited(&self) -> bool {
        self.scurve_legs_inited
    }

    /// True after init has asked PosControl for
    /// `D_init_controller_stopping_point` and `NE_init_controller_stopping_point`.
    #[must_use]
    pub fn pos_control_stopping_point_inited(&self) -> bool {
        self.pos_control_stopping_point_inited
    }

    /// Limits init would write into PosControl.
    #[must_use]
    pub fn pos_speed_accel(&self) -> PosControlSpeedAccel {
        self.pos_speed_accel
    }

    /// Override `WP_SPD` before init. Used by tests and later param load.
    pub fn set_wp_speed_ms(&mut self, speed_ms: f32) {
        self.wp_speed_ms = speed_ms;
    }

    /// Override `WP_RADIUS_M` before init.
    pub fn set_wp_radius_m(&mut self, radius_m: f32) {
        self.wp_radius_m = radius_m;
    }

    /// Override `WP_JERK` before init.
    pub fn set_wp_jerk_msss(&mut self, jerk_msss: f32) {
        self.wp_jerk_msss = jerk_msss;
    }

    /// Override `WP_ACC` before init.
    pub fn set_wp_accel_mss(&mut self, accel_mss: f32) {
        self.wp_accel_mss = accel_mss;
    }

    /// Override `WP_SPD_UP` before a tick. Used by tests and later param load.
    pub fn set_wp_speed_up_ms(&mut self, speed_up_ms: f32) {
        self.wp_speed_up_ms = speed_up_ms;
    }

    /// Override `WP_SPD_DN` before a tick. Used by tests and later param load.
    pub fn set_wp_speed_down_ms(&mut self, speed_down_ms: f32) {
        self.wp_speed_down_ms = speed_down_ms;
    }

    /// Initialise waypoint and spline navigation.
    ///
    /// Upstream `AC_WPNav::wp_and_spline_init_m`. `speed_ms` of zero
    /// (or non-positive) means "use `WP_SPD` and watch it for changes".
    /// A zero `stopping_point_ned_m` is the current stopping point — the
    /// caller has already asked PosControl, the same way the C++ method
    /// does when the argument is the zero vector.
    pub fn wp_and_spline_init_m(
        &mut self,
        speed_ms: f32,
        stopping_point_ned_m: Vector3f,
        now_ms: u32,
        attitude: AttitudeJerkLimits,
    ) {
        // ensure waypoint radius is not below minimum allowed value
        self.wp_radius_m = self.wp_radius_m.max(WP_RADIUS_M_MIN);

        // ensure waypoint speed is not below minimum allowed value
        self.wp_speed_ms = self.wp_speed_ms.max(WP_SPD_MIN);

        // initialise position controller
        self.pos_control_stopping_point_inited = true;

        // determine desired waypoint speed; fallback to default if not provided
        self.check_wp_speed_change = !is_positive(speed_ms);
        self.wp_desired_speed_ne_ms = if is_positive(speed_ms) {
            speed_ms
        } else {
            self.default_speed_ne_ms()
        };
        self.wp_desired_speed_ne_ms = self.wp_desired_speed_ne_ms.max(WP_SPD_MIN);

        // initialise position controller speed and acceleration
        self.pos_speed_accel = PosControlSpeedAccel {
            ne_speed_ms: self.wp_desired_speed_ne_ms,
            ne_accel_mss: self.wp_acceleration_mss(),
            speed_down_ms: self.default_speed_down_ms(),
            speed_up_ms: self.default_speed_up_ms(),
            accel_d_mss: self.accel_d_mss(),
        };

        // calculate jerk limit if not explicitly set by parameter
        if !is_positive(self.wp_jerk_msss) {
            self.wp_jerk_msss = self.wp_acceleration_mss();
        }
        self.calc_scurve_jerk_and_snap(attitude);

        // SCurve::init on prev / this / next — objects stay in ap-math.
        self.scurve_legs_inited = true;
        self.track_dt_scalar = 1.0;

        self.flags.reached_destination = true;
        self.flags.fast_waypoint = false;

        self.origin_ned_m = stopping_point_ned_m;
        self.destination_ned_m = stopping_point_ned_m;
        self.is_terrain_alt = false;
        self.this_leg_is_spline = false;
        self.next_leg_is_spline = false;
        self.next_destination_ned_m = Vector3f::zero();
        self.scurve_this_leg_calculated = false;
        self.pos_terrain_d_m = 0.0;
        self.last_arc_rad = 0.0;
        self.spline_this_leg_set = false;
        self.spline_origin_vel_ned_ms = Vector3f::zero();
        self.spline_destination_vel_ned_ms = Vector3f::zero();
        self.spline_next_leg_set = false;
        self.spline_next_destination_ned_m = Vector3f::zero();
        self.spline_next_origin_vel_ned_ms = Vector3f::zero();
        self.spline_next_destination_vel_ned_ms = Vector3f::zero();
        self.need_this_leg_dest_speed_max = false;

        self.offset_vel_ms = self.wp_desired_speed_ne_ms;
        self.offset_accel_mss = 0.0;
        self.paused = false;

        self.wp_last_update_ms = now_ms;
    }

    /// Sets the current destination from a NEU centimetre vector.
    /// Upstream `AC_WPNav::set_wp_destination_NEU_cm`.
    pub fn set_wp_destination_neu_cm(
        &mut self,
        destination_neu_cm: Vector3f,
        is_terrain_alt: bool,
        ctx: SetWpDestinationContext,
    ) -> bool {
        let destination_ned_m = Vector3f::new(
            destination_neu_cm.x * 0.01,
            destination_neu_cm.y * 0.01,
            -destination_neu_cm.z * 0.01,
        );
        self.set_wp_destination_ned_m(destination_ned_m, is_terrain_alt, 0.0, ctx)
    }

    /// Sets the current destination from a NED metre vector.
    ///
    /// Upstream `AC_WPNav::set_wp_destination_NED_m`. Re-inits when the
    /// previous destination was interrupted (`!is_active` or not yet
    /// reached). Previous destination becomes the new origin. Terrain
    /// frame changes need `ctx.terrain_d_m`; missing terrain returns
    /// false. `SCurve::calculate_track` stays in `ap-math` (COP-002) —
    /// this slice records that a new this-leg is required.
    pub fn set_wp_destination_ned_m(
        &mut self,
        destination_ned_m: Vector3f,
        is_terrain_alt: bool,
        arc_rad: f32,
        ctx: SetWpDestinationContext,
    ) -> bool {
        // re-initialise if previous destination has been interrupted
        if !self.is_active(ctx.now_ms) || !self.flags.reached_destination {
            self.wp_and_spline_init_m(
                self.wp_desired_speed_ne_ms,
                ctx.stopping_point_ned_m,
                ctx.now_ms,
                ctx.attitude,
            );
        }

        // _scurve_prev_leg.init() — object stays in ap-math.

        // use previous destination as origin
        self.origin_ned_m = self.destination_ned_m;

        if is_terrain_alt == self.is_terrain_alt {
            // Matching frame: spline origin-speed seed and scurve-prev
            // copy need the stored SCurve / SplineCurve objects (COP-002
            // / COP-003). After init, this_leg_is_spline is false.
        } else {
            let Some(terrain_d_m) = ctx.terrain_d_m else {
                return false;
            };
            if is_terrain_alt {
                self.origin_ned_m.z -= terrain_d_m;
                self.pos_terrain_d_m = terrain_d_m;
            } else {
                self.origin_ned_m.z += terrain_d_m;
                self.pos_terrain_d_m = 0.0;
            }
        }

        self.destination_ned_m = destination_ned_m;
        self.is_terrain_alt = is_terrain_alt;

        // calculate_track / reuse of _scurve_next_leg is COP-002 plus
        // set_wp_destination_next. Record that a new this-leg is needed.
        self.scurve_this_leg_calculated = true;
        self.last_arc_rad = arc_rad;

        self.this_leg_is_spline = false;
        self.next_leg_is_spline = false;
        self.next_destination_ned_m = Vector3f::zero();
        self.spline_this_leg_set = false;
        self.spline_origin_vel_ned_ms = Vector3f::zero();
        self.spline_destination_vel_ned_ms = Vector3f::zero();
        self.spline_next_leg_set = false;
        self.spline_next_destination_ned_m = Vector3f::zero();
        self.spline_next_origin_vel_ned_ms = Vector3f::zero();
        self.spline_next_destination_vel_ned_ms = Vector3f::zero();
        self.need_this_leg_dest_speed_max = false;
        self.flags.fast_waypoint = false;
        self.flags.reached_destination = false;

        true
    }

    /// Sets the current spline waypoint from NED metre vectors.
    ///
    /// Upstream `AC_WPNav::set_spline_destination_NED_m`. Re-inits when
    /// the previous destination was interrupted. Previous destination
    /// becomes the new origin. Terrain frame changes need
    /// `ctx.terrain_d_m`; missing terrain returns false.
    /// `SplineCurve::set_speed_accel` / `set_origin_and_destination`
    /// stay in `ap-math` (COP-003) — this slice records the origin and
    /// destination velocity vectors that leftover would consume.
    pub fn set_spline_destination_ned_m(
        &mut self,
        destination_ned_m: Vector3f,
        is_terrain_alt: bool,
        next_destination_ned_m: Vector3f,
        next_is_terrain_alt: bool,
        next_is_spline: bool,
        ctx: SetWpDestinationContext,
    ) -> bool {
        // re-initialise path state if previous destination was not
        // completed or controller inactive
        if !self.is_active(ctx.now_ms) || !self.flags.reached_destination {
            self.wp_and_spline_init_m(
                self.wp_desired_speed_ne_ms,
                ctx.stopping_point_ned_m,
                ctx.now_ms,
                ctx.attitude,
            );
        }

        // `_spline_this_leg.set_speed_accel` — object stays in ap-math.

        // calculate origin and origin velocity vector
        let mut origin_vector_ned_m = Vector3f::zero();
        if is_terrain_alt == self.is_terrain_alt {
            if self.flags.fast_waypoint {
                if self.this_leg_is_spline {
                    // leftover of `_spline_this_leg.get_destination_vel`
                    origin_vector_ned_m = self.spline_destination_vel_ned_ms;
                } else {
                    origin_vector_ned_m = self.destination_ned_m - self.origin_ned_m;
                }
            }
            self.origin_ned_m = self.destination_ned_m;
        } else {
            self.origin_ned_m = self.destination_ned_m;
            let Some(terrain_d_m) = ctx.terrain_d_m else {
                return false;
            };
            if is_terrain_alt {
                self.origin_ned_m.z -= terrain_d_m;
                self.pos_terrain_d_m = terrain_d_m;
            } else {
                self.origin_ned_m.z += terrain_d_m;
                self.pos_terrain_d_m = 0.0;
            }
        }

        self.destination_ned_m = destination_ned_m;
        self.is_terrain_alt = is_terrain_alt;

        // calculate destination velocity vector
        let mut destination_vector_ned_m = Vector3f::zero();
        if is_terrain_alt == next_is_terrain_alt {
            if next_is_spline {
                destination_vector_ned_m = next_destination_ned_m - self.origin_ned_m;
            } else {
                destination_vector_ned_m = next_destination_ned_m - self.destination_ned_m;
            }
        }

        self.flags.fast_waypoint = !destination_vector_ned_m.is_zero();
        self.next_destination_ned_m = next_destination_ned_m;
        self.spline_origin_vel_ned_ms = origin_vector_ned_m;
        self.spline_destination_vel_ned_ms = destination_vector_ned_m;
        self.spline_this_leg_set = true;
        self.scurve_this_leg_calculated = false;
        self.last_arc_rad = 0.0;
        self.this_leg_is_spline = true;
        self.next_leg_is_spline = false;
        self.spline_next_leg_set = false;
        self.spline_next_destination_ned_m = Vector3f::zero();
        self.spline_next_origin_vel_ned_ms = Vector3f::zero();
        self.spline_next_destination_vel_ned_ms = Vector3f::zero();
        self.need_this_leg_dest_speed_max = false;
        self.flags.reached_destination = false;

        true
    }

    /// Sets the next spline segment from NED metre vectors.
    ///
    /// Upstream `AC_WPNav::set_spline_destination_next_NED_m`. Does not
    /// add the next point when the next dest terrain frame does not
    /// match the current leg (returns true, state unchanged).
    /// `SplineCurve::set_speed_accel` / `set_origin_and_destination`
    /// stay in `ap-math` (COP-003) — this slice records the leftover
    /// origin / destination velocity vectors for `_spline_next_leg`,
    /// marks `next_leg_is_spline`, and records that this-leg
    /// `set_destination_speed_max` must run.
    pub fn set_spline_destination_next_ned_m(
        &mut self,
        next_destination_ned_m: Vector3f,
        next_is_terrain_alt: bool,
        next_next_destination_ned_m: Vector3f,
        next_next_is_terrain_alt: bool,
        next_next_is_spline: bool,
    ) -> bool {
        // do not add next point if alt types don't match
        if next_is_terrain_alt != self.is_terrain_alt {
            return true;
        }

        // calculate origin and origin velocity vector
        let origin_vector_ned_m = if self.this_leg_is_spline {
            // leftover of `_spline_this_leg.get_destination_vel`
            self.spline_destination_vel_ned_ms
        } else {
            self.destination_ned_m - self.origin_ned_m
        };

        // calculate destination velocity vector
        let destination_vector_ned_m = if next_is_terrain_alt == next_next_is_terrain_alt {
            if next_next_is_spline {
                next_next_destination_ned_m - self.destination_ned_m
            } else {
                next_next_destination_ned_m - next_destination_ned_m
            }
        } else {
            Vector3f::zero()
        };

        // `_spline_next_leg.set_speed_accel` / `set_origin_and_destination`
        // stay in ap-math. Record leftover vectors that leftover would consume.
        self.spline_next_destination_ned_m = next_destination_ned_m;
        self.spline_next_origin_vel_ned_ms = origin_vector_ned_m;
        self.spline_next_destination_vel_ned_ms = destination_vector_ned_m;
        self.spline_next_leg_set = true;
        self.next_leg_is_spline = true;

        // next destination provided so fast waypoint
        self.flags.fast_waypoint = true;

        // leftover of this-leg `set_destination_speed_max` to match
        // `_spline_next_leg.get_origin_speed_max` (scurve or spline).
        self.need_this_leg_dest_speed_max = true;

        true
    }

    /// True if `wp_and_spline_init_m` or `update_wpnav` ran within 200 ms.
    /// Upstream `AC_WPNav::is_active`.
    #[must_use]
    pub fn is_active(&self, now_ms: u32) -> bool {
        now_ms.wrapping_sub(self.wp_last_update_ms) < WPNAV_ACTIVE_TIMEOUT_MS
    }

    /// Horizontal ground distance to the destination, metres.
    /// Upstream `AC_WPNav::get_wp_distance_to_destination_m`. Z is ignored.
    #[must_use]
    pub fn get_wp_distance_to_destination_m(&self, pos_estimate_ned_m: Vector3f) -> f32 {
        libm::hypotf(
            pos_estimate_ned_m.x - self.destination_ned_m.x,
            pos_estimate_ned_m.y - self.destination_ned_m.y,
        )
    }

    /// Horizontal ground distance to the destination, centimetres.
    /// Upstream `AC_WPNav::get_wp_distance_to_destination_cm`.
    #[must_use]
    pub fn get_wp_distance_to_destination_cm(&self, pos_estimate_ned_m: Vector3f) -> f32 {
        self.get_wp_distance_to_destination_m(pos_estimate_ned_m) * 100.0
    }

    /// Bearing to the destination, radians clockwise from North.
    /// Upstream `AC_WPNav::get_wp_bearing_to_destination_rad`.
    #[must_use]
    pub fn get_wp_bearing_to_destination_rad(&self, pos_estimate_ned_m: Vector3f) -> f32 {
        get_bearing_rad(pos_estimate_ned_m.xy(), self.destination_ned_m.xy())
    }

    /// Bearing to the destination, centidegrees clockwise from North.
    /// Upstream `AC_WPNav::get_wp_bearing_to_destination_cd`.
    #[must_use]
    pub fn get_wp_bearing_to_destination_cd(&self, pos_estimate_ned_m: Vector3f) -> i32 {
        get_bearing_cd(pos_estimate_ned_m.xy(), self.destination_ned_m.xy()) as i32
    }

    /// Sets the target horizontal speed during waypoint navigation.
    /// Upstream `AC_WPNav::set_speed_NE_ms`. Scales `_offset_vel_ms` so
    /// terrain-margin shaping keeps its current ratio, then records the
    /// PosControl NE limits. `update_track_with_speed_accel_limits` stays
    /// a leftover — the caller sees it via the update leftover flag.
    pub fn set_speed_ne_ms(&mut self, speed_ms: f32) -> bool {
        if speed_ms >= WP_SPD_MIN && is_positive(self.wp_desired_speed_ne_ms) {
            self.offset_vel_ms = speed_ms * self.offset_vel_ms / self.wp_desired_speed_ne_ms;
            self.wp_desired_speed_ne_ms = speed_ms;
            self.pos_speed_accel.ne_speed_ms = self.wp_desired_speed_ne_ms;
            self.pos_speed_accel.ne_accel_mss = self.wp_acceleration_mss();
            true
        } else {
            false
        }
    }

    /// Sets the climb speed. Upstream `AC_WPNav::set_speed_up_ms`.
    pub fn set_speed_up_ms(&mut self, speed_up_ms: f32) {
        self.pos_speed_accel.speed_up_ms = speed_up_ms;
    }

    /// Sets the descent speed. Upstream `AC_WPNav::set_speed_down_ms`.
    pub fn set_speed_down_ms(&mut self, speed_down_ms: f32) {
        self.pos_speed_accel.speed_down_ms = speed_down_ms;
    }

    /// Pause waypoint progression. Upstream `AC_WPNav::set_pause`.
    pub fn set_pause(&mut self) {
        self.paused = true;
    }

    /// Resume waypoint progression. Upstream `AC_WPNav::set_resume`.
    pub fn set_resume(&mut self) {
        self.paused = false;
    }

    /// Marks the current dest as a fast waypoint. Upstream this bit is
    /// written by `set_wp_destination_next_NED_m` when the next dest is
    /// preloaded; that setter is a later slice.
    pub fn set_fast_waypoint(&mut self, fast: bool) {
        self.flags.fast_waypoint = fast;
    }

    /// Runs one waypoint-navigation tick.
    ///
    /// Upstream `AC_WPNav::update_wpnav`. Watches `WP_SPD` / `WP_SPD_UP` /
    /// `WP_SPD_DN` for in-flight changes, then records the leftover of
    /// `advance_wp_target_along_track` and `NE_update_controller`. The
    /// last-update stamp is written even when advance would fail, matching
    /// the C++ order. The S-curve / spline advance itself stays later.
    pub fn update_wpnav(&mut self, ctx: UpdateWpNavContext) -> UpdateWpNavLeftover {
        let mut leftover = UpdateWpNavLeftover {
            applied_speed_ne: false,
            applied_speed_up: false,
            applied_speed_down: false,
            need_update_track_limits: false,
            need_advance_track: true,
            need_ne_update_controller: true,
            advance_ok: true,
            dt_s: ctx.dt_s,
        };

        // check for changes in WPNAV_SPEED parameter (horizontal speed target)
        if self.check_wp_speed_change && !is_equal(self.wp_speed_ms, self.last_wp_speed_ms) {
            leftover.applied_speed_ne = self.set_speed_ne_ms(self.default_speed_ne_ms());
            self.last_wp_speed_ms = self.wp_speed_ms;
            leftover.need_update_track_limits |= leftover.applied_speed_ne;
        }

        // check for climb and descent speed updates
        if !is_equal(self.wp_speed_up_ms, self.last_wp_speed_up_ms) {
            self.set_speed_up_ms(self.default_speed_up_ms());
            self.last_wp_speed_up_ms = self.wp_speed_up_ms;
            leftover.applied_speed_up = true;
            leftover.need_update_track_limits = true;
        }
        if !is_equal(self.wp_speed_down_ms, self.last_wp_speed_down_ms) {
            self.set_speed_down_ms(self.default_speed_down_ms());
            self.last_wp_speed_down_ms = self.wp_speed_down_ms;
            leftover.applied_speed_down = true;
            leftover.need_update_track_limits = true;
        }

        // advance_wp_target_along_track fails only when terrain-alt dest
        // has no terrain offset. The rest of that function is a later slice.
        leftover.advance_ok = if self.is_terrain_alt {
            ctx.terrain_d_m.is_some()
        } else {
            true
        };

        // run the horizontal position controller — leftover, always after
        // advance, even when advance fails.
        leftover.need_ne_update_controller = true;

        // record update time for is_active()
        self.wp_last_update_ms = ctx.now_ms;

        leftover
    }

    /// Advances the intermediate target along the current track.
    ///
    /// Upstream `AC_WPNav::advance_wp_target_along_track`. Terrain fail,
    /// track-time / offset-velocity shaping, and the reached-destination
    /// flag live here. `SCurve::advance_target_along_track` /
    /// `SplineCurve::advance_target_along_track` stay in `ap-math`; the
    /// caller supplies [`AdvanceWpTargetContext::path_finished`].
    /// `PosControl::set_pos_vel_accel_NED_m` is recorded as leftover.
    pub fn advance_wp_target_along_track(
        &mut self,
        ctx: AdvanceWpTargetContext,
    ) -> AdvanceWpTargetLeftover {
        let fail = AdvanceWpTargetLeftover {
            ok: false,
            need_set_pos_terrain_target: false,
            need_scurve_advance: false,
            need_spline_advance: false,
            need_set_pos_vel_accel: false,
            raw_track_dt_scalar: 1.0,
            vel_dt_scalar: 1.0,
            dt_along_track_s: 0.0,
        };

        // calculate terrain offset if using alt-above-terrain frame
        let terr_offset_d_m = if self.is_terrain_alt {
            let Some(terrain_d_m) = ctx.terrain_d_m else {
                return fail;
            };
            terrain_d_m
        } else {
            0.0
        };

        // calculate terrain-based velocity scaling factor
        let offset_d_scalar = ctx.terrain_scaler;

        // input shape the terrain offset — leftover `set_pos_terrain_target_D_m`.

        // compute current position in NED frame, adjusted to destination frame
        let mut curr_pos_ned_m = ctx.pos_estimate_ned_m - ctx.pos_offset_ned_m;
        curr_pos_ned_m.z -= terr_offset_d_m;

        // get desired velocity and remove offset
        let mut curr_target_vel_ned_ms = ctx.vel_desired_ned_ms;
        curr_target_vel_ned_ms.z -= ctx.vel_offset_d_ms;

        // scale progression time based on aircraft speed alignment with path
        let mut raw_track_dt_scalar = 1.0;
        if is_positive(curr_target_vel_ned_ms.length_squared()) {
            let track_direction = curr_target_vel_ned_ms.normalized_or_zero();
            let track_error_ned_m = ctx.pos_error_ned_m.dot(track_direction);
            let track_velocity_ned_ms = ctx.vel_estimate_ned_ms.dot(track_direction);
            raw_track_dt_scalar = constrain_value(
                0.05 + (track_velocity_ned_ms - ctx.pos_p_kp * track_error_ned_m)
                    / curr_target_vel_ned_ms.length(),
                0.0,
                1.0,
            );
        }

        // compute velocity scaling and apply jerk-limited velocity shaping
        let mut vel_dt_scalar = 1.0;
        if is_positive(self.wp_desired_speed_ne_ms) {
            update_vel_accel(
                &mut self.offset_vel_ms,
                self.offset_accel_mss,
                ctx.dt_s,
                0.0,
                0.0,
            );
            let vel_input_ms = if !self.paused {
                self.wp_desired_speed_ne_ms * offset_d_scalar
            } else {
                0.0
            };
            let accel_min = -self.wp_acceleration_mss();
            let accel_max = self.wp_acceleration_mss();
            let _ = shape_vel_accel(
                vel_input_ms,
                0.0,
                self.offset_vel_ms,
                &mut self.offset_accel_mss,
                accel_min,
                accel_max,
                ctx.shaping_jerk_ne_msss,
                ctx.dt_s,
                true,
            );
            vel_dt_scalar = self.offset_vel_ms / self.wp_desired_speed_ne_ms;
        }

        // apply exponential filter to track_dt_scalar using jerk-based tc
        let mut track_dt_scalar_tc = 1.0;
        if !is_zero(self.wp_jerk_msss) {
            track_dt_scalar_tc = self.wp_acceleration_mss() / self.wp_jerk_msss;
        }
        self.track_dt_scalar +=
            (raw_track_dt_scalar - self.track_dt_scalar) * (ctx.dt_s / track_dt_scalar_tc);

        let dt_along_track_s = self.track_dt_scalar * vel_dt_scalar * ctx.dt_s;

        // SCurve / SplineCurve advance_target_along_track — leftover.

        // check if waypoint has been reached based on mode and radius
        if !self.flags.reached_destination && ctx.path_finished {
            // "fast" waypoints are complete once the intermediate point
            // reaches the destination
            if self.flags.fast_waypoint {
                self.flags.reached_destination = true;
            } else {
                // regular waypoints also require the copter inside the radius
                let dist_to_dest_m = curr_pos_ned_m - self.destination_ned_m;
                if dist_to_dest_m.length_squared() <= self.wp_radius_m * self.wp_radius_m {
                    self.flags.reached_destination = true;
                }
            }
        }

        AdvanceWpTargetLeftover {
            ok: true,
            need_set_pos_terrain_target: true,
            need_scurve_advance: !self.this_leg_is_spline,
            need_spline_advance: self.this_leg_is_spline,
            need_set_pos_vel_accel: true,
            raw_track_dt_scalar,
            vel_dt_scalar,
            dt_along_track_s,
        }
    }

    /// Jerk and snap from attitude capability. Upstream
    /// `AC_WPNav::calc_scurve_jerk_and_snap`.
    fn calc_scurve_jerk_and_snap(&mut self, attitude: AttitudeJerkLimits) {
        self.scurve_jerk_max_msss = (attitude.ang_vel_roll_max_rads * GRAVITY_MSS)
            .min(attitude.ang_vel_pitch_max_rads * GRAVITY_MSS);
        if is_zero(self.scurve_jerk_max_msss) {
            self.scurve_jerk_max_msss = self.wp_jerk_msss;
        } else {
            self.scurve_jerk_max_msss = self.scurve_jerk_max_msss.min(self.wp_jerk_msss);
        }

        let tc = attitude.input_tc.max(0.1);
        self.scurve_snap_max_mssss =
            (self.scurve_jerk_max_msss * core::f32::consts::PI) / (2.0 * tc);

        let snap = attitude
            .accel_roll_max_radss
            .min(attitude.accel_pitch_max_radss)
            * GRAVITY_MSS;
        if is_positive(snap) {
            self.scurve_snap_max_mssss = self.scurve_snap_max_mssss.min(snap);
        }

        self.scurve_snap_max_mssss *= 0.5;
    }
}
