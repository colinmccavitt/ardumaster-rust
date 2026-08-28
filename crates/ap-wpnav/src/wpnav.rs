//! Constructor, `wp_and_spline_init_m`, and destination set/get, upstream `AC_WPNav`.

use ap_math::scalar::{is_positive, is_zero, GRAVITY_MSS};
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
        self.flags.fast_waypoint = false;
        self.flags.reached_destination = false;

        true
    }

    /// True if `wp_and_spline_init_m` or `update_wpnav` ran within 200 ms.
    /// Upstream `AC_WPNav::is_active`.
    #[must_use]
    pub fn is_active(&self, now_ms: u32) -> bool {
        now_ms.wrapping_sub(self.wp_last_update_ms) < WPNAV_ACTIVE_TIMEOUT_MS
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
