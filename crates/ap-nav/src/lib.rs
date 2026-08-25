//! Port of `AP_L1_Control`, the fixed-wing lateral navigation law. FW-016.
//!
//! L1 answers one question: given where the aircraft is, where it is going and
//! how fast, what lateral acceleration would put it on the track? Everything
//! downstream — the bank angle, the roll rate demand, the aileron — follows
//! from that number.
//!
//! # Four values from the AHRS, passed in
//!
//! Upstream holds an `AP_AHRS &` and reads yaw, the groundspeed vector, the
//! current location and pitch from it. ADR-0004 rules out that reference, so
//! they arrive in [`NavInputs`]. That is also what lets this be replay-verified
//! without porting `AP_AHRS` (8,563 lines) or the EKF behind it: all four are
//! recorded in an ordinary flight log.
//!
//! # State that persists between calls
//!
//! The crosstrack integrator, the previous `Nu` used to break turning
//! indecision, and the loiter-capture latch all carry across calls, so this is
//! a controller rather than a function. The replay has to run it in order.

#![no_std]
#![allow(
    clippy::approx_constant,
    reason = "upstream writes these as truncated decimals -- 0.3183099 for 1/pi, 0.7071 for the sine limit, 1.5708 for pi/2, 6.2832 for tau, 4.4428 for sqrt(2)*pi -- and substituting Rust's exact constants would change the arithmetic. Reproducing the literal upstream flies with is the point."
)]

use ap_math::location::Location;
use ap_math::scalar::{
    cd_to_rad, constrain_value, degrees, is_equal, rad_to_cd, radians, wrap_180_cd, wrap_pi, Real,
    GRAVITY_MSS,
};
use ap_math::vector2::Vector2f;

/// L1 tuning, upstream's `NAVL1_*` parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct L1Gains {
    /// Tracking loop period, seconds. Upstream `NAVL1_PERIOD`.
    pub period: f32,
    /// Tracking loop damping ratio. Upstream `NAVL1_DAMPING`.
    pub damping: f32,
    /// Gain on the crosstrack integrator. Zero disables it, and any change
    /// resets it — upstream does that deliberately so retuning re-converges.
    /// Upstream `NAVL1_XTRACK_I`.
    pub xtrack_i_gain: f32,
    /// Bank angle a loiter may not exceed, degrees. Upstream `NAVL1_LIM_BANK`.
    pub loiter_bank_limit: f32,
}

impl Default for L1Gains {
    /// Upstream's parameter defaults.
    fn default() -> Self {
        Self {
            period: 17.0,
            damping: 0.75,
            xtrack_i_gain: 0.02,
            loiter_bank_limit: 0.0,
        }
    }
}

/// What L1 reads from the vehicle each call.
#[derive(Debug, Clone, Copy)]
pub struct NavInputs {
    /// Microseconds since boot. Upstream reads `AP_HAL::micros()`, a `u32`
    /// that wraps roughly every 71 minutes; the subtraction is done in `u32`
    /// so the wrap cancels, and that is reproduced here.
    pub now_us: u32,
    /// Milliseconds since boot, for the loiter-capture latch.
    pub now_ms: u32,
    /// Current position, or `None` when the AHRS has no fix. Upstream's
    /// `get_location` returning false, on which it keeps the previous demand
    /// rather than steering on a guess.
    pub location: Option<Location>,
    /// Ground velocity, north and east, m/s.
    pub groundspeed_vector: Vector2f,
    /// Yaw, radians. Upstream `ahrs.get_yaw_rad()`.
    pub yaw_rad: f32,
    /// Yaw, centidegrees. Upstream `ahrs.yaw_sensor`.
    pub yaw_sensor_cd: i32,
    /// Pitch, radians. Upstream `ahrs.get_pitch_rad()`.
    pub pitch_rad: f32,
    /// Equivalent-to-true airspeed ratio.
    pub eas2tas: f32,
}

/// Remembers the loiter the aircraft last captured, upstream's `_last_loiter`.
#[derive(Debug, Clone, Copy)]
struct LastLoiter {
    reached_ms: u32,
    radius: f32,
    direction: i8,
    center: Location,
}

/// Fixed-wing lateral navigation, upstream `AP_L1_Control`.
#[derive(Debug, Clone, Copy)]
pub struct L1Control {
    /// Tuning. Public because upstream exposes the parameters and they are
    /// changed in flight.
    pub gains: L1Gains,

    lat_acc_dem: f32,
    l1_dist: f32,
    wp_circle: bool,
    nav_bearing: f32,
    bearing_error: f32,
    crosstrack_error: f32,
    target_bearing_cd: i32,
    last_nu: f32,
    xtrack_i: f32,
    xtrack_i_gain_prev: f32,
    last_update_waypoint_us: u32,
    data_is_stale: bool,
    last_loiter: LastLoiter,
    reverse: bool,
}

/// Beyond this the aircraft counts as pointed away from the target, upstream's
/// `0.9 * PI` band in `_prevent_indecision`.
const INDECISION_NU_LIMIT: f32 = 0.9 * core::f32::consts::PI;

/// Pitch beyond which the bank-angle conversion clamps, upstream's
/// `pitchLimL1 = radians(60)`.
const PITCH_LIMIT_DEG: f32 = 60.0;

impl L1Control {
    /// A controller with the given tuning.
    #[must_use]
    pub fn new(gains: L1Gains) -> Self {
        Self {
            gains,
            lat_acc_dem: 0.0,
            l1_dist: 0.0,
            wp_circle: false,
            nav_bearing: 0.0,
            bearing_error: 0.0,
            crosstrack_error: 0.0,
            target_bearing_cd: 0,
            last_nu: 0.0,
            xtrack_i: 0.0,
            xtrack_i_gain_prev: 0.0,
            last_update_waypoint_us: 0,
            data_is_stale: true,
            last_loiter: LastLoiter {
                reached_ms: 0,
                radius: 0.0,
                direction: 0,
                center: Location::new(0, 0),
            },
            reverse: false,
        }
    }

    /// Fly the track backwards, upstream `set_reverse`.
    pub fn set_reverse(&mut self, reverse: bool) {
        self.reverse = reverse;
    }

    /// Yaw as the navigation law sees it, upstream `get_yaw`.
    fn yaw(&self, inp: &NavInputs) -> f32 {
        if self.reverse {
            wrap_pi(core::f32::consts::PI + inp.yaw_rad)
        } else {
            inp.yaw_rad
        }
    }

    /// Yaw in centidegrees as the navigation law sees it, upstream
    /// `get_yaw_sensor`.
    fn yaw_sensor(&self, inp: &NavInputs) -> i32 {
        if self.reverse {
            wrap_180_cd(18000 + inp.yaw_sensor_cd)
        } else {
            inp.yaw_sensor_cd
        }
    }

    /// The lateral acceleration the last update asked for, m/s2, positive to
    /// the right. Upstream `lateral_acceleration`.
    #[must_use]
    pub const fn lateral_acceleration(&self) -> f32 {
        self.lat_acc_dem
    }

    /// Bank angle that would produce the demanded lateral acceleration,
    /// centidegrees. Upstream `nav_roll_cd`.
    ///
    /// From the balanced-spiral relation: lift resolves against gravity times
    /// the cosine of pitch, so a nose-high aircraft needs more bank for the
    /// same turn. Upstream clamps the pitch used here to +-60 degrees rather
    /// than to the configured pitch limits, and notes the choice could be
    /// revisited.
    #[must_use]
    pub fn nav_roll_cd(&self, inp: &NavInputs) -> f32 {
        let pitch_lim = radians(PITCH_LIMIT_DEG);
        let pitch = constrain_value(inp.pitch_rad, -pitch_lim, pitch_lim);
        let ret = degrees(Real::atan(
            self.lat_acc_dem * (1.0 / (GRAVITY_MSS * Real::cos(pitch))),
        )) * 100.0;
        constrain_value(ret, -9000.0, 9000.0)
    }

    /// Bearing to the L1 reference point, centidegrees. Upstream
    /// `nav_bearing_cd`.
    #[must_use]
    pub fn nav_bearing_cd(&self) -> i32 {
        wrap_180_cd(rad_to_cd(self.nav_bearing) as i32)
    }

    /// Angle between demanded and achieved velocity, centidegrees, positive to
    /// the left of track. Upstream `bearing_error_cd`.
    #[must_use]
    pub fn bearing_error_cd(&self) -> i32 {
        rad_to_cd(self.bearing_error) as i32
    }

    /// Bearing to the target, centidegrees. Upstream `target_bearing_cd`.
    #[must_use]
    pub fn target_bearing_cd(&self) -> i32 {
        wrap_180_cd(self.target_bearing_cd)
    }

    /// Distance from the loiter circle, metres. Upstream `crosstrack_error`.
    #[must_use]
    pub const fn crosstrack_error(&self) -> f32 {
        self.crosstrack_error
    }

    /// Whether the aircraft has started circling. Upstream
    /// `reached_loiter_target`.
    #[must_use]
    pub const fn reached_loiter_target(&self) -> bool {
        self.wp_circle
    }

    /// Whether the last call had no position and kept the previous demand.
    #[must_use]
    pub const fn data_is_stale(&self) -> bool {
        self.data_is_stale
    }

    /// Distance before a waypoint at which to start a 90 degree turn, metres.
    /// Upstream `turn_distance`.
    #[must_use]
    pub fn turn_distance(&self, wp_radius: f32, eas2tas: f32) -> f32 {
        let scaled = wp_radius * (eas2tas * eas2tas);
        scaled.min(self.l1_dist)
    }

    /// The same, reduced linearly for turns shallower than 90 degrees.
    ///
    /// This is what stops a straight-ahead mission leg deciding it has arrived
    /// early, which matters for anything triggered at an exact position.
    #[must_use]
    pub fn turn_distance_for_angle(&self, wp_radius: f32, turn_angle: f32, eas2tas: f32) -> f32 {
        let distance_90 = self.turn_distance(wp_radius, eas2tas);
        let turn_angle = Real::abs(turn_angle);
        if turn_angle >= 90.0 {
            distance_90
        } else {
            distance_90 * turn_angle / 90.0
        }
    }

    /// Loiter radius after altitude and bank-limit scaling, upstream
    /// `loiter_radius`.
    ///
    /// `target_airspeed` is the speed TECS is aiming for; upstream reads it
    /// through a pointer that may be null, in which case the bank limit cannot
    /// be honoured and only the altitude scaling applies.
    #[must_use]
    pub fn loiter_radius(&self, radius: f32, target_airspeed: Option<f32>, eas2tas: f32) -> f32 {
        let bank_limit = constrain_value(self.gains.loiter_bank_limit, 0.0, 89.0);
        let lateral_accel_sea_level = Real::tan(radians(bank_limit)) * GRAVITY_MSS;
        let nominal_velocity_sea_level = target_airspeed.unwrap_or(0.0);
        let eas2tas_sq = eas2tas * eas2tas;

        if ap_math::scalar::is_zero(bank_limit)
            || ap_math::scalar::is_zero(nominal_velocity_sea_level)
            || ap_math::scalar::is_zero(lateral_accel_sea_level)
        {
            // Nothing sane to compute a limit from, or the user asked for
            // plain altitude scaling. Still protects the airframe.
            return radius * eas2tas_sq;
        }
        let sea_level_radius =
            (nominal_velocity_sea_level * nominal_velocity_sea_level) / lateral_accel_sea_level;
        if sea_level_radius > radius {
            // the requested radius is unachievable at the bank limit
            radius * eas2tas_sq
        } else {
            (sea_level_radius * eas2tas_sq).max(radius)
        }
    }

    /// Keep turning the way we already were when the choice is a coin flip.
    ///
    /// Upstream `_prevent_indecision`. When the target is nearly dead astern,
    /// left and right are almost equally good, and noise can flip the decision
    /// every loop — so if the sign of `Nu` has changed while both the old and
    /// new values are in the narrow band near +-180 degrees, and the aircraft
    /// is genuinely pointed away from the target, the previous decision wins.
    fn prevent_indecision(&self, nu: &mut f32, inp: &NavInputs) {
        if Real::abs(*nu) > INDECISION_NU_LIMIT
            && Real::abs(self.last_nu) > INDECISION_NU_LIMIT
            && (wrap_180_cd(self.target_bearing_cd - self.yaw_sensor(inp))).abs() > 12000
            && *nu * self.last_nu < 0.0
        {
            *nu = self.last_nu;
        }
    }

    /// Track from `prev_wp` to `next_wp`, upstream `update_waypoint`.
    ///
    /// `dist_min` is a floor on the L1 distance, which the caller raises near
    /// the ground so the aircraft does not chase a reference point it cannot
    /// reach.
    pub fn update_waypoint(
        &mut self,
        prev_wp: Location,
        next_wp: Location,
        dist_min: f32,
        inp: &NavInputs,
    ) {
        // The subtraction is in u32 so that the microsecond counter wrapping
        // every 71 minutes cancels rather than producing a huge dt.
        let dt = inp.now_us.wrapping_sub(self.last_update_waypoint_us) as f32 * 1.0e-6;
        if dt > 1.0 {
            // Not called for a long time: the integrator's history is
            // meaningless, so drop it.
            self.xtrack_i = 0.0;
        }
        let dt = if dt > 0.1 { 0.1 } else { dt };
        self.last_update_waypoint_us = inp.now_us;

        let k_l1 = 4.0 * self.gains.damping * self.gains.damping;

        let Some(current_loc) = inp.location else {
            // No fix: keep the previous demand rather than steering on a guess.
            self.data_is_stale = true;
            return;
        };

        let mut groundspeed_vector = inp.groundspeed_vector;
        self.target_bearing_cd = current_loc.get_bearing_to(next_wp);
        let mut ground_speed = groundspeed_vector.length();

        // Flying backwards, or barely moving, means the velocity vector says
        // nothing useful about where the nose is pointed -- so substitute a
        // small vector along the heading and let the compass carry it.
        let moving_forwards = Real::abs(wrap_pi(groundspeed_vector.angle() - self.yaw(inp)))
            < core::f32::consts::FRAC_PI_2;
        if ground_speed < 0.1 || !moving_forwards {
            ground_speed = 0.1;
            let yaw = self.yaw(inp);
            groundspeed_vector = Vector2f::new(Real::cos(yaw), Real::sin(yaw)) * ground_speed;
        }

        // 0.3183099 is 1/pi, written as a literal upstream.
        self.l1_dist =
            (0.318_309_9 * self.gains.damping * self.gains.period * ground_speed).max(dist_min);

        let mut ab = prev_wp.get_distance_ne(next_wp);
        let ab_length = ab.length();
        if ab.length() < 1.0e-6 {
            // The two waypoints coincide; steer straight at the destination,
            // and if we are already on it, hold the current heading.
            ab = current_loc.get_distance_ne(next_wp);
            if ab.length() < 1.0e-6 {
                let yaw = self.yaw(inp);
                ab = Vector2f::new(Real::cos(yaw), Real::sin(yaw));
            }
        }
        ab.normalize();

        let a_air = prev_wp.get_distance_ne(current_loc);
        self.crosstrack_error = a_air.cross(ab);

        let wp_a_dist = a_air.length();
        let along_track_dist = a_air.dot(ab);

        let nu;
        if wp_a_dist > self.l1_dist && along_track_dist / wp_a_dist.max(1.0) < -0.707_1 {
            // Behind a 135 degree arc centred on A and further than the L1
            // distance: steer at A rather than at the track.
            let unit = a_air.normalized_or_zero();
            let xtrack_vel = groundspeed_vector.cross(-unit);
            let ltrack_vel = groundspeed_vector.dot(-unit);
            nu = Real::atan2(xtrack_vel, ltrack_vel);
            self.nav_bearing = Real::atan2(-unit.y, -unit.x);
        } else if along_track_dist > ab_length + ground_speed * 3.0 {
            // Three seconds past B: turn back towards it.
            let b_air = next_wp.get_distance_ne(current_loc);
            let unit = b_air.normalized_or_zero();
            let xtrack_vel = groundspeed_vector.cross(-unit);
            let ltrack_vel = groundspeed_vector.dot(-unit);
            nu = Real::atan2(xtrack_vel, ltrack_vel);
            self.nav_bearing = Real::atan2(-unit.y, -unit.x);
        } else {
            // Normal case: track the AB line.
            let xtrack_vel = groundspeed_vector.cross(ab);
            let ltrack_vel = groundspeed_vector.dot(ab);
            let nu2 = Real::atan2(xtrack_vel, ltrack_vel);

            // The angle to the L1 reference point. Limiting its sine to
            // 0.7071 caps the track capture angle at 45 degrees, so a large
            // crosstrack error does not produce a perpendicular intercept.
            let sine_nu1 = constrain_value(
                self.crosstrack_error / self.l1_dist.max(0.1),
                -0.707_1,
                0.707_1,
            );
            let mut nu1 = Real::asin(sine_nu1);

            // The integrator exists to drive a steady crosstrack offset to
            // zero -- the kind a trim error leaves. Upstream clears it when
            // the gain is disabled OR changed, so that retuning re-converges
            // from scratch rather than from whatever the old gain accumulated.
            if self.gains.xtrack_i_gain <= 0.0
                || !is_equal(self.gains.xtrack_i_gain, self.xtrack_i_gain_prev)
            {
                self.xtrack_i = 0.0;
                self.xtrack_i_gain_prev = self.gains.xtrack_i_gain;
            } else if Real::abs(nu1) < radians(5.0) {
                self.xtrack_i += nu1 * self.gains.xtrack_i_gain * dt;
                // An AHRS_TRIM_X of 0.1 drifts to about 0.08, so 0.1 is a
                // reasonable worst case to clip at.
                self.xtrack_i = constrain_value(self.xtrack_i, -0.1, 0.1);
            }
            nu1 += self.xtrack_i;

            nu = nu1 + nu2;
            self.nav_bearing = wrap_pi(Real::atan2(ab.y, ab.x) + nu1);
        }

        let mut nu = nu;
        self.prevent_indecision(&mut nu, inp);
        self.last_nu = nu;

        nu = constrain_value(nu, -1.5708, 1.5708);
        self.lat_acc_dem = k_l1 * ground_speed * ground_speed / self.l1_dist * Real::sin(nu);

        self.wp_circle = false;
        self.last_loiter.reached_ms = 0;
        self.bearing_error = nu;
        self.data_is_stale = false;
    }

    /// Circle a point, upstream `update_loiter`.
    ///
    /// `loiter_direction` is +1 clockwise, -1 anticlockwise.
    /// `target_airspeed` is what TECS is aiming for, used only by the bank
    /// limit in [`Self::loiter_radius`].
    ///
    /// Two laws run at once and the controller flies whichever asks for less
    /// turn: the L1 capture law that steers at the centre, and a PD loop on
    /// radial error plus the centripetal acceleration the circle needs. The
    /// crossover is where their demands meet, which makes the transition from
    /// approaching to circling seamless rather than a mode switch.
    pub fn update_loiter(
        &mut self,
        center_wp: Location,
        radius: f32,
        loiter_direction: i8,
        target_airspeed: Option<f32>,
        inp: &NavInputs,
    ) {
        let radius_unscaled = radius;
        let radius = self.loiter_radius(Real::abs(radius), target_airspeed, inp.eas2tas);

        // Guidance gains for the circle-tracking PD loop.
        let omega = 6.2832 / self.gains.period;
        let kx = omega * omega;
        let kv = 2.0 * self.gains.damping * omega;
        // and the L1 gain for the capture law
        let k_l1 = 4.0 * self.gains.damping * self.gains.damping;

        let Some(current_loc) = inp.location else {
            self.data_is_stale = true;
            return;
        };

        let groundspeed_vector = inp.groundspeed_vector;
        let ground_speed = groundspeed_vector.length().max(1.0);

        self.target_bearing_cd = current_loc.get_bearing_to(center_wp);
        // 0.3183099 is 1/pi, written as a literal upstream. Note there is no
        // dist_min floor here, unlike waypoint tracking.
        self.l1_dist = 0.318_309_9 * self.gains.damping * self.gains.period * ground_speed;

        let a_air = center_wp.get_distance_ne(current_loc);

        // Sitting on the centre with no velocity leaves nothing to point at,
        // so fall back through velocity to heading.
        let a_air_unit = if a_air.length() > 0.1 {
            a_air.normalized_or_zero()
        } else if groundspeed_vector.length() < 0.1 {
            let yaw = inp.yaw_rad;
            Vector2f::new(Real::cos(yaw), Real::sin(yaw))
        } else {
            groundspeed_vector.normalized_or_zero()
        };

        // Capture: the same L1 law as waypoint tracking, aimed at the centre.
        let xtrack_vel_cap = a_air_unit.cross(groundspeed_vector);
        let ltrack_vel_cap = -(groundspeed_vector.dot(a_air_unit));
        let mut nu = Real::atan2(xtrack_vel_cap, ltrack_vel_cap);

        self.prevent_indecision(&mut nu, inp);
        self.last_nu = nu;
        nu = constrain_value(
            nu,
            -core::f32::consts::FRAC_PI_2,
            core::f32::consts::FRAC_PI_2,
        );
        let lat_acc_dem_cap = k_l1 * ground_speed * ground_speed / self.l1_dist * Real::sin(nu);

        // Circle: PD on radial error, plus the centripetal term.
        let xtrack_vel_circ = -ltrack_vel_cap;
        let xtrack_err_circ = a_air.length() - radius;
        self.crosstrack_error = xtrack_err_circ;

        let mut lat_acc_dem_circ_pd = xtrack_err_circ * kx + xtrack_vel_circ * kv;
        let vel_tangent = xtrack_vel_cap * f32::from(loiter_direction);
        // Flying the wrong way round, the PD term would turn us further the
        // wrong way; clamp it so it can only help.
        if ltrack_vel_cap < 0.0 && vel_tangent < 0.0 {
            lat_acc_dem_circ_pd = lat_acc_dem_circ_pd.max(0.0);
        }
        let lat_acc_dem_circ_ctr =
            vel_tangent * vel_tangent / (0.5 * radius).max(radius + xtrack_err_circ);
        let lat_acc_dem_circ =
            f32::from(loiter_direction) * (lat_acc_dem_circ_pd + lat_acc_dem_circ_ctr);

        let dir = f32::from(loiter_direction);
        if xtrack_err_circ > 0.0 && dir * lat_acc_dem_cap < dir * lat_acc_dem_circ {
            self.lat_acc_dem = lat_acc_dem_cap;

            // Having reached the circle once, a gust or an unachievable radius
            // should not make reached_loiter_target flicker back to false --
            // so the latch survives as long as the same loiter keeps being
            // commanded and no more than 200ms passes.
            if self.wp_circle
                && self.last_loiter.reached_ms != 0
                && inp.now_ms.wrapping_sub(self.last_loiter.reached_ms) < 200
                && loiter_direction == self.last_loiter.direction
                && is_equal(radius_unscaled, self.last_loiter.radius)
                && center_wp == self.last_loiter.center
            {
                self.last_loiter.reached_ms = inp.now_ms;
            } else {
                self.wp_circle = false;
                self.last_loiter.reached_ms = 0;
            }
            self.bearing_error = nu;
        } else {
            self.lat_acc_dem = lat_acc_dem_circ;
            self.wp_circle = true;
            self.last_loiter.reached_ms = inp.now_ms;
            self.bearing_error = 0.0;
        }
        self.nav_bearing = Real::atan2(-a_air_unit.y, -a_air_unit.x);

        self.last_loiter.radius = radius_unscaled;
        self.last_loiter.direction = loiter_direction;
        self.last_loiter.center = center_wp;
        self.data_is_stale = false;
    }

    /// Hold a heading, upstream `update_heading_hold`.
    pub fn update_heading_hold(&mut self, navigation_heading_cd: i32, inp: &NavInputs) {
        // sqrt(2)*pi/period, written as a literal upstream
        let omega_a = 4.4428 / self.gains.period;

        self.target_bearing_cd = wrap_180_cd(navigation_heading_cd);
        self.nav_bearing = cd_to_rad(navigation_heading_cd as f32);

        let nu_cd = wrap_180_cd(self.target_bearing_cd - wrap_180_cd(inp.yaw_sensor_cd));
        let mut nu = cd_to_rad(nu_cd as f32);

        let ground_speed = inp.groundspeed_vector.length();

        // The L1 distance moves with speed so the tracking loop keeps a
        // constant frequency.
        self.l1_dist = ground_speed / omega_a;
        let v_omega_a = ground_speed * omega_a;

        self.wp_circle = false;
        self.last_loiter.reached_ms = 0;
        self.crosstrack_error = 0.0;
        self.bearing_error = nu;

        nu = constrain_value(
            nu,
            -core::f32::consts::FRAC_PI_2,
            core::f32::consts::FRAC_PI_2,
        );
        self.lat_acc_dem = 2.0 * Real::sin(nu) * v_omega_a;
        self.data_is_stale = false;
    }

    /// Fly level on the current heading, upstream `update_level_flight`.
    pub fn update_level_flight(&mut self, inp: &NavInputs) {
        self.target_bearing_cd = inp.yaw_sensor_cd;
        self.nav_bearing = inp.yaw_rad;
        self.bearing_error = 0.0;
        self.crosstrack_error = 0.0;
        self.wp_circle = false;
        self.last_loiter.reached_ms = 0;
        self.lat_acc_dem = 0.0;
        self.data_is_stale = false;
    }
}
