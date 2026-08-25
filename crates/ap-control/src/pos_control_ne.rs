//! The horizontal half of the position controller, upstream `AC_PosControl`'s
//! `NE_*` and `input_*_NE` family.
//!
//! Everything here works in the NE plane of the NED frame, in metres. The
//! vertical axis is a separate controller with its own limits, because a
//! multirotor's authority in the two is not remotely the same.

use ap_math::control::{
    shape_accel_xy, shape_pos_vel_accel_xy, shape_vel_accel_xy, stopping_distance,
    update_pos_vel_accel_xy, update_vel_accel_xy, Postype,
};
use ap_math::scalar::is_positive;
use ap_math::vector2::{Vector2, Vector2f};

/// Default horizontal jerk, upstream `POSCONTROL_JERK_NE_MSSS`.
pub const JERK_NE_MSSS: f32 = 5.0;

/// Decay time constant for the relax paths, upstream `POSCONTROL_RELAX_TC`.
///
/// Chosen so an I term decays to five percent in half a second — fast enough
/// that a pilot does not feel the controller still fighting, slow enough not
/// to be a step.
pub const RELAX_TC: f32 = 0.16;

/// Gravity, upstream `GRAVITY_MSS`.
const GRAVITY_MSS: f32 = 9.80665;

/// What the attitude controller can deliver, which bounds what the position
/// controller may ask for.
#[derive(Debug, Clone, Copy)]
pub struct AttitudeCapability {
    /// Maximum roll rate, radians per second.
    pub ang_vel_roll_max_rads: f32,
    /// Maximum pitch rate, radians per second.
    pub ang_vel_pitch_max_rads: f32,
    /// Maximum roll acceleration, radians per second squared.
    pub accel_roll_max_radss: f32,
    /// Maximum pitch acceleration, radians per second squared.
    pub accel_pitch_max_radss: f32,
    /// Whether body-frame feedforward is enabled, upstream
    /// `get_bf_feedforward`.
    ///
    /// With it off the attitude controller does not track a rate command, so
    /// the derived jerk limits below would be describing a capability the
    /// aircraft is not being asked to use.
    pub bf_feedforward: bool,
}

/// The horizontal limits, derived once and then held.
#[derive(Debug, Clone, Copy)]
pub struct NeLimits {
    /// Maximum horizontal speed, metres per second.
    pub vel_max_ne_ms: f32,
    /// Maximum horizontal acceleration, metres per second squared.
    pub accel_max_ne_mss: f32,
    /// Maximum horizontal jerk, metres per second cubed.
    pub jerk_max_ne_msss: f32,
}

impl NeLimits {
    /// Derive the horizontal limits, upstream `NE_set_max_speed_accel_m`.
    ///
    /// The speed and acceleration are taken as given — absolute values, so a
    /// caller passing a negative limit gets the magnitude rather than an
    /// inverted one.
    ///
    /// The jerk is where the attitude controller gets a say, and it is worth
    /// following. A multirotor accelerates horizontally by leaning, so
    /// changing its horizontal acceleration means changing its lean angle.
    /// The rate at which it can do that is an *angular rate*, which is why the
    /// jerk limit is bounded by the attitude controller's rate limit times
    /// gravity: an angular rate of one radian per second corresponds to about
    /// `g` metres per second cubed of horizontal jerk near level flight.
    ///
    /// The second bound comes from angular *acceleration*. The vehicle cannot
    /// reach its maximum lean rate instantly, so over a manoeuvre of a given
    /// size the achievable average jerk is lower than the peak. The half in
    /// `0.5 * sqrt(accel_max * snap_max)` is that averaging.
    ///
    /// Both bounds apply only when body-frame feedforward is on. Without it
    /// the attitude controller is not tracking a rate command at all, so its
    /// rate limits describe a capability that is not in use.
    #[must_use]
    pub fn derive(
        speed_ne_ms: f32,
        accel_ne_mss: f32,
        shaping_jerk_ne_msss: f32,
        attitude: &AttitudeCapability,
    ) -> Self {
        let vel_max_ne_ms = speed_ne_ms.abs();
        let accel_max_ne_mss = accel_ne_mss.abs();

        let jerk_max_msss = attitude
            .ang_vel_roll_max_rads
            .min(attitude.ang_vel_pitch_max_rads)
            * GRAVITY_MSS;
        let snap_max_mssss = attitude
            .accel_roll_max_radss
            .min(attitude.accel_pitch_max_radss)
            * GRAVITY_MSS;

        let mut jerk_max_ne_msss = shaping_jerk_ne_msss;

        if is_positive(jerk_max_msss) && attitude.bf_feedforward {
            jerk_max_ne_msss = jerk_max_ne_msss.min(jerk_max_msss);
        }

        if is_positive(snap_max_mssss) && attitude.bf_feedforward {
            jerk_max_ne_msss =
                (0.5 * libm::sqrtf(accel_max_ne_mss * snap_max_mssss)).min(jerk_max_ne_msss);
        }

        Self {
            vel_max_ne_ms,
            accel_max_ne_mss,
            jerk_max_ne_msss,
        }
    }
}

/// The horizontal controller's kinematic state.
///
/// `desired` is the trajectory a caller is flying; `target` is that plus any
/// offset. The two are kept apart so an offset can move — for a moving
/// landing pad, say — without the trajectory having to be recomputed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PosControlNe {
    /// Desired position, upstream `_pos_desired_ned_m.xy()`.
    pub pos_desired_m: Vector2<Postype>,
    /// Desired velocity, upstream `_vel_desired_ned_ms.xy()`.
    pub vel_desired_ms: Vector2f,
    /// Desired acceleration, upstream `_accel_desired_ned_mss.xy()`.
    pub accel_desired_mss: Vector2f,
    /// Directional limit from the last controller run, upstream
    /// `_limit_vector_ned.xy()`.
    pub limit_vector: Vector2f,
}

impl Default for PosControlNe {
    fn default() -> Self {
        Self::new()
    }
}

impl PosControlNe {
    /// Everything at zero.
    #[must_use]
    pub fn new() -> Self {
        Self {
            pos_desired_m: Vector2::new(0.0, 0.0),
            vel_desired_ms: Vector2f::new(0.0, 0.0),
            accel_desired_mss: Vector2f::new(0.0, 0.0),
            limit_vector: Vector2f::new(0.0, 0.0),
        }
    }

    /// Advance the desired state by one step, upstream's `update_pos_vel_accel_xy`
    /// call at the head of every `input_*_NE` function.
    ///
    /// Every entry point begins with this, which is what makes them
    /// composable: whatever a caller commanded last cycle has already been
    /// integrated before the new command is shaped against it.
    fn advance(&mut self, dt: f32, pos_error: Vector2f, vel_error: Vector2f) {
        update_pos_vel_accel_xy(
            &mut self.pos_desired_m,
            &mut self.vel_desired_ms,
            self.accel_desired_mss,
            dt,
            self.limit_vector,
            pos_error,
            vel_error,
        );
    }

    /// Command a horizontal acceleration, upstream `input_accel_NE_m`.
    ///
    /// The bluntest entry point: no velocity or position target at all, just a
    /// jerk-limited acceleration. Used where something else owns the
    /// trajectory and the position controller is only smoothing.
    pub fn input_accel(
        &mut self,
        accel_ne_mss: Vector2f,
        limits: &NeLimits,
        dt: f32,
        pos_error: Vector2f,
        vel_error: Vector2f,
    ) {
        self.advance(dt, pos_error, vel_error);
        shape_accel_xy(
            accel_ne_mss,
            &mut self.accel_desired_mss,
            limits.jerk_max_ne_msss,
            dt,
        );
    }

    /// Command a horizontal velocity, upstream `input_vel_accel_NE_m`.
    ///
    /// The velocity argument is borrowed mutably because upstream writes back
    /// the value it actually achieved: shaping may not reach the request, and
    /// a caller integrating its own trajectory needs to know what happened
    /// rather than what it asked for.
    #[expect(
        clippy::too_many_arguments,
        reason = "upstream's signature plus the errors it reads from members this port does not own; a struct would make the call sites disagree with the code they are checked against"
    )]
    pub fn input_vel_accel(
        &mut self,
        vel_ne_ms: &mut Vector2f,
        accel_ne_mss: Vector2f,
        limits: &NeLimits,
        dt: f32,
        limit_output: bool,
        pos_error: Vector2f,
        vel_error: Vector2f,
    ) {
        self.advance(dt, pos_error, vel_error);

        shape_vel_accel_xy(
            *vel_ne_ms,
            accel_ne_mss,
            self.vel_desired_ms,
            &mut self.accel_desired_mss,
            limits.accel_max_ne_mss,
            limits.jerk_max_ne_msss,
            dt,
            limit_output,
        );

        // The caller's velocity is advanced too, with no limit vector: it is
        // the caller's own trajectory, not the controller's, and suppressing
        // it here would silently rewrite what the caller believes it asked
        // for. Velocity only -- this entry point has no position to advance.
        update_vel_accel_xy(
            vel_ne_ms,
            accel_ne_mss,
            dt,
            Vector2f::new(0.0, 0.0),
            Vector2f::new(0.0, 0.0),
        );
    }

    /// Command a horizontal position, upstream `input_pos_vel_accel_NE_m`.
    ///
    /// The full outer loop. Position and velocity are both borrowed mutably
    /// for the same reason as above.
    #[expect(
        clippy::too_many_arguments,
        reason = "upstream's signature; the alternative is a struct the call \
sites do not have"
    )]
    pub fn input_pos_vel_accel(
        &mut self,
        pos_ne_m: &mut Vector2<Postype>,
        vel_ne_ms: &mut Vector2f,
        accel_ne_mss: Vector2f,
        limits: &NeLimits,
        dt: f32,
        limit_output: bool,
        pos_error: Vector2f,
        vel_error: Vector2f,
    ) {
        self.advance(dt, pos_error, vel_error);

        shape_pos_vel_accel_xy(
            *pos_ne_m,
            *vel_ne_ms,
            accel_ne_mss,
            self.pos_desired_m,
            self.vel_desired_ms,
            &mut self.accel_desired_mss,
            limits.vel_max_ne_ms,
            limits.accel_max_ne_mss,
            limits.jerk_max_ne_msss,
            dt,
            limit_output,
        );

        update_pos_vel_accel_xy(
            pos_ne_m,
            vel_ne_ms,
            accel_ne_mss,
            dt,
            Vector2f::new(0.0, 0.0),
            Vector2f::new(0.0, 0.0),
            Vector2f::new(0.0, 0.0),
        );
    }

    /// Decay the acceleration toward zero, upstream
    /// `NE_relax_velocity_controller`.
    ///
    /// Roughly ninety-five percent gone in half a second. The point is that a
    /// controller handing over does not drop its lean instantly — the attitude
    /// target follows this acceleration, so zeroing it in one step would be a
    /// step input to the attitude loop.
    pub fn relax_velocity(&mut self, accel_target: &mut Vector2f, dt: f32) {
        if is_positive(dt) {
            let decay = 1.0 - dt / (dt + RELAX_TC);
            *accel_target *= decay;
        }
    }
}

/// Where the vehicle would come to rest, upstream `get_stopping_point_NE_m`.
///
/// Offsets are removed from both position and velocity first, so the answer is
/// in the trajectory's own frame rather than the target's. A vehicle tracking
/// a moving pad should stop relative to where it is going, not relative to the
/// pad.
///
/// The speed is clamped to the configured maximum before the distance is
/// computed. That is deliberate and not merely defensive: a vehicle that is
/// somehow travelling faster than its own limit would otherwise be told to
/// plan a stop it has no authority to make, and would arrive somewhere short
/// of the point it had committed to.
#[must_use]
pub fn stopping_point_ne(
    pos_estimate_m: Vector2<Postype>,
    pos_offset_m: Vector2<Postype>,
    vel_estimate_ms: Vector2f,
    vel_offset_ms: Vector2f,
    kp: f32,
    limits: &NeLimits,
) -> Vector2<Postype> {
    let mut stopping_point = Vector2::new(
        pos_estimate_m.x - pos_offset_m.x,
        pos_estimate_m.y - pos_offset_m.y,
    );

    let vel = vel_estimate_ms - vel_offset_ms;
    let speed_ms = vel.length();
    if !is_positive(speed_ms) {
        return stopping_point;
    }

    let stopping_dist_m = stopping_distance(
        speed_ms.clamp(0.0, limits.vel_max_ne_ms),
        kp,
        limits.accel_max_ne_mss,
    );
    if !is_positive(stopping_dist_m) {
        return stopping_point;
    }

    let stopping_time_s = stopping_dist_m / speed_ms;
    stopping_point.x += Postype::from(vel.x * stopping_time_s);
    stopping_point.y += Postype::from(vel.y * stopping_time_s);
    stopping_point
}
