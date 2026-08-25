//! The horizontal half of the position controller, upstream `AC_PosControl`'s
//! `NE_*` and `input_*_NE` family.
//!
//! Everything here works in the NE plane of the NED frame, in metres. The
//! vertical axis is a separate controller with its own limits, because a
//! multirotor's authority in the two is not remotely the same.

use ap_math::control::{
    shape_accel, shape_accel_xy, shape_pos_vel_accel, shape_pos_vel_accel_xy, shape_vel_accel,
    shape_vel_accel_xy, stopping_distance, update_pos_vel_accel, update_pos_vel_accel_xy,
    update_vel_accel, update_vel_accel_xy, Postype,
};
use ap_math::scalar::{is_positive, is_zero};
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

/// Gain applied to acceleration and jerk when already over speed, upstream
/// `POSCONTROL_OVERSPEED_GAIN_U`.
pub const OVERSPEED_GAIN_D: f32 = 2.0;

/// Largest upward stopping distance the vertical controller will plan,
/// upstream `POSCONTROL_STOPPING_DIST_UP_MAX_M`.
pub const STOPPING_DIST_UP_MAX_M: f32 = 3.0;

/// Largest downward stopping distance, upstream
/// `POSCONTROL_STOPPING_DIST_DOWN_MAX_M`.
pub const STOPPING_DIST_DOWN_MAX_M: f32 = 2.0;

/// The largest downward acceleration the vertical shaper will command,
/// regardless of what the limits say.
///
/// Upstream writes it inline as `constrain_float(accel_max, 0.0, 7.5)`. A
/// multirotor can push *up* with everything the motors have, but it can only
/// accelerate *down* by unloading them — and past free fall there is nothing
/// left to give. Seven and a half leaves margin below `g` so the aircraft
/// keeps some authority while descending hard.
const ACCEL_DOWN_MAX_MSS: f32 = 7.5;

/// The vertical limits.
#[derive(Debug, Clone, Copy)]
pub struct DLimits {
    /// Maximum descent speed, metres per second, positive.
    pub vel_max_down_ms: f32,
    /// Maximum climb speed, metres per second, positive.
    pub vel_max_up_ms: f32,
    /// Maximum vertical acceleration, metres per second squared.
    pub accel_max_d_mss: f32,
    /// Maximum vertical jerk, metres per second cubed.
    pub jerk_max_d_msss: f32,
}

impl DLimits {
    /// Derive the vertical limits, upstream `D_set_max_speed_accel_m`.
    ///
    /// Zero means *leave unchanged*, not "no limit" — the opposite of the
    /// horizontal setter, which takes whatever it is given. So this needs the
    /// previous limits to update rather than replacing them, and a caller
    /// wanting to change only the climb rate passes zero for the others.
    ///
    /// The jerk bound is a filter-bandwidth argument. The acceleration PID
    /// low-passes its target and its error, and commanding jerk faster than
    /// those filters can follow buys nothing — the command is smoothed away
    /// and all that remains is phase lag. So the jerk is capped at
    /// `min(g, accel_max) * 2π·f / 5`, one fifth of the filter's own corner
    /// rate. The `min` with gravity is there because a jerk budget derived
    /// from an acceleration the aircraft cannot reach is not a real budget.
    #[must_use]
    pub fn derive(
        previous: Self,
        descent_speed_max_ms: f32,
        climb_speed_max_ms: f32,
        accel_max_d_mss: f32,
        shaping_jerk_d_msss: f32,
        accel_pid_filt_t_hz: f32,
        accel_pid_filt_e_hz: f32,
    ) -> Self {
        let mut out = previous;
        if !is_zero(descent_speed_max_ms) {
            out.vel_max_down_ms = descent_speed_max_ms.abs();
        }
        if !is_zero(climb_speed_max_ms) {
            out.vel_max_up_ms = climb_speed_max_ms.abs();
        }
        if !is_zero(accel_max_d_mss) {
            out.accel_max_d_mss = accel_max_d_mss.abs();
        }

        out.jerk_max_d_msss = shaping_jerk_d_msss;
        let ceiling = GRAVITY_MSS.min(out.accel_max_d_mss);
        for filt_hz in [accel_pid_filt_t_hz, accel_pid_filt_e_hz] {
            if is_positive(filt_hz) {
                let bound = ceiling * (core::f32::consts::TAU * filt_hz) / 5.0;
                out.jerk_max_d_msss = out.jerk_max_d_msss.min(bound);
            }
        }
        out
    }

    /// Scale acceleration and jerk when already exceeding a speed limit,
    /// upstream `calculate_overspeed_gain`.
    ///
    /// A vehicle travelling faster than it is supposed to needs more authority
    /// than usual, not less — it has further to slow down and the same
    /// distance to do it in. The gain grows in proportion to the overspeed, so
    /// twice the permitted speed gets four times the acceleration budget.
    ///
    /// Both branches guard against a zero limit, which would otherwise divide.
    /// A zero limit means unconfigured, and an unconfigured axis should not be
    /// told it is over speed.
    #[must_use]
    pub fn overspeed_gain(&self, vel_desired_d_ms: f32) -> f32 {
        if vel_desired_d_ms > self.vel_max_down_ms && !is_zero(self.vel_max_down_ms) {
            return OVERSPEED_GAIN_D * vel_desired_d_ms / self.vel_max_down_ms;
        }
        if vel_desired_d_ms < -self.vel_max_up_ms && !is_zero(self.vel_max_up_ms) {
            return -OVERSPEED_GAIN_D * vel_desired_d_ms / self.vel_max_up_ms;
        }
        1.0
    }
}

/// The vertical controller's kinematic state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PosControlD {
    /// Desired position along down, upstream `_pos_desired_ned_m.z`.
    pub pos_desired_m: Postype,
    /// Desired velocity along down.
    pub vel_desired_ms: f32,
    /// Desired acceleration along down.
    pub accel_desired_mss: f32,
    /// Directional limit from the last controller run.
    pub limit: f32,
}

impl Default for PosControlD {
    fn default() -> Self {
        Self::new()
    }
}

impl PosControlD {
    /// Everything at zero.
    #[must_use]
    pub fn new() -> Self {
        Self {
            pos_desired_m: 0.0,
            vel_desired_ms: 0.0,
            accel_desired_mss: 0.0,
            limit: 0.0,
        }
    }

    fn advance(&mut self, dt: f32, pos_error: f32, vel_error: f32) {
        update_pos_vel_accel(
            &mut self.pos_desired_m,
            &mut self.vel_desired_ms,
            self.accel_desired_mss,
            dt,
            self.limit,
            pos_error,
            vel_error,
        );
    }

    /// The asymmetric acceleration bounds this axis shapes against.
    ///
    /// Down is positive, so the *lower* bound is upward acceleration and gets
    /// the full budget, while the upper is capped at [`ACCEL_DOWN_MAX_MSS`].
    fn accel_bounds(accel_max: f32) -> (f32, f32) {
        (-accel_max, accel_max.clamp(0.0, ACCEL_DOWN_MAX_MSS))
    }

    /// Command a vertical acceleration, upstream `input_accel_D_m`.
    pub fn input_accel(
        &mut self,
        accel_d_mss: f32,
        limits: &DLimits,
        dt: f32,
        pos_error: f32,
        vel_error: f32,
    ) {
        // Read before advancing: the gain describes the state the command is
        // being issued against.
        let jerk_max = limits.jerk_max_d_msss * limits.overspeed_gain(self.vel_desired_ms);
        self.advance(dt, pos_error, vel_error);
        let _ = shape_accel(accel_d_mss, &mut self.accel_desired_mss, jerk_max, dt);
    }

    /// Command a vertical velocity, upstream `input_vel_accel_D_m`.
    #[expect(
        clippy::too_many_arguments,
        reason = "upstream's signature plus the errors it reads from members \
this port does not own"
    )]
    pub fn input_vel_accel(
        &mut self,
        vel_d_ms: &mut f32,
        accel_d_mss: f32,
        limits: &DLimits,
        dt: f32,
        limit_output: bool,
        pos_error: f32,
        vel_error: f32,
    ) {
        let gain = limits.overspeed_gain(self.vel_desired_ms);
        let accel_max = limits.accel_max_d_mss * gain;
        let jerk_max = limits.jerk_max_d_msss * gain;
        let (accel_min, accel_up_max) = Self::accel_bounds(accel_max);

        self.advance(dt, pos_error, vel_error);

        let _ = shape_vel_accel(
            *vel_d_ms,
            accel_d_mss,
            self.vel_desired_ms,
            &mut self.accel_desired_mss,
            accel_min,
            accel_up_max,
            jerk_max,
            dt,
            limit_output,
        );

        update_vel_accel(vel_d_ms, accel_d_mss, dt, 0.0, 0.0);
    }

    /// Command a vertical position, upstream `input_pos_vel_accel_D_m`.
    #[expect(
        clippy::too_many_arguments,
        reason = "upstream's signature plus the errors it reads from members \
this port does not own"
    )]
    pub fn input_pos_vel_accel(
        &mut self,
        pos_d_m: &mut f32,
        vel_d_ms: &mut f32,
        accel_d_mss: f32,
        limits: &DLimits,
        dt: f32,
        limit_output: bool,
        pos_error: f32,
        vel_error: f32,
    ) {
        let gain = limits.overspeed_gain(self.vel_desired_ms);
        let accel_max = limits.accel_max_d_mss * gain;
        let jerk_max = limits.jerk_max_d_msss * gain;
        let (accel_min, accel_up_max) = Self::accel_bounds(accel_max);

        self.advance(dt, pos_error, vel_error);

        let _ = shape_pos_vel_accel(
            Postype::from(*pos_d_m),
            *vel_d_ms,
            accel_d_mss,
            self.pos_desired_m,
            self.vel_desired_ms,
            &mut self.accel_desired_mss,
            // Velocity bounds are the *climb* limit downward-negated and the
            // descent limit: up is negative in this frame.
            -limits.vel_max_up_ms,
            limits.vel_max_down_ms,
            accel_min,
            accel_up_max,
            jerk_max,
            dt,
            limit_output,
        );

        let mut pos = Postype::from(*pos_d_m);
        update_pos_vel_accel(&mut pos, vel_d_ms, accel_d_mss, dt, 0.0, 0.0, 0.0);
        *pos_d_m = pos as f32;
    }

    /// Fly a climb rate, upstream `D_set_pos_target_from_climb_rate_ms`.
    ///
    /// The sign flip is the whole function: a climb rate is positive upward
    /// and this axis is positive downward.
    ///
    /// `ignore_descent_limit` clamps the stored limit to at most zero, which
    /// removes any *downward* restriction while leaving an upward one intact.
    /// Used on landing, where the vehicle must be allowed to keep descending
    /// even though something has reported it cannot.
    pub fn set_pos_target_from_climb_rate(
        &mut self,
        climb_rate_ms: f32,
        ignore_descent_limit: bool,
        limits: &DLimits,
        dt: f32,
        pos_error: f32,
        vel_error: f32,
    ) {
        if ignore_descent_limit {
            self.limit = self.limit.min(0.0);
        }
        let mut vel_d_ms = -climb_rate_ms;
        self.input_vel_accel(&mut vel_d_ms, 0.0, limits, dt, true, pos_error, vel_error);
    }
}

/// Where the vehicle would come to rest vertically, upstream
/// `get_stopping_point_D_m`.
///
/// Bounded asymmetrically, and the asymmetry is not arbitrary: three metres up
/// against two metres down. Overshooting upward costs altitude the vehicle can
/// recover; overshooting downward may cost the vehicle. The tighter bound is
/// on the direction where being wrong is unrecoverable.
///
/// An unconfigured axis — no position gain or no acceleration limit — reports
/// the current position rather than guessing.
#[must_use]
pub fn stopping_point_d(
    pos_estimate_m: f32,
    pos_offset_m: f32,
    vel_estimate_ms: f32,
    vel_offset_ms: f32,
    kp: f32,
    limits: &DLimits,
) -> f32 {
    let curr_pos_d_m = pos_estimate_m - pos_offset_m;
    let curr_vel_d_ms = vel_estimate_ms - vel_offset_ms;

    if !is_positive(kp) || !is_positive(limits.accel_max_d_mss) {
        return curr_pos_d_m;
    }

    curr_pos_d_m
        + stopping_distance(curr_vel_d_ms, kp, limits.accel_max_d_mss)
            .clamp(-STOPPING_DIST_UP_MAX_M, STOPPING_DIST_DOWN_MAX_M)
}
