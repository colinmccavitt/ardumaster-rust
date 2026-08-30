//! Simulated inputs for driving the port without a flight log.
//!
//! Log replay answers "does the port match ArduPilot?". It cannot answer "does
//! the estimator actually estimate?", because a log contains upstream's
//! *estimate*, not the truth it was estimating. This crate supplies the
//! missing half: a vehicle whose true state is known exactly, emitting the
//! sensor samples that state would produce.
//!
//! It is also the shape of the external input the port is designed to receive,
//! so it doubles as a reference for what that side has to provide.
//!
//! # It shares no arithmetic with the port
//!
//! The truth is propagated here with Rodrigues' rotation formula, computed in
//! `f64`, deliberately *not* using `ap_math` or the port's DCM step. A
//! simulator that integrated attitude the same way the estimator does would
//! agree with it by construction and prove nothing. The truth must be better
//! than the thing under test, and visibly independent of it.
//!
//! # What it is not
//!
//! There is no aerodynamics here — no lift curve, no stall, no control
//! derivatives. The vehicle turns because it is told to turn. That is enough
//! to exercise an estimator, a navigation law and a control loop against known
//! truth, and it is not enough to say anything about how a real airframe would
//! behave. For that, fly SITL.

use core::f64::consts::PI;

/// A 3-vector in `f64`, kept separate from `ap_math` so the simulator cannot
/// accidentally inherit the port's arithmetic.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct V3 {
    /// X component.
    pub x: f64,
    /// Y component.
    pub y: f64,
    /// Z component.
    pub z: f64,
}

impl V3 {
    /// A vector.
    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// The zero vector.
    #[must_use]
    pub const fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    /// Euclidean length.
    #[must_use]
    pub fn length(self) -> f64 {
        self.dot(self).sqrt()
    }

    /// Dot product.
    #[must_use]
    pub fn dot(self, o: Self) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    /// Cross product.
    #[must_use]
    pub fn cross(self, o: Self) -> Self {
        Self::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }

    /// Scaled by a factor.
    #[must_use]
    pub fn scaled(self, k: f64) -> Self {
        Self::new(self.x * k, self.y * k, self.z * k)
    }

    /// Sum.
    #[must_use]
    pub fn plus(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

/// A 3x3 matrix in `f64`, rows first.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct M3 {
    /// First row.
    pub a: V3,
    /// Second row.
    pub b: V3,
    /// Third row.
    pub c: V3,
}

impl M3 {
    /// The identity.
    #[must_use]
    pub const fn identity() -> Self {
        Self {
            a: V3::new(1.0, 0.0, 0.0),
            b: V3::new(0.0, 1.0, 0.0),
            c: V3::new(0.0, 0.0, 1.0),
        }
    }

    /// Matrix product, `self * other`.
    #[must_use]
    pub fn times(self, o: Self) -> Self {
        let col = |k: usize| match k {
            0 => V3::new(o.a.x, o.b.x, o.c.x),
            1 => V3::new(o.a.y, o.b.y, o.c.y),
            _ => V3::new(o.a.z, o.b.z, o.c.z),
        };
        Self {
            a: V3::new(self.a.dot(col(0)), self.a.dot(col(1)), self.a.dot(col(2))),
            b: V3::new(self.b.dot(col(0)), self.b.dot(col(1)), self.b.dot(col(2))),
            c: V3::new(self.c.dot(col(0)), self.c.dot(col(1)), self.c.dot(col(2))),
        }
    }

    /// The vector transformed by this matrix.
    #[must_use]
    pub fn apply(self, v: V3) -> V3 {
        V3::new(self.a.dot(v), self.b.dot(v), self.c.dot(v))
    }

    /// An exact rotation about `axis` by `|axis|` radians, by Rodrigues'
    /// formula.
    ///
    /// Exact rather than first-order on purpose: this propagates the truth,
    /// and the estimator under test integrates with a first-order step. If
    /// both used the same approximation the comparison would be meaningless.
    #[must_use]
    pub fn from_rotation_vector(axis: V3) -> Self {
        let theta = axis.length();
        if theta < 1e-12 {
            return Self::identity();
        }
        let k = axis.scaled(1.0 / theta);
        let (s, c) = (theta.sin(), theta.cos());
        let one_c = 1.0 - c;
        Self {
            a: V3::new(
                c + k.x * k.x * one_c,
                k.x * k.y * one_c - k.z * s,
                k.x * k.z * one_c + k.y * s,
            ),
            b: V3::new(
                k.y * k.x * one_c + k.z * s,
                c + k.y * k.y * one_c,
                k.y * k.z * one_c - k.x * s,
            ),
            c: V3::new(
                k.z * k.x * one_c - k.y * s,
                k.z * k.y * one_c + k.x * s,
                c + k.z * k.z * one_c,
            ),
        }
    }

    /// The transpose, which for a rotation matrix is also its inverse.
    #[must_use]
    pub const fn transposed(self) -> Self {
        Self {
            a: V3::new(self.a.x, self.b.x, self.c.x),
            b: V3::new(self.a.y, self.b.y, self.c.y),
            c: V3::new(self.a.z, self.b.z, self.c.z),
        }
    }

    /// The rotation vector this matrix represents -- the inverse of
    /// [`M3::from_rotation_vector`].
    ///
    /// This is what makes the coning tests possible: it turns "the rotation
    /// that actually occurred over this window" into a vector that can be
    /// compared directly against an accumulated delta angle, without ever
    /// summing a rate.
    ///
    /// # Domain
    ///
    /// Valid for rotations under a half turn. The axis is extracted from the
    /// antisymmetric part, which vanishes as the angle approaches pi, so the
    /// result is meaningless there. Every window a flight loop measures is
    /// far smaller than that, and the assertion below says so out loud rather
    /// than returning a quietly wrong vector.
    #[must_use]
    pub fn to_rotation_vector(self) -> V3 {
        let trace = self.a.x + self.b.y + self.c.z;
        let theta = ((trace - 1.0) / 2.0).clamp(-1.0, 1.0).acos();
        if theta < 1e-15 {
            return V3::zero();
        }
        assert!(
            theta < 3.0,
            "to_rotation_vector is ill-conditioned near a half turn; got {theta} rad"
        );
        let s = theta.sin();
        V3::new(
            self.c.y - self.b.z,
            self.a.z - self.c.x,
            self.b.x - self.a.y,
        )
        .scaled(theta / (2.0 * s))
    }

    /// Roll, pitch and yaw in radians, from a body-to-earth matrix.
    #[must_use]
    pub fn to_euler(self) -> (f64, f64, f64) {
        let pitch = (-self.c.x).clamp(-1.0, 1.0).asin();
        let roll = self.c.y.atan2(self.c.z);
        let yaw = self.b.x.atan2(self.a.x);
        (roll, pitch, yaw)
    }
}

/// Standard gravity, m/s2.
pub const GRAVITY: f64 = 9.806_65;


/// Per-tick motor/throttle state from a host-side plane simulator frame.
///
/// Kept in `ap-sim` (not `ap-ins`) so kinematic truth stays independent of
/// firmware types; [`ap_plane::sitl_ins_noise_hookup`] converts into
/// [`SitlInsMotorRuntime`].
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PlaneMotorFrame {
    pub motors_on: bool,
    /// Throttle demand, percent `0..100`.
    pub throttle_pct: f64,
    pub motor_rpm: [f64; 8],
}

impl PlaneMotorFrame {
    /// Build from a throttle percentage, motors on when throttle is positive.
    #[must_use]
    pub fn from_throttle_pct(throttle_pct: f64) -> Self {
        Self {
            motors_on: throttle_pct > 0.0,
            throttle_pct,
            ..Self::default()
        }
    }
}

/// One IMU sample, in the form `AP_InertialSensor` would hand over.
#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    /// Integrated rotation over the interval, radians. What
    /// `get_delta_angle` returns.
    pub delta_angle: V3,
    /// The interval the delta angle covers, seconds.
    pub delta_angle_dt: f64,
    /// Instantaneous body rates, radians/s. What `get_gyro` returns.
    pub gyro: V3,
    /// Specific force in body frame, m/s2 — what an accelerometer measures,
    /// which is acceleration *minus* gravity, so a stationary vehicle reads
    /// `+g` upward rather than zero.
    pub accel: V3,
}

/// Sensor imperfections to apply on top of the truth.
///
/// All default to zero, so a simulation is exact until something is asked for.
/// The point of each is to exercise a specific correction in the estimator:
/// bias is what the integral drift term exists to remove, and scale error is
/// what shows up as attitude error proportional to rotation rate.
#[derive(Debug, Clone, Copy, Default)]
pub struct SensorErrors {
    /// Constant gyro bias, radians/s, added to the true rate.
    pub gyro_bias: V3,
    /// Gyro scale error as a fraction: 0.01 makes the gyro read 1% high.
    pub gyro_scale_error: f64,
    /// Constant accelerometer bias, m/s2.
    pub accel_bias: V3,
}

impl Default for V3 {
    fn default() -> Self {
        Self::zero()
    }
}

/// A vehicle whose true attitude is known exactly.
#[derive(Debug, Clone, Copy)]
pub struct AttitudeSim {
    /// The true attitude, body to earth. Not an estimate — this is what the
    /// vehicle is actually doing.
    pub truth: M3,
    /// Sensor imperfections applied to the samples this emits.
    pub errors: SensorErrors,
    /// Seconds since the simulation started.
    pub time_s: f64,
}

impl Default for AttitudeSim {
    fn default() -> Self {
        Self::new()
    }
}

impl AttitudeSim {
    /// A vehicle level and pointing north, with perfect sensors.
    #[must_use]
    pub fn new() -> Self {
        Self {
            truth: M3::identity(),
            errors: SensorErrors::default(),
            time_s: 0.0,
        }
    }

    /// Start from a given attitude, in radians.
    #[must_use]
    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        // Built by composing exact axis rotations rather than a closed-form
        // Euler matrix, so this file has exactly one rotation primitive.
        let truth = M3::from_rotation_vector(V3::new(0.0, 0.0, yaw))
            .times(M3::from_rotation_vector(V3::new(0.0, pitch, 0.0)))
            .times(M3::from_rotation_vector(V3::new(roll, 0.0, 0.0)));
        Self {
            truth,
            ..Self::new()
        }
    }

    /// Advance by `dt` seconds at the given true body rates, and return the
    /// sample an IMU would have produced.
    ///
    /// The rates are taken as constant across the interval, so the delta angle
    /// is exactly `rates * dt` — there is no coning to correct, which is what
    /// makes this a clean reference. A model with rotating rates would need
    /// coning compensation to stay honest.
    pub fn step(&mut self, rates: V3, dt: f64) -> ImuSample {
        let delta = rates.scaled(dt);
        // Exact propagation of the truth.
        self.truth = self.truth.times(M3::from_rotation_vector(delta));
        self.time_s += dt;

        let scale = 1.0 + self.errors.gyro_scale_error;
        let measured_rate = rates.scaled(scale).plus(self.errors.gyro_bias);

        // A stationary accelerometer measures specific force: gravity pushing
        // up through the airframe, expressed in body axes. Earth-frame -g on
        // the down axis becomes +g on body z when level.
        let gravity_body = self.gravity_in_body();

        ImuSample {
            delta_angle: measured_rate.scaled(dt),
            delta_angle_dt: dt,
            gyro: measured_rate,
            accel: gravity_body.plus(self.errors.accel_bias),
        }
    }

    /// Advance by `dt` along a *continuously varying* rate, and return the
    /// sample a real IMU would produce -- the instantaneous rate at the end of
    /// the interval.
    ///
    /// # When to use this rather than [`AttitudeSim::step`]
    ///
    /// `step` holds the rate constant across the interval, which makes the
    /// true rotation exactly `rate * dt` and the truth trivially checkable.
    /// That is what the attitude tests want.
    ///
    /// It is the wrong model for anything that integrates *between* samples.
    /// A trapezoidal integrator assumes its samples are points on a smooth
    /// curve; against a staircase it is a half-step lag, and on a sweeping
    /// rate that lag is a rotating phase error large enough to swamp the
    /// effect being measured. Ask for this mode instead: truth advances in
    /// `substeps` sub-intervals using the rate at each sub-interval's
    /// midpoint, so the composed rotation approaches the true continuous one,
    /// and the reported gyro is a genuine point sample of the curve.
    ///
    /// `substeps` trades cost against how exact the truth is. The residual is
    /// second order in the sub-interval, so 64 leaves the truth several orders
    /// below anything an f32 integrator can resolve.
    ///
    /// # Panics
    ///
    /// If `substeps` is zero.
    pub fn step_continuous(&mut self, profile: RateProfile, dt: f64, substeps: usize) -> ImuSample {
        assert!(substeps > 0, "substeps must be positive");
        let sub = dt / substeps as f64;
        for i in 0..substeps {
            #[allow(
                clippy::cast_precision_loss,
                reason = "substeps is a small loop count chosen by the caller"
            )]
            let midpoint = self.time_s + sub * (i as f64 + 0.5);
            let rates = profile(midpoint);
            self.truth = self
                .truth
                .times(M3::from_rotation_vector(rates.scaled(sub)));
        }
        self.time_s += dt;

        let rates = profile(self.time_s);
        let scale = 1.0 + self.errors.gyro_scale_error;
        let measured_rate = rates.scaled(scale).plus(self.errors.gyro_bias);

        ImuSample {
            // Reported for completeness. An integrator being tested on this
            // mode should be using `gyro`; the delta angle here is the same
            // rectangular approximation the sensor would report, and carries
            // the sampling error this mode exists to expose.
            delta_angle: measured_rate.scaled(dt),
            delta_angle_dt: dt,
            gyro: measured_rate,
            accel: self.gravity_in_body().plus(self.errors.accel_bias),
        }
    }

    /// Gravity as the accelerometer sees it, body frame.
    fn gravity_in_body(&self) -> V3 {
        // The truth is body-to-earth, so its transpose takes earth to body.
        let t = M3 {
            a: V3::new(self.truth.a.x, self.truth.b.x, self.truth.c.x),
            b: V3::new(self.truth.a.y, self.truth.b.y, self.truth.c.y),
            c: V3::new(self.truth.a.z, self.truth.b.z, self.truth.c.z),
        };
        t.apply(V3::new(0.0, 0.0, -GRAVITY))
    }

    /// The true attitude as roll, pitch and yaw in radians.
    #[must_use]
    pub fn true_euler(&self) -> (f64, f64, f64) {
        self.truth.to_euler()
    }
}

/// A body-rate profile: what the vehicle is commanded to do over time.
///
/// Deliberately a plain function of time rather than a control loop, so a test
/// can state exactly what motion it is checking against.
pub type RateProfile = fn(f64) -> V3;

pub mod sim_plane;


/// Level flight: no rotation at all.
#[must_use]
pub fn level(_t: f64) -> V3 {
    V3::zero()
}

/// A steady roll at 30 degrees per second.
#[must_use]
pub fn steady_roll(_t: f64) -> V3 {
    V3::new(30.0 * PI / 180.0, 0.0, 0.0)
}

/// A coordinated-looking turn: mostly yaw, a little roll to enter it.
#[must_use]
pub fn turning(t: f64) -> V3 {
    let roll_in = if t < 1.0 { 20.0 } else { 0.0 };
    V3::new(roll_in * PI / 180.0, 0.0, 15.0 * PI / 180.0)
}

/// Rotation about all three axes at once, which no simple test motion
/// exercises and which is where an estimator's cross-axis terms show up.
#[must_use]
pub fn tumbling(t: f64) -> V3 {
    V3::new(
        0.4 * (t * 0.7).sin(),
        0.3 * (t * 1.1).cos(),
        0.2 * (t * 0.5).sin(),
    )
}

/// A rate vector of constant magnitude sweeping around the body x axis at
/// 30 Hz -- the classic coning motion.
///
/// Nothing here is rotating "about" a fixed axis: the axis itself turns, which
/// is precisely the condition under which the integral of the rate stops being
/// the rotation that occurred. Vibration through a compliant airframe mount
/// produces the same thing at similar frequencies, which is why the correction
/// is worth having on a real vehicle and not only in a paper.
#[must_use]
pub fn coning(t: f64) -> V3 {
    let sweep = 2.0 * PI * 30.0;
    let amplitude = 4.0;
    V3::new(
        0.0,
        amplitude * (sweep * t).cos(),
        amplitude * (sweep * t).sin(),
    )
}
