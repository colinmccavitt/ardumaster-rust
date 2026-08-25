//! Port of `APM_Control/AP_SteerController`. Tracked as FW-017.
//!
//! Nosewheel and tailwheel steering, used on the takeoff roll, the landing
//! rollout and while taxiing. It is a hand-rolled PID rather than an
//! [`ap_pid::AcPid`], and its integrator *is* the reported `I` — there is no
//! separate state — so resetting the report resets the controller.
//!
//! The gain conversion constants (`45.0`, `4.0`) exist so that `STEER2SRV_P`
//! and friends keep the meaning they had under an older control law. They are
//! reproduced as written rather than folded into the gains, because the
//! parameter values in every existing airframe file assume them.
//!
//! # Divergence
//!
//! D-018: the integrator is gated on the *measured* groundspeed, not on the
//! clamped one. Upstream's guard cannot fire. See [`SteerController::steering_out_rate`].

use ap_math::scalar::{constrain_value, degrees, is_zero, radians};
use ap_pid::PidInfo;

/// Steering gains, upstream's `STEER2SRV_*` parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SteerGains {
    /// Time constant from angle error to demanded rate, seconds. Upstream
    /// `STEER2SRV_TCONST`; clamped up to 0.1 when used.
    pub tau: f32,
    /// Feed-forward gain, upstream `STEER2SRV_FF`.
    pub k_ff: f32,
    /// Proportional gain, upstream `STEER2SRV_P`.
    pub k_p: f32,
    /// Integral gain, upstream `STEER2SRV_I`.
    pub k_i: f32,
    /// Derivative gain, upstream `STEER2SRV_D`.
    pub k_d: f32,
    /// Speed floor for the inverse-steering approximation, m/s. Upstream
    /// `STEER2SRV_MINSPD`.
    pub minspeed: f32,
    /// Integrator limit in centidegrees, upstream `STEER2SRV_IMAX`, an
    /// `AP_Int16`.
    pub imax: i16,
    /// Speed above which the output is derated, m/s. Zero disables it.
    /// Upstream `STEER2SRV_DRTSPD`.
    pub deratespeed: f32,
    /// Derating slope, upstream `STEER2SRV_DRTFCT`.
    pub deratefactor: f32,
    /// Floor for the derated limit, centidegrees. Upstream `STEER2SRV_DRTMIN`.
    pub mindegree: f32,
}

impl Default for SteerGains {
    /// Upstream's parameter defaults.
    fn default() -> Self {
        Self {
            tau: 0.75,
            k_ff: 0.0,
            k_p: 1.8,
            k_i: 0.2,
            k_d: 0.005,
            minspeed: 1.0,
            imax: 1500,
            deratespeed: 0.0,
            deratefactor: 10.0,
            mindegree: 4500.0,
        }
    }
}

/// Lower bound upstream writes back into the time constant parameter.
const MIN_TAU: f32 = 0.1;

/// Full surface deflection, centidegrees.
const FULL_DEFLECTION_CD: f32 = 4500.0;

/// Gap after which the controller counts as inactive, milliseconds. Also the
/// gap after which the loop period is treated as zero.
const RESTART_GAP_MS: u32 = 1000;

/// Ground steering controller, upstream `AP_SteerController`.
#[derive(Debug, Clone, Copy)]
pub struct SteerController {
    /// The gains.
    pub gains: SteerGains,
    reverse: bool,
    last_t_ms: u32,
    last_out: f32,
    info: PidInfo,
}

impl SteerController {
    /// A steering controller with the given gains.
    #[must_use]
    pub fn new(gains: SteerGains) -> Self {
        Self {
            gains,
            reverse: false,
            last_t_ms: 0,
            last_out: 0.0,
            info: PidInfo::default(),
        }
    }

    /// The PID info the vehicle logs.
    #[must_use]
    pub fn info(&self) -> PidInfo {
        self.info
    }

    /// Invert the sense of the measured and demanded yaw rate, upstream
    /// `set_reverse`. Set when the vehicle is travelling backwards.
    pub fn set_reverse(&mut self, reverse: bool) {
        self.reverse = reverse;
    }

    /// Clear the integrator, upstream `reset_I`. The integrator and its
    /// reported value are the same field, as upstream.
    pub fn reset_i(&mut self) {
        self.info.i = 0.0;
    }

    /// True if the controller has run within the last second, upstream
    /// `active()`.
    #[must_use]
    pub fn active(&self, now_ms: u32) -> bool {
        now_ms.wrapping_sub(self.last_t_ms) < RESTART_GAP_MS
    }

    /// Steering rate controller, upstream `get_steering_out_rate`.
    ///
    /// `desired_rate_deg` is a yaw rate in deg/s, positive clockwise. Returns
    /// a steering demand in centidegrees.
    ///
    /// # D-018 — the integrator gate uses the measured groundspeed
    ///
    /// Upstream writes `if (ki_rate > 0 && speed >= _minspeed)`, where `speed`
    /// is the local that was clamped **up** to `_minspeed` twenty lines
    /// earlier. The comparison is therefore always true and the gate has never
    /// fired. The commit that added it (39bfd809c2, 2013) is titled "disable
    /// integrator below minimum speed" and says "this reduces the impact on
    /// initial takeoff", so the intent is not in doubt. The port gates on the
    /// measured speed.
    #[allow(
        clippy::cast_possible_truncation,
        reason = "upstream returns int32 centidegrees from a float already \
constrained to the derate limit"
    )]
    pub fn steering_out_rate(&mut self, desired_rate_deg: f32, inp: &SteerInputs) -> i32 {
        let mut dt_ms = inp.now_ms.wrapping_sub(self.last_t_ms);
        if self.last_t_ms == 0 || dt_ms > RESTART_GAP_MS {
            dt_ms = 0;
        }
        self.last_t_ms = inp.now_ms;

        // A speed floor keeps the inverse-steering approximation from blowing
        // up, and stops the oscillation the vehicle would otherwise develop as
        // it first starts to move.
        let speed = inp.groundspeed.max(self.gains.minspeed);

        // A linear approximation of the inverse steering equation for a ground
        // vehicle, giving a steering angle between -45 and 45 degrees.
        let scaler = 1.0 / speed;

        self.info.target = desired_rate_deg;

        // Measured in the earth frame so that a vehicle leaning through a hard
        // corner still reports the yaw rate the steering has to match.
        let mut yaw_rate_earth = degrees(inp.yaw_rate_earth_rad);
        if self.reverse {
            yaw_rate_earth *= -1.0;
        }
        self.info.actual = yaw_rate_earth;

        let rate_error = (desired_rate_deg - yaw_rate_earth) * scaler;

        // Equivalent gains, so that K_P and K_I keep the meaning they had
        // under the older control law. K_D needs no conversion.
        let ki_rate = self.gains.k_i * self.gains.tau * 45.0;
        let kp_ff = ((self.gains.k_p - self.gains.k_i * self.gains.tau) * self.gains.tau
            - self.gains.k_d)
            .max(0.0)
            * 45.0;
        let k_ff = self.gains.k_ff * 45.0;
        let delta_time = dt_ms as f32 * 0.001;

        // D-018: `inp.groundspeed`, not `speed`. Upstream compares the clamped
        // value against the bound it was clamped to.
        if ki_rate > 0.0 && inp.groundspeed >= self.gains.minspeed {
            if dt_ms > 0 {
                let mut integrator_delta = rate_error * ki_rate * delta_time * scaler;
                // Do not wind further into a demand that is already saturated.
                if self.last_out < -45.0 {
                    integrator_delta = integrator_delta.max(0.0);
                } else if self.last_out > 45.0 {
                    integrator_delta = integrator_delta.min(0.0);
                }
                self.info.i += integrator_delta;
            }
        } else {
            self.info.i = 0.0;
        }

        let int_lim_scaled = f32::from(self.gains.imax) * 0.01;
        self.info.i = constrain_value(self.info.i, -int_lim_scaled, int_lim_scaled);

        self.info.d = rate_error * self.gains.k_d * 4.0;
        self.info.p = radians(desired_rate_deg) * kp_ff * scaler;
        self.info.ff = radians(desired_rate_deg) * k_ff * scaler;

        self.last_out = self.info.d + self.info.ff + self.info.p + self.info.i;

        // Above the derate speed the available deflection is reduced, so that
        // a fast taxi cannot command a turn sharp enough to drag a wingtip.
        let mut derate_constraint = FULL_DEFLECTION_CD;
        if !is_zero(self.gains.deratespeed) && speed > self.gains.deratespeed {
            derate_constraint = FULL_DEFLECTION_CD
                - (speed - self.gains.deratespeed) * self.gains.deratefactor * 100.0;
            if derate_constraint < self.gains.mindegree {
                derate_constraint = self.gains.mindegree;
            }
        }

        constrain_value(self.last_out * 100.0, -derate_constraint, derate_constraint) as i32
    }

    /// Lateral acceleration controller, upstream
    /// `get_steering_out_lat_accel`. `desired_accel` is in m/s2.
    ///
    /// Note this applies its own speed floor before converting to a rate, and
    /// then [`Self::steering_out_rate`] applies it again — upstream computes
    /// the floor twice rather than passing the clamped speed down.
    pub fn steering_out_lat_accel(&mut self, desired_accel: f32, inp: &SteerInputs) -> i32 {
        let speed = inp.groundspeed.max(self.gains.minspeed);
        let mut desired_rate = degrees(desired_accel / speed);
        if self.reverse {
            desired_rate *= -1.0;
        }
        self.steering_out_rate(desired_rate, inp)
    }

    /// Angle-error controller, upstream `get_steering_out_angle_error`.
    /// `angle_err_cd` is a heading error in centidegrees.
    pub fn steering_out_angle_error(&mut self, angle_err_cd: i32, inp: &SteerInputs) -> i32 {
        // Upstream writes the clamp back into the parameter, so a too-small
        // configured tau is corrected permanently rather than per call.
        if self.gains.tau < MIN_TAU {
            self.gains.tau = MIN_TAU;
        }
        #[allow(
            clippy::cast_precision_loss,
            reason = "matches upstream's `angle_err * 0.01f` on an int32; a \
heading error is bounded by +-18000"
        )]
        let desired_rate = angle_err_cd as f32 * 0.01 / self.gains.tau;
        self.steering_out_rate(desired_rate, inp)
    }
}

/// What the steering controller reads from the vehicle each loop.
#[derive(Debug, Clone, Copy)]
pub struct SteerInputs {
    /// Milliseconds since boot. The controller measures its own loop period
    /// from this rather than being told one.
    pub now_ms: u32,
    /// Groundspeed, m/s. Upstream `ahrs.groundspeed()`.
    pub groundspeed: f32,
    /// Earth-frame yaw rate, radians/s. Upstream `ahrs.get_yaw_rate_earth()`.
    pub yaw_rate_earth_rad: f32,
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp, reason = "exact values on exact inputs")]

    use super::*;

    fn inputs() -> SteerInputs {
        SteerInputs {
            now_ms: 0,
            groundspeed: 10.0,
            yaw_rate_earth_rad: 0.0,
        }
    }

    fn controller() -> SteerController {
        SteerController::new(SteerGains::default())
    }

    /// D-018. Upstream's gate compares the clamped speed against the bound it
    /// was clamped to, so it is always true and the integrator winds while the
    /// aircraft is standing still. The port gates on the measured speed.
    #[test]
    fn d018_the_integrator_is_gated_on_the_measured_groundspeed() {
        let mut c = controller();
        let mut inp = inputs();
        inp.groundspeed = 0.0; // stationary, below MINSPD of 1.0
        c.steering_out_rate(30.0, &inp);
        for step in 1..40 {
            inp.now_ms = step * 20;
            c.steering_out_rate(30.0, &inp);
        }
        assert_eq!(
            c.info().i,
            0.0,
            "the integrator must not wind up while stationary"
        );

        // upstream's behaviour, for contrast: above the floor it does wind
        inp.groundspeed = 5.0;
        for step in 40..80 {
            inp.now_ms = step * 20;
            c.steering_out_rate(30.0, &inp);
        }
        assert!(
            c.info().i > 0.0,
            "above the floor the integrator must still work, got {}",
            c.info().i
        );
    }

    /// The first call measures a zero loop period, so nothing integrates.
    #[test]
    fn the_first_call_integrates_nothing() {
        let mut c = controller();
        let mut inp = inputs();
        inp.now_ms = 5000;
        c.steering_out_rate(30.0, &inp);
        assert_eq!(c.info().i, 0.0);
    }

    /// The integrator is the reported value, so resetting the report resets
    /// the controller — unlike the yaw damper, where they are separate.
    #[test]
    fn the_integrator_and_its_report_are_the_same_field() {
        let mut c = controller();
        let mut inp = inputs();
        c.steering_out_rate(30.0, &inp);
        for step in 1..40 {
            inp.now_ms = step * 20;
            c.steering_out_rate(30.0, &inp);
        }
        assert!(c.info().i > 0.0);
        c.reset_i();
        assert_eq!(c.info().i, 0.0);
    }

    /// Above the derate speed the available deflection shrinks linearly, with
    /// a floor.
    #[test]
    fn the_output_is_derated_above_the_derate_speed() {
        let mut c = controller();
        c.gains.deratespeed = 10.0;
        c.gains.deratefactor = 10.0;
        c.gains.mindegree = 1000.0;
        let mut inp = inputs();

        // 15 m/s: 4500 - (15-10)*10*100 = -500, floored to 1000
        inp.groundspeed = 15.0;
        let out = c.steering_out_rate(1000.0, &inp);
        assert_eq!(out, 1000, "expected the floor, got {out}");

        // 10.5 m/s: 4500 - 0.5*10*100 = 4000
        inp.now_ms = 20;
        inp.groundspeed = 10.5;
        let out = c.steering_out_rate(1000.0, &inp);
        assert_eq!(out, 4000, "expected the derated limit, got {out}");
    }

    /// A gap longer than a second makes the controller inactive and zeroes the
    /// loop period, so the first call after it integrates nothing.
    #[test]
    fn a_long_gap_suppresses_integration_and_reports_inactive() {
        let mut c = controller();
        let mut inp = inputs();
        c.steering_out_rate(30.0, &inp);
        for step in 1..40 {
            inp.now_ms = step * 20;
            c.steering_out_rate(30.0, &inp);
        }
        let wound = c.info().i;
        assert!(wound > 0.0);
        assert!(c.active(inp.now_ms));

        inp.now_ms = 30_000;
        assert!(!c.active(inp.now_ms));
        c.steering_out_rate(30.0, &inp);
        assert_eq!(c.info().i, wound, "a zero period must not integrate");
    }

    /// Reverse flips the sense of the measured rate, so the same physical yaw
    /// produces the opposite error.
    #[test]
    fn reverse_flips_the_measured_rate() {
        let mut c = controller();
        let mut inp = inputs();
        inp.yaw_rate_earth_rad = 0.5;
        c.steering_out_rate(0.0, &inp);
        let forward = c.info().actual;

        c.set_reverse(true);
        inp.now_ms = 20;
        c.steering_out_rate(0.0, &inp);
        assert_eq!(c.info().actual, -forward);
    }

    /// The time constant is clamped up, and upstream writes the clamp back
    /// into the parameter rather than applying it per call.
    #[test]
    fn tau_below_the_minimum_is_raised_permanently() {
        let mut c = controller();
        c.gains.tau = 0.01;
        let inp = inputs();
        c.steering_out_angle_error(1000, &inp);
        assert_eq!(c.gains.tau, MIN_TAU);
    }
}
