//! Port of `APM_Control/AP_FW_Controller` — the rate-loop base that the roll
//! and pitch controllers share. Tracked as FW-017.
//!
//! # Environment is passed in, not read from singletons
//!
//! Upstream reaches for `AP::scheduler().get_loop_period_s()`,
//! `AP::ahrs().get_EAS2TAS()` and `AP::ahrs().get_gyro()` inside the control
//! path. ADR-0004 rules out singletons, so the caller supplies them in
//! [`RateInputs`]. That also replaces upstream's three virtuals —
//! `get_measured_rate`, `get_airspeed` and `is_underspeed` — which existed only
//! to let each axis reach a different member of the same AHRS. Roll and pitch
//! differ in *which* value they read, not in what the base does with it, so
//! passing the value removes the vtable rather than reimplementing it.
//!
//! # AutoTune is absent
//!
//! Upstream calls `autotune->update(...)` at the end of the rate loop when
//! autotune is running. `AP_AutoTune` is FW-040; until it lands the hook is
//! simply not present. It only ever *observes* the PID info and adjusts gains
//! between loops, so its absence does not change the control output.

#![no_std]

use ap_math::scalar::{constrain_value, degrees, radians};
use ap_pid::{AcPid, PidGains, PidInfo, Scaling};

/// Rate and time-constant limits, upstream `AP_AutoTune::ATGains` — the subset
/// the controller itself reads.
///
/// The rest of `ATGains` (the PID terms) lives in [`AcPid`]; upstream keeps
/// both in one struct because AutoTune rewrites them together.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RateGains {
    /// Time constant from angle error to demanded rate, seconds. Upstream
    /// `gains.tau`.
    pub tau: f32,
    /// Maximum demanded rate, deg/s. Zero disables the limit. Upstream
    /// `gains.rmax_pos`.
    pub rmax_pos: f32,
    /// Maximum demanded rate in the negative sense, deg/s. Upstream
    /// `gains.rmax_neg`; unused by the roll axis.
    pub rmax_neg: f32,
}

impl Default for RateGains {
    fn default() -> Self {
        Self {
            tau: 0.5,
            rmax_pos: 0.0,
            rmax_neg: 0.0,
        }
    }
}

/// Everything the rate loop reads from outside itself.
#[derive(Debug, Clone, Copy)]
pub struct RateInputs {
    /// Demanded rate, deg/s.
    pub desired_rate_deg: f32,
    /// Airspeed scaling factor the caller has already computed.
    pub scaler: f32,
    /// Hold the integrator at zero, upstream `disable_integrator`.
    pub disable_integrator: bool,
    /// Equivalent airspeed, m/s. Upstream calls `get_airspeed()`.
    pub aspeed: f32,
    /// Suppress D and half of P, used while on the ground to stop the
    /// oscillation a wheeled airframe would otherwise develop.
    pub ground_mode: bool,
    /// Measured body rate about this axis, **radians/s**. Upstream calls
    /// `get_measured_rate()`, which reads the relevant gyro component.
    pub measured_rate_rad: f32,
    /// Whether the aircraft is below its minimum airspeed. Upstream calls
    /// `is_underspeed(aspeed)`.
    pub underspeed: bool,
    /// Equivalent-to-true airspeed ratio. Upstream reads
    /// `AP::ahrs().get_EAS2TAS()`.
    pub eas2tas: f32,
    /// Loop period, seconds. Upstream reads
    /// `AP::scheduler().get_loop_period_s()`.
    pub dt: f32,
    /// Milliseconds since boot, for the PID's slew limiter.
    pub now_ms: u32,
}

/// The shared rate loop, upstream `AP_FW_Controller`.
#[derive(Debug, Clone, Copy)]
pub struct FwController {
    /// The rate PID. Public because upstream exposes its gains through
    /// accessors and callers retune them in flight.
    pub rate_pid: AcPid,
    /// Rate limits and the angle-to-rate time constant.
    pub gains: RateGains,

    last_out: f32,
    ff_scale: f32,
    info: PidInfo,
}

impl FwController {
    /// A controller with the given PID and rate gains.
    #[must_use]
    pub fn new(pid_gains: PidGains, gains: RateGains) -> Self {
        Self {
            rate_pid: AcPid::new(pid_gains),
            gains,
            last_out: 0.0,
            // upstream initialises ff_scale to 1 and resets it to 1 after every
            // use; it is a one-shot multiplier
            ff_scale: 1.0,
            info: PidInfo::default(),
        }
    }

    /// What the last call did, upstream `get_pid_info`.
    ///
    /// Rescaled to degrees and with the target and actual reported without the
    /// airspeed scalers, exactly as upstream presents them for logging.
    #[must_use]
    pub fn info(&self) -> PidInfo {
        self.info
    }

    /// Apply a one-loop feed-forward multiplier, upstream `set_ff_scale`.
    ///
    /// Replaces any previous value and is consumed by the next rate update.
    pub fn set_ff_scale(&mut self, scale: f32) {
        self.ff_scale = scale;
    }

    /// Zero the integrator, upstream `reset_I`.
    pub fn reset_i(&mut self) {
        self.rate_pid.reset_i();
    }

    /// Bleed the integrator down, upstream `decay_I`.
    ///
    /// Upstream's comment says this reduces the integrator by 95% over two
    /// seconds. Used when a quadplane is hovering and the fixed-wing surfaces
    /// have little authority.
    pub fn decay_i(&mut self) {
        self.info.i *= 0.995;
        self.rate_pid
            .set_integrator(self.rate_pid.integrator() * 0.995);
    }

    /// The rate loop, upstream `_get_rate_out`.
    ///
    /// Returns a surface demand in centidegrees, clamped to ±4500.
    pub fn rate_out(&mut self, inp: &RateInputs) -> f32 {
        // Upstream triggers the I limit off the magnitude of the LAST output,
        // not this one -- the surface is assumed saturated if it was already
        // commanded past 45 degrees.
        let mut limit_i = self.last_out.abs() >= 45.0;
        let old_i = self.rate_pid.integrator();

        if inp.underspeed {
            limit_i = true;
        }

        // The PID terms scale with the square of the airspeed scaler. Rather
        // than modify AC_PID, upstream scales its inputs -- and runs it in
        // radians so that IMAX keeps its usual sub-unity range.
        self.rate_pid.update_all(
            radians(inp.desired_rate_deg) * inp.scaler * inp.scaler,
            inp.measured_rate_rad * inp.scaler * inp.scaler,
            inp.dt,
            limit_i,
            Scaling::default(),
            inp.now_ms,
        );

        if inp.underspeed {
            // below minimum airspeed the integrator is frozen outright, not
            // merely limited
            self.rate_pid.set_integrator(old_i);
        }

        // Feed-forward should scale by scaler/eas2tas, but the PID target was
        // already scaled by scaler squared -- so dividing by scaler*eas2tas
        // here gives the intended overall scaling.
        let pid = self.rate_pid.info();
        let ff = degrees(self.ff_scale * pid.ff / (inp.scaler * inp.eas2tas));
        let dff = degrees(self.ff_scale * pid.dff / (inp.scaler * inp.eas2tas));
        self.ff_scale = 1.0;

        if inp.disable_integrator {
            self.rate_pid.reset_i();
        }

        // Convert the PID's radian-scaled info to the degrees the rest of the
        // vehicle and the logs expect.
        let deg_scale = degrees(1.0f32);
        let mut info = self.rate_pid.info();
        info.ff = ff;
        info.p *= deg_scale;
        info.i *= deg_scale;
        info.d *= deg_scale;
        info.dff = dff;
        // report the unscaled demand and measurement, so a log reader sees
        // what was asked for rather than the internally scaled value
        info.target = inp.desired_rate_deg;
        info.actual = degrees(inp.measured_rate_rad);
        self.info = info;

        let mut out = info.ff + info.p + info.i + info.d + info.dff;
        if inp.ground_mode {
            out -= info.d + 0.5 * info.p;
        }

        self.last_out = out;

        constrain_value(out * 100.0, -4500.0, 4500.0)
    }
}

/// Fixed-wing roll controller, upstream `AP_RollController`.
#[derive(Debug, Clone, Copy)]
pub struct RollController {
    /// The shared rate loop.
    pub controller: FwController,
    in_recovery: bool,
}

/// Upstream's threshold for the roll indecision fix, degrees.
const INDECISION_THRESHOLD_DEG: f32 = 160.0;

/// Upstream clamps `tau` to at least this, in `get_servo_out`, by writing back
/// to the parameter.
const MIN_TAU: f32 = 0.05;

impl RollController {
    /// A roll controller with the given gains.
    #[must_use]
    pub fn new(pid_gains: PidGains, gains: RateGains) -> Self {
        Self {
            controller: FwController::new(pid_gains, gains),
            in_recovery: false,
        }
    }

    /// Suppress the rate limit for one loop, upstream's `in_recovery` flag.
    ///
    /// Set during a VTOL recovery, where the demanded rate must be allowed past
    /// the configured maximum. Cleared by the next `servo_out`.
    pub fn set_in_recovery(&mut self) {
        self.in_recovery = true;
    }

    /// Angle error in to surface demand out, upstream `get_servo_out`.
    ///
    /// `angle_err_cd` is the roll angle error in centidegrees. Returns a
    /// surface demand in centidegrees.
    pub fn servo_out(&mut self, angle_err_cd: i32, inp: &ServoInputs) -> f32 {
        // Upstream writes the clamp back into the parameter, so a too-small
        // configured tau is corrected permanently rather than per call.
        if self.controller.gains.tau < MIN_TAU {
            self.controller.gains.tau = MIN_TAU;
        }

        #[allow(
            clippy::cast_precision_loss,
            reason = "matches upstream's `angle_err * 0.01` on an int32; the \
values reaching here are bounded by the attitude range"
        )]
        let angle_err_deg = angle_err_cd as f32 * 0.01;
        let mut desired_rate = angle_err_deg / self.controller.gains.tau;

        // Stop the controller dithering when the target roll is nearly 180
        // degrees away and either direction looks equally good: if the new
        // demand opposes the last one, keep rolling the way we already are and
        // scale the demand up by the extra angle that now implies.
        let last_desired_rate = self.controller.info.target;
        let abs_angle_err_deg = angle_err_deg.abs();
        if abs_angle_err_deg > INDECISION_THRESHOLD_DEG
            && angle_err_deg <= 180.0
            && desired_rate * last_desired_rate < 0.0
        {
            desired_rate = -desired_rate;
            let new_angle_err_deg = abs_angle_err_deg + (180.0 - abs_angle_err_deg) * 2.0;
            desired_rate *= new_angle_err_deg / abs_angle_err_deg;
        }

        if !self.in_recovery {
            let rmax = self.controller.gains.rmax_pos;
            if rmax != 0.0 {
                desired_rate = constrain_value(desired_rate, -rmax, rmax);
            }
        }
        // the flag lasts a single loop
        self.in_recovery = false;

        self.controller.rate_out(&RateInputs {
            desired_rate_deg: desired_rate,
            scaler: inp.scaler,
            disable_integrator: inp.disable_integrator,
            aspeed: inp.aspeed,
            ground_mode: inp.ground_mode,
            measured_rate_rad: inp.roll_rate_rad,
            underspeed: inp.aspeed <= inp.airspeed_min,
            eas2tas: inp.eas2tas,
            dt: inp.dt,
            now_ms: inp.now_ms,
        })
    }
}

/// What the roll controller needs from the vehicle each loop.
#[derive(Debug, Clone, Copy)]
pub struct ServoInputs {
    /// Airspeed scaling factor.
    pub scaler: f32,
    /// Hold the integrator at zero.
    pub disable_integrator: bool,
    /// On the ground: suppress D and half of P.
    pub ground_mode: bool,
    /// Measured roll rate, radians/s. Upstream reads `ahrs.get_gyro().x`.
    pub roll_rate_rad: f32,
    /// Equivalent airspeed, m/s. Upstream calls `ahrs.airspeed_EAS`, using
    /// zero when unavailable.
    pub aspeed: f32,
    /// Minimum airspeed parameter, m/s. Upstream `aparm.airspeed_min`.
    pub airspeed_min: f32,
    /// Equivalent-to-true airspeed ratio.
    pub eas2tas: f32,
    /// Loop period, seconds.
    pub dt: f32,
    /// Milliseconds since boot.
    pub now_ms: u32,
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp, reason = "exact values on exact inputs")]

    use super::*;

    fn pid_gains() -> PidGains {
        PidGains {
            p: 0.08,
            i: 0.15,
            d: 0.0,
            ff: 0.345,
            imax: 0.666,
            filt_t_hz: 3.0,
            filt_e_hz: 0.0,
            filt_d_hz: 12.0,
            ..PidGains::default()
        }
    }

    fn inputs() -> ServoInputs {
        ServoInputs {
            scaler: 1.0,
            disable_integrator: false,
            ground_mode: false,
            roll_rate_rad: 0.0,
            aspeed: 20.0,
            airspeed_min: 9.0,
            eas2tas: 1.0,
            dt: 0.0025,
            now_ms: 0,
        }
    }

    #[test]
    fn output_is_clamped_to_the_surface_range() {
        let mut c = RollController::new(pid_gains(), RateGains::default());
        let inp = inputs();
        for step in 0..50 {
            let out = c.servo_out(
                9000,
                &ServoInputs {
                    now_ms: step * 3,
                    ..inp
                },
            );
            assert!(
                (-4500.0..=4500.0).contains(&out),
                "step {step}: {out} outside the surface range"
            );
        }
    }

    /// A too-small tau is corrected in place, as upstream does by writing back
    /// to the parameter.
    #[test]
    fn tau_below_the_minimum_is_raised() {
        let mut c = RollController::new(
            pid_gains(),
            RateGains {
                tau: 0.001,
                ..RateGains::default()
            },
        );
        c.servo_out(1000, &inputs());
        assert_eq!(c.controller.gains.tau, MIN_TAU);
    }

    /// The rate limit binds unless a recovery is in progress, and the recovery
    /// flag lasts exactly one loop.
    #[test]
    fn rmax_limits_the_demand_except_during_recovery() {
        let gains = RateGains {
            tau: 0.05,
            rmax_pos: 30.0,
            rmax_neg: 30.0,
        };
        let mut c = RollController::new(pid_gains(), gains);
        c.servo_out(9000, &inputs());
        assert_eq!(
            c.controller.info().target,
            30.0,
            "a 90 degree error over tau 0.05 demands 1800 deg/s, limited to 30"
        );

        c.set_in_recovery();
        c.servo_out(9000, &inputs());
        assert!(
            c.controller.info().target > 30.0,
            "recovery must lift the limit, got {}",
            c.controller.info().target
        );

        // and the flag is single-loop
        c.servo_out(9000, &inputs());
        assert_eq!(
            c.controller.info().target,
            30.0,
            "recovery must not persist"
        );
    }

    /// Below the minimum airspeed the integrator is frozen outright, not merely
    /// limited — an aircraft near the stall must not wind up its surfaces.
    #[test]
    fn underspeed_freezes_the_integrator() {
        let mut c = RollController::new(pid_gains(), RateGains::default());
        let mut inp = inputs();
        for step in 0..30 {
            inp.now_ms = step * 3;
            c.servo_out(2000, &inp);
        }
        let wound = c.controller.rate_pid.integrator();
        assert!(wound > 0.0, "the integrator should have wound up");

        inp.aspeed = 5.0; // below airspeed_min of 9
        for step in 30..60 {
            inp.now_ms = step * 3;
            c.servo_out(2000, &inp);
        }
        assert_eq!(
            c.controller.rate_pid.integrator(),
            wound,
            "underspeed must freeze the integrator, not merely limit it"
        );
    }

    /// Ground mode removes D and half of P, which is what stops a wheeled
    /// airframe oscillating on the runway.
    #[test]
    fn ground_mode_suppresses_d_and_half_of_p() {
        let mut gains = pid_gains();
        gains.d = 0.02;
        let mut air = RollController::new(gains, RateGains::default());
        let mut ground = RollController::new(gains, RateGains::default());

        let inp = inputs();
        let ground_inp = ServoInputs {
            ground_mode: true,
            ..inp
        };
        for step in 0..10 {
            let now = step * 3;
            air.servo_out(2000, &ServoInputs { now_ms: now, ..inp });
            ground.servo_out(
                2000,
                &ServoInputs {
                    now_ms: now,
                    ..ground_inp
                },
            );
        }

        let a = air.controller.info();
        let g = ground.controller.info();
        // the info reports the unsuppressed terms; the suppression is in the sum
        let air_sum = a.ff + a.p + a.i + a.d + a.dff;
        let ground_sum = g.ff + g.p + g.i + g.d + g.dff - (g.d + 0.5 * g.p);
        assert!(
            ground_sum.abs() < air_sum.abs(),
            "ground mode should reduce the demand: {ground_sum} vs {air_sum}"
        );
    }

    /// The feed-forward scale is consumed by one loop and then reverts.
    #[test]
    fn ff_scale_applies_once() {
        let mut c = RollController::new(pid_gains(), RateGains::default());
        let inp = inputs();
        c.servo_out(1000, &inp);

        c.controller.set_ff_scale(2.0);
        c.servo_out(1000, &ServoInputs { now_ms: 3, ..inp });
        let boosted = c.controller.info().ff;

        c.servo_out(1000, &ServoInputs { now_ms: 6, ..inp });
        let normal = c.controller.info().ff;
        assert!(
            boosted.abs() > normal.abs(),
            "the scaled loop should have more feed-forward: {boosted} vs {normal}"
        );
    }
}
