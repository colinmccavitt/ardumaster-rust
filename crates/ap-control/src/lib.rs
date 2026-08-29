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

pub mod attitude_controller;
pub mod attitude_error;
pub mod attitude_kinematics;
pub mod pos_control_ne;
pub mod rate_loop;
pub mod multi_var_info;
pub mod pos_control_var_info;
mod steer;
pub mod throttle_mix;
mod yaw;

pub use steer::{SteerController, SteerGains, SteerInputs};
pub use yaw::{SideslipInputs, YawController, YawGains, YawRateInputs};

use ap_math::scalar::{constrain_value, degrees, radians, Real, GRAVITY_MSS};
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
    pub fn servo_out(&mut self, angle_err_cd: i32, inp: &RollInputs) -> f32 {
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

        // Upstream's roll `get_airspeed()` falls back to zero, which then trips
        // `is_underspeed` and freezes the integrator -- the intended behaviour
        // when there is no airspeed estimate at all.
        let aspeed = inp.airspeed_eas.unwrap_or(0.0);

        self.controller.rate_out(&RateInputs {
            desired_rate_deg: desired_rate,
            scaler: inp.scaler,
            disable_integrator: inp.disable_integrator,
            aspeed,
            ground_mode: inp.ground_mode,
            measured_rate_rad: inp.roll_rate_rad,
            underspeed: aspeed <= f32::from(inp.airspeed_min),
            eas2tas: inp.eas2tas,
            dt: inp.dt,
            now_ms: inp.now_ms,
        })
    }
}

/// What the roll controller needs from the vehicle each loop.
#[derive(Debug, Clone, Copy)]
pub struct RollInputs {
    /// Airspeed scaling factor.
    pub scaler: f32,
    /// Hold the integrator at zero.
    pub disable_integrator: bool,
    /// On the ground: suppress D and half of P.
    pub ground_mode: bool,
    /// Measured roll rate, radians/s. Upstream reads `ahrs.get_gyro().x`.
    pub roll_rate_rad: f32,
    /// Equivalent airspeed, m/s, or `None` when the AHRS has no estimate.
    /// Upstream's `airspeed_EAS(&aspeed)` returning false; roll then uses zero.
    pub airspeed_eas: Option<f32>,
    /// Minimum airspeed parameter, m/s. Upstream `aparm.airspeed_min`, an
    /// `AP_Int16`.
    pub airspeed_min: i16,
    /// Equivalent-to-true airspeed ratio.
    pub eas2tas: f32,
    /// Loop period, seconds.
    pub dt: f32,
    /// Milliseconds since boot.
    pub now_ms: u32,
}

/// Fixed-wing pitch controller, upstream `AP_PitchController`.
///
/// The rate loop is shared with roll. What is specific to pitch is the path
/// from angle error to demanded rate: a turn-coordination offset, an inverted
/// flight case, and a taper that gives up on pitch once the aircraft is rolled
/// past its own roll limit.
#[derive(Debug, Clone, Copy)]
pub struct PitchController {
    /// The shared rate loop.
    pub controller: FwController,
    /// Turn-coordination gain, upstream `PTCH_RLL_FF`.
    pub roll_ff: f32,
}

/// Above this pitch attitude, upstream stops applying turn coordination:
/// the geometry the offset assumes no longer holds. Centidegrees.
const COORDINATION_PITCH_LIMIT_CD: i32 = 7000;

/// Roll attitude at which the pitch demand has been tapered to zero,
/// centidegrees. Upstream hard-codes 90 degrees.
const PITCH_GIVEUP_ROLL_CD: f32 = 9000.0;

impl PitchController {
    /// A pitch controller with the given gains.
    #[must_use]
    pub fn new(pid_gains: PidGains, gains: RateGains, roll_ff: f32) -> Self {
        Self {
            controller: FwController::new(pid_gains, gains),
            roll_ff,
        }
    }

    /// Pitch rate offset needed to hold height through a coordinated turn,
    /// deg/s. Upstream `_get_coordination_rate_offset`.
    ///
    /// Returns the offset and whether the aircraft is inverted, which the
    /// caller needs as well: upstream passes it back through an out-parameter.
    fn coordination_rate_offset(&self, aspeed: f32, inp: &PitchInputs) -> (f32, bool) {
        let mut bank_angle = inp.roll_rad;

        // Right way up, the bank angle is clamped to +-80 degrees so the
        // tangent stays finite. Inverted, it is clamped away from 90 degrees
        // from the other side.
        let inverted = if bank_angle.abs() < radians(90.0) {
            bank_angle = constrain_value(bank_angle, -radians(80.0), radians(80.0));
            false
        } else {
            bank_angle = if bank_angle > 0.0 {
                constrain_value(bank_angle, radians(100.0), radians(180.0))
            } else {
                constrain_value(bank_angle, -radians(180.0), -radians(100.0))
            };
            true
        };

        let mut rate_offset = if inp.pitch_sensor_cd.abs() > COORDINATION_PITCH_LIMIT_CD {
            // At extreme pitch the level-turn geometry does not hold.
            0.0
        } else {
            // Upstream takes an integer maximum against 1 here, because
            // airspeed_min is an AP_Int16; the guard is against dividing by
            // zero when the parameter is unset.
            let floor = f32::from(inp.airspeed_min.max(1));
            let speed = (aspeed * inp.eas2tas).max(floor);
            // Called through the trait rather than as methods. Under cfg(test)
            // the harness links std, whose inherent f32::cos/tan/sin shadow the
            // trait -- so the tests would exercise glibc while the firmware
            // no_std build exercises libm, and the two disagree by an ulp.
            Real::cos(inp.pitch_rad)
                * degrees(GRAVITY_MSS / speed * Real::tan(bank_angle) * Real::sin(bank_angle)).abs()
                * self.roll_ff
        };

        if inverted {
            rate_offset = -rate_offset;
        }
        (rate_offset, inverted)
    }

    /// Angle error in to elevator demand out, upstream `get_servo_out`.
    ///
    /// `angle_err_cd` is the pitch angle error in centidegrees, positive nose
    /// up. Returns an elevator demand in centidegrees.
    pub fn servo_out(&mut self, angle_err_cd: i32, inp: &PitchInputs) -> f32 {
        // Upstream writes the clamp back into the parameter, so a too-small
        // configured tau is corrected permanently rather than per call.
        if self.controller.gains.tau < MIN_TAU {
            self.controller.gains.tau = MIN_TAU;
        }

        // Upstream's pitch `get_airspeed()` falls back to the midpoint of the
        // configured airspeed range rather than to zero, because the turn
        // coordination offset divides by it.
        let aspeed = inp
            .airspeed_eas
            .unwrap_or_else(|| 0.5 * (f32::from(inp.airspeed_min) + f32::from(inp.airspeed_max)));

        let (rate_offset, inverted) = self.coordination_rate_offset(aspeed, inp);

        #[allow(
            clippy::cast_precision_loss,
            reason = "matches upstream's `angle_err * 0.01` on an int32; the \
values reaching here are bounded by the attitude range"
        )]
        let angle_err_deg = angle_err_cd as f32 * 0.01;
        let mut desired_rate = angle_err_deg / self.controller.gains.tau;

        if inverted {
            // Inverted, the sense of the angle error flips but the turn
            // coordination offset does not: it was already negated for it.
            // Upstream deliberately skips the rate limit here, because the
            // limits are tuned upright and inverted flight commonly needs more.
            desired_rate = -desired_rate + rate_offset;
        } else {
            desired_rate += rate_offset;
            let gains = self.controller.gains;
            if gains.rmax_neg != 0.0 && desired_rate < -gains.rmax_neg {
                desired_rate = -gains.rmax_neg;
            } else if gains.rmax_pos != 0.0 && desired_rate > gains.rmax_pos {
                desired_rate = gains.rmax_pos;
            }
        }

        // Past the configured roll limit, getting the wings back is worth more
        // than holding pitch: elevator is ineffective at large bank angles and
        // the earth-frame yaw it induces works against the recovery. The demand
        // is tapered linearly to zero at 90 degrees of roll.
        #[allow(
            clippy::cast_precision_loss,
            reason = "an attitude in centidegrees is far inside f32's exact \
integer range"
        )]
        let mut roll_wrapped = inp.roll_sensor_cd.abs() as f32;
        if roll_wrapped > PITCH_GIVEUP_ROLL_CD {
            roll_wrapped = 18000.0 - roll_wrapped;
        }
        let roll_limit_margin = (inp.roll_limit_deg * 100.0 + 500.0).min(8500.0);
        if roll_wrapped > roll_limit_margin
            && inp.pitch_sensor_cd.abs() < COORDINATION_PITCH_LIMIT_CD
        {
            let roll_prop =
                (roll_wrapped - roll_limit_margin) / (PITCH_GIVEUP_ROLL_CD - roll_limit_margin);
            desired_rate *= 1.0 - roll_prop;
        }

        self.controller.rate_out(&RateInputs {
            desired_rate_deg: desired_rate,
            scaler: inp.scaler,
            disable_integrator: inp.disable_integrator,
            aspeed,
            ground_mode: inp.ground_mode,
            measured_rate_rad: inp.pitch_rate_rad,
            // Pitch tolerates half the minimum airspeed before it calls itself
            // underspeed, where roll uses the whole of it.
            underspeed: aspeed <= 0.5 * f32::from(inp.airspeed_min),
            eas2tas: inp.eas2tas,
            dt: inp.dt,
            now_ms: inp.now_ms,
        })
    }
}

/// What the pitch controller needs from the vehicle each loop.
///
/// Larger than the roll equivalent because turn coordination and the roll-limit
/// taper both read the current attitude.
#[derive(Debug, Clone, Copy)]
pub struct PitchInputs {
    /// Airspeed scaling factor.
    pub scaler: f32,
    /// Hold the integrator at zero.
    pub disable_integrator: bool,
    /// On the ground: suppress D and half of P.
    pub ground_mode: bool,
    /// Measured pitch rate, radians/s. Upstream reads `ahrs.get_gyro().y`.
    pub pitch_rate_rad: f32,
    /// Equivalent airspeed, m/s, or `None` when the AHRS has no estimate.
    pub airspeed_eas: Option<f32>,
    /// Minimum airspeed parameter, m/s. Upstream `aparm.airspeed_min`.
    pub airspeed_min: i16,
    /// Maximum airspeed parameter, m/s. Upstream `aparm.airspeed_max`, read
    /// only for the no-airspeed fallback.
    pub airspeed_max: i16,
    /// Configured roll limit, degrees. Upstream `aparm.roll_limit`.
    pub roll_limit_deg: f32,
    /// Roll attitude, radians. Upstream `ahrs.get_roll_rad()`.
    pub roll_rad: f32,
    /// Pitch attitude, radians. Upstream `ahrs.get_pitch_rad()`.
    pub pitch_rad: f32,
    /// Roll attitude, centidegrees. Upstream `ahrs.roll_sensor`.
    pub roll_sensor_cd: i32,
    /// Pitch attitude, centidegrees. Upstream `ahrs.pitch_sensor`.
    pub pitch_sensor_cd: i32,
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

    fn inputs() -> RollInputs {
        RollInputs {
            scaler: 1.0,
            disable_integrator: false,
            ground_mode: false,
            roll_rate_rad: 0.0,
            airspeed_eas: Some(20.0),
            airspeed_min: 9,
            eas2tas: 1.0,
            dt: 0.0025,
            now_ms: 0,
        }
    }

    fn pitch_inputs() -> PitchInputs {
        PitchInputs {
            scaler: 1.0,
            disable_integrator: false,
            ground_mode: false,
            pitch_rate_rad: 0.0,
            airspeed_eas: Some(20.0),
            airspeed_min: 10,
            airspeed_max: 30,
            roll_limit_deg: 45.0,
            roll_rad: 0.0,
            pitch_rad: 0.0,
            roll_sensor_cd: 0,
            pitch_sensor_cd: 0,
            eas2tas: 1.0,
            dt: 0.02,
            now_ms: 0,
        }
    }

    fn pitch_gains() -> RateGains {
        RateGains {
            tau: 0.5,
            rmax_pos: 90.0,
            rmax_neg: 90.0,
        }
    }

    /// Upright and banked, the controller asks for nose-up rate to hold height
    /// through the turn. At 45 degrees of bank and 20 m/s:
    ///
    ///   degrees(9.80665 / 20 * tan(45) * sin(45)) = 19.866 deg/s
    #[test]
    fn turn_coordination_demands_nose_up_rate_when_banked() {
        let mut c = PitchController::new(pid_gains(), pitch_gains(), 1.0);
        let mut inp = pitch_inputs();
        inp.roll_rad = core::f32::consts::FRAC_PI_4;
        inp.roll_sensor_cd = 4500; // inside the roll limit margin, so no taper

        c.servo_out(0, &inp);
        let demanded = c.controller.info().target;
        assert!(
            (demanded - 19.866).abs() < 0.01,
            "expected 19.866 deg/s of coordination offset, got {demanded}"
        );
    }

    /// Inverted, the sense of the angle error flips but the coordination offset
    /// keeps its own sign. At 170 degrees of bank and 20 m/s the offset is
    /// -0.860 deg/s, and a 10 degree error over a 0.5 s time constant that would
    /// upright demand +20 becomes -20.
    #[test]
    fn inverted_flight_flips_the_error_but_not_the_offset() {
        let mut c = PitchController::new(pid_gains(), pitch_gains(), 1.0);
        let mut inp = pitch_inputs();
        inp.roll_rad = radians(170.0);
        inp.roll_sensor_cd = 17000; // wraps to 1000, inside the margin

        c.servo_out(1000, &inp);
        let demanded = c.controller.info().target;
        assert!(
            (demanded - -20.860).abs() < 0.01,
            "expected -20.860 deg/s inverted, got {demanded}"
        );
    }

    /// Past 70 degrees of pitch the level-turn geometry the offset assumes no
    /// longer holds, so upstream drops the offset entirely. The same threshold
    /// also switches off the roll-limit taper, so the demand is the bare
    /// angle-error term.
    #[test]
    fn extreme_pitch_suppresses_turn_coordination() {
        let mut c = PitchController::new(pid_gains(), pitch_gains(), 1.0);
        let mut inp = pitch_inputs();
        inp.roll_rad = core::f32::consts::FRAC_PI_4;
        inp.pitch_sensor_cd = 8000;
        inp.roll_sensor_cd = 8000; // beyond the margin, but the taper is off too

        c.servo_out(1000, &inp);
        assert_eq!(
            c.controller.info().target,
            20.0,
            "beyond 70 degrees of pitch the demand should be angle error over tau alone"
        );
    }

    /// Rolled to 90 degrees the elevator cannot produce pitch and the yaw it
    /// induces fights the recovery, so the demand is tapered to nothing. With a
    /// 45 degree roll limit the taper starts at 50 degrees and reaches zero at
    /// 90.
    #[test]
    fn roll_past_the_limit_tapers_the_demand_to_zero() {
        // roll_ff is zero so the taper is isolated. At these bank angles the
        // coordination offset is far larger than the angle-error term -- 72.5
        // deg/s at 70 degrees -- and it saturates the rate limit, so the taper
        // would be scaling rmax rather than the demand under test.
        let mut c = PitchController::new(pid_gains(), pitch_gains(), 0.0);
        let mut inp = pitch_inputs();
        inp.roll_sensor_cd = 9000;
        inp.roll_rad = core::f32::consts::FRAC_PI_2;

        c.servo_out(1000, &inp);
        assert_eq!(c.controller.info().target, 0.0);

        // halfway through the taper, half the demand survives
        inp.roll_sensor_cd = 7000;
        inp.roll_rad = radians(70.0);
        c.servo_out(1000, &inp);
        let demanded = c.controller.info().target;
        assert!(
            (demanded - 10.0).abs() < 0.01,
            "expected half the demand at 70 degrees of roll, got {demanded}"
        );
    }

    /// Pitch calls itself underspeed at half the minimum airspeed, where roll
    /// uses the whole of it. The difference is deliberate upstream: freezing
    /// the elevator integrator early costs height control just when it is
    /// needed most.
    #[test]
    fn pitch_tolerates_half_the_minimum_airspeed() {
        let mut c = PitchController::new(pid_gains(), pitch_gains(), 0.0);
        let mut inp = pitch_inputs();

        // 6 m/s is below airspeed_min of 10, but above half of it
        inp.airspeed_eas = Some(6.0);
        for step in 0..30 {
            inp.now_ms = step * 20;
            c.servo_out(2000, &inp);
        }
        let wound = c.controller.rate_pid.integrator();
        assert!(wound > 0.0, "the integrator should still wind up at 6 m/s");

        inp.airspeed_eas = Some(4.0);
        for step in 30..60 {
            inp.now_ms = step * 20;
            c.servo_out(2000, &inp);
        }
        assert_eq!(
            c.controller.rate_pid.integrator(),
            wound,
            "below half the minimum airspeed the integrator must freeze"
        );
    }

    /// With no airspeed estimate pitch falls back to the midpoint of the
    /// configured range, not to zero as roll does: the coordination offset
    /// divides by airspeed, so zero would be useless there.
    #[test]
    fn absent_airspeed_falls_back_to_the_midpoint_of_the_range() {
        let mut inp = pitch_inputs();
        inp.roll_rad = core::f32::consts::FRAC_PI_4;
        inp.roll_sensor_cd = 4500;

        let mut fallback = PitchController::new(pid_gains(), pitch_gains(), 1.0);
        inp.airspeed_eas = None;
        fallback.servo_out(0, &inp);

        // airspeed_min 10 and airspeed_max 30 give a midpoint of 20
        let mut explicit = PitchController::new(pid_gains(), pitch_gains(), 1.0);
        inp.airspeed_eas = Some(20.0);
        explicit.servo_out(0, &inp);

        assert_eq!(
            fallback.controller.info().target,
            explicit.controller.info().target
        );
    }

    #[test]
    fn output_is_clamped_to_the_surface_range() {
        let mut c = RollController::new(pid_gains(), RateGains::default());
        let inp = inputs();
        for step in 0..50 {
            let out = c.servo_out(
                9000,
                &RollInputs {
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

        inp.airspeed_eas = Some(5.0); // below airspeed_min of 9
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
        let ground_inp = RollInputs {
            ground_mode: true,
            ..inp
        };
        for step in 0..10 {
            let now = step * 3;
            air.servo_out(2000, &RollInputs { now_ms: now, ..inp });
            ground.servo_out(
                2000,
                &RollInputs {
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
        c.servo_out(1000, &RollInputs { now_ms: 3, ..inp });
        let boosted = c.controller.info().ff;

        c.servo_out(1000, &RollInputs { now_ms: 6, ..inp });
        let normal = c.controller.info().ff;
        assert!(
            boosted.abs() > normal.abs(),
            "the scaled loop should have more feed-forward: {boosted} vs {normal}"
        );
    }
}
