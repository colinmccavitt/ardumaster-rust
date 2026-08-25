//! Port of `APM_Control/AP_YawController`. Tracked as FW-017.
//!
//! Yaw is the odd axis. Roll and pitch share `AP_FW_Controller`'s rate loop;
//! yaw does not derive from it and carries its own copy, which has drifted:
//! it folds `disable_integrator` into the I-limit rather than handling it
//! separately, sets `pinfo.limit`, has no ground mode, and has no feed-forward
//! scale hook. [`YawController::rate_out`] reproduces that copy rather than
//! calling the shared loop, because the differences are behavioural.
//!
//! There are two independent outputs, and the vehicle picks between them:
//!
//! - [`YawController::servo_out`] — the sideslip damper. A hand-rolled PI-D
//!   with a high-pass filter, not an [`AcPid`] at all, driven by lateral
//!   acceleration rather than by a rate demand.
//! - [`YawController::rate_out`] — direct rate control, used when `YAW_RATE_ENABLE`
//!   is set.
//!
//! Both write the same `_last_out` member upstream, so whichever ran last
//! decides the next call's integrator limit even across the two paths. That is
//! reproduced here with a single field rather than tidied into two.

use ap_math::scalar::{constrain_value, degrees, radians, Real, GRAVITY_MSS};
use ap_pid::{AcPid, PidGains, PidInfo, Scaling};

/// The sideslip damper's gains, upstream's `_K_*` parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct YawGains {
    /// Lateral acceleration gain into the integrator, upstream `YAW2SRV_SLIP`.
    pub k_a: f32,
    /// Integrator gain, upstream `YAW2SRV_INT`.
    pub k_i: f32,
    /// Yaw damping gain, upstream `YAW2SRV_DAMP`. Also scales the integrator,
    /// so a zero here disables the whole controller.
    pub k_d: f32,
    /// Turn-coordination feed-forward, upstream `YAW2SRV_RLL`.
    pub k_ff: f32,
    /// Integrator limit in centidegrees of deflection, upstream `YAW2SRV_IMAX`,
    /// an `AP_Int16`.
    pub imax: i16,
}

impl Default for YawGains {
    /// Upstream's parameter defaults.
    fn default() -> Self {
        Self {
            k_a: 0.0,
            k_i: 0.0,
            k_d: 0.0,
            k_ff: 1.0,
            imax: 1500,
        }
    }
}

/// Upstream clamps the bank angle to +-80 degrees when upright, written as a
/// literal rather than `radians(80)`. The two agree to the bit; the literal is
/// kept so a reader comparing the two files sees the same number.
const BANK_LIMIT_RAD: f32 = 1.396_263_4;

/// Upstream's upright test, a literal pi/2.
const INVERTED_THRESHOLD_RAD: f32 = 1.570_796_4;

/// High-pass coefficient, upstream's `0.9960080f` — a 0.2 rad/s cut-off at the
/// loop rate the controller was tuned at. Upstream notes it could be
/// `1 - omega * dt` but hard-codes it, so the filter's actual cut-off moves
/// with the loop rate.
const RATE_HP_COEFF: f32 = 0.996_008;

/// Below this the damper is off entirely: the integrator is scaled by `k_d`,
/// so a smaller value would divide the integrator limit by nearly zero.
const MIN_K_D: f32 = 0.0001;

/// Gap after which upstream treats the controller as restarted, milliseconds.
const RESTART_GAP_MS: u32 = 1000;

/// Fixed-wing yaw controller, upstream `AP_YawController`.
#[derive(Debug, Clone, Copy)]
pub struct YawController {
    /// The rate PID, used only by [`YawController::rate_out`].
    pub rate_pid: AcPid,
    /// The damper's gains.
    pub gains: YawGains,

    last_t_ms: u32,
    last_out: f32,
    last_rate_hp_out: f32,
    last_rate_hp_in: f32,
    k_d_last: f32,
    integrator: f32,
    info: PidInfo,
}

impl YawController {
    /// A yaw controller with the given damper and rate-PID gains.
    #[must_use]
    pub fn new(gains: YawGains, pid_gains: PidGains) -> Self {
        Self {
            rate_pid: AcPid::new(pid_gains),
            gains,
            last_t_ms: 0,
            last_out: 0.0,
            last_rate_hp_out: 0.0,
            last_rate_hp_in: 0.0,
            k_d_last: 0.0,
            integrator: 0.0,
            info: PidInfo::default(),
        }
    }

    /// The PID info the vehicle logs.
    #[must_use]
    pub fn info(&self) -> PidInfo {
        self.info
    }

    /// True if either damping or rate control would do anything, upstream
    /// `enabled()`. The caller supplies whether rate control is enabled,
    /// since that is a vehicle parameter rather than controller state.
    #[must_use]
    pub fn enabled(&self, rate_control_enabled: bool) -> bool {
        rate_control_enabled || self.gains.k_d > 0.0
    }

    /// The damper's integrator state. Upstream keeps this private; the port
    /// exposes it because it is the only way to tell the state apart from the
    /// reported value, which they deliberately diverge (see `decay_i`).
    #[must_use]
    pub fn integrator(&self) -> f32 {
        self.integrator
    }

    /// Clear both integrators, upstream `reset_I`.
    pub fn reset_i(&mut self) {
        self.info.i = 0.0;
        self.rate_pid.reset_i();
        self.integrator = 0.0;
    }

    /// Clear the rate PID's integrator and filters, upstream `reset_rate_PID`.
    pub fn reset_rate_pid(&mut self) {
        self.rate_pid.reset_i();
        self.rate_pid.reset_filter();
    }

    /// Decay the reported integrator, upstream `decay_I`.
    ///
    /// Note this touches only the *reported* value, not the state: upstream
    /// writes `_pid_info.I *= 0.995f` and leaves `_integrator` alone, and
    /// `servo_out` recomputes the report from the state on the next call. So
    /// the decay lasts exactly until the next loop.
    pub fn decay_i(&mut self) {
        self.info.i *= 0.995;
    }

    /// Sideslip damper, upstream `get_servo_out`. Returns rudder demand in
    /// centidegrees; positive yaws the nose right.
    #[allow(
        clippy::cast_possible_truncation,
        reason = "upstream returns int32 centidegrees from a float already \
constrained to +-4500"
    )]
    pub fn servo_out(&mut self, inp: &SideslipInputs) -> i32 {
        let mut dt_ms = inp.now_ms.wrapping_sub(self.last_t_ms);
        if self.last_t_ms == 0 || dt_ms > RESTART_GAP_MS {
            dt_ms = 0;
            // Only the REPORTED integrator is cleared here. `integrator`, the
            // state that actually reaches the output, survives the gap and is
            // recomputed into `info.i` at the end of this same call -- so this
            // line is observable only on the early return below, when damping
            // is disabled. Upstream's behaviour, reproduced deliberately; see
            // the note on `d_yaw_gap_clears_the_report_not_the_state`.
            self.info.i = 0.0;
        }
        self.last_t_ms = inp.now_ms;

        let aspd_min = inp.airspeed_min.max(1);
        let delta_time = dt_ms as f32 * 0.001;

        // Yaw rate needed to hold height through a level coordinated turn.
        let mut bank_angle = inp.roll_rad;
        if Real::abs(bank_angle) < INVERTED_THRESHOLD_RAD {
            bank_angle = constrain_value(bank_angle, -BANK_LIMIT_RAD, BANK_LIMIT_RAD);
        }
        // Note there is no inverted branch, unlike the pitch controller: past
        // 90 degrees the bank angle is used unclamped.

        let aspeed = inp
            .airspeed_eas
            .unwrap_or_else(|| 0.5 * (f32::from(aspd_min) + f32::from(inp.airspeed_max)));
        let rate_offset =
            GRAVITY_MSS / aspeed.max(f32::from(aspd_min)) * Real::sin(bank_angle) * self.gains.k_ff;

        // The EKF's accelerometer bias estimate is removed before use.
        let accel_y = inp.accel_y - inp.accel_bias_y;

        // Take out the steady turn component so what remains is the rate
        // relative to the turn requirement, then high-pass it to wash out the
        // steady-state error left by bias in `rate_offset`.
        let rate_hp_in = degrees(inp.yaw_rate_rad - rate_offset);
        let rate_hp_out = RATE_HP_COEFF * self.last_rate_hp_out + rate_hp_in - self.last_rate_hp_in;
        self.last_rate_hp_out = rate_hp_out;
        self.last_rate_hp_in = rate_hp_in;

        let integ_in = -self.gains.k_i * (self.gains.k_a * accel_y + rate_hp_out);

        // Integrate only above the minimum airspeed, and only in the direction
        // that unwinds the surface when the demand is already saturated.
        // Stabilise mode disables it outright, as does a zero damping gain --
        // the integrator is scaled by k_d, so it would wind up unopposed.
        if !inp.disable_integrator && self.gains.k_d > 0.0 {
            if aspeed > f32::from(aspd_min) {
                let step = integ_in * delta_time;
                if self.last_out < -45.0 {
                    self.integrator += step.max(0.0);
                } else if self.last_out > 45.0 {
                    self.integrator += step.min(0.0);
                } else {
                    self.integrator += step;
                }
            }
        } else {
            self.integrator = 0.0;
        }

        if self.gains.k_d < MIN_K_D {
            // damping off, and the integrator is scaled by it, so there is
            // nothing to output
            return 0;
        }

        let int_lim_scaled =
            f32::from(self.gains.imax) * 0.01 / (self.gains.k_d * inp.scaler * inp.scaler);
        self.integrator = constrain_value(self.integrator, -int_lim_scaled, int_lim_scaled);

        // Raising the damping gain in flight would otherwise turn a stored
        // integrator into a control transient, so the stored value is rescaled
        // to keep its contribution to the output constant.
        if self.gains.k_d > self.k_d_last && self.gains.k_d > 0.0 {
            // same rounding as upstream: the division is evaluated first
            // either way, and float multiplication commutes
            self.integrator *= self.k_d_last / self.gains.k_d;
        }
        self.k_d_last = self.gains.k_d;

        // Scaled by inverse dynamic pressure. `last_out` is saved before the
        // limiter so that the next call's saturation test sees the demand that
        // was actually asked for.
        self.info.i = self.gains.k_d * self.integrator * inp.scaler * inp.scaler;
        self.info.d = self.gains.k_d * -rate_hp_out * inp.scaler * inp.scaler;
        self.last_out = self.info.i + self.info.d;

        constrain_value(self.last_out * 100.0, -4500.0, 4500.0) as i32
    }

    /// Direct yaw rate control, upstream `get_rate_out`. Returns rudder demand
    /// in centidegrees.
    ///
    /// This is `AP_FW_Controller::_get_rate_out` with three differences that
    /// are behavioural rather than cosmetic, so it is not routed through
    /// `FwController`: `disable_integrator` also forces the I-limit, there is
    /// no ground mode, and there is no feed-forward scale hook.
    pub fn rate_out(&mut self, inp: &YawRateInputs) -> f32 {
        let aspeed = inp.airspeed_eas.unwrap_or(0.0);
        let mut limit_i = Real::abs(self.last_out) >= 45.0 || inp.disable_integrator;
        let old_i = self.rate_pid.integrator();

        let underspeed = aspeed <= f32::from(inp.airspeed_min);
        if underspeed {
            limit_i = true;
        }

        self.rate_pid.update_all(
            radians(inp.desired_rate_deg) * inp.scaler * inp.scaler,
            inp.yaw_rate_rad * inp.scaler * inp.scaler,
            inp.dt,
            limit_i,
            Scaling::default(),
            inp.now_ms,
        );

        if underspeed {
            self.rate_pid.set_integrator(old_i);
        }

        let pid = self.rate_pid.info();
        let ff = degrees(pid.ff / (inp.scaler * inp.eas2tas));
        let dff = degrees(pid.dff / (inp.scaler * inp.eas2tas));

        if inp.disable_integrator {
            self.rate_pid.reset_i();
        }

        let deg_scale = degrees(1.0f32);
        let mut info = self.rate_pid.info();
        info.ff = ff;
        info.p *= deg_scale;
        info.i *= deg_scale;
        info.d *= deg_scale;
        info.dff = dff;
        info.limit = limit_i;
        info.target = inp.desired_rate_deg;
        info.actual = degrees(inp.yaw_rate_rad);
        self.info = info;

        let out = info.ff + info.p + info.i + info.d + info.dff;
        self.last_out = out;

        constrain_value(out * 100.0, -4500.0, 4500.0)
    }
}

/// What the sideslip damper reads from the vehicle each loop.
#[derive(Debug, Clone, Copy)]
pub struct SideslipInputs {
    /// Airspeed scaling factor.
    pub scaler: f32,
    /// Stabilise mode: do not integrate against the pilot's own inputs.
    pub disable_integrator: bool,
    /// Milliseconds since boot. The damper measures its own loop period from
    /// this rather than being told one.
    pub now_ms: u32,
    /// Roll attitude, radians. Upstream `ahrs.get_roll_rad()`.
    pub roll_rad: f32,
    /// Equivalent airspeed, m/s, or `None` when the AHRS has no estimate.
    pub airspeed_eas: Option<f32>,
    /// Minimum airspeed parameter, m/s. Upstream `aparm.airspeed_min`.
    pub airspeed_min: i16,
    /// Maximum airspeed parameter, m/s, read only for the no-airspeed fallback.
    pub airspeed_max: i16,
    /// Measured yaw rate, radians/s. Upstream `ahrs.get_gyro().z`.
    pub yaw_rate_rad: f32,
    /// Lateral acceleration, m/s2. Upstream `AP::ins().get_accel().y`.
    pub accel_y: f32,
    /// The EKF's lateral accelerometer bias estimate, m/s2. Upstream
    /// `ahrs.get_accel_bias().y`, subtracted from `accel_y`.
    pub accel_bias_y: f32,
}

/// What direct yaw rate control reads from the vehicle each loop.
#[derive(Debug, Clone, Copy)]
pub struct YawRateInputs {
    /// Demanded yaw rate, deg/s.
    pub desired_rate_deg: f32,
    /// Airspeed scaling factor.
    pub scaler: f32,
    /// Hold the integrator at zero. Unlike roll and pitch, this also forces
    /// the I-limit for this call.
    pub disable_integrator: bool,
    /// Measured yaw rate, radians/s.
    pub yaw_rate_rad: f32,
    /// Equivalent airspeed, m/s, or `None` when the AHRS has no estimate.
    /// Upstream uses zero when absent, which then trips the underspeed freeze.
    pub airspeed_eas: Option<f32>,
    /// Minimum airspeed parameter, m/s.
    pub airspeed_min: i16,
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

    /// Upstream's `AC_PID rate_pid{0.04, 0.15, 0, 0.15, 0.666, 3, 0, 12, 150, 1}`.
    fn rate_gains() -> PidGains {
        PidGains {
            p: 0.04,
            i: 0.15,
            d: 0.0,
            ff: 0.15,
            dff: 0.0,
            imax: 0.666,
            pdmax: 0.0,
            filt_t_hz: 3.0,
            filt_e_hz: 0.0,
            filt_d_hz: 12.0,
            srmax: 150.0,
            srtau: 1.0,
        }
    }

    fn damper_gains() -> YawGains {
        YawGains {
            k_a: 1.0,
            k_i: 0.05,
            k_d: 0.1,
            k_ff: 1.0,
            imax: 1500,
        }
    }

    fn sideslip() -> SideslipInputs {
        SideslipInputs {
            scaler: 1.0,
            disable_integrator: false,
            now_ms: 0,
            roll_rad: 0.0,
            airspeed_eas: Some(20.0),
            airspeed_min: 10,
            airspeed_max: 30,
            yaw_rate_rad: 0.0,
            accel_y: 0.0,
            accel_bias_y: 0.0,
        }
    }

    fn controller() -> YawController {
        YawController::new(damper_gains(), rate_gains())
    }

    /// The first call measures a zero loop period, so nothing integrates
    /// however large the acceleration.
    #[test]
    fn the_first_call_integrates_nothing() {
        let mut c = controller();
        let mut inp = sideslip();
        inp.now_ms = 5000;
        inp.accel_y = 4.0;
        c.servo_out(&inp);
        assert_eq!(c.integrator(), 0.0);
    }

    /// Lateral acceleration to the right drives the integrator negative, so
    /// the rudder is commanded to yaw the nose the other way.
    #[test]
    fn lateral_acceleration_is_opposed() {
        let mut c = controller();
        let mut inp = sideslip();
        c.servo_out(&inp); // establishes the time base

        inp.accel_y = 2.0;
        for step in 1..20 {
            inp.now_ms = step * 20;
            c.servo_out(&inp);
        }
        assert!(c.integrator() < 0.0, "integrator {}", c.integrator());
        assert!(c.info().i < 0.0);
    }

    /// Below the minimum airspeed the integrator is frozen outright.
    #[test]
    fn below_the_minimum_airspeed_the_integrator_is_frozen() {
        let mut c = controller();
        let mut inp = sideslip();
        inp.accel_y = 2.0;
        c.servo_out(&inp);
        for step in 1..20 {
            inp.now_ms = step * 20;
            c.servo_out(&inp);
        }
        let wound = c.integrator();
        assert!(wound < 0.0);

        inp.airspeed_eas = Some(9.0); // below airspeed_min of 10
        for step in 20..40 {
            inp.now_ms = step * 20;
            c.servo_out(&inp);
        }
        assert_eq!(c.integrator(), wound);
    }

    /// A damping gain below the threshold disables the controller: the
    /// integrator is scaled by that gain, so there is nothing to output.
    #[test]
    fn damping_below_the_threshold_outputs_nothing() {
        let mut c = controller();
        c.gains.k_d = 0.00005;
        let mut inp = sideslip();
        inp.accel_y = 2.0;
        c.servo_out(&inp);
        for step in 1..20 {
            inp.now_ms = step * 20;
            assert_eq!(c.servo_out(&inp), 0);
        }
    }

    /// Raising the damping gain in flight must not produce a control
    /// transient: the stored integrator is rescaled so its contribution to the
    /// output is unchanged across the gain change.
    #[test]
    fn raising_the_damping_gain_preserves_the_integrator_contribution() {
        let mut c = controller();
        let mut inp = sideslip();
        inp.accel_y = 2.0;
        c.servo_out(&inp);
        for step in 1..30 {
            inp.now_ms = step * 20;
            c.servo_out(&inp);
        }
        let before = c.info().i;
        assert!(before < 0.0, "expected a wound integrator, got {before}");

        // stop winding so the rescale is the only thing acting
        c.gains.k_i = 0.0;
        c.gains.k_d = 0.2;
        inp.now_ms = 600;
        c.servo_out(&inp);

        assert!(
            (c.info().i - before).abs() < 1e-6,
            "doubling the damping gain moved the integrator contribution from \
             {before} to {}",
            c.info().i
        );
    }

    /// After a gap longer than a second upstream clears the REPORTED
    /// integrator but leaves the state that reaches the output. The report is
    /// then recomputed from that surviving state within the same call, so the
    /// clear is invisible here -- it is observable only when damping is
    /// disabled and the call returns early.
    ///
    /// Reproduced deliberately rather than "fixed": dropping the state after a
    /// gap is a flight-behaviour change, not a correctness cleanup.
    #[test]
    fn a_long_gap_clears_the_report_but_not_the_state() {
        let mut c = controller();
        let mut inp = sideslip();
        inp.accel_y = 2.0;
        c.servo_out(&inp);
        for step in 1..30 {
            inp.now_ms = step * 20;
            c.servo_out(&inp);
        }
        let wound = c.integrator();
        assert!(wound < 0.0);

        inp.now_ms = 30_000; // a 29 second gap
        c.servo_out(&inp);

        assert_eq!(c.integrator(), wound, "the state must survive the gap");
        assert!(c.info().i < 0.0, "and the report is recomputed from it");
    }

    /// Yaw folds `disable_integrator` into the I-limit, where roll and pitch
    /// handle it only by resetting afterwards. The limit is reported.
    #[test]
    fn disable_integrator_also_forces_the_limit() {
        let mut c = controller();
        let inp = YawRateInputs {
            desired_rate_deg: 10.0,
            scaler: 1.0,
            disable_integrator: true,
            yaw_rate_rad: 0.0,
            airspeed_eas: Some(20.0),
            airspeed_min: 10,
            eas2tas: 1.0,
            dt: 0.02,
            now_ms: 20,
        };
        c.rate_out(&inp);
        assert!(c.info().limit, "disable_integrator must set the I-limit");
        assert_eq!(c.rate_pid.integrator(), 0.0);
    }

    /// The high-pass washes out a steady rate error, which is its whole
    /// purpose: bias in the turn-coordination offset must not accumulate.
    ///
    /// Held to the coefficient rather than to a decay rate in seconds, because
    /// upstream applies it per call: the cut-off it produces moves with the
    /// loop rate, which upstream's own comment acknowledges by noting that
    /// `0.9960080f` could have been `1 - omega * dt` and is not.
    #[test]
    fn a_steady_rate_error_is_washed_out() {
        let mut c = controller();
        let mut inp = sideslip();
        inp.yaw_rate_rad = 0.2;
        c.servo_out(&inp);

        // With the input held constant the recurrence collapses to a plain
        // geometric decay, so every step is exactly the coefficient.
        inp.now_ms = 20;
        c.servo_out(&inp);
        let first = c.info().d;
        assert!(first != 0.0);

        inp.now_ms = 40;
        c.servo_out(&inp);
        let ratio = c.info().d / first;
        assert!(
            (ratio - RATE_HP_COEFF).abs() < 1e-5,
            "expected each step to decay by {RATE_HP_COEFF}, got {ratio}"
        );

        // and over a long enough run it does actually wash out
        for step in 3..1200 {
            inp.now_ms = step * 20;
            c.servo_out(&inp);
        }
        assert!(
            c.info().d.abs() < first.abs() * 0.01,
            "after 1200 calls the damping term should be negligible, is {}",
            c.info().d
        );
    }
}
