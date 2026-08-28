//! QuadPlane MAVLink motor-test, upstream `ArduPlane/motor_test.cpp`
//! (Plane-4.7.0) — `MAV_CMD_DO_MOTOR_TEST` start / output / stop.
//!
//! Tracked as **VT-001**. [`QuadPlane::mavlink_motor_test_start`]
//! refuses an unavailable object, an already-armed vehicle (unless a
//! test is already running), and a failed motors `motor_test_checks`.
//! A successful first start latches [`QuadPlane::motors_armed`].
//! [`QuadPlane::motor_test_output`] maps throttle type to PWM, walks a
//! multi-motor sequence (zero for 0.5×timeout between motors), and
//! [`QuadPlane::motor_test_stop`] disarms. This is not a rewrite of
//! ap-motors mixing, `setup()` frame-class, or weathervane.

use crate::QuadPlane;

/// Upstream `#define MOTOR_TEST_TIMEOUT_MS_MAX 30000`.
pub const MOTOR_TEST_TIMEOUT_MS_MAX: u32 = 30_000;

/// Max motors cycled in one start, upstream `MIN(motor_count, 8)`.
pub const MOTOR_TEST_MOTOR_COUNT_MAX: u8 = 8;

/// `RC_Channel::RC_MIN_LIMIT_PWM`.
pub const RC_MIN_LIMIT_PWM: i16 = 800;

/// `RC_Channel::RC_MAX_LIMIT_PWM`.
pub const RC_MAX_LIMIT_PWM: i16 = 2200;

/// Default motors PWM floor used when a view does not override it.
pub const MOTOR_PWM_MIN_DEFAULT: i16 = 1000;

/// Default motors PWM ceiling used when a view does not override it.
pub const MOTOR_PWM_MAX_DEFAULT: i16 = 2000;

/// Upstream `MOTOR_TEST_THROTTLE_TYPE` / `motor_test.throttle_type`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MotorTestThrottleType {
    /// `MOTOR_TEST_THROTTLE_PERCENT` — 0..100 of the motors PWM span.
    Percent = 0,
    /// `MOTOR_TEST_THROTTLE_PWM` — absolute PWM.
    Pwm = 1,
    /// `MOTOR_TEST_THROTTLE_PILOT` — `get_throttle_input()` percent.
    Pilot = 2,
}

impl MotorTestThrottleType {
    /// Inverse of the MAVLink discriminant.
    #[must_use]
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Percent),
            1 => Some(Self::Pwm),
            2 => Some(Self::Pilot),
            _ => None,
        }
    }

    /// MAVLink discriminant.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// `MAV_RESULT` for [`QuadPlane::mavlink_motor_test_start`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MavResult {
    /// `MAV_RESULT_ACCEPTED`.
    Accepted = 0,
    /// `MAV_RESULT_FAILED`.
    Failed = 4,
}

/// One `motor_test_output` tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotorTestTick {
    /// `!motor_test.running` — early return.
    Idle,
    /// Drive `motors->output_test_seq(seq, pwm)`.
    Drive {
        /// `motor_test.seq`.
        seq: u8,
        /// PWM sent to that motor.
        pwm: i16,
    },
    /// Inter-motor gap: `motors->output_min()` for 0.5×timeout.
    OutputMin,
    /// Timed out this motor; advanced `seq` and decremented `motor_count`.
    NextMotor,
    /// Test ended this tick (`motor_test_stop`).
    Stopped,
}

/// Arguments for [`QuadPlane::mavlink_motor_test_start`].
///
/// `checks_ok` is `motors->motor_test_checks(...)` (lives in
/// ap-motors; this crate does not own the mixer).
#[derive(Clone, Copy, Debug)]
pub struct MotorTestStart {
    /// `motor_seq`.
    pub motor_seq: u8,
    /// `throttle_type`.
    pub throttle_type: u8,
    /// `throttle_value`.
    pub throttle_value: u16,
    /// `timeout_sec`.
    pub timeout_sec: f32,
    /// `motor_count`.
    pub motor_count: u8,
    /// `AP_HAL::millis()` at start.
    pub now_ms: u32,
    /// `motors->motor_test_checks` succeeded.
    pub checks_ok: bool,
}

impl MotorTestStart {
    /// Single-motor 50% / 1 s test, checks passing, `now = 0`.
    #[must_use]
    pub const fn percent50() -> Self {
        Self {
            motor_seq: 1,
            throttle_type: MotorTestThrottleType::Percent as u8,
            throttle_value: 50,
            timeout_sec: 1.0,
            motor_count: 1,
            now_ms: 0,
            checks_ok: true,
        }
    }
}

/// Vehicle bits `motor_test_output` reads from motors / RC.
#[derive(Clone, Copy, Debug)]
pub struct MotorTestOutputView {
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
    /// `motors->get_pwm_output_min()`.
    pub thr_min_pwm: i16,
    /// `motors->get_pwm_output_max()`.
    pub thr_max_pwm: i16,
    /// `plane.get_throttle_input()` (0..100) for the pilot type.
    pub pilot_throttle: f32,
    /// `motors->output_test_seq` returned true.
    pub output_seq_ok: bool,
}

impl MotorTestOutputView {
    /// Default 1000..2000 PWM span, pilot idle, output-seq ok.
    #[must_use]
    pub const fn at(now_ms: u32) -> Self {
        Self {
            now_ms,
            thr_min_pwm: MOTOR_PWM_MIN_DEFAULT,
            thr_max_pwm: MOTOR_PWM_MAX_DEFAULT,
            pilot_throttle: 0.0,
            output_seq_ok: true,
        }
    }
}

/// Upstream `QuadPlane::motor_test` block.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MotorTest {
    running: bool,
    start_ms: u32,
    timeout_ms: u32,
    seq: u8,
    throttle_type: u8,
    throttle_value: u16,
    motor_count: u8,
}

impl Default for MotorTest {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorTest {
    /// Idle test block, matching the C++ in-class initialisers.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            running: false,
            start_ms: 0,
            timeout_ms: 0,
            seq: 0,
            throttle_type: 0,
            throttle_value: 0,
            motor_count: 0,
        }
    }

    /// `motor_test.running`.
    #[must_use]
    pub const fn running(&self) -> bool {
        self.running
    }

    /// `motor_test.start_ms`.
    #[must_use]
    pub const fn start_ms(&self) -> u32 {
        self.start_ms
    }

    /// `motor_test.timeout_ms`.
    #[must_use]
    pub const fn timeout_ms(&self) -> u32 {
        self.timeout_ms
    }

    /// `motor_test.seq`.
    #[must_use]
    pub const fn seq(&self) -> u8 {
        self.seq
    }

    /// `motor_test.throttle_type`.
    #[must_use]
    pub const fn throttle_type(&self) -> u8 {
        self.throttle_type
    }

    /// `motor_test.throttle_value`.
    #[must_use]
    pub const fn throttle_value(&self) -> u16 {
        self.throttle_value
    }

    /// `motor_test.motor_count`.
    #[must_use]
    pub const fn motor_count(&self) -> u8 {
        self.motor_count
    }
}

/// Cap `timeout_sec * 1000` at [`MOTOR_TEST_TIMEOUT_MS_MAX`].
///
/// Upstream `MIN(timeout_sec * 1000, MOTOR_TEST_TIMEOUT_MS_MAX)`.
#[must_use]
pub fn timeout_ms_from_sec(timeout_sec: f32) -> u32 {
    let ms = timeout_sec * 1000.0;
    if !ms.is_finite() || ms <= 0.0 {
        0
    } else if ms >= MOTOR_TEST_TIMEOUT_MS_MAX as f32 {
        MOTOR_TEST_TIMEOUT_MS_MAX
    } else {
        ms as u32
    }
}

/// PWM for a percent / PWM / pilot throttle type.
///
/// `None` is an unknown type (output then stops) or percent > 100
/// (upstream leaves `pwm = 0`, which then fails the RC-limit check).
#[must_use]
pub fn throttle_to_pwm(
    throttle_type: u8,
    throttle_value: u16,
    thr_min_pwm: i16,
    thr_max_pwm: i16,
    pilot_throttle: f32,
) -> Option<i16> {
    match MotorTestThrottleType::from_u8(throttle_type) {
        Some(MotorTestThrottleType::Percent) => {
            if throttle_value > 100 {
                Some(0)
            } else {
                Some(lerp_pwm(
                    thr_min_pwm,
                    thr_max_pwm,
                    throttle_value as f32 * 0.01,
                ))
            }
        }
        Some(MotorTestThrottleType::Pwm) => Some(throttle_value as i16),
        Some(MotorTestThrottleType::Pilot) => {
            Some(lerp_pwm(thr_min_pwm, thr_max_pwm, pilot_throttle * 0.01))
        }
        None => None,
    }
}

fn lerp_pwm(thr_min_pwm: i16, thr_max_pwm: i16, frac: f32) -> i16 {
    let span = f32::from(thr_max_pwm) - f32::from(thr_min_pwm);
    (f32::from(thr_min_pwm) + span * frac) as i16
}

/// Upstream RC-limit sanity: `pwm < RC_MIN || pwm > RC_MAX`.
#[must_use]
pub const fn pwm_in_rc_limits(pwm: i16) -> bool {
    pwm >= RC_MIN_LIMIT_PWM && pwm <= RC_MAX_LIMIT_PWM
}

impl QuadPlane {
    /// Upstream `QuadPlane::motor_test` block.
    #[must_use]
    pub const fn motor_test(&self) -> &MotorTest {
        &self.motor_test
    }

    /// `motor_test.running`.
    #[must_use]
    pub const fn motor_test_running(&self) -> bool {
        self.motor_test.running
    }

    /// Motors-armed latch written by start / stop (`motors->armed`).
    ///
    /// The full `set_armed` path (`guided_wait_takeoff` /
    /// `init_throttle_wait`) lives on [`crate::motors_output`].
    #[must_use]
    pub const fn motors_armed(&self) -> bool {
        self.motors_armed
    }

    /// Write the motors-armed latch (`motors->armed(armed)`).
    ///
    /// No-op when `!initialised`, matching `QuadPlane::set_armed`.
    pub fn set_motors_armed(&mut self, armed: bool) {
        if !self.initialised {
            return;
        }
        self.motors_armed = armed;
    }

    /// Upstream `QuadPlane::mavlink_motor_test_start`.
    ///
    /// Failed when `!available()` / motors not inited, when starting
    /// while already armed, or when `checks_ok` is false. A running
    /// test may be retargeted without the armed check.
    pub fn mavlink_motor_test_start(&mut self, req: &MotorTestStart) -> MavResult {
        if !self.available() || !self.motors_inited {
            return MavResult::Failed;
        }
        if !self.motor_test.running && self.motors_armed {
            return MavResult::Failed;
        }
        if !req.checks_ok {
            return MavResult::Failed;
        }
        if !self.motor_test.running {
            self.motor_test.running = true;
            self.set_motors_armed(true);
        }
        self.motor_test.start_ms = req.now_ms;
        self.motor_test.timeout_ms = timeout_ms_from_sec(req.timeout_sec);
        self.motor_test.seq = req.motor_seq;
        self.motor_test.throttle_type = req.throttle_type;
        self.motor_test.throttle_value = req.throttle_value;
        self.motor_test.motor_count = if req.motor_count > MOTOR_TEST_MOTOR_COUNT_MAX {
            MOTOR_TEST_MOTOR_COUNT_MAX
        } else {
            req.motor_count
        };
        MavResult::Accepted
    }

    /// Upstream `QuadPlane::motor_test_stop`.
    pub fn motor_test_stop(&mut self) {
        if !self.motor_test.running {
            return;
        }
        self.motor_test.running = false;
        self.set_motors_armed(false);
        self.motor_test.start_ms = 0;
        self.motor_test.timeout_ms = 0;
    }

    /// Upstream `QuadPlane::motor_test_output`.
    pub fn motor_test_output(&mut self, view: &MotorTestOutputView) -> MotorTestTick {
        if !self.motor_test.running {
            return MotorTestTick::Idle;
        }
        let elapsed = view.now_ms.wrapping_sub(self.motor_test.start_ms);
        if elapsed >= self.motor_test.timeout_ms {
            if self.motor_test.motor_count > 1 {
                let half_gap = (self.motor_test.timeout_ms as f32) * 1.5;
                if (elapsed as f32) < half_gap {
                    return MotorTestTick::OutputMin;
                }
                self.motor_test.seq = self.motor_test.seq.saturating_add(1);
                self.motor_test.motor_count -= 1;
                self.motor_test.start_ms = view.now_ms;
                return MotorTestTick::NextMotor;
            }
            self.motor_test_stop();
            return MotorTestTick::Stopped;
        }

        let Some(pwm) = throttle_to_pwm(
            self.motor_test.throttle_type,
            self.motor_test.throttle_value,
            view.thr_min_pwm,
            view.thr_max_pwm,
            view.pilot_throttle,
        ) else {
            self.motor_test_stop();
            return MotorTestTick::Stopped;
        };
        if !pwm_in_rc_limits(pwm) {
            self.motor_test_stop();
            return MotorTestTick::Stopped;
        }
        if !view.output_seq_ok {
            self.motor_test_stop();
            return MotorTestTick::Stopped;
        }
        MotorTestTick::Drive {
            seq: self.motor_test.seq,
            pwm,
        }
    }
}
