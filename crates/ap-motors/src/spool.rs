//! The spool state machine, upstream `AP_MotorsMulticopter::output_logic`.
//! COP-004.
//!
//! A multirotor cannot go from stopped to flying in one step. Between "motors
//! off" and "throttle follows the pilot" there is a staged sequence: let the
//! ESCs see a PWM signal and start up, bring the rotors to a visible idle,
//! hold there long enough for pre-takeoff checks, then raise a throttle
//! ceiling smoothly rather than letting the controller command full authority
//! the instant it is allowed to. This is that sequence.
//!
//! # The two ramps
//!
//! Two separate ramps run, and they matter at different stages.
//!
//! `spin_up_ratio` is the *spin* ramp: 0 to 1 across the range between stopped
//! and `SPIN_MIN`. It moves while the rotors are coming up to idle, and is
//! pinned at 1.0 from `SPOOLING_UP` onward.
//!
//! `throttle_thrust_max` is the *authority* ramp: the ceiling on commanded
//! throttle. It stays at zero until the spin ramp is finished and the checks
//! have cleared, then rises to the current-limited maximum.
//!
//! Keeping them apart is what lets the aircraft sit at idle with attitude
//! control running but no thrust authority.
//!
//! # Explicit context
//!
//! Per ADR-0004 there is no singleton. What upstream reads off the motors
//! object arrives here as [`SpoolInputs`], and the tunables it reads off
//! parameters arrive as [`SpoolParams`] — by `&mut`, because upstream really
//! does write back to one of them (see [`SpoolParams::spool_up_time`]).

use ap_math::scalar::{is_positive, is_zero};

/// Smallest spool time the machine will run with, upstream's
/// `minimum_spool_time`.
///
/// The ramps are `dt / time`, so a zero would divide by zero and a very small
/// one would step the ramp far enough to be a jump. 0.05 s is short enough to
/// read as instant and long enough to stay a ramp.
const MINIMUM_SPOOL_TIME: f32 = 0.05;

/// Where the motors actually are.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SpoolState {
    /// Motors stopped, PWM possibly disabled, no attitude authority.
    #[default]
    ShutDown = 0,
    /// Motors at or near idle. Attitude control runs, thrust does not.
    GroundIdle = 1,
    /// Throttle ceiling rising.
    SpoolingUp = 2,
    /// Throttle follows demand, bounded only by current limiting.
    ThrottleUnlimited = 3,
    /// Throttle ceiling falling.
    SpoolingDown = 4,
}

/// Where the vehicle code wants the motors to be.
///
/// Note this has three values to the actual state's five: the two spooling
/// states are transitions, not destinations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DesiredSpoolState {
    /// Stop the motors.
    #[default]
    ShutDown = 0,
    /// Hold at idle: rotors turning, no thrust authority.
    GroundIdle = 1,
    /// Fly. Throttle bounded only by current limiting.
    ThrottleUnlimited = 2,
}

/// Which axes have run out of authority, upstream's `AP_Motors_limit`.
///
/// The spool machine only ever sets all five together — every limit is on
/// while the motors are gated and off once the controller is allowed to fly.
/// The per-axis granularity exists for the mixer, which sets them
/// individually when a demand cannot be met.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Limits {
    /// Roll demand could not be met in full.
    pub roll: bool,
    /// Pitch demand could not be met in full.
    pub pitch: bool,
    /// Yaw demand could not be met in full.
    pub yaw: bool,
    /// Throttle is against its lower bound.
    pub throttle_lower: bool,
    /// Throttle is against its upper bound.
    pub throttle_upper: bool,
}

impl Limits {
    /// Upstream `set_all`.
    pub fn set_all(&mut self, flag: bool) {
        *self = Self {
            roll: flag,
            pitch: flag,
            yaw: flag,
            throttle_lower: flag,
            throttle_upper: flag,
        };
    }

    /// Upstream `set_rpy`.
    pub fn set_rpy(&mut self, flag: bool) {
        self.roll = flag;
        self.pitch = flag;
        self.yaw = flag;
    }

    /// Upstream `set_throttle`.
    pub fn set_throttle(&mut self, flag: bool) {
        self.throttle_lower = flag;
        self.throttle_upper = flag;
    }

    /// Merge limits a script has asserted, upstream `update_external_limits`.
    ///
    /// A logical OR in every axis, never an assignment. A script can say "I
    /// have run out of authority here" and be believed, but cannot clear a
    /// limit the mixer set -- the mixer knows something about the frame the
    /// script does not.
    pub fn merge_external(&mut self, external: Self) {
        self.roll |= external.roll;
        self.pitch |= external.pitch;
        self.yaw |= external.yaw;
        self.throttle_lower |= external.throttle_lower;
        self.throttle_upper |= external.throttle_upper;
    }
}

/// The tunables the machine reads, upstream's `AP_Float` parameters.
#[derive(Debug, Clone, Copy)]
pub struct SpoolParams {
    /// `MOT_SPOOL_TIME`: seconds for the throttle ceiling to cross its range.
    ///
    /// Taken by `&mut` through [`Spool::update`] because upstream clamps this
    /// back into the parameter itself, not into a local. A vehicle configured
    /// with `MOT_SPOOL_TIME` below [`MINIMUM_SPOOL_TIME`] has the parameter
    /// rewritten in memory on the first iteration, and every later read — by
    /// this machine, by a GCS, by anything — sees the clamped value.
    pub spool_up_time: f32,
    /// `MOT_SPOOL_TIM_DN`: the down-ramp time. Falls back to
    /// [`SpoolParams::spool_up_time`] when it is not above the minimum, which
    /// is how "unset" is spelled.
    pub spool_down_time: f32,
    /// `MOT_SAFE_TIME`: how long after arming to hold at shut down so ESCs can
    /// see PWM return and finish starting.
    pub safe_time: f32,
    /// `MOT_SPIN_ARM`: the output that produces the armed idle spin, as a
    /// fraction of the full output range.
    pub spin_arm: f32,
    /// `MOT_SPOOL_IDLE_T`: how long to hold at ground idle once the rotors get
    /// there, before spool-up may continue.
    pub idle_time_delay_s: f32,
    /// `MOT_PWM_TYPE`-adjacent: whether PWM is off entirely while disarmed.
    /// When it is, the safe-time delay applies.
    pub disarm_disable_pwm: bool,
}

/// What the machine reads from the rest of the vehicle each iteration.
#[derive(Debug, Clone, Copy)]
pub struct SpoolInputs {
    /// Whether the vehicle is armed. False forces shut down.
    pub armed: bool,
    /// The motor interlock. False forces shut down with no ramp, same as
    /// disarming.
    pub interlock: bool,
    /// Seconds since the last iteration.
    pub dt_s: f32,
    /// `SPIN_MIN` from the thrust linearisation: the output at which the
    /// rotors start turning.
    pub spin_min: f32,
    /// The currently commanded throttle, upstream `get_throttle()`.
    pub throttle: f32,
    /// The steady-state ceiling battery current limiting allows, upstream
    /// `get_current_limit_max_throttle()`.
    pub current_limit_max_throttle: f32,
    /// Whether the mixer considers thrust well balanced across motors. Only
    /// consulted while a thrust boost is active.
    pub thrust_balanced: bool,
}

/// The machine's own state.
#[derive(Debug, Clone, Copy, Default)]
pub struct Spool {
    state: SpoolState,
    desired: DesiredSpoolState,
    spin_up_ratio: f32,
    throttle_thrust_max: f32,
    idle_time: f32,
    disarm_safe_timer: f32,
    spin_up_complete: bool,
    spoolup_block: bool,
    thrust_boost: bool,
    thrust_boost_ratio: f32,
    limits: Limits,
}

impl Spool {
    /// A machine in `ShutDown` with both ramps at zero.
    pub fn new() -> Self {
        Self::default()
    }

    /// Where the motors currently are.
    pub fn state(&self) -> SpoolState {
        self.state
    }

    /// The standing request, after any safety override.
    pub fn desired(&self) -> DesiredSpoolState {
        self.desired
    }

    /// The spin ramp, 0 to 1 across stopped to `SPIN_MIN`.
    pub fn spin_up_ratio(&self) -> f32 {
        self.spin_up_ratio
    }

    /// The moving ceiling on commanded throttle.
    pub fn throttle_thrust_max(&self) -> f32 {
        self.throttle_thrust_max
    }

    /// Which axes are out of authority. All set while the motors are
    /// gated, all clear once the controller may fly.
    pub fn limits(&self) -> Limits {
        self.limits
    }

    /// Whether a failed-motor thrust boost is active.
    pub fn thrust_boost(&self) -> bool {
        self.thrust_boost
    }

    /// How far the thrust boost has slewed in, 0 to 1.
    pub fn thrust_boost_ratio(&self) -> f32 {
        self.thrust_boost_ratio
    }

    /// Upstream `set_thrust_boost`, used by the mixer when a motor fails.
    pub fn set_thrust_boost(&mut self, enable: bool) {
        self.thrust_boost = enable;
    }

    /// Whether spool-up is being held at ground idle.
    ///
    /// The machine *sets* this itself when the spin ramp completes; only the
    /// vehicle's pre-takeoff checks clear it. That asymmetry is deliberate:
    /// reaching idle is something the motors know, but being safe to take off
    /// is not.
    pub fn spoolup_block(&self) -> bool {
        self.spoolup_block
    }

    /// Upstream `set_spoolup_block`.
    pub fn set_spoolup_block(&mut self, set: bool) {
        self.spoolup_block = set;
    }

    /// Upstream `set_desired_spool_state`'s effect on the stored request.
    ///
    /// Upstream refuses to accept `THROTTLE_UNLIMITED` while disarmed; that
    /// check lives in `set_desired_spool_state` rather than here, so it is
    /// applied by the caller that owns the armed flag.
    pub fn set_desired(&mut self, desired: DesiredSpoolState) {
        self.desired = desired;
    }

    /// One iteration, upstream `output_logic`.
    #[expect(
        clippy::too_many_lines,
        reason = "one upstream function, kept as one function so it can be \
read against the original; splitting it would hide the fall-through order \
between the states, which is the part that has to be right"
    )]
    pub fn update(&mut self, params: &mut SpoolParams, input: &SpoolInputs) {
        // The disarm-PWM safety window. The timer only advances while armed,
        // and resets the moment it is not, so a disarm always costs the full
        // delay again rather than resuming a part-elapsed one.
        if input.armed {
            if params.disarm_disable_pwm && self.disarm_safe_timer < params.safe_time {
                self.disarm_safe_timer += input.dt_s;
            } else {
                self.disarm_safe_timer = params.safe_time;
            }
        } else {
            self.disarm_safe_timer = 0.0;
        }

        // Unconditional safety rule. Not a transition — both the request and
        // the state are forced, with no ramp, so nothing downstream can
        // observe a partially spooled machine after an interlock drop.
        if !input.armed || !input.interlock {
            self.desired = DesiredSpoolState::ShutDown;
            self.state = SpoolState::ShutDown;
        }

        if params.spool_up_time < MINIMUM_SPOOL_TIME {
            params.spool_up_time = MINIMUM_SPOOL_TIME;
        }

        match self.state {
            SpoolState::ShutDown => {
                // Motors stationary, thrust disabled, no authority.
                self.limits.set_all(true);

                self.spin_up_ratio = 0.0;
                self.throttle_thrust_max = 0.0;
                self.idle_time = 0.0;

                self.thrust_boost = false;
                self.thrust_boost_ratio = 0.0;

                if self.desired != DesiredSpoolState::ShutDown
                    && self.disarm_safe_timer >= params.safe_time
                {
                    self.state = SpoolState::GroundIdle;
                }
            }

            SpoolState::GroundIdle => {
                // Attitude control runs here; thrust does not. Limits stay set
                // because the controller has no throttle authority to use.
                self.limits.set_all(true);

                // The spin ramp is normalised against SPIN_MIN, so the armed
                // idle sits at some fraction of it. With no SPIN_MIN there is
                // nothing to normalise against and the target is zero.
                let mut spin_up_ground_idle_ratio = 0.0;
                if is_positive(input.spin_min) {
                    spin_up_ground_idle_ratio = params.spin_arm / input.spin_min;
                }

                if self.spin_up_ratio >= spin_up_ground_idle_ratio {
                    // The delay measures time *at* idle, not time since the
                    // request, so slow rotors do not eat into it.
                    self.idle_time = (self.idle_time + input.dt_s).min(params.idle_time_delay_s);
                }

                match self.desired {
                    DesiredSpoolState::ShutDown => {
                        let spool_time = if params.spool_down_time > MINIMUM_SPOOL_TIME {
                            params.spool_down_time
                        } else {
                            params.spool_up_time
                        };
                        let spool_step = input.dt_s / spool_time;
                        self.spin_up_ratio -= spool_step;

                        if self.spin_up_ratio <= 0.0 {
                            self.spin_up_ratio = 0.0;
                            self.state = SpoolState::ShutDown;
                        }
                    }

                    DesiredSpoolState::ThrottleUnlimited => {
                        let spool_step = input.dt_s / params.spool_up_time;
                        self.spin_up_ratio += spool_step;

                        // Hold at idle while the delay runs, so the ESCs get
                        // their startup window with PWM live.
                        if self.idle_time < params.idle_time_delay_s {
                            self.spin_up_ratio = self.spin_up_ratio.min(spin_up_ground_idle_ratio);
                        } else {
                            if self.spin_up_ratio < 1.0 {
                                self.spin_up_complete = false;
                            } else {
                                self.spin_up_ratio = 1.0;
                                if !self.spin_up_complete {
                                    // Reaching idle raises the block; only the
                                    // vehicle's checks lower it.
                                    self.spin_up_complete = true;
                                    self.spoolup_block = true;
                                }
                            }
                            if self.spin_up_complete && !self.spoolup_block {
                                self.state = SpoolState::SpoolingUp;
                            }
                        }
                    }

                    DesiredSpoolState::GroundIdle => {
                        // Asymmetric slew toward the idle target: down is
                        // limited by the down time, up by the up time, so the
                        // two directions can have different feel without
                        // either becoming a step.
                        let spool_up_step = input.dt_s / params.spool_up_time;
                        let spool_down_time = if params.spool_down_time > MINIMUM_SPOOL_TIME {
                            params.spool_down_time
                        } else {
                            params.spool_up_time
                        };
                        let spool_down_step = input.dt_s / spool_down_time;

                        self.spin_up_ratio += (spin_up_ground_idle_ratio - self.spin_up_ratio)
                            .clamp(-spool_down_step, spool_up_step);
                    }
                }

                self.throttle_thrust_max = 0.0;

                self.thrust_boost = false;
                self.thrust_boost_ratio = 0.0;
            }

            SpoolState::SpoolingUp => {
                let spool_step = input.dt_s / params.spool_up_time;

                self.limits.set_all(false);

                if self.desired != DesiredSpoolState::ThrottleUnlimited {
                    self.state = SpoolState::SpoolingDown;
                    return;
                }

                self.spin_up_ratio = 1.0;
                self.throttle_thrust_max += spool_step;

                // Done once the moving ceiling stops being what limits the
                // commanded throttle — comparing against the demand, not
                // against 1.0, so a gentle takeoff finishes spooling early.
                if self.throttle_thrust_max >= input.throttle.min(input.current_limit_max_throttle)
                {
                    self.throttle_thrust_max = input.current_limit_max_throttle;
                    self.state = SpoolState::ThrottleUnlimited;
                } else if self.throttle_thrust_max < 0.0 {
                    self.throttle_thrust_max = 0.0;
                }

                self.thrust_boost = false;
                self.thrust_boost_ratio = (self.thrust_boost_ratio - spool_step).max(0.0);
            }

            SpoolState::ThrottleUnlimited => {
                let spool_step = input.dt_s / params.spool_up_time;

                self.limits.set_all(false);

                if self.desired != DesiredSpoolState::ThrottleUnlimited {
                    self.state = SpoolState::SpoolingDown;
                    return;
                }

                self.spin_up_ratio = 1.0;
                self.throttle_thrust_max = input.current_limit_max_throttle;

                if self.thrust_boost && !input.thrust_balanced {
                    self.thrust_boost_ratio = (self.thrust_boost_ratio + spool_step).min(1.0);
                } else {
                    self.thrust_boost_ratio = (self.thrust_boost_ratio - spool_step).max(0.0);
                }
            }

            SpoolState::SpoolingDown => {
                self.limits.set_all(false);

                if self.desired == DesiredSpoolState::ThrottleUnlimited {
                    self.state = SpoolState::SpoolingUp;
                    return;
                }

                // Spin stays at 1.0 through the whole down-ramp; the rotors
                // only slow once GROUND_IDLE takes over.
                self.spin_up_ratio = 1.0;

                let spool_time = if params.spool_down_time > MINIMUM_SPOOL_TIME {
                    params.spool_down_time
                } else {
                    params.spool_up_time
                };
                let spool_step = input.dt_s / spool_time;

                self.throttle_thrust_max -= spool_step;

                if self.throttle_thrust_max <= 0.0 {
                    self.throttle_thrust_max = 0.0;
                }
                // Ordered as upstream orders it. The current-limit clamp is
                // tested first, so when the limit is itself zero the ceiling
                // is clamped to zero and the `is_zero` branch never runs — the
                // machine stays in SPOOLING_DOWN. That needs a current limit
                // of exactly zero, which means a vehicle that cannot draw any
                // current at all, so it is unreachable rather than latent.
                if self.throttle_thrust_max >= input.current_limit_max_throttle {
                    self.throttle_thrust_max = input.current_limit_max_throttle;
                } else if is_zero(self.throttle_thrust_max) {
                    self.state = SpoolState::GroundIdle;
                }

                self.thrust_boost_ratio = (self.thrust_boost_ratio - spool_step).max(0.0);
            }
        }
    }
}
