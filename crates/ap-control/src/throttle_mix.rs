//! The multicopter throttle and lean-angle logic, upstream
//! `AC_AttitudeControl_Multi`.
//!
//! Everything here exists because a multirotor steers by tilting the same
//! rotors that hold it up. Roll, pitch and yaw are all paid for out of the
//! throttle budget, so attitude authority and altitude are in direct
//! competition and the vehicle has to decide, continuously, how much of one to
//! give up for the other.

use ap_math::scalar::is_zero;
use ap_math::vector3::Vector3f;

/// Throttle above which the lean-angle limit starts closing in, upstream
/// `AC_ATTITUDE_CONTROL_ANGLE_LIMIT_THROTTLE_MAX`.
const ANGLE_LIMIT_THROTTLE_MAX: f32 = 0.8;

/// Throttle slew rate that triggers the gain boost, upstream
/// `AC_ATTITUDE_CONTROL_THR_G_BOOST_THRESH`.
const THR_G_BOOST_THRESH: f32 = 1.0;

/// Upstream `AC_ATTITUDE_CONTROL_MIN_DEFAULT`.
pub const THR_MIX_MIN_DEFAULT: f32 = 0.1;
/// Upstream `AC_ATTITUDE_CONTROL_MAX_DEFAULT`.
pub const THR_MIX_MAX_DEFAULT: f32 = 0.5;
/// Upstream `AC_ATTITUDE_CONTROL_MIN_LIMIT`.
pub const THR_MIX_MIN_LIMIT: f32 = 0.5;
/// Upstream `AC_ATTITUDE_CONTROL_MAN_LIMIT`.
pub const THR_MIX_MAN_LIMIT: f32 = 4.0;
/// Upstream `AC_ATTITUDE_CONTROL_MAX`, the ceiling on the mix itself.
pub const THR_MIX_MAX: f32 = 5.0;

/// The tuning this logic reads.
#[derive(Debug, Clone, Copy)]
pub struct ThrottleMixConfig {
    /// Time constant for the lean-angle limit's filter, upstream
    /// `_angle_limit_tc`.
    pub angle_limit_tc: f32,
    /// Mix while flying manually, upstream `_thr_mix_man`.
    pub thr_mix_man: f32,
    /// Floor on the mix, upstream `_thr_mix_min`.
    pub thr_mix_min: f32,
    /// Ceiling on the mix, upstream `_thr_mix_max`.
    pub thr_mix_max: f32,
    /// How hard to boost gains on a throttle slew, upstream
    /// `_throttle_gain_boost`.
    pub throttle_gain_boost: f32,
    /// Whether tilt compensation is applied at all, upstream
    /// `_angle_boost_enabled`.
    pub angle_boost_enabled: bool,
}

/// What the vehicle is currently doing, as this logic needs to see it.
#[derive(Debug, Clone, Copy)]
pub struct VehicleThrottleState {
    /// Largest thrust the motors will accept, upstream
    /// `get_throttle_thrust_max`.
    pub throttle_thrust_max: f32,
    /// The configured hover throttle, upstream `get_throttle_hover`.
    pub throttle_hover: f32,
    /// What was last commanded, upstream `_motors.get_throttle`.
    pub throttle_in: f32,
    /// What the mixer actually produced, upstream `get_throttle_out`.
    pub throttle_out: f32,
    /// Upstream `get_throttle_slew_rate`.
    pub throttle_slew_rate: f32,
    /// `cos(pitch) * cos(roll)` from the AHRS — how upright the vehicle is.
    pub cos_tilt: f32,
    /// The controller's `_thrust_angle_rad` — how far the *target* leans.
    pub thrust_angle_rad: f32,
}

/// The state this logic carries between iterations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThrottleMix {
    throttle_rpy_mix: f32,
    throttle_rpy_mix_desired: f32,
    althold_lean_angle_max_rad: f32,
    angle_boost: f32,
}

impl Default for ThrottleMix {
    fn default() -> Self {
        Self::new()
    }
}

impl ThrottleMix {
    /// Upstream's construction state: the mix at its default floor.
    #[must_use]
    pub fn new() -> Self {
        Self {
            throttle_rpy_mix: THR_MIX_MIN_DEFAULT,
            throttle_rpy_mix_desired: THR_MIX_MIN_DEFAULT,
            althold_lean_angle_max_rad: 0.0,
            angle_boost: 0.0,
        }
    }

    /// The current mix, upstream `_throttle_rpy_mix`.
    #[must_use]
    pub fn mix(&self) -> f32 {
        self.throttle_rpy_mix
    }

    /// The lean-angle ceiling altitude hold is willing to allow, radians.
    #[must_use]
    pub fn althold_lean_angle_max_rad(&self) -> f32 {
        self.althold_lean_angle_max_rad
    }

    /// How much throttle the last boost added, upstream `_angle_boost`.
    #[must_use]
    pub fn angle_boost(&self) -> f32 {
        self.angle_boost
    }

    /// Set both the current mix and the request, upstream
    /// `set_throttle_mix_value`.
    ///
    /// The one setter that bypasses the slew. A caller reaching for this is
    /// saying the vehicle's situation just changed discontinuously — landed,
    /// disarmed — and that easing there would be modelling a transition that
    /// is not happening.
    pub fn set_throttle_mix_value(&mut self, value: f32) {
        self.throttle_rpy_mix = value;
        self.throttle_rpy_mix_desired = value;
    }

    /// Request a mix directly, without going through a ratio.
    ///
    /// Upstream's three setters — `set_throttle_mix_min`, `_man` and `_max` —
    /// all reduce to writing this one member, the first two straight from a
    /// parameter. Exposing it directly says the same thing without making a
    /// caller pick which parameter its intent happens to match.
    pub fn set_throttle_mix_desired(&mut self, value: f32) {
        self.throttle_rpy_mix_desired = value;
    }

    /// Set the mix to the landing value, upstream `set_throttle_mix_min`.
    pub fn set_throttle_mix_min(&mut self, config: &ThrottleMixConfig) {
        self.throttle_rpy_mix_desired = config.thr_mix_min;
    }

    /// Ask for a mix, upstream `set_throttle_mix_max`.
    ///
    /// `ratio` interpolates between the floor and the ceiling. It is a request,
    /// not an assignment: [`Self::update_throttle_rpy_mix`] slews toward it.
    pub fn set_throttle_mix_ratio(&mut self, ratio: f32, config: &ThrottleMixConfig) {
        let ratio = ratio.clamp(0.0, 1.0);
        self.throttle_rpy_mix_desired =
            (1.0 - ratio) * config.thr_mix_min + ratio * config.thr_mix_max;
    }

    /// Set the mix directly to the manual value, upstream `set_throttle_mix_man`.
    pub fn set_throttle_mix_man(&mut self, config: &ThrottleMixConfig) {
        self.throttle_rpy_mix_desired = config.thr_mix_man;
    }

    /// How far the vehicle may lean before altitude hold objects — upstream
    /// `update_althold_lean_angle_max`.
    ///
    /// Leaning costs altitude: at tilt θ only `cos θ` of the thrust holds the
    /// aircraft up. This inverts that. At 80% of maximum thrust the permitted
    /// lean is zero, because there is no headroom left to pay for a lean.
    ///
    /// The result is filtered rather than applied directly, over a one-second
    /// time constant. Without it a throttle transient would snap the lean
    /// limit shut, and the pilot would feel the aircraft refuse to bank in the
    /// middle of a turn.
    pub fn update_althold_lean_angle_max(
        &mut self,
        throttle_in: f32,
        state: &VehicleThrottleState,
        config: &ThrottleMixConfig,
        dt: f32,
    ) {
        if is_zero(state.throttle_thrust_max) {
            self.althold_lean_angle_max_rad = 0.0;
            return;
        }

        let ratio =
            (throttle_in / (ANGLE_LIMIT_THROTTLE_MAX * state.throttle_thrust_max)).clamp(0.0, 1.0);
        let target = libm::acosf(ratio);
        self.althold_lean_angle_max_rad +=
            (dt / (dt + config.angle_limit_tc)) * (target - self.althold_lean_angle_max_rad);
    }

    /// Throttle with tilt compensation, upstream `get_throttle_boosted`.
    ///
    /// Two factors, and they answer different questions.
    ///
    /// `boost_factor` is `1/cos` of the *target* lean: leaning by θ costs a
    /// factor `cos θ` of vertical thrust, so the throttle is divided by it to
    /// hold altitude through a bank. Clamped at a tenth, capping the boost at
    /// ten times rather than letting it run away toward vertical.
    ///
    /// `inverted_factor` looks at where the vehicle *is*, and fades the whole
    /// boost out between 60 and 90 degrees of actual tilt. Past 60 degrees more
    /// throttle stops buying altitude, and past 90 it actively drives the
    /// aircraft down. Boosting there would be exactly wrong, so the boost is
    /// withdrawn instead.
    ///
    /// Records `angle_boost` for logging as a side effect, which is why this
    /// takes `&mut self`.
    pub fn get_throttle_boosted(
        &mut self,
        throttle_in: f32,
        state: &VehicleThrottleState,
        config: &ThrottleMixConfig,
    ) -> f32 {
        if !config.angle_boost_enabled {
            self.angle_boost = 0.0;
            return throttle_in;
        }

        let inverted_factor = (10.0 * state.cos_tilt).clamp(0.0, 1.0);
        let cos_tilt_target = libm::cosf(state.thrust_angle_rad);
        let boost_factor = 1.0 / cos_tilt_target.clamp(0.1, 1.0);

        let throttle_out = throttle_in * inverted_factor * boost_factor;
        self.angle_boost = (throttle_out - throttle_in).clamp(-1.0, 1.0);
        throttle_out
    }

    /// The average-maximum throttle handed to the mixer, upstream
    /// `get_throttle_avg_max`.
    ///
    /// Blends the commanded throttle toward hover in proportion to the mix,
    /// then takes whichever is larger. The `MAX` is what makes it safe: the
    /// blend can only ever raise the figure, never lower it, so a large mix
    /// cannot quietly starve a vehicle that is genuinely asking for throttle.
    #[must_use]
    pub fn get_throttle_avg_max(&self, throttle_in: f32, state: &VehicleThrottleState) -> f32 {
        let throttle_in = throttle_in.clamp(0.0, 1.0);
        let blended = throttle_in * (1.0 - self.throttle_rpy_mix).max(0.0)
            + state.throttle_hover * self.throttle_rpy_mix;
        throttle_in.max(blended)
    }

    /// Boost the gains through a fast throttle change, upstream
    /// `update_throttle_gain_boost`.
    ///
    /// Returns the PD and angle-P multipliers to apply this cycle, or `None`
    /// when the slew rate is below threshold and nothing should change.
    ///
    /// Angle P is boosted by the *square* of what PD gets, because angle P
    /// feeds the rate loop which PD then acts on — the two multiply, so
    /// squaring one keeps the loop's overall gain moving with the other rather
    /// than lagging it. Yaw is left alone in both: a throttle slew disturbs
    /// roll and pitch, while yaw comes from rotor drag and is barely touched.
    #[must_use]
    pub fn update_throttle_gain_boost(
        state: &VehicleThrottleState,
        config: &ThrottleMixConfig,
    ) -> Option<GainBoost> {
        if state.throttle_slew_rate <= THR_G_BOOST_THRESH {
            return None;
        }
        let pd = (config.throttle_gain_boost + 1.0).clamp(1.0, 2.0);
        let angle_p = ((config.throttle_gain_boost + 1.0) * (config.throttle_gain_boost + 1.0))
            .clamp(1.0, 4.0);
        Some(GainBoost {
            pd_scale: Vector3f::new(pd, pd, 1.0),
            angle_p_scale: Vector3f::new(angle_p, angle_p, 1.0),
        })
    }

    /// Slew the mix toward what was asked for, upstream
    /// `update_throttle_rpy_mix`.
    ///
    /// Deliberately asymmetric: it rises about four times faster than it
    /// falls. Giving attitude control more of the throttle budget is urgent —
    /// the vehicle may be losing control right now — while taking it back is
    /// not, so the retreat is gentle and unnoticeable.
    ///
    /// While falling it also checks how much mix the mixer *actually* used and
    /// snaps down to that if it is lower. The slew is there to avoid a jolt,
    /// and there is no jolt to avoid below a level the vehicle was already
    /// flying at.
    pub fn update_throttle_rpy_mix(&mut self, state: &VehicleThrottleState, dt: f32) {
        if self.throttle_rpy_mix < self.throttle_rpy_mix_desired {
            self.throttle_rpy_mix +=
                (2.0 * dt).min(self.throttle_rpy_mix_desired - self.throttle_rpy_mix);
        } else if self.throttle_rpy_mix > self.throttle_rpy_mix_desired {
            self.throttle_rpy_mix -=
                (0.5 * dt).min(self.throttle_rpy_mix - self.throttle_rpy_mix_desired);

            let throttle_hover = state.throttle_hover;
            let throttle_in = state.throttle_in;
            let throttle_out = state.throttle_out.max(throttle_in);

            // No divide-by-zero guard, and upstream is right not to have one.
            // This branch has `throttle_out >= throttle_in` by construction,
            // so reaching the first case means
            // `throttle_in <= throttle_out < throttle_hover`, and the
            // denominator is strictly positive.
            let mix_used = if throttle_out < throttle_hover {
                (throttle_out - throttle_in) / (throttle_hover - throttle_in)
            } else {
                throttle_out / throttle_hover
            };

            self.throttle_rpy_mix = self
                .throttle_rpy_mix
                .min(mix_used.max(self.throttle_rpy_mix_desired));
        }
        self.throttle_rpy_mix = self
            .throttle_rpy_mix
            .clamp(THR_MIX_MIN_DEFAULT, THR_MIX_MAX);
    }
}

/// The gain multipliers a throttle slew asks for.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GainBoost {
    /// Multiplier on the rate loop's P and D terms.
    pub pd_scale: Vector3f,
    /// Multiplier on the angle loop's P term.
    pub angle_p_scale: Vector3f,
}

/// Bring the mix parameters back into range, upstream `parameter_sanity_check`.
///
/// Returns the corrected values. Upstream writes them back to storage with
/// `set_and_save`; that belongs to whatever owns the parameters, so this only
/// computes.
///
/// The last rule reads as the important one — if the floor ends up above the
/// ceiling, both are thrown away and replaced with defaults rather than one
/// being clamped to the other, on the grounds that either could be the wrong
/// one. It is also unreachable.
///
/// The clamps above it put `min` in `[0.1, 0.5]` and `max` in `[0.5, 5.0]`, so
/// afterwards `min <= 0.5 <= max` always holds and `min > max` would require
/// `min > 0.5`, which its own clamp forbids. NaN does not get there either:
/// every comparison against NaN is false, so no clamp applies and the pair
/// test fails as well.
///
/// Reproduced anyway. It costs nothing, it is what upstream does, and if
/// either limit is ever retuned so the ranges stop overlapping it becomes live
/// again — at which point having deleted it would be the bug.
#[must_use]
pub fn parameter_sanity_check(man: f32, min: f32, max: f32) -> (f32, f32, f32) {
    let man = if !(0.1..=THR_MIX_MAN_LIMIT).contains(&man) {
        man.clamp(0.1, THR_MIX_MAN_LIMIT)
    } else {
        man
    };
    let mut min_out = if !(0.1..=THR_MIX_MIN_LIMIT).contains(&min) {
        min.clamp(0.1, THR_MIX_MIN_LIMIT)
    } else {
        min
    };
    let mut max_out = if !(0.5..=THR_MIX_MAX).contains(&max) {
        max.clamp(0.5, THR_MIX_MAX)
    } else {
        max
    };
    if min_out > max_out {
        min_out = THR_MIX_MIN_DEFAULT;
        max_out = THR_MIX_MAX_DEFAULT;
    }
    (man, min_out, max_out)
}
