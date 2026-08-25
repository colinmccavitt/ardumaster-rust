//! The descent demand every ArduCopter landing flies, upstream
//! `ArduCopter/mode.cpp:704`, `Mode::land_run_vertical_control`.
//!
//! # What is here and what is not
//!
//! Upstream's function computes a climb rate and then hands it to the position
//! controller. Only the computation is here; the two controller calls that
//! follow it belong to the caller, for the same reason the spool command is
//! returned rather than issued in [`crate::alt_hold`] — a decision that is
//! also an action is easier to test and harder to misuse when the two are
//! separated.
//!
//! The precision-landing adjustment is **not ported here**. Upstream guards it
//! with `AC_PRECLAND_ENABLED`, which defaults on, and when a target is
//! acquired it can override the demand entirely — holding the descent at zero
//! while the vehicle is too far from the target, or slowing it to a crawl near
//! the ground. [`land_descent`] computes the demand *before* that override.
//!
//! That boundary is deliberate and it is a real limit: a caller that has
//! precision landing active must not use this result unmodified. It is drawn
//! here because the override reads `AC_PrecLand`, which is not ported, and
//! porting a branch whose inputs are all invented would produce a function
//! that could not be recorded against the firmware.

use ap_math::control::sqrt_controller;
use ap_math::scalar::constrain_value;

/// The settings a landing descent reads.
///
/// All of these are parameters or controller limits rather than state, so a
/// caller assembles this once and reuses it across iterations.
#[derive(Debug, Clone, Copy)]
pub struct LandDescentConfig {
    /// `LAND_ALT_LOW`, the height at which the descent slows to its final
    /// speed. Metres above ground.
    pub land_alt_low_m: f32,
    /// `LAND_SPEED_HIGH`, the descent speed used above that height. Zero means
    /// "no separate high-speed descent", and the position controller's own
    /// maximum is used instead.
    pub land_speed_high_ms: f32,
    /// `LAND_SPEED`, the final descent speed. Read through `fabsf`, so its
    /// sign is not trusted.
    pub land_speed_ms: f32,
    /// `pos_control->get_max_speed_down_ms()`.
    pub max_speed_down_ms: f32,
    /// The vertical position controller's proportional gain.
    pub pos_p_kp: f32,
    /// The vertical position controller's acceleration limit, m/s².
    pub max_accel_mss: f32,
}

/// A descent demand and whether the descent limit should be lifted for it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LandDescent {
    /// Climb rate, m/s, up positive — so a descent is negative.
    pub climb_rate_ms: f32,
    /// Whether to lift the position controller's limit on downward travel.
    pub ignore_descent_limit: bool,
}

/// The descent demand, upstream `Mode::land_run_vertical_control`'s
/// computation.
///
/// # The floor under `land_alt_low_m`
///
/// Every use of the slowdown height is wrapped in `MAX(land_alt_low_m, 1)`, so
/// a parameter of zero — or anything below a metre — behaves as one metre. The
/// aircraft therefore always has a slowdown region, and the controller below
/// always has a non-zero distance to work against.
///
/// # Why the target is the slowdown height and not the ground
///
/// The proportional term drives the aircraft towards `land_alt_low_m` rather
/// than towards zero. On its own that would leave it hovering there, which is
/// exactly what the constraint below prevents: the demand is clamped to at
/// most `-|land_speed_ms|`, so it can never reach zero and the aircraft keeps
/// descending through the slowdown region at the final speed. The controller
/// shapes the *approach* to that region; the clamp carries it the rest of the
/// way down.
///
/// # The descent speed is a floor as well as a ceiling
///
/// `max_land_descent_speed_ms` is raised to at least `|land_speed_ms|` before
/// it is used as a bound — "don't speed up for landing", in upstream's words.
/// Without it, a `LAND_SPEED` set faster than `LAND_SPEED_HIGH` would make the
/// aircraft accelerate as it neared the ground, which is the opposite of what
/// both parameters are for.
#[must_use]
pub fn land_descent(
    pause_descent: bool,
    alt_above_ground_m: f32,
    land_complete_maybe: bool,
    config: &LandDescentConfig,
    dt: f32,
) -> LandDescent {
    if pause_descent {
        // Nothing is computed at all, and in particular the descent limit is
        // left in place: a paused descent is not a landing that has arrived.
        return LandDescent {
            climb_rate_ms: 0.0,
            ignore_descent_limit: false,
        };
    }

    let land_alt_low_m = libm::fmaxf(config.land_alt_low_m, 1.0);

    // Do not lift the limit until the aircraft has slowed for landing. Below
    // the slowdown height, or once the vehicle might already be down, the
    // limit would fight a descent that is meant to continue.
    let ignore_descent_limit = land_alt_low_m > alt_above_ground_m || land_complete_maybe;

    let mut max_land_descent_speed_ms = if config.land_speed_high_ms > 0.0 {
        config.land_speed_high_ms
    } else {
        config.max_speed_down_ms
    };

    // Don't speed up for landing.
    max_land_descent_speed_ms =
        libm::fmaxf(max_land_descent_speed_ms, libm::fabsf(config.land_speed_ms));

    let climb_rate_ms = sqrt_controller(
        land_alt_low_m - alt_above_ground_m,
        config.pos_p_kp,
        config.max_accel_mss,
        dt,
    );

    LandDescent {
        climb_rate_ms: constrain_value(
            climb_rate_ms,
            -max_land_descent_speed_ms,
            -libm::fabsf(config.land_speed_ms),
        ),
        ignore_descent_limit,
    }
}
