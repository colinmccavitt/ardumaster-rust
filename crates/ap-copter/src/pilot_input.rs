//! Pilot stick conversions shared by the manual multirotor modes, upstream
//! `Copter`'s `Mode::get_pilot_desired_*`.
//!
//! Each takes raw stick input and produces a demand for a controller. All of
//! them return neutral when the radio has no valid input — a failsafe with a
//! stale stick position is worse than one with none.
//!
//! # Bringing a Copter harness up
//!
//! These were briefly ported without a recording, because the hover throttle
//! is read through `copter.motors` — a pointer assigned during the vehicle's
//! `setup()`, which a parity harness does not run.
//!
//! `Copter::allocate_motors()` turns out to be the one function in `setup()`
//! that assigns it, along with the attitude and position controllers. Calling
//! it directly, after the scheduler is up so it can read the loop rate, gives
//! a vehicle whose controllers are the firmware's own — which unblocks
//! recording for the `Mode` layer generally, not only for these.

use ap_math::control::input_expo;
use ap_math::scalar::{constrain_value, radians};

/// The pilot's desired yaw rate, radians per second, upstream
/// `get_pilot_desired_yaw_rate_rads`.
///
/// The expo is applied to the *stick*, then scaled by the configured rate —
/// not the other way round. That ordering is what makes the expo mean
/// "sensitivity around centre" rather than something that changes with the
/// rate setting: a pilot who raises their maximum yaw rate gets a
/// proportionally faster response everywhere, with the same feel near centre.
#[must_use]
pub fn pilot_desired_yaw_rate_rads(
    yaw_in_norm: f32,
    rate_degs: f32,
    expo: f32,
    has_valid_input: bool,
) -> f32 {
    if !has_valid_input {
        return 0.0;
    }
    radians(rate_degs) * input_expo(yaw_in_norm, expo)
}

/// The pilot's desired throttle, 0 to 1, upstream
/// `get_pilot_desired_throttle`.
///
/// # Two straight lines, not one
///
/// The stick maps piecewise: the bottom half of its travel spans 0 to 0.5 of
/// throttle and the top half spans 0.5 to 1. So mid-stick is always exactly
/// half throttle regardless of where mid-stick physically sits, and moving the
/// trim changes the *slope* of each half rather than shifting the whole curve.
/// A pilot who trims mid-stick low gets finer control below it and coarser
/// above, which is the intent — that is the region they hover in.
///
/// # Then a cubic, whose strength comes from the hover throttle
///
/// `expo` is derived from the configured hover throttle rather than set
/// directly: `-(thr_mid - 0.5) / 0.375`. An aircraft that hovers at half
/// throttle gets no shaping at all. One that hovers *low* — a powerful
/// airframe — gets positive expo, which flattens the curve near centre and
/// gives it finer control where it spends its time. One that hovers high gets
/// negative expo, steepening it there.
///
/// The bounds are asymmetric, −0.5 to 1.0, because the two cases are not
/// symmetric: a very powerful aircraft benefits from a lot of softening, while
/// a marginal one cannot afford much sharpening before the stick becomes
/// twitchy at exactly the point it needs to be precise.
///
/// # The divide-by-zero guards
///
/// A mid-stick at or below zero falls back to 500. Upstream calls it unlikely
/// rather than impossible, and the fallback is the default rather than
/// something derived — there is nothing sensible to derive it from.
///
/// There is a second such case upstream does not guard, at the other end: a
/// mid-stick of 1000 leaves the upper span no width to divide by. This port
/// closes it, at no cost to any other input. See DIVERGENCES.md D-026.
#[must_use]
pub fn pilot_desired_throttle(throttle_control: i16, mid_stick: i16, throttle_hover: f32) -> f32 {
    let mid_stick = if mid_stick <= 0 { 500 } else { mid_stick };
    let throttle_control = throttle_control.clamp(0, 1000);

    // Upstream writes `<` here, which sends a stick sitting exactly at mid
    // through the branch below. The two agree at that point — both are
    // exactly 0.5 — for every mid-stick but 1000, where the branch below
    // divides by zero and returns NaN. See DIVERGENCES.md D-026.
    let throttle_in = if throttle_control <= mid_stick {
        f32::from(throttle_control) * 0.5 / f32::from(mid_stick)
    } else {
        0.5 + f32::from(throttle_control - mid_stick) * 0.5 / f32::from(1000 - mid_stick)
    };

    let expo = constrain_value(-(throttle_hover - 0.5) / 0.375, -0.5, 1.0);
    throttle_in * (1.0 - expo) + expo * throttle_in * throttle_in * throttle_in
}
