//! Pilot stick conversions shared by the manual multirotor modes, upstream
//! `Copter`'s `Mode::get_pilot_desired_*`.
//!
//! Each takes raw stick input and produces a demand for a controller. All of
//! them return neutral when the radio has no valid input — a failsafe with a
//! stale stick position is worse than one with none.
//!
//! # Not parity-tested, and why
//!
//! These are the first ported functions in this port with no recording behind
//! them, so the reason is worth stating.
//!
//! A harness can reach the real methods — Copter's `Mode` instances are in the
//! linked firmware, and the RC channels can be pointed at the singleton's own
//! array. What it cannot do is read the hover throttle: `copter.motors` is a
//! pointer assigned during the vehicle's `setup()`, which a parity harness
//! does not run, and dereferencing it crashes.
//!
//! The tempting shortcut is to transcribe the arithmetic into the harness and
//! sweep that. It would produce a fixture, and the fixture would be worthless:
//! it would compare this port against a C++ copy of itself rather than against
//! the firmware.
//!
//! So the tests below are derived from upstream's source and reason about what
//! it does, which is weaker and is labelled as such. What would fix it is a
//! way to bring a Copter harness far enough up that `motors` is live —
//! `AP_Landing` was in exactly this position until `plane_link` existed.

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
/// # The divide-by-zero guard
///
/// A mid-stick at or below zero falls back to 500. Upstream calls it unlikely
/// rather than impossible, and the fallback is the default rather than
/// something derived — there is nothing sensible to derive it from.
#[must_use]
pub fn pilot_desired_throttle(throttle_control: i16, mid_stick: i16, throttle_hover: f32) -> f32 {
    let mid_stick = if mid_stick <= 0 { 500 } else { mid_stick };
    let throttle_control = throttle_control.clamp(0, 1000);

    let throttle_in = if throttle_control < mid_stick {
        f32::from(throttle_control) * 0.5 / f32::from(mid_stick)
    } else {
        0.5 + f32::from(throttle_control - mid_stick) * 0.5 / f32::from(1000 - mid_stick)
    };

    let expo = constrain_value(-(throttle_hover - 0.5) / 0.375, -0.5, 1.0);
    throttle_in * (1.0 - expo) + expo * throttle_in * throttle_in * throttle_in
}
