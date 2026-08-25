//! Pilot conversions, tested against what upstream's source does.
//!
//! Not parity tests — see the module documentation on
//! `ap_copter::pilot_input` for why there is no recording behind these, and
//! what would be needed to get one. They are written to pin the *decisions* in
//! the code rather than to restate its arithmetic, so that a port which
//! computed the right numbers by a different route would still pass and one
//! that lost a decision would not.

#![allow(
    clippy::float_cmp,
    reason = "these comparisons are exact on purpose: a clamped stick must give bit-identical output to the value it clamps to, and a fallback must be indistinguishable from the value it falls back on. A tolerance would let a near-miss through, which is the defect."
)]

use ap_copter::pilot_input::{pilot_desired_throttle, pilot_desired_yaw_rate_rads};

/// Mid-stick is half throttle wherever mid-stick physically sits.
///
/// The map is two straight lines rather than one: the bottom half of the
/// stick's travel spans 0 to 0.5 of throttle and the top half spans 0.5 to 1.
/// So moving the trim changes the *slope* of each half rather than shifting
/// the whole curve, and a pilot who trims low gets finer control below mid and
/// coarser above — which is the region they hover in.
#[test]
fn mid_stick_is_half_throttle_wherever_mid_sits() {
    // With no shaping (hover at half), mid-stick is exactly half throttle.
    for mid in [100_i16, 300, 500, 700, 900] {
        let out = pilot_desired_throttle(mid, mid, 0.5);
        assert!(
            (out - 0.5).abs() < 1e-6,
            "mid-stick at {mid} gave {out}, not half throttle"
        );
    }

    // And the halves have different slopes when mid is off-centre.
    let low_mid = 250_i16;
    let below = pilot_desired_throttle(125, low_mid, 0.5);
    let above = pilot_desired_throttle(625, low_mid, 0.5);
    assert!(
        (below - 0.25).abs() < 1e-6,
        "half way up the lower half should be a quarter throttle, got {below}"
    );
    assert!(
        (above - 0.75).abs() < 1e-6,
        "half way up the upper half should be three quarters, got {above}"
    );
}

/// The expo strength comes from the hover throttle, not from a setting.
///
/// An aircraft hovering at half throttle gets no shaping. One hovering *low* —
/// a powerful airframe — gets positive expo, flattening the curve near centre
/// so it has finer control where it spends its time. One hovering high gets
/// negative expo, steepening it there.
#[test]
fn the_expo_follows_the_hover_throttle() {
    let stick = 250_i16; // a quarter of the way up

    let neutral = pilot_desired_throttle(stick, 500, 0.5);
    let powerful = pilot_desired_throttle(stick, 500, 0.2);
    let marginal = pilot_desired_throttle(stick, 500, 0.8);

    assert!(
        (neutral - 0.25).abs() < 1e-6,
        "hovering at half throttle should shape nothing, got {neutral}"
    );
    assert!(
        powerful < neutral,
        "a low hover throttle should flatten the curve near centre: \
         {powerful} against {neutral}"
    );
    assert!(
        marginal > neutral,
        "a high hover throttle should steepen it: {marginal} against {neutral}"
    );
}

/// The expo bounds are asymmetric, and deliberately so.
///
/// Minus a half to plus one. A very powerful aircraft benefits from a lot of
/// softening; a marginal one cannot afford much sharpening before the stick
/// becomes twitchy at exactly the point it needs to be precise. So the two
/// extremes saturate at different distances from neutral.
#[test]
fn the_expo_bounds_are_asymmetric() {
    let stick = 250_i16;

    // Beyond the bounds the answer stops moving.
    let very_powerful = pilot_desired_throttle(stick, 500, 0.0);
    let also_very_powerful = pilot_desired_throttle(stick, 500, 0.05);
    assert!(
        (very_powerful - also_very_powerful).abs() < 1e-6,
        "past the positive bound the shaping should saturate"
    );

    let very_marginal = pilot_desired_throttle(stick, 500, 1.0);
    let also_very_marginal = pilot_desired_throttle(stick, 500, 0.95);
    assert!(
        (very_marginal - also_very_marginal).abs() < 1e-6,
        "past the negative bound the shaping should saturate"
    );

    // And they saturate at different distances from the neutral hover, which
    // is what "asymmetric" means here: +1.0 is reached at a hover of 0.125,
    // -0.5 only at 0.6875.
    let at_positive_bound = pilot_desired_throttle(stick, 500, 0.125);
    let past_it = pilot_desired_throttle(stick, 500, 0.1);
    assert!((at_positive_bound - past_it).abs() < 1e-6);

    let at_negative_bound = pilot_desired_throttle(stick, 500, 0.6875);
    let past_negative = pilot_desired_throttle(stick, 500, 0.7);
    assert!((at_negative_bound - past_negative).abs() < 1e-6);
}

/// A mid-stick at or below zero falls back to 500 rather than dividing.
#[test]
fn a_degenerate_mid_stick_falls_back() {
    let fallback = pilot_desired_throttle(250, 0, 0.5);
    let explicit = pilot_desired_throttle(250, 500, 0.5);
    assert!(
        (fallback - explicit).abs() < 1e-9,
        "a zero mid should behave as 500, got {fallback} against {explicit}"
    );
    assert!(
        (pilot_desired_throttle(250, -100, 0.5) - explicit).abs() < 1e-9,
        "a negative mid should too"
    );
}

/// The stick is clamped before the map, so out-of-range input saturates
/// rather than extrapolating.
#[test]
fn out_of_range_stick_saturates() {
    assert_eq!(
        pilot_desired_throttle(-200, 500, 0.5),
        pilot_desired_throttle(0, 500, 0.5)
    );
    assert_eq!(
        pilot_desired_throttle(1200, 500, 0.5),
        pilot_desired_throttle(1000, 500, 0.5)
    );
}

/// Yaw: the expo shapes the stick, then the rate scales the result.
///
/// That ordering is what makes the expo mean "sensitivity around centre"
/// rather than something that changes with the rate setting. A pilot who
/// raises their maximum yaw rate gets a proportionally faster response
/// everywhere, with the same feel near centre.
#[test]
fn the_yaw_expo_shapes_the_stick_not_the_rate() {
    let stick = 0.3;
    let expo = 0.5;

    let slow = pilot_desired_yaw_rate_rads(stick, 45.0, expo, true);
    let fast = pilot_desired_yaw_rate_rads(stick, 90.0, expo, true);

    assert!(
        (fast - slow * 2.0).abs() < 1e-6,
        "doubling the rate should double the output exactly: {fast} against {slow}"
    );

    // Full stick reaches the configured rate regardless of expo, because the
    // shaping passes ±1 through unchanged.
    for e in [-0.5_f32, 0.0, 0.5, 0.94] {
        let full = pilot_desired_yaw_rate_rads(1.0, 90.0, e, true);
        assert!(
            (full - 90.0_f32.to_radians()).abs() < 1e-5,
            "full stick with expo {e} gave {full}, not the configured rate"
        );
    }
}

/// No valid radio input means neutral, not the last stick position.
#[test]
fn an_invalid_radio_gives_neutral() {
    assert_eq!(pilot_desired_yaw_rate_rads(1.0, 90.0, 0.0, false), 0.0);
    assert_ne!(pilot_desired_yaw_rate_rads(1.0, 90.0, 0.0, true), 0.0);
}
