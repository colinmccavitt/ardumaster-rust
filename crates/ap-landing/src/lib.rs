//! Fixed-wing landing, upstream `libraries/AP_Landing`. FW-029.
//!
//! This slice is the slope geometry: where to aim, how steeply to descend, and
//! how far along the approach the vehicle is. Deepstall landing, the abort and
//! go-around paths, rangefinder slope correction and the servo overrides are
//! not here.
//!
//! # The shape of a slope landing
//!
//! The vehicle descends a straight line to a point *short of* the runway
//! threshold, then flares. The flare is where the sink rate is bled off, and
//! its length depends on how fast the vehicle is descending when it starts —
//! so the aim point cannot be chosen without first working out the descent.
//! That circularity is why [`setup_landing_glide_slope`] computes a sink rate,
//! uses it to size the flare, and only then places the aim point.
//!
//! # The aim point is 500 m past the runway
//!
//! Having found the flare point, upstream projects 500 m *beyond* it along the
//! same slope and aims there. The vehicle therefore always has a target below
//! and ahead of it rather than one it arrives at and overshoots, and the
//! altitude controller is given a proportion along a line rather than a
//! distance to a point.
//!
//! # No callbacks
//!
//! Upstream writes its result through two function pointers the vehicle
//! supplies (`set_target_altitude_proportion`, `constrain_target_altitude_
//! location`) and reads two more values off the AHRS and TECS. Per ADR-0004
//! this returns what it computed and lets the caller apply it; the inputs
//! arrive as [`SlopeInputs`]. Nothing is lost — the callbacks are called once
//! each, at the end, with values this returns.

#![no_std]

pub mod deepstall;
pub mod go_around;
pub mod slope_stage;

use ap_math::location::{AltContext, AltFrame, Location};
use ap_math::scalar::{constrain_value, is_zero};

/// Metres past the landing point the aim point is projected, upstream's local
/// `land_projection`.
pub const LAND_PROJECTION_M: f32 = 500.0;

/// Tuning for the slope landing, upstream's `AP_Landing` parameters.
#[derive(Debug, Clone, Copy)]
pub struct SlopeConfig {
    /// Seconds of flare, upstream `LAND_FLARE_SEC`. Multiplied by the sink
    /// rate to give the height to aim for.
    pub flare_sec: f32,
    /// Flare altitude, metres, upstream `LAND_FLARE_ALT`. Used as a floor
    /// when the computed aim height is not positive, and as a ceiling at
    /// twice its value.
    pub flare_alt: f32,
    /// How much of the flare's sink rate comes from TECS rather than from the
    /// approach, percent. Upstream `LAND_FLARE_EFFECT`.
    pub flare_effectivness_pct: u8,
}

/// What the vehicle knows when the slope is set up.
#[derive(Debug, Clone, Copy)]
pub struct SlopeInputs {
    /// The waypoint the approach starts from.
    pub prev_wp: Location,
    /// The landing point.
    pub next_wp: Location,
    /// Where the vehicle is now.
    pub current: Location,
    /// Ground speed, m/s, upstream `ahrs.groundspeed()`.
    pub groundspeed: f32,
    /// The sink rate TECS will hold at touchdown, m/s. Upstream
    /// `tecs_Controller->get_land_sinkrate()`.
    pub land_sinkrate: f32,
    /// Home, origin and terrain altitudes, for resolving waypoint altitudes
    /// to AMSL.
    pub alt_ctx: AltContext,
}

/// The slope, and what the vehicle should do with it.
#[derive(Debug, Clone, Copy)]
pub struct SlopeResult {
    /// The point to aim at: 500 m past the landing point, on the slope.
    /// Upstream passes this to both of its callbacks.
    pub aim_point: Location,
    /// Descent per metre travelled. Positive means descending.
    pub slope: f32,
    /// Altitude offset for the target-altitude controller, cm. Upstream
    /// `target_altitude_offset_cm`.
    pub target_altitude_offset_cm: i32,
    /// How far along the approach the vehicle is: 0 at the previous waypoint,
    /// 1 at the aim point.
    pub land_proportion: f32,
    /// What upstream passes to `set_target_altitude_proportion`, which is
    /// `1 - land_proportion`.
    pub altitude_proportion: f32,
    /// Distance from the landing point back to the flare, metres.
    pub flare_distance: f32,
    /// Height above the landing point the flare begins at, metres.
    pub aim_height: f32,
    /// Whether this was the first calculation. Upstream announces the slope
    /// angle to the ground station exactly once, on this.
    pub first_calculation: bool,
}

/// A location's altitude as centimetres AMSL, upstream
/// `AP_Landing::loc_alt_AMSL_cm`.
///
/// `None` where upstream raises an internal error and returns `loc.alt`
/// unconverted — see [`setup_landing_glide_slope`] for why that is not
/// reproduced.
#[must_use]
pub fn loc_alt_amsl_cm(loc: Location, ctx: &AltContext) -> Option<i32> {
    if let Some(cm) = loc.get_alt_cm(AltFrame::Absolute, ctx) {
        return Some(cm);
    }
    if loc.alt_frame() == AltFrame::AboveTerrain {
        // No terrain data, so assume the ground is as flat as it is at home.
        // Upstream does the same, and it is a real approximation rather than
        // a fallback: over a runway it is usually true.
        return Some(loc.alt.saturating_add(ctx.home_alt_cm?));
    }
    None
}

/// Work out the glide slope and where to aim, upstream
/// `type_slope_setup_landing_glide_slope`.
///
/// `slope` is the landing's persistent slope, read to detect the first
/// calculation and written with the new value.
///
/// # Returns `None` when a waypoint altitude cannot be resolved
///
/// DIVERGENCE D-024: upstream's `loc_alt_AMSL_cm` raises `INTERNAL_ERROR` and
/// then returns `loc.alt` — the raw field, in whatever frame it was stored in.
/// The glide slope is computed from it regardless, so a waypoint whose
/// altitude is above home and whose home is not set produces a slope built
/// from a number whose datum is unknown. Reporting it is the only honest
/// answer: without the waypoint altitudes there is no glide slope, and a wrong
/// one is worse than none on an approach.
///
/// # Panics
///
/// Never. Every divisor below is floored before use, which is upstream's
/// arrangement and the reason for each floor is given at the point it is
/// applied.
#[must_use]
pub fn setup_landing_glide_slope(
    cfg: &SlopeConfig,
    inp: &SlopeInputs,
    slope: &mut f32,
) -> Option<SlopeResult> {
    let mut total_distance = inp.prev_wp.get_distance(inp.next_wp) as f32;

    // A LAND command left at all zeros gives a distance of zero and a division
    // by it a moment later. Upstream guards the input rather than the
    // division, which also keeps the slope finite.
    if total_distance < 1.0 {
        total_distance = 1.0;
    }

    let prev_alt = loc_alt_amsl_cm(inp.prev_wp, &inp.alt_ctx)?;
    let next_alt = loc_alt_amsl_cm(inp.next_wp, &inp.alt_ctx)?;

    // Height to lose over this leg, metres.
    let sink_height = (prev_alt - next_alt) as f32 * 0.01;

    // Floored so a stationary vehicle does not divide by nothing. Half a metre
    // per second is slower than any fixed-wing aircraft flies.
    let mut groundspeed = inp.groundspeed;
    if groundspeed < 0.5 {
        groundspeed = 0.5;
    }

    let mut sink_time = total_distance / groundspeed;
    if sink_time < 0.5 {
        sink_time = 0.5;
    }

    let sink_rate = sink_height / sink_time;

    // The height to aim for is the one that puts the flare in the right place.
    let mut aim_height = cfg.flare_sec * sink_rate;
    if aim_height <= 0.0 {
        // Level or climbing: the sink rate says nothing useful, so fall back
        // to the configured flare altitude.
        aim_height = cfg.flare_alt;
    }
    if cfg.flare_alt > 0.0 && aim_height > cfg.flare_alt * 2.0 {
        aim_height = cfg.flare_alt * 2.0;
    }

    // Time in the flare, assuming the sink rate falls from its approach value
    // to TECS's touchdown value. The weighting is how much of that TECS is
    // expected to achieve.
    let weight = constrain_value(0.01 * f32::from(cfg.flare_effectivness_pct), 0.0, 1.0);
    let flare_sink_rate_avg = (weight * inp.land_sinkrate + (1.0 - weight) * sink_rate).max(0.1);
    let flare_time = aim_height / flare_sink_rate_avg;

    // Ground distance the flare covers. Using ground speed rather than
    // airspeed takes the wind into account without measuring it.
    let mut flare_distance = groundspeed * flare_time;
    if flare_distance > total_distance * 0.5 {
        // Never flare before half way down the final leg.
        flare_distance = total_distance * 0.5;
    }

    let land_bearing_cd = inp.prev_wp.get_bearing_to(inp.next_wp);

    // The aim point: back from the landing point by the flare distance, and up
    // by the aim height.
    let mut loc = inp.next_wp;
    if !loc.change_alt_frame(AltFrame::Absolute, &inp.alt_ctx) {
        return None;
    }
    loc.offset_bearing(
        f64::from(land_bearing_cd) as ap_math::Ftype * 0.01,
        -flare_distance as ap_math::Ftype,
    );
    // Upstream is `loc.alt += aim_height*100` on an int32, so the truncation
    // is on the sum rather than on the offset -- the same promotion as
    // Location::offset_up_m. Converting the offset first differs by a
    // centimetre for negative fractional offsets, and while aim_height is
    // never negative, matching the arithmetic exactly costs nothing and stops
    // this becoming the one place that rounds differently.
    #[allow(
        clippy::cast_possible_truncation,
        clippy::cast_precision_loss,
        reason = "reproduces upstream's int32 += float promotion exactly"
    )]
    {
        loc.alt = (loc.alt as f32 + aim_height * 100.0) as i32;
    }

    let first_calculation = is_zero(*slope);
    *slope = (sink_height - aim_height) / (total_distance - flare_distance);

    // Project along the slope, 500 m past the landing point.
    loc.offset_bearing(
        f64::from(land_bearing_cd) as ap_math::Ftype * 0.01,
        LAND_PROJECTION_M as ap_math::Ftype,
    );
    loc.offset_up_m(-*slope * LAND_PROJECTION_M);

    let target_altitude_offset_cm = loc.alt.saturating_sub(prev_alt);
    let land_proportion = inp.current.line_path_proportion(inp.prev_wp, loc);

    Some(SlopeResult {
        aim_point: loc,
        slope: *slope,
        target_altitude_offset_cm,
        land_proportion,
        altitude_proportion: 1.0 - land_proportion,
        flare_distance,
        aim_height,
        first_calculation,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> SlopeConfig {
        SlopeConfig {
            flare_sec: 2.0,
            flare_alt: 3.0,
            flare_effectivness_pct: 50,
        }
    }

    /// A 1 km approach descending 100 m, at 20 m/s.
    fn approach() -> SlopeInputs {
        let prev = Location::new_with_alt(0, 0, 10_000, AltFrame::Absolute);
        let mut next = prev;
        next.offset(1000.0, 0.0);
        next.set_alt_cm(0, AltFrame::Absolute);

        SlopeInputs {
            prev_wp: prev,
            next_wp: next,
            current: prev,
            groundspeed: 20.0,
            land_sinkrate: 1.0,
            alt_ctx: AltContext {
                home_alt_cm: Some(0),
                origin_alt_cm: Some(0),
                terrain_alt_cm: Some(0),
            },
        }
    }

    #[test]
    fn a_descending_approach_gives_a_descending_slope() {
        let mut slope = 0.0;
        let r = setup_landing_glide_slope(&cfg(), &approach(), &mut slope).expect("resolvable");

        assert!(r.slope > 0.0, "descending, so positive: {}", r.slope);
        // 100 m over about 1 km
        assert!(
            (0.08..0.12).contains(&r.slope),
            "about a ten to one glide, got {}",
            r.slope
        );
        assert!(r.first_calculation, "the slope started at zero");
    }

    /// The aim point sits below the landing point, because it is 500 m past it
    /// on a descending slope.
    #[test]
    fn the_aim_point_is_beyond_and_below_the_landing_point() {
        let inp = approach();
        let mut slope = 0.0;
        let r = setup_landing_glide_slope(&cfg(), &inp, &mut slope).expect("resolvable");

        assert!(
            r.aim_point.alt < 0,
            "500 m past the threshold on a descending slope is below it, got {}",
            r.aim_point.alt
        );
        let d = inp.next_wp.get_distance(r.aim_point) as f32;
        assert!(
            (400.0..600.0).contains(&d),
            "roughly 500 m past the landing point, got {d}"
        );
    }

    /// The second call is not the first.
    #[test]
    fn only_the_first_calculation_is_flagged() {
        let inp = approach();
        let mut slope = 0.0;
        let a = setup_landing_glide_slope(&cfg(), &inp, &mut slope).expect("resolvable");
        assert!(a.first_calculation);
        let b = setup_landing_glide_slope(&cfg(), &inp, &mut slope).expect("resolvable");
        assert!(!b.first_calculation);
    }

    /// A LAND command left at all zeros must not divide by zero.
    #[test]
    fn coincident_waypoints_do_not_divide_by_zero() {
        let mut inp = approach();
        inp.next_wp = inp.prev_wp;
        let mut slope = 0.0;
        let r = setup_landing_glide_slope(&cfg(), &inp, &mut slope).expect("resolvable");
        assert!(r.slope.is_finite(), "slope {}", r.slope);
        assert!(r.flare_distance.is_finite());
    }

    /// A stationary vehicle must not divide by zero either.
    #[test]
    fn zero_groundspeed_does_not_divide_by_zero() {
        let mut inp = approach();
        inp.groundspeed = 0.0;
        let mut slope = 0.0;
        let r = setup_landing_glide_slope(&cfg(), &inp, &mut slope).expect("resolvable");
        assert!(r.slope.is_finite() && r.flare_distance.is_finite());
    }

    /// The flare never starts before half way down the final leg, however long
    /// the computed flare would be.
    #[test]
    fn the_flare_never_starts_before_half_way() {
        let mut inp = approach();
        // Very fast and a long flare: the uncapped distance would exceed the leg.
        inp.groundspeed = 200.0;
        let mut slope = 0.0;
        let r = setup_landing_glide_slope(
            &SlopeConfig {
                flare_sec: 20.0,
                flare_alt: 100.0,
                flare_effectivness_pct: 0,
            },
            &inp,
            &mut slope,
        )
        .expect("resolvable");

        let total = inp.prev_wp.get_distance(inp.next_wp) as f32;
        assert!(
            r.flare_distance <= total * 0.5 + 0.01,
            "flare {} of a {total} m leg",
            r.flare_distance
        );
    }

    /// A level approach has no sink rate to derive an aim height from, so the
    /// configured flare altitude is used instead.
    #[test]
    fn a_level_approach_falls_back_to_the_flare_altitude() {
        let mut inp = approach();
        inp.next_wp.set_alt_cm(10_000, AltFrame::Absolute); // same as prev
        let mut slope = 0.0;
        let r = setup_landing_glide_slope(&cfg(), &inp, &mut slope).expect("resolvable");
        assert!(
            (r.aim_height - 3.0).abs() < 1e-4,
            "expected the flare altitude, got {}",
            r.aim_height
        );
    }

    /// The aim height is capped at twice the flare altitude, so a steep
    /// approach does not aim absurdly high.
    #[test]
    fn the_aim_height_is_capped_at_twice_the_flare_altitude() {
        let mut inp = approach();
        inp.prev_wp.set_alt_cm(100_000, AltFrame::Absolute); // 1 km up
        let mut slope = 0.0;
        let r = setup_landing_glide_slope(&cfg(), &inp, &mut slope).expect("resolvable");
        assert!(
            (r.aim_height - 6.0).abs() < 1e-4,
            "expected twice the 3 m flare altitude, got {}",
            r.aim_height
        );
    }

    /// D-024. Without the waypoint altitudes there is no glide slope.
    /// Upstream computes one anyway, from a number whose datum is unknown.
    #[test]
    fn d024_an_unresolvable_altitude_has_no_slope() {
        let mut inp = approach();
        inp.prev_wp.set_alt_cm(10_000, AltFrame::AboveHome);
        inp.alt_ctx = AltContext::default(); // home not set
        let mut slope = 0.0;
        assert!(setup_landing_glide_slope(&cfg(), &inp, &mut slope).is_none());
    }

    /// The proportion runs from the previous waypoint to the aim point, and
    /// the altitude proportion is its complement.
    #[test]
    fn the_proportions_are_complementary() {
        let inp = approach();
        let mut slope = 0.0;
        let r = setup_landing_glide_slope(&cfg(), &inp, &mut slope).expect("resolvable");
        assert!(
            (r.land_proportion + r.altitude_proportion - 1.0).abs() < 1e-6,
            "{} and {}",
            r.land_proportion,
            r.altitude_proportion
        );
        assert!(
            r.land_proportion.abs() < 1e-3,
            "the vehicle is at the previous waypoint, so zero; got {}",
            r.land_proportion
        );
    }
}
