//! The slope landing's stage machine and the decisions that hang off it,
//! upstream `AP_Landing`'s `type_slope_*` predicates.

use ap_math::scalar::constrain_value;

/// Where the aircraft is in a slope landing, upstream `SlopeStage`.
///
/// Strictly ordered — an approach only ever moves forward through these — and
/// the discriminants match upstream's because they are logged.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum SlopeStage {
    /// Not yet landing.
    Normal = 0,
    /// Descending the glide slope.
    Approach = 1,
    /// Committed, close in, not yet flaring.
    Preflare = 2,
    /// Flaring.
    Final = 3,
}

impl SlopeStage {
    /// Whether the aircraft is flaring, upstream `type_slope_is_flaring`.
    #[must_use]
    pub fn is_flaring(self) -> bool {
        self == Self::Final
    }

    /// Whether the aircraft is on final, upstream `type_slope_is_on_final`.
    ///
    /// Preflare *and* final. The name suggests the last stage alone; it means
    /// committed — past the point where the approach would be flown again.
    #[must_use]
    pub fn is_on_final(self) -> bool {
        matches!(self, Self::Preflare | Self::Final)
    }

    /// Whether the aircraft is on approach, upstream
    /// `type_slope_is_on_approach`.
    ///
    /// Approach and preflare, so it overlaps [`Self::is_on_final`] at
    /// preflare. The two are not a partition: preflare is both the end of the
    /// approach and the beginning of the commitment, and callers of each ask
    /// different questions about it.
    #[must_use]
    pub fn is_on_approach(self) -> bool {
        matches!(self, Self::Approach | Self::Preflare)
    }

    /// Whether ground contact is expected, upstream
    /// `type_slope_is_expecting_impact`.
    ///
    /// Exactly [`Self::is_on_final`] upstream, which is worth keeping separate
    /// rather than collapsing: they answer different questions and only
    /// coincide for this landing type.
    #[must_use]
    pub fn is_expecting_impact(self) -> bool {
        self.is_on_final()
    }

    /// Whether the landing is complete, upstream `type_slope_is_complete`.
    ///
    /// True at flare, not at touchdown — this landing type has no sense of
    /// having stopped, only of having committed to the last manoeuvre.
    #[must_use]
    pub fn is_complete(self) -> bool {
        self == Self::Final
    }

    /// Bound the roll command during the flare, upstream
    /// `type_slope_constrain_roll`.
    ///
    /// Only at flare, and this is the one place the stage machine reaches into
    /// the attitude command. Wings level matters more than tracking at the
    /// moment of touchdown: a wing down puts a tip into the ground.
    #[must_use]
    pub fn constrain_roll(self, desired_roll_cd: i32, level_roll_limit_cd: i32) -> i32 {
        if self == Self::Final {
            desired_roll_cd.clamp(-level_roll_limit_cd, level_roll_limit_cd)
        } else {
            desired_roll_cd
        }
    }
}

/// The airspeed parameters this decision reads.
#[derive(Debug, Clone, Copy)]
pub struct LandingAirspeedParams {
    /// Cruise airspeed, m/s. Upstream `aparm.airspeed_cruise`.
    pub airspeed_cruise_ms: f32,
    /// Minimum airspeed, m/s. Upstream `aparm.airspeed_min`.
    pub airspeed_min_ms: f32,
    /// Maximum airspeed, m/s. Upstream `aparm.airspeed_max`.
    pub airspeed_max_ms: f32,
    /// TECS's landing airspeed, m/s. Negative means unset.
    pub land_airspeed_ms: f32,
    /// Pre-flare airspeed, m/s. Non-positive means unset. Upstream
    /// `pre_flare_airspeed`.
    pub pre_flare_airspeed_ms: f32,
    /// Percentage of head wind to add, upstream `wind_comp`.
    pub wind_comp_pct: f32,
    /// Whether the maximum airspeed may be used on landing, upstream
    /// `allow_max_airspeed_on_land`.
    pub allow_max_airspeed: bool,
}

/// The target airspeed for the current stage, in centimetres per second,
/// upstream `type_slope_get_target_airspeed_cm`.
///
/// # The base speed
///
/// TECS's landing airspeed wins when it is set. Otherwise the fallback is the
/// *mean* of cruise and minimum — not cruise, and not minimum. Landing wants
/// slower than cruise for a shorter roll-out, but a margin above minimum
/// because an approach is exactly where a stall is unrecoverable.
///
/// # Then the stage overrides it
///
/// `Normal` resets to plain cruise, discarding the landing speed entirely: the
/// aircraft is not landing yet. `Approach` keeps whatever the base gave.
/// `Preflare` and `Final` take the pre-flare speed if one is configured, and
/// the comment upstream is explicit that final deliberately keeps using the
/// pre-flare value rather than choosing its own.
///
/// # Head wind
///
/// Half the head wind by default, added on. Ground speed is what the runway
/// sees, so flying an airspeed into a head wind arrives slower over the
/// ground — adding some of it back keeps the approach from becoming
/// interminable, while adding only part of it keeps the airspeed margin.
///
/// The final constraint's *lower* bound is the target itself, so the head-wind
/// term can only ever add. In a tail wind `head_wind` goes negative, and
/// without that floor the aircraft would be told to fly slower than its
/// landing speed on the approach where it already has the least margin.
#[must_use]
pub fn target_airspeed_cm(
    stage: SlopeStage,
    params: &LandingAirspeedParams,
    head_wind_ms: f32,
) -> i32 {
    let mut target_airspeed_cm = if params.land_airspeed_ms >= 0.0 {
        params.land_airspeed_ms * 100.0
    } else {
        100.0 * 0.5 * (params.airspeed_cruise_ms + params.airspeed_min_ms)
    } as i32;

    match stage {
        SlopeStage::Normal => {
            target_airspeed_cm = (params.airspeed_cruise_ms * 100.0) as i32;
        }
        SlopeStage::Approach => {}
        SlopeStage::Preflare | SlopeStage::Final => {
            if params.pre_flare_airspeed_ms > 0.0 {
                target_airspeed_cm = (params.pre_flare_airspeed_ms * 100.0) as i32;
            }
        }
    }

    let head_wind_comp = constrain_value(params.wind_comp_pct, 0.0, 100.0) * 0.01;
    let head_wind_compensation_cm = (head_wind_ms * head_wind_comp * 100.0) as i32;

    let max_airspeed_cm = if params.allow_max_airspeed {
        params.airspeed_max_ms * 100.0
    } else {
        params.airspeed_cruise_ms * 100.0
    } as i32;

    constrain_i32(
        target_airspeed_cm + head_wind_compensation_cm,
        target_airspeed_cm,
        max_airspeed_cm,
    )
}

/// Upstream `constrain_int32`, which is not `i32::clamp`.
///
/// The low bound is tested first, so with the bounds crossed this returns the
/// *low* one where `clamp` would panic. That is not a hypothetical here: if
/// TECS supplies a landing airspeed above cruise and the maximum is not
/// allowed on landing, the target exceeds its own ceiling and upstream flies
/// the target.
fn constrain_i32(amount: i32, low: i32, high: i32) -> i32 {
    if amount < low {
        return low;
    }
    if amount > high {
        return high;
    }
    amount
}

/// What the stage machine looks at, upstream's locals in
/// `type_slope_verify_land`.
#[derive(Debug, Clone, Copy)]
pub struct TransitionInputs {
    /// How far along the leg, 0 at the previous waypoint and 1 at the aim
    /// point. Upstream `wp_proportion`.
    pub wp_proportion: f32,
    /// Height above the landing point, metres.
    pub height: f32,
    /// Current sink rate, metres per second, positive downward.
    pub sink_rate: f32,
    /// Heading error to the runway, centidegrees. Upstream
    /// `nav_controller->bearing_error_cd()`.
    pub bearing_error_cd: i32,
    /// Crosstrack error, metres.
    pub crosstrack_error_m: f32,
    /// Whether the navigation solution is stale.
    pub nav_data_is_stale: bool,
    /// Whether the vehicle is below the previous waypoint's altitude.
    pub below_prev_wp: bool,
    /// Whether the previous mission command was a loiter-to-altitude.
    pub prev_cmd_is_loiter_to_alt: bool,
    /// Whether the rangefinder is giving usable height.
    pub rangefinder_in_range: bool,
    /// Whether the vehicle believes it is flying.
    pub is_flying: bool,
    /// Whether crash detection is enabled, upstream
    /// `aparm.crash_detection_enable`.
    pub crash_detection_enable: bool,
}

/// The flare and pre-flare thresholds.
#[derive(Debug, Clone, Copy)]
pub struct FlareConfig {
    /// Flare altitude, metres. Upstream `flare_alt`.
    pub flare_alt: f32,
    /// Flare time, seconds. Zero disables the sink-rate test. Upstream
    /// `flare_sec`.
    pub flare_sec: f32,
    /// Pre-flare altitude, metres. Non-positive disables it.
    pub pre_flare_alt: f32,
    /// Pre-flare time, seconds. Non-positive disables it.
    pub pre_flare_sec: f32,
    /// Pre-flare airspeed. Non-positive disables the whole pre-flare stage.
    pub pre_flare_airspeed: f32,
}

impl SlopeStage {
    /// Advance the stage, upstream the state machine inside
    /// `type_slope_verify_land`.
    ///
    /// # Normal to approach: four independent ways in
    ///
    /// A loiter-to-altitude before the landing counts on its own, because the
    /// aircraft has already been positioned deliberately. Otherwise it needs
    /// heading *and* crosstrack together, or heading and being below the
    /// previous waypoint once past fifteen percent of the leg, or simply
    /// being past halfway.
    ///
    /// That last one has no quality test at all. It is the backstop: past the
    /// midpoint the aircraft is committed whether or not it ever lined up,
    /// and refusing to enter the approach would leave it descending with the
    /// approach logic switched off.
    ///
    /// Both the heading and crosstrack tests require fresh navigation data.
    /// The below-previous-waypoint test does not — it reads altitude, which
    /// the navigation controller does not supply.
    ///
    /// # Into the flare: three ways, and one of them is a crash
    ///
    /// Below the flare altitude, or within the flare time by the current sink
    /// rate — but both of those require being *on approach* first, and the
    /// sink-rate one additionally requires being past halfway. Upstream's
    /// comment explains why: with the thresholds set large, an aircraft on a
    /// hard turn to line up would otherwise flare early, and the flare's roll
    /// limits would then make it hard to line up at all.
    ///
    /// The third way needs neither: past the landing point with no
    /// rangefinder. That is the baro-drift case — the aircraft may already be
    /// on the ground while the barometer still reports height.
    ///
    /// The fourth is "probably crashed": crash detection enabled, almost no
    /// sink rate, and not flying. Flaring shuts the motor down, which is the
    /// point.
    ///
    /// # Pre-flare only from approach
    ///
    /// And only when a pre-flare airspeed is configured — the stage exists to
    /// change speed, so without one there is nothing for it to do.
    #[must_use]
    pub fn next(self, inp: &TransitionInputs, cfg: &FlareConfig) -> Self {
        let mut stage = self;

        if stage == Self::Normal {
            let heading_lined_up = inp.bearing_error_cd.abs() < 1000 && !inp.nav_data_is_stale;
            let on_flight_line = inp.crosstrack_error_m.abs() < 5.0 && !inp.nav_data_is_stale;

            if inp.prev_cmd_is_loiter_to_alt
                || (inp.wp_proportion >= 0.0 && heading_lined_up && on_flight_line)
                || (inp.wp_proportion > 0.15 && heading_lined_up && inp.below_prev_wp)
                || (inp.wp_proportion > 0.5)
            {
                stage = Self::Approach;
            }
        }

        // Read after the transition above, so a leg that enters the approach
        // this cycle can also flare this cycle. Upstream reads it at the same
        // point for the same reason.
        let on_approach_stage = stage.is_on_approach();
        let below_flare_alt = inp.height <= cfg.flare_alt;
        let below_flare_sec = cfg.flare_sec > 0.0 && inp.height <= inp.sink_rate * cfg.flare_sec;
        let probably_crashed =
            inp.crash_detection_enable && inp.sink_rate.abs() < 0.2 && !inp.is_flying;

        if (on_approach_stage && below_flare_alt)
            || (on_approach_stage && below_flare_sec && inp.wp_proportion > 0.5)
            || (!inp.rangefinder_in_range && inp.wp_proportion >= 1.0)
            || probably_crashed
        {
            stage = Self::Final;
        } else if stage == Self::Approach && cfg.pre_flare_airspeed > 0.0 {
            let reached_pre_flare_alt = cfg.pre_flare_alt > 0.0 && inp.height <= cfg.pre_flare_alt;
            let reached_pre_flare_sec =
                cfg.pre_flare_sec > 0.0 && inp.height <= inp.sink_rate * cfg.pre_flare_sec;
            if reached_pre_flare_alt || reached_pre_flare_sec {
                stage = Self::Preflare;
            }
        }

        stage
    }
}
