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
