//! TECS tuning parameters, ported from `AP_TECS`'s `var_info` table.
//!
//! Upstream declares these as `AP_Float`/`AP_Int8`/`AP_Int32` members bound to
//! the parameter system. Here they are a plain struct: the values are what the
//! controller reads, and binding them to storage is `AP_Param`'s job (FW-004),
//! not TECS's. That keeps this crate free of the parameter machinery and makes
//! a replay test able to set a parameter set directly.
//!
//! Defaults are upstream's, taken from the `AP_GROUPINFO` table. They matter:
//! a replay whose parameters differ from the recorded flight will diverge for
//! reasons that have nothing to do with the port.

/// Parameter names as they appear to a user, kept alongside each field so the
/// mapping to upstream is greppable in both directions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TecsParams {
    /// `TECS_CLMB_MAX` — maximum climb rate, m/s.
    pub max_climb_rate: f32,
    /// `TECS_SINK_MIN` — minimum sink rate at idle throttle, m/s.
    pub min_sink_rate: f32,
    /// `TECS_SINK_MAX` — maximum sink rate, m/s.
    pub max_sink_rate: f32,
    /// `TECS_TIME_CONST` — controller time constant, s.
    pub time_const: f32,
    /// `TECS_LAND_TCONST` — time constant during landing, s.
    pub land_time_const: f32,
    /// `TECS_PTCH_DAMP` — pitch demand damping.
    pub ptch_damp: f32,
    /// `TECS_LAND_PDAMP` — pitch damping during landing; 0 means use `ptch_damp`.
    pub land_pitch_damp: f32,
    /// `TECS_LAND_DAMP` — landing flare damping.
    pub land_damp: f32,
    /// `TECS_THR_DAMP` — throttle demand damping.
    pub thr_damp: f32,
    /// `TECS_LAND_TDAMP` — throttle damping during landing; 0 means use `thr_damp`.
    pub land_throttle_damp: f32,
    /// `TECS_INTEG_GAIN` — integrator gain.
    pub integ_gain: f32,
    /// `TECS_TKOFF_IGAIN` — integrator gain during takeoff; 0 means use `integ_gain`.
    pub integ_gain_takeoff: f32,
    /// `TECS_LAND_IGAIN` — integrator gain during landing; 0 means use `integ_gain`.
    pub integ_gain_land: f32,
    /// `TECS_VERT_ACC` — vertical acceleration limit, m/s².
    pub vert_acc_lim: f32,
    /// `TECS_HGT_OMEGA` — height complementary filter frequency, rad/s.
    pub hgt_comp_filt_omega: f32,
    /// `TECS_SPD_OMEGA` — speed complementary filter frequency, rad/s.
    pub spd_comp_filt_omega: f32,
    /// `TECS_RLL2THR` — bank-angle to throttle compensation.
    pub roll_comp: f32,
    /// `TECS_SPDWEIGHT` — speed vs height weighting, 0..2.
    pub spd_weight: f32,
    /// `TECS_LAND_SPDWGT` — weighting during landing; negative means automatic.
    pub spd_weight_land: f32,
    /// `TECS_LAND_THR` — throttle during landing, percent; negative means unset.
    pub land_throttle: f32,
    /// `TECS_LAND_ARSPD` — airspeed during landing, m/s; negative means unset.
    pub land_airspeed: f32,
    /// `TECS_LAND_SINK` — sink rate during the flare, m/s.
    pub land_sink: f32,
    /// `TECS_LAND_SRC` — land sink rate change with distance.
    pub land_sink_rate_change: f32,
    /// `TECS_PITCH_MAX` — maximum pitch demand, degrees.
    pub pitch_max: i8,
    /// `TECS_PITCH_MIN` — minimum pitch demand, degrees.
    pub pitch_min: i8,
    /// `TECS_LAND_PMAX` — maximum pitch during landing, degrees.
    pub land_pitch_max: i8,
    /// `TECS_APPR_SMAX` — maximum sink rate on approach, m/s.
    pub max_sink_rate_approach: f32,
    /// `TECS_OPTIONS` — option bitmask.
    pub options: i32,
    /// `TECS_FLARE_HGT` — flare hold-off height, m.
    pub flare_holdoff_hgt: f32,
    /// `TECS_HDEM_TCONST` — height demand time constant, s.
    pub hgt_dem_tconst: f32,
    /// `TECS_PTCH_FF_V0` — pitch feed-forward reference airspeed, m/s.
    pub pitch_ff_v0: f32,
    /// `TECS_PTCH_FF_K` — pitch feed-forward gain.
    pub pitch_ff_k: f32,
    /// `TECS_SYNAIRSPEED` — use synthetic airspeed.
    pub use_synthetic_airspeed: i8,
    /// `TECS_THR_ERATE` — minimum throttle percent for external rate limiting.
    pub thr_min_pct_ext_rate_lim: i8,
}

impl Default for TecsParams {
    /// Upstream's `AP_GROUPINFO` defaults, verbatim.
    fn default() -> Self {
        Self {
            max_climb_rate: 5.0,
            min_sink_rate: 2.0,
            max_sink_rate: 5.0,
            time_const: 5.0,
            land_time_const: 2.0,
            ptch_damp: 0.3,
            land_pitch_damp: 0.0,
            land_damp: 0.5,
            thr_damp: 0.5,
            land_throttle_damp: 0.0,
            integ_gain: 0.3,
            integ_gain_takeoff: 0.0,
            integ_gain_land: 0.0,
            vert_acc_lim: 7.0,
            hgt_comp_filt_omega: 3.0,
            spd_comp_filt_omega: 2.0,
            roll_comp: 10.0,
            spd_weight: 1.0,
            spd_weight_land: -1.0,
            land_throttle: -1.0,
            land_airspeed: -1.0,
            land_sink: 0.25,
            land_sink_rate_change: 0.0,
            pitch_max: 15,
            pitch_min: 0,
            land_pitch_max: 10,
            max_sink_rate_approach: 0.0,
            options: 0,
            flare_holdoff_hgt: 1.0,
            hgt_dem_tconst: 3.0,
            pitch_ff_v0: 12.0,
            pitch_ff_k: 0.0,
            use_synthetic_airspeed: 0,
            thr_min_pct_ext_rate_lim: 20,
        }
    }
}

/// Flight stage, ported from `AP_FixedWing::FlightStage`.
///
/// Discriminants are upstream's, from `AP_Vehicle/AP_FixedWing.h:48`:
///
/// ```text
/// TAKEOFF = 1, VTOL = 2, NORMAL = 3, LAND = 4, ABORT_LANDING = 7
/// ```
///
/// They are **not** sequential and there is no zero variant. The logged `stg`
/// field carries the raw discriminant, so these must match exactly — an
/// earlier version of this enum invented sequential values, which decoded
/// NORMAL (3) as takeoff and would have engaged climbout logic during cruise.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FlightStage {
    /// Takeoff climb-out.
    Takeoff = 1,
    /// VTOL flight.
    Vtol = 2,
    /// Normal cruise.
    Normal = 3,
    /// Landing approach or flare.
    Land = 4,
    /// Abort of a landing. Note the gap: upstream skips 5 and 6.
    AbortLanding = 7,
}

impl FlightStage {
    /// Convert from the logged discriminant, or `None` if unrecognised.
    ///
    /// Returns `Option` rather than defaulting to `Normal`: silently treating
    /// an unknown stage as cruise would change gains and limits without any
    /// signal, and a fixture carrying an unexpected value means the port's
    /// enum has drifted from upstream's.
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            1 => Some(Self::Takeoff),
            2 => Some(Self::Vtol),
            3 => Some(Self::Normal),
            4 => Some(Self::Land),
            7 => Some(Self::AbortLanding),
            _ => None,
        }
    }

    /// Whether this stage is part of a landing, which selects the landing
    /// variants of several gains.
    pub fn is_landing(self) -> bool {
        matches!(self, Self::Land)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;

    /// Defaults must match upstream's AP_GROUPINFO table exactly. A replay run
    /// with different parameters diverges for reasons unrelated to the port,
    /// so these are worth pinning rather than trusting.
    #[test]
    fn defaults_match_upstream_group_info() {
        let p = TecsParams::default();
        assert_eq!(p.max_climb_rate, 5.0, "TECS_CLMB_MAX");
        assert_eq!(p.min_sink_rate, 2.0, "TECS_SINK_MIN");
        assert_eq!(p.max_sink_rate, 5.0, "TECS_SINK_MAX");
        assert_eq!(p.time_const, 5.0, "TECS_TIME_CONST");
        assert_eq!(p.land_time_const, 2.0, "TECS_LAND_TCONST");
        assert_eq!(p.ptch_damp, 0.3, "TECS_PTCH_DAMP");
        assert_eq!(p.land_damp, 0.5, "TECS_LAND_DAMP");
        assert_eq!(p.thr_damp, 0.5, "TECS_THR_DAMP");
        assert_eq!(p.integ_gain, 0.3, "TECS_INTEG_GAIN");
        assert_eq!(p.vert_acc_lim, 7.0, "TECS_VERT_ACC");
        assert_eq!(p.hgt_comp_filt_omega, 3.0, "TECS_HGT_OMEGA");
        assert_eq!(p.spd_comp_filt_omega, 2.0, "TECS_SPD_OMEGA");
        assert_eq!(p.roll_comp, 10.0, "TECS_RLL2THR");
        assert_eq!(p.spd_weight, 1.0, "TECS_SPDWEIGHT");
        assert_eq!(p.spd_weight_land, -1.0, "TECS_LAND_SPDWGT");
        assert_eq!(p.land_sink, 0.25, "TECS_LAND_SINK");
        assert_eq!(p.pitch_max, 15, "TECS_PITCH_MAX");
        assert_eq!(p.pitch_min, 0, "TECS_PITCH_MIN");
        assert_eq!(p.land_pitch_max, 10, "TECS_LAND_PMAX");
        assert_eq!(p.flare_holdoff_hgt, 1.0, "TECS_FLARE_HGT");
        assert_eq!(p.hgt_dem_tconst, 3.0, "TECS_HDEM_TCONST");
        assert_eq!(p.pitch_ff_v0, 12.0, "TECS_PTCH_FF_V0");
        assert_eq!(p.pitch_ff_k, 0.0, "TECS_PTCH_FF_K");
        assert_eq!(p.thr_min_pct_ext_rate_lim, 20, "TECS_THR_ERATE");
    }

    /// Several parameters use a negative or zero sentinel meaning "fall back to
    /// the non-landing value". Reproduced rather than normalised, because the
    /// fallback is applied at the point of use, not at load.
    #[test]
    fn sentinel_defaults_are_preserved() {
        let p = TecsParams::default();
        assert_eq!(p.land_airspeed, -1.0, "negative means unset");
        assert_eq!(p.land_throttle, -1.0, "negative means unset");
        assert_eq!(p.spd_weight_land, -1.0, "negative means automatic");
        // zero means "use the non-landing gain", not "no gain"
        assert_eq!(p.land_pitch_damp, 0.0);
        assert_eq!(p.land_throttle_damp, 0.0);
        assert_eq!(p.integ_gain_takeoff, 0.0);
        assert_eq!(p.integ_gain_land, 0.0);
    }

    /// Discriminants must match upstream, since the logged `stg` field carries
    /// the raw value.
    ///
    /// The literals below are quoted from `AP_Vehicle/AP_FixedWing.h:48` —
    /// TAKEOFF=1, VTOL=2, NORMAL=3, LAND=4, ABORT_LANDING=7 — deliberately as
    /// numbers rather than through the port's own constants.
    ///
    /// An earlier version of this test asserted `FlightStage::Takeoff as u8 ==
    /// 3` using the port's own definition, so it checked the port against
    /// itself and passed while every value was wrong. The replay against real
    /// logged data is what exposed it.
    #[test]
    fn flight_stage_discriminants_match_upstream() {
        assert_eq!(FlightStage::Takeoff as u8, 1);
        assert_eq!(FlightStage::Vtol as u8, 2);
        assert_eq!(FlightStage::Normal as u8, 3);
        assert_eq!(FlightStage::Land as u8, 4);
        assert_eq!(FlightStage::AbortLanding as u8, 7);

        assert_eq!(FlightStage::from_u8(1), Some(FlightStage::Takeoff));
        assert_eq!(FlightStage::from_u8(2), Some(FlightStage::Vtol));
        assert_eq!(FlightStage::from_u8(3), Some(FlightStage::Normal));
        assert_eq!(FlightStage::from_u8(4), Some(FlightStage::Land));
        assert_eq!(FlightStage::from_u8(7), Some(FlightStage::AbortLanding));
    }

    /// The values are not contiguous: 0, 5 and 6 are not stages, and must be
    /// reported rather than decoded as something.
    #[test]
    fn unknown_flight_stage_is_none() {
        assert_eq!(FlightStage::from_u8(0), None, "there is no zero stage");
        assert_eq!(FlightStage::from_u8(5), None, "gap before ABORT_LANDING");
        assert_eq!(FlightStage::from_u8(6), None, "gap before ABORT_LANDING");
        assert_eq!(FlightStage::from_u8(9), None);
        assert_eq!(FlightStage::from_u8(255), None);
    }

    #[test]
    fn only_land_counts_as_landing() {
        assert!(FlightStage::Land.is_landing());
        assert!(!FlightStage::Normal.is_landing());
        assert!(!FlightStage::Takeoff.is_landing());
        assert!(!FlightStage::AbortLanding.is_landing());
    }
}
