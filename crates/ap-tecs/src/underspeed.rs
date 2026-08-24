//! Underspeed detection, ported from `AP_TECS::_detect_underspeed`.
//!
//! Underspeed is a latching condition: TECS enters it readily and leaves it
//! only under stricter conditions, because oscillating in and out of an
//! underspeed response is worse than staying in it a little too long.
//!
//! # The hysteresis is the point
//!
//! Entry: airspeed below **90%** of minimum while throttle is at **95%** or
//! more — the aircraft is asking for everything it has and still slowing.
//!
//! Exit: airspeed above **115%** of minimum **and** at least **3 seconds**
//! since it was last below the entry threshold. Both conditions, not either.
//!
//! Those asymmetric thresholds and the timer are what stop the flag chattering
//! around the boundary, so they are reproduced exactly rather than simplified
//! to a single comparison.
//!
//! Time is a [`Millis`] rather than a global `AP_HAL::millis()` call, per
//! ADR-0004. The elapsed check uses wrapping arithmetic, matching upstream's
//! unsigned subtraction across the ~49-day rollover.

use ap_hal::time::Millis;

use crate::params::FlightStage;

/// Inputs to one underspeed check.
#[derive(Debug, Clone, Copy)]
pub struct UnderspeedInputs {
    /// Current true airspeed estimate, upstream `_TAS_state`.
    pub tas_state: f32,
    /// Minimum true airspeed, upstream `_TASmin`.
    pub tas_min: f32,
    /// Current throttle demand, upstream `_throttle_dem`.
    pub throttle_dem: f32,
    /// Maximum throttle, upstream `_THRmaxf`.
    pub thr_max: f32,
    /// Whether the vehicle is flaring, upstream `_landing.is_flaring()`.
    pub is_flaring: bool,
    /// Current height, upstream `_height`.
    pub height: f32,
    /// Demanded height, upstream `_hgt_dem`.
    pub hgt_dem: f32,
    /// Current flight stage.
    pub flight_stage: FlightStage,
}

/// The latching underspeed detector.
#[derive(Debug, Clone, Copy, Default)]
pub struct UnderspeedDetector {
    /// Whether underspeed is currently latched, upstream `_flags.underspeed`.
    underspeed: bool,
    /// When airspeed was last below the entry threshold, upstream
    /// `_underspeed_start_ms`.
    start_ms: Millis,
}

impl UnderspeedDetector {
    /// A detector in the clear state.
    pub fn new() -> Self {
        Self::default()
    }

    /// Whether underspeed is currently latched.
    pub fn is_underspeed(&self) -> bool {
        self.underspeed
    }

    /// One `_detect_underspeed` step.
    ///
    /// The ordering matters and is upstream's: the clear-condition is evaluated
    /// **first**, then the set-conditions may re-latch it within the same call.
    pub fn update(&mut self, inp: &UnderspeedInputs, now: Millis) -> bool {
        // Clear a previous condition only if well clear of the limit AND it has
        // been clear for a sustained period. Both, not either.
        if self.underspeed && inp.tas_state >= inp.tas_min * 1.15 && now.since(self.start_ms) > 3000
        {
            self.underspeed = false;
        }

        if inp.flight_stage == FlightStage::Vtol {
            // airspeed is not meaningful under VTOL lift
            self.underspeed = false;
        } else if ((inp.tas_state < inp.tas_min * 0.9)
            && (inp.throttle_dem >= inp.thr_max * 0.95)
            && !inp.is_flaring)
            || ((inp.height < inp.hgt_dem) && self.underspeed)
        {
            self.underspeed = true;
            if inp.tas_state < inp.tas_min * 0.9 {
                // still below the threshold, so restart the clear timer
                self.start_ms = now;
            }
        } else {
            // reached the demanded height with throttle below 95% or airspeed
            // above 90% of minimum
            self.underspeed = false;
        }

        self.underspeed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // PORT-DERIVED: upstream ships no AP_TECS unit tests. From reading
    // AP_TECS.cpp:647-676.

    fn nominal() -> UnderspeedInputs {
        UnderspeedInputs {
            tas_state: 20.0,
            tas_min: 10.0,
            throttle_dem: 0.5,
            thr_max: 1.0,
            is_flaring: false,
            height: 100.0,
            hgt_dem: 100.0,
            flight_stage: FlightStage::Normal,
        }
    }

    /// Entry needs BOTH low airspeed and near-max throttle: the aircraft is
    /// asking for everything it has and still slowing.
    #[test]
    fn latches_only_when_slow_and_at_full_throttle() {
        let mut d = UnderspeedDetector::new();
        let mut i = nominal();

        // slow but throttle not committed - not underspeed
        i.tas_state = 8.0;
        i.throttle_dem = 0.5;
        assert!(!d.update(&i, Millis(1000)));

        // throttle committed but not slow - not underspeed
        i.tas_state = 20.0;
        i.throttle_dem = 1.0;
        assert!(!d.update(&i, Millis(2000)));

        // both - underspeed
        i.tas_state = 8.0;
        i.throttle_dem = 1.0;
        assert!(d.update(&i, Millis(3000)));
    }

    /// Flaring suppresses entry: bleeding speed there is intended.
    #[test]
    fn flaring_suppresses_entry() {
        let mut d = UnderspeedDetector::new();
        let mut i = nominal();
        i.tas_state = 8.0;
        i.throttle_dem = 1.0;
        i.is_flaring = true;
        assert!(!d.update(&i, Millis(1000)));
    }

    /// The margin+timer clear path needs speed above 115% of minimum AND 3
    /// seconds elapsed.
    ///
    /// The aircraft is held BELOW its demanded height throughout, which keeps
    /// the set-branch latching. Without that the trailing `else` clears the
    /// flag immediately and this path is never reached - see
    /// `else_branch_clears_without_waiting_for_the_timer`.
    #[test]
    fn exit_requires_both_margin_and_time() {
        let mut d = UnderspeedDetector::new();
        let mut i = nominal();
        i.height = 50.0;
        i.hgt_dem = 100.0;

        i.tas_state = 8.0;
        i.throttle_dem = 1.0;
        assert!(d.update(&i, Millis(1000)));

        // fast enough, but only 1s later - still latched
        i.tas_state = 12.0; // > 10 * 1.15
        i.throttle_dem = 0.5;
        assert!(d.update(&i, Millis(2000)), "3s not yet elapsed");

        // 3s elapsed but not enough margin (11.0 < 11.5) - still latched
        i.tas_state = 11.0;
        assert!(d.update(&i, Millis(5000)), "margin insufficient");

        // both satisfied - clears
        i.tas_state = 12.0;
        assert!(!d.update(&i, Millis(6000)));
    }

    /// The OTHER clear path, which the tests above deliberately suppress.
    ///
    /// Once the aircraft is neither slow-at-full-throttle nor below its
    /// demanded height, upstream's trailing `else` clears the flag with no
    /// regard to the 115% margin or the 3-second timer. Discovering this is
    /// what corrected three test premises in this module.
    #[test]
    fn else_branch_clears_without_waiting_for_the_timer() {
        let mut d = UnderspeedDetector::new();
        let mut i = nominal();

        i.tas_state = 8.0;
        i.throttle_dem = 1.0;
        assert!(d.update(&i, Millis(1000)));

        // at demanded height with throttle backed off: clears immediately,
        // only 1ms later, and well under the 115% margin
        i.tas_state = 10.5; // below 11.5
        i.throttle_dem = 0.5;
        i.height = 100.0;
        i.hgt_dem = 100.0;
        assert!(!d.update(&i, Millis(1001)), "else-branch clears at once");
    }

    /// Staying below the entry threshold restarts the clear timer, so the
    /// 3-second window measures time since it was last genuinely slow.
    #[test]
    fn remaining_slow_restarts_the_clear_timer() {
        let mut d = UnderspeedDetector::new();
        let mut i = nominal();
        i.tas_state = 8.0;
        i.throttle_dem = 1.0;

        // held below demanded height so the set-branch keeps latching and the
        // timer path is the one under test
        i.height = 50.0;
        i.hgt_dem = 100.0;

        d.update(&i, Millis(1000));
        // still slow at 4s - timer restarts here
        d.update(&i, Millis(4000));

        // now fast, 2s after the restart - not yet 3s, still latched
        i.tas_state = 12.0;
        i.throttle_dem = 0.5;
        assert!(d.update(&i, Millis(6000)));

        // 3s after the restart - clears
        assert!(!d.update(&i, Millis(7100)));
    }

    /// Below the demanded height, an existing condition persists even once
    /// throttle backs off - the aircraft has not recovered its energy yet.
    #[test]
    fn persists_below_demanded_height() {
        let mut d = UnderspeedDetector::new();
        let mut i = nominal();
        i.tas_state = 8.0;
        i.throttle_dem = 1.0;
        assert!(d.update(&i, Millis(1000)));

        // throttle backed off, still below demanded height, margin not met
        i.throttle_dem = 0.4;
        i.height = 50.0;
        i.hgt_dem = 100.0;
        assert!(d.update(&i, Millis(2000)), "should persist below hgt_dem");
    }

    /// VTOL clears unconditionally - airspeed is not meaningful under lift.
    #[test]
    fn vtol_clears_unconditionally() {
        let mut d = UnderspeedDetector::new();
        let mut i = nominal();
        i.tas_state = 8.0;
        i.throttle_dem = 1.0;
        assert!(d.update(&i, Millis(1000)));

        i.flight_stage = FlightStage::Vtol;
        assert!(!d.update(&i, Millis(1100)), "VTOL clears regardless");
    }

    /// The elapsed check wraps like upstream's unsigned subtraction, so the
    /// ~49-day rollover does not spuriously clear or latch the flag.
    #[test]
    fn elapsed_check_survives_millis_rollover() {
        let mut d = UnderspeedDetector::new();
        let mut i = nominal();
        i.tas_state = 8.0;
        i.throttle_dem = 1.0;
        // below demanded height, so the timer path is what is exercised
        i.height = 50.0;
        i.hgt_dem = 100.0;

        // latch just before the rollover
        d.update(&i, Millis(u32::MAX - 1000));

        // 2s later, having wrapped: fast enough but under 3s, still latched
        i.tas_state = 12.0;
        i.throttle_dem = 0.5;
        assert!(d.update(&i, Millis(1000)), "2s across rollover");

        // 4s after latching, still wrapped: clears
        assert!(!d.update(&i, Millis(3000)));
    }
}
