//! Rangefinder correction glue from slope-landing rangefinder state.
//!
//! Upstream `Plane::rangefinder_correction_m` feeds mission altitude offset
//! and TECS terrain correction during LAND when the rangefinder is active.

use ap_landing::slope_stage::RangefinderState;

/// Inputs for one rangefinder-correction glue tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RangefinderCorrectionGlueInputs {
    pub flight_stage_is_land: bool,
    pub rf_in_use: bool,
    pub correction_m: f32,
}

/// Current rangefinder baro correction in metres for TECS height demand.
#[must_use]
pub fn rangefinder_correction_glue_tick(inp: RangefinderCorrectionGlueInputs) -> f32 {
    if !inp.flight_stage_is_land || !inp.rf_in_use {
        return 0.0;
    }
    inp.correction_m
}

/// Build glue inputs from slope-landing rangefinder state.
#[must_use]
pub fn rangefinder_correction_glue_inputs(
    flight_stage_is_land: bool,
    rf: RangefinderState,
) -> RangefinderCorrectionGlueInputs {
    RangefinderCorrectionGlueInputs {
        flight_stage_is_land,
        rf_in_use: rf.in_use,
        correction_m: rf.correction,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_when_not_in_land() {
        assert_eq!(
            rangefinder_correction_glue_tick(RangefinderCorrectionGlueInputs {
                flight_stage_is_land: false,
                rf_in_use: true,
                correction_m: 4.0,
            }),
            0.0
        );
    }

    #[test]
    fn zero_when_rangefinder_not_in_use() {
        assert_eq!(
            rangefinder_correction_glue_tick(RangefinderCorrectionGlueInputs {
                flight_stage_is_land: true,
                rf_in_use: false,
                correction_m: 4.0,
            }),
            0.0
        );
    }

    #[test]
    fn passes_correction_during_land() {
        assert!(
            (rangefinder_correction_glue_tick(RangefinderCorrectionGlueInputs {
                flight_stage_is_land: true,
                rf_in_use: true,
                correction_m: 3.5,
            }) - 3.5)
                .abs()
                < 1e-6
        );
    }

    #[test]
    fn builder_copies_rf_fields() {
        let inp = rangefinder_correction_glue_inputs(
            true,
            RangefinderState {
                in_use: true,
                correction: 2.5,
                last_stable_correction: 0.0,
            },
        );
        assert_eq!(rangefinder_correction_glue_tick(inp), 2.5);
    }
}
