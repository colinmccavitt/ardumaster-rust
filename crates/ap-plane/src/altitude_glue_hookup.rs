//! Relative altitude glue from baro publish, upstream `Plane::relative_altitude`.
//!
//! Uses baro-calibrated relative altitude when available, otherwise falls back
//! to baro AMSL minus home reference.

/// Inputs for one relative-altitude update tick.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct AltitudeGlueInputs {
    pub baro_altitude_m: f32,
    pub baro_relative_m: Option<f32>,
    pub home_altitude_m: f32,
    pub have_baro_sample: bool,
}

/// Update vehicle relative altitude above home/reference.
#[must_use]
pub fn altitude_glue_tick(inp: AltitudeGlueInputs) -> f32 {
    if !inp.have_baro_sample {
        return 0.0;
    }
    if let Some(rel) = inp.baro_relative_m {
        return rel;
    }
    inp.baro_altitude_m - inp.home_altitude_m
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prefers_baro_calibrated_relative() {
        let alt = altitude_glue_tick(AltitudeGlueInputs {
            baro_altitude_m: 500.0,
            baro_relative_m: Some(42.0),
            home_altitude_m: 100.0,
            have_baro_sample: true,
        });
        assert!((alt - 42.0).abs() < 1e-6);
    }

    #[test]
    fn falls_back_to_home_reference() {
        let alt = altitude_glue_tick(AltitudeGlueInputs {
            baro_altitude_m: 250.0,
            baro_relative_m: None,
            home_altitude_m: 200.0,
            have_baro_sample: true,
        });
        assert!((alt - 50.0).abs() < 1e-6);
    }

    #[test]
    fn zero_without_baro_sample() {
        assert_eq!(altitude_glue_tick(AltitudeGlueInputs::default()), 0.0);
    }
}
