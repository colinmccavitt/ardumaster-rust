//! Compass declination, upstream `Compass::try_set_initial_location`. FW-014.

use ap_declination::get_declination;
use ap_math::scalar::radians;
use crate::params::CompassParams;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct GpsDeclinationFix {
    pub latitude_deg: f32,
    pub longitude_deg: f32,
    pub have_fix: bool,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct CompassDeclinationState {
    pub initial_location_set: bool,
    pub declination_rad: f32,
}

impl CompassDeclinationState {
    pub fn try_set_initial_location(
        &mut self,
        params: &CompassParams,
        gps: Option<GpsDeclinationFix>,
        enabled: bool,
    ) {
        if !params.auto_declination || !enabled || self.initial_location_set {
            return;
        }
        let Some(gps) = gps else { return };
        if !gps.have_fix {
            return;
        }
        self.initial_location_set = true;
        self.declination_rad = radians(get_declination(gps.latitude_deg, gps.longitude_deg));
    }

    #[must_use]
    pub fn effective_declination_rad(&self, params: &CompassParams) -> f32 {
        if params.auto_declination {
            if self.initial_location_set { self.declination_rad } else { 0.0 }
        } else {
            params.declination_rad
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_math::scalar::degrees;

    const LONDON: GpsDeclinationFix = GpsDeclinationFix {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        have_fix: true,
    };

    #[test]
    fn auto_declination_latches_on_first_gps_fix() {
        let mut state = CompassDeclinationState::default();
        let params = CompassParams::default();
        state.try_set_initial_location(&params, Some(LONDON), true);
        assert!(state.initial_location_set);
        assert!(state.declination_rad.abs() > 0.0);
        let before = state.declination_rad;
        state.try_set_initial_location(
            &params,
            Some(GpsDeclinationFix { latitude_deg: -35.3, longitude_deg: 149.1, have_fix: true }),
            true,
        );
        assert!((state.declination_rad - before).abs() < f32::EPSILON);
    }

    #[test]
    fn auto_declination_waits_for_gps_fix() {
        let mut state = CompassDeclinationState::default();
        let params = CompassParams::default();
        state.try_set_initial_location(&params, Some(GpsDeclinationFix { have_fix: false, ..LONDON }), true);
        assert!(!state.initial_location_set);
        assert_eq!(state.effective_declination_rad(&params), 0.0);
    }

    #[test]
    fn manual_declination_uses_compass_dec_param() {
        let state = CompassDeclinationState::default();
        let mut params = CompassParams::default();
        params.auto_declination = false;
        params.declination_rad = degrees(12.5);
        assert!((state.effective_declination_rad(&params) - degrees(12.5)).abs() < 1e-5);
    }

    #[test]
    fn auto_declination_skipped_when_disabled() {
        let mut state = CompassDeclinationState::default();
        let mut params = CompassParams::default();
        params.auto_declination = false;
        state.try_set_initial_location(&params, Some(LONDON), true);
        assert!(!state.initial_location_set);
    }
}
