//! ARSPD_OFF_PCNT Plane-only offset-cal speed-error warning stub.
//!
//! Vehicle-level (`AP_Airspeed::max_speed_pcnt`). Default 0 disables. When
//! set, a calibration offset jump larger than `OFF_PCNT` percent of
//! `ARSPD_FBW_MIN` flags an uncovered-pitot warning.

use ap_airspeed::fbw::ARSPD_FBW_MIN_DEFAULT;
use ap_airspeed::off_pcnt::{
    off_pcnt_enabled, offset_change_warns, offset_max_change, ARSPD_OFF_PCNT_DEFAULT,
};
use ap_airspeed::params::AirspeedParams;

/// Frontend OFF_PCNT hookup for the vehicle loop.
#[derive(Debug, Clone)]
pub struct AirspeedOffPcntHookup {
    params: AirspeedParams,
}

impl Default for AirspeedOffPcntHookup {
    fn default() -> Self {
        Self::from_params(AirspeedParams::default())
    }
}

/// `ARSPD_OFF_PCNT` check published from a stored vs new calibration offset.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirspeedOffPcntPublish {
    /// Bound `ARSPD_OFF_PCNT` (percent of `AIRSPEED_MIN`).
    pub off_pcnt: i8,
    /// `ARSPD_FBW_MIN` / `AIRSPEED_MIN` used as the speed base (m/s).
    pub airspeed_min: f32,
    /// Allowed |offset| change (pressure-like, upstream 1/2 v^2).
    pub max_change: f32,
    /// True when `ARSPD_OFF_PCNT` is greater than zero.
    pub enabled: bool,
    /// True when the enabled threshold is exceeded.
    pub exceeded: bool,
}

impl AirspeedOffPcntHookup {
    /// Build an OFF_PCNT hookup from vehicle params.
    #[must_use]
    pub fn from_params(params: AirspeedParams) -> Self {
        Self { params }
    }

    /// Current `ARSPD_*` params.
    #[must_use]
    pub const fn airspeed_params(&self) -> &AirspeedParams {
        &self.params
    }

    /// Apply vehicle-level `ARSPD_OFF_PCNT` / `ARSPD_FBW_MIN`.
    pub fn apply_airspeed_params(&mut self, params: AirspeedParams) {
        self.params = params;
    }

    /// Set `ARSPD_OFF_PCNT` (percent).
    pub fn set_off_pcnt(&mut self, off_pcnt: i8) {
        let mut params = self.params;
        params.off_pcnt = off_pcnt;
        self.params = params;
    }

    /// Set `ARSPD_FBW_MIN` used as `AIRSPEED_MIN`.
    pub fn set_fbw_min(&mut self, fbw_min: f32) {
        let mut params = self.params;
        params.fbw_min = fbw_min;
        self.params = params;
    }

    /// Publish the OFF_PCNT check for `stored_offset` vs `calibrated_offset`.
    #[must_use]
    pub fn publish(&self, stored_offset: f32, calibrated_offset: f32) -> AirspeedOffPcntPublish {
        check_airspeed_off_pcnt(
            stored_offset,
            calibrated_offset,
            self.params.off_pcnt,
            self.params.fbw_min,
        )
    }
}

/// Map stored `ARSPD_OFF_PCNT` plus offsets to the published warning.
#[must_use]
pub fn check_airspeed_off_pcnt(
    stored_offset: f32,
    calibrated_offset: f32,
    off_pcnt: i8,
    airspeed_min: f32,
) -> AirspeedOffPcntPublish {
    AirspeedOffPcntPublish {
        off_pcnt,
        airspeed_min,
        max_change: offset_max_change(off_pcnt, airspeed_min),
        enabled: off_pcnt_enabled(off_pcnt),
        exceeded: offset_change_warns(stored_offset, calibrated_offset, off_pcnt, airspeed_min),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_off_pcnt_disables_check() {
        let hookup = AirspeedOffPcntHookup::default();
        assert_eq!(hookup.airspeed_params().off_pcnt, ARSPD_OFF_PCNT_DEFAULT);
        assert!((hookup.airspeed_params().fbw_min - ARSPD_FBW_MIN_DEFAULT).abs() < 1e-6);
        let out = hookup.publish(100.0, 0.0);
        assert_eq!(out.off_pcnt, 0);
        assert!(!out.enabled);
        assert!(!out.exceeded);
    }

    #[test]
    fn explicit_percent_flags_large_offset_jump() {
        let mut hookup = AirspeedOffPcntHookup::default();
        hookup.set_off_pcnt(10);
        let fail = hookup.publish(10.0, 30.0);
        assert!(fail.enabled);
        assert!(fail.exceeded);
        let ok = hookup.publish(10.0, 12.0);
        assert!(!ok.exceeded);
    }
}
