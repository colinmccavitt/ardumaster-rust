//! ARSPD_OPTIONS bitfield stub, upstream `AP_Airspeed::_options`.
//!
//! Vehicle-level bitmask. Default matches upstream `OPTIONS_DEFAULT`
//! (wind-max disable, recovery re-enable, EKF consistency).

use ap_airspeed::options::{
    disable_on_wind_max_failure, disable_voltage_correction, reenable_on_wind_max_recovery,
    report_offset, use_ekf_consistency,
};
use ap_airspeed::params::AirspeedParams;

/// Frontend options hookup for the vehicle loop.
#[derive(Debug, Clone)]
pub struct AirspeedOptionsHookup {
    params: AirspeedParams,
}

impl Default for AirspeedOptionsHookup {
    fn default() -> Self {
        Self::from_params(AirspeedParams::default())
    }
}

/// Decoded `ARSPD_OPTIONS` published from the vehicle-level bitmask.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AirspeedOptionsPublish {
    /// Bound `ARSPD_OPTIONS`.
    pub options: u32,
    /// Bit 0: disable use on `ARSPD_WIND_MAX` mismatch.
    pub disable_on_wind_max_failure: bool,
    /// Bit 1: re-enable use after mismatch recovery.
    pub reenable_on_wind_max_recovery: bool,
    /// Bit 2: skip analog voltage correction.
    pub disable_voltage_correction: bool,
    /// Bit 3: require EKF3 consistency.
    pub use_ekf_consistency: bool,
    /// Bit 4: report offset cal to GCS.
    pub report_offset: bool,
}

impl AirspeedOptionsHookup {
    /// Build an options hookup from vehicle params.
    #[must_use]
    pub fn from_params(params: AirspeedParams) -> Self {
        Self { params }
    }

    /// Current `ARSPD_*` params.
    #[must_use]
    pub const fn airspeed_params(&self) -> &AirspeedParams {
        &self.params
    }

    /// Apply vehicle-level `ARSPD_OPTIONS`.
    pub fn apply_airspeed_params(&mut self, params: AirspeedParams) {
        self.params = params;
    }

    /// Set `ARSPD_OPTIONS`.
    pub fn set_options(&mut self, options: u32) {
        let mut params = self.params;
        params.options = options;
        self.params = params;
    }

    /// Publish decoded `ARSPD_OPTIONS`.
    #[must_use]
    pub fn publish(&self) -> AirspeedOptionsPublish {
        select_airspeed_options(self.params.options)
    }
}

/// Map stored `ARSPD_OPTIONS` to decoded flags.
#[must_use]
pub fn select_airspeed_options(options: u32) -> AirspeedOptionsPublish {
    AirspeedOptionsPublish {
        options,
        disable_on_wind_max_failure: disable_on_wind_max_failure(options),
        reenable_on_wind_max_recovery: reenable_on_wind_max_recovery(options),
        disable_voltage_correction: disable_voltage_correction(options),
        use_ekf_consistency: use_ekf_consistency(options),
        report_offset: report_offset(options),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_airspeed::options::{
        option_enabled, ARSPD_OPTIONS_DEFAULT, ARSPD_OPTION_REPORT_OFFSET,
    };

    #[test]
    fn default_options_match_upstream() {
        let hookup = AirspeedOptionsHookup::default();
        assert_eq!(hookup.airspeed_params().options, ARSPD_OPTIONS_DEFAULT);
        let out = hookup.publish();
        assert_eq!(out.options, ARSPD_OPTIONS_DEFAULT);
        assert!(out.disable_on_wind_max_failure);
        assert!(out.reenable_on_wind_max_recovery);
        assert!(!out.disable_voltage_correction);
        assert!(out.use_ekf_consistency);
        assert!(!out.report_offset);
    }

    #[test]
    fn report_offset_bit_is_published() {
        let mut hookup = AirspeedOptionsHookup::default();
        hookup.set_options(ARSPD_OPTIONS_DEFAULT | ARSPD_OPTION_REPORT_OFFSET);
        let out = hookup.publish();
        assert!(out.report_offset);
        assert!(option_enabled(out.options, ARSPD_OPTION_REPORT_OFFSET));
        hookup.set_options(0);
        assert_eq!(hookup.publish().options, 0);
        assert!(!hookup.publish().use_ekf_consistency);
    }
}
