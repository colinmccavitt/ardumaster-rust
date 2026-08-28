//! Airspeed options bitmask, upstream `AP_Airspeed::_options` / `ARSPD_OPTIONS`.
//!
//! Vehicle-level (not per-instance). Default enables wind-max disable,
//! recovery re-enable, and EKF consistency — upstream `OPTIONS_DEFAULT`.

/// If set, disable use on airspeed / groundspeed mismatch (`ARSPD_WIND_MAX`).
pub const ARSPD_OPTION_ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE: u32 = 1 << 0;
/// If set, automatically re-enable use when the sensor is healthy again.
pub const ARSPD_OPTION_ON_FAILURE_AHRS_WIND_MAX_RECOVERY_DO_REENABLE: u32 = 1 << 1;
/// If set, skip analog voltage correction.
pub const ARSPD_OPTION_DISABLE_VOLTAGE_CORRECTION: u32 = 1 << 2;
/// If set, require EKF3 statistical consistency before using airspeed.
pub const ARSPD_OPTION_USE_EKF_CONSISTENCY: u32 = 1 << 3;
/// If set, report offset calibration to the GCS.
pub const ARSPD_OPTION_REPORT_OFFSET: u32 = 1 << 4;

/// Upstream `OPTIONS_DEFAULT`: bits 0, 1, and 3.
pub const ARSPD_OPTIONS_DEFAULT: u32 = ARSPD_OPTION_ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE
    | ARSPD_OPTION_ON_FAILURE_AHRS_WIND_MAX_RECOVERY_DO_REENABLE
    | ARSPD_OPTION_USE_EKF_CONSISTENCY;

/// Whether `bit` is set in `ARSPD_OPTIONS`.
#[must_use]
pub const fn option_enabled(options: u32, bit: u32) -> bool {
    options & bit != 0
}

/// Disable TAS use after an `ARSPD_WIND_MAX` mismatch, upstream bit 0.
#[must_use]
pub const fn disable_on_wind_max_failure(options: u32) -> bool {
    option_enabled(options, ARSPD_OPTION_ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE)
}

/// Re-enable TAS use after mismatch recovery, upstream bit 1.
#[must_use]
pub const fn reenable_on_wind_max_recovery(options: u32) -> bool {
    option_enabled(options, ARSPD_OPTION_ON_FAILURE_AHRS_WIND_MAX_RECOVERY_DO_REENABLE)
}

/// Skip analog voltage correction, upstream bit 2.
#[must_use]
pub const fn disable_voltage_correction(options: u32) -> bool {
    option_enabled(options, ARSPD_OPTION_DISABLE_VOLTAGE_CORRECTION)
}

/// Require EKF3 consistency, upstream bit 3.
#[must_use]
pub const fn use_ekf_consistency(options: u32) -> bool {
    option_enabled(options, ARSPD_OPTION_USE_EKF_CONSISTENCY)
}

/// Report offset cal to GCS, upstream bit 4.
#[must_use]
pub const fn report_offset(options: u32) -> bool {
    option_enabled(options, ARSPD_OPTION_REPORT_OFFSET)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_options_match_upstream_bits() {
        assert_eq!(ARSPD_OPTIONS_DEFAULT, (1 << 0) | (1 << 1) | (1 << 3));
        assert_eq!(ARSPD_OPTIONS_DEFAULT, 11);
        assert!(disable_on_wind_max_failure(ARSPD_OPTIONS_DEFAULT));
        assert!(reenable_on_wind_max_recovery(ARSPD_OPTIONS_DEFAULT));
        assert!(!disable_voltage_correction(ARSPD_OPTIONS_DEFAULT));
        assert!(use_ekf_consistency(ARSPD_OPTIONS_DEFAULT));
        assert!(!report_offset(ARSPD_OPTIONS_DEFAULT));
    }

    #[test]
    fn option_enabled_decodes_each_bit() {
        let all = ARSPD_OPTION_ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE
            | ARSPD_OPTION_ON_FAILURE_AHRS_WIND_MAX_RECOVERY_DO_REENABLE
            | ARSPD_OPTION_DISABLE_VOLTAGE_CORRECTION
            | ARSPD_OPTION_USE_EKF_CONSISTENCY
            | ARSPD_OPTION_REPORT_OFFSET;
        assert!(option_enabled(all, ARSPD_OPTION_REPORT_OFFSET));
        assert!(disable_voltage_correction(all));
        assert!(!option_enabled(0, ARSPD_OPTION_USE_EKF_CONSISTENCY));
        assert!(report_offset(ARSPD_OPTION_REPORT_OFFSET));
    }
}
