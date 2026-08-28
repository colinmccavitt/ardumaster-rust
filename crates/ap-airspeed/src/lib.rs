//! Airspeed drivers, upstream `libraries/AP_Airspeed`. FW-010.
//!
//! The SITL backend produces pitot true/equivalent airspeed from body-frame
//! air-relative velocity, matching `SIM_Aircraft` before `AP_Airspeed_SITL`
//! reads the pitot tube.

#![no_std]

pub mod analog;
pub mod backend;
pub mod bus;
pub mod devid;
pub mod options;
pub mod params;
pub mod sitl;
pub mod tube_order;
pub mod wind_max;

pub use sitl::{
    apply_autocal_ratio, apply_pitot_ratio, apply_temp_compensation, eas_from_tas,
    pitot_tas_from_body, sitl_airspeed_temperature_c, tas_for_nav, tas_for_tecs,
    use_airspeed_for_control, use_airspeed_for_tecs,
    AirspeedHealthFlags, AirspeedSampleState, SitlAirspeedBackend, SitlAirspeedCluster,
    SitlAirspeedConfig, ARSPD_AUTOCAL_DEFAULT, ARSPD_RATIO_DEFAULT, ARSPD_SKIP_CAL_DEFAULT, ARSPD_TEMP_COEFF_DEFAULT,
    ARSPD_TEMP_REF_C, ARSPD_USE_DEFAULT, ISA_LAPSE_K_PER_M, SITL_AIRSPEED_MAX_INSTANCES,
    SITL_AIRSPEED_UPDATE_MS,
};

pub use analog::{
    differential_pressure_pa, AnalogAirspeedBackend, AnalogAirspeedConfig,
    ARSPD_PIN_DEFAULT, ARSPD_PIN_DISABLED, ARSPD_PSI_RANGE_DEFAULT, ARSPD_TYPE_ANALOG,
    VOLTS_TO_PASCAL,
};
pub use backend::{
    active_backend_kind, airspeed_type_enabled, backend_for_kind, backend_kind_from_type,
    AirspeedBackendKind, ARSPD_TYPE_DEFAULT, ARSPD_TYPE_MS4525, ARSPD_TYPE_NONE,
    ARSPD_TYPE_SITL,
};
pub use params::{
    AirspeedInstanceParams, AirspeedParams, ARSPD_RATIO_PARAM_DEFAULT,
};
pub use tube_order::{
    airspeed_from_pressure, last_pressure_pa, PitotTubeOrder, ARSPD_TUBE_ORDER_AUTO,
    ARSPD_TUBE_ORDER_DEFAULT, ARSPD_TUBE_ORDER_NEGATIVE, ARSPD_TUBE_ORDER_POSITIVE,
};
pub use bus::{
    i2c_probe_bus, uses_i2c_bus, ARSPD_BUS_DEFAULT, ARSPD_BUS_EXTERNAL, ARSPD_BUS_EXTERNAL2,
    ARSPD_BUS_INTERNAL,
};
pub use devid::{
    clear_devid_if_not_found, devid_address, devid_after_probe, devid_bus, devid_bus_type,
    devid_devtype, devid_for_configured, devid_is_set, make_bus_id, ARSPD_DEVID_DEFAULT,
    BUS_TYPE_I2C, BUS_TYPE_SITL, MS4525_I2C_ADDR,
};
pub use options::{
    disable_on_wind_max_failure, disable_voltage_correction, option_enabled,
    reenable_on_wind_max_recovery, report_offset, use_ekf_consistency, ARSPD_OPTIONS_DEFAULT,
    ARSPD_OPTION_DISABLE_VOLTAGE_CORRECTION, ARSPD_OPTION_ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE,
    ARSPD_OPTION_ON_FAILURE_AHRS_WIND_MAX_RECOVERY_DO_REENABLE, ARSPD_OPTION_REPORT_OFFSET,
    ARSPD_OPTION_USE_EKF_CONSISTENCY,
};
pub use wind_max::{
    airspeed_groundspeed_delta, wind_max_enabled, wind_max_exceeded, ARSPD_WIND_MAX_DEFAULT,
};
