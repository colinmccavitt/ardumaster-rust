//! Airspeed drivers, upstream `libraries/AP_Airspeed`. FW-010.
//!
//! The SITL backend produces pitot true/equivalent airspeed from body-frame
//! air-relative velocity, matching `SIM_Aircraft` before `AP_Airspeed_SITL`
//! reads the pitot tube.

#![no_std]

pub mod analog;
pub mod backend;
pub mod bus;
pub mod params;
pub mod sitl;
pub mod tube_order;

pub use sitl::{
    apply_autocal_ratio, apply_pitot_ratio, apply_temp_compensation, eas_from_tas,
    pitot_tas_from_body, sitl_airspeed_temperature_c, tas_for_nav, use_airspeed_for_control,
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
