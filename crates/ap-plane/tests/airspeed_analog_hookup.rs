//! ARSPD_PIN analog backend stub: ratiometric volts become differential pressure.

use ap_airspeed::analog::{ARSPD_PIN_DISABLED, VOLTS_TO_PASCAL};
use ap_airspeed::params::AirspeedParams;
use ap_ins::LoopTiming;
use ap_plane::airspeed_analog_hookup::AirspeedAnalogHookup;
use ap_plane::main_loop::PlaneMainLoop;

#[test]
fn hookup_default_pin_converts_one_volt_to_pascal() {
    let mut hookup = AirspeedAnalogHookup::default();
    assert_eq!(hookup.airspeed_params().primary_pin(), 0);
    hookup.set_voltage(1.0);
    let published = hookup.publish();
    assert!(published.have_pressure);
    assert_eq!(published.pin, 0);
    assert!((published.pressure_pa - VOLTS_TO_PASCAL).abs() < 1e-2);
    assert!((published.psi_range - 1.0).abs() < 1e-6);
}

#[test]
fn hookup_pin_and_psi_range_scale_pressure() {
    let mut hookup = AirspeedAnalogHookup::default();
    let mut params = AirspeedParams::default();
    params.airspeed1.pin = 13;
    params.airspeed1.psi_range = 2.0;
    hookup.apply_airspeed_params(params);
    hookup.set_voltage(1.0);
    let published = hookup.publish();
    assert_eq!(published.pin, 13);
    assert!(published.have_pressure);
    assert!((published.pressure_pa - (VOLTS_TO_PASCAL / 2.0)).abs() < 1e-2);
}

#[test]
fn hookup_disabled_pin_has_no_pressure() {
    let mut hookup = AirspeedAnalogHookup::default();
    hookup.set_voltage(1.0);
    hookup.set_pin(ARSPD_PIN_DISABLED);
    let published = hookup.publish();
    assert!(!published.have_pressure);
    assert_eq!(published.pin, ARSPD_PIN_DISABLED);
}

#[test]
fn main_loop_ahrs_update_reads_analog_pin() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    let mut hookup = AirspeedAnalogHookup::default();
    hookup.set_voltage(1.0);
    vehicle.analog_airspeed = Some(hookup);

    vehicle.ahrs_update();

    assert_eq!(vehicle.airspeed_pin, 0);
    assert!(vehicle.airspeed_analog_have_pressure);
    assert!((vehicle.airspeed_diff_pressure_pa - VOLTS_TO_PASCAL).abs() < 1e-2);
}
