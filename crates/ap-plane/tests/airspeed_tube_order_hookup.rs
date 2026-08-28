//! ARSPD_TUBE_ORDER stub: pitot connector order remaps differential pressure.

use ap_airspeed::analog::VOLTS_TO_PASCAL;
use ap_airspeed::params::AirspeedParams;
use ap_airspeed::tube_order::{
    ARSPD_TUBE_ORDER_AUTO, ARSPD_TUBE_ORDER_DEFAULT, ARSPD_TUBE_ORDER_NEGATIVE,
    ARSPD_TUBE_ORDER_POSITIVE,
};
use ap_ins::LoopTiming;
use ap_plane::airspeed_analog_hookup::AirspeedAnalogHookup;
use ap_plane::airspeed_tube_order_hookup::AirspeedTubeOrderHookup;
use ap_plane::main_loop::PlaneMainLoop;

#[test]
fn hookup_default_tube_order_is_auto() {
    let hookup = AirspeedTubeOrderHookup::default();
    assert_eq!(
        hookup.airspeed_params().primary_tube_order(),
        ARSPD_TUBE_ORDER_DEFAULT
    );
    let published = hookup.publish(-16.0);
    assert_eq!(published.tube_order, ARSPD_TUBE_ORDER_AUTO);
    assert!((published.last_pressure_pa - 16.0).abs() < 1e-5);
}

#[test]
fn hookup_positive_and_negative_orders() {
    let mut hookup = AirspeedTubeOrderHookup::default();
    hookup.set_tube_order(ARSPD_TUBE_ORDER_POSITIVE);
    let positive = hookup.publish(-16.0);
    assert_eq!(positive.tube_order, ARSPD_TUBE_ORDER_POSITIVE);
    assert!((positive.last_pressure_pa + 16.0).abs() < 1e-5);
    assert_eq!(positive.airspeed_mps, 0.0);

    hookup.set_tube_order(ARSPD_TUBE_ORDER_NEGATIVE);
    let negative = hookup.publish(16.0);
    assert_eq!(negative.tube_order, ARSPD_TUBE_ORDER_NEGATIVE);
    assert!((negative.last_pressure_pa + 16.0).abs() < 1e-5);
    assert_eq!(negative.airspeed_mps, 0.0);

    let swapped = hookup.publish(-16.0);
    assert!((swapped.last_pressure_pa - 16.0).abs() < 1e-5);
    assert!(swapped.airspeed_mps > 0.0);
}

#[test]
fn analog_hookup_negative_order_flips_pressure() {
    let mut hookup = AirspeedAnalogHookup::default();
    let mut params = AirspeedParams::default();
    params.airspeed1.tube_order = ARSPD_TUBE_ORDER_NEGATIVE;
    hookup.apply_airspeed_params(params);
    hookup.set_voltage(1.0);
    let published = hookup.publish();
    assert!(published.have_pressure);
    assert_eq!(published.tube_order, ARSPD_TUBE_ORDER_NEGATIVE);
    assert!((published.pressure_pa + VOLTS_TO_PASCAL).abs() < 1e-2);
}

#[test]
fn main_loop_ahrs_update_honors_tube_order() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    let mut hookup = AirspeedAnalogHookup::default();
    hookup.set_tube_order(ARSPD_TUBE_ORDER_NEGATIVE);
    hookup.set_voltage(1.0);
    vehicle.analog_airspeed = Some(hookup);

    vehicle.ahrs_update();

    assert_eq!(vehicle.airspeed_tube_order, ARSPD_TUBE_ORDER_NEGATIVE);
    assert!(vehicle.airspeed_analog_have_pressure);
    assert!((vehicle.airspeed_diff_pressure_pa + VOLTS_TO_PASCAL).abs() < 1e-2);
}
