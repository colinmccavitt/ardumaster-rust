use ap_airspeed::params::{
    AirspeedInstanceParams, AirspeedParams, ARSPD_RATIO_PARAM_DEFAULT,
};
use ap_airspeed::sitl::{
    apply_autocal_ratio, apply_pitot_ratio, apply_temp_compensation, sitl_airspeed_temperature_c,
    SitlAirspeedBackend, SitlAirspeedCluster, ARSPD_AUTOCAL_DEFAULT, ARSPD_RATIO_DEFAULT,
    ARSPD_SKIP_CAL_DEFAULT, ARSPD_TEMP_REF_C,
};

#[test]
fn airspeed_params_defaults_match_upstream_ratio() {
    let params = AirspeedParams::default();
    assert!((ARSPD_RATIO_PARAM_DEFAULT - 2.0).abs() < 1e-6);
    assert!((ARSPD_RATIO_DEFAULT - 2.0).abs() < 1e-6);
    assert!(!params.airspeed1.disabled);
    assert!((params.airspeed1.ratio - 2.0).abs() < 1e-6);
    assert!((params.primary_ratio() - 2.0).abs() < 1e-6);
    assert_eq!(params.airspeed1.use_airspeed, 1);
    assert_eq!(params.primary_use_airspeed(), 1);
    assert!((params.airspeed1.temperature_c - ARSPD_TEMP_REF_C).abs() < 1e-6);
    assert!((params.primary_temperature_c() - 15.0).abs() < 1e-6);
    assert_eq!(params.airspeed1.temp_coeff, 0.0);
    assert_eq!(params.airspeed1.autocal, ARSPD_AUTOCAL_DEFAULT);
    assert_eq!(params.primary_autocal(), 0);
    assert_eq!(params.airspeed1.skip_cal, ARSPD_SKIP_CAL_DEFAULT);
    assert!(!params.primary_skip_cal());
    assert_eq!(params.airspeed1.pin, 0);
    assert_eq!(params.primary_pin(), 0);
    assert!((params.airspeed1.psi_range - 1.0).abs() < 1e-6);
    assert!((params.primary_psi_range() - 1.0).abs() < 1e-6);
}

#[test]
fn apply_airspeed_params_sets_ratio_on_both_instances() {
    let mut cluster = SitlAirspeedCluster::default();
    let _ = cluster.register(SitlAirspeedBackend::default());
    let mut params = AirspeedParams::default();
    params.airspeed1.ratio = 1.0;
    params.airspeed2.ratio = 4.0;
    params.primary = 1;
    params.apply_to_cluster(&mut cluster);
    assert_eq!(cluster.primary(), 1);
    assert!((cluster.backend(0).unwrap().config().ratio - 1.0).abs() < 1e-6);
    assert!((cluster.backend(1).unwrap().config().ratio - 4.0).abs() < 1e-6);
}

#[test]
fn instance_params_apply_to_sitl_config() {
    let cfg = AirspeedInstanceParams {
        disabled: true,
        offset_mps: 1.5,
        skip_cal: true,
        ratio: 1.0,
        use_airspeed: 0,
        temperature_c: 25.0,
        temp_coeff: 0.01,
        autocal: 1,
        pin: 13,
        psi_range: 2.0,
    }
    .apply_to_config();
    assert!(cfg.disabled);
    assert!((cfg.offset_mps - 1.5).abs() < 1e-6);
    assert!(cfg.skip_cal);
    assert!((cfg.ratio - 1.0).abs() < 1e-6);
    assert_eq!(cfg.use_airspeed, 0);
    assert!((cfg.temperature_c - 25.0).abs() < 1e-6);
    assert!((cfg.temp_coeff - 0.01).abs() < 1e-6);
    assert_eq!(cfg.autocal, 1);
}

#[test]
fn instance_params_apply_to_analog_config() {
    let analog = AirspeedInstanceParams {
        pin: 13,
        psi_range: 2.0,
        ..AirspeedInstanceParams::default()
    }
    .analog_config();
    assert_eq!(analog.pin, 13);
    assert!((analog.psi_range - 2.0).abs() < 1e-6);
}

#[test]
fn apply_pitot_ratio_scales_against_default() {
    assert!((apply_pitot_ratio(20.0, 2.0) - 20.0).abs() < 1e-6);
    assert!((apply_pitot_ratio(20.0, 1.0) - 10.0).abs() < 1e-6);
    assert!((apply_pitot_ratio(20.0, 4.0) - 40.0).abs() < 1e-6);
    assert_eq!(apply_pitot_ratio(20.0, 0.0), 0.0);
}

#[test]
fn apply_temp_compensation_is_identity_at_isa_or_zero_coeff() {
    assert!((sitl_airspeed_temperature_c(0.0) - 15.0).abs() < 1e-6);
    assert!((apply_temp_compensation(20.0, 15.0, 0.02) - 20.0).abs() < 1e-6);
    assert!((apply_temp_compensation(20.0, 25.0, 0.0) - 20.0).abs() < 1e-6);
    assert!((apply_temp_compensation(20.0, 25.0, 0.01) - 22.0).abs() < 1e-6);
}

#[test]
fn apply_airspeed_params_sets_temp_comp_on_both_instances() {
    let mut cluster = SitlAirspeedCluster::default();
    let _ = cluster.register(SitlAirspeedBackend::default());
    let mut params = AirspeedParams::default();
    params.airspeed1.temperature_c = 5.0;
    params.airspeed1.temp_coeff = 0.02;
    params.airspeed2.temperature_c = 25.0;
    params.airspeed2.temp_coeff = 0.01;
    params.apply_to_cluster(&mut cluster);
    assert!((cluster.backend(0).unwrap().config().temperature_c - 5.0).abs() < 1e-6);
    assert!((cluster.backend(0).unwrap().config().temp_coeff - 0.02).abs() < 1e-6);
    assert!((cluster.backend(1).unwrap().config().temperature_c - 25.0).abs() < 1e-6);
    assert!((cluster.backend(1).unwrap().config().temp_coeff - 0.01).abs() < 1e-6);
}

#[test]
fn apply_autocal_ratio_is_identity_when_disabled() {
    assert_eq!(ARSPD_AUTOCAL_DEFAULT, 0);
    assert!((apply_autocal_ratio(2.0, 25.0, 20.0, 0) - 2.0).abs() < 1e-6);
    assert!((apply_autocal_ratio(2.0, 25.0, 20.0, 1) - 2.5).abs() < 1e-6);
    assert!((apply_autocal_ratio(2.0, 0.0, 20.0, 1) - 2.0).abs() < 1e-6);
}

#[test]
fn apply_airspeed_params_sets_autocal_on_both_instances() {
    let mut cluster = SitlAirspeedCluster::default();
    let _ = cluster.register(SitlAirspeedBackend::default());
    let mut params = AirspeedParams::default();
    params.airspeed1.autocal = 1;
    params.airspeed2.autocal = 1;
    params.apply_to_cluster(&mut cluster);
    assert_eq!(cluster.backend(0).unwrap().config().autocal, 1);
    assert_eq!(cluster.backend(1).unwrap().config().autocal, 1);
}

#[test]
fn apply_airspeed_params_sets_skip_cal_on_both_instances() {
    let mut cluster = SitlAirspeedCluster::default();
    let _ = cluster.register(SitlAirspeedBackend::default());
    let mut params = AirspeedParams::default();
    params.airspeed1.skip_cal = true;
    params.airspeed2.skip_cal = true;
    params.apply_to_cluster(&mut cluster);
    assert!(cluster.backend(0).unwrap().config().skip_cal);
    assert!(cluster.backend(1).unwrap().config().skip_cal);
}
