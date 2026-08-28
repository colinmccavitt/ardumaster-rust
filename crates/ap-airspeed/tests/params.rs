use ap_airspeed::params::{
    AirspeedInstanceParams, AirspeedParams, ARSPD_RATIO_PARAM_DEFAULT,
};
use ap_airspeed::sitl::{
    apply_pitot_ratio, SitlAirspeedBackend, SitlAirspeedCluster, ARSPD_RATIO_DEFAULT,
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
    }
    .apply_to_config();
    assert!(cfg.disabled);
    assert!((cfg.offset_mps - 1.5).abs() < 1e-6);
    assert!(cfg.skip_cal);
    assert!((cfg.ratio - 1.0).abs() < 1e-6);
    assert_eq!(cfg.use_airspeed, 0);
}

#[test]
fn apply_pitot_ratio_scales_against_default() {
    assert!((apply_pitot_ratio(20.0, 2.0) - 20.0).abs() < 1e-6);
    assert!((apply_pitot_ratio(20.0, 1.0) - 10.0).abs() < 1e-6);
    assert!((apply_pitot_ratio(20.0, 4.0) - 40.0).abs() < 1e-6);
    assert_eq!(apply_pitot_ratio(20.0, 0.0), 0.0);
}
