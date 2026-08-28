use ap_compass::params::{CompassInstanceParams, CompassParams, COMPASS_AUTODEC_DEFAULT, COMPASS_USE_DEFAULT};
use ap_compass::sitl::{SitlCompassBackend, SitlCompassCluster};

#[test]
fn compass_params_defaults_match_upstream() {
    let params = CompassParams::default();
    assert!(COMPASS_AUTODEC_DEFAULT);
    assert!(COMPASS_USE_DEFAULT);
    assert!(!params.compass1.disabled);
    assert!(params.compass1.use_for_yaw);
    assert!(params.auto_declination);
}

#[test]
fn apply_compass_params_disables_backend_and_sets_primary() {
    let mut cluster = SitlCompassCluster::default();
    let _ = cluster.register(SitlCompassBackend::default());
    let mut params = CompassParams::default();
    params.compass1.disabled = true;
    params.primary = 1;
    params.apply_to_cluster(&mut cluster);
    assert_eq!(cluster.primary(), 1);
    assert!(cluster.backend(0).unwrap().config().disabled);
    assert!(!cluster.backend(1).unwrap().config().disabled);
}

#[test]
fn primary_use_for_yaw_follows_configured_instance() {
    let mut params = CompassParams::default();
    params.compass2.use_for_yaw = false;
    params.primary = 1;
    assert!(!params.primary_use_for_yaw());
    params.compass2.use_for_yaw = true;
    assert!(params.primary_use_for_yaw());
}

#[test]
fn instance_params_apply_to_sitl_config() {
    let cfg = CompassInstanceParams {
        disabled: true,
        use_for_yaw: false,
        ..Default::default()
    }
    .apply_to_config();
    assert!(cfg.disabled);
}
