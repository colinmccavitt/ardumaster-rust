//! Persist learned COMPASS_OFS into the param table so offsets survive reboot.

use ap_compass::offset::{learn_offsets_enabled, COMPASS_LEARN_INFLIGHT, COMPASS_OFFSETS_MAX_DEFAULT};
use ap_compass::params::CompassParams;
use ap_compass::persist::{offsets_already_saved, save_offsets};
use ap_compass::sitl::{
    mag_field_body_ned, SitlCompassBackend, SitlCompassCluster, SitlCompassConfig,
};
use ap_math::matrix3::Matrix3f;
use ap_math::vector3::Vector3f;

#[test]
fn persist_after_learn_restores_on_fresh_cluster() {
    let bias = Vector3f::new(0.05, -0.02, 0.01);
    let mut cluster = SitlCompassCluster::default();
    cluster.backend_mut(0).unwrap().set_config(SitlCompassConfig {
        hardiron_bias: bias,
        ..SitlCompassConfig::default()
    });
    assert!(cluster
        .backend_mut(0)
        .unwrap()
        .timer_tick(51.875, -0.154, Matrix3f::identity(), 10));
    assert!(learn_offsets_enabled(COMPASS_LEARN_INFLIGHT));
    assert!(cluster.learn_offsets(COMPASS_OFFSETS_MAX_DEFAULT));

    let mut params = CompassParams::default();
    assert!(save_offsets(&mut params, &cluster));
    assert!(offsets_already_saved(&params, &cluster));
    assert!((params.compass1.offset.x + bias.x).abs() < 1e-5);

    let mut restored = SitlCompassCluster::default();
    restored.backend_mut(0).unwrap().set_config(SitlCompassConfig {
        hardiron_bias: bias,
        ..SitlCompassConfig::default()
    });
    params.apply_to_cluster(&mut restored);
    assert_eq!(restored.backend(0).unwrap().config().offset, params.compass1.offset);

    assert!(restored
        .backend_mut(0)
        .unwrap()
        .timer_tick(51.875, -0.154, Matrix3f::identity(), 10));
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, Matrix3f::identity());
    let sample = restored.backend_mut(0).unwrap().update().expect("restored sample");
    assert!((sample.mag_body.x - wmm.x).abs() < 1e-5);
    assert!((sample.mag_body.y - wmm.y).abs() < 1e-5);
    assert!((sample.mag_body.z - wmm.z).abs() < 1e-5);
}

#[test]
fn persist_both_instances() {
    let ofs0 = Vector3f::new(-0.03, 0.0, 0.0);
    let ofs1 = Vector3f::new(0.0, 0.04, 0.0);
    let mut cluster = SitlCompassCluster::default();
    let _ = cluster.register(SitlCompassBackend::default());
    cluster.backend_mut(0).unwrap().set_config(SitlCompassConfig {
        offset: ofs0,
        ..SitlCompassConfig::default()
    });
    cluster.backend_mut(1).unwrap().set_config(SitlCompassConfig {
        offset: ofs1,
        ..SitlCompassConfig::default()
    });
    let mut params = CompassParams::default();
    assert!(save_offsets(&mut params, &cluster));
    assert_eq!(params.compass1.offset, ofs0);
    assert_eq!(params.compass2.offset, ofs1);
}
