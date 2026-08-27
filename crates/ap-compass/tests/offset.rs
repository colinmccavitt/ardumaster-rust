//! Mag offset / learn-offsets stub: COMPASS_OFS apply and latch.

use ap_compass::offset::{
    apply_offsets, learn_offsets, learn_offsets_enabled, offsets_within_max, COMPASS_LEARN_DEFAULT,
    COMPASS_LEARN_INFLIGHT, COMPASS_LEARN_NONE, COMPASS_OFFSETS_MAX_DEFAULT,
};
use ap_compass::params::CompassParams;
use ap_compass::sitl::{mag_field_body_ned, SitlCompassBackend, SitlCompassCluster, SitlCompassConfig};
use ap_math::matrix3::Matrix3f;
use ap_math::vector3::Vector3f;

#[test]
fn compass_params_learn_defaults_match_upstream() {
    let params = CompassParams::default();
    assert_eq!(params.learn, COMPASS_LEARN_DEFAULT);
    assert_eq!(params.learn, COMPASS_LEARN_NONE);
    assert!((params.offsets_max - COMPASS_OFFSETS_MAX_DEFAULT).abs() < f32::EPSILON);
    assert_eq!(params.compass1.offset, Vector3f::zero());
}

#[test]
fn apply_compass_ofs_shifts_published_field() {
    let ofs = Vector3f::new(-0.05, 0.02, 0.01);
    let mut compass = SitlCompassBackend::with_config(SitlCompassConfig {
        offset: ofs,
        ..SitlCompassConfig::default()
    });
    assert!(compass.timer_tick(51.875, -0.154, Matrix3f::identity(), 10));
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, Matrix3f::identity());
    let sample = compass.update().expect("pending sample");
    let expected = apply_offsets(wmm, ofs);
    assert!((sample.mag_body.x - expected.x).abs() < 1e-5);
    assert!((sample.mag_body.y - expected.y).abs() < 1e-5);
    assert!((sample.mag_body.z - expected.z).abs() < 1e-5);
}

#[test]
fn learn_offsets_cancels_hardiron_bias() {
    let bias = Vector3f::new(0.05, -0.02, 0.01);
    let mut compass = SitlCompassBackend::with_config(SitlCompassConfig {
        hardiron_bias: bias,
        ..SitlCompassConfig::default()
    });
    assert!(compass.timer_tick(51.875, -0.154, Matrix3f::identity(), 10));
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, Matrix3f::identity());
    let before = compass.state().mag_body;
    assert!((before.x - (wmm.x + bias.x)).abs() < 1e-5);

    assert!(learn_offsets_enabled(COMPASS_LEARN_INFLIGHT));
    assert!(compass.learn_offset(COMPASS_OFFSETS_MAX_DEFAULT));
    let ofs = compass.config().offset;
    let expected_ofs = learn_offsets(wmm + bias, wmm);
    assert!((ofs.x - expected_ofs.x).abs() < 1e-5);
    assert!(offsets_within_max(ofs, COMPASS_OFFSETS_MAX_DEFAULT));

    assert!(compass.timer_tick(51.875, -0.154, Matrix3f::identity(), 20));
    let after = compass.update().expect("learned sample");
    assert!((after.mag_body.x - wmm.x).abs() < 1e-5);
    assert!((after.mag_body.y - wmm.y).abs() < 1e-5);
    assert!((after.mag_body.z - wmm.z).abs() < 1e-5);
}

#[test]
fn cluster_learn_offsets_both_instances() {
    let bias = Vector3f::new(0.04, 0.0, 0.0);
    let mut cluster = SitlCompassCluster::default();
    let _ = cluster.register(SitlCompassBackend::with_config(SitlCompassConfig {
        hardiron_bias: bias,
        ..SitlCompassConfig::default()
    }));
    cluster.backend_mut(0).unwrap().set_config(SitlCompassConfig {
        hardiron_bias: bias,
        ..SitlCompassConfig::default()
    });
    cluster.timer_tick_all(51.875, -0.154, Matrix3f::identity(), 10);
    assert!(cluster.learn_offsets(COMPASS_OFFSETS_MAX_DEFAULT));
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, Matrix3f::identity());
    for i in 0..cluster.instance_count() {
        let ofs = cluster.backend(i).unwrap().config().offset;
        assert!((ofs.x + bias.x).abs() < 1e-5);
        assert!((ofs.y + bias.y).abs() < 1e-5);
        assert!((ofs.z + bias.z).abs() < 1e-5);
    }
    let _ = wmm;
}

#[test]
fn learn_offset_skipped_when_disabled_or_no_sample() {
    let mut compass = SitlCompassBackend::with_config(SitlCompassConfig {
        disabled: true,
        ..SitlCompassConfig::default()
    });
    assert!(!compass.learn_offset(COMPASS_OFFSETS_MAX_DEFAULT));
    let mut fresh = SitlCompassBackend::default();
    assert!(!fresh.learn_offset(COMPASS_OFFSETS_MAX_DEFAULT));
}
