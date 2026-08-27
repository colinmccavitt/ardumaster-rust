use ap_plane::altitude_glue_hookup::{altitude_glue_tick, AltitudeGlueInputs};

#[test]
fn integration_relative_altitude_tick() {
    let alt = altitude_glue_tick(AltitudeGlueInputs {
        baro_altitude_m: 120.0,
        baro_relative_m: Some(20.0),
        home_altitude_m: 0.0,
        have_baro_sample: true,
    });
    assert!((alt - 20.0).abs() < 1e-6);
}
