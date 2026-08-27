use ap_plane::altitude_tecs_feed_hookup::{
    apply_baro_terrain_offset, relative_target_altitude_cm, altitude_tecs_feed_tick,
    AltitudeTecsFeedInputs,
};
use ap_plane::target_altitude::TargetAltitude;
use ap_plane::tecs_baro_hookup::TecsBaroFeed;
use ap_tecs::tecs::Tecs;

#[test]
fn integration_alt_hold_target_cm() {
    let cm = relative_target_altitude_cm(&AltitudeTecsFeedInputs {
        relative_altitude_m: 25.0,
        target: TargetAltitude::HoldCurrentAndResetOffset,
        ..Default::default()
    });
    assert!((cm - 2500.0).abs() < 1e-6);
}

#[test]
fn integration_baro_feed_wired_with_terrain_offset() {
    let feed = TecsBaroFeed {
        height_m: 60.0,
        climb_rate_mps: 0.0,
        hgt_afe_m: 60.0,
    };
    let adjusted = apply_baro_terrain_offset(feed, 3.0);
    assert!((adjusted.hgt_afe_m - 63.0).abs() < 1e-6);

    let mut tecs = Tecs::default();
    let out = altitude_tecs_feed_tick(
        &mut tecs,
        &AltitudeTecsFeedInputs {
            baro_feed: feed,
            have_baro_sample: true,
            relative_altitude_m: 60.0,
            terrain_offset_m: 3.0,
            home_altitude_m: 0.0,
            next_wp_alt_m: 100.0,
            target_airspeed_cm: 1500.0,
            ..Default::default()
        },
    );
    assert!(out.ran);
}
