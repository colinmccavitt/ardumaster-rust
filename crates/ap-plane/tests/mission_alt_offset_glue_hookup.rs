use ap_plane::altitude_tecs_feed_hookup::{relative_target_altitude_cm, AltitudeTecsFeedInputs};
use ap_plane::mission_alt_offset_glue_hookup::{mission_alt_offset_glue_tick, MissionAltOffsetGlueInputs};
use ap_plane::target_altitude::TargetAltitude;

#[test]
fn integration_offset_feeds_tecs_target() {
    let offset = mission_alt_offset_glue_tick(MissionAltOffsetGlueInputs {
        offset_cm: 500,
        target: TargetAltitude::ProportionalToNextWaypoint,
    });
    let cm = relative_target_altitude_cm(&AltitudeTecsFeedInputs {
        home_altitude_m: 100.0,
        next_wp_alt_m: 150.0,
        mission_alt_offset_cm: offset,
        target: TargetAltitude::FromNextWaypoint,
        ..Default::default()
    });
    assert!((cm - 5500.0).abs() < 1e-6);
}

#[test]
fn integration_soaring_resets_offset_for_tecs() {
    let offset = mission_alt_offset_glue_tick(MissionAltOffsetGlueInputs {
        offset_cm: 2500,
        target: TargetAltitude::HoldCurrentAndResetOffset,
    });
    assert_eq!(offset, 0);
}
