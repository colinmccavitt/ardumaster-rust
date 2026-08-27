use ap_landing::slope_stage::RangefinderState;
use ap_plane::altitude_tecs_feed_hookup::{relative_target_altitude_cm, AltitudeTecsFeedInputs};
use ap_plane::rangefinder_correction_glue_hookup::{
    rangefinder_correction_glue_inputs, rangefinder_correction_glue_tick,
};
use ap_plane::target_altitude::TargetAltitude;

#[test]
fn correction_feeds_relative_target_altitude_cm() {
    let correction = rangefinder_correction_glue_tick(rangefinder_correction_glue_inputs(
        true,
        RangefinderState {
            in_use: true,
            correction: 2.0,
            last_stable_correction: 0.0,
        },
    ));
    let cm = relative_target_altitude_cm(&AltitudeTecsFeedInputs {
        home_altitude_m: 100.0,
        next_wp_alt_m: 200.0,
        mission_alt_offset_cm: 0,
        rangefinder_correction_m: correction,
        target: TargetAltitude::FromNextWaypoint,
        ..Default::default()
    });
    // (200-100)*100 + 2.0*100 = 10200
    assert!((cm - 10200.0).abs() < 1e-6);
}
