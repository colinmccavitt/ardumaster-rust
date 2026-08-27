use ap_plane::altitude_tecs_feed_hookup::{relative_target_altitude_cm, AltitudeTecsFeedInputs};
use ap_plane::target_altitude::TargetAltitude;

#[test]
fn integration_alt_hold_target_cm() {
    let cm = relative_target_altitude_cm(&AltitudeTecsFeedInputs {
        relative_altitude_m: 25.0,
        target: TargetAltitude::HoldCurrentAndResetOffset,
        ..Default::default()
    });
    assert!((cm - 2500.0).abs() < 1e-6);
}
