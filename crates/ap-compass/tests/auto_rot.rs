//! Compass auto-orientation stub: `COMPASS_AUTO_ROT`.

use ap_compass::auto_rot::{
    accept_detected_orientation, settings_for_start, AutoRot, COMPASS_AUTO_ROT_CHECK_AND_FIX,
    COMPASS_AUTO_ROT_CHECK_ONLY, COMPASS_AUTO_ROT_DEFAULT, COMPASS_AUTO_ROT_DISABLED,
};
use ap_compass::orientation::COMPASS_ORIENT_YAW_90;
use ap_compass::params::CompassParams;

#[test]
fn compass_params_auto_rot_default_is_check_and_fix() {
    let params = CompassParams::default();
    assert_eq!(params.rotate_auto, COMPASS_AUTO_ROT_DEFAULT);
    assert_eq!(
        AutoRot::from_u8(params.rotate_auto),
        Some(AutoRot::CheckAndFix)
    );
}

#[test]
fn accept_saves_external_orient_when_check_and_fix() {
    let mut params = CompassParams::default();
    params.rotate_auto = COMPASS_AUTO_ROT_CHECK_AND_FIX;
    params.compass1.external = true;
    params.compass1.orientation = 0;
    let detected = COMPASS_ORIENT_YAW_90;
    let saved = accept_detected_orientation(
        params.rotate_auto,
        params.compass1.external,
        detected,
    );
    assert_eq!(saved, Some(detected));
    params.compass1.orientation = saved.expect("saved");
    assert_eq!(params.compass1.orientation, COMPASS_ORIENT_YAW_90);
}

#[test]
fn check_only_does_not_rewrite_orient() {
    let mut params = CompassParams::default();
    params.rotate_auto = COMPASS_AUTO_ROT_CHECK_ONLY;
    params.compass1.external = true;
    assert!(settings_for_start(params.rotate_auto, 0, true).is_some());
    assert_eq!(
        accept_detected_orientation(params.rotate_auto, true, COMPASS_ORIENT_YAW_90),
        None
    );
    assert!(settings_for_start(COMPASS_AUTO_ROT_DISABLED, 0, true).is_none());
}
