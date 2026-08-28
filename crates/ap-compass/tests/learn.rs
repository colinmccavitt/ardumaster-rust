//! COMPASS_LEARN mode enum stub: `Compass::LearnType`.

use ap_compass::learn::LearnType;
use ap_compass::offset::{
    COMPASS_LEARN_DEFAULT, COMPASS_LEARN_EKF, COMPASS_LEARN_INFLIGHT, COMPASS_LEARN_NONE,
};
use ap_compass::params::CompassParams;

#[test]
fn compass_params_learn_default_is_none() {
    let params = CompassParams::default();
    assert_eq!(params.learn, COMPASS_LEARN_DEFAULT);
    assert_eq!(LearnType::from_u8(params.learn), Some(LearnType::None));
}

#[test]
fn learn_type_round_trips_param() {
    let mut params = CompassParams::default();
    params.learn = COMPASS_LEARN_INFLIGHT;
    let mode = LearnType::from_u8(params.learn).expect("inflight");
    assert_eq!(mode, LearnType::Inflight);
    assert_eq!(mode.as_u8(), COMPASS_LEARN_INFLIGHT);
    assert!(mode.inflight_offsets_enabled());

    params.learn = COMPASS_LEARN_EKF;
    let mode = LearnType::from_u8(params.learn).expect("ekf");
    assert_eq!(mode, LearnType::CopyFromEkf);
    assert!(!mode.inflight_offsets_enabled());
    assert!(mode.offsets_learn_enabled());

    params.learn = COMPASS_LEARN_NONE;
    assert_eq!(LearnType::from_u8(params.learn), Some(LearnType::None));
}
