//! QuadPlane weathervane / assist-handoff — upstream
//! `get_weathervane_yaw_rate_cds`, `get_desired_yaw_rate_cds`, and
//! the `should_assist` → `assisted_flight` latch.

use ap_quadplane::weathervane::{
    WeatherVane, WeatherVaneDirection, WeathervaneSample, PILOT_YAW_RATE_DPS_DEFAULT,
    WVANE_ACTIVATE_MS, WVANE_ANG_MIN_DEFAULT, WVANE_ENABLE_DEFAULT, WVANE_GAIN_DEFAULT,
};
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

/// Tick `get_weathervane_yaw_rate_cds` every 250 ms through the 2 s dwell.
fn settle_weathervane(qp: &mut QuadPlane, sample: &mut WeathervaneSample) -> f32 {
    let mut rate = 0.0;
    let start = sample.now_ms;
    let mut t = start;
    while t <= start + WVANE_ACTIVATE_MS {
        sample.now_ms = t;
        rate = qp.get_weathervane_yaw_rate_cds(sample);
        t += 250;
    }
    rate
}

#[test]
fn plane_weathervane_defaults_enable_one_gain_zero() {
    let wv = WeatherVane::new();
    assert_eq!(wv.direction(), WVANE_ENABLE_DEFAULT);
    assert_eq!(wv.direction(), WeatherVaneDirection::NoseIn as i8);
    assert_eq!(WVANE_GAIN_DEFAULT as i32, 0);
    assert!(wv.gain() <= 0.0);
    assert!(wv.min_dz_ang_deg() > 0.0);
    assert_eq!(wv.min_dz_ang_deg() as i32, WVANE_ANG_MIN_DEFAULT as i32);
    assert!(wv.allowed());
}

#[test]
fn weathervane_direction_discriminants_match_upstream() {
    assert_eq!(WeatherVaneDirection::TakeoffOrLandOnly as i8, -1);
    assert_eq!(WeatherVaneDirection::Off as i8, 0);
    assert_eq!(WeatherVaneDirection::NoseIn as i8, 1);
    assert_eq!(WeatherVaneDirection::NoseOrTailIn as i8, 2);
    assert_eq!(WeatherVaneDirection::SideIn as i8, 3);
    assert_eq!(WeatherVaneDirection::TailIn as i8, 4);
    assert_eq!(
        WeatherVaneDirection::from_i8(1),
        Some(WeatherVaneDirection::NoseIn)
    );
    assert!(WeatherVaneDirection::from_i8(9).is_none());
}

#[test]
fn setup_allocates_weathervane_like_motors() {
    let mut qp = QuadPlane::new();
    assert!(!qp.weathervane_inited());
    assert!(!qp.setup());
    assert!(!qp.weathervane_inited());

    let mut qp = QuadPlane::with_enable(1);
    assert!(!qp.weathervane_inited());
    assert!(qp.setup());
    assert!(qp.weathervane_inited());
    assert!(qp.motors_inited());
    assert_eq!(
        qp.weathervane().direction(),
        WeatherVaneDirection::NoseIn as i8
    );
}

#[test]
fn weathervane_zero_when_not_in_vtol_mode() {
    let mut qp = available_qp();
    qp.weathervane_mut().set_gain(2.0);
    let mut sample = WeathervaneSample::fixed_wing();
    sample.roll_cd = 3000.0;
    assert!(qp.get_weathervane_yaw_rate_cds(&sample).abs() < 1e-6);
    assert!(qp.weathervane().last_output().abs() < 1e-6);
}

#[test]
fn weathervane_zero_in_qhover_and_qstabilize() {
    let mut qp = available_qp();
    qp.weathervane_mut().set_gain(2.0);
    let mut sample = WeathervaneSample::new();
    sample.roll_cd = 3000.0;
    sample.qhover = true;
    assert!(qp.get_weathervane_yaw_rate_cds(&sample).abs() < 1e-6);
    sample.qhover = false;
    sample.qstabilize = true;
    assert!(qp.get_weathervane_yaw_rate_cds(&sample).abs() < 1e-6);
}

#[test]
fn weathervane_zero_when_transition_disallows() {
    let mut qp = available_qp();
    qp.weathervane_mut().set_gain(2.0);
    let mut sample = WeathervaneSample::new();
    sample.allow_weathervane = false;
    sample.roll_cd = 3000.0;
    assert!(qp.get_weathervane_yaw_rate_cds(&sample).abs() < 1e-6);
}

#[test]
fn weathervane_zero_when_disarmed_or_not_unlimited() {
    let mut qp = available_qp();
    qp.weathervane_mut().set_gain(2.0);
    let mut sample = WeathervaneSample::new();
    sample.roll_cd = 3000.0;
    sample.motors_armed = false;
    assert!(qp.get_weathervane_yaw_rate_cds(&sample).abs() < 1e-6);
    sample.motors_armed = true;
    sample.throttle_unlimited = false;
    assert!(qp.get_weathervane_yaw_rate_cds(&sample).abs() < 1e-6);
}

#[test]
fn default_gain_zero_never_weathervanes() {
    let mut qp = available_qp();
    let mut sample = WeathervaneSample::new();
    sample.roll_cd = 3000.0;
    assert!(settle_weathervane(&mut qp, &mut sample).abs() < 1e-6);
}

#[test]
fn pilot_yaw_overrides_weathervane() {
    let mut qp = available_qp();
    qp.weathervane_mut().set_gain(2.0);
    let mut sample = WeathervaneSample::new();
    sample.roll_cd = 3000.0;
    sample.pilot_yaw = 200;
    assert!(settle_weathervane(&mut qp, &mut sample).abs() < 1e-6);
}

#[test]
fn nose_in_yaw_follows_roll_after_activate_dwell() {
    let mut qp = available_qp();
    qp.weathervane_mut().set_gain(2.0);
    let mut sample = WeathervaneSample::new();
    sample.roll_cd = 3000.0;
    let pos = settle_weathervane(&mut qp, &mut sample);
    assert!(pos > 0.0, "nose-in + right roll must yaw right, got {pos}");

    // Restart from a reset so the opposite roll is a clean first dwell.
    qp.weathervane_mut().reset(0);
    sample.now_ms = 10_000;
    sample.roll_cd = -3000.0;
    let neg = settle_weathervane(&mut qp, &mut sample);
    assert!(neg < 0.0, "nose-in + left roll must yaw left, got {neg}");
}

#[test]
fn weathervane_stays_zero_until_two_second_dwell() {
    let mut qp = available_qp();
    qp.weathervane_mut().set_gain(2.0);
    let mut sample = WeathervaneSample::new();
    sample.roll_cd = 3000.0;
    let mut t = 1u32;
    while t < 1 + WVANE_ACTIVATE_MS {
        sample.now_ms = t;
        assert!(
            qp.get_weathervane_yaw_rate_cds(&sample).abs() < 1e-6,
            "still in dwell at t={t}"
        );
        t += 250;
    }
    sample.now_ms = 1 + WVANE_ACTIVATE_MS;
    assert!(qp.get_weathervane_yaw_rate_cds(&sample) > 0.0);
}

#[test]
fn assist_handoff_latches_assisted_flight_when_should_assist() {
    let mut qp = available_qp();
    assert!(!qp.assisted_flight());
    assert!(!qp.in_assisted_flight());
    assert!(qp.apply_assist_handoff(true));
    assert!(qp.assisted_flight());
    assert!(qp.in_assisted_flight());
    assert!(!qp.apply_assist_handoff(false));
    assert!(!qp.assisted_flight());
    assert!(!qp.in_assisted_flight());
}

#[test]
fn assist_handoff_requires_available_for_in_assisted_flight() {
    // The latch writes `assisted_flight` even before setup; the public
    // `in_assisted_flight` query is still `available() && assisted_flight`.
    let mut qp = QuadPlane::with_enable(1);
    assert!(!qp.apply_assist_handoff(true));
    assert!(qp.assisted_flight());
    assert!(!qp.in_assisted_flight());
    assert!(qp.setup());
    assert!(qp.in_assisted_flight());
}

#[test]
fn desired_yaw_adds_auto_when_assist_handoff_is_active() {
    let mut qp = available_qp();
    let sample = WeathervaneSample::new();
    let without = qp.get_desired_yaw_rate_cds(false, 10.0, 40.0, &sample);
    assert!((without - 10.0).abs() < 0.01);
    assert!(qp.apply_assist_handoff(true));
    let with_assist = qp.get_desired_yaw_rate_cds(false, 10.0, 40.0, &sample);
    assert!((with_assist - 50.0).abs() < 0.01);
}

#[test]
fn desired_yaw_adds_weathervane_when_requested() {
    let mut qp = available_qp();
    qp.weathervane_mut().set_gain(2.0);
    let mut sample = WeathervaneSample::new();
    sample.roll_cd = 3000.0;
    let wv = settle_weathervane(&mut qp, &mut sample);
    assert!(wv > 0.0);
    let desired = qp.get_desired_yaw_rate_cds(true, 0.0, 0.0, &sample);
    assert!(desired > 0.0);
    let skipped = qp.get_desired_yaw_rate_cds(false, 0.0, 0.0, &sample);
    assert!(skipped.abs() < 1e-6);
}

#[test]
fn pilot_rate_default_is_one_hundred_dps() {
    assert_eq!(PILOT_YAW_RATE_DPS_DEFAULT as i32, 100);
}
