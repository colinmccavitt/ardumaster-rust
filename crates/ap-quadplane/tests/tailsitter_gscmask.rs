//! Tailsitter `Q_TAILSIT_GSCMSK` speed scaling — leftover
//! `Tailsitter::speed_scaling` after the transition-ramp throttle scaler.

use ap_quadplane::tailsitter::{
    GainScalePath, GainScaling, SpeedScaleInput, ATT_THR_C_TRANS_ANGLE, ATT_THR_NEG_TC,
    ATT_THR_POS_TC, DISK_LOADING_DEFAULT, GAIN_SCALING_MASK_DEFAULT, GRAVITY_MSS, SSL_AIR_DENSITY,
    TAILSITTER_GSCL_ALTITUDE, TAILSITTER_GSCL_ATT_THR, TAILSITTER_GSCL_DISK_THEORY,
    TAILSITTER_GSCL_THROTTLE,
};
use ap_quadplane::transition::{GAIN_SCALING_MIN_DEFAULT, THROTTLE_SCALE_MAX_DEFAULT};

#[test]
fn groupinfo_defaults_match_upstream() {
    let g = GainScaling::new();
    assert_eq!(GAIN_SCALING_MASK_DEFAULT, TAILSITTER_GSCL_THROTTLE);
    assert_eq!(GAIN_SCALING_MASK_DEFAULT, 1);
    assert_eq!(g.mask(), GAIN_SCALING_MASK_DEFAULT);
    assert!((g.throttle_scale_max() - THROTTLE_SCALE_MAX_DEFAULT).abs() < f32::EPSILON);
    assert!((g.gain_scaling_min() - GAIN_SCALING_MIN_DEFAULT).abs() < f32::EPSILON);
    assert!((g.disk_loading() - DISK_LOADING_DEFAULT).abs() < f32::EPSILON);
    assert!((g.last_spd_scaler() - 1.0).abs() < f32::EPSILON);
    assert!((SSL_AIR_DENSITY - 1.225).abs() < 1e-6);
    assert!((GRAVITY_MSS - 9.806_65).abs() < 1e-5);
    assert!((ATT_THR_C_TRANS_ANGLE - 0.923_879_5).abs() < 1e-6);
    assert!((ATT_THR_POS_TC - 2.0).abs() < f32::EPSILON);
    assert!((ATT_THR_NEG_TC - 1.0).abs() < f32::EPSILON);
}

#[test]
fn default_mask_is_the_throttle_path() {
    let g = GainScaling::new();
    assert_eq!(g.path(false), GainScalePath::Throttle);
    assert_eq!(g.path(true), GainScalePath::Throttle);
}

#[test]
fn throttle_path_matches_ramp_scaler() {
    let mut g = GainScaling::new();
    let mut inp = SpeedScaleInput::hover_level();
    inp.throttle = 0.2;
    let out = g.scale(&inp);
    assert_eq!(out.path, GainScalePath::Throttle);
    // hover 0.4 / throttle 0.2 = 2.0, at GSCMAX.
    assert!((out.throttle_scaler - 2.0).abs() < 1e-6);
    assert!((out.speed_scaler - 2.0).abs() < 1e-6);
}

#[test]
fn zero_throttle_is_gscmax() {
    let g = GainScaling::new();
    assert!((g.throttle_scaler(0.35, 0.0) - THROTTLE_SCALE_MAX_DEFAULT).abs() < 1e-6);
}

#[test]
fn att_thr_wins_over_disk_and_throttle() {
    let mut g = GainScaling::new();
    g.set_mask(TAILSITTER_GSCL_ATT_THR | TAILSITTER_GSCL_DISK_THEORY | TAILSITTER_GSCL_THROTTLE);
    g.set_disk_loading(5.0);
    assert_eq!(g.path(true), GainScalePath::AttThr);
}

#[test]
fn disk_theory_needs_positive_dskld() {
    let mut g = GainScaling::new();
    g.set_mask(TAILSITTER_GSCL_DISK_THEORY);
    assert_eq!(g.path(true), GainScalePath::Unity);
    g.set_disk_loading(8.0);
    assert_eq!(g.path(true), GainScalePath::DiskTheory);
    assert_eq!(g.path(false), GainScalePath::DiskTheoryFallback);
}

#[test]
fn disk_fallback_uses_throttle_scaler() {
    let mut g = GainScaling::new();
    g.set_mask(TAILSITTER_GSCL_DISK_THEORY);
    g.set_disk_loading(8.0);
    let inp = SpeedScaleInput::hover_level();
    let out = g.scale(&inp);
    assert_eq!(out.path, GainScalePath::DiskTheoryFallback);
    assert!((out.speed_scaler - out.throttle_scaler).abs() < 1e-6);
    assert!((out.throttle_scaler - 1.0).abs() < 1e-6);
}

#[test]
fn disk_theory_hover_at_rest_is_one() {
    // t/t_h = 1, U0 = 0 → Ue^2 == hover Ue^2 → scaler 1.
    let mut g = GainScaling::new();
    g.set_mask(TAILSITTER_GSCL_DISK_THEORY);
    g.set_disk_loading(8.0);
    let mut inp = SpeedScaleInput::hover_level();
    inp.have_airspeed = true;
    inp.airspeed = 0.0;
    let out = g.scale(&inp);
    assert_eq!(out.path, GainScalePath::DiskTheory);
    assert!((out.speed_scaler - 1.0).abs() < 1e-5);
}

#[test]
fn disk_theory_airspeed_reduces_scaler() {
    let mut g = GainScaling::new();
    g.set_mask(TAILSITTER_GSCL_DISK_THEORY);
    g.set_disk_loading(8.0);
    let mut inp = SpeedScaleInput::hover_level();
    inp.have_airspeed = true;
    inp.airspeed = 15.0;
    let out = g.scale(&inp);
    assert_eq!(out.path, GainScalePath::DiskTheory);
    assert!(out.speed_scaler < 1.0);
    assert!(out.speed_scaler >= GAIN_SCALING_MIN_DEFAULT - 1e-6);
}

#[test]
fn empty_mask_is_unity() {
    let mut g = GainScaling::new();
    g.set_mask(0);
    let out = g.scale(&SpeedScaleInput::hover_level());
    assert_eq!(out.path, GainScalePath::Unity);
    assert!((out.speed_scaler - 1.0).abs() < 1e-6);
}

#[test]
fn att_thr_level_hover_stays_one_then_maybe_throttle() {
    // Level, throttle == hover → pre-slew 1, slew 1, then THROTTLE bit
    // is also set on the default mask so MAX(throttle_scaler, 1) = 1.
    let mut g = GainScaling::new();
    g.set_mask(TAILSITTER_GSCL_ATT_THR);
    let out = g.scale(&SpeedScaleInput::hover_level());
    assert_eq!(out.path, GainScalePath::AttThr);
    assert!((out.speed_scaler - 1.0).abs() < 1e-6);
}

#[test]
fn att_thr_plus_throttle_lifts_when_throttle_is_low() {
    let mut g = GainScaling::new();
    g.set_mask(TAILSITTER_GSCL_ATT_THR | TAILSITTER_GSCL_THROTTLE);
    let mut inp = SpeedScaleInput::hover_level();
    inp.throttle = 0.2; // hover/throttle = 2
    let out = g.scale(&inp);
    assert_eq!(out.path, GainScalePath::AttThr);
    // pre-slew 1 (level, throttle 0.2 < 1.25*0.4), slew stays 1, then MAX(2, 1).
    assert!((out.speed_scaler - 2.0).abs() < 1e-6);
}

#[test]
fn att_thr_tilt_reduces_pre_slew() {
    let g = GainScaling::new();
    // c.z = 0 is 90 deg tilt, well past c_trans_angle.
    let pre = g.att_thr_pre_slew(0.4, 0.4, 0.0);
    assert!(pre < 1.0);
    assert!(pre >= GAIN_SCALING_MIN_DEFAULT - 1e-6);
}

#[test]
fn att_thr_slew_limits_step() {
    let mut g = GainScaling::new();
    let dt = 0.002_5;
    let limited = g.slew(0.0, dt);
    // last starts at 1; negdelta = dt / 1 = 0.0025 → 0.9975.
    assert!((limited - (1.0 - dt / ATT_THR_NEG_TC)).abs() < 1e-6);
    assert!((g.last_spd_scaler() - limited).abs() < 1e-6);
}

#[test]
fn altitude_divides_by_density_ratio() {
    let mut g = GainScaling::new();
    g.set_mask(TAILSITTER_GSCL_THROTTLE | TAILSITTER_GSCL_ALTITUDE);
    let mut inp = SpeedScaleInput::hover_level();
    inp.density_ratio = 0.8;
    let out = g.scale(&inp);
    // throttle path scaler 1, then / 0.8.
    assert!((out.speed_scaler - 1.25).abs() < 1e-5);
}

#[test]
fn surfaces_take_speed_scaler_tilts_take_throttle() {
    assert!((GainScaling::scale_surface(1000.0, 0.5) - 500.0).abs() < 1e-6);
    assert!((GainScaling::scale_tilt(1000.0, 2.0) - 2000.0).abs() < 1e-6);
}
