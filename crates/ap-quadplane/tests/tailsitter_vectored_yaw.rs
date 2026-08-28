//! Tailsitter vectored-yaw tilt-motor mix — upstream `Tailsitter::output`
//! `Q_TAILSIT_VHGAIN` / `Q_TAILSIT_VFGAIN` and motors pitch±yaw tilt.

use ap_quadplane::tailsitter::{
    VectoredYawMix, SERVO_MAX, VECTORED_FORWARD_GAIN_DEFAULT, VECTORED_HOVER_GAIN_DEFAULT,
    VECTORED_HOVER_POWER_DEFAULT,
};

#[test]
fn groupinfo_defaults_match_upstream() {
    let mix = VectoredYawMix::new();
    assert!((mix.hover_gain() - VECTORED_HOVER_GAIN_DEFAULT).abs() < f32::EPSILON);
    assert!((mix.hover_gain() - 0.5).abs() < f32::EPSILON);
    assert!((mix.forward_gain() - VECTORED_FORWARD_GAIN_DEFAULT).abs() < f32::EPSILON);
    assert!(mix.forward_gain().abs() < f32::EPSILON);
    assert!((mix.hover_power() - VECTORED_HOVER_POWER_DEFAULT).abs() < f32::EPSILON);
    assert!((mix.hover_power() - 2.5).abs() < f32::EPSILON);
    assert!((SERVO_MAX - 4500.0).abs() < f32::EPSILON);
}

#[test]
fn zero_vhgain_kills_hover_tilt() {
    let mut mix = VectoredYawMix::new();
    mix.set_hover_gain(0.0);
    let (left, right) = mix.hover_tilt(1.0, 0.5);
    assert!(left.abs() < 1e-6);
    assert!(right.abs() < 1e-6);
}

#[test]
fn negative_vhgain_is_also_zero() {
    // Upstream is `vectored_hover_gain > 0`, not `>=` / abs.
    let mut mix = VectoredYawMix::new();
    mix.set_hover_gain(-0.5);
    let (left, right) = mix.hover_tilt(1.0, 0.0);
    assert!(left.abs() < 1e-6);
    assert!(right.abs() < 1e-6);
}

#[test]
fn hover_pitch_tilts_both_motors_together() {
    // pitch=1, yaw=0 → left = right = 4500 * 0.5 = 2250.
    let mix = VectoredYawMix::new();
    let (left, right) = mix.hover_tilt(1.0, 0.0);
    assert!((left - 2250.0).abs() < 1e-4);
    assert!((right - 2250.0).abs() < 1e-4);
}

#[test]
fn hover_yaw_tilts_motors_opposite() {
    // pitch=0, yaw=1 → left = (0-1)*4500*0.5 = -2250, right = +2250.
    let mix = VectoredYawMix::new();
    let (left, right) = mix.hover_tilt(0.0, 1.0);
    assert!((left - (-2250.0)).abs() < 1e-4);
    assert!((right - 2250.0).abs() < 1e-4);
}

#[test]
fn hover_pitch_and_yaw_add() {
    // pitch=0.4, yaw=0.2 → left=(0.2)*4500*0.5=450, right=(0.6)*4500*0.5=1350.
    let mix = VectoredYawMix::new();
    let (left, right) = mix.hover_tilt(0.4, 0.2);
    assert!((left - 450.0).abs() < 1e-4);
    assert!((right - 1350.0).abs() < 1e-4);
}

#[test]
fn extra_elevator_adds_to_both_tilts() {
    let mix = VectoredYawMix::new();
    let (left, right) = mix.mix_hover(1.0, 0.0, 800.0, 1.0);
    assert!((left - 3050.0).abs() < 1e-4);
    assert!((right - 3050.0).abs() < 1e-4);
}

#[test]
fn assist_throttle_scaler_scales_tilt_not_extra() {
    // Assist path: tilt * VHGAIN * scaler, extra left at 0 by the caller.
    let mix = VectoredYawMix::new();
    let (left, right) = mix.mix_hover(1.0, 0.0, 0.0, 2.0);
    assert!((left - 4500.0).abs() < 1e-4);
    assert!((right - 4500.0).abs() < 1e-4);
}

#[test]
fn default_vfgain_is_zero_so_forward_tilt_is_off() {
    let mix = VectoredYawMix::new();
    let (left, right) = mix.mix_forward(4500.0, 1000.0, 1.0);
    assert!(left.abs() < 1e-6);
    assert!(right.abs() < 1e-6);
}

#[test]
fn forward_mix_is_elevator_plus_minus_aileron() {
    let mut mix = VectoredYawMix::new();
    mix.set_forward_gain(0.5);
    // elevator 1000, aileron 400, scaler 1 → left=700, right=300.
    let (left, right) = mix.mix_forward(1000.0, 400.0, 1.0);
    assert!((left - 700.0).abs() < 1e-4);
    assert!((right - 300.0).abs() < 1e-4);
}

#[test]
fn forward_scaler_applies() {
    let mut mix = VectoredYawMix::new();
    mix.set_forward_gain(1.0);
    let (left, right) = mix.mix_forward(1000.0, 0.0, 0.5);
    assert!((left - 500.0).abs() < 1e-4);
    assert!((right - 500.0).abs() < 1e-4);
}

#[test]
fn extra_elevator_zero_when_not_vtol_or_no_error() {
    let mix = VectoredYawMix::new();
    assert!(mix.extra_elevator(2250.0, false).abs() < 1e-6);
    assert!(mix.extra_elevator(0.0, true).abs() < 1e-6);
}

#[test]
fn extra_elevator_is_power_law_of_pitch_error() {
    // extra_pitch = 2250/4500 = 0.5; 0.5^2.5 * 4500 ≈ 795.495.
    let mix = VectoredYawMix::new();
    let extra = mix.extra_elevator(2250.0, true);
    assert!((extra - 0.5f32.powf(2.5) * 4500.0).abs() < 1e-3);
}

#[test]
fn extra_elevator_full_scale_is_servo_max() {
    let mix = VectoredYawMix::new();
    let extra = mix.extra_elevator(4500.0, true);
    assert!((extra - 4500.0).abs() < 1e-3);
}

#[test]
fn extra_elevator_clamps_past_servo_max() {
    let mix = VectoredYawMix::new();
    let extra = mix.extra_elevator(9000.0, true);
    assert!((extra - 4500.0).abs() < 1e-3);
}

#[test]
fn extra_elevator_keeps_the_error_sign() {
    let mix = VectoredYawMix::new();
    let extra = mix.extra_elevator(-2250.0, true);
    assert!((extra + 0.5f32.powf(2.5) * 4500.0).abs() < 1e-3);
}

#[test]
fn zero_vhgain_kills_extra_elevator() {
    let mut mix = VectoredYawMix::new();
    mix.set_hover_gain(0.0);
    assert!(mix.extra_elevator(4500.0, true).abs() < 1e-6);
}
