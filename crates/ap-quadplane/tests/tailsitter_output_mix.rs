//! Tailsitter copter output mix / write_log / setup leftover —
//! leftover `Tailsitter::output` after `motors_output`, plus
//! `write_log` and the `setup` surface / enable-always bits.

use ap_quadplane::air_mode::AirMode;
use ap_quadplane::tailsitter::{
    unconfigured_transition_rate_fw, CopterOutputMix, MotorAttitude, SurfaceAssign, Tailsitter,
    TailsitterConfig, MIXING_OFFSET_DEFAULT, PLANE_MIXING_GAIN_DEFAULT,
    Q_OPTIONS_ONLY_ARM_IN_QMODE_OR_AUTO, SERVO_MAX, TAILSITTER_MIXING_GAIN_DEFAULT,
    TAILSITTER_TRANSITION_MS_DEFAULT, TAILSIT_ENABLE_ALWAYS, VTOL_PITCH_SCALE_DEFAULT,
    VTOL_ROLL_SCALE_DEFAULT, VTOL_YAW_SCALE_DEFAULT,
};
use ap_quadplane::transition::TRANSITION_ANGLE_FW_DEFAULT;

#[test]
fn groupinfo_and_table_defaults_match_upstream() {
    let mix = CopterOutputMix::new();
    assert!((mix.roll_scale() - VTOL_ROLL_SCALE_DEFAULT).abs() < f32::EPSILON);
    assert!((mix.pitch_scale() - VTOL_PITCH_SCALE_DEFAULT).abs() < f32::EPSILON);
    assert!((mix.yaw_scale() - VTOL_YAW_SCALE_DEFAULT).abs() < f32::EPSILON);
    assert!((mix.roll_scale() - 1.0).abs() < f32::EPSILON);
    assert!((mix.mixing_gain() - TAILSITTER_MIXING_GAIN_DEFAULT).abs() < f32::EPSILON);
    assert!((mix.mixing_gain() - 1.0).abs() < f32::EPSILON);
    assert!((PLANE_MIXING_GAIN_DEFAULT - 0.5).abs() < f32::EPSILON);
    assert_eq!(mix.mixing_offset(), MIXING_OFFSET_DEFAULT);
    assert_eq!(mix.mixing_offset(), 0);
    assert_eq!(mix.surface_assign(), SurfaceAssign::NONE);
}

#[test]
fn yaw_writes_negated_aileron() {
    // yaw=1 → aileron = 1 * -4500 * 1 = -4500.
    let mix = CopterOutputMix::conventional();
    let mut att = MotorAttitude::zero();
    att.yaw = 1.0;
    let s = mix.surfaces(att);
    assert!((s.aileron - (-SERVO_MAX)).abs() < 1e-4);
    assert!(s.elevator.abs() < 1e-6);
    assert!(s.rudder.abs() < 1e-6);
}

#[test]
fn pitch_writes_elevator() {
    let mix = CopterOutputMix::conventional();
    let mut att = MotorAttitude::zero();
    att.pitch = 0.5;
    let s = mix.surfaces(att);
    assert!((s.elevator - 2250.0).abs() < 1e-4);
    assert!(s.aileron.abs() < 1e-6);
    assert!(s.rudder.abs() < 1e-6);
}

#[test]
fn roll_writes_rudder() {
    let mix = CopterOutputMix::conventional();
    let mut att = MotorAttitude::zero();
    att.roll = -0.2;
    let s = mix.surfaces(att);
    assert!((s.rudder - (-900.0)).abs() < 1e-4);
}

#[test]
fn feedforward_adds_before_scale() {
    let mix = CopterOutputMix::conventional();
    let att = MotorAttitude {
        yaw: 0.2,
        yaw_ff: 0.1,
        pitch: 0.0,
        pitch_ff: 0.4,
        roll: 0.3,
        roll_ff: -0.1,
    };
    let s = mix.surfaces(att);
    assert!((s.aileron - (-1350.0)).abs() < 1e-4);
    assert!((s.elevator - 1800.0).abs() < 1e-4);
    assert!((s.rudder - 900.0).abs() < 1e-4);
}

#[test]
fn vtol_scales_favour_surfaces() {
    let mut mix = CopterOutputMix::conventional();
    mix.set_yaw_scale(2.0);
    mix.set_pitch_scale(0.5);
    mix.set_roll_scale(0.0);
    let mut att = MotorAttitude::zero();
    att.yaw = 1.0;
    att.pitch = 1.0;
    att.roll = 1.0;
    let s = mix.surfaces(att);
    assert!((s.aileron - (-9000.0)).abs() < 1e-4);
    assert!((s.elevator - 2250.0).abs() < 1e-4);
    assert!(s.rudder.abs() < 1e-6);
}

#[test]
fn elevon_mix_is_elevator_plus_minus_aileron() {
    let mix = CopterOutputMix::elevon_vtail();
    let mut att = MotorAttitude::zero();
    att.pitch = 0.4; // elevator = 1800
    att.yaw = 0.2; // aileron = -900
    let out = mix.mix(att, 0.0, 0.0, false);
    // elevon_left = 1800 - (-900) = 2700, elevon_right = 1800 + (-900) = 900
    assert!((out.elevon_vtail.elevon_left - 2700.0).abs() < 1e-3);
    assert!((out.elevon_vtail.elevon_right - 900.0).abs() < 1e-3);
}

#[test]
fn vtail_mix_is_elevator_plus_minus_rudder() {
    let mix = CopterOutputMix::elevon_vtail();
    let mut att = MotorAttitude::zero();
    att.pitch = 0.4; // elevator = 1800
    att.roll = 0.2; // rudder = 900
    let out = mix.mix(att, 0.0, 0.0, false);
    // vtail_left = 1800 + 900 = 2700, vtail_right = 1800 - 900 = 900
    assert!((out.elevon_vtail.vtail_left - 2700.0).abs() < 1e-3);
    assert!((out.elevon_vtail.vtail_right - 900.0).abs() < 1e-3);
}

#[test]
fn mixing_offset_favours_aileron() {
    // offset +50: elevator * 0.5, aileron * 1.5 (gain 1).
    let mut mix = CopterOutputMix::elevon_vtail();
    mix.set_mixing_offset(50);
    let mut att = MotorAttitude::zero();
    att.pitch = 0.4; // elevator 1800 → mix 900
    att.yaw = 0.2; // aileron -900 → mix -1350
    let out = mix.mix(att, 0.0, 0.0, false);
    assert!((out.elevon_vtail.elevon_left - (900.0 - (-1350.0))).abs() < 1e-3);
    assert!((out.elevon_vtail.elevon_right - (900.0 + (-1350.0))).abs() < 1e-3);
}

#[test]
fn headroom_clips_aileron_and_sets_yaw_on_elevon() {
    // elevator = 4000, aileron = -2000 → headroom 500, |aileron_mix| 2000.
    let mix = CopterOutputMix::elevon_vtail();
    let mut att = MotorAttitude::zero();
    att.pitch = 4000.0 / SERVO_MAX;
    att.yaw = 2000.0 / SERVO_MAX; // aileron = -2000
    let out = mix.mix(att, 0.0, 0.0, false);
    let headroom = SERVO_MAX - 4000.0;
    assert!((out.elevon_vtail.elevon_left - (4000.0 - (-headroom))).abs() < 1e-3);
    assert!((out.elevon_vtail.elevon_right - (4000.0 + (-headroom))).abs() < 1e-3);
    assert!(out.limits.yaw);
    assert!(!out.limits.roll);
    assert!(!out.limits.pitch);
}

#[test]
fn zero_headroom_zeros_aileron_rudder_and_trips_elevon_vtail_limits() {
    let mix = CopterOutputMix::elevon_vtail();
    let mut att = MotorAttitude::zero();
    att.pitch = 1.0; // elevator = 4500, headroom 0
    att.yaw = 0.2;
    att.roll = 0.2;
    let out = mix.mix(att, 0.0, 0.0, false);
    assert!((out.elevon_vtail.elevon_left - SERVO_MAX).abs() < 1e-3);
    assert!((out.elevon_vtail.elevon_right - SERVO_MAX).abs() < 1e-3);
    assert!((out.elevon_vtail.vtail_left - SERVO_MAX).abs() < 1e-3);
    assert!((out.elevon_vtail.vtail_right - SERVO_MAX).abs() < 1e-3);
    assert!(out.limits.yaw);
    assert!(out.limits.pitch);
    assert!(out.limits.roll);
}

#[test]
fn dedicated_surface_saturation_needs_assignment() {
    let mut att = MotorAttitude::zero();
    att.yaw = 1.0; // aileron = -4500
    att.pitch = 1.0; // elevator = 4500
    att.roll = 1.0; // rudder = 4500

    let none = CopterOutputMix::new().mix(att, 0.0, 0.0, false);
    assert!(!none.limits.roll);
    assert!(!none.limits.pitch);
    assert!(!none.limits.yaw);

    let conv = CopterOutputMix::conventional().mix(att, 0.0, 0.0, false);
    assert!(conv.limits.roll);
    assert!(conv.limits.pitch);
    assert!(conv.limits.yaw);
}

#[test]
fn vectored_tilt_saturation_sets_pitch_and_yaw() {
    let mix = CopterOutputMix::conventional();
    let out = mix.mix(MotorAttitude::zero(), SERVO_MAX, 0.0, true);
    assert!(out.limits.pitch);
    assert!(out.limits.yaw);
    assert!(!out.limits.roll);

    let not_vectored = mix.mix(MotorAttitude::zero(), SERVO_MAX, 0.0, false);
    assert!(!not_vectored.limits.pitch);
    assert!(!not_vectored.limits.yaw);
}

#[test]
fn setup_records_surface_assign() {
    let mut cfg = TailsitterConfig::tailsitter_frame();
    cfg.surfaces = SurfaceAssign::CONVENTIONAL;
    let ts = Tailsitter::setup(cfg);
    assert_eq!(ts.surface_assign(), SurfaceAssign::CONVENTIONAL);
    assert!(ts.surface_assign().elevator);
    assert!(!ts.surface_assign().elevon);
}

#[test]
fn write_log_is_none_when_disabled() {
    let ts = Tailsitter::setup(TailsitterConfig::new());
    assert!(!ts.enabled());
    assert!(ts.write_log(1, 1.0, 1.0, 0.0).is_none());
}

#[test]
fn write_log_copies_scalers_when_enabled() {
    let ts = Tailsitter::setup(TailsitterConfig::tailsitter_frame());
    let pkt = ts.write_log(42, 1.25, 0.8, 0.15).expect("enabled");
    assert_eq!(pkt.time_us, 42);
    assert!((pkt.throttle_scaler - 1.25).abs() < f32::EPSILON);
    assert!((pkt.speed_scaler - 0.8).abs() < f32::EPSILON);
    assert!((pkt.min_throttle - 0.15).abs() < f32::EPSILON);
}

#[test]
fn enable_always_forces_assist_and_qmode_arm() {
    let mut cfg = TailsitterConfig::tailsitter_frame();
    cfg.enable = Some(TAILSIT_ENABLE_ALWAYS);
    let ts = Tailsitter::setup(cfg);
    assert!(ts.enabled());
    let fx = ts.enable_always_setup().expect("enable 2");
    assert!(fx.force_assist);
    assert_eq!(fx.air_mode, AirMode::AssistedFlightOnly);
    assert_eq!(fx.only_arm_option, Q_OPTIONS_ONLY_ARM_IN_QMODE_OR_AUTO);
    assert_eq!(fx.only_arm_option, 1 << 18);
}

#[test]
fn enable_one_has_no_always_side_effects() {
    let ts = Tailsitter::setup(TailsitterConfig::tailsitter_frame());
    assert_eq!(ts.enable(), 1);
    assert!(ts.enable_always_setup().is_none());
}

#[test]
fn unconfigured_rate_fw_uses_angle_over_half_transition() {
    // tailsitter table Q_TRANSITION_MS = 2000 → 45 / 1 = 45.
    let rate = unconfigured_transition_rate_fw(
        TRANSITION_ANGLE_FW_DEFAULT,
        TAILSITTER_TRANSITION_MS_DEFAULT,
    );
    assert!((rate - 45.0).abs() < 1e-4);
    // QuadPlane GROUPINFO 5000 → 45 / 2.5 = 18.
    let rate = unconfigured_transition_rate_fw(TRANSITION_ANGLE_FW_DEFAULT, 5000);
    assert!((rate - 18.0).abs() < 1e-4);
}
