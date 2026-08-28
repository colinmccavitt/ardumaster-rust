//! AUTOTUNE_OPTIONS filter bits and AUTOTUNE_AXES single-axis start mask.

use ap_autotune::options::{
    apply_filter_options, fltd_hz, fltt_hz, AutotuneAxes, AutotuneOption, AutotuneOptions,
    AUTOTUNE_AXES_DEFAULT, AUTOTUNE_AXIS_PITCH, AUTOTUNE_AXIS_ROLL, AUTOTUNE_AXIS_YAW,
    AUTOTUNE_OPTION_DISABLE_FLTD_UPDATE, AUTOTUNE_OPTION_DISABLE_FLTT_UPDATE,
    AUTOTUNE_OPTIONS_DEFAULT,
};
use ap_autotune::state::AtType;

fn close(a: f32, b: f32) {
    assert!((a - b).abs() < 1e-5, "{a} != {b}");
}

#[test]
fn options_default_is_zero() {
    assert_eq!(AUTOTUNE_OPTIONS_DEFAULT, 0);
    let opts = AutotuneOptions::default();
    assert_eq!(opts.bits, AUTOTUNE_OPTIONS_DEFAULT);
    assert!(!opts.disable_fltd_update());
    assert!(!opts.disable_fltt_update());
}

#[test]
fn options_bits_match_upstream_enum() {
    assert_eq!(AutotuneOption::DisableFltdUpdate.as_u8(), 0);
    assert_eq!(AutotuneOption::DisableFlttUpdate.as_u8(), 1);
    assert_eq!(
        AutotuneOption::DisableFltdUpdate.bit(),
        AUTOTUNE_OPTION_DISABLE_FLTD_UPDATE
    );
    assert_eq!(
        AutotuneOption::DisableFlttUpdate.bit(),
        AUTOTUNE_OPTION_DISABLE_FLTT_UPDATE
    );
    assert_eq!(AutotuneOption::from_u8(0), Some(AutotuneOption::DisableFltdUpdate));
    assert_eq!(AutotuneOption::from_u8(1), Some(AutotuneOption::DisableFlttUpdate));
    assert_eq!(AutotuneOption::from_u8(2), None);
}

#[test]
fn has_option_decodes_each_filter_bit() {
    let both = AutotuneOptions::from_bits(
        AUTOTUNE_OPTION_DISABLE_FLTD_UPDATE | AUTOTUNE_OPTION_DISABLE_FLTT_UPDATE,
    );
    assert!(both.has_option(AutotuneOption::DisableFltdUpdate));
    assert!(both.has_option(AutotuneOption::DisableFlttUpdate));
    assert!(both.disable_fltd_update());
    assert!(both.disable_fltt_update());

    let unknown = AutotuneOptions::from_bits(1 << 2);
    assert!(!unknown.disable_fltd_update());
    assert!(!unknown.disable_fltt_update());
}

#[test]
fn apply_filter_options_writes_unless_disabled() {
    let tau = 0.50;
    let gyro = 20.0;
    close(fltt_hz(tau), 10.0 / (tau * 2.0 * core::f32::consts::PI));
    close(fltd_hz(gyro), 10.0);

    let open = apply_filter_options(AutotuneOptions::default(), tau, gyro);
    close(open.fltt_hz.expect("FLTT written"), fltt_hz(tau));
    close(open.flte_hz, 0.0);
    close(open.fltd_hz.expect("FLTD written"), 10.0);

    let muted = apply_filter_options(
        AutotuneOptions::from_bits(
            AUTOTUNE_OPTION_DISABLE_FLTD_UPDATE | AUTOTUNE_OPTION_DISABLE_FLTT_UPDATE,
        ),
        tau,
        gyro,
    );
    assert!(muted.fltt_hz.is_none());
    close(muted.flte_hz, 0.0);
    assert!(muted.fltd_hz.is_none());
}

#[test]
fn axes_default_tunes_roll_pitch_and_yaw() {
    assert_eq!(AUTOTUNE_AXES_DEFAULT, 7);
    assert_eq!(
        AUTOTUNE_AXES_DEFAULT,
        AUTOTUNE_AXIS_ROLL | AUTOTUNE_AXIS_PITCH | AUTOTUNE_AXIS_YAW
    );
    let axes = AutotuneAxes::default();
    assert!(axes.tune_roll());
    assert!(axes.tune_pitch());
    assert!(axes.tune_yaw());
    assert!(axes.any_selected());
    assert!(axes.starts_type(AtType::Roll));
    assert!(axes.starts_type(AtType::Pitch));
    assert!(axes.starts_type(AtType::Yaw));
}

#[test]
fn single_axis_roll_only_pitch_only_or_both() {
    let roll = AutotuneAxes::roll_only();
    assert_eq!(roll.bits, 1);
    assert!(roll.tune_roll());
    assert!(!roll.tune_pitch());
    assert!(!roll.tune_yaw());
    assert!(roll.any_selected());
    assert!(roll.starts_type(AtType::Roll));
    assert!(!roll.starts_type(AtType::Pitch));

    let pitch = AutotuneAxes::pitch_only();
    assert_eq!(pitch.bits, 2);
    assert!(!pitch.tune_roll());
    assert!(pitch.tune_pitch());
    assert!(!pitch.tune_yaw());
    assert!(pitch.starts_type(AtType::Pitch));
    assert!(!pitch.starts_type(AtType::Roll));

    let both = AutotuneAxes::roll_and_pitch();
    assert_eq!(both.bits, 3);
    assert!(both.tune_roll());
    assert!(both.tune_pitch());
    assert!(!both.tune_yaw());
    assert!(both.starts_type(AtType::Roll));
    assert!(both.starts_type(AtType::Pitch));
    assert!(!both.starts_type(AtType::Yaw));
}

#[test]
fn empty_axis_mask_starts_nothing() {
    let none = AutotuneAxes::from_bits(0);
    assert!(!none.any_selected());
    assert!(!none.tune_roll());
    assert!(!none.tune_pitch());
    assert!(!none.tune_yaw());
    assert!(!none.starts_type(AtType::Roll));
    assert!(!none.starts_type(AtType::Pitch));
    assert!(!none.starts_type(AtType::Yaw));
}
