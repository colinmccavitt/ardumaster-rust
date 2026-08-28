//! ARMING_OPTIONS bitfield decode and apply.

use ap_arming::arming_options::{
    apply_arming_options, apply_imu_consistency_check, apply_prearm_display,
    apply_statustext_on_state_change, ArmingOption, ArmingOptions, IceState,
    ARMING_OPTIONS_DEFAULT,
};

#[test]
fn plane_default_options_are_zero() {
    assert_eq!(ARMING_OPTIONS_DEFAULT, 0);
    let opts = ArmingOptions::default();
    assert_eq!(opts.bits, ARMING_OPTIONS_DEFAULT);
    assert!(!opts.disable_prearm_display());
    assert!(!opts.disable_statustext_on_state_change());
    assert!(!opts.skip_imu_consistency_ice_running());
}

#[test]
fn each_documented_bit_decodes() {
    assert_eq!(ArmingOption::DisablePrearmDisplay.bit(), 1 << 0);
    assert_eq!(ArmingOption::DisableStatustextOnStateChange.bit(), 1 << 1);
    assert_eq!(ArmingOption::SkipImuConsistencyIceRunning.bit(), 1 << 2);

    let bits = ArmingOption::DisablePrearmDisplay.bit()
        | ArmingOption::DisableStatustextOnStateChange.bit()
        | ArmingOption::SkipImuConsistencyIceRunning.bit();
    let opts = ArmingOptions::from_bits(bits);
    assert!(opts.option_enabled(ArmingOption::DisablePrearmDisplay));
    assert!(opts.option_enabled(ArmingOption::DisableStatustextOnStateChange));
    assert!(opts.option_enabled(ArmingOption::SkipImuConsistencyIceRunning));
    assert!(opts.disable_prearm_display());
    assert!(opts.disable_statustext_on_state_change());
    assert!(opts.skip_imu_consistency_ice_running());
}

#[test]
fn unknown_bits_do_not_enable_named_options() {
    let opts = ArmingOptions::from_bits(1 << 3);
    assert!(!opts.disable_prearm_display());
    assert!(!opts.disable_statustext_on_state_change());
    assert!(!opts.skip_imu_consistency_ice_running());
}

#[test]
fn disable_prearm_display_forces_display_fail_false() {
    let default = ArmingOptions::default();
    assert!(apply_prearm_display(default, true));
    assert!(!apply_prearm_display(default, false));

    let muted = ArmingOptions::from_bits(ArmingOption::DisablePrearmDisplay.bit());
    assert!(!apply_prearm_display(muted, true));
    assert!(!apply_prearm_display(muted, false));
}

#[test]
fn disable_statustext_mutes_arm_disarm_text() {
    assert!(apply_statustext_on_state_change(ArmingOptions::default()));
    let muted = ArmingOptions::from_bits(ArmingOption::DisableStatustextOnStateChange.bit());
    assert!(!apply_statustext_on_state_change(muted));
}

#[test]
fn imu_consistency_runs_unless_option_and_ice_live() {
    let default = ArmingOptions::default();
    assert!(apply_imu_consistency_check(default, None));
    assert!(apply_imu_consistency_check(default, Some(IceState::Running)));

    let skip = ArmingOptions::from_bits(ArmingOption::SkipImuConsistencyIceRunning.bit());
    assert!(apply_imu_consistency_check(skip, None));
    assert!(apply_imu_consistency_check(skip, Some(IceState::Off)));
    assert!(!apply_imu_consistency_check(skip, Some(IceState::Starting)));
    assert!(!apply_imu_consistency_check(skip, Some(IceState::Running)));
}

#[test]
fn apply_combines_decode_and_ice_sample() {
    let opts = ArmingOptions::from_bits(
        ArmingOption::DisablePrearmDisplay.bit()
            | ArmingOption::SkipImuConsistencyIceRunning.bit(),
    );
    let applied = apply_arming_options(opts, true, Some(IceState::Starting));
    assert!(!applied.display_prearm_failures);
    assert!(applied.send_statustext);
    assert!(!applied.run_imu_consistency);

    let open = apply_arming_options(ArmingOptions::default(), true, Some(IceState::Running));
    assert!(open.display_prearm_failures);
    assert!(open.send_statustext);
    assert!(open.run_imu_consistency);
}
