//! RC_Channel PWM scale + deadzone, upstream `RC_Channel::norm_input_dz`.

use ap_rc::{
    norm_input, norm_input_dz, RcChannel, RC_CHAN_DEADZONE_DEFAULT, RC_CHAN_MAX_DEFAULT,
    RC_CHAN_MIN_DEFAULT, RC_CHAN_TRIM_DEFAULT,
};

#[test]
fn default_calibration_matches_upstream_radio_cpp() {
    let ch = RcChannel::default();
    assert_eq!(ch.radio_min, RC_CHAN_MIN_DEFAULT);
    assert_eq!(ch.radio_trim, RC_CHAN_TRIM_DEFAULT);
    assert_eq!(ch.radio_max, RC_CHAN_MAX_DEFAULT);
    assert_eq!(ch.deadzone, RC_CHAN_DEADZONE_DEFAULT);
}

#[test]
fn pwm_scales_through_trim_and_deadzone() {
    let ch = RcChannel::default();
    assert!((norm_input(1500, &ch)).abs() < 1e-6);
    assert!((norm_input(1300, &ch) + 0.5).abs() < 1e-5);
    assert!((norm_input(1700, &ch) - 0.5).abs() < 1e-5);

    // 30 µs either side of trim is still zero; just outside is not.
    assert!((norm_input_dz(1520, &ch)).abs() < 1e-6);
    let just_out = norm_input_dz(1531, &ch);
    assert!(just_out > 0.0);
    assert!(just_out < 0.02);
}

#[test]
fn out_of_range_pwm_is_constrained() {
    let ch = RcChannel::default();
    assert!((norm_input(900, &ch) + 1.0).abs() < 1e-6);
    assert!((norm_input(2100, &ch) - 1.0).abs() < 1e-6);
    assert!((norm_input_dz(900, &ch) + 1.0).abs() < 1e-6);
    assert!((norm_input_dz(2100, &ch) - 1.0).abs() < 1e-6);
}
