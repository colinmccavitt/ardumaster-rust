//! RC_Channel PWM scale + deadzone vehicle hookup.

use ap_plane::rc_channel_scale_hookup::{scale_rc_sticks, RcChannelScaleHookup};
use ap_rc::RcChannel;

#[test]
fn hookup_default_trim_is_neutral() {
    let hookup = RcChannelScaleHookup::default();
    let sticks = hookup.publish(1500, 1500, 1500);
    assert!((sticks.roll_norm_dz).abs() < 1e-6);
    assert!((sticks.pitch_norm_dz).abs() < 1e-6);
    assert!((sticks.yaw_norm_dz).abs() < 1e-6);
}

#[test]
fn hookup_scales_deflections_with_deadzone() {
    let hookup = RcChannelScaleHookup::default();
    let inside = hookup.publish(1520, 1480, 1500);
    assert!((inside.roll_norm_dz).abs() < 1e-6);
    assert!((inside.pitch_norm_dz).abs() < 1e-6);

    let out = hookup.publish(1700, 1300, 1900);
    assert!(out.roll_norm_dz > 0.4);
    assert!(out.pitch_norm_dz < -0.4);
    assert!((out.yaw_norm_dz - 1.0).abs() < 1e-6);
}

#[test]
fn scale_rc_sticks_honors_reversed_roll() {
    let roll = RcChannel {
        reversed: true,
        ..RcChannel::default()
    };
    let pitch = RcChannel::default();
    let yaw = RcChannel::default();
    let sticks = scale_rc_sticks(&roll, &pitch, &yaw, 1900, 1500, 1500);
    assert!((sticks.roll_norm_dz + 1.0).abs() < 1e-6);
    assert!((sticks.pitch_norm_dz).abs() < 1e-6);
}
