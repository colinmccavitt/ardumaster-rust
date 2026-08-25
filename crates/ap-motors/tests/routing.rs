//! Behaviour of the motor output routing.
//!
//! # Why these are not parity tests
//!
//! `rc_write` and `motor_mask_to_srv_channel_mask` both go through
//! `SRV_Channels::get_motor_function` — the channel-function registry — and
//! the port has no equivalent yet. `ap-servo` models an individual channel's
//! PWM conversions, not the registry that maps a motor number to a function to
//! an output channel. That registry is the `SRV_Channel` library's own port,
//! not this ticket's.
//!
//! So these pin the routing *decision*, which is the part that belongs to
//! `AP_Motors`: which form a channel is written in, and what the scaled value
//! is. The mapping from motor number to output channel is deliberately absent
//! rather than approximated, and COP-004 records the dependency.

use ap_motors::output::{rc_write, MotorPwmScaled, PwmType, RcWrite};
use ap_motors::spool::Limits;

fn same(a: f32, b: f32) -> bool {
    (a.is_nan() && b.is_nan()) || a.to_bits() == b.to_bits()
}

/// Only the DShot rates are digital.
///
/// Brushed is a duty cycle rather than a pulse, and the two scaled types hand
/// a normalised value to the servo layer — neither is a digital ESC protocol,
/// and upstream does not treat them as one. `update_throttle_range` tests the
/// scaled types separately for exactly that reason.
#[test]
fn only_the_dshot_rates_are_digital() {
    for t in [
        PwmType::DShot150,
        PwmType::DShot300,
        PwmType::DShot600,
        PwmType::DShot1200,
    ] {
        assert!(t.is_digital(), "{t:?} should be digital");
    }
    for t in [
        PwmType::Normal,
        PwmType::OneShot,
        PwmType::OneShot125,
        PwmType::Brushed,
        PwmType::PwmRange,
        PwmType::PwmAngle,
    ] {
        assert!(!t.is_digital(), "{t:?} should not be digital");
    }
}

/// Only the two scaled types have an offset, and the offsets differ.
///
/// Range offsets by 1000 to turn a 1000-2000 pulse into 0..1000; angle offsets
/// by 1500 to turn it into -500..500. Using one for the other puts every
/// output half a range out.
#[test]
fn the_scaled_offsets_are_type_specific() {
    assert_eq!(PwmType::PwmRange.scaled_offset(), Some(1000.0));
    assert_eq!(PwmType::PwmAngle.scaled_offset(), Some(1500.0));

    for t in [
        PwmType::Normal,
        PwmType::OneShot,
        PwmType::OneShot125,
        PwmType::Brushed,
        PwmType::DShot150,
        PwmType::DShot300,
        PwmType::DShot600,
        PwmType::DShot1200,
    ] {
        assert_eq!(t.scaled_offset(), None, "{t:?} should not be scaled");
    }
}

/// The mask decides per channel, not per vehicle.
///
/// A vehicle can have some motors on scaled channels and others not, so the
/// routing has to be a per-channel question. A port that keyed it off
/// `MOT_PWM_TYPE` alone would write every channel the same way.
#[test]
fn routing_is_decided_per_channel() {
    let scaled = MotorPwmScaled {
        // Motors 0 and 3 only.
        mask: 0b1001,
        offset: 1000.0,
    };

    assert_eq!(rc_write(0, 1500, &scaled), RcWrite::Scaled(500.0));
    assert_eq!(rc_write(1, 1500, &scaled), RcWrite::Pwm(1500));
    assert_eq!(rc_write(2, 1500, &scaled), RcWrite::Pwm(1500));
    assert_eq!(rc_write(3, 1500, &scaled), RcWrite::Scaled(500.0));
}

/// The angle offset produces a signed range around zero.
#[test]
fn an_angle_channel_is_centred_on_zero() {
    let scaled = MotorPwmScaled {
        mask: 0b1,
        offset: 1500.0,
    };

    let RcWrite::Scaled(low) = rc_write(0, 1000, &scaled) else {
        panic!("expected a scaled write");
    };
    let RcWrite::Scaled(mid) = rc_write(0, 1500, &scaled) else {
        panic!("expected a scaled write");
    };
    let RcWrite::Scaled(high) = rc_write(0, 2000, &scaled) else {
        panic!("expected a scaled write");
    };

    assert!(same(low, -500.0), "got {low}");
    assert!(same(mid, 0.0), "got {mid}");
    assert!(same(high, 500.0), "got {high}");
}

/// An empty mask routes everything as a pulse width.
#[test]
fn no_scaled_channels_means_every_write_is_a_pulse() {
    let scaled = MotorPwmScaled::default();
    for chan in 0..32_u8 {
        assert_eq!(rc_write(chan, 1234, &scaled), RcWrite::Pwm(1234));
    }
}

/// A script may add limits but never remove them.
///
/// The merge is a logical OR in every axis. A script can say "I have run out
/// of authority here" and be believed; it cannot clear a limit the mixer set,
/// because the mixer knows something about the frame that the script does not.
#[test]
fn external_limits_can_only_add() {
    let mixer_said = Limits {
        roll: true,
        pitch: false,
        yaw: true,
        throttle_lower: false,
        throttle_upper: false,
    };
    let script_said = Limits {
        roll: false,
        pitch: true,
        yaw: false,
        throttle_lower: true,
        throttle_upper: false,
    };

    let mut merged = mixer_said;
    merged.merge_external(script_said);

    assert!(merged.roll, "the mixer's roll limit must survive");
    assert!(merged.pitch, "the script's pitch limit must apply");
    assert!(merged.yaw, "the mixer's yaw limit must survive");
    assert!(
        merged.throttle_lower,
        "the script's throttle limit must apply"
    );
    assert!(!merged.throttle_upper, "neither set this one");
}

/// Merging nothing changes nothing.
#[test]
fn merging_empty_external_limits_is_a_no_op() {
    let before = Limits {
        roll: true,
        pitch: false,
        yaw: true,
        throttle_lower: true,
        throttle_upper: false,
    };

    let mut after = before;
    after.merge_external(Limits::default());

    assert_eq!(after, before);
}
