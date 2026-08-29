//! Hardware send leftover — `SRV_Channels::push`. COP-030.
//!
//! The interesting content is a fan-out order: `hal.rcout->push()` first,
//! then each enabled output protocol. Collapsing the order, visiting a
//! compiled-out protocol, or treating a null CAN driver as present would
//! all still "send something" and look fine on a board that only uses PWM.

use ap_hal::rc::{MockRcOutput, RcOutput, MAX_RC_CHANNELS};
use ap_servo::push::{
    cork, protocol_visits, push, zero_rc_outputs, CanDriver, PushFeatures, PushVisit, REMAINING,
};
use ap_servo::NUM_SERVO_CHANNELS;

fn visits_of(features: PushFeatures, can: &[CanDriver]) -> Vec<PushVisit> {
    let mut rcout = MockRcOutput::new();
    let mut out = Vec::new();
    push(&mut rcout, features, can, |v| out.push(v));
    assert_eq!(rcout.push_count(), 1, "rcout.push must run exactly once");
    out
}

fn protocols_of(features: PushFeatures, can: &[CanDriver]) -> Vec<PushVisit> {
    let mut buf = [PushVisit::Rcout; 16];
    let n = protocol_visits(features, can, &mut buf);
    buf.get(..n).unwrap_or(&[]).to_vec()
}

#[test]
fn leftover_catalog_is_empty() {
    assert!(REMAINING.is_empty());
    assert!(!REMAINING.contains(&"push"));
    assert!(!REMAINING.contains(&"upgrade_parameters"));
}

#[test]
fn rcout_runs_even_when_every_protocol_is_compiled_out() {
    let got = visits_of(PushFeatures::NONE, &[]);
    assert_eq!(got, [PushVisit::Rcout]);
}

#[test]
fn rcout_is_always_first() {
    let can = [
        CanDriver::DroneCan { present: true },
        CanDriver::PiccoloCan { present: true },
    ];
    let got = visits_of(PushFeatures::ALL, &can);
    assert_eq!(got.first().copied(), Some(PushVisit::Rcout));
    assert_eq!(
        got,
        [
            PushVisit::Rcout,
            PushVisit::Volz,
            PushVisit::Sbus,
            PushVisit::Robotis,
            PushVisit::BlheliTelemetry,
            PushVisit::FettecOnewire,
            PushVisit::KdeCan,
            PushVisit::DroneCan { driver: 0 },
            PushVisit::PiccoloCan { driver: 1 },
        ]
    );
}

#[test]
fn compiled_out_protocols_are_not_visited() {
    let mut features = PushFeatures::ALL;
    features.volz = false;
    features.robotis = false;
    features.fettec = false;
    features.can_drivers = false;
    let got = protocols_of(features, &[CanDriver::DroneCan { present: true }]);
    assert_eq!(
        got,
        [
            PushVisit::Sbus,
            PushVisit::BlheliTelemetry,
            PushVisit::KdeCan,
        ]
    );
}

#[test]
fn kdecan_feature_without_a_singleton_is_skipped() {
    let mut features = PushFeatures::NONE;
    features.kdecan = true;
    features.kdecan_present = false;
    assert!(protocols_of(features, &[]).is_empty());

    features.kdecan_present = true;
    assert_eq!(protocols_of(features, &[]), [PushVisit::KdeCan]);
}

#[test]
fn can_loop_is_gated_as_a_whole() {
    let can = [CanDriver::DroneCan { present: true }];
    let mut features = PushFeatures::NONE;
    features.can_drivers = false;
    assert!(protocols_of(features, &can).is_empty());

    features.can_drivers = true;
    assert_eq!(
        protocols_of(features, &can),
        [PushVisit::DroneCan { driver: 0 }]
    );
}

/// A leading unused slot still occupies its index. A port that compacted
/// successful visits would call `SRV_push_servos` on the wrong driver.
#[test]
fn can_driver_index_is_the_slot_not_a_compact_count() {
    let can = [
        CanDriver::None,
        CanDriver::DroneCan { present: true },
        CanDriver::PiccoloCan { present: true },
    ];
    let mut features = PushFeatures::NONE;
    features.can_drivers = true;
    features.piccolocan = true;
    assert_eq!(
        protocols_of(features, &can),
        [
            PushVisit::DroneCan { driver: 1 },
            PushVisit::PiccoloCan { driver: 2 },
        ]
    );
}

#[test]
fn null_dronecan_pointer_is_a_skip() {
    let can = [
        CanDriver::DroneCan { present: false },
        CanDriver::DroneCan { present: true },
    ];
    let mut features = PushFeatures::NONE;
    features.can_drivers = true;
    assert_eq!(
        protocols_of(features, &can),
        [PushVisit::DroneCan { driver: 1 }]
    );
}

/// PiccoloCAN compiled out falls through to the default branch even when
/// the slot's type is PiccoloCAN.
#[test]
fn piccolocan_compiled_out_is_not_visited() {
    let can = [CanDriver::PiccoloCan { present: true }];
    let mut features = PushFeatures::NONE;
    features.can_drivers = true;
    features.piccolocan = false;
    assert!(protocols_of(features, &can).is_empty());

    features.piccolocan = true;
    assert_eq!(
        protocols_of(features, &can),
        [PushVisit::PiccoloCan { driver: 0 }]
    );
}

#[test]
fn cork_does_not_fan_out() {
    let mut rcout = MockRcOutput::new();
    cork(&mut rcout);
    assert_eq!(rcout.cork_count(), 1);
    assert_eq!(rcout.push_count(), 0);
}

/// A 1500 µs "neutral" cut short looks like throttle, so every channel
/// gets an invalid pulse. Then the same fan-out as `push`.
#[test]
fn zero_rc_outputs_corks_writes_zero_then_pushes() {
    let mut rcout = MockRcOutput::new();
    let mut got = Vec::new();
    zero_rc_outputs(&mut rcout, PushFeatures::NONE, &[], |v| got.push(v));

    assert_eq!(rcout.cork_count(), 1);
    assert_eq!(rcout.push_count(), 1);
    assert_eq!(got, [PushVisit::Rcout]);

    let writable = MAX_RC_CHANNELS.min(NUM_SERVO_CHANNELS);
    for ch in 0..writable {
        assert_eq!(
            rcout.read(ch as u8),
            Some(0),
            "channel {ch} must be the invalid pulse"
        );
    }
}

#[test]
fn protocol_visits_stop_when_the_buffer_is_full() {
    let mut buf = [PushVisit::Rcout; 2];
    let n = protocol_visits(PushFeatures::ALL, &[], &mut buf);
    assert_eq!(n, 2);
    assert_eq!(buf, [PushVisit::Volz, PushVisit::Sbus]);
}
