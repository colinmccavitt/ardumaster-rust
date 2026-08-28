//! MANUAL_CONTROL / RC_CHANNELS_OVERRIDE ingest from a framed packet.

use ap_gcs::{
    decode_v2, encode_v2, Dispatch, Frame, GcsMavlink, ManualControl, RcChannelsOverride,
    MANUAL_CONTROL_CRC, MANUAL_CONTROL_MIN_LEN, MSG_ID_MANUAL_CONTROL, MSG_ID_RC_CHANNELS_OVERRIDE,
    OVERRIDE_CHANNEL_COUNT, OVERRIDE_IGNORE, RC_CHANNELS_OVERRIDE_CRC, RC_CHANNELS_OVERRIDE_LEN,
};

fn override_frame(pkt: &RcChannelsOverride, sysid: u8) -> Frame {
    let mut payload = [0u8; RC_CHANNELS_OVERRIDE_LEN];
    assert_eq!(pkt.encode(&mut payload), Some(RC_CHANNELS_OVERRIDE_LEN));
    Frame::new(5, sysid, 190, MSG_ID_RC_CHANNELS_OVERRIDE, &payload).expect("frame")
}

fn manual_frame(pkt: &ManualControl, sysid: u8) -> Frame {
    let mut payload = [0u8; MANUAL_CONTROL_MIN_LEN];
    assert_eq!(pkt.encode(&mut payload), Some(MANUAL_CONTROL_MIN_LEN));
    Frame::new(6, sysid, 190, MSG_ID_MANUAL_CONTROL, &payload).expect("frame")
}

fn sample_override() -> RcChannelsOverride {
    let mut chan = [OVERRIDE_IGNORE; OVERRIDE_CHANNEL_COUNT];
    if let Some(ch) = chan.get_mut(0) {
        *ch = 1_510;
    }
    if let Some(ch) = chan.get_mut(1) {
        *ch = 1_480;
    }
    if let Some(ch) = chan.get_mut(2) {
        *ch = 1_100;
    }
    if let Some(ch) = chan.get_mut(3) {
        *ch = 1_620;
    }
    if let Some(ch) = chan.get_mut(8) {
        *ch = 1_400;
    }
    RcChannelsOverride::new(1, 1, chan)
}

#[test]
fn rc_channels_override_payload_roundtrip() {
    let pkt = sample_override();
    let mut buf = [0u8; RC_CHANNELS_OVERRIDE_LEN];
    assert_eq!(pkt.encode(&mut buf), Some(RC_CHANNELS_OVERRIDE_LEN));
    let decoded = RcChannelsOverride::decode(&buf).expect("payload");
    assert_eq!(decoded.target_system, 1);
    assert_eq!(decoded.target_component, 1);
    assert_eq!(decoded.chan.get(0).copied(), Some(1_510));
    assert_eq!(decoded.chan.get(1).copied(), Some(1_480));
    assert_eq!(decoded.chan.get(8).copied(), Some(1_400));
    assert_eq!(decoded.chan.get(4).copied(), Some(OVERRIDE_IGNORE));
}

#[test]
fn manual_control_payload_roundtrip() {
    let pkt = ManualControl::new(1, 200, -100, 800, 50, 0x0003);
    let mut buf = [0u8; MANUAL_CONTROL_MIN_LEN];
    assert_eq!(pkt.encode(&mut buf), Some(MANUAL_CONTROL_MIN_LEN));
    let decoded = ManualControl::decode(&buf).expect("payload");
    assert_eq!(decoded, pkt);
}

#[test]
fn override_frames_use_pinned_crc_extras() {
    assert_eq!(RC_CHANNELS_OVERRIDE_CRC, 124);
    assert_eq!(MANUAL_CONTROL_CRC, 243);

    let pkt = sample_override();
    let frame = override_frame(&pkt, 255);
    let mut wire = [0u8; 64];
    let n = encode_v2(&frame, &mut wire).expect("encode");
    assert_eq!(n, 10 + RC_CHANNELS_OVERRIDE_LEN + 2);
    let parsed = decode_v2(wire.get(..n).expect("slice")).expect("decode");
    assert_eq!(parsed.msgid, MSG_ID_RC_CHANNELS_OVERRIDE);
    let again = RcChannelsOverride::from_frame(&parsed).expect("from_frame");
    assert_eq!(again.chan.get(0).copied(), Some(1_510));
}

#[test]
fn framed_rc_override_stores_channel_pwms() {
    let mut gcs = GcsMavlink::new();
    let pkt = sample_override();
    let frame = override_frame(&pkt, 255);
    match gcs.handle_message(&frame, 4_000) {
        Dispatch::RcChannelsOverride { applied } => assert_eq!(applied, 5),
        other => panic!("unexpected {other:?}"),
    }
    assert_eq!(gcs.override_channel(0), Some(1_510));
    assert_eq!(gcs.override_channel(1), Some(1_480));
    assert_eq!(gcs.override_channel(2), Some(1_100));
    assert_eq!(gcs.override_channel(3), Some(1_620));
    assert_eq!(gcs.override_channel(4), None);
    assert_eq!(gcs.override_channel(8), Some(1_400));
    assert_eq!(gcs.last_override_ms(), 4_000);
    assert_eq!(gcs.last_gcs_heartbeat_ms(), 4_000);
}

#[test]
fn framed_manual_control_stores_plane_axis_map() {
    let mut gcs = GcsMavlink::new();
    // y=1000 → roll 2000; x=0 → pitch 1500 (reversed around mid);
    // z=500 → throttle 1500; r=-1000 → rudder 1000.
    let pkt = ManualControl::new(1, 0, 1_000, 500, -1_000, 0);
    let frame = manual_frame(&pkt, 255);
    match gcs.handle_message(&frame, 5_000) {
        Dispatch::ManualControl { applied } => assert_eq!(applied, 4),
        other => panic!("unexpected {other:?}"),
    }
    assert_eq!(gcs.override_channel(0), Some(2_000));
    assert_eq!(gcs.override_channel(1), Some(1_500));
    assert_eq!(gcs.override_channel(2), Some(1_500));
    assert_eq!(gcs.override_channel(3), Some(1_000));
    assert_eq!(gcs.last_gcs_heartbeat_ms(), 5_000);
}

#[test]
fn from_frame_rejects_other_msgid() {
    let frame = Frame::new(0, 1, 1, 0, &[0u8; 9]).expect("heartbeat-shaped");
    assert!(RcChannelsOverride::from_frame(&frame).is_none());
    assert!(ManualControl::from_frame(&frame).is_none());
}

#[test]
fn override_not_from_gcs_is_unknown() {
    let mut gcs = GcsMavlink::new();
    let frame = override_frame(&sample_override(), 42);
    assert_eq!(
        gcs.handle_message(&frame, 0),
        Dispatch::Unknown {
            msgid: MSG_ID_RC_CHANNELS_OVERRIDE
        }
    );
    assert_eq!(gcs.override_channel(0), None);
}

#[test]
fn manual_not_addressed_is_unknown() {
    let mut gcs = GcsMavlink::new();
    let pkt = ManualControl::new(9, 0, 0, 0, 0, 0);
    let frame = manual_frame(&pkt, 255);
    assert_eq!(
        gcs.handle_message(&frame, 0),
        Dispatch::Unknown {
            msgid: MSG_ID_MANUAL_CONTROL
        }
    );
}
