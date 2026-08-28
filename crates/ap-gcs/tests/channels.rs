//! RC_CHANNELS / SERVO_OUTPUT_RAW stream send from a channel snapshot.

use ap_gcs::{
    decode_v2, encode_v2, ChannelSnapshot, Frame, GcsMavlink, RcChannels, ServoOutputRaw,
    MSG_ID_RC_CHANNELS, MSG_ID_SERVO_OUTPUT_RAW, RC_CHANNELS_COUNT, RC_CHANNELS_CRC,
    RC_CHANNELS_LEN, RC_CHANNEL_UNUSED, SERVO_OUTPUT_COUNT, SERVO_OUTPUT_RAW_CRC,
    SERVO_OUTPUT_RAW_LEN,
};

fn sample_channels() -> ChannelSnapshot {
    let mut chan = [RC_CHANNEL_UNUSED; RC_CHANNELS_COUNT];
    if let Some(ch) = chan.get_mut(0) {
        *ch = 1_500;
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
    let mut servo = [0u16; SERVO_OUTPUT_COUNT];
    if let Some(s) = servo.get_mut(0) {
        *s = 1_510;
    }
    if let Some(s) = servo.get_mut(1) {
        *s = 1_490;
    }
    if let Some(s) = servo.get_mut(2) {
        *s = 1_200;
    }
    if let Some(s) = servo.get_mut(8) {
        *s = 1_400;
    }
    ChannelSnapshot {
        time_boot_ms: 12_345,
        time_usec: 12_345_000,
        chancount: 16,
        chan,
        rssi: 180,
        port: 0,
        servo,
    }
}

#[test]
fn rc_channels_payload_roundtrip() {
    let snap = sample_channels();
    let rc = snap.rc_channels();
    let mut buf = [0u8; RC_CHANNELS_LEN];
    assert_eq!(rc.encode(&mut buf), Some(RC_CHANNELS_LEN));
    let decoded = RcChannels::decode(&buf).expect("payload");
    assert_eq!(decoded, rc);
    assert_eq!(decoded.time_boot_ms, 12_345);
    assert_eq!(decoded.chancount, 16);
    assert_eq!(decoded.rssi, 180);
    assert_eq!(decoded.chan.get(0).copied(), Some(1_500));
    assert_eq!(decoded.chan.get(1).copied(), Some(1_480));
    assert_eq!(decoded.chan.get(17).copied(), Some(RC_CHANNEL_UNUSED));
}

#[test]
fn servo_output_raw_payload_roundtrip() {
    let snap = sample_channels();
    let servo = snap.servo_output_raw();
    let mut buf = [0u8; SERVO_OUTPUT_RAW_LEN];
    assert_eq!(servo.encode(&mut buf), Some(SERVO_OUTPUT_RAW_LEN));
    let decoded = ServoOutputRaw::decode(&buf).expect("payload");
    assert_eq!(decoded, servo);
    assert_eq!(decoded.time_usec, 12_345_000);
    assert_eq!(decoded.port, 0);
    assert_eq!(decoded.servo.get(0).copied(), Some(1_510));
    assert_eq!(decoded.servo.get(2).copied(), Some(1_200));
    assert_eq!(decoded.servo.get(8).copied(), Some(1_400));
}

#[test]
fn servo_output_raw_decode_accepts_min_len() {
    let snap = sample_channels();
    let servo = snap.servo_output_raw();
    let mut buf = [0u8; SERVO_OUTPUT_RAW_LEN];
    assert_eq!(servo.encode(&mut buf), Some(SERVO_OUTPUT_RAW_LEN));
    let decoded = ServoOutputRaw::decode(buf.get(..21).expect("min")).expect("min payload");
    assert_eq!(decoded.time_usec, servo.time_usec);
    assert_eq!(decoded.port, servo.port);
    assert_eq!(decoded.servo.get(0).copied(), Some(1_510));
    assert_eq!(decoded.servo.get(8).copied(), Some(0));
}

#[test]
fn channel_stream_frames_use_pinned_crc_extras() {
    assert_eq!(RC_CHANNELS_CRC, 118);
    assert_eq!(SERVO_OUTPUT_RAW_CRC, 222);

    let snap = sample_channels();
    let mut rc_payload = [0u8; RC_CHANNELS_LEN];
    assert_eq!(
        snap.rc_channels().encode(&mut rc_payload),
        Some(RC_CHANNELS_LEN)
    );
    let rc_frame = Frame::new(3, 1, 1, MSG_ID_RC_CHANNELS, &rc_payload).expect("rc frame");
    let mut rc_wire = [0u8; 64];
    let rn = encode_v2(&rc_frame, &mut rc_wire).expect("encode rc");
    assert_eq!(rn, 10 + RC_CHANNELS_LEN + 2);
    let rc_parsed = decode_v2(rc_wire.get(..rn).expect("slice")).expect("decode rc");
    assert_eq!(rc_parsed.msgid, MSG_ID_RC_CHANNELS);

    let mut servo_payload = [0u8; SERVO_OUTPUT_RAW_LEN];
    assert_eq!(
        snap.servo_output_raw().encode(&mut servo_payload),
        Some(SERVO_OUTPUT_RAW_LEN)
    );
    let servo_frame =
        Frame::new(4, 1, 1, MSG_ID_SERVO_OUTPUT_RAW, &servo_payload).expect("servo frame");
    let mut servo_wire = [0u8; 64];
    let sn = encode_v2(&servo_frame, &mut servo_wire).expect("encode servo");
    assert_eq!(sn, 10 + SERVO_OUTPUT_RAW_LEN + 2);
    let servo_parsed = decode_v2(servo_wire.get(..sn).expect("slice")).expect("decode servo");
    assert_eq!(servo_parsed.msgid, MSG_ID_SERVO_OUTPUT_RAW);
}

#[test]
fn send_rc_channels_and_servo_output_raw_from_channel_snapshot() {
    let mut gcs = GcsMavlink::new();
    let snap = sample_channels();

    let mut rc_wire = [0u8; 64];
    let rn = gcs.send_rc_channels(&mut rc_wire, &snap).expect("send rc");
    assert_eq!(rn, 10 + RC_CHANNELS_LEN + 2);
    assert_eq!(rc_wire.first().copied(), Some(0xFD));
    let rc_frame = decode_v2(rc_wire.get(..rn).expect("slice")).expect("decode rc");
    assert_eq!(rc_frame.msgid, MSG_ID_RC_CHANNELS);
    assert_eq!(rc_frame.sysid, 1);
    assert_eq!(rc_frame.compid, 1);
    assert_eq!(rc_frame.seq, 0);
    let rc = RcChannels::from_frame(&rc_frame).expect("rc");
    assert_eq!(rc, snap.rc_channels());

    let mut servo_wire = [0u8; 64];
    let sn = gcs
        .send_servo_output_raw(&mut servo_wire, &snap)
        .expect("send servo");
    assert_eq!(sn, 10 + SERVO_OUTPUT_RAW_LEN + 2);
    assert_eq!(servo_wire.first().copied(), Some(0xFD));
    let servo_frame = decode_v2(servo_wire.get(..sn).expect("slice")).expect("decode servo");
    assert_eq!(servo_frame.msgid, MSG_ID_SERVO_OUTPUT_RAW);
    assert_eq!(servo_frame.sysid, 1);
    assert_eq!(servo_frame.compid, 1);
    assert_eq!(servo_frame.seq, 1);
    let servo = ServoOutputRaw::from_frame(&servo_frame).expect("servo");
    assert_eq!(servo, snap.servo_output_raw());
}

#[test]
fn from_frame_rejects_other_msgid() {
    let frame = Frame::new(0, 1, 1, 0, &[0u8; 9]).expect("heartbeat-shaped");
    assert!(RcChannels::from_frame(&frame).is_none());
    assert!(ServoOutputRaw::from_frame(&frame).is_none());
}
