//! REQUEST_DATA_STREAM / SET_MESSAGE_INTERVAL rate-table stub.

use ap_gcs::{
    decode_v2, encode_v2, CommandLong, Dispatch, Frame, GcsMavlink, HudSnapshot, RequestDataStream,
    MAV_CMD_SET_MESSAGE_INTERVAL, MAV_DATA_STREAM_EXTRA1, MAV_DATA_STREAM_EXTRA2, MSG_ID_ATTITUDE,
    MSG_ID_COMMAND_LONG, MSG_ID_REQUEST_DATA_STREAM, MSG_ID_VFR_HUD, REQUEST_DATA_STREAM_CRC,
    REQUEST_DATA_STREAM_LEN, VFR_HUD_LEN,
};

fn sample_hud() -> HudSnapshot {
    HudSnapshot {
        airspeed: 22.5,
        groundspeed: 20.0,
        heading: 270,
        throttle: 65,
        alt: 120.5,
        climb: 1.25,
        nav_roll: 12.0,
        nav_pitch: 3.5,
        nav_bearing: 275,
        target_bearing: 280,
        wp_dist: 450,
        alt_error: -2.0,
        aspd_error: 50.0,
        xtrack_error: 8.5,
    }
}

fn request_frame(req: &RequestDataStream, sysid: u8) -> Frame {
    let mut payload = [0u8; REQUEST_DATA_STREAM_LEN];
    assert_eq!(req.encode(&mut payload), Some(REQUEST_DATA_STREAM_LEN));
    Frame::new(5, sysid, 190, MSG_ID_REQUEST_DATA_STREAM, &payload).expect("frame")
}

fn set_interval_long(msgid: u32, interval_us: f32) -> CommandLong {
    CommandLong::new(
        1,
        1,
        MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        [msgid as f32, interval_us, 0.0, 0.0, 0.0, 0.0, 0.0],
    )
}

#[test]
fn request_data_stream_payload_roundtrip() {
    let req = RequestDataStream::new(1, 1, MAV_DATA_STREAM_EXTRA2, 10, 1);
    let mut buf = [0u8; REQUEST_DATA_STREAM_LEN];
    assert_eq!(req.encode(&mut buf), Some(REQUEST_DATA_STREAM_LEN));
    let decoded = RequestDataStream::decode(&buf).expect("payload");
    assert_eq!(decoded.target_system, 1);
    assert_eq!(decoded.target_component, 1);
    assert_eq!(decoded.req_stream_id, MAV_DATA_STREAM_EXTRA2);
    assert_eq!(decoded.req_message_rate, 10);
    assert_eq!(decoded.start_stop, 1);
}

#[test]
fn request_data_stream_frame_uses_pinned_crc_extra() {
    assert_eq!(REQUEST_DATA_STREAM_CRC, 148);
    let req = RequestDataStream::new(1, 1, MAV_DATA_STREAM_EXTRA1, 4, 1);
    let frame = request_frame(&req, 255);
    let mut wire = [0u8; 32];
    let n = encode_v2(&frame, &mut wire).expect("encode");
    assert_eq!(n, 10 + REQUEST_DATA_STREAM_LEN + 2);
    let parsed = decode_v2(wire.get(..n).expect("slice")).expect("decode");
    assert_eq!(parsed.msgid, MSG_ID_REQUEST_DATA_STREAM);
    let again = RequestDataStream::from_frame(&parsed).expect("from_frame");
    assert_eq!(again.req_stream_id, MAV_DATA_STREAM_EXTRA1);
    assert_eq!(again.req_message_rate, 4);
}

#[test]
fn request_data_stream_sets_stream_msgid_interval() {
    let mut gcs = GcsMavlink::new();
    let req = RequestDataStream::new(1, 1, MAV_DATA_STREAM_EXTRA2, 10, 1);
    let frame = request_frame(&req, 255);
    match gcs.handle_message(&frame, 1_000) {
        Dispatch::RequestDataStream {
            stream_id,
            rate_hz,
            written,
        } => {
            assert_eq!(stream_id, MAV_DATA_STREAM_EXTRA2);
            assert_eq!(rate_hz, 10);
            assert_eq!(written, 1);
        }
        other => panic!("unexpected {other:?}"),
    }
    assert_eq!(gcs.stream_interval_ms(MSG_ID_VFR_HUD), Some(100));
    assert_eq!(gcs.stream_interval_ms(MSG_ID_ATTITUDE), None);
}

#[test]
fn set_message_interval_command_stores_msgid_period() {
    let mut gcs = GcsMavlink::new();
    let cmd = set_interval_long(MSG_ID_ATTITUDE, 200_000.0);
    let mut payload = [0u8; ap_gcs::COMMAND_LONG_LEN];
    assert_eq!(cmd.encode(&mut payload), Some(ap_gcs::COMMAND_LONG_LEN));
    let frame = Frame::new(6, 255, 190, MSG_ID_COMMAND_LONG, &payload).expect("frame");
    match gcs.handle_message(&frame, 2_000) {
        Dispatch::SetMessageInterval {
            msgid,
            interval_ms,
            applied,
        } => {
            assert_eq!(msgid, MSG_ID_ATTITUDE);
            assert_eq!(interval_ms, 200);
            assert!(applied);
        }
        other => panic!("unexpected {other:?}"),
    }
    assert_eq!(gcs.stream_interval_ms(MSG_ID_ATTITUDE), Some(200));
}

#[test]
fn send_skips_until_period_elapses() {
    let mut gcs = GcsMavlink::new();
    let cmd = set_interval_long(MSG_ID_VFR_HUD, 100_000.0);
    let mut payload = [0u8; ap_gcs::COMMAND_LONG_LEN];
    assert_eq!(cmd.encode(&mut payload), Some(ap_gcs::COMMAND_LONG_LEN));
    let frame = Frame::new(7, 255, 190, MSG_ID_COMMAND_LONG, &payload).expect("frame");
    assert!(matches!(
        gcs.handle_message(&frame, 0),
        Dispatch::SetMessageInterval {
            applied: true,
            interval_ms: 100,
            ..
        }
    ));

    let snap = sample_hud();
    let mut wire = [0u8; 48];
    let first = gcs
        .send_vfr_hud_if_due(&mut wire, &snap, 10)
        .expect("first send is due");
    assert_eq!(first, 10 + VFR_HUD_LEN + 2);
    assert_eq!(wire.first().copied(), Some(0xFD));

    assert!(
        gcs.send_vfr_hud_if_due(&mut wire, &snap, 50).is_none(),
        "50 ms is still inside the 100 ms period"
    );
    let again = gcs
        .send_vfr_hud_if_due(&mut wire, &snap, 110)
        .expect("period elapsed");
    assert_eq!(again, 10 + VFR_HUD_LEN + 2);
}

#[test]
fn stop_interval_never_sends() {
    let mut gcs = GcsMavlink::new();
    let cmd = set_interval_long(MSG_ID_VFR_HUD, -1.0);
    let mut payload = [0u8; ap_gcs::COMMAND_LONG_LEN];
    assert_eq!(cmd.encode(&mut payload), Some(ap_gcs::COMMAND_LONG_LEN));
    let frame = Frame::new(8, 255, 190, MSG_ID_COMMAND_LONG, &payload).expect("frame");
    assert!(matches!(
        gcs.handle_message(&frame, 0),
        Dispatch::SetMessageInterval {
            applied: true,
            interval_ms: 0,
            ..
        }
    ));
    let snap = sample_hud();
    let mut wire = [0u8; 48];
    assert!(gcs.send_vfr_hud_if_due(&mut wire, &snap, 1_000).is_none());
}

#[test]
fn from_frame_rejects_other_msgid() {
    let frame = Frame::new(0, 1, 1, 0, &[0u8; 9]).expect("heartbeat-shaped");
    assert!(RequestDataStream::from_frame(&frame).is_none());
}

#[test]
fn request_not_addressed_is_unknown() {
    let mut gcs = GcsMavlink::new();
    let req = RequestDataStream::new(9, 1, MAV_DATA_STREAM_EXTRA2, 10, 1);
    let frame = request_frame(&req, 255);
    assert_eq!(
        gcs.handle_message(&frame, 0),
        Dispatch::Unknown {
            msgid: MSG_ID_REQUEST_DATA_STREAM
        }
    );
}
