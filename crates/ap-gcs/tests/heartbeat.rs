//! HEARTBEAT encode/decode, MAVLink 2 framing, and msgid dispatch.

use ap_gcs::{
    decode_v2, encode_v2, DecodeError, Dispatch, Frame, GcsMavlink, Heartbeat,
    MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_TYPE_FIXED_WING, MSG_ID_HEARTBEAT,
};

fn plane_heartbeat() -> Heartbeat {
    Heartbeat::plane(MAV_TYPE_FIXED_WING, 0x81, 12, 4)
}

#[test]
fn heartbeat_payload_roundtrip() {
    let hb = plane_heartbeat();
    let mut buf = [0u8; 9];
    assert_eq!(hb.encode(&mut buf), Some(9));
    let decoded = Heartbeat::decode(&buf).expect("payload");
    assert_eq!(decoded.custom_mode, 12);
    assert_eq!(decoded.mav_type, MAV_TYPE_FIXED_WING);
    assert_eq!(decoded.autopilot, MAV_AUTOPILOT_ARDUPILOTMEGA);
    assert_eq!(decoded.base_mode, 0x81);
    assert_eq!(decoded.system_status, 4);
    assert_eq!(decoded.mavlink_version, 3);
}

#[test]
fn mavlink2_frame_roundtrip_and_crc() {
    let hb = plane_heartbeat();
    let mut payload = [0u8; 9];
    assert_eq!(hb.encode(&mut payload), Some(9));
    let frame = Frame::new(7, 1, 1, MSG_ID_HEARTBEAT, &payload).expect("frame");
    let mut wire = [0u8; 32];
    let n = encode_v2(&frame, &mut wire).expect("encode");
    assert_eq!(n, 21);
    assert_eq!(wire.first().copied(), Some(0xFD));
    let parsed = decode_v2(wire.get(..n).expect("slice")).expect("decode");
    assert_eq!(parsed.seq, 7);
    assert_eq!(parsed.sysid, 1);
    assert_eq!(parsed.compid, 1);
    assert_eq!(parsed.msgid, MSG_ID_HEARTBEAT);
    assert_eq!(Heartbeat::from_frame(&parsed), Some(hb));
}

#[test]
fn mavlink2_rejects_bad_crc() {
    let hb = plane_heartbeat();
    let mut payload = [0u8; 9];
    assert_eq!(hb.encode(&mut payload), Some(9));
    let frame = Frame::new(0, 1, 1, MSG_ID_HEARTBEAT, &payload).expect("frame");
    let mut wire = [0u8; 32];
    let n = encode_v2(&frame, &mut wire).expect("encode");
    let crc_lo = wire.get_mut(n - 2).expect("crc");
    *crc_lo ^= 0x01;
    assert_eq!(
        decode_v2(wire.get(..n).expect("slice")),
        Err(DecodeError::BadCrc)
    );
}

#[test]
fn send_heartbeat_dispatches_as_msgid_zero() {
    let mut gcs = GcsMavlink::new();
    let mut wire = [0u8; 32];
    let n = gcs
        .send_heartbeat(&mut wire, MAV_TYPE_FIXED_WING, 0x81, 12, 4)
        .expect("send");
    let frame = decode_v2(wire.get(..n).expect("slice")).expect("decode");
    assert_eq!(frame.msgid, MSG_ID_HEARTBEAT);

    // A peer vehicle heartbeat does not refresh the GCS-seen timer.
    let mut peer = GcsMavlink::new();
    match peer.handle_message(&frame, 1_000) {
        Dispatch::Heartbeat { heartbeat, from_gcs } => {
            assert!(!from_gcs);
            assert_eq!(heartbeat.custom_mode, 12);
            assert_eq!(heartbeat.autopilot, MAV_AUTOPILOT_ARDUPILOTMEGA);
        }
        other => panic!("expected HEARTBEAT, got {other:?}"),
    }
    assert_eq!(peer.last_gcs_heartbeat_ms(), 0);
}

#[test]
fn handle_heartbeat_from_gcs_sysid_records_seen() {
    let hb = plane_heartbeat();
    let mut payload = [0u8; 9];
    assert_eq!(hb.encode(&mut payload), Some(9));
    // Ground station sysid 255, matching MAV_GCS_SYSID default.
    let frame = Frame::new(1, 255, 190, MSG_ID_HEARTBEAT, &payload).expect("frame");
    let mut gcs = GcsMavlink::new();
    match gcs.handle_message(&frame, 42) {
        Dispatch::Heartbeat { from_gcs, .. } => assert!(from_gcs),
        other => panic!("expected HEARTBEAT, got {other:?}"),
    }
    assert_eq!(gcs.last_gcs_heartbeat_ms(), 42);
}

#[test]
fn unknown_msgid_is_not_dispatched() {
    let frame = Frame::new(0, 255, 190, 253, &[]).expect("STATUSTEXT msgid stub");
    let mut gcs = GcsMavlink::new();
    assert_eq!(
        gcs.handle_message(&frame, 0),
        Dispatch::Unknown { msgid: 253 }
    );
    assert_eq!(gcs.last_gcs_heartbeat_ms(), 0);
}
