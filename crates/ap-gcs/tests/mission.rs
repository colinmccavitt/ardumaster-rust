//! MISSION_ITEM_INT / MISSION_REQUEST_INT upload-then-download stub.

use ap_gcs::{
    decode_v2, encode_v2, Dispatch, Frame, GcsMavlink, MissionItemInt, MissionRequestInt,
    MAV_CMD_NAV_WAYPOINT, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_MISSION_TYPE_MISSION,
    MISSION_ITEM_INT_CRC, MISSION_ITEM_INT_LEN, MISSION_REQUEST_INT_CRC, MISSION_REQUEST_INT_LEN,
    MSG_ID_MISSION_ITEM_INT, MSG_ID_MISSION_REQUEST_INT,
};

fn sample_waypoint(target_system: u8, seq: u16) -> MissionItemInt {
    MissionItemInt::waypoint(target_system, 1, seq, 377_749_000, -1_224_191_000, 120.0)
}

fn item_frame(target_system: u8, seq: u16) -> Frame {
    let item = sample_waypoint(target_system, seq);
    let mut payload = [0u8; MISSION_ITEM_INT_LEN];
    assert_eq!(item.encode(&mut payload), Some(MISSION_ITEM_INT_LEN));
    Frame::new(5, 255, 190, MSG_ID_MISSION_ITEM_INT, &payload).expect("frame")
}

fn request_frame(target_system: u8, seq: u16) -> Frame {
    let req = MissionRequestInt::new(target_system, 1, seq, MAV_MISSION_TYPE_MISSION);
    let mut payload = [0u8; MISSION_REQUEST_INT_LEN];
    assert_eq!(req.encode(&mut payload), Some(MISSION_REQUEST_INT_LEN));
    Frame::new(6, 255, 190, MSG_ID_MISSION_REQUEST_INT, &payload).expect("frame")
}

#[test]
fn mission_request_int_payload_roundtrip() {
    let req = MissionRequestInt::new(1, 1, 3, MAV_MISSION_TYPE_MISSION);
    let mut buf = [0u8; MISSION_REQUEST_INT_LEN];
    assert_eq!(req.encode(&mut buf), Some(MISSION_REQUEST_INT_LEN));
    assert_eq!(buf.get(..2), Some([3, 0].as_slice()));
    let decoded = MissionRequestInt::decode(&buf).expect("payload");
    assert_eq!(decoded, req);
}

#[test]
fn mission_item_int_payload_roundtrip() {
    let item = sample_waypoint(1, 1);
    let mut buf = [0u8; MISSION_ITEM_INT_LEN];
    assert_eq!(item.encode(&mut buf), Some(MISSION_ITEM_INT_LEN));
    let decoded = MissionItemInt::decode(&buf).expect("payload");
    assert_eq!(decoded.seq, 1);
    assert_eq!(decoded.command, MAV_CMD_NAV_WAYPOINT);
    assert_eq!(decoded.frame, MAV_FRAME_GLOBAL_RELATIVE_ALT);
    assert_eq!(decoded.mission_type, MAV_MISSION_TYPE_MISSION);
    assert_eq!(decoded.target_system, 1);
    assert_eq!(decoded.target_component, 1);
    assert_eq!(decoded.x, 377_749_000);
    assert_eq!(decoded.y, -1_224_191_000);
    assert_eq!(decoded.z.to_le_bytes(), 120.0f32.to_le_bytes());
    assert_eq!(decoded.autocontinue, 1);
}

#[test]
fn mission_frames_use_pinned_crc_extras() {
    assert_eq!(MISSION_REQUEST_INT_CRC, 196);
    assert_eq!(MISSION_ITEM_INT_CRC, 38);

    let req = request_frame(1, 1);
    let mut wire = [0u8; 32];
    let n = encode_v2(&req, &mut wire).expect("encode request");
    assert_eq!(n, 10 + MISSION_REQUEST_INT_LEN + 2);
    let parsed = decode_v2(wire.get(..n).expect("slice")).expect("decode");
    assert_eq!(parsed.msgid, MSG_ID_MISSION_REQUEST_INT);

    let item = item_frame(1, 1);
    let mut item_wire = [0u8; 64];
    let in_ = encode_v2(&item, &mut item_wire).expect("encode item");
    assert_eq!(in_, 10 + MISSION_ITEM_INT_LEN + 2);
    let item_parsed = decode_v2(item_wire.get(..in_).expect("slice")).expect("decode");
    assert_eq!(item_parsed.msgid, MSG_ID_MISSION_ITEM_INT);
    let decoded = MissionItemInt::from_frame(&item_parsed).expect("item");
    assert_eq!(decoded.seq, 1);
    assert_eq!(decoded.command, MAV_CMD_NAV_WAYPOINT);
}

#[test]
fn upload_then_download_one_waypoint() {
    let mut gcs = GcsMavlink::new();
    assert_eq!(gcs.mission_count(), 0);

    match gcs.handle_message(&item_frame(1, 1), 0) {
        Dispatch::MissionItemInt { seq, stored } => {
            assert_eq!(seq, 1);
            assert!(stored);
        }
        other => panic!("expected MissionItemInt, got {other:?}"),
    }
    assert_eq!(gcs.mission_count(), 1);
    let stored = gcs.mission_item(1).expect("stored");
    assert_eq!(stored.command, MAV_CMD_NAV_WAYPOINT);
    assert_eq!(stored.x, 377_749_000);
    assert_eq!(stored.y, -1_224_191_000);
    assert_eq!(stored.z.to_le_bytes(), 120.0f32.to_le_bytes());

    match gcs.handle_message(&request_frame(1, 1), 0) {
        Dispatch::MissionRequestInt { seq, found } => {
            assert_eq!(seq, 1);
            assert!(found);
        }
        other => panic!("expected MissionRequestInt, got {other:?}"),
    }

    let mut out = [0u8; 64];
    let n = gcs.send_mission_item_int(&mut out, 1).expect("send");
    assert_eq!(n, 10 + MISSION_ITEM_INT_LEN + 2);
    assert_eq!(out.first().copied(), Some(0xFD));
    let frame = decode_v2(out.get(..n).expect("wire")).expect("frame");
    assert_eq!(frame.msgid, MSG_ID_MISSION_ITEM_INT);
    assert_eq!(frame.sysid, 1);
    assert_eq!(frame.compid, 1);
    let downloaded = MissionItemInt::from_frame(&frame).expect("item");
    assert_eq!(downloaded.seq, 1);
    assert_eq!(downloaded.command, MAV_CMD_NAV_WAYPOINT);
    assert_eq!(downloaded.frame, MAV_FRAME_GLOBAL_RELATIVE_ALT);
    assert_eq!(downloaded.x, 377_749_000);
    assert_eq!(downloaded.y, -1_224_191_000);
    assert_eq!(downloaded.z.to_le_bytes(), 120.0f32.to_le_bytes());
    assert_eq!(downloaded.autocontinue, 1);
}

#[test]
fn mission_request_missing_seq_is_not_found() {
    let mut gcs = GcsMavlink::new();
    match gcs.handle_message(&request_frame(1, 1), 0) {
        Dispatch::MissionRequestInt { seq, found } => {
            assert_eq!(seq, 1);
            assert!(!found);
        }
        other => panic!("expected MissionRequestInt, got {other:?}"),
    }
    assert!(gcs.send_mission_item_int(&mut [0u8; 64], 1).is_none());
}

#[test]
fn mission_not_addressed_to_us_is_unknown() {
    let mut gcs = GcsMavlink::new();
    assert_eq!(
        gcs.handle_message(&item_frame(9, 1), 0),
        Dispatch::Unknown {
            msgid: MSG_ID_MISSION_ITEM_INT
        }
    );
    assert_eq!(
        gcs.handle_message(&request_frame(9, 1), 0),
        Dispatch::Unknown {
            msgid: MSG_ID_MISSION_REQUEST_INT
        }
    );
    assert_eq!(gcs.mission_count(), 0);
}

#[test]
fn from_frame_rejects_other_msgid() {
    let frame = Frame::new(0, 1, 1, 0, &[0u8; 9]).expect("heartbeat-shaped");
    assert!(MissionRequestInt::from_frame(&frame).is_none());
    assert!(MissionItemInt::from_frame(&frame).is_none());
}
