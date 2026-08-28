//! PARAM_REQUEST_LIST / PARAM_SET list-then-set protocol stub.

use ap_gcs::{
    decode_v2, encode_v2, Dispatch, Frame, GcsMavlink, ParamRequestList, ParamSet, ParamValue,
    MAV_PARAM_TYPE_REAL32, MSG_ID_PARAM_REQUEST_LIST, MSG_ID_PARAM_SET, MSG_ID_PARAM_VALUE,
    PARAM_REQUEST_LIST_LEN, PARAM_SET_LEN, PARAM_VALUE_LEN,
};

fn list_frame(target_system: u8) -> Frame {
    let req = ParamRequestList::new(target_system, 1);
    let mut payload = [0u8; PARAM_REQUEST_LIST_LEN];
    assert_eq!(req.encode(&mut payload), Some(PARAM_REQUEST_LIST_LEN));
    Frame::new(5, 255, 190, MSG_ID_PARAM_REQUEST_LIST, &payload).expect("frame")
}

fn set_frame(target_system: u8, name: &str, value: f32) -> Frame {
    let set = ParamSet::new(target_system, 1, name, value, MAV_PARAM_TYPE_REAL32);
    let mut payload = [0u8; PARAM_SET_LEN];
    assert_eq!(set.encode(&mut payload), Some(PARAM_SET_LEN));
    Frame::new(6, 255, 190, MSG_ID_PARAM_SET, &payload).expect("frame")
}

#[test]
fn param_request_list_payload_roundtrip() {
    let req = ParamRequestList::new(1, 1);
    let mut buf = [0u8; PARAM_REQUEST_LIST_LEN];
    assert_eq!(req.encode(&mut buf), Some(PARAM_REQUEST_LIST_LEN));
    let decoded = ParamRequestList::decode(&buf).expect("payload");
    assert_eq!(decoded.target_system, 1);
    assert_eq!(decoded.target_component, 1);
}

#[test]
fn param_set_and_value_payload_roundtrip() {
    let set = ParamSet::new(1, 1, "ARSPD_ENABLE", 1.0, MAV_PARAM_TYPE_REAL32);
    let mut buf = [0u8; PARAM_SET_LEN];
    assert_eq!(set.encode(&mut buf), Some(PARAM_SET_LEN));
    let decoded = ParamSet::decode(&buf).expect("payload");
    assert_eq!(decoded.target_system, 1);
    assert_eq!(decoded.target_component, 1);
    assert_eq!(decoded.param_type, MAV_PARAM_TYPE_REAL32);
    assert_eq!(decoded.name(), Some("ARSPD_ENABLE"));
    assert_eq!(decoded.param_value.to_le_bytes(), 1.0f32.to_le_bytes());

    let value = ParamValue::new("ARSPD_ENABLE", 1.0, MAV_PARAM_TYPE_REAL32, 3, 1);
    let mut vbuf = [0u8; PARAM_VALUE_LEN];
    assert_eq!(value.encode(&mut vbuf), Some(PARAM_VALUE_LEN));
    let again = ParamValue::decode(&vbuf).expect("value");
    assert_eq!(again.name(), Some("ARSPD_ENABLE"));
    assert_eq!(again.param_count, 3);
    assert_eq!(again.param_index, 1);
    assert_eq!(again.param_type, MAV_PARAM_TYPE_REAL32);
    assert_eq!(again.param_value.to_le_bytes(), 1.0f32.to_le_bytes());
}

#[test]
fn param_frames_use_pinned_crc_extras() {
    let list = list_frame(1);
    let mut wire = [0u8; 32];
    let n = encode_v2(&list, &mut wire).expect("encode list");
    assert_eq!(n, 10 + PARAM_REQUEST_LIST_LEN + 2);
    let parsed = decode_v2(wire.get(..n).expect("slice")).expect("decode");
    assert_eq!(parsed.msgid, MSG_ID_PARAM_REQUEST_LIST);

    let set = set_frame(1, "ARSPD_ENABLE", 1.0);
    let mut set_wire = [0u8; 48];
    let sn = encode_v2(&set, &mut set_wire).expect("encode set");
    assert_eq!(sn, 10 + PARAM_SET_LEN + 2);
    let set_parsed = decode_v2(set_wire.get(..sn).expect("slice")).expect("decode");
    assert_eq!(set_parsed.msgid, MSG_ID_PARAM_SET);
    assert_eq!(
        ParamSet::from_frame(&set_parsed).expect("set").name(),
        Some("ARSPD_ENABLE")
    );
}

#[test]
fn list_then_set_named_param_in_table() {
    let mut gcs = GcsMavlink::new();
    assert_eq!(gcs.param_count(), 3);

    match gcs.handle_message(&list_frame(1), 0) {
        Dispatch::ParamRequestList { count } => assert_eq!(count, 3),
        other => panic!("expected ParamRequestList, got {other:?}"),
    }

    let expected = ["SYSID_THISMAV", "ARSPD_ENABLE", "TRIM_PITCH"];
    let mut i = 0usize;
    loop {
        let mut out = [0u8; 48];
        let Some(n) = gcs.queued_param_send(&mut out) else {
            break;
        };
        let frame = decode_v2(out.get(..n).expect("wire")).expect("frame");
        assert_eq!(frame.msgid, MSG_ID_PARAM_VALUE);
        let value = ParamValue::from_frame(&frame).expect("value");
        assert_eq!(value.param_count, 3);
        assert_eq!(value.param_index as usize, i);
        assert_eq!(value.param_type, MAV_PARAM_TYPE_REAL32);
        assert_eq!(value.name(), expected.get(i).copied());
        i += 1;
    }
    assert_eq!(i, 3);
    assert_eq!(
        gcs.param_value("ARSPD_ENABLE").map(f32::to_le_bytes),
        Some(0.0f32.to_le_bytes())
    );

    match gcs.handle_message(&set_frame(1, "ARSPD_ENABLE", 1.0), 0) {
        Dispatch::ParamSet { applied } => assert!(applied),
        other => panic!("expected ParamSet, got {other:?}"),
    }
    assert_eq!(
        gcs.param_value("ARSPD_ENABLE").map(f32::to_le_bytes),
        Some(1.0f32.to_le_bytes())
    );

    let mut ack = [0u8; 48];
    let n = gcs
        .send_parameter_value(&mut ack, "ARSPD_ENABLE")
        .expect("ack");
    let ack_frame = decode_v2(ack.get(..n).expect("wire")).expect("frame");
    let ack_value = ParamValue::from_frame(&ack_frame).expect("value");
    assert_eq!(ack_value.name(), Some("ARSPD_ENABLE"));
    assert_eq!(ack_value.param_index, 1);
    assert_eq!(ack_value.param_count, 3);
    assert_eq!(ack_value.param_value.to_le_bytes(), 1.0f32.to_le_bytes());
}

#[test]
fn param_set_unknown_name_is_not_applied() {
    let mut gcs = GcsMavlink::new();
    match gcs.handle_message(&set_frame(1, "NO_SUCH_PARAM", 4.0), 0) {
        Dispatch::ParamSet { applied } => assert!(!applied),
        other => panic!("expected ParamSet, got {other:?}"),
    }
}

#[test]
fn param_not_addressed_to_us_is_unknown() {
    let mut gcs = GcsMavlink::new();
    assert_eq!(
        gcs.handle_message(&list_frame(9), 0),
        Dispatch::Unknown {
            msgid: MSG_ID_PARAM_REQUEST_LIST
        }
    );
    assert_eq!(
        gcs.handle_message(&set_frame(9, "ARSPD_ENABLE", 1.0), 0),
        Dispatch::Unknown {
            msgid: MSG_ID_PARAM_SET
        }
    );
}

#[test]
fn from_frame_rejects_other_msgid() {
    let frame = Frame::new(0, 1, 1, 0, &[0u8; 9]).expect("heartbeat-shaped");
    assert!(ParamRequestList::from_frame(&frame).is_none());
    assert!(ParamSet::from_frame(&frame).is_none());
    assert!(ParamValue::from_frame(&frame).is_none());
}
