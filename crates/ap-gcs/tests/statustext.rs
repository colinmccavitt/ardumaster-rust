//! STATUSTEXT encode and `send_text` framing for Write.

use ap_gcs::{
    decode_v2, encode_v2, Frame, GcsMavlink, StatusText, MAV_SEVERITY_INFO, MAV_SEVERITY_WARNING,
    MSG_ID_STATUSTEXT, STATUSTEXT_LEN, TEXT_LEN,
};

#[test]
fn statustext_payload_roundtrip() {
    let st = StatusText::new(MAV_SEVERITY_WARNING, "PreArm: RC not calibrated");
    let mut buf = [0u8; STATUSTEXT_LEN];
    assert_eq!(st.encode(&mut buf), Some(STATUSTEXT_LEN));
    let decoded = StatusText::decode(&buf).expect("payload");
    assert_eq!(decoded.severity, MAV_SEVERITY_WARNING);
    assert_eq!(decoded.id, 0);
    assert_eq!(decoded.chunk_seq, 0);
    let text = decoded.text();
    assert_eq!(
        text.get(..25),
        Some(b"PreArm: RC not calibrated".as_slice())
    );
    assert!(text.get(25..).expect("tail").iter().all(|&b| b == 0));
}

#[test]
fn send_text_frames_msgid_253_for_write() {
    let mut gcs = GcsMavlink::new();
    let mut wire = [0u8; 80];
    let n = gcs
        .send_text(&mut wire, MAV_SEVERITY_INFO, "ArduPlane V4.7.0")
        .expect("send");
    // STX + 10-byte header + 54-byte payload + 2-byte CRC.
    assert_eq!(n, 10 + STATUSTEXT_LEN + 2);
    assert_eq!(wire.first().copied(), Some(0xFD));

    let frame = decode_v2(wire.get(..n).expect("slice")).expect("decode");
    assert_eq!(frame.msgid, MSG_ID_STATUSTEXT);
    assert_eq!(frame.sysid, 1);
    assert_eq!(frame.compid, 1);
    assert_eq!(usize::from(frame.payload_len()), STATUSTEXT_LEN);

    let st = StatusText::from_frame(&frame).expect("statustext");
    assert_eq!(st.severity, MAV_SEVERITY_INFO);
    assert_eq!(st.text().get(..16), Some(b"ArduPlane V4.7.0".as_slice()));
    assert_eq!(st.id, 0);
    assert_eq!(st.chunk_seq, 0);
}

#[test]
fn send_text_increments_seq_and_survives_encode_v2_roundtrip() {
    let mut gcs = GcsMavlink::new();
    let mut first = [0u8; 80];
    let mut second = [0u8; 80];
    let n1 = gcs
        .send_text(&mut first, MAV_SEVERITY_WARNING, "one")
        .expect("first");
    let n2 = gcs
        .send_text(&mut second, MAV_SEVERITY_WARNING, "two")
        .expect("second");
    let a = decode_v2(first.get(..n1).expect("slice")).expect("a");
    let b = decode_v2(second.get(..n2).expect("slice")).expect("b");
    assert_eq!(a.seq, 0);
    assert_eq!(b.seq, 1);
    assert_eq!(a.msgid, MSG_ID_STATUSTEXT);
    assert_eq!(b.msgid, MSG_ID_STATUSTEXT);

    // Re-encode the parsed frame; CRC extra 83 must stay registered.
    let mut again = [0u8; 80];
    let n = encode_v2(&a, &mut again).expect("re-encode");
    assert_eq!(n, n1);
    assert_eq!(again.get(..n), first.get(..n1));
}

#[test]
fn from_frame_rejects_other_msgid() {
    let frame = Frame::new(0, 1, 1, 0, &[0u8; 9]).expect("heartbeat-shaped");
    assert!(StatusText::from_frame(&frame).is_none());
}

#[test]
fn text_field_is_fifty_bytes() {
    assert_eq!(TEXT_LEN, 50);
}
