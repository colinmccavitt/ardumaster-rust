//! ATTITUDE / GLOBAL_POSITION_INT stream send from a pose snapshot.

use ap_gcs::{
    decode_v2, encode_v2, Attitude, Frame, GcsMavlink, GlobalPositionInt, PoseSnapshot,
    ATTITUDE_CRC, ATTITUDE_LEN, GLOBAL_POSITION_INT_CRC, GLOBAL_POSITION_INT_LEN, MSG_ID_ATTITUDE,
    MSG_ID_GLOBAL_POSITION_INT,
};

fn sample_pose() -> PoseSnapshot {
    PoseSnapshot {
        time_boot_ms: 12_345,
        roll: 0.1,
        pitch: -0.2,
        yaw: 1.57,
        rollspeed: 0.01,
        pitchspeed: -0.02,
        yawspeed: 0.03,
        lat: 377_749_000,
        lon: -1_224_191_000,
        alt: 12_000,
        relative_alt: 3_500,
        vx: 150,
        vy: -40,
        vz: 10,
        hdg: 27_000,
    }
}

#[test]
fn attitude_payload_roundtrip() {
    let pose = sample_pose();
    let att = pose.attitude();
    let mut buf = [0u8; ATTITUDE_LEN];
    assert_eq!(att.encode(&mut buf), Some(ATTITUDE_LEN));
    let decoded = Attitude::decode(&buf).expect("payload");
    assert_eq!(decoded.time_boot_ms, 12_345);
    assert_eq!(decoded.roll.to_le_bytes(), 0.1f32.to_le_bytes());
    assert_eq!(decoded.pitch.to_le_bytes(), (-0.2f32).to_le_bytes());
    assert_eq!(decoded.yaw.to_le_bytes(), 1.57f32.to_le_bytes());
    assert_eq!(decoded.rollspeed.to_le_bytes(), 0.01f32.to_le_bytes());
    assert_eq!(decoded.pitchspeed.to_le_bytes(), (-0.02f32).to_le_bytes());
    assert_eq!(decoded.yawspeed.to_le_bytes(), 0.03f32.to_le_bytes());
}

#[test]
fn global_position_int_payload_roundtrip() {
    let pose = sample_pose();
    let gpi = pose.global_position_int();
    let mut buf = [0u8; GLOBAL_POSITION_INT_LEN];
    assert_eq!(gpi.encode(&mut buf), Some(GLOBAL_POSITION_INT_LEN));
    let decoded = GlobalPositionInt::decode(&buf).expect("payload");
    assert_eq!(decoded, gpi);
    assert_eq!(decoded.lat, 377_749_000);
    assert_eq!(decoded.lon, -1_224_191_000);
    assert_eq!(decoded.alt, 12_000);
    assert_eq!(decoded.relative_alt, 3_500);
    assert_eq!(decoded.vx, 150);
    assert_eq!(decoded.vy, -40);
    assert_eq!(decoded.vz, 10);
    assert_eq!(decoded.hdg, 27_000);
}

#[test]
fn pose_stream_frames_use_pinned_crc_extras() {
    assert_eq!(ATTITUDE_CRC, 39);
    assert_eq!(GLOBAL_POSITION_INT_CRC, 104);

    let pose = sample_pose();
    let mut att_payload = [0u8; ATTITUDE_LEN];
    assert_eq!(pose.attitude().encode(&mut att_payload), Some(ATTITUDE_LEN));
    let att_frame = Frame::new(3, 1, 1, MSG_ID_ATTITUDE, &att_payload).expect("att frame");
    let mut att_wire = [0u8; 48];
    let an = encode_v2(&att_frame, &mut att_wire).expect("encode att");
    assert_eq!(an, 10 + ATTITUDE_LEN + 2);
    let att_parsed = decode_v2(att_wire.get(..an).expect("slice")).expect("decode att");
    assert_eq!(att_parsed.msgid, MSG_ID_ATTITUDE);

    let mut gpi_payload = [0u8; GLOBAL_POSITION_INT_LEN];
    assert_eq!(
        pose.global_position_int().encode(&mut gpi_payload),
        Some(GLOBAL_POSITION_INT_LEN)
    );
    let gpi_frame =
        Frame::new(4, 1, 1, MSG_ID_GLOBAL_POSITION_INT, &gpi_payload).expect("gpi frame");
    let mut gpi_wire = [0u8; 48];
    let gn = encode_v2(&gpi_frame, &mut gpi_wire).expect("encode gpi");
    assert_eq!(gn, 10 + GLOBAL_POSITION_INT_LEN + 2);
    let gpi_parsed = decode_v2(gpi_wire.get(..gn).expect("slice")).expect("decode gpi");
    assert_eq!(gpi_parsed.msgid, MSG_ID_GLOBAL_POSITION_INT);
}

#[test]
fn send_attitude_and_global_position_int_from_pose_snapshot() {
    let mut gcs = GcsMavlink::new();
    let pose = sample_pose();

    let mut att_wire = [0u8; 48];
    let an = gcs.send_attitude(&mut att_wire, &pose).expect("send att");
    assert_eq!(an, 10 + ATTITUDE_LEN + 2);
    assert_eq!(att_wire.first().copied(), Some(0xFD));
    let att_frame = decode_v2(att_wire.get(..an).expect("slice")).expect("decode att");
    assert_eq!(att_frame.msgid, MSG_ID_ATTITUDE);
    assert_eq!(att_frame.sysid, 1);
    assert_eq!(att_frame.compid, 1);
    assert_eq!(att_frame.seq, 0);
    let att = Attitude::from_frame(&att_frame).expect("attitude");
    assert_eq!(att.time_boot_ms, pose.time_boot_ms);
    assert_eq!(att.roll.to_le_bytes(), pose.roll.to_le_bytes());
    assert_eq!(att.yaw.to_le_bytes(), pose.yaw.to_le_bytes());

    let mut gpi_wire = [0u8; 48];
    let gn = gcs
        .send_global_position_int(&mut gpi_wire, &pose)
        .expect("send gpi");
    assert_eq!(gn, 10 + GLOBAL_POSITION_INT_LEN + 2);
    assert_eq!(gpi_wire.first().copied(), Some(0xFD));
    let gpi_frame = decode_v2(gpi_wire.get(..gn).expect("slice")).expect("decode gpi");
    assert_eq!(gpi_frame.msgid, MSG_ID_GLOBAL_POSITION_INT);
    assert_eq!(gpi_frame.sysid, 1);
    assert_eq!(gpi_frame.compid, 1);
    assert_eq!(gpi_frame.seq, 1);
    let gpi = GlobalPositionInt::from_frame(&gpi_frame).expect("gpi");
    assert_eq!(gpi, pose.global_position_int());
}

#[test]
fn from_frame_rejects_other_msgid() {
    let frame = Frame::new(0, 1, 1, 0, &[0u8; 9]).expect("heartbeat-shaped");
    assert!(Attitude::from_frame(&frame).is_none());
    assert!(GlobalPositionInt::from_frame(&frame).is_none());
}
