//! VFR_HUD / NAV_CONTROLLER_OUTPUT stream send from an air-data + nav snapshot.

use ap_gcs::{
    decode_v2, encode_v2, Frame, GcsMavlink, HudSnapshot, NavControllerOutput, VfrHud,
    MSG_ID_NAV_CONTROLLER_OUTPUT, MSG_ID_VFR_HUD, NAV_CONTROLLER_OUTPUT_CRC,
    NAV_CONTROLLER_OUTPUT_LEN, VFR_HUD_CRC, VFR_HUD_LEN,
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

fn f32_bits_eq(a: f32, b: f32) -> bool {
    a.to_le_bytes() == b.to_le_bytes()
}

#[test]
fn vfr_hud_payload_roundtrip() {
    let snap = sample_hud();
    let hud = snap.vfr_hud();
    let mut buf = [0u8; VFR_HUD_LEN];
    assert_eq!(hud.encode(&mut buf), Some(VFR_HUD_LEN));
    let decoded = VfrHud::decode(&buf).expect("payload");
    assert!(f32_bits_eq(decoded.airspeed, 22.5));
    assert!(f32_bits_eq(decoded.groundspeed, 20.0));
    assert_eq!(decoded.heading, 270);
    assert_eq!(decoded.throttle, 65);
    assert!(f32_bits_eq(decoded.alt, 120.5));
    assert!(f32_bits_eq(decoded.climb, 1.25));
}

#[test]
fn nav_controller_output_payload_roundtrip() {
    let snap = sample_hud();
    let nav = snap.nav_controller_output();
    let mut buf = [0u8; NAV_CONTROLLER_OUTPUT_LEN];
    assert_eq!(nav.encode(&mut buf), Some(NAV_CONTROLLER_OUTPUT_LEN));
    let decoded = NavControllerOutput::decode(&buf).expect("payload");
    assert!(f32_bits_eq(decoded.nav_roll, 12.0));
    assert!(f32_bits_eq(decoded.nav_pitch, 3.5));
    assert_eq!(decoded.nav_bearing, 275);
    assert_eq!(decoded.target_bearing, 280);
    assert_eq!(decoded.wp_dist, 450);
    assert!(f32_bits_eq(decoded.alt_error, -2.0));
    assert!(f32_bits_eq(decoded.aspd_error, 50.0));
    assert!(f32_bits_eq(decoded.xtrack_error, 8.5));
}

#[test]
fn hud_stream_frames_use_pinned_crc_extras() {
    assert_eq!(VFR_HUD_CRC, 20);
    assert_eq!(NAV_CONTROLLER_OUTPUT_CRC, 183);

    let snap = sample_hud();
    let mut hud_payload = [0u8; VFR_HUD_LEN];
    assert_eq!(snap.vfr_hud().encode(&mut hud_payload), Some(VFR_HUD_LEN));
    let hud_frame = Frame::new(3, 1, 1, MSG_ID_VFR_HUD, &hud_payload).expect("hud frame");
    let mut hud_wire = [0u8; 48];
    let hn = encode_v2(&hud_frame, &mut hud_wire).expect("encode hud");
    assert_eq!(hn, 10 + VFR_HUD_LEN + 2);
    let hud_parsed = decode_v2(hud_wire.get(..hn).expect("slice")).expect("decode hud");
    assert_eq!(hud_parsed.msgid, MSG_ID_VFR_HUD);

    let mut nav_payload = [0u8; NAV_CONTROLLER_OUTPUT_LEN];
    assert_eq!(
        snap.nav_controller_output().encode(&mut nav_payload),
        Some(NAV_CONTROLLER_OUTPUT_LEN)
    );
    let nav_frame =
        Frame::new(4, 1, 1, MSG_ID_NAV_CONTROLLER_OUTPUT, &nav_payload).expect("nav frame");
    let mut nav_wire = [0u8; 48];
    let nn = encode_v2(&nav_frame, &mut nav_wire).expect("encode nav");
    assert_eq!(nn, 10 + NAV_CONTROLLER_OUTPUT_LEN + 2);
    let nav_parsed = decode_v2(nav_wire.get(..nn).expect("slice")).expect("decode nav");
    assert_eq!(nav_parsed.msgid, MSG_ID_NAV_CONTROLLER_OUTPUT);
}

#[test]
fn send_vfr_hud_and_nav_controller_output_from_hud_snapshot() {
    let mut gcs = GcsMavlink::new();
    let snap = sample_hud();

    let mut hud_wire = [0u8; 48];
    let hn = gcs.send_vfr_hud(&mut hud_wire, &snap).expect("send hud");
    assert_eq!(hn, 10 + VFR_HUD_LEN + 2);
    assert_eq!(hud_wire.first().copied(), Some(0xFD));
    let hud_frame = decode_v2(hud_wire.get(..hn).expect("slice")).expect("decode hud");
    assert_eq!(hud_frame.msgid, MSG_ID_VFR_HUD);
    assert_eq!(hud_frame.sysid, 1);
    assert_eq!(hud_frame.compid, 1);
    assert_eq!(hud_frame.seq, 0);
    let hud = VfrHud::from_frame(&hud_frame).expect("vfr_hud");
    assert!(f32_bits_eq(hud.airspeed, snap.airspeed));
    assert!(f32_bits_eq(hud.groundspeed, snap.groundspeed));
    assert_eq!(hud.heading, snap.heading);
    assert_eq!(hud.throttle, snap.throttle);
    assert!(f32_bits_eq(hud.alt, snap.alt));
    assert!(f32_bits_eq(hud.climb, snap.climb));

    let mut nav_wire = [0u8; 48];
    let nn = gcs
        .send_nav_controller_output(&mut nav_wire, &snap)
        .expect("send nav");
    assert_eq!(nn, 10 + NAV_CONTROLLER_OUTPUT_LEN + 2);
    assert_eq!(nav_wire.first().copied(), Some(0xFD));
    let nav_frame = decode_v2(nav_wire.get(..nn).expect("slice")).expect("decode nav");
    assert_eq!(nav_frame.msgid, MSG_ID_NAV_CONTROLLER_OUTPUT);
    assert_eq!(nav_frame.sysid, 1);
    assert_eq!(nav_frame.compid, 1);
    assert_eq!(nav_frame.seq, 1);
    let nav = NavControllerOutput::from_frame(&nav_frame).expect("nav");
    assert!(f32_bits_eq(nav.nav_roll, snap.nav_roll));
    assert!(f32_bits_eq(nav.nav_pitch, snap.nav_pitch));
    assert_eq!(nav.nav_bearing, snap.nav_bearing);
    assert_eq!(nav.target_bearing, snap.target_bearing);
    assert_eq!(nav.wp_dist, snap.wp_dist);
    assert!(f32_bits_eq(nav.alt_error, snap.alt_error));
    assert!(f32_bits_eq(nav.aspd_error, snap.aspd_error));
    assert!(f32_bits_eq(nav.xtrack_error, snap.xtrack_error));
}

#[test]
fn from_frame_rejects_other_msgid() {
    let frame = Frame::new(0, 1, 1, 0, &[0u8; 9]).expect("heartbeat-shaped");
    assert!(VfrHud::from_frame(&frame).is_none());
    assert!(NavControllerOutput::from_frame(&frame).is_none());
}
