//! COMMAND_LONG / COMMAND_INT encode and Plane command-table dispatch.

use ap_gcs::{
    classify, decode_v2, encode_v2, CommandInt, CommandLong, CommandVia, Dispatch, Frame,
    GcsMavlink, PlaneCommand, ARM_DISARM_FORCE, COMMAND_INT_LEN, COMMAND_LONG_LEN,
    MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_DO_SET_MODE, MAV_CMD_NAV_TAKEOFF, MSG_ID_COMMAND_INT,
    MSG_ID_COMMAND_LONG,
};

fn command_long_frame(cmd: &CommandLong, sysid: u8) -> Frame {
    let mut payload = [0u8; COMMAND_LONG_LEN];
    assert_eq!(cmd.encode(&mut payload), Some(COMMAND_LONG_LEN));
    Frame::new(3, sysid, 190, MSG_ID_COMMAND_LONG, &payload).expect("frame")
}

fn command_int_frame(cmd: &CommandInt, sysid: u8) -> Frame {
    let mut payload = [0u8; COMMAND_INT_LEN];
    assert_eq!(cmd.encode(&mut payload), Some(COMMAND_INT_LEN));
    Frame::new(4, sysid, 190, MSG_ID_COMMAND_INT, &payload).expect("frame")
}

#[test]
fn command_long_payload_roundtrip_arm_disarm() {
    let cmd = CommandLong::new(
        1,
        1,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        [1.0, ARM_DISARM_FORCE, 0.0, 0.0, 0.0, 0.0, 0.0],
    );
    let mut buf = [0u8; COMMAND_LONG_LEN];
    assert_eq!(cmd.encode(&mut buf), Some(COMMAND_LONG_LEN));
    let decoded = CommandLong::decode(&buf).expect("payload");
    assert_eq!(decoded.command, MAV_CMD_COMPONENT_ARM_DISARM);
    assert_eq!(decoded.target_system, 1);
    assert_eq!(decoded.target_component, 1);
    assert_eq!(decoded.confirmation, 0);
    assert_eq!(decoded.param1.to_le_bytes(), 1.0f32.to_le_bytes());
    assert_eq!(decoded.param2.to_le_bytes(), ARM_DISARM_FORCE.to_le_bytes());
}

#[test]
fn command_int_payload_roundtrip_nav_takeoff() {
    let cmd = CommandInt::new(
        1,
        1,
        ap_gcs::MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        10.0,
        0.0,
        0.0,
        0.0,
        334_572_800,
        -1_121_241_000,
        50.0,
    );
    let mut buf = [0u8; COMMAND_INT_LEN];
    assert_eq!(cmd.encode(&mut buf), Some(COMMAND_INT_LEN));
    let decoded = CommandInt::decode(&buf).expect("payload");
    assert_eq!(decoded.command, MAV_CMD_NAV_TAKEOFF);
    assert_eq!(decoded.frame, ap_gcs::MAV_FRAME_GLOBAL_RELATIVE_ALT);
    assert_eq!(decoded.x, 334_572_800);
    assert_eq!(decoded.y, -1_121_241_000);
    assert_eq!(decoded.z.to_le_bytes(), 50.0f32.to_le_bytes());
    assert_eq!(decoded.param1.to_le_bytes(), 10.0f32.to_le_bytes());
}

#[test]
fn handle_command_long_arm_disarm() {
    let cmd = CommandLong::new(
        1,
        1,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    );
    let frame = command_long_frame(&cmd, 255);
    let mut gcs = GcsMavlink::new();
    match gcs.handle_message(&frame, 0) {
        Dispatch::Command { via, command, kind } => {
            assert_eq!(via, CommandVia::Long);
            assert_eq!(command, MAV_CMD_COMPONENT_ARM_DISARM);
            assert_eq!(kind, Some(PlaneCommand::ArmDisarm));
        }
        other => panic!("expected Command, got {other:?}"),
    }
}

#[test]
fn handle_command_long_do_set_mode() {
    // param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; param2 = plane custom mode.
    let cmd = CommandLong::new(
        1,
        1,
        MAV_CMD_DO_SET_MODE,
        0,
        [1.0, 12.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    );
    let frame = command_long_frame(&cmd, 255);
    let mut gcs = GcsMavlink::new();
    match gcs.handle_message(&frame, 0) {
        Dispatch::Command { via, command, kind } => {
            assert_eq!(via, CommandVia::Long);
            assert_eq!(command, MAV_CMD_DO_SET_MODE);
            assert_eq!(kind, Some(PlaneCommand::DoSetMode));
        }
        other => panic!("expected Command, got {other:?}"),
    }
}

#[test]
fn handle_command_int_nav_takeoff() {
    let cmd = CommandInt::new(
        1,
        1,
        ap_gcs::MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0.0,
        0.0,
        0.0,
        0.0,
        0,
        0,
        80.0,
    );
    let frame = command_int_frame(&cmd, 255);
    let mut gcs = GcsMavlink::new();
    match gcs.handle_message(&frame, 0) {
        Dispatch::Command { via, command, kind } => {
            assert_eq!(via, CommandVia::Int);
            assert_eq!(command, MAV_CMD_NAV_TAKEOFF);
            assert_eq!(kind, Some(PlaneCommand::NavTakeoff));
        }
        other => panic!("expected Command, got {other:?}"),
    }
}

#[test]
fn unsupported_command_is_dispatched_with_no_kind() {
    // MAV_CMD_NAV_LAND is in the dialect but not this Plane table.
    let cmd = CommandLong::new(1, 1, 21, 0, [0.0; 7]);
    let frame = command_long_frame(&cmd, 255);
    let mut gcs = GcsMavlink::new();
    match gcs.handle_message(&frame, 0) {
        Dispatch::Command { command, kind, .. } => {
            assert_eq!(command, 21);
            assert_eq!(kind, None);
            assert_eq!(classify(21), None);
        }
        other => panic!("expected Command, got {other:?}"),
    }
}

#[test]
fn command_not_addressed_to_us_is_unknown() {
    let cmd = CommandLong::new(
        9,
        1,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    );
    let frame = command_long_frame(&cmd, 255);
    let mut gcs = GcsMavlink::new();
    assert_eq!(
        gcs.handle_message(&frame, 0),
        Dispatch::Unknown {
            msgid: MSG_ID_COMMAND_LONG
        }
    );
}

#[test]
fn broadcast_target_system_zero_is_addressed() {
    let cmd = CommandLong::new(
        0,
        0,
        MAV_CMD_DO_SET_MODE,
        0,
        [1.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    );
    let frame = command_long_frame(&cmd, 255);
    let mut gcs = GcsMavlink::new();
    match gcs.handle_message(&frame, 0) {
        Dispatch::Command { kind, .. } => assert_eq!(kind, Some(PlaneCommand::DoSetMode)),
        other => panic!("expected Command, got {other:?}"),
    }
}

#[test]
fn command_long_frames_with_crc_extra_152() {
    let cmd = CommandLong::new(1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, [0.0; 7]);
    let frame = command_long_frame(&cmd, 1);
    let mut wire = [0u8; 64];
    let n = encode_v2(&frame, &mut wire).expect("encode");
    // STX + 10-byte header + 33-byte payload + 2-byte CRC.
    assert_eq!(n, 10 + COMMAND_LONG_LEN + 2);
    assert_eq!(wire.first().copied(), Some(0xFD));
    let parsed = decode_v2(wire.get(..n).expect("slice")).expect("decode");
    assert_eq!(parsed.msgid, MSG_ID_COMMAND_LONG);
    let again = CommandLong::from_frame(&parsed).expect("command");
    assert_eq!(again.command, MAV_CMD_COMPONENT_ARM_DISARM);
}

#[test]
fn command_int_frames_with_crc_extra_158() {
    let cmd = CommandInt::new(
        1,
        1,
        0,
        MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0.0,
        0.0,
        0.0,
        0.0,
        0,
        0,
        0.0,
    );
    let frame = command_int_frame(&cmd, 1);
    let mut wire = [0u8; 64];
    let n = encode_v2(&frame, &mut wire).expect("encode");
    assert_eq!(n, 10 + COMMAND_INT_LEN + 2);
    let parsed = decode_v2(wire.get(..n).expect("slice")).expect("decode");
    assert_eq!(parsed.msgid, MSG_ID_COMMAND_INT);
    assert_eq!(
        CommandInt::from_frame(&parsed).expect("command").command,
        MAV_CMD_NAV_TAKEOFF
    );
}

#[test]
fn from_frame_rejects_other_msgid() {
    let frame = Frame::new(0, 1, 1, 0, &[0u8; 9]).expect("heartbeat-shaped");
    assert!(CommandLong::from_frame(&frame).is_none());
    assert!(CommandInt::from_frame(&frame).is_none());
}
