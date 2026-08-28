//! AP_Mission command/item storage: seq, command, frame, lat/lon/alt.

use ap_math::location::AltFrame;
use ap_mission::{
    MavFrame, Mission, MissionCommand, CMD_ID_NONE, CMD_INDEX_NONE, FIRST_REAL_COMMAND,
    MAV_CMD_NAV_WAYPOINT, MAX_COMMANDS,
};

#[test]
fn nav_waypoint_stores_seq_command_frame_and_lla() {
    let cmd = MissionCommand::waypoint(
        FIRST_REAL_COMMAND,
        MavFrame::GlobalRelativeAlt,
        -35_363_261,
        149_165_237,
        10_000,
    );
    assert_eq!(cmd.seq, 1);
    assert_eq!(cmd.command, MAV_CMD_NAV_WAYPOINT);
    assert!(cmd.is_nav_waypoint());
    assert_eq!(cmd.frame, MavFrame::GlobalRelativeAlt);
    assert_eq!(cmd.frame.as_u8(), 3);
    assert_eq!(cmd.location.lat, -35_363_261);
    assert_eq!(cmd.location.lng, 149_165_237);
    assert_eq!(cmd.location.alt, 10_000);
    assert_eq!(cmd.location.alt_frame(), AltFrame::AboveHome);
}

#[test]
fn write_then_read_round_trips_a_waypoint_item() {
    let mut mission = Mission::new();
    assert_eq!(mission.num_commands(), 0);

    let home = MissionCommand::waypoint(0, MavFrame::Global, -35_363_000, 149_165_000, 58_400);
    assert!(mission.write_cmd(home));
    assert_eq!(mission.num_commands(), 1);

    let wp = MissionCommand::waypoint(
        FIRST_REAL_COMMAND,
        MavFrame::GlobalRelativeAlt,
        -35_362_000,
        149_166_000,
        12_000,
    );
    assert!(mission.add_cmd(wp));
    assert_eq!(mission.num_commands(), 2);

    let stored = mission.read_cmd(1).expect("seq 1 written");
    assert_eq!(stored.seq, 1);
    assert_eq!(stored.command, MAV_CMD_NAV_WAYPOINT);
    assert_eq!(stored.frame, MavFrame::GlobalRelativeAlt);
    assert_eq!(stored.location.lat, -35_362_000);
    assert_eq!(stored.location.lng, 149_166_000);
    assert_eq!(stored.location.alt, 12_000);
}

#[test]
fn home_is_seq_zero_and_the_first_real_command_is_one() {
    let mut mission = Mission::new();
    assert!(mission.add_cmd(MissionCommand::waypoint(99, MavFrame::Global, 1, 2, 3)));
    let home = mission.read_cmd(0).expect("home");
    assert_eq!(
        home.seq, 0,
        "add_cmd assigns the next seq, starting at home"
    );
    assert_eq!(FIRST_REAL_COMMAND, 1);
    assert!(mission.read_cmd(FIRST_REAL_COMMAND).is_none());
}

#[test]
fn missing_or_out_of_range_seq_reads_as_none() {
    let mission = Mission::new();
    assert!(mission.read_cmd(0).is_none());
    assert!(mission.read_cmd(CMD_INDEX_NONE).is_none());
    let empty = MissionCommand::none();
    assert_eq!(empty.seq, CMD_INDEX_NONE);
    assert_eq!(empty.command, CMD_ID_NONE);
    assert!(!empty.is_nav_waypoint());
}

#[test]
fn write_rejects_a_hole_and_capacity() {
    let mut mission = Mission::new();
    let skipped = MissionCommand::waypoint(1, MavFrame::Global, 1, 1, 1);
    assert!(!mission.write_cmd(skipped), "cannot skip seq 0");
    assert_eq!(mission.num_commands(), 0);

    for i in 0..MAX_COMMANDS {
        assert!(
            mission.add_cmd(MissionCommand::waypoint(
                0,
                MavFrame::Global,
                i as i32 + 1,
                1,
                0
            )),
            "slot {i}"
        );
    }
    assert!(!mission.add_cmd(MissionCommand::waypoint(0, MavFrame::Global, 99, 99, 0)));
    assert_eq!(mission.num_commands(), MAX_COMMANDS as u16);
}

#[test]
fn mav_frame_int_aliases_and_unknowns() {
    assert_eq!(MavFrame::from_u8(0), Some(MavFrame::Global));
    assert_eq!(MavFrame::from_u8(5), Some(MavFrame::Global));
    assert_eq!(MavFrame::from_u8(3), Some(MavFrame::GlobalRelativeAlt));
    assert_eq!(MavFrame::from_u8(6), Some(MavFrame::GlobalRelativeAlt));
    assert_eq!(MavFrame::from_u8(10), Some(MavFrame::GlobalTerrainAlt));
    assert_eq!(MavFrame::from_u8(11), Some(MavFrame::GlobalTerrainAlt));
    assert_eq!(
        MavFrame::from_u8(1),
        None,
        "LOCAL_NED is not a waypoint frame"
    );
    assert_eq!(
        MavFrame::GlobalTerrainAlt.to_alt_frame(),
        AltFrame::AboveTerrain
    );
}

#[test]
fn clear_drops_every_item() {
    let mut mission = Mission::new();
    assert!(mission.add_cmd(MissionCommand::waypoint(0, MavFrame::Global, 1, 2, 3)));
    mission.clear();
    assert_eq!(mission.num_commands(), 0);
    assert!(mission.read_cmd(0).is_none());
}
