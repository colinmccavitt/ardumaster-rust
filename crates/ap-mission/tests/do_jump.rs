//! DO_JUMP / jump-to-seq mission command (one hop of get_next_cmd).

use ap_mission::{
    do_jump, do_jump_cmd, is_do_jump, jump_content, jump_should_take, jump_target_valid,
    DoJumpInputs, Mission, CMD_INDEX_NONE, FIRST_REAL_COMMAND, JUMP_REPEAT_FOREVER, JUMP_TIMES_MAX,
    MAV_CMD_DO_JUMP, MAV_CMD_NAV_WAYPOINT,
};

#[test]
fn command_id_is_mav_cmd_do_jump() {
    let cmd = do_jump_cmd(FIRST_REAL_COMMAND);
    assert_eq!(MAV_CMD_DO_JUMP, 177);
    assert_eq!(cmd.command, MAV_CMD_DO_JUMP);
    assert!(is_do_jump(cmd.command));
    assert!(!is_do_jump(MAV_CMD_NAV_WAYPOINT));
    assert_eq!(cmd.seq, 1);
}

#[test]
fn jump_content_packs_target_and_repeat() {
    let jump = jump_content(3, 2);
    assert_eq!(jump.target, 3);
    assert_eq!(jump.num_times, 2);
    let forever = jump_content(1, JUMP_REPEAT_FOREVER);
    assert_eq!(forever.num_times, -1);
}

#[test]
fn do_jump_cmd_round_trips_through_mission_storage() {
    let mut mission = Mission::new();
    assert!(mission.add_cmd(ap_mission::MissionCommand::waypoint(
        0,
        ap_mission::MavFrame::Global,
        1,
        2,
        3,
    )));
    assert!(mission.add_cmd(do_jump_cmd(99)));
    let stored = mission.read_cmd(1).expect("seq 1 written");
    assert_eq!(stored.seq, 1);
    assert_eq!(stored.command, MAV_CMD_DO_JUMP);
    assert!(is_do_jump(stored.command));
}

#[test]
fn do_jump_takes_target_when_repeats_remain() {
    let out = do_jump(&DoJumpInputs {
        cmd_seq: 4,
        target: 2,
        num_times: 3,
        times_run: 0,
        num_commands: 6,
        increment: true,
    });
    assert!(out.valid);
    assert!(out.jumped, "first hop must resume at the target seq");
    assert_eq!(out.next_seq, 2);
    assert_eq!(out.times_run, 1, "advancing increments get_jump_times_run");
    assert!(!out.reset_counter);
}

#[test]
fn do_jump_lookahead_does_not_increment() {
    let out = do_jump(&DoJumpInputs {
        cmd_seq: 4,
        target: 2,
        num_times: 3,
        times_run: 1,
        num_commands: 6,
        increment: false,
    });
    assert!(out.valid);
    assert!(out.jumped);
    assert_eq!(out.next_seq, 2);
    assert_eq!(
        out.times_run, 1,
        "get_next_cmd lookahead leaves num_times_run alone"
    );
}

#[test]
fn do_jump_skips_when_repeat_count_exhausted() {
    let out = do_jump(&DoJumpInputs {
        cmd_seq: 4,
        target: 2,
        num_times: 3,
        times_run: 3,
        num_commands: 6,
        increment: true,
    });
    assert!(out.valid);
    assert!(!out.jumped, "budget spent: search continues after the jump");
    assert_eq!(out.next_seq, 5);
    assert_eq!(out.times_run, 3);
    assert!(
        out.reset_counter,
        "exhausted jump zeros the tracker so a later WP restart can loop again"
    );
}

#[test]
fn do_jump_forever_always_takes() {
    let out = do_jump(&DoJumpInputs {
        cmd_seq: 4,
        target: 1,
        num_times: JUMP_REPEAT_FOREVER,
        times_run: JUMP_TIMES_MAX,
        num_commands: 5,
        increment: true,
    });
    assert!(out.valid);
    assert!(out.jumped);
    assert_eq!(out.next_seq, 1);
    assert!(jump_should_take(JUMP_REPEAT_FOREVER, 0));
    assert!(jump_should_take(JUMP_REPEAT_FOREVER, JUMP_TIMES_MAX));
}

#[test]
fn do_jump_rejects_home_and_past_end() {
    let home = do_jump(&DoJumpInputs {
        cmd_seq: 3,
        target: 0,
        num_times: 1,
        times_run: 0,
        num_commands: 4,
        increment: true,
    });
    assert!(!home.valid, "target 0 is home, not a real command");
    assert!(!home.jumped);
    assert_eq!(home.next_seq, CMD_INDEX_NONE);

    let past = do_jump(&DoJumpInputs {
        cmd_seq: 3,
        target: 4,
        num_times: 1,
        times_run: 0,
        num_commands: 4,
        increment: true,
    });
    assert!(!past.valid, "target >= num_commands fails the search");
    assert!(!jump_target_valid(0, 4));
    assert!(!jump_target_valid(4, 4));
    assert!(jump_target_valid(1, 4));
}

#[test]
fn jump_should_take_is_strictly_less_than_repeat() {
    assert!(jump_should_take(3, 0));
    assert!(jump_should_take(3, 2));
    assert!(
        !jump_should_take(3, 3),
        "times_run == num_times means the jump is spent"
    );
    assert!(!jump_should_take(0, 0));
}
