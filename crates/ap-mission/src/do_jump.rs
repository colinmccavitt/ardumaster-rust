//! DO_JUMP / jump-to-seq mission command.
//!
//! Upstream `AP_Mission::get_next_cmd` (`libraries/AP_Mission/AP_Mission.cpp`).
//! Plane has no `do_jump` of its own — the mission walker consumes
//! `MAV_CMD_DO_JUMP` before `start_command` / `verify_command`.
//!
//! Wire: `param1` is the target sequence, `param2` is the repeat count
//! (`-1` = forever). A target of `0` or `>= num_commands` fails the search.
//! Jump-tag conversion, the 64-hop walker loop, and EEPROM-backed jump
//! tracking come later.

use ap_math::location::Location;

use crate::{MavFrame, MissionCommand, CMD_INDEX_NONE};

/// `MAV_CMD_DO_JUMP` — jump to a mission sequence.
pub const MAV_CMD_DO_JUMP: u16 = 177;

/// Repeat forever, upstream `AP_MISSION_JUMP_REPEAT_FOREVER`.
pub const JUMP_REPEAT_FOREVER: i16 = -1;

/// Sentinel when jump tracking is full, upstream `AP_MISSION_JUMP_TIMES_MAX`.
pub const JUMP_TIMES_MAX: i16 = 32767;

/// Max jump hops in one `get_next_cmd` walk, upstream `max_loops = 64`.
pub const JUMP_MAX_LOOPS: u8 = 64;

/// Packed jump payload, upstream `AP_Mission::Jump_Command`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct JumpCommand {
    /// Target command index, upstream `jump.target`.
    pub target: u16,
    /// Times to take the jump, `-1` forever. Upstream `jump.num_times`.
    pub num_times: i16,
}

impl JumpCommand {
    /// Pack a jump payload from mavlink `param1` / `param2`.
    #[must_use]
    pub const fn new(target: u16, num_times: i16) -> Self {
        Self { target, num_times }
    }
}

/// Inputs for one DO_JUMP hop, upstream `get_next_cmd` jump arm.
#[derive(Debug, Clone, Copy)]
pub struct DoJumpInputs {
    /// Index of this DO_JUMP item, upstream `cmd.index`.
    pub cmd_seq: u16,
    /// Jump target seq, upstream `cmd.content.jump.target`.
    pub target: u16,
    /// Repeat budget, upstream `cmd.content.jump.num_times`.
    pub num_times: i16,
    /// Times this jump has already been taken, upstream `get_jump_times_run`.
    pub times_run: i16,
    /// Mission length including home, upstream `_cmd_total`.
    pub num_commands: u16,
    /// True when advancing the active nav command (count the hop).
    pub increment: bool,
}

impl Default for DoJumpInputs {
    fn default() -> Self {
        Self {
            cmd_seq: 0,
            target: 0,
            num_times: 0,
            times_run: 0,
            num_commands: 0,
            increment: true,
        }
    }
}

/// Result of one DO_JUMP hop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DoJumpOutput {
    /// False when the target is invalid (`0` or past the list).
    pub valid: bool,
    /// True when the walker should resume at `target`.
    pub jumped: bool,
    /// Next `cmd_index` to search from (`target` or `cmd_seq + 1`).
    pub next_seq: u16,
    /// Updated times-run after this hop.
    pub times_run: i16,
    /// True when the repeat budget is exhausted and the tracker should zero.
    pub reset_counter: bool,
}

impl Default for DoJumpOutput {
    fn default() -> Self {
        Self {
            valid: false,
            jumped: false,
            next_seq: CMD_INDEX_NONE,
            times_run: 0,
            reset_counter: false,
        }
    }
}

/// A `MAV_CMD_DO_JUMP` item at `seq`. Target and repeat live on [`JumpCommand`].
#[must_use]
pub const fn do_jump_cmd(seq: u16) -> MissionCommand {
    MissionCommand {
        seq,
        command: MAV_CMD_DO_JUMP,
        frame: MavFrame::Global,
        location: Location::new(0, 0),
    }
}

/// Pack a jump payload, upstream `Jump_Command` from mavlink param1/param2.
#[must_use]
pub const fn jump_content(target: u16, num_times: i16) -> JumpCommand {
    JumpCommand::new(target, num_times)
}

/// Whether `command` is `MAV_CMD_DO_JUMP`.
#[must_use]
pub const fn is_do_jump(command: u16) -> bool {
    command == MAV_CMD_DO_JUMP
}

/// True when `target` is a real command index inside the mission.
///
/// Upstream `get_next_cmd`: reject `target == 0` (home) and
/// `target >= _cmd_total`.
#[must_use]
pub const fn jump_target_valid(target: u16, num_commands: u16) -> bool {
    target != 0 && target < num_commands
}

/// True when the jump still has remaining repeats.
///
/// Upstream: `num_times == AP_MISSION_JUMP_REPEAT_FOREVER ||
/// get_jump_times_run(cmd) < num_times`.
#[must_use]
pub const fn jump_should_take(num_times: i16, times_run: i16) -> bool {
    num_times == JUMP_REPEAT_FOREVER || times_run < num_times
}

/// Resolve one DO_JUMP hop, upstream `get_next_cmd` jump arm.
///
/// On a valid remaining jump, resume at `target` and increment `times_run`
/// when `increment` is set. Once the repeat budget is exhausted, continue at
/// `cmd_seq + 1` and report that the tracker should zero. Invalid targets
/// fail the search (`valid = false`).
#[must_use]
pub fn do_jump(inp: &DoJumpInputs) -> DoJumpOutput {
    if !jump_target_valid(inp.target, inp.num_commands) {
        return DoJumpOutput {
            valid: false,
            jumped: false,
            next_seq: CMD_INDEX_NONE,
            times_run: inp.times_run,
            reset_counter: false,
        };
    }
    if jump_should_take(inp.num_times, inp.times_run) {
        let times_run = if inp.increment {
            inp.times_run.saturating_add(1)
        } else {
            inp.times_run
        };
        DoJumpOutput {
            valid: true,
            jumped: true,
            next_seq: inp.target,
            times_run,
            reset_counter: false,
        }
    } else {
        DoJumpOutput {
            valid: true,
            jumped: false,
            next_seq: inp.cmd_seq.saturating_add(1),
            times_run: inp.times_run,
            reset_counter: inp.increment,
        }
    }
}
