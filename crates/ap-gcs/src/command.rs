//! COMMAND_LONG / COMMAND_INT dispatch, extracted from the pinned
//! Plane-4.7.0 `modules/mavlink/message_definitions/v1.0/common.xml`.
//!
//! This slice is those two messages plus the three Plane-relevant
//! `MAV_CMD` ids (ARM/DISARM, DO_SET_MODE, NAV_TAKEOFF). A full dialect
//! generator is a later FW-028 slice. Vehicle-side execution
//! (`handle_command_component_arm_disarm`, mode change, takeoff) is not
//! here — only the msgid / command-id table that `handle_message` uses.

use crate::framing::Frame;

/// COMMAND_INT message id, upstream `MAVLINK_MSG_ID_COMMAND_INT`.
pub const MSG_ID_COMMAND_INT: u32 = 75;

/// COMMAND_LONG message id, upstream `MAVLINK_MSG_ID_COMMAND_LONG`.
pub const MSG_ID_COMMAND_LONG: u32 = 76;

/// Packed payload length, upstream `MAVLINK_MSG_ID_COMMAND_INT_LEN`.
pub const COMMAND_INT_LEN: usize = 35;

/// Packed payload length, upstream `MAVLINK_MSG_ID_COMMAND_LONG_LEN`.
pub const COMMAND_LONG_LEN: usize = 33;

/// CRC extra, upstream `MAVLINK_MSG_ID_COMMAND_INT_CRC`.
pub const COMMAND_INT_CRC: u8 = 158;

/// CRC extra, upstream `MAVLINK_MSG_ID_COMMAND_LONG_CRC`.
pub const COMMAND_LONG_CRC: u8 = 152;

/// `MAV_CMD_NAV_TAKEOFF` — first flown mission item on Plane.
pub const MAV_CMD_NAV_TAKEOFF: u16 = 22;

/// `MAV_CMD_DO_SET_MODE` — GCS flight-mode change.
pub const MAV_CMD_DO_SET_MODE: u16 = 176;

/// `MAV_CMD_COMPONENT_ARM_DISARM`.
pub const MAV_CMD_COMPONENT_ARM_DISARM: u16 = 400;

/// `param2` force-arm magic, common.xml increment 21196.
pub const ARM_DISARM_FORCE: f32 = 21196.0;

/// `MAV_FRAME_GLOBAL_RELATIVE_ALT` — typical Plane takeoff altitude frame.
pub const MAV_FRAME_GLOBAL_RELATIVE_ALT: u8 = 3;

/// Which command message carried the `MAV_CMD`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandVia {
    /// msgid 76 — `COMMAND_LONG` (seven floats).
    Long,
    /// msgid 75 — `COMMAND_INT` (scaled x/y + frame).
    Int,
}

/// Plane-relevant `MAV_CMD` ids handled by this slice's table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlaneCommand {
    /// `MAV_CMD_COMPONENT_ARM_DISARM` (400).
    ArmDisarm,
    /// `MAV_CMD_DO_SET_MODE` (176).
    DoSetMode,
    /// `MAV_CMD_NAV_TAKEOFF` (22).
    NavTakeoff,
}

/// Look up a `MAV_CMD` in the Plane table. `None` is unsupported here.
#[must_use]
pub const fn classify(command: u16) -> Option<PlaneCommand> {
    match command {
        MAV_CMD_NAV_TAKEOFF => Some(PlaneCommand::NavTakeoff),
        MAV_CMD_DO_SET_MODE => Some(PlaneCommand::DoSetMode),
        MAV_CMD_COMPONENT_ARM_DISARM => Some(PlaneCommand::ArmDisarm),
        _ => None,
    }
}

/// Packed COMMAND_LONG fields, upstream `mavlink_command_long_t`.
///
/// Wire order is size-sorted (`mavlink_msg_command_long_pack`): seven
/// floats, then `command`, `target_system`, `target_component`,
/// `confirmation`.
#[derive(Debug, Clone, Copy)]
pub struct CommandLong {
    /// Parameter 1 (command-specific).
    pub param1: f32,
    /// Parameter 2 (command-specific).
    pub param2: f32,
    /// Parameter 3 (command-specific).
    pub param3: f32,
    /// Parameter 4 (command-specific).
    pub param4: f32,
    /// Parameter 5 (command-specific).
    pub param5: f32,
    /// Parameter 6 (command-specific).
    pub param6: f32,
    /// Parameter 7 (command-specific).
    pub param7: f32,
    /// `MAV_CMD` id.
    pub command: u16,
    /// System which should execute the command.
    pub target_system: u8,
    /// Component which should execute the command, 0 for all.
    pub target_component: u8,
    /// 0 first transmission; 1-255 confirmation.
    pub confirmation: u8,
}

impl CommandLong {
    /// Build a COMMAND_LONG from the XML field order (not wire order).
    #[must_use]
    pub const fn new(
        target_system: u8,
        target_component: u8,
        command: u16,
        confirmation: u8,
        params: [f32; 7],
    ) -> Self {
        let [param1, param2, param3, param4, param5, param6, param7] = params;
        Self {
            param1,
            param2,
            param3,
            param4,
            param5,
            param6,
            param7,
            command,
            target_system,
            target_component,
            confirmation,
        }
    }

    /// The seven command parameters in `param1`..`param7` order.
    #[must_use]
    pub const fn params(&self) -> [f32; 7] {
        [
            self.param1,
            self.param2,
            self.param3,
            self.param4,
            self.param5,
            self.param6,
            self.param7,
        ]
    }

    /// Pack into 33 little-endian bytes. `None` if `buf` is too short.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..COMMAND_LONG_LEN)?;
        write_f32_le(dest, 0, self.param1)?;
        write_f32_le(dest, 4, self.param2)?;
        write_f32_le(dest, 8, self.param3)?;
        write_f32_le(dest, 12, self.param4)?;
        write_f32_le(dest, 16, self.param5)?;
        write_f32_le(dest, 20, self.param6)?;
        write_f32_le(dest, 24, self.param7)?;
        write_u16_le(dest, 28, self.command)?;
        *dest.get_mut(30)? = self.target_system;
        *dest.get_mut(31)? = self.target_component;
        *dest.get_mut(32)? = self.confirmation;
        Some(COMMAND_LONG_LEN)
    }

    /// Unpack 33 bytes. `None` if `buf` is shorter than the min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..COMMAND_LONG_LEN)?;
        Some(Self {
            param1: read_f32_le(src, 0)?,
            param2: read_f32_le(src, 4)?,
            param3: read_f32_le(src, 8)?,
            param4: read_f32_le(src, 12)?,
            param5: read_f32_le(src, 16)?,
            param6: read_f32_le(src, 20)?,
            param7: read_f32_le(src, 24)?,
            command: read_u16_le(src, 28)?,
            target_system: *src.get(30)?,
            target_component: *src.get(31)?,
            confirmation: *src.get(32)?,
        })
    }

    /// Decode a framed COMMAND_LONG. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_COMMAND_LONG {
            return None;
        }
        Self::decode(frame.payload())
    }
}

/// Packed COMMAND_INT fields, upstream `mavlink_command_int_t`.
///
/// Wire order is size-sorted (`mavlink_msg_command_int_pack`): four
/// floats, `x`, `y`, `z`, then `command` and the five `uint8` fields.
#[derive(Debug, Clone, Copy)]
pub struct CommandInt {
    /// PARAM1, see `MAV_CMD`.
    pub param1: f32,
    /// PARAM2, see `MAV_CMD`.
    pub param2: f32,
    /// PARAM3, see `MAV_CMD`.
    pub param3: f32,
    /// PARAM4, see `MAV_CMD`.
    pub param4: f32,
    /// PARAM5 / local x × 1e4 / global latitude × 1e7.
    pub x: i32,
    /// PARAM6 / local y × 1e4 / global longitude × 1e7.
    pub y: i32,
    /// PARAM7 / altitude (frame-dependent).
    pub z: f32,
    /// `MAV_CMD` id.
    pub command: u16,
    /// System ID.
    pub target_system: u8,
    /// Component ID.
    pub target_component: u8,
    /// Coordinate system (`MAV_FRAME`).
    pub frame: u8,
    /// Not used.
    pub current: u8,
    /// Not used (set 0).
    pub autocontinue: u8,
}

impl CommandInt {
    /// Build a COMMAND_INT from the XML field order (not wire order).
    #[must_use]
    pub const fn new(
        target_system: u8,
        target_component: u8,
        frame: u8,
        command: u16,
        current: u8,
        autocontinue: u8,
        param1: f32,
        param2: f32,
        param3: f32,
        param4: f32,
        x: i32,
        y: i32,
        z: f32,
    ) -> Self {
        Self {
            param1,
            param2,
            param3,
            param4,
            x,
            y,
            z,
            command,
            target_system,
            target_component,
            frame,
            current,
            autocontinue,
        }
    }

    /// Pack into 35 little-endian bytes. `None` if `buf` is too short.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..COMMAND_INT_LEN)?;
        write_f32_le(dest, 0, self.param1)?;
        write_f32_le(dest, 4, self.param2)?;
        write_f32_le(dest, 8, self.param3)?;
        write_f32_le(dest, 12, self.param4)?;
        write_i32_le(dest, 16, self.x)?;
        write_i32_le(dest, 20, self.y)?;
        write_f32_le(dest, 24, self.z)?;
        write_u16_le(dest, 28, self.command)?;
        *dest.get_mut(30)? = self.target_system;
        *dest.get_mut(31)? = self.target_component;
        *dest.get_mut(32)? = self.frame;
        *dest.get_mut(33)? = self.current;
        *dest.get_mut(34)? = self.autocontinue;
        Some(COMMAND_INT_LEN)
    }

    /// Unpack 35 bytes. `None` if `buf` is shorter than the min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..COMMAND_INT_LEN)?;
        Some(Self {
            param1: read_f32_le(src, 0)?,
            param2: read_f32_le(src, 4)?,
            param3: read_f32_le(src, 8)?,
            param4: read_f32_le(src, 12)?,
            x: read_i32_le(src, 16)?,
            y: read_i32_le(src, 20)?,
            z: read_f32_le(src, 24)?,
            command: read_u16_le(src, 28)?,
            target_system: *src.get(30)?,
            target_component: *src.get(31)?,
            frame: *src.get(32)?,
            current: *src.get(33)?,
            autocontinue: *src.get(34)?,
        })
    }

    /// Decode a framed COMMAND_INT. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_COMMAND_INT {
            return None;
        }
        Self::decode(frame.payload())
    }
}

fn write_f32_le(buf: &mut [u8], off: usize, value: f32) -> Option<()> {
    let end = off.checked_add(4)?;
    buf.get_mut(off..end)?.copy_from_slice(&value.to_le_bytes());
    Some(())
}

fn read_f32_le(buf: &[u8], off: usize) -> Option<f32> {
    let end = off.checked_add(4)?;
    let bytes = buf.get(off..end)?;
    let mut le = [0u8; 4];
    le.copy_from_slice(bytes);
    Some(f32::from_le_bytes(le))
}

fn write_i32_le(buf: &mut [u8], off: usize, value: i32) -> Option<()> {
    let end = off.checked_add(4)?;
    buf.get_mut(off..end)?.copy_from_slice(&value.to_le_bytes());
    Some(())
}

fn read_i32_le(buf: &[u8], off: usize) -> Option<i32> {
    let end = off.checked_add(4)?;
    let bytes = buf.get(off..end)?;
    let mut le = [0u8; 4];
    le.copy_from_slice(bytes);
    Some(i32::from_le_bytes(le))
}

fn write_u16_le(buf: &mut [u8], off: usize, value: u16) -> Option<()> {
    let end = off.checked_add(2)?;
    buf.get_mut(off..end)?.copy_from_slice(&value.to_le_bytes());
    Some(())
}

fn read_u16_le(buf: &[u8], off: usize) -> Option<u16> {
    let end = off.checked_add(2)?;
    let bytes = buf.get(off..end)?;
    let mut le = [0u8; 2];
    le.copy_from_slice(bytes);
    Some(u16::from_le_bytes(le))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classify_table_is_plane_commands_only() {
        assert_eq!(
            classify(MAV_CMD_NAV_TAKEOFF),
            Some(PlaneCommand::NavTakeoff)
        );
        assert_eq!(classify(MAV_CMD_DO_SET_MODE), Some(PlaneCommand::DoSetMode));
        assert_eq!(
            classify(MAV_CMD_COMPONENT_ARM_DISARM),
            Some(PlaneCommand::ArmDisarm)
        );
        // NAV_WAYPOINT / NAV_LAND stay ungenerated this slice.
        assert_eq!(classify(16), None);
        assert_eq!(classify(21), None);
    }

    #[test]
    fn encode_rejects_short_buffer() {
        let cmd = CommandLong::new(1, 1, MAV_CMD_DO_SET_MODE, 0, [0.0; 7]);
        assert!(cmd.encode(&mut [0u8; 8]).is_none());
        let int = CommandInt::new(
            1,
            1,
            MAV_FRAME_GLOBAL_RELATIVE_ALT,
            MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0.0,
            0.0,
            0.0,
            0.0,
            0,
            0,
            50.0,
        );
        assert!(int.encode(&mut [0u8; 8]).is_none());
    }

    #[test]
    fn decode_rejects_short_payload() {
        assert!(CommandLong::decode(&[0, 1, 2]).is_none());
        assert!(CommandInt::decode(&[0, 1, 2]).is_none());
    }
}
