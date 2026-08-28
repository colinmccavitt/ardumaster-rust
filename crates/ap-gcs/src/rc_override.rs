//! MANUAL_CONTROL / RC_CHANNELS_OVERRIDE ingest, extracted from the pinned
//! Plane-4.7.0 `modules/mavlink/message_definitions/v1.0` defs
//! (`common.xml` msgid 69, msgid 70).
//!
//! Upstream `GCS_MAVLINK::handle_rc_channels_override` stores PWM overrides
//! (`RC_Channels::set_override`) and `handle_manual_control` maps joystick
//! axes through Plane `handle_manual_control_axes` / `manual_override`. This
//! slice is that ingest stub — store channel overrides from a framed packet.
//! The rest of the dialect and HAL RC output stay for later.

use crate::framing::Frame;

/// MANUAL_CONTROL message id, upstream `MAVLINK_MSG_ID_MANUAL_CONTROL`.
pub const MSG_ID_MANUAL_CONTROL: u32 = 69;

/// RC_CHANNELS_OVERRIDE message id, upstream `MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE`.
pub const MSG_ID_RC_CHANNELS_OVERRIDE: u32 = 70;

/// Packed payload length, upstream `MAVLINK_MSG_ID_MANUAL_CONTROL_LEN`.
pub const MANUAL_CONTROL_LEN: usize = 30;

/// Minimum payload length, upstream `MAVLINK_MSG_ID_MANUAL_CONTROL_MIN_LEN`.
pub const MANUAL_CONTROL_MIN_LEN: usize = 11;

/// Packed payload length, upstream `MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_LEN`.
pub const RC_CHANNELS_OVERRIDE_LEN: usize = 38;

/// Minimum payload length, upstream `MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_MIN_LEN`.
pub const RC_CHANNELS_OVERRIDE_MIN_LEN: usize = 18;

/// CRC extra, upstream `MAVLINK_MSG_ID_MANUAL_CONTROL_CRC`.
pub const MANUAL_CONTROL_CRC: u8 = 243;

/// CRC extra, upstream `MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE_CRC`.
pub const RC_CHANNELS_OVERRIDE_CRC: u8 = 124;

/// Override slots stored by this stub, upstream `override_data` (chan1..chan16).
pub const OVERRIDE_CHANNEL_COUNT: usize = 16;

/// Ignore this RC_CHANNELS_OVERRIDE field, upstream `UINT16_MAX`.
pub const OVERRIDE_IGNORE: u16 = u16::MAX;

/// Release an extension channel (chan9+) back to radio, upstream `UINT16_MAX-1`.
pub const OVERRIDE_RELEASE_EXT: u16 = u16::MAX - 1;

/// Invalid MANUAL_CONTROL axis, upstream `INT16_MAX`.
pub const MANUAL_AXIS_INVALID: i16 = i16::MAX;

/// Default radio-min used by the MANUAL_CONTROL PWM map stub.
pub const MANUAL_RADIO_MIN: u16 = 1_000;

/// Default radio-max used by the MANUAL_CONTROL PWM map stub.
pub const MANUAL_RADIO_MAX: u16 = 2_000;

/// Packed RC_CHANNELS_OVERRIDE fields, upstream `mavlink_rc_channels_override_t`.
///
/// Wire order is size-sorted for the base message
/// (`mavlink_msg_rc_channels_override_pack`): eight `chanN_raw` words,
/// `target_system`, `target_component`, then the eight-word extension block
/// (chan9..chan16; chan17/18 stay ungenerated).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcChannelsOverride {
    /// System ID.
    pub target_system: u8,
    /// Component ID.
    pub target_component: u8,
    /// PWM microseconds for chan1..chan16.
    pub chan: [u16; OVERRIDE_CHANNEL_COUNT],
}

impl RcChannelsOverride {
    /// Build an override from the XML field order (`target_*` then chan1..16).
    #[must_use]
    pub const fn new(
        target_system: u8,
        target_component: u8,
        chan: [u16; OVERRIDE_CHANNEL_COUNT],
    ) -> Self {
        Self {
            target_system,
            target_component,
            chan,
        }
    }

    /// Pack into 38 little-endian bytes. `None` if `buf` is too short.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..RC_CHANNELS_OVERRIDE_LEN)?;
        write_u16_array(dest.get_mut(..16)?, self.chan.get(..8)?)?;
        *dest.get_mut(16)? = self.target_system;
        *dest.get_mut(17)? = self.target_component;
        write_u16_array(dest.get_mut(18..34)?, self.chan.get(8..)?)?;
        dest.get_mut(34..38)?.fill(0);
        Some(RC_CHANNELS_OVERRIDE_LEN)
    }

    /// Unpack at least 18 bytes. Extension channels default to 0 when the
    /// buffer is shorter than [`RC_CHANNELS_OVERRIDE_LEN`].
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..RC_CHANNELS_OVERRIDE_MIN_LEN)?;
        let mut chan = [0u16; OVERRIDE_CHANNEL_COUNT];
        let base = read_u16_array::<8>(src.get(..16)?)?;
        let mut i = 0usize;
        while i < 8 {
            *chan.get_mut(i)? = *base.get(i)?;
            i = i.checked_add(1)?;
        }
        let ext = read_u16_array_or_zero::<8>(buf, 18);
        i = 0;
        while i < 8 {
            *chan.get_mut(i.checked_add(8)?)? = *ext.get(i)?;
            i = i.checked_add(1)?;
        }
        Some(Self {
            target_system: *src.get(16)?,
            target_component: *src.get(17)?,
            chan,
        })
    }

    /// Decode a framed RC_CHANNELS_OVERRIDE. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_RC_CHANNELS_OVERRIDE {
            return None;
        }
        Self::decode(frame.payload())
    }
}

/// Packed MANUAL_CONTROL fields, upstream `mavlink_manual_control_t`.
///
/// Wire order is size-sorted for the base message
/// (`mavlink_msg_manual_control_pack`): `x`, `y`, `z`, `r`, `buttons`,
/// then `target`. Extension axes (`s`/`t`/aux) stay for later slices.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ManualControl {
    /// Pitch-axis stick, −1000..1000. [`MANUAL_AXIS_INVALID`] if unused.
    pub x: i16,
    /// Roll-axis stick, −1000..1000. [`MANUAL_AXIS_INVALID`] if unused.
    pub y: i16,
    /// Throttle-axis stick, −1000..1000. [`MANUAL_AXIS_INVALID`] if unused.
    pub z: i16,
    /// Yaw-axis stick, −1000..1000. [`MANUAL_AXIS_INVALID`] if unused.
    pub r: i16,
    /// Joystick button bitfield.
    pub buttons: u16,
    /// System to be controlled.
    pub target: u8,
}

impl ManualControl {
    /// Build a MANUAL_CONTROL from the XML field order.
    #[must_use]
    pub const fn new(target: u8, x: i16, y: i16, z: i16, r: i16, buttons: u16) -> Self {
        Self {
            x,
            y,
            z,
            r,
            buttons,
            target,
        }
    }

    /// Pack the 11-byte base payload. `None` if `buf` is too short.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..MANUAL_CONTROL_MIN_LEN)?;
        dest.get_mut(..2)?.copy_from_slice(&self.x.to_le_bytes());
        dest.get_mut(2..4)?.copy_from_slice(&self.y.to_le_bytes());
        dest.get_mut(4..6)?.copy_from_slice(&self.z.to_le_bytes());
        dest.get_mut(6..8)?.copy_from_slice(&self.r.to_le_bytes());
        dest.get_mut(8..10)?
            .copy_from_slice(&self.buttons.to_le_bytes());
        *dest.get_mut(10)? = self.target;
        Some(MANUAL_CONTROL_MIN_LEN)
    }

    /// Unpack at least 11 bytes. Extension fields are ignored.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..MANUAL_CONTROL_MIN_LEN)?;
        Some(Self {
            x: i16::from_le_bytes(src.get(..2)?.try_into().ok()?),
            y: i16::from_le_bytes(src.get(2..4)?.try_into().ok()?),
            z: i16::from_le_bytes(src.get(4..6)?.try_into().ok()?),
            r: i16::from_le_bytes(src.get(6..8)?.try_into().ok()?),
            buttons: u16::from_le_bytes(src.get(8..10)?.try_into().ok()?),
            target: *src.get(10)?,
        })
    }

    /// Decode a framed MANUAL_CONTROL. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_MANUAL_CONTROL {
            return None;
        }
        Self::decode(frame.payload())
    }
}

/// In-memory GCS override table, upstream `RC_Channels::set_override`.
///
/// `active[i]` is `true` when channel `i` currently holds a PWM override.
/// A stored PWM of `0` with `active` means "released back to RC radio"
/// (upstream `set_override(i, 0, tnow)`).
#[derive(Debug, Clone)]
pub struct OverrideStore {
    chan: [u16; OVERRIDE_CHANNEL_COUNT],
    active: [bool; OVERRIDE_CHANNEL_COUNT],
    last_ms: u32,
}

impl Default for OverrideStore {
    fn default() -> Self {
        Self::new()
    }
}

impl OverrideStore {
    /// Empty table — no channel is overridden.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            chan: [0; OVERRIDE_CHANNEL_COUNT],
            active: [false; OVERRIDE_CHANNEL_COUNT],
            last_ms: 0,
        }
    }

    /// Last successful ingest timestamp (`sysid_mygcs_seen` / `tnow`).
    #[must_use]
    pub const fn last_ms(&self) -> u32 {
        self.last_ms
    }

    /// Stored PWM for channel `i` (0-based) when that slot is active.
    #[must_use]
    pub fn get(&self, i: usize) -> Option<u16> {
        if *self.active.get(i)? {
            self.chan.get(i).copied()
        } else {
            None
        }
    }

    /// Apply a decoded RC_CHANNELS_OVERRIDE using the upstream ignore rules.
    ///
    /// Chan1–8: `UINT16_MAX` is ignored; any other value (including 0 =
    /// release) is stored. Chan9–16: `0` and `UINT16_MAX` are ignored;
    /// `UINT16_MAX-1` releases. Returns how many slots were written.
    pub fn apply_rc_channels_override(&mut self, pkt: &RcChannelsOverride, now_ms: u32) -> usize {
        let mut applied = 0usize;
        let mut i = 0usize;
        while i < 8 {
            if let Some(&pwm) = pkt.chan.get(i) {
                if pwm != OVERRIDE_IGNORE && self.set_override(i, pwm) {
                    applied = applied.saturating_add(1);
                }
            }
            i = i.saturating_add(1);
        }
        while i < OVERRIDE_CHANNEL_COUNT {
            if let Some(&pwm) = pkt.chan.get(i) {
                if pwm != 0 && pwm != OVERRIDE_IGNORE {
                    let value = if pwm == OVERRIDE_RELEASE_EXT { 0 } else { pwm };
                    if self.set_override(i, value) {
                        applied = applied.saturating_add(1);
                    }
                }
            }
            i = i.saturating_add(1);
        }
        self.last_ms = now_ms;
        applied
    }

    /// Map Plane `handle_manual_control_axes` onto roll/pitch/throttle/rudder.
    ///
    /// `y` → ch1 (roll), `x` → ch2 (pitch, reversed), `z` → ch3 (throttle),
    /// `r` → ch4 (rudder). `INT16_MAX` writes a release (`0`). Returns how
    /// many of the four axes were written.
    pub fn apply_manual_control(&mut self, pkt: &ManualControl, now_ms: u32) -> usize {
        let axes = [
            map_manual_axis(pkt.y, 1_000, 2_000, false),
            map_manual_axis(pkt.x, 1_000, 2_000, true),
            map_manual_axis(pkt.z, 0, 1_000, false),
            map_manual_axis(pkt.r, 1_000, 2_000, false),
        ];
        let mut applied = 0usize;
        let mut i = 0usize;
        while i < axes.len() {
            if let Some(&pwm) = axes.get(i) {
                if self.set_override(i, pwm) {
                    applied = applied.saturating_add(1);
                }
            }
            i = i.saturating_add(1);
        }
        self.last_ms = now_ms;
        applied
    }

    fn set_override(&mut self, i: usize, pwm: u16) -> bool {
        let Some(slot) = self.chan.get_mut(i) else {
            return false;
        };
        let Some(flag) = self.active.get_mut(i) else {
            return false;
        };
        *slot = pwm;
        *flag = true;
        true
    }
}

/// Plane `GCS_MAVLINK::manual_override` PWM map with default radio 1000..2000.
#[must_use]
pub fn map_manual_axis(value_in: i16, offset: i32, scaler: i32, reversed: bool) -> u16 {
    if value_in == MANUAL_AXIS_INVALID || scaler == 0 {
        return 0;
    }
    let mut value = i32::from(value_in);
    if reversed {
        value = value.saturating_neg();
    }
    let span = i32::from(MANUAL_RADIO_MAX).saturating_sub(i32::from(MANUAL_RADIO_MIN));
    let shifted = value.saturating_add(offset);
    let pwm = i32::from(MANUAL_RADIO_MIN).saturating_add(span.saturating_mul(shifted) / scaler);
    if pwm < 0 {
        0
    } else if pwm > i32::from(u16::MAX) {
        u16::MAX
    } else {
        pwm as u16
    }
}

fn write_u16_array(dest: &mut [u8], values: &[u16]) -> Option<()> {
    if dest.len() < values.len().checked_mul(2)? {
        return None;
    }
    let mut i = 0usize;
    while i < values.len() {
        let off = i.checked_mul(2)?;
        dest.get_mut(off..off.checked_add(2)?)?
            .copy_from_slice(&values.get(i)?.to_le_bytes());
        i = i.checked_add(1)?;
    }
    Some(())
}

fn read_u16_array<const N: usize>(src: &[u8]) -> Option<[u16; N]> {
    let mut out = [0u16; N];
    let mut i = 0usize;
    while i < N {
        let off = i.checked_mul(2)?;
        *out.get_mut(i)? = u16::from_le_bytes(src.get(off..off.checked_add(2)?)?.try_into().ok()?);
        i = i.checked_add(1)?;
    }
    Some(out)
}

fn read_u16_array_or_zero<const N: usize>(buf: &[u8], off: usize) -> [u16; N] {
    let need = N.saturating_mul(2);
    buf.get(off..off.saturating_add(need))
        .and_then(read_u16_array)
        .unwrap_or([0u16; N])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_writes_chan1_first() {
        let mut chan = [OVERRIDE_IGNORE; OVERRIDE_CHANNEL_COUNT];
        if let Some(ch) = chan.get_mut(0) {
            *ch = 0x0201;
        }
        let pkt = RcChannelsOverride::new(1, 1, chan);
        let mut buf = [0u8; RC_CHANNELS_OVERRIDE_LEN];
        assert_eq!(pkt.encode(&mut buf), Some(RC_CHANNELS_OVERRIDE_LEN));
        assert_eq!(buf.get(..2), Some([0x01, 0x02].as_slice()));
        assert_eq!(buf.get(16).copied(), Some(1));
        assert_eq!(buf.get(17).copied(), Some(1));
    }

    #[test]
    fn decode_rejects_short_payload() {
        assert!(RcChannelsOverride::decode(&[0, 1, 2]).is_none());
        assert!(ManualControl::decode(&[0, 1, 2]).is_none());
    }

    #[test]
    fn ignore_does_not_write_base_channel() {
        let mut store = OverrideStore::new();
        let pkt = RcChannelsOverride::new(1, 1, [OVERRIDE_IGNORE; OVERRIDE_CHANNEL_COUNT]);
        assert_eq!(store.apply_rc_channels_override(&pkt, 10), 0);
        assert_eq!(store.get(0), None);
    }
}
