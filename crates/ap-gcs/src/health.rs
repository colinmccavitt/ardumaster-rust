//! SYS_STATUS / BATTERY_STATUS stream send, extracted from the pinned
//! Plane-4.7.0 `modules/mavlink/message_definitions/v1.0` defs
//! (`common.xml` msgid 1, msgid 147).
//!
//! Upstream `GCS_MAVLINK::try_send_message` emits SYS_STATUS on
//! `MSG_SYS_STATUS` and BATTERY_STATUS on `MSG_BATTERY_STATUS`. This slice
//! packs both from a small health snapshot (`send_sys_status` /
//! `send_battery_status`) and frames them for Write. Stream-rate
//! scheduling, the extended sensor-flag walk, and the rest of the dialect
//! stay for later FW-028 slices.

use crate::framing::Frame;

/// SYS_STATUS message id, upstream `MAVLINK_MSG_ID_SYS_STATUS`.
pub const MSG_ID_SYS_STATUS: u32 = 1;

/// BATTERY_STATUS message id, upstream `MAVLINK_MSG_ID_BATTERY_STATUS`.
pub const MSG_ID_BATTERY_STATUS: u32 = 147;

/// Packed payload length, upstream `MAVLINK_MSG_ID_SYS_STATUS_LEN`.
pub const SYS_STATUS_LEN: usize = 43;

/// Minimum payload length, upstream `MAVLINK_MSG_ID_SYS_STATUS_MIN_LEN`.
pub const SYS_STATUS_MIN_LEN: usize = 31;

/// Packed payload length, upstream `MAVLINK_MSG_ID_BATTERY_STATUS_LEN`.
pub const BATTERY_STATUS_LEN: usize = 54;

/// Minimum payload length, upstream `MAVLINK_MSG_ID_BATTERY_STATUS_MIN_LEN`.
pub const BATTERY_STATUS_MIN_LEN: usize = 36;

/// CRC extra, upstream `MAVLINK_MSG_ID_SYS_STATUS_CRC`.
pub const SYS_STATUS_CRC: u8 = 124;

/// CRC extra, upstream `MAVLINK_MSG_ID_BATTERY_STATUS_CRC`.
pub const BATTERY_STATUS_CRC: u8 = 154;

/// Cell-voltage array length, upstream `MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN`.
pub const BATTERY_VOLTAGES_LEN: usize = 10;

/// Extended cell-voltage array length, upstream
/// `MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_EXT_LEN`.
pub const BATTERY_VOLTAGES_EXT_LEN: usize = 4;

/// `MAV_BATTERY_FUNCTION_UNKNOWN`.
pub const MAV_BATTERY_FUNCTION_UNKNOWN: u8 = 0;

/// `MAV_BATTERY_TYPE_UNKNOWN`.
pub const MAV_BATTERY_TYPE_UNKNOWN: u8 = 0;

/// Unknown battery temperature, upstream `INT16_MAX` cdegC.
pub const BATTERY_TEMPERATURE_UNKNOWN: i16 = i16::MAX;

/// Sensor / battery snapshot used by `send_sys_status` and
/// `send_battery_status`.
///
/// Mirrors the packed-unit fields those two upstream senders pull from
/// `get_sensor_status_flags`, `AP_BattMonitor`, and `AP::internalerror()`.
/// This is the on-wire snapshot (mV, cA, %, cdegC), not the SI battery
/// types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HealthSnapshot {
    /// `MAV_SYS_STATUS_SENSOR` present bitmap.
    pub sensors_present: u32,
    /// `MAV_SYS_STATUS_SENSOR` enabled bitmap.
    pub sensors_enabled: u32,
    /// `MAV_SYS_STATUS_SENSOR` health bitmap.
    pub sensors_health: u32,
    /// Mainloop load, d% (`scheduler.load_average() * 1000`).
    pub load: u16,
    /// Pack voltage, millivolts (`battery.gcs_voltage() * 1000`).
    pub voltage_battery: u16,
    /// Pack current, centiamps (`current_amps * 100`). `-1` if unmeasured.
    pub current_battery: i16,
    /// Remaining energy, percent. `-1` if unknown.
    pub battery_remaining: i8,
    /// Communication drop rate, c%.
    pub drop_rate_comm: u16,
    /// Communication drop count (`packet_rx_drop_count`).
    pub errors_comm: u16,
    /// `AP::internalerror().errors()` low 16 bits.
    pub errors_count1: u16,
    /// `AP::internalerror().errors()` high 16 bits.
    pub errors_count2: u16,
    /// Dropped log-message count (`AP::logger().num_dropped()`).
    pub errors_count3: u16,
    /// Internal-error count (`AP::internalerror().count()`).
    pub errors_count4: u16,
    /// Extended present bitmap (`onboard_control_sensors_present_extended`).
    pub sensors_present_ext: u32,
    /// Extended enabled bitmap.
    pub sensors_enabled_ext: u32,
    /// Extended health bitmap.
    pub sensors_health_ext: u32,
    /// Battery instance id.
    pub battery_id: u8,
    /// `MAV_BATTERY_FUNCTION`.
    pub battery_function: u8,
    /// `MAV_BATTERY_TYPE`.
    pub battery_type: u8,
    /// Battery temperature, cdegC. [`BATTERY_TEMPERATURE_UNKNOWN`] if unknown.
    pub temperature: i16,
    /// Cells 1–10, millivolts. Unused cells are `u16::MAX`.
    pub voltages: [u16; BATTERY_VOLTAGES_LEN],
    /// Consumed charge, mAh. `-1` if unknown.
    pub current_consumed: i32,
    /// Consumed energy, hJ. `-1` if unknown.
    pub energy_consumed: i32,
    /// Remaining time, seconds. `0` if unknown.
    pub time_remaining: i32,
    /// `MAV_BATTERY_CHARGE_STATE`.
    pub charge_state: u8,
    /// Cells 11–14, millivolts. Unsupported cells are `0`.
    pub voltages_ext: [u16; BATTERY_VOLTAGES_EXT_LEN],
    /// `MAV_BATTERY_MODE`. `0` if unreported.
    pub battery_mode: u8,
    /// `MAV_BATTERY_FAULT` bitmask.
    pub fault_bitmask: u32,
}

impl HealthSnapshot {
    /// Build the SYS_STATUS payload from this snapshot.
    #[must_use]
    pub const fn sys_status(&self) -> SysStatus {
        SysStatus {
            sensors_present: self.sensors_present,
            sensors_enabled: self.sensors_enabled,
            sensors_health: self.sensors_health,
            load: self.load,
            voltage_battery: self.voltage_battery,
            current_battery: self.current_battery,
            battery_remaining: self.battery_remaining,
            drop_rate_comm: self.drop_rate_comm,
            errors_comm: self.errors_comm,
            errors_count1: self.errors_count1,
            errors_count2: self.errors_count2,
            errors_count3: self.errors_count3,
            errors_count4: self.errors_count4,
            sensors_present_ext: self.sensors_present_ext,
            sensors_enabled_ext: self.sensors_enabled_ext,
            sensors_health_ext: self.sensors_health_ext,
        }
    }

    /// Build the BATTERY_STATUS payload from this snapshot.
    #[must_use]
    pub const fn battery_status(&self) -> BatteryStatus {
        BatteryStatus {
            current_consumed: self.current_consumed,
            energy_consumed: self.energy_consumed,
            temperature: self.temperature,
            voltages: self.voltages,
            current_battery: self.current_battery,
            id: self.battery_id,
            battery_function: self.battery_function,
            battery_type: self.battery_type,
            battery_remaining: self.battery_remaining,
            time_remaining: self.time_remaining,
            charge_state: self.charge_state,
            voltages_ext: self.voltages_ext,
            mode: self.battery_mode,
            fault_bitmask: self.fault_bitmask,
        }
    }
}

/// Packed SYS_STATUS fields, upstream `mavlink_sys_status_t`.
///
/// Wire order matches `mavlink_msg_sys_status_pack`: three 4-byte sensor
/// bitmaps, nine 2-byte counters, `battery_remaining`, then the three
/// extended bitmaps.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SysStatus {
    /// Onboard controllers / sensors present.
    pub sensors_present: u32,
    /// Onboard controllers / sensors enabled.
    pub sensors_enabled: u32,
    /// Onboard controllers / sensors healthy.
    pub sensors_health: u32,
    /// Mainloop load, d%.
    pub load: u16,
    /// Battery voltage, millivolts.
    pub voltage_battery: u16,
    /// Battery current, centiamps. `-1` if unmeasured.
    pub current_battery: i16,
    /// Remaining energy, percent. `-1` if unknown.
    pub battery_remaining: i8,
    /// Communication drop rate, c%.
    pub drop_rate_comm: u16,
    /// Communication errors (dropped packets).
    pub errors_comm: u16,
    /// Autopilot-specific errors (low 16 bits of `internalerror`).
    pub errors_count1: u16,
    /// Autopilot-specific errors (high 16 bits of `internalerror`).
    pub errors_count2: u16,
    /// Autopilot-specific errors (dropped log messages).
    pub errors_count3: u16,
    /// Autopilot-specific errors (`internalerror` count).
    pub errors_count4: u16,
    /// Extended present bitmap.
    pub sensors_present_ext: u32,
    /// Extended enabled bitmap.
    pub sensors_enabled_ext: u32,
    /// Extended health bitmap.
    pub sensors_health_ext: u32,
}

impl SysStatus {
    /// Pack into 43 little-endian bytes. `None` if `buf` is too short.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..SYS_STATUS_LEN)?;
        dest.get_mut(..4)?
            .copy_from_slice(&self.sensors_present.to_le_bytes());
        dest.get_mut(4..8)?
            .copy_from_slice(&self.sensors_enabled.to_le_bytes());
        dest.get_mut(8..12)?
            .copy_from_slice(&self.sensors_health.to_le_bytes());
        dest.get_mut(12..14)?
            .copy_from_slice(&self.load.to_le_bytes());
        dest.get_mut(14..16)?
            .copy_from_slice(&self.voltage_battery.to_le_bytes());
        dest.get_mut(16..18)?
            .copy_from_slice(&self.current_battery.to_le_bytes());
        dest.get_mut(18..20)?
            .copy_from_slice(&self.drop_rate_comm.to_le_bytes());
        dest.get_mut(20..22)?
            .copy_from_slice(&self.errors_comm.to_le_bytes());
        dest.get_mut(22..24)?
            .copy_from_slice(&self.errors_count1.to_le_bytes());
        dest.get_mut(24..26)?
            .copy_from_slice(&self.errors_count2.to_le_bytes());
        dest.get_mut(26..28)?
            .copy_from_slice(&self.errors_count3.to_le_bytes());
        dest.get_mut(28..30)?
            .copy_from_slice(&self.errors_count4.to_le_bytes());
        dest.get_mut(30..31)?
            .copy_from_slice(&self.battery_remaining.to_le_bytes());
        dest.get_mut(31..35)?
            .copy_from_slice(&self.sensors_present_ext.to_le_bytes());
        dest.get_mut(35..39)?
            .copy_from_slice(&self.sensors_enabled_ext.to_le_bytes());
        dest.get_mut(39..43)?
            .copy_from_slice(&self.sensors_health_ext.to_le_bytes());
        Some(SYS_STATUS_LEN)
    }

    /// Unpack at least 31 bytes. Extension bitmaps default to 0 when
    /// the buffer is shorter than [`SYS_STATUS_LEN`].
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..SYS_STATUS_MIN_LEN)?;
        Some(Self {
            sensors_present: u32::from_le_bytes(src.get(..4)?.try_into().ok()?),
            sensors_enabled: u32::from_le_bytes(src.get(4..8)?.try_into().ok()?),
            sensors_health: u32::from_le_bytes(src.get(8..12)?.try_into().ok()?),
            load: u16::from_le_bytes(src.get(12..14)?.try_into().ok()?),
            voltage_battery: u16::from_le_bytes(src.get(14..16)?.try_into().ok()?),
            current_battery: i16::from_le_bytes(src.get(16..18)?.try_into().ok()?),
            drop_rate_comm: u16::from_le_bytes(src.get(18..20)?.try_into().ok()?),
            errors_comm: u16::from_le_bytes(src.get(20..22)?.try_into().ok()?),
            errors_count1: u16::from_le_bytes(src.get(22..24)?.try_into().ok()?),
            errors_count2: u16::from_le_bytes(src.get(24..26)?.try_into().ok()?),
            errors_count3: u16::from_le_bytes(src.get(26..28)?.try_into().ok()?),
            errors_count4: u16::from_le_bytes(src.get(28..30)?.try_into().ok()?),
            battery_remaining: i8::from_le_bytes(src.get(30..31)?.try_into().ok()?),
            sensors_present_ext: read_u32_or_zero(buf, 31),
            sensors_enabled_ext: read_u32_or_zero(buf, 35),
            sensors_health_ext: read_u32_or_zero(buf, 39),
        })
    }

    /// Decode a framed SYS_STATUS. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_SYS_STATUS {
            return None;
        }
        Self::decode(frame.payload())
    }
}

/// Packed BATTERY_STATUS fields, upstream `mavlink_battery_status_t`.
///
/// Wire order is size-sorted in the base message
/// (`mavlink_msg_battery_status_pack`): two 4-byte consumed totals,
/// temperature, ten cell voltages, current, then the four 1-byte ids,
/// then the extension block in XML order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BatteryStatus {
    /// Consumed charge, mAh. `-1` if unknown.
    pub current_consumed: i32,
    /// Consumed energy, hJ. `-1` if unknown.
    pub energy_consumed: i32,
    /// Temperature, cdegC. [`BATTERY_TEMPERATURE_UNKNOWN`] if unknown.
    pub temperature: i16,
    /// Cells 1–10, millivolts.
    pub voltages: [u16; BATTERY_VOLTAGES_LEN],
    /// Battery current, centiamps. `-1` if unmeasured.
    pub current_battery: i16,
    /// Battery instance id.
    pub id: u8,
    /// `MAV_BATTERY_FUNCTION`.
    pub battery_function: u8,
    /// `MAV_BATTERY_TYPE`.
    pub battery_type: u8,
    /// Remaining energy, percent. `-1` if unknown.
    pub battery_remaining: i8,
    /// Remaining time, seconds. `0` if unknown.
    pub time_remaining: i32,
    /// `MAV_BATTERY_CHARGE_STATE`.
    pub charge_state: u8,
    /// Cells 11–14, millivolts.
    pub voltages_ext: [u16; BATTERY_VOLTAGES_EXT_LEN],
    /// `MAV_BATTERY_MODE`.
    pub mode: u8,
    /// `MAV_BATTERY_FAULT` bitmask.
    pub fault_bitmask: u32,
}

impl BatteryStatus {
    /// Pack into 54 little-endian bytes. `None` if `buf` is too short.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..BATTERY_STATUS_LEN)?;
        dest.get_mut(..4)?
            .copy_from_slice(&self.current_consumed.to_le_bytes());
        dest.get_mut(4..8)?
            .copy_from_slice(&self.energy_consumed.to_le_bytes());
        dest.get_mut(8..10)?
            .copy_from_slice(&self.temperature.to_le_bytes());
        write_u16_array(dest.get_mut(10..30)?, &self.voltages)?;
        dest.get_mut(30..32)?
            .copy_from_slice(&self.current_battery.to_le_bytes());
        *dest.get_mut(32)? = self.id;
        *dest.get_mut(33)? = self.battery_function;
        *dest.get_mut(34)? = self.battery_type;
        dest.get_mut(35..36)?
            .copy_from_slice(&self.battery_remaining.to_le_bytes());
        dest.get_mut(36..40)?
            .copy_from_slice(&self.time_remaining.to_le_bytes());
        *dest.get_mut(40)? = self.charge_state;
        write_u16_array(dest.get_mut(41..49)?, &self.voltages_ext)?;
        *dest.get_mut(49)? = self.mode;
        dest.get_mut(50..54)?
            .copy_from_slice(&self.fault_bitmask.to_le_bytes());
        Some(BATTERY_STATUS_LEN)
    }

    /// Unpack at least 36 bytes. Extension fields default to 0 when the
    /// buffer is shorter than [`BATTERY_STATUS_LEN`].
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..BATTERY_STATUS_MIN_LEN)?;
        Some(Self {
            current_consumed: i32::from_le_bytes(src.get(..4)?.try_into().ok()?),
            energy_consumed: i32::from_le_bytes(src.get(4..8)?.try_into().ok()?),
            temperature: i16::from_le_bytes(src.get(8..10)?.try_into().ok()?),
            voltages: read_u16_array(src.get(10..30)?)?,
            current_battery: i16::from_le_bytes(src.get(30..32)?.try_into().ok()?),
            id: *src.get(32)?,
            battery_function: *src.get(33)?,
            battery_type: *src.get(34)?,
            battery_remaining: i8::from_le_bytes(src.get(35..36)?.try_into().ok()?),
            time_remaining: read_i32_or_zero(buf, 36),
            charge_state: buf.get(40).copied().unwrap_or(0),
            voltages_ext: read_u16_array_or_zero(buf, 41),
            mode: buf.get(49).copied().unwrap_or(0),
            fault_bitmask: read_u32_or_zero(buf, 50),
        })
    }

    /// Decode a framed BATTERY_STATUS. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_BATTERY_STATUS {
            return None;
        }
        Self::decode(frame.payload())
    }
}

fn read_u32_or_zero(buf: &[u8], off: usize) -> u32 {
    buf.get(off..off.saturating_add(4))
        .and_then(|b| b.try_into().ok())
        .map(u32::from_le_bytes)
        .unwrap_or(0)
}

fn read_i32_or_zero(buf: &[u8], off: usize) -> i32 {
    buf.get(off..off.saturating_add(4))
        .and_then(|b| b.try_into().ok())
        .map(i32::from_le_bytes)
        .unwrap_or(0)
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
    fn encode_writes_sensor_present_first() {
        let sys = SysStatus {
            sensors_present: 0x0102_0304,
            sensors_enabled: 0,
            sensors_health: 0,
            load: 0,
            voltage_battery: 0,
            current_battery: 0,
            battery_remaining: 0,
            drop_rate_comm: 0,
            errors_comm: 0,
            errors_count1: 0,
            errors_count2: 0,
            errors_count3: 0,
            errors_count4: 0,
            sensors_present_ext: 0,
            sensors_enabled_ext: 0,
            sensors_health_ext: 0,
        };
        let mut buf = [0u8; SYS_STATUS_LEN];
        assert_eq!(sys.encode(&mut buf), Some(SYS_STATUS_LEN));
        assert_eq!(buf.get(..4), Some([0x04, 0x03, 0x02, 0x01].as_slice()));
    }

    #[test]
    fn decode_rejects_short_payload() {
        assert!(SysStatus::decode(&[0, 1, 2]).is_none());
        assert!(BatteryStatus::decode(&[0, 1, 2]).is_none());
    }
}
