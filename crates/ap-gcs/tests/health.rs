//! SYS_STATUS / BATTERY_STATUS stream send from a health snapshot.

use ap_gcs::{
    decode_v2, encode_v2, BatteryStatus, Frame, GcsMavlink, HealthSnapshot, SysStatus,
    BATTERY_STATUS_CRC, BATTERY_STATUS_LEN, BATTERY_TEMPERATURE_UNKNOWN, BATTERY_VOLTAGES_EXT_LEN,
    BATTERY_VOLTAGES_LEN, MAV_BATTERY_FUNCTION_UNKNOWN, MAV_BATTERY_TYPE_UNKNOWN,
    MSG_ID_BATTERY_STATUS, MSG_ID_SYS_STATUS, SYS_STATUS_CRC, SYS_STATUS_LEN,
};

fn sample_health() -> HealthSnapshot {
    let mut voltages = [u16::MAX; BATTERY_VOLTAGES_LEN];
    if let Some(cell0) = voltages.get_mut(0) {
        *cell0 = 12_600;
    }
    HealthSnapshot {
        sensors_present: 0x0000_002F,
        sensors_enabled: 0x0000_0027,
        sensors_health: 0x0000_0023,
        load: 240,
        voltage_battery: 12_600,
        current_battery: 150,
        battery_remaining: 87,
        drop_rate_comm: 0,
        errors_comm: 2,
        errors_count1: 0x0001,
        errors_count2: 0,
        errors_count3: 4,
        errors_count4: 1,
        sensors_present_ext: 0,
        sensors_enabled_ext: 0,
        sensors_health_ext: 0,
        battery_id: 0,
        battery_function: MAV_BATTERY_FUNCTION_UNKNOWN,
        battery_type: MAV_BATTERY_TYPE_UNKNOWN,
        temperature: BATTERY_TEMPERATURE_UNKNOWN,
        voltages,
        current_consumed: 420,
        energy_consumed: 36,
        time_remaining: 1_800,
        charge_state: 1,
        voltages_ext: [0u16; BATTERY_VOLTAGES_EXT_LEN],
        battery_mode: 0,
        fault_bitmask: 0,
    }
}

#[test]
fn sys_status_payload_roundtrip() {
    let health = sample_health();
    let sys = health.sys_status();
    let mut buf = [0u8; SYS_STATUS_LEN];
    assert_eq!(sys.encode(&mut buf), Some(SYS_STATUS_LEN));
    let decoded = SysStatus::decode(&buf).expect("payload");
    assert_eq!(decoded, sys);
    assert_eq!(decoded.sensors_present, 0x0000_002F);
    assert_eq!(decoded.voltage_battery, 12_600);
    assert_eq!(decoded.current_battery, 150);
    assert_eq!(decoded.battery_remaining, 87);
    assert_eq!(decoded.errors_comm, 2);
    assert_eq!(decoded.errors_count3, 4);
}

#[test]
fn battery_status_payload_roundtrip() {
    let health = sample_health();
    let batt = health.battery_status();
    let mut buf = [0u8; BATTERY_STATUS_LEN];
    assert_eq!(batt.encode(&mut buf), Some(BATTERY_STATUS_LEN));
    let decoded = BatteryStatus::decode(&buf).expect("payload");
    assert_eq!(decoded, batt);
    assert_eq!(decoded.id, 0);
    assert_eq!(decoded.current_consumed, 420);
    assert_eq!(decoded.energy_consumed, 36);
    assert_eq!(decoded.time_remaining, 1_800);
    assert_eq!(decoded.voltages.get(0).copied(), Some(12_600));
    assert_eq!(decoded.temperature, BATTERY_TEMPERATURE_UNKNOWN);
}

#[test]
fn health_stream_frames_use_pinned_crc_extras() {
    assert_eq!(SYS_STATUS_CRC, 124);
    assert_eq!(BATTERY_STATUS_CRC, 154);

    let health = sample_health();
    let mut sys_payload = [0u8; SYS_STATUS_LEN];
    assert_eq!(
        health.sys_status().encode(&mut sys_payload),
        Some(SYS_STATUS_LEN)
    );
    let sys_frame = Frame::new(3, 1, 1, MSG_ID_SYS_STATUS, &sys_payload).expect("sys frame");
    let mut sys_wire = [0u8; 64];
    let sn = encode_v2(&sys_frame, &mut sys_wire).expect("encode sys");
    assert_eq!(sn, 10 + SYS_STATUS_LEN + 2);
    let sys_parsed = decode_v2(sys_wire.get(..sn).expect("slice")).expect("decode sys");
    assert_eq!(sys_parsed.msgid, MSG_ID_SYS_STATUS);

    let mut batt_payload = [0u8; BATTERY_STATUS_LEN];
    assert_eq!(
        health.battery_status().encode(&mut batt_payload),
        Some(BATTERY_STATUS_LEN)
    );
    let batt_frame = Frame::new(4, 1, 1, MSG_ID_BATTERY_STATUS, &batt_payload).expect("batt frame");
    let mut batt_wire = [0u8; 80];
    let bn = encode_v2(&batt_frame, &mut batt_wire).expect("encode batt");
    assert_eq!(bn, 10 + BATTERY_STATUS_LEN + 2);
    let batt_parsed = decode_v2(batt_wire.get(..bn).expect("slice")).expect("decode batt");
    assert_eq!(batt_parsed.msgid, MSG_ID_BATTERY_STATUS);
}

#[test]
fn send_sys_status_and_battery_status_from_health_snapshot() {
    let mut gcs = GcsMavlink::new();
    let health = sample_health();

    let mut sys_wire = [0u8; 64];
    let sn = gcs
        .send_sys_status(&mut sys_wire, &health)
        .expect("send sys");
    assert_eq!(sn, 10 + SYS_STATUS_LEN + 2);
    assert_eq!(sys_wire.first().copied(), Some(0xFD));
    let sys_frame = decode_v2(sys_wire.get(..sn).expect("slice")).expect("decode sys");
    assert_eq!(sys_frame.msgid, MSG_ID_SYS_STATUS);
    assert_eq!(sys_frame.sysid, 1);
    assert_eq!(sys_frame.compid, 1);
    assert_eq!(sys_frame.seq, 0);
    let sys = SysStatus::from_frame(&sys_frame).expect("sys");
    assert_eq!(sys, health.sys_status());

    let mut batt_wire = [0u8; 80];
    let bn = gcs
        .send_battery_status(&mut batt_wire, &health)
        .expect("send batt");
    assert_eq!(bn, 10 + BATTERY_STATUS_LEN + 2);
    assert_eq!(batt_wire.first().copied(), Some(0xFD));
    let batt_frame = decode_v2(batt_wire.get(..bn).expect("slice")).expect("decode batt");
    assert_eq!(batt_frame.msgid, MSG_ID_BATTERY_STATUS);
    assert_eq!(batt_frame.sysid, 1);
    assert_eq!(batt_frame.compid, 1);
    assert_eq!(batt_frame.seq, 1);
    let batt = BatteryStatus::from_frame(&batt_frame).expect("batt");
    assert_eq!(batt, health.battery_status());
}

#[test]
fn from_frame_rejects_other_msgid() {
    let frame = Frame::new(0, 1, 1, 0, &[0u8; 9]).expect("heartbeat-shaped");
    assert!(SysStatus::from_frame(&frame).is_none());
    assert!(BatteryStatus::from_frame(&frame).is_none());
}
