//! Integration coverage of the `LOG_BITMASK` / logging-started gate.
//!
//! Upstream `AP_Logger::should_log` enables a message class from the
//! stored bitmask; `logging_started` is the backend latch. `Write` is a
//! no-op when the class is disabled or no log is open.

use ap_logger::{
    calc_msg_len, write_message, LogBackend, LogGate, LogStructure, LogValue, MemoryBackend,
    DEFAULT_LOG_BITMASK, MASK_LOG_GPS, MASK_LOG_IMU,
};

fn imu_row() -> LogStructure {
    LogStructure {
        msg_type: 7,
        msg_len: calc_msg_len("BH").expect("len"),
        name: "IMU",
        format: "BH",
        labels: "I,V",
    }
}

#[test]
fn should_log_follows_log_bitmask_class_bits() {
    let mut gate = LogGate::new(0);
    assert!(!gate.should_log(MASK_LOG_IMU));
    gate.set_log_bitmask(MASK_LOG_IMU);
    assert!(gate.should_log(MASK_LOG_IMU));
    assert!(!gate.should_log(MASK_LOG_GPS));
}

#[test]
fn write_is_noop_until_started_then_records() {
    let mut gate = LogGate::new(DEFAULT_LOG_BITMASK);
    let row = imu_row();
    let fields = [LogValue::U8(3), LogValue::U16(9)];
    let mut log = MemoryBackend::<32>::new();

    assert!(!gate.logging_started());
    assert!(!gate.write(&mut log, MASK_LOG_IMU, &row, &fields));
    assert!(log.recorded().is_empty());

    gate.start_logging();
    assert!(gate.logging_started());
    assert!(gate.write(&mut log, MASK_LOG_IMU, &row, &fields));
    assert_eq!(log.recorded().len(), usize::from(row.msg_len));

    // Ungated Write still packs; the gate is what drops the no-op case.
    let mut direct = MemoryBackend::<32>::new();
    assert!(write_message(&mut direct, &row, &fields));
    assert_eq!(direct.recorded(), log.recorded());
}

#[test]
fn write_is_noop_when_bitmask_disables_class() {
    let mut gate = LogGate::new(MASK_LOG_GPS);
    gate.start_logging();
    let row = imu_row();
    let fields = [LogValue::U8(1), LogValue::U16(2)];
    let mut log = MemoryBackend::<32>::new();
    log.start_write(1);
    assert!(!gate.write(&mut log, MASK_LOG_IMU, &row, &fields));
    log.end_write();
    assert!(log.recorded().is_empty());
    assert_eq!(log.ended_writes(), 1);
}
