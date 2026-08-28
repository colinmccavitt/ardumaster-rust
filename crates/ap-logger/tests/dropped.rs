//! Integration coverage of the dropped-message / buffer-full counter.
//!
//! Upstream `AP_Logger_Backend::_dropped` increments when
//! `WritePrioritisedBlock` cannot fit a message (`space < size`).
//! `num_dropped()` exposes the count. Packing failures are not drops.

use ap_logger::{
    calc_msg_len, DroppedMessages, FileBackend, LogBackend, LogStructure, LogValue, MemoryBackend,
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
fn write_increments_dropped_when_memory_backend_is_full() {
    let row = imu_row();
    let fields = [LogValue::U8(3), LogValue::U16(9)];
    assert_eq!(row.msg_len, 6);

    let mut drops = DroppedMessages::new();
    let mut log = MemoryBackend::<6>::new();
    log.start_write(1);

    assert_eq!(drops.num_dropped(), 0);
    assert!(drops.write(&mut log, &row, &fields));
    assert_eq!(drops.num_dropped(), 0);
    assert_eq!(log.recorded().len(), 6);

    assert!(!drops.write(&mut log, &row, &fields));
    assert_eq!(drops.num_dropped(), 1);
    assert_eq!(log.recorded().len(), 6);

    assert!(!drops.write(&mut log, &row, &fields));
    assert_eq!(drops.num_dropped(), 2);
    log.end_write();
}

#[test]
fn write_increments_dropped_when_file_backend_buffer_is_full() {
    let row = imu_row();
    let fields = [LogValue::U8(1), LogValue::U16(2)];

    let mut drops = DroppedMessages::new();
    let mut log = FileBackend::<6>::new();
    assert!(log.start_write("/APM/LOGS/00000001.BIN"));

    assert!(drops.write(&mut log, &row, &fields));
    assert_eq!(drops.num_dropped(), 0);
    assert!(!drops.write(&mut log, &row, &fields));
    assert_eq!(drops.num_dropped(), 1);
    assert_eq!(log.recorded().len(), 6);

    log.end_write();
    assert_eq!(log.ended_writes(), 1);
}

#[test]
fn packing_mismatch_does_not_increment_dropped() {
    let row = imu_row();
    let fields = [LogValue::I16(1), LogValue::U16(2)];
    let mut drops = DroppedMessages::new();
    let mut log = MemoryBackend::<32>::new();
    assert!(!drops.write(&mut log, &row, &fields));
    assert_eq!(drops.num_dropped(), 0);
    assert!(log.recorded().is_empty());
}
