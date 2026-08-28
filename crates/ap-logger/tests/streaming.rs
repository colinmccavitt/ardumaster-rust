//! Integration coverage of WriteStreaming / rate-limited periodic write.
//!
//! Upstream `AP_Logger::WriteStreaming` marks the write as streaming;
//! `AP_Logger_RateLimiter::should_log_streaming` then emits only when
//! `1000 / rate_hz` milliseconds have elapsed since the last send of
//! that msgid. A rate of 0 disables the gate.

use ap_logger::{
    calc_msg_len, write_message, LogStructure, LogValue, MemoryBackend, WriteStreaming,
    DEFAULT_STREAM_RATE_HZ,
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

fn att_row() -> LogStructure {
    LogStructure {
        msg_type: 1,
        msg_len: calc_msg_len("BH").expect("len"),
        name: "ATT",
        format: "BH",
        labels: "I,V",
    }
}

#[test]
fn write_emits_only_when_streaming_period_has_elapsed() {
    let row = imu_row();
    let fields = [LogValue::U8(3), LogValue::U16(9)];
    let mut stream = WriteStreaming::with_rate_hz(10);
    let mut log = MemoryBackend::<64>::new();

    assert_eq!(stream.rate_hz(), 10);
    assert_eq!(stream.period_ms(), Some(100));
    assert_eq!(DEFAULT_STREAM_RATE_HZ, 10);

    assert!(stream.write(&mut log, 0, &row, &fields));
    assert_eq!(log.recorded().len(), usize::from(row.msg_len));
    assert_eq!(stream.last_send_ms(row.msg_type), Some(0));

    assert!(!stream.write(&mut log, 99, &row, &fields));
    assert_eq!(log.recorded().len(), usize::from(row.msg_len));

    assert!(stream.write(&mut log, 100, &row, &fields));
    assert_eq!(log.recorded().len(), usize::from(row.msg_len) * 2);
    assert_eq!(stream.last_send_ms(row.msg_type), Some(100));
}

#[test]
fn write_rate_limits_each_msgid_on_its_own_clock() {
    let imu = imu_row();
    let att = att_row();
    let fields = [LogValue::U8(1), LogValue::U16(2)];
    let mut stream = WriteStreaming::with_rate_hz(10);
    let mut log = MemoryBackend::<64>::new();

    assert!(stream.write(&mut log, 0, &imu, &fields));
    assert!(stream.write(&mut log, 20, &att, &fields));
    assert!(!stream.write(&mut log, 50, &imu, &fields));
    assert!(stream.write(&mut log, 120, &att, &fields));
    assert!(stream.write(&mut log, 100, &imu, &fields));
    assert_eq!(log.recorded().len(), usize::from(imu.msg_len) * 4);
}

#[test]
fn zero_rate_matches_ungated_write() {
    let row = imu_row();
    let fields = [LogValue::U8(3), LogValue::U16(9)];
    let mut stream = WriteStreaming::with_rate_hz(0);
    let mut gated = MemoryBackend::<32>::new();
    let mut direct = MemoryBackend::<32>::new();

    assert_eq!(stream.period_ms(), None);
    assert!(stream.write(&mut gated, 0, &row, &fields));
    assert!(stream.write(&mut gated, 0, &row, &fields));
    assert!(write_message(&mut direct, &row, &fields));
    assert!(write_message(&mut direct, &row, &fields));
    assert_eq!(gated.recorded(), direct.recorded());
}
