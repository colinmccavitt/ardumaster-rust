//! Integration coverage of the file-backend StartWrite / EndWrite path.
//!
//! Upstream `AP_Logger_File::start_new_log` / `stop_logging` open and
//! close a named log (`_write_filename` + `_write_fd`). This drives
//! that path through [`FileBackend`]'s buffer mock — no POSIX
//! filesystem is required.

use ap_logger::{
    calc_msg_len, write_message, FileBackend, LogBackend, LogStructure, LogValue, LOG_FILE_PATH_MAX,
};

#[test]
fn file_backend_start_write_end_write_named_path() {
    let mut log = FileBackend::<64>::new();
    assert!(!log.logging_started());
    assert!(log.start_write("/APM/LOGS/00000001.BIN"));
    assert_eq!(log.path(), "/APM/LOGS/00000001.BIN");
    assert!(log.logging_started());

    assert!(log.write_block(b"HEAD"));
    assert!(log.write_block(&[0x10, 0x20]));

    log.end_write();
    assert!(!log.logging_started());
    assert_eq!(log.ended_writes(), 1);
    assert_eq!(log.path(), "/APM/LOGS/00000001.BIN");
    assert_eq!(log.recorded(), b"HEAD\x10\x20");
    assert!(!log.write_block(&[0x30]));
}

#[test]
fn file_backend_write_message_after_start_write() {
    let row = LogStructure {
        msg_type: 7,
        msg_len: calc_msg_len("BH").expect("len"),
        name: "IMU",
        format: "BH",
        labels: "I,V",
    };
    let fields = [LogValue::U8(3), LogValue::U16(9)];

    let mut log = FileBackend::<32>::new();
    assert!(!write_message(&mut log, &row, &fields));
    assert!(log.recorded().is_empty());

    assert!(log.start_write("logs/00000003.BIN"));
    assert!(write_message(&mut log, &row, &fields));
    log.end_write();

    assert_eq!(log.recorded().len(), usize::from(row.msg_len));
    assert_eq!(log.path(), "logs/00000003.BIN");
    assert!(!log.logging_started());
}

#[test]
fn file_backend_start_write_rejects_overlong_path() {
    let mut log = FileBackend::<8>::new();
    let too_long = [b'x'; LOG_FILE_PATH_MAX + 1];
    let path = core::str::from_utf8(&too_long).expect("ascii");
    assert!(!log.start_write(path));
    assert_eq!(log.path(), "");
}
