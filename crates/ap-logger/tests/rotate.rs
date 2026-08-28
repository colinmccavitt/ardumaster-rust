//! Integration coverage of log-file rotation / max-files.
//!
//! Upstream `AP_Logger_File` caps onboard logs at `get_max_num_logs()`
//! and unlinks the oldest when `StartNewLog` would overflow. This
//! drives that path through [`LogRotate`] over the file-backend mock
//! catalog. Not POSIX unlink.

use ap_logger::{
    FileBackend, LogBackend, LogEntry, LogRequestList, LogRotate, LogTransfer,
    DEFAULT_MAX_LOG_FILES, MAX_LOGS,
};

#[test]
fn rotate_caps_mock_catalog_and_drops_oldest() {
    let mut rot = LogRotate::<8>::with_max_files(2);
    assert_eq!(rot.max_files(), 2);
    assert_eq!(rot.num_logs(), 0);
    assert_eq!(rot.last_log_id(), 0);

    assert!(rot.start_write("/APM/LOGS/00000001.BIN"));
    assert!(rot.write_block(b"ONE"));
    rot.end_write();
    assert!(rot.start_write("/APM/LOGS/00000002.BIN"));
    assert!(rot.write_block(b"TWO!"));
    rot.end_write();
    assert_eq!(rot.num_logs(), 2);
    assert_eq!(rot.log_id_at(0), Some(1));
    assert_eq!(rot.log_id_at(1), Some(2));

    assert!(rot.start_write("/APM/LOGS/00000005.BIN"));
    assert!(rot.write_block(b"FIVE"));
    rot.end_write();

    assert_eq!(rot.num_logs(), 2);
    assert_eq!(rot.log_id_at(0), Some(2));
    assert_eq!(rot.log_id_at(1), Some(5));
    assert_eq!(rot.last_log_id(), 5);
    assert_eq!(rot.file().path(), "/APM/LOGS/00000005.BIN");
    assert_eq!(rot.file().recorded(), b"FIVE");

    let mut entries = [LogEntry::default(); 4];
    let n = rot.handle_log_request_list(
        LogRequestList {
            start: 0,
            end: 0xffff,
            target_system: 1,
            target_component: 1,
        },
        &mut entries,
    );
    assert_eq!(n, 2);
    assert_eq!(entries[0].num_logs, 2);
    assert_eq!(entries[0].id, 1);
    assert_eq!(entries[1].id, 2);
    assert_eq!(entries[0].size, 4);
    assert_eq!(entries[1].size, 4);

    rot.set_max_files(1);
    assert_eq!(rot.num_logs(), 1);
    assert_eq!(rot.log_id_at(0), Some(5));
    assert_eq!(rot.last_log_id(), 5);

    assert_eq!(DEFAULT_MAX_LOG_FILES, 15);
    assert!(u16::try_from(MAX_LOGS).is_ok());

    let mut xfer = LogTransfer::<8>::new();
    assert!(xfer.start_write("/APM/LOGS/00000001.BIN"));
    xfer.end_write();
    assert!(xfer.start_write("/APM/LOGS/00000002.BIN"));
    xfer.end_write();
    assert_eq!(xfer.drop_oldest(), Some(1));
    assert_eq!(xfer.num_logs(), 1);
    assert_eq!(xfer.log_id_at(0), Some(2));
    assert_eq!(xfer.last_log_id(), 2);

    let mut log = FileBackend::<8>::new();
    assert!(log.start_write("/APM/LOGS/00000005.BIN"));
    assert!(log.write_block(b"DF"));
    log.end_write();
    assert_eq!(ap_logger::log_id_from_path(log.path()), Some(5));
    assert!(!LogBackend::write_block(&mut log, b"X"));
}

#[test]
fn rotate_default_cap_is_inside_mock_table() {
    let rot = LogRotate::<8>::new();
    assert_eq!(rot.max_files(), DEFAULT_MAX_LOG_FILES);
    assert!(rot.max_files() <= u16::try_from(MAX_LOGS).expect("fits"));
}
