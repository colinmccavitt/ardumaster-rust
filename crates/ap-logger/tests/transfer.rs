//! Integration coverage of MAVLink `LOG_REQUEST_LIST` listing.
//!
//! Upstream `AP_Logger::handle_log_request_list` answers a GCS catalog
//! request from `get_num_logs` / `find_last_log` on `AP_Logger_File`.
//! This drives that path through [`LogTransfer`] over the file-backend
//! mock — no POSIX directory walk.

use ap_logger::{
    write_message, FileBackend, LogBackend, LogEntry, LogRequestList, LogStructure, LogTransfer,
    LogValue, LOG_ENTRY_LEN, MSG_ID_LOG_ENTRY, MSG_ID_LOG_REQUEST_LIST,
};

#[test]
fn log_request_list_reports_count_and_last_id_from_file_backend() {
    let mut xfer = LogTransfer::<64>::new();
    assert_eq!(xfer.num_logs(), 0);
    assert_eq!(xfer.last_log_id(), 0);
    assert_eq!(xfer.file().path(), "");

    let mut entries = [LogEntry::default(); 4];
    let n = xfer.handle_log_request_list(
        LogRequestList {
            start: 0,
            end: 0xffff,
            target_system: 1,
            target_component: 1,
        },
        &mut entries,
    );
    assert_eq!(n, 1);
    assert_eq!(entries[0].id, 0);
    assert_eq!(entries[0].num_logs, 0);
    assert_eq!(entries[0].last_log_num, 0);

    assert!(xfer.start_write("/APM/LOGS/00000001.BIN"));
    assert!(xfer.write_block(b"HEAD"));
    xfer.end_write();
    assert!(!xfer.file().logging_started());
    assert_eq!(xfer.file().path(), "/APM/LOGS/00000001.BIN");
    assert_eq!(xfer.file().recorded(), b"HEAD");

    assert!(xfer.start_write("/APM/LOGS/00000002.BIN"));
    let row = LogStructure {
        msg_type: 7,
        msg_len: 6,
        name: "IMU",
        format: "BH",
        labels: "I,V",
    };
    let fields = [LogValue::U8(3), LogValue::U16(9)];
    assert!(write_message(&mut xfer, &row, &fields));
    xfer.end_write();

    assert_eq!(xfer.num_logs(), 2);
    assert_eq!(xfer.last_log_id(), 2);
    assert_eq!(xfer.file().path(), "/APM/LOGS/00000002.BIN");
    assert_eq!(xfer.file().recorded().len(), usize::from(row.msg_len));

    let n = xfer.handle_log_request_list(
        LogRequestList {
            start: 0,
            end: 0xffff,
            ..LogRequestList::default()
        },
        &mut entries,
    );
    assert_eq!(n, 2);
    assert_eq!(entries[0].id, 1);
    assert_eq!(entries[0].num_logs, 2);
    assert_eq!(entries[0].size, 4);
    assert_eq!(entries[0].last_log_num, 2);
    assert_eq!(entries[1].id, 2);
    assert_eq!(entries[1].num_logs, 2);
    assert_eq!(entries[1].size, u32::from(row.msg_len));
    assert_eq!(entries[1].last_log_num, 2);

    assert_eq!(MSG_ID_LOG_REQUEST_LIST, 117);
    assert_eq!(MSG_ID_LOG_ENTRY, 118);
    let mut buf = [0u8; LOG_ENTRY_LEN];
    assert_eq!(entries[1].encode(&mut buf), Some(LOG_ENTRY_LEN));
    assert_eq!(LogEntry::decode(&buf), Some(entries[1]));
}

#[test]
fn last_log_id_comes_from_file_backend_path_not_list_index() {
    let mut xfer = LogTransfer::<16>::new();
    assert!(xfer.start_write("/APM/LOGS/00000007.BIN"));
    assert!(xfer.write_block(&[0xA3, 0x95]));
    xfer.end_write();

    assert_eq!(xfer.num_logs(), 1);
    assert_eq!(xfer.last_log_id(), 7);
    assert_eq!(xfer.file().path(), "/APM/LOGS/00000007.BIN");
    assert_eq!(xfer.file().recorded(), &[0xA3, 0x95]);

    let mut entries = [LogEntry::default(); 2];
    let n = xfer.handle_log_request_list(
        LogRequestList {
            start: 0,
            end: 0xffff,
            ..LogRequestList::default()
        },
        &mut entries,
    );
    assert_eq!(n, 1);
    assert_eq!(entries[0].id, 1);
    assert_eq!(entries[0].num_logs, 1);
    assert_eq!(entries[0].size, 2);
}

#[test]
fn file_backend_alone_still_exposes_path_for_listing() {
    let mut log = FileBackend::<16>::new();
    assert!(log.start_write("/APM/LOGS/00000005.BIN"));
    assert!(log.write_block(b"DF"));
    log.end_write();
    assert_eq!(ap_logger::log_id_from_path(log.path()), Some(5));
    assert_eq!(log.recorded().len(), 2);
}
