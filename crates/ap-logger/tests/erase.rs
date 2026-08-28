//! Integration coverage of log erase / MAVLink `LOG_ERASE`.
//!
//! Upstream `AP_Logger::handle_log_request_erase` calls `EraseAll()`,
//! which `AP_Logger_File` implements as `stop_logging` then a catalog
//! wipe. This drives that path through [`LogErase`] over the
//! file-backend mock — listing count, last-log id, and the drop
//! counter all reset. Not POSIX unlink.

use ap_logger::{
    write_message, DroppedMessages, FileBackend, LogBackend, LogEntry, LogErase, LogEraseRequest,
    LogRequestList, LogStructure, LogTransfer, LogValue, LOG_ERASE_LEN, MSG_ID_LOG_ERASE,
};

#[test]
fn log_erase_clears_file_backend_catalog_last_id_and_drops() {
    let mut erase = LogErase::<6>::new();
    assert_eq!(erase.num_logs(), 0);
    assert_eq!(erase.last_log_id(), 0);
    assert_eq!(erase.num_dropped(), 0);
    assert_eq!(erase.file().path(), "");

    assert!(erase.start_write("/APM/LOGS/00000001.BIN"));
    assert!(erase.write_block(b"HEAD"));
    erase.end_write();

    assert!(erase.start_write("/APM/LOGS/00000004.BIN"));
    let row = LogStructure {
        msg_type: 7,
        msg_len: 6,
        name: "IMU",
        format: "BH",
        labels: "I,V",
    };
    let fields = [LogValue::U8(3), LogValue::U16(9)];
    assert!(write_message(&mut erase, &row, &fields));
    // StartWrite truncates the 6-byte mock; a second 6-byte row does not fit.
    assert!(!write_message(&mut erase, &row, &fields));
    erase.end_write();

    assert_eq!(erase.num_logs(), 2);
    assert_eq!(erase.last_log_id(), 4);
    assert_eq!(erase.num_dropped(), 1);
    assert_eq!(erase.file().path(), "/APM/LOGS/00000004.BIN");
    assert_eq!(erase.file().recorded().len(), usize::from(row.msg_len));

    let mut entries = [LogEntry::default(); 4];
    let n = erase.handle_log_request_list(
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

    erase.handle_log_erase(LogEraseRequest {
        target_system: 1,
        target_component: 1,
    });

    assert_eq!(erase.num_logs(), 0);
    assert_eq!(erase.last_log_id(), 0);
    assert_eq!(erase.num_dropped(), 0);
    assert_eq!(erase.file().path(), "");
    assert!(!erase.file().logging_started());
    assert!(erase.file().recorded().is_empty());

    let n = erase.handle_log_request_list(
        LogRequestList {
            start: 0,
            end: 0xffff,
            ..LogRequestList::default()
        },
        &mut entries,
    );
    assert_eq!(n, 1);
    assert_eq!(entries[0].id, 0);
    assert_eq!(entries[0].num_logs, 0);
    assert_eq!(entries[0].last_log_num, 0);
    assert_eq!(entries[0].size, 0);

    assert_eq!(MSG_ID_LOG_ERASE, 121);
    let mut buf = [0u8; LOG_ERASE_LEN];
    let req = LogEraseRequest {
        target_system: 1,
        target_component: 1,
    };
    assert_eq!(req.encode(&mut buf), Some(LOG_ERASE_LEN));
    assert_eq!(LogEraseRequest::decode(&buf), Some(req));
}

#[test]
fn erase_all_resets_transfer_catalog_and_dropped_without_posix() {
    let mut xfer = LogTransfer::<8>::new();
    let mut drops = DroppedMessages::new();
    assert!(xfer.start_write("/APM/LOGS/00000009.BIN"));
    assert!(drops.write_block(&mut xfer, b"ABCDEFGH"));
    assert!(!drops.write_block(&mut xfer, b"X"));
    xfer.end_write();
    assert_eq!(xfer.num_logs(), 1);
    assert_eq!(xfer.last_log_id(), 9);
    assert_eq!(drops.num_dropped(), 1);

    // Same wipe the front-end applies: replace the mock catalog and
    // clear `_dropped`. FileBackend itself has no directory walk.
    let mut erase = LogErase::<8>::new();
    assert!(erase.start_write("/APM/LOGS/00000009.BIN"));
    assert!(erase.write_block(b"ABCDEFGH"));
    assert!(!erase.write_block(b"X"));
    erase.end_write();
    erase.erase_all();
    assert_eq!(erase.num_logs(), 0);
    assert_eq!(erase.last_log_id(), 0);
    assert_eq!(erase.num_dropped(), 0);
    assert_eq!(erase.file().path(), "");

    let mut log = FileBackend::<8>::new();
    assert!(log.start_write("/APM/LOGS/00000009.BIN"));
    assert!(log.write_block(b"DF"));
    log.end_write();
    assert_eq!(ap_logger::log_id_from_path(log.path()), Some(9));
    assert!(!LogBackend::write_block(&mut log, b"X"));
}
