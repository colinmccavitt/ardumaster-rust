//! Integration coverage of MAVLink `LOG_REQUEST_DATA` / `LOG_DATA` replay.
//!
//! Upstream `AP_Logger::handle_log_request_data` answers a GCS download
//! from `get_log_data` on `AP_Logger_File`. This drives that path through
//! [`LogReplay`] over the file-backend mock — recorded bytes come back as
//! `LOG_DATA` chunks of [`LOG_DATA_CHUNK_LEN`]. Listing stays in
//! `tests/transfer.rs`.

use ap_logger::{
    write_message, FileBackend, LogBackend, LogData, LogReplay, LogRequestData, LogStructure,
    LogValue, LOG_DATA_CHUNK_LEN, LOG_DATA_LEN, LOG_REQUEST_DATA_LEN, MSG_ID_LOG_DATA,
    MSG_ID_LOG_REQUEST_DATA,
};

#[test]
fn log_request_data_replays_file_backend_recorded_bytes() {
    let mut replay = LogReplay::<256>::new();
    assert_eq!(replay.num_logs(), 0);
    assert_eq!(replay.last_log_id(), 0);
    assert_eq!(replay.file().path(), "");

    let mut chunks = [LogData::default(); 4];
    let n = replay.handle_log_request_data(
        LogRequestData {
            ofs: 0,
            count: u32::MAX,
            id: 1,
            target_system: 1,
            target_component: 1,
        },
        &mut chunks,
    );
    assert_eq!(n, 0);

    assert!(replay.start_write("/APM/LOGS/00000003.BIN"));
    let mut body = [0u8; 200];
    let mut i = 0usize;
    while i < body.len() {
        if let Some(slot) = body.get_mut(i) {
            *slot = (i & 0xff) as u8;
        }
        i = i.saturating_add(1);
    }
    assert!(replay.write_block(&body));
    replay.end_write();

    assert!(!replay.file().logging_started());
    assert_eq!(replay.file().path(), "/APM/LOGS/00000003.BIN");
    assert_eq!(replay.file().recorded(), &body);
    assert_eq!(replay.num_logs(), 1);
    assert_eq!(replay.last_log_id(), 3);

    let n = replay.handle_log_request_data(
        LogRequestData {
            ofs: 0,
            count: u32::MAX,
            id: 1,
            ..LogRequestData::default()
        },
        &mut chunks,
    );
    assert_eq!(n, 3);
    assert_eq!(chunks[0].id, 1);
    assert_eq!(chunks[0].ofs, 0);
    assert_eq!(chunks[0].count, 90);
    assert_eq!(chunks[0].payload(), body.get(..90).unwrap_or(&[]));
    assert_eq!(chunks[1].ofs, 90);
    assert_eq!(chunks[1].count, 90);
    assert_eq!(chunks[1].payload(), body.get(90..180).unwrap_or(&[]));
    assert_eq!(chunks[2].ofs, 180);
    assert_eq!(chunks[2].count, 20);
    assert_eq!(chunks[2].payload(), body.get(180..200).unwrap_or(&[]));

    let mut rebuilt = [0u8; 200];
    let mut off = 0usize;
    let mut c = 0usize;
    while c < n {
        let Some(pkt) = chunks.get(c) else {
            break;
        };
        let pay = pkt.payload();
        if let Some(dst) = rebuilt.get_mut(off..off.saturating_add(pay.len())) {
            dst.copy_from_slice(pay);
        }
        off = off.saturating_add(pay.len());
        c = c.saturating_add(1);
    }
    assert_eq!(&rebuilt, &body);

    assert_eq!(MSG_ID_LOG_REQUEST_DATA, 119);
    assert_eq!(MSG_ID_LOG_DATA, 120);
    assert_eq!(LOG_DATA_CHUNK_LEN, 90);
    let mut req_buf = [0u8; LOG_REQUEST_DATA_LEN];
    let req = LogRequestData {
        ofs: 0,
        count: 200,
        id: 1,
        target_system: 1,
        target_component: 1,
    };
    assert_eq!(req.encode(&mut req_buf), Some(LOG_REQUEST_DATA_LEN));
    assert_eq!(LogRequestData::decode(&req_buf), Some(req));
    let mut data_buf = [0u8; LOG_DATA_LEN];
    assert_eq!(chunks[2].encode(&mut data_buf), Some(LOG_DATA_LEN));
    assert_eq!(LogData::decode(&data_buf), Some(chunks[2]));
}

#[test]
fn log_request_data_serves_write_message_bytes() {
    let mut replay = LogReplay::<64>::new();
    assert!(replay.start_write("/APM/LOGS/00000005.BIN"));
    let row = LogStructure {
        msg_type: 7,
        msg_len: 6,
        name: "IMU",
        format: "BH",
        labels: "I,V",
    };
    let fields = [LogValue::U8(3), LogValue::U16(9)];
    assert!(write_message(&mut replay, &row, &fields));
    replay.end_write();

    let mut recorded = [0u8; 16];
    let rec = replay.file().recorded();
    assert_eq!(rec.len(), usize::from(row.msg_len));
    let Some(dst) = recorded.get_mut(..rec.len()) else {
        panic!("recorded");
    };
    dst.copy_from_slice(rec);
    let rec_len = rec.len();

    let mut chunks = [LogData::default(); 2];
    let n = replay.handle_log_request_data(
        LogRequestData {
            ofs: 0,
            count: u32::MAX,
            id: 1,
            ..LogRequestData::default()
        },
        &mut chunks,
    );
    assert_eq!(n, 1);
    assert_eq!(chunks[0].count, row.msg_len);
    assert_eq!(chunks[0].payload(), recorded.get(..rec_len).unwrap_or(&[]));
    assert_eq!(replay.last_log_id(), 5);
}

#[test]
fn file_backend_alone_still_exposes_recorded_bytes_for_replay() {
    let mut log = FileBackend::<16>::new();
    assert!(log.start_write("/APM/LOGS/00000004.BIN"));
    assert!(log.write_block(b"DF"));
    log.end_write();
    assert_eq!(ap_logger::log_id_from_path(log.path()), Some(4));
    assert_eq!(log.recorded(), b"DF");
    assert!(!LogBackend::write_block(&mut log, b"X"));
}
