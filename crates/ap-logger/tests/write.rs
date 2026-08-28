//! Integration coverage of the Write() typed-message dispatcher.
//!
//! Upstream `AP_Logger::Write` packs a FMT-described payload
//! (`HEAD_BYTE1` / `HEAD_BYTE2` / `msgid` + format fields) and hands
//! it to `WriteBlock`. This drives that path through [`MemoryBackend`].

use ap_logger::{
    calc_msg_len, write_message, LogBackend, LogStructure, LogValue, MemoryBackend, HEAD_BYTE1,
    HEAD_BYTE2, LOG_FORMAT_LEN,
};

#[test]
fn write_packs_fmt_described_message_into_write_block() {
    let row = LogStructure {
        msg_type: 42,
        msg_len: calc_msg_len("QBH").expect("len"),
        name: "TEST",
        format: "QBH",
        labels: "TimeUS,Id,Val",
    };
    assert_eq!(row.msg_len, 14);

    let fields = [
        LogValue::U64(1_000_000),
        LogValue::U8(7),
        LogValue::U16(0xABCD),
    ];

    let mut log = MemoryBackend::<64>::new();
    log.start_write(3);
    assert!(write_message(&mut log, &row, &fields));
    log.end_write();

    let rec = log.recorded();
    assert_eq!(rec.len(), 14);
    assert_eq!(rec.get(0), Some(&HEAD_BYTE1));
    assert_eq!(rec.get(1), Some(&HEAD_BYTE2));
    assert_eq!(rec.get(2), Some(&42));
    assert_eq!(rec.get(3..11), Some(1_000_000u64.to_le_bytes().as_slice()));
    assert_eq!(rec.get(11), Some(&7));
    assert_eq!(rec.get(12..14), Some(0xABCDu16.to_le_bytes().as_slice()));
    assert_eq!(log.page_adr(), 3);
    assert!(!log.is_writing());
}

#[test]
fn calc_msg_len_of_fmt_row_is_log_format_len() {
    assert_eq!(calc_msg_len("BBnNZ"), Some(LOG_FORMAT_LEN as u8));
}
