//! Integration coverage of the FMT / `LogStructure` header.
//!
//! Upstream `libraries/AP_Logger/LogStructure.h`: every packet starts
//! with `HEAD_BYTE1`/`HEAD_BYTE2`/`msgid`, and FMT (type 128) names
//! the type, length, name, format, and labels of every other message.

use ap_logger::{
    fill_format, LogBackend, LogStructure, MemoryBackend, HEAD_BYTE1, HEAD_BYTE2, LOG_FORMAT_LEN,
    LOG_FORMAT_MSG,
};

#[test]
fn fmt_structure_fields_are_type_length_name_format_labels() {
    let row = LogStructure {
        msg_type: 0x80,
        msg_len: 89,
        name: "FMT",
        format: "BBnNZ",
        labels: "Type,Length,Name,Format,Columns",
    };
    assert_eq!(row.msg_type, LOG_FORMAT_MSG);
    assert_eq!(row.msg_len, LOG_FORMAT_LEN as u8);
    assert_eq!(row.name, "FMT");
    assert_eq!(row.format, "BBnNZ");
    assert_eq!(row.labels, "Type,Length,Name,Format,Columns");
}

#[test]
fn fill_format_writes_fmt_packet_through_backend() {
    let pkt = fill_format(&LogStructure::fmt());
    let bytes = pkt.pack();

    let mut log = MemoryBackend::<128>::new();
    log.start_write(0);
    assert!(log.write_block(&bytes));
    log.end_write();

    let recorded = log.recorded();
    assert_eq!(recorded.len(), LOG_FORMAT_LEN);
    assert_eq!(recorded.get(0), Some(&HEAD_BYTE1));
    assert_eq!(recorded.get(1), Some(&HEAD_BYTE2));
    assert_eq!(recorded.get(2), Some(&LOG_FORMAT_MSG));
    assert_eq!(recorded.get(3), Some(&LOG_FORMAT_MSG));
    assert_eq!(recorded.get(4), Some(&(LOG_FORMAT_LEN as u8)));
    assert_eq!(recorded.get(5..8), Some(b"FMT".as_slice()));
    assert_eq!(recorded.get(9..14), Some(b"BBnNZ".as_slice()));
}
