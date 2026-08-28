//! Integration coverage of [`ap_logger::LogBackend`] via the in-memory backend.

use ap_logger::{LogBackend, MemoryBackend};

#[test]
fn memory_backend_records_write_block_bytes() {
    let mut log = MemoryBackend::<32>::new();
    log.start_write(1);
    assert!(log.write_block(b"HEAD"));
    assert!(log.write_block(&[0x10, 0x20, 0x30]));
    log.end_write();

    assert_eq!(log.recorded(), b"HEAD\x10\x20\x30");
    assert_eq!(log.page_adr(), 1);
    assert_eq!(log.ended_writes(), 1);
    assert!(!log.is_writing());
}
