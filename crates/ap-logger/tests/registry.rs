//! Integration coverage of the FMT registry / LogStructure table lookup.
//!
//! Upstream `AP_Logger` resolves `Write(name, ...)` against the
//! registered `LogStructure` table (`msg_fmt_for_name`). This drives
//! name → type/len from registered FMT rows.

use ap_logger::{
    calc_msg_len, FmtRegistry, LogStructure, LOG_FORMAT_LEN, LOG_FORMAT_MSG, MAX_FMT_ROWS,
};

fn gps_row() -> LogStructure {
    LogStructure {
        msg_type: 9,
        msg_len: calc_msg_len("QBI").expect("len"),
        name: "GPS",
        format: "QBI",
        labels: "TimeUS,Status,NSats",
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
fn lookup_returns_type_and_len_from_registered_fmt_rows() {
    let mut reg = FmtRegistry::<8>::new();
    assert_eq!(reg.capacity(), 8);
    assert!(reg.is_empty());

    assert!(reg.register(LogStructure::fmt()));
    assert!(reg.register(gps_row()));
    assert!(reg.register(att_row()));
    assert_eq!(reg.len(), 3);

    assert_eq!(
        reg.type_and_len("FMT"),
        Some((LOG_FORMAT_MSG, LOG_FORMAT_LEN as u8))
    );
    assert_eq!(reg.type_and_len("GPS"), Some((9, 16)));
    assert_eq!(reg.type_and_len("ATT"), Some((1, 6)));
    assert_eq!(reg.type_and_len("IMU"), None);

    let gps = reg.lookup("GPS").expect("GPS");
    assert_eq!(gps.msg_type, 9);
    assert_eq!(gps.msg_len, 16);
    assert_eq!(gps.format, "QBI");
}

#[test]
fn default_capacity_matches_stub_table_cap() {
    let reg = FmtRegistry::<MAX_FMT_ROWS>::new();
    assert_eq!(reg.capacity(), MAX_FMT_ROWS);
    assert_eq!(MAX_FMT_ROWS, 16);
}

#[test]
fn with_fmt_then_vehicle_rows_keep_independent_lookups() {
    let mut reg = FmtRegistry::<MAX_FMT_ROWS>::with_fmt();
    assert_eq!(
        reg.type_and_len("FMT"),
        Some((LOG_FORMAT_MSG, LOG_FORMAT_LEN as u8))
    );
    assert!(reg.register(gps_row()));
    assert!(reg.register(att_row()));
    assert_eq!(reg.type_and_len("GPS"), Some((9, gps_row().msg_len)));
    assert_eq!(reg.type_and_len("ATT"), Some((1, att_row().msg_len)));
    assert!(!reg.register(gps_row()));
}
