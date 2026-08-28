//! FW-030 AP_Logger completeness: surfaces already on main vs remaining.
//!
//! Catalogs the SITL-first `AP_Logger` / DataFlash port. Items marked
//! [`PortStatus::OnMain`] landed in earlier slices and must not be
//! redone. [`PortStatus::ThisSlice`] is this table.
//! [`PortStatus::Remaining`] are documented-deferred (DataFlash page
//! map, full `AP_Logger` front-end, POSIX/SD file backend, sitl-diff
//! replay) outside this ticket's stub surface.
//!
//! This module does not rewrite [`crate::registry`], [`crate::streaming`],
//! or [`crate::write`].

/// Whether a catalog row is already hooked up or left for later work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortStatus {
    /// Present on `main` before this closing slice.
    OnMain,
    /// Added by the FW-030 closing slice (this table).
    ThisSlice,
    /// Documented-deferred: not blocking the FW-030 SITL stub close.
    Remaining,
}

/// One AP_Logger surface in the completeness table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoggerPortItem {
    /// Surface name.
    pub name: &'static str,
    /// Hooked up on main / this slice, or remaining.
    pub status: PortStatus,
    /// Short note (upstream symbol or why remaining).
    pub note: &'static str,
}

/// Completeness table: ported logger stubs vs documented-deferred gaps.
///
/// Row names match the closer catalog: backend, FMT, Write, bitmask,
/// file, transfer, replay, drop, erase, rotate, streaming, registry.
pub const LOGGER_COMPLETENESS: &[LoggerPortItem] = &[
    LoggerPortItem {
        name: "backend",
        status: PortStatus::OnMain,
        note: "LogBackend WriteBlock/StartWrite/EndWrite + MemoryBackend",
    },
    LoggerPortItem {
        name: "FMT",
        status: PortStatus::OnMain,
        note: "LogStructure / FMT header type, length, name, format, labels",
    },
    LoggerPortItem {
        name: "Write",
        status: PortStatus::OnMain,
        note: "Write() typed-message dispatcher packs FMT into WriteBlock",
    },
    LoggerPortItem {
        name: "bitmask",
        status: PortStatus::OnMain,
        note: "LOG_BITMASK / logging-started gate; Write no-op when disabled",
    },
    LoggerPortItem {
        name: "file",
        status: PortStatus::OnMain,
        note: "AP_Logger_File StartWrite/EndWrite path + buffer mock",
    },
    LoggerPortItem {
        name: "transfer",
        status: PortStatus::OnMain,
        note: "MAVLink LOG_REQUEST_LIST listing (LOG_ENTRY count / last-log id)",
    },
    LoggerPortItem {
        name: "replay",
        status: PortStatus::OnMain,
        note: "LOG_REQUEST_DATA / LOG_DATA replay 90-byte chunks from file mock",
    },
    LoggerPortItem {
        name: "drop",
        status: PortStatus::OnMain,
        note: "dropped-message / buffer-full counter num_dropped",
    },
    LoggerPortItem {
        name: "erase",
        status: PortStatus::OnMain,
        note: "LOG_ERASE / EraseAll clears file-backend catalog and drop count",
    },
    LoggerPortItem {
        name: "rotate",
        status: PortStatus::OnMain,
        note: "log-file rotation / max-files drops oldest when catalog full",
    },
    LoggerPortItem {
        name: "streaming",
        status: PortStatus::OnMain,
        note: "WriteStreaming rate-limit gate emit after 1000/rate_hz ms",
    },
    LoggerPortItem {
        name: "registry",
        status: PortStatus::OnMain,
        note: "FmtRegistry name → type/len lookup (msg_fmt_for_name)",
    },
    LoggerPortItem {
        name: "completeness table",
        status: PortStatus::ThisSlice,
        note: "this catalog",
    },
    LoggerPortItem {
        name: "DataFlash page map",
        status: PortStatus::Remaining,
        note: "page-based DataFlash layout; stub uses in-memory / file-buffer mocks",
    },
    LoggerPortItem {
        name: "AP_Logger front-end",
        status: PortStatus::Remaining,
        note: "full AP_Logger class / Init / periodic Write_... vehicle wrappers",
    },
    LoggerPortItem {
        name: "POSIX/SD File backend",
        status: PortStatus::Remaining,
        note: "AP_Logger_File on real filesystem; file.rs is a buffer mock",
    },
    LoggerPortItem {
        name: "sitl-diff log-replay",
        status: PortStatus::Remaining,
        note: "ADR-0008 differential vs recorded outputs; not MAVLink LOG_DATA",
    },
];

/// Rows already hooked up on `main` (must not be redone).
#[must_use]
pub fn on_main_items() -> impl Iterator<Item = &'static LoggerPortItem> {
    LOGGER_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::OnMain)
}

/// Rows added by this closing slice.
#[must_use]
pub fn this_slice_items() -> impl Iterator<Item = &'static LoggerPortItem> {
    LOGGER_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::ThisSlice)
}

/// Rows left documented-deferred (not blocking FW-030 SITL close).
#[must_use]
pub fn remaining_items() -> impl Iterator<Item = &'static LoggerPortItem> {
    LOGGER_COMPLETENESS
        .iter()
        .filter(|item| item.status == PortStatus::Remaining)
}

/// Count rows in each status bucket.
#[must_use]
pub fn completeness_counts() -> (usize, usize, usize) {
    let mut on_main = 0;
    let mut this_slice = 0;
    let mut remaining = 0;
    for item in LOGGER_COMPLETENESS {
        match item.status {
            PortStatus::OnMain => on_main += 1,
            PortStatus::ThisSlice => this_slice += 1,
            PortStatus::Remaining => remaining += 1,
        }
    }
    (on_main, this_slice, remaining)
}

/// True when `name` is listed with `status`.
#[must_use]
pub fn completeness_has(name: &str, status: PortStatus) -> bool {
    LOGGER_COMPLETENESS
        .iter()
        .any(|item| item.name == name && item.status == status)
}

/// True when every name in the table appears once.
#[must_use]
pub fn completeness_unique_names() -> bool {
    for (i, item) in LOGGER_COMPLETENESS.iter().enumerate() {
        for other in LOGGER_COMPLETENESS.iter().skip(i + 1) {
            if item.name == other.name {
                return false;
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn table_covers_main_surfaces_and_this_slice() {
        assert!(completeness_unique_names());
        let (on_main, this_slice, remaining) = completeness_counts();
        assert_eq!(on_main, 12);
        assert_eq!(this_slice, 1);
        assert_eq!(remaining, 4);
        assert!(completeness_has("backend", PortStatus::OnMain));
        assert!(completeness_has("FMT", PortStatus::OnMain));
        assert!(completeness_has("Write", PortStatus::OnMain));
        assert!(completeness_has("bitmask", PortStatus::OnMain));
        assert!(completeness_has("file", PortStatus::OnMain));
        assert!(completeness_has("transfer", PortStatus::OnMain));
        assert!(completeness_has("replay", PortStatus::OnMain));
        assert!(completeness_has("drop", PortStatus::OnMain));
        assert!(completeness_has("erase", PortStatus::OnMain));
        assert!(completeness_has("rotate", PortStatus::OnMain));
        assert!(completeness_has("streaming", PortStatus::OnMain));
        assert!(completeness_has("registry", PortStatus::OnMain));
        assert!(completeness_has(
            "completeness table",
            PortStatus::ThisSlice
        ));
        assert!(completeness_has(
            "DataFlash page map",
            PortStatus::Remaining
        ));
        assert!(completeness_has(
            "AP_Logger front-end",
            PortStatus::Remaining
        ));
        assert!(completeness_has(
            "POSIX/SD File backend",
            PortStatus::Remaining
        ));
        assert!(completeness_has(
            "sitl-diff log-replay",
            PortStatus::Remaining
        ));
        assert_eq!(on_main_items().count(), 12);
        assert_eq!(this_slice_items().count(), 1);
        assert_eq!(remaining_items().count(), 4);
    }

    #[test]
    fn remaining_does_not_repeat_hooked_surfaces() {
        for item in remaining_items() {
            assert!(
                !completeness_has(item.name, PortStatus::OnMain),
                "{} listed remaining but already on main",
                item.name
            );
            assert!(
                !completeness_has(item.name, PortStatus::ThisSlice),
                "{} listed remaining but added this slice",
                item.name
            );
        }
    }
}
