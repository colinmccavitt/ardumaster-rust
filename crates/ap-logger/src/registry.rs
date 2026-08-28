//! FMT registry / LogStructure table lookup, upstream `AP_Logger` structure list.
//!
//! `Write(name, ...)` and FMT emission resolve a four-character message
//! name against the registered `LogStructure` table (`_structures` /
//! `msg_fmt_for_name`). This stub is that table: register FMT rows,
//! then look up `type` and `length` by name. Not units, not
//! multipliers, not the Write() dispatcher, not FMT packet emission.

use crate::structure::LogStructure;

/// Stub table cap. Upstream grows with `LOG_COMMON_STRUCTURES` plus
/// vehicle structures; this mock keeps a small fixed table so lookup
/// stays `no_std`.
pub const MAX_FMT_ROWS: usize = 16;

/// Registered FMT / [`LogStructure`] rows, keyed by message name.
///
/// Upstream `AP_Logger::_structures` / `msg_fmt_for_name`.
#[derive(Clone, Copy, Debug)]
pub struct FmtRegistry<const N: usize> {
    rows: [Option<LogStructure>; N],
    count: usize,
}

impl<const N: usize> Default for FmtRegistry<N> {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> FmtRegistry<N> {
    /// Empty table. Upstream starts with compiled-in structures; this
    /// stub is filled by [`register`](Self::register).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            rows: [None; N],
            count: 0,
        }
    }

    /// Empty table that already holds the FMT row itself.
    ///
    /// Every DataFlash log begins with FMT (`LOG_FORMAT_MSG`); this
    /// matches a registry that has emitted its own format first.
    #[must_use]
    pub fn with_fmt() -> Self {
        let mut reg = Self::new();
        let _ = reg.register(LogStructure::fmt());
        reg
    }

    /// How many FMT rows are registered.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.count
    }

    /// Whether the table is empty.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Table capacity. Unused slots stay `None`.
    #[must_use]
    pub const fn capacity(&self) -> usize {
        N
    }

    /// Register a FMT / [`LogStructure`] row.
    ///
    /// Returns `false` when the table is full or `row.name` is already
    /// present. Duplicate names would make `Write(name, ...)`
    /// ambiguous; upstream assigns one `msg_type` per name.
    #[must_use]
    pub fn register(&mut self, row: LogStructure) -> bool {
        if self.lookup(row.name).is_some() {
            return false;
        }
        if self.count >= N {
            return false;
        }
        let Some(slot) = self.rows.get_mut(self.count) else {
            return false;
        };
        *slot = Some(row);
        self.count += 1;
        true
    }

    /// The registered row whose [`LogStructure::name`] equals `name`.
    ///
    /// Upstream `msg_fmt_for_name` / `structure_for`.
    #[must_use]
    pub fn lookup(&self, name: &str) -> Option<&LogStructure> {
        self.rows
            .iter()
            .take(self.count)
            .filter_map(|slot| slot.as_ref())
            .find(|row| row.name == name)
    }

    /// Message `type` and `length` for a registered name.
    ///
    /// The two fields a `Write(name, ...)` caller needs after the
    /// FMT table hit: `msg_type` and `msg_len`.
    #[must_use]
    pub fn type_and_len(&self, name: &str) -> Option<(u8, u8)> {
        let row = self.lookup(name)?;
        Some((row.msg_type, row.msg_len))
    }

    /// Drop every registered row. Upstream `StartNewLog` keeps the
    /// compiled-in table; this stub is a mock and can be emptied.
    pub fn clear(&mut self) {
        self.rows = [None; N];
        self.count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::structure::{LOG_FORMAT_LEN, LOG_FORMAT_MSG};
    use crate::write::calc_msg_len;

    fn test_row() -> LogStructure {
        LogStructure {
            msg_type: 42,
            msg_len: calc_msg_len("QBH").expect("len"),
            name: "TEST",
            format: "QBH",
            labels: "TimeUS,Id,Val",
        }
    }

    #[test]
    fn empty_lookup_is_none() {
        let reg = FmtRegistry::<4>::new();
        assert!(reg.is_empty());
        assert_eq!(reg.len(), 0);
        assert_eq!(reg.capacity(), 4);
        assert!(reg.lookup("FMT").is_none());
        assert!(reg.type_and_len("FMT").is_none());
    }

    #[test]
    fn lookup_returns_type_and_len_from_registered_row() {
        let mut reg = FmtRegistry::<4>::new();
        let row = test_row();
        assert!(reg.register(row));
        assert_eq!(reg.len(), 1);
        assert_eq!(reg.type_and_len("TEST"), Some((42, 14)));
        let got = reg.lookup("TEST").expect("row");
        assert_eq!(got.msg_type, 42);
        assert_eq!(got.msg_len, 14);
        assert_eq!(got.format, "QBH");
        assert!(reg.type_and_len("GPS").is_none());
    }

    #[test]
    fn with_fmt_registers_the_fmt_row() {
        let reg = FmtRegistry::<4>::with_fmt();
        assert_eq!(reg.len(), 1);
        assert_eq!(
            reg.type_and_len("FMT"),
            Some((LOG_FORMAT_MSG, LOG_FORMAT_LEN as u8))
        );
        let row = reg.lookup("FMT").expect("FMT");
        assert_eq!(row.name, "FMT");
        assert_eq!(row.format, "BBnNZ");
    }

    #[test]
    fn duplicate_name_is_rejected() {
        let mut reg = FmtRegistry::<4>::new();
        assert!(reg.register(test_row()));
        let again = LogStructure {
            msg_type: 99,
            msg_len: 8,
            name: "TEST",
            format: "Q",
            labels: "T",
        };
        assert!(!reg.register(again));
        assert_eq!(reg.len(), 1);
        assert_eq!(reg.type_and_len("TEST"), Some((42, 14)));
    }

    #[test]
    fn full_table_rejects_new_names() {
        let mut reg = FmtRegistry::<1>::new();
        assert!(reg.register(LogStructure::fmt()));
        assert!(!reg.register(test_row()));
        assert_eq!(reg.len(), 1);
        assert!(reg.lookup("TEST").is_none());
    }

    #[test]
    fn clear_empties_the_table() {
        let mut reg = FmtRegistry::<4>::with_fmt();
        assert!(!reg.is_empty());
        reg.clear();
        assert!(reg.is_empty());
        assert!(reg.lookup("FMT").is_none());
        assert_eq!(FmtRegistry::<4>::default().len(), 0);
    }
}
