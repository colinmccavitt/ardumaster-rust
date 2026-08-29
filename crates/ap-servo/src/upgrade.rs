//! SERVOn parameter loading, upstream `SRV_Channels::upgrade_parameters`.
//! COP-030 leftover.
//!
//! FUNCTION used to be `AP_Int8`. Vehicles in the field still have that
//! width in EEPROM. This walks every channel and widens it to `AP_Int16`
//! in place — same key, same group element, new type. It does not touch
//! MIN/MAX/TRIM/REVERSED: those were already 16-bit.
//!
//! A channel already stored as Int16 is left alone (the user or a previous
//! upgrade wrote it). A channel with nothing in storage stays at the
//! default. Those two must not be collapsed: treating "not stored" as
//! "already converted" would skip a real Int8 and leave the vehicle on
//! the default function.
//!
//! The conversion is numeric, not bitwise. An Int8 of `-1` becomes Int16
//! `-1`, not 255. Bitmask widening is a different helper, used for masks,
//! and would silently remap a stored function.

use ap_param::info::GroupInfo;
use ap_param::{
    configured_in_storage, migrate_parameter_width, ConvertOutcome, ParamHeader, Storage,
    StorageError, VarType,
};

/// Default `SERVOn_MIN`, upstream `var_info` `MIN`.
pub const SERVO_MIN_DEFAULT: u16 = 1100;
/// Default `SERVOn_MAX`, upstream `var_info` `MAX`.
pub const SERVO_MAX_DEFAULT: u16 = 1900;
/// Default `SERVOn_TRIM`, upstream `var_info` `TRIM`.
pub const SERVO_TRIM_DEFAULT: u16 = 1500;
/// Default `SERVOn_REVERSED`.
pub const SERVO_REVERSED_DEFAULT: i8 = 0;
/// Default `SERVOn_FUNCTION` — `k_none`.
pub const SERVO_FUNCTION_DEFAULT: i16 = 0;

/// Per-channel `SRV_Channel::var_info`: MIN, MAX, TRIM, REVERSED, FUNCTION.
///
/// FUNCTION is Int16 now. `upgrade_parameters` is what finds an old Int8
/// at the same key and widens it.
pub const CHANNEL_VAR_INFO: &[GroupInfo<'static>] = &[
    GroupInfo {
        name: "MIN",
        idx: 1,
        ptype: VarType::Int16.as_u8(),
        flags: 0,
        group: None,
    },
    GroupInfo {
        name: "MAX",
        idx: 2,
        ptype: VarType::Int16.as_u8(),
        flags: 0,
        group: None,
    },
    GroupInfo {
        name: "TRIM",
        idx: 3,
        ptype: VarType::Int16.as_u8(),
        flags: 0,
        group: None,
    },
    GroupInfo {
        name: "REVERSED",
        idx: 4,
        ptype: VarType::Int8.as_u8(),
        flags: 0,
        group: None,
    },
    GroupInfo {
        name: "FUNCTION",
        idx: 5,
        ptype: VarType::Int16.as_u8(),
        flags: 0,
        group: None,
    },
];

/// What `upgrade_parameters` did across the channels it was given.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct UpgradeStats {
    /// Old Int8 found and written as Int16.
    pub saved: usize,
    /// Live Int16 already in storage — left alone.
    pub skipped_configured: usize,
    /// Nothing stored under either width — stays at the default.
    pub not_found: usize,
}

/// Widen each channel's FUNCTION from Int8 to Int16, upstream
/// `SRV_Channels::upgrade_parameters`.
///
/// `function_headers` are the *live* Int16 headers — one per channel,
/// same key and group element the Int8 was stored under. The parameter
/// table that produced those headers is a separate concern from the
/// widening (see [`CHANNEL_VAR_INFO`]).
pub fn upgrade_parameters<S: Storage + ?Sized>(
    storage: &mut S,
    function_headers: &[ParamHeader],
) -> Result<UpgradeStats, StorageError> {
    let mut stats = UpgradeStats::default();
    for header in function_headers {
        match migrate_parameter_width(
            storage,
            *header,
            VarType::Int8,
            VarType::Int16,
            configured_in_storage(storage, *header),
            1.0,
            false,
        )? {
            ConvertOutcome::Saved => stats.saved += 1,
            ConvertOutcome::SkippedConfigured => stats.skipped_configured += 1,
            ConvertOutcome::NotFound => stats.not_found += 1,
            ConvertOutcome::Unchanged => {}
        }
    }
    Ok(stats)
}

/// Whether `SERVOn_FUNCTION` is configured in storage, upstream
/// `function_configured`. Used by plane's parameter upgrade.
#[must_use]
pub fn function_configured<S: Storage + ?Sized>(storage: &S, header: ParamHeader) -> bool {
    configured_in_storage(storage, header)
}

/// Set reversed only when it changed, upstream
/// `reversed_set_and_save_ifchanged`.
///
/// Returns whether a save is required. The save itself is the caller's:
/// this leftover does not own the parameter object.
#[must_use]
pub fn reversed_set_and_save_ifchanged(reversed: &mut bool, want: bool) -> bool {
    if *reversed == want {
        return false;
    }
    *reversed = want;
    true
}
