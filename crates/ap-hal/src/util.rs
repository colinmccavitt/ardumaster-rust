//! Utility surfaces, ported from `AP_HAL/Util.h`.
//!
//! Safety-switch state, board serial number, the watchdog-restored
//! [`PersistentData`] dump, and free-memory reporting. Soft-arm, RTC, tone
//! alarm, and the DMA allocators are board surfaces and land with a consumer
//! that needs them.
//!
//! Upstream keeps `persistent_data` as a public member on `AP_HAL::Util` and
//! documents that callers may only read it after a watchdog reset. The port
//! exposes the same struct through accessors so a backend can own the storage
//! without a global `Util` singleton (ADR-0004).

/// Length of the buffer `get_system_id` fills, including the NUL.
///
/// Upstream `get_system_id(char buf[50])`.
pub const SYSTEM_ID_LEN: usize = 50;

/// Default free-memory report when the backend cannot measure it.
///
/// Upstream `available_memory()` returns this (4096) if unknown.
pub const AVAILABLE_MEMORY_UNKNOWN: u32 = 4096;

/// Safety-switch state. Upstream `AP_HAL::Util::safety_state`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum SafetyState {
    /// No safety switch on this board. Upstream `SAFETY_NONE`.
    #[default]
    None = 0,
    /// Switch present, outputs disarmed. Upstream `SAFETY_DISARMED`.
    Disarmed = 1,
    /// Switch present, outputs armed. Upstream `SAFETY_ARMED`.
    Armed = 2,
}

impl SafetyState {
    /// The upstream `enum safety_state` ordinal.
    #[inline]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// Watchdog-restored snapshot. Upstream `AP_HAL::Util::PersistentData`.
///
/// Upstream caps this at 76 bytes on STM32. The stub carries the fields
/// that arming and the safety path read after a reset; attitude, home, and
/// the fault dump land with a consumer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PersistentData {
    /// Vehicle was armed when the watchdog fired. Upstream `armed`.
    pub armed: bool,
    /// Safety-switch state at the reset. Upstream `safety_state`.
    pub safety_state: SafetyState,
}

/// Board utilities. Upstream `AP_HAL::Util`.
///
/// `safety_switch_state`, `get_system_id`, and `available_memory` default the
/// same way the C++ base class does: no switch, no serial number, 4096 bytes
/// free. `persistent_data` has no default because a backend must own the
/// storage.
pub trait Util {
    /// State of the safety switch, if the board has one.
    ///
    /// Upstream `safety_switch_state()`, default `SAFETY_NONE`.
    fn safety_switch_state(&self) -> SafetyState {
        SafetyState::None
    }

    /// Fill `buf` with a printable, NUL-terminated board identifier.
    ///
    /// Returns `false` if no identifier is available, or if `buf` is shorter
    /// than the identifier plus NUL. Upstream `get_system_id(char buf[50])`.
    fn get_system_id(&self, buf: &mut [u8]) -> bool {
        let _ = buf;
        false
    }

    /// The live persistent-data dump. Upstream `persistent_data`.
    fn persistent_data(&self) -> &PersistentData;

    /// Mutable access so a backend can update the dump before a reboot.
    fn persistent_data_mut(&mut self) -> &mut PersistentData;

    /// Free memory in bytes. Upstream `available_memory()`.
    ///
    /// Unknown backends return [`AVAILABLE_MEMORY_UNKNOWN`], matching the
    /// C++ default of 4096.
    fn available_memory(&self) -> u32 {
        AVAILABLE_MEMORY_UNKNOWN
    }
}

/// In-memory utilities for tests and SITL bring-up.
#[derive(Debug, Clone)]
pub struct MockUtil {
    safety: SafetyState,
    system_id: [u8; SYSTEM_ID_LEN],
    system_id_len: usize,
    has_system_id: bool,
    persistent: PersistentData,
    memory: u32,
}

impl Default for MockUtil {
    fn default() -> Self {
        Self {
            safety: SafetyState::None,
            system_id: [0; SYSTEM_ID_LEN],
            system_id_len: 0,
            has_system_id: false,
            persistent: PersistentData::default(),
            memory: AVAILABLE_MEMORY_UNKNOWN,
        }
    }
}

impl MockUtil {
    /// Defaults matching the C++ base class: no switch, no id, 4096 bytes.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the reported safety-switch state.
    #[inline]
    pub fn set_safety_switch_state(&mut self, state: SafetyState) {
        self.safety = state;
    }

    /// Install a printable system id. `bytes` is copied without a NUL;
    /// [`Util::get_system_id`] appends one.
    ///
    /// Returns `false` if `bytes` does not fit in [`SYSTEM_ID_LEN`] minus NUL.
    pub fn set_system_id(&mut self, bytes: &[u8]) -> bool {
        if bytes.len() + 1 > SYSTEM_ID_LEN {
            return false;
        }
        self.system_id[..bytes.len()].copy_from_slice(bytes);
        self.system_id_len = bytes.len();
        self.has_system_id = true;
        true
    }

    /// Forget any installed system id so [`Util::get_system_id`] fails.
    #[inline]
    pub fn clear_system_id(&mut self) {
        self.has_system_id = false;
        self.system_id_len = 0;
    }

    /// Override the free-memory report.
    #[inline]
    pub fn set_available_memory(&mut self, bytes: u32) {
        self.memory = bytes;
    }
}

impl Util for MockUtil {
    fn safety_switch_state(&self) -> SafetyState {
        self.safety
    }

    fn get_system_id(&self, buf: &mut [u8]) -> bool {
        if !self.has_system_id {
            return false;
        }
        let need = self.system_id_len + 1;
        if buf.len() < need {
            return false;
        }
        buf[..self.system_id_len].copy_from_slice(&self.system_id[..self.system_id_len]);
        buf[self.system_id_len] = 0;
        true
    }

    fn persistent_data(&self) -> &PersistentData {
        &self.persistent
    }

    fn persistent_data_mut(&mut self) -> &mut PersistentData {
        &mut self.persistent
    }

    fn available_memory(&self) -> u32 {
        self.memory
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn safety_switch_defaults_to_none() {
        let util = MockUtil::new();
        assert_eq!(util.safety_switch_state(), SafetyState::None);
        assert_eq!(SafetyState::None.as_u8(), 0);
        assert_eq!(SafetyState::Disarmed.as_u8(), 1);
        assert_eq!(SafetyState::Armed.as_u8(), 2);
    }

    #[test]
    fn safety_switch_round_trip() {
        let mut util = MockUtil::new();
        util.set_safety_switch_state(SafetyState::Disarmed);
        assert_eq!(util.safety_switch_state(), SafetyState::Disarmed);
        util.set_safety_switch_state(SafetyState::Armed);
        assert_eq!(util.safety_switch_state(), SafetyState::Armed);
    }

    #[test]
    fn get_system_id_fails_when_unset() {
        let util = MockUtil::new();
        let mut buf = [0u8; SYSTEM_ID_LEN];
        assert!(!util.get_system_id(&mut buf));
    }

    #[test]
    fn get_system_id_fills_nul_terminated_buffer() {
        let mut util = MockUtil::new();
        assert!(util.set_system_id(b"TEST-BOARD-01"));
        let mut buf = [0xFFu8; SYSTEM_ID_LEN];
        assert!(util.get_system_id(&mut buf));
        assert_eq!(&buf[..13], b"TEST-BOARD-01");
        assert_eq!(buf[13], 0);
    }

    #[test]
    fn get_system_id_refuses_short_buffer() {
        let mut util = MockUtil::new();
        assert!(util.set_system_id(b"ABC"));
        let mut tiny = [0u8; 3];
        assert!(!util.get_system_id(&mut tiny));
        let mut just_enough = [0xFFu8; 4];
        assert!(util.get_system_id(&mut just_enough));
        assert_eq!(&just_enough, b"ABC\0");
    }

    /// PersistentData is the dump a watchdog restore reads. Armed and
    /// safety_state are the fields the Util surface itself consults.
    #[test]
    fn persistent_data_armed_and_safety_round_trip() {
        let mut util = MockUtil::new();
        assert_eq!(util.persistent_data(), &PersistentData::default());

        util.persistent_data_mut().armed = true;
        util.persistent_data_mut().safety_state = SafetyState::Armed;
        assert!(util.persistent_data().armed);
        assert_eq!(util.persistent_data().safety_state, SafetyState::Armed);
    }

    #[test]
    fn available_memory_defaults_to_4096() {
        let mut util = MockUtil::new();
        assert_eq!(util.available_memory(), AVAILABLE_MEMORY_UNKNOWN);
        assert_eq!(AVAILABLE_MEMORY_UNKNOWN, 4096);
        util.set_available_memory(128 * 1024);
        assert_eq!(util.available_memory(), 128 * 1024);
    }

    /// The trait stays object-safe, which is what allows `&dyn` in the HAL
    /// context. If a future method breaks object safety this fails to compile
    /// here rather than at some distant call site.
    #[test]
    fn util_trait_is_object_safe() {
        let mut util = MockUtil::new();
        util.set_safety_switch_state(SafetyState::Disarmed);
        assert!(util.set_system_id(b"OBJ"));
        let u: &mut dyn Util = &mut util;
        assert_eq!(u.safety_switch_state(), SafetyState::Disarmed);
        let mut buf = [0u8; SYSTEM_ID_LEN];
        assert!(u.get_system_id(&mut buf));
        assert_eq!(u.available_memory(), AVAILABLE_MEMORY_UNKNOWN);
        u.persistent_data_mut().armed = true;
        assert!(u.persistent_data().armed);
    }
}
