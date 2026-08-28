//! ATRP log at 25 Hz, upstream `AP_AutoTune::update` `WriteBlock`.
//!
//! When `now - last_log_ms >= 40`, AutoTune writes a packed `log_ATRP`
//! block (`LOG_ATRP_MSG`) at 25 Hz. The live logger `WriteBlock` bind
//! is deferred with the AP_Logger port; this module is the packet and
//! the 40 ms gate.
//!
//! Completeness leftover rows (EEPROM `save_*_if_changed`, `update_rmax`
//! inverse-tau, LOW_RATE/SHORT rejects, clipped actuator without I)
//! stay deferred.

use crate::action::Action;
use crate::state::{AtState, AtType, AutoTune};

/// ATRP log period, upstream `now - last_log_ms >= 40`.
pub const ATRP_LOG_PERIOD_MS: u32 = 40;

/// ATRP log rate implied by [`ATRP_LOG_PERIOD_MS`].
pub const ATRP_LOG_HZ: u32 = 1000 / ATRP_LOG_PERIOD_MS;

/// DataFlash / AP_Logger first header byte, upstream `HEAD_BYTE1`.
pub const HEAD_BYTE1: u8 = 0xA3;

/// DataFlash / AP_Logger second header byte, upstream `HEAD_BYTE2`.
pub const HEAD_BYTE2: u8 = 0x95;

/// Logger message name, upstream ArduPlane `Log.cpp` `"ATRP"`.
pub const ATRP_NAME: &str = "ATRP";

/// Logger format string, upstream `"QBBffffffffBff"`.
pub const ATRP_FORMAT: &str = "QBBffffffffBff";

/// Logger field labels, upstream `TimeUS,Axis,State,Sur,...`.
pub const ATRP_LABELS: &str = "TimeUS,Axis,State,Sur,PSlew,DSlew,FF0,FF,P,I,D,Action,RMAX,TAU";

/// Packed `AP_AutoTune::log_ATRP` payload plus `LOG_PACKET_HEADER`.
///
/// Layout matches the C++ `PACKED` struct field list. `msgid` is 0 in
/// this stub — `LOG_ATRP_MSG` is a generated logger enum, not a fixed
/// literal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LogAtrp {
    /// Upstream `head1` (`HEAD_BYTE1` = 0xA3).
    pub head1: u8,
    /// Upstream `head2` (`HEAD_BYTE2` = 0x95).
    pub head2: u8,
    /// Upstream `msgid` (`LOG_ATRP_MSG`). Stub writes 0.
    pub msgid: u8,
    /// Upstream `time_us` (`AP_HAL::micros64()`).
    pub time_us: u64,
    /// Upstream `type` (`ATType` / logger `Axis`).
    pub axis: u8,
    /// Upstream `state` (`ATState`).
    pub state: u8,
    /// Upstream `actuator` (logger `Sur`).
    pub actuator: f32,
    /// Upstream `P_slew` (`max_SRate_P`).
    pub p_slew: f32,
    /// Upstream `D_slew` (`max_SRate_D`).
    pub d_slew: f32,
    /// Upstream `FF_single`.
    pub ff_single: f32,
    /// Upstream `FF` (`current.FF`).
    pub ff: f32,
    /// Upstream `P` (`current.P`).
    pub p: f32,
    /// Upstream `I` (`current.I`).
    pub i: f32,
    /// Upstream `D` (`current.D`).
    pub d: f32,
    /// Upstream `action` (`Action`).
    pub action: u8,
    /// Upstream `rmax` (`current.rmax_pos`).
    pub rmax: f32,
    /// Upstream `tau` (`current.tau`).
    pub tau: f32,
}

impl LogAtrp {
    /// Fill a packet the way `update` designates `const struct log_ATRP pkt`.
    #[must_use]
    pub fn from_update(
        time_us: u64,
        axis: AtType,
        state: AtState,
        actuator: f32,
        p_slew: f32,
        d_slew: f32,
        ff_single: f32,
        ff: f32,
        p: f32,
        i: f32,
        d: f32,
        action: Action,
        rmax: f32,
        tau: f32,
    ) -> Self {
        Self {
            head1: HEAD_BYTE1,
            head2: HEAD_BYTE2,
            msgid: 0,
            time_us,
            axis: axis.as_u8(),
            state: state.as_u8(),
            actuator,
            p_slew,
            d_slew,
            ff_single,
            ff,
            p,
            i,
            d,
            action: action.as_u8(),
            rmax,
            tau,
        }
    }

    /// Build from a live session plus the per-cycle slew / FF / actuator samples.
    #[must_use]
    pub fn from_session(
        tuner: &AutoTune,
        time_us: u64,
        actuator: f32,
        p_slew: f32,
        d_slew: f32,
        ff_single: f32,
        action: Action,
    ) -> Self {
        Self::from_update(
            time_us,
            tuner.axis,
            tuner.state,
            actuator,
            p_slew,
            d_slew,
            ff_single,
            tuner.ff,
            tuner.current.p,
            tuner.current.i,
            tuner.current.d,
            action,
            tuner.current.rmax_pos,
            tuner.current.tau,
        )
    }
}

/// True when a 25 Hz ATRP block is due, upstream `now - last_log_ms >= 40`.
#[must_use]
pub fn should_log_atrp(now_ms: u32, last_log_ms: u32) -> bool {
    now_ms.wrapping_sub(last_log_ms) >= ATRP_LOG_PERIOD_MS
}

/// Next `last_log_ms` after a write, upstream `last_log_ms = now`.
#[must_use]
pub const fn stamp_log_ms(now_ms: u32) -> u32 {
    now_ms
}

/// 25 Hz gate holding upstream `AP_AutoTune::last_log_ms`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AtrpLogGate {
    /// Upstream `AP_AutoTune::last_log_ms`.
    pub last_log_ms: u32,
}

impl Default for AtrpLogGate {
    fn default() -> Self {
        Self::new()
    }
}

impl AtrpLogGate {
    /// Zeroed stamp, matching a default-constructed C++ member.
    #[must_use]
    pub const fn new() -> Self {
        Self { last_log_ms: 0 }
    }

    /// Whether `now_ms` is due for a WriteBlock.
    #[must_use]
    pub fn due(&self, now_ms: u32) -> bool {
        should_log_atrp(now_ms, self.last_log_ms)
    }

    /// Emit a packet when due; otherwise `None` (no WriteBlock).
    pub fn maybe_write(
        &mut self,
        now_ms: u32,
        tuner: &AutoTune,
        time_us: u64,
        actuator: f32,
        p_slew: f32,
        d_slew: f32,
        ff_single: f32,
        action: Action,
    ) -> Option<LogAtrp> {
        if !self.due(now_ms) {
            return None;
        }
        self.last_log_ms = stamp_log_ms(now_ms);
        Some(LogAtrp::from_session(
            tuner, time_us, actuator, p_slew, d_slew, ff_single, action,
        ))
    }
}
