//! Main-loop lockup heartbeat, upstream `Plane::failsafe_check`.
//!
//! `ArduPlane/failsafe.cpp` and `hal.scheduler->register_timer_failsafe`
//! in `ArduPlane/system.cpp` (1 kHz). The interrupt watches
//! `scheduler.ticks()`: an advancing tick is the last-heartbeat that the
//! scheduler is still running. A stall older than 200 ms latches
//! `in_failsafe`. While latched, every 20 ms the interrupt would pass RC
//! through to servos and, when `in_calibration`, pulse `afs.heartbeat()`
//! so Advanced Failsafe does not treat a log-erase / sensor-cal as a
//! lockup.
//!
//! This stub decides heartbeat health, the lockup latch, the 20 ms RC
//! passthrough cadence, and the AFS calibration heartbeat. Servo I/O,
//! `FS_*` action tables, and the short/long/off-event modules are left
//! alone.

/// Upstream lockup age: `tnow - last_timestamp > 200000` microseconds.
pub const FAILSAFE_CHECK_LOCKUP_US: u32 = 200_000;
/// Upstream RC-passthrough / AFS-heartbeat cadence: `> 20000` microseconds.
pub const FAILSAFE_CHECK_PASSTHROUGH_US: u32 = 20_000;
/// `hal.scheduler->register_timer_failsafe(..., 1000)` period, microseconds.
pub const FAILSAFE_CHECK_TIMER_PERIOD_US: u32 = 1_000;
/// `RC_Channels::get_valid_channel_count() < 5` blocks passthrough.
pub const FAILSAFE_CHECK_MIN_RC_CHANNELS: u8 = 5;

/// Persistent interrupt state (`last_ticks` / `last_timestamp` / `in_failsafe`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FailsafeCheckState {
    /// Last `scheduler.ticks()` observed by the interrupt.
    pub last_ticks: u16,
    /// `micros()` stamped on a healthy tick or after a passthrough pulse.
    pub last_timestamp_us: u32,
    /// Latched when the scheduler has been silent for 200 ms.
    pub in_failsafe: bool,
}

impl Default for FailsafeCheckState {
    fn default() -> Self {
        Self {
            last_ticks: 0,
            last_timestamp_us: 0,
            in_failsafe: false,
        }
    }
}

impl FailsafeCheckState {
    /// Fold a [`FailsafeCheckResult`] back into the interrupt statics.
    pub fn apply(&mut self, out: &FailsafeCheckResult) {
        self.last_ticks = out.last_ticks;
        self.last_timestamp_us = out.last_timestamp_us;
        self.in_failsafe = out.in_failsafe;
    }
}

/// One 1 kHz sample of `Plane::failsafe_check`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FailsafeCheckInputs {
    /// `micros()`.
    pub now_us: u32,
    /// `scheduler.ticks()`.
    pub scheduler_ticks: u16,
    /// `in_calibration` — sensor cal / log erase, pulse `afs.heartbeat()`.
    pub in_calibration: bool,
    /// `RC_Channels::get_valid_channel_count()`.
    pub valid_channel_count: u8,
    /// `arming.is_armed_and_safety_off()` — otherwise throttle is forced 0.
    pub armed_and_safety_off: bool,
}

impl Default for FailsafeCheckInputs {
    fn default() -> Self {
        Self {
            now_us: 0,
            scheduler_ticks: 0,
            in_calibration: false,
            valid_channel_count: 8,
            armed_and_safety_off: true,
        }
    }
}

/// What one `failsafe_check` sample asks the vehicle to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FailsafeCheckResult {
    /// Updated `in_failsafe` latch.
    pub in_failsafe: bool,
    /// Pass RC in to servo out this pulse (valid channels >= 5).
    pub pass_rc_through: bool,
    /// Pulse `afs.heartbeat()` this sample (`in_calibration` during lockup).
    pub afs_heartbeat: bool,
    /// Force throttle to 0 on the passthrough (`!armed_and_safety_off`).
    pub zero_throttle: bool,
    /// Updated `last_ticks`.
    pub last_ticks: u16,
    /// Updated `last_timestamp`.
    pub last_timestamp_us: u32,
}

/// Age of the last healthy tick / passthrough pulse, wrapping like `micros()`.
#[must_use]
pub fn failsafe_check_age_us(now_us: u32, last_timestamp_us: u32) -> u32 {
    now_us.wrapping_sub(last_timestamp_us)
}

/// Upstream `Plane::failsafe_check` heartbeat / lockup decision.
///
/// Advancing `scheduler.ticks()` is always healthy: stamp time, clear the
/// latch, and return. A stall older than [`FAILSAFE_CHECK_LOCKUP_US`]
/// raises `in_failsafe`. While latched, ages older than
/// [`FAILSAFE_CHECK_PASSTHROUGH_US`] request RC passthrough (and an AFS
/// heartbeat during calibration) and restamp `last_timestamp`.
#[must_use]
pub fn failsafe_check(state: &FailsafeCheckState, inp: &FailsafeCheckInputs) -> FailsafeCheckResult {
    if inp.scheduler_ticks != state.last_ticks {
        return FailsafeCheckResult {
            in_failsafe: false,
            pass_rc_through: false,
            afs_heartbeat: false,
            zero_throttle: false,
            last_ticks: inp.scheduler_ticks,
            last_timestamp_us: inp.now_us,
        };
    }

    let age_us = failsafe_check_age_us(inp.now_us, state.last_timestamp_us);
    let in_failsafe = state.in_failsafe || age_us > FAILSAFE_CHECK_LOCKUP_US;

    let mut last_timestamp_us = state.last_timestamp_us;
    let mut pass_rc_through = false;
    let mut afs_heartbeat = false;
    let mut zero_throttle = false;

    if in_failsafe && age_us > FAILSAFE_CHECK_PASSTHROUGH_US {
        last_timestamp_us = inp.now_us;
        afs_heartbeat = inp.in_calibration;
        if inp.valid_channel_count >= FAILSAFE_CHECK_MIN_RC_CHANNELS {
            pass_rc_through = true;
            zero_throttle = !inp.armed_and_safety_off;
        }
    }

    FailsafeCheckResult {
        in_failsafe,
        pass_rc_through,
        afs_heartbeat,
        zero_throttle,
        last_ticks: state.last_ticks,
        last_timestamp_us,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lockup_and_passthrough_ages_match_upstream() {
        assert_eq!(FAILSAFE_CHECK_LOCKUP_US, 200_000);
        assert_eq!(FAILSAFE_CHECK_PASSTHROUGH_US, 20_000);
        assert_eq!(FAILSAFE_CHECK_TIMER_PERIOD_US, 1_000);
        assert_eq!(FAILSAFE_CHECK_MIN_RC_CHANNELS, 5);
    }

    #[test]
    fn advancing_ticks_clears_lockup() {
        let state = FailsafeCheckState {
            last_ticks: 10,
            last_timestamp_us: 0,
            in_failsafe: true,
        };
        let out = failsafe_check(
            &state,
            &FailsafeCheckInputs {
                now_us: 500_000,
                scheduler_ticks: 11,
                ..FailsafeCheckInputs::default()
            },
        );
        assert!(!out.in_failsafe);
        assert!(!out.pass_rc_through);
        assert_eq!(out.last_ticks, 11);
        assert_eq!(out.last_timestamp_us, 500_000);
    }
}
