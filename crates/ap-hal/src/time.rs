//! Time sources, ported from `AP_HAL/system.h` and `AP_HAL/Scheduler.h`.
//!
//! Upstream exposes time as free functions in the `AP_HAL` namespace —
//! `millis()`, `micros()`, `millis64()`, `micros64()` — backed by a global
//! clock. That is the singleton pattern ADR-0004 forbids, so here time is a
//! [`Clock`] trait obtained from the HAL context and passed explicitly.
//!
//! # Why this matters beyond style
//!
//! `SlewLimiter` in `ap-filter` already takes `now_ms` as a parameter for this
//! reason, and that is what let its startup behavior be unit-tested without
//! mocking a clock — the test that pins divergence D-006. A global clock would
//! have made that test impossible to write.

/// Milliseconds since boot, wrapping at `u32::MAX` (~49.7 days).
///
/// A newtype rather than a bare `u32`, because the wrap is load-bearing:
/// upstream's elapsed-time comparisons rely on unsigned wraparound, and mixing
/// a timestamp with a duration by accident is a real porting hazard.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
#[repr(transparent)]
pub struct Millis(pub u32);

/// Microseconds since boot, wrapping at `u32::MAX` (~71.6 minutes).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
#[repr(transparent)]
pub struct Micros(pub u32);

impl Millis {
    /// Milliseconds elapsed since `earlier`, using wrapping arithmetic.
    ///
    /// Mirrors upstream's `now - past_time` on unsigned types, which is well
    /// defined in C++ and relied upon across the rollover.
    #[inline]
    pub fn since(self, earlier: Millis) -> u32 {
        self.0.wrapping_sub(earlier.0)
    }

    /// Whether `timeout` ms have elapsed since `past`.
    ///
    /// Upstream `AP_HAL::timeout_expired()`, which static-asserts that both
    /// operands are the same unsigned type — a constraint the newtype makes
    /// structural rather than a compile-time check.
    #[inline]
    pub fn timeout_expired(self, past: Millis, timeout: u32) -> bool {
        self.since(past) >= timeout
    }

    /// Milliseconds remaining before `timeout` elapses since `past`, saturating
    /// at zero. Upstream `AP_HAL::timeout_remaining()`.
    #[inline]
    pub fn timeout_remaining(self, past: Millis, timeout: u32) -> u32 {
        timeout.saturating_sub(self.since(past))
    }
}

impl Micros {
    /// Microseconds elapsed since `earlier`, using wrapping arithmetic.
    #[inline]
    pub fn since(self, earlier: Micros) -> u32 {
        self.0.wrapping_sub(earlier.0)
    }

    /// Whether `timeout` us have elapsed since `past`.
    #[inline]
    pub fn timeout_expired(self, past: Micros, timeout: u32) -> bool {
        self.since(past) >= timeout
    }
}

/// A monotonic time source. Upstream's `AP_HAL::millis()` family.
///
/// The 64-bit variants do not wrap and are what long-horizon logic should use;
/// the 32-bit ones match upstream's hot-path calls, where the wrap is expected
/// and handled.
pub trait Clock {
    /// Milliseconds since boot, wrapping. Upstream `millis()`.
    fn millis(&self) -> Millis;

    /// Microseconds since boot, wrapping. Upstream `micros()`.
    fn micros(&self) -> Micros;

    /// Milliseconds since boot, non-wrapping. Upstream `millis64()`.
    fn millis64(&self) -> u64;

    /// Microseconds since boot, non-wrapping. Upstream `micros64()`.
    fn micros64(&self) -> u64;
}

/// Blocking delays. Upstream `AP_HAL::Scheduler`.
///
/// Separate from [`Clock`] on purpose: most flight code needs to *read* time
/// and must never block. Splitting the traits means a module that only reads
/// the clock cannot accidentally acquire the ability to sleep in a control
/// loop.
pub trait Delay {
    /// Block for `ms` milliseconds. Upstream `Scheduler::delay()`.
    fn delay_ms(&self, ms: u16);

    /// Block for `us` microseconds. Upstream `Scheduler::delay_microseconds()`.
    fn delay_us(&self, us: u16);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A test clock, which is the point of the trait: no global to mock.
    struct FixedClock(u64);

    impl Clock for FixedClock {
        fn millis(&self) -> Millis {
            Millis(self.0 as u32)
        }
        fn micros(&self) -> Micros {
            Micros((self.0 * 1000) as u32)
        }
        fn millis64(&self) -> u64 {
            self.0
        }
        fn micros64(&self) -> u64 {
            self.0 * 1000
        }
    }

    #[test]
    fn elapsed_uses_wrapping_arithmetic() {
        let now = Millis(5);
        let past = Millis(u32::MAX - 4);
        // 10 ms really elapsed across the rollover
        assert_eq!(now.since(past), 10);
        assert!(now.timeout_expired(past, 10));
        assert!(!now.timeout_expired(past, 11));
        assert_eq!(now.timeout_remaining(past, 25), 15);
        assert_eq!(now.timeout_remaining(past, 5), 0, "must saturate, not wrap");
    }

    #[test]
    fn micros_elapsed_wraps_too() {
        let now = Micros(100);
        let past = Micros(u32::MAX - 99);
        assert_eq!(now.since(past), 200);
        assert!(now.timeout_expired(past, 200));
    }

    #[test]
    fn clock_trait_is_mockable_without_globals() {
        let c = FixedClock(1234);
        assert_eq!(c.millis(), Millis(1234));
        assert_eq!(c.millis64(), 1234);
        assert_eq!(c.micros64(), 1_234_000);
    }
}
