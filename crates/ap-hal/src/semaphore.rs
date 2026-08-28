//! Semaphores, ported from `AP_HAL/Semaphores.h`.
//!
//! Mutex-style take/give used to serialize shared HAL and library state.
//! Binary semaphores (`AP_HAL::BinarySemaphore`) are a separate wait/signal
//! surface and are not in this stub.
//!
//! Upstream documents that every `AP_HAL::Semaphore` is recursive: the thread
//! that holds it may take again, and must give the same number of times. The
//! port keeps that contract so a nested critical section cannot deadlock a
//! later board backend that implements it.
//!
//! # Blocking
//!
//! `take(0)` is "block forever" (`HAL_SEMAPHORE_BLOCK_FOREVER`), not a
//! zero-timeout try. The non-blocking path is [`Semaphore::take_nonblocking`].
//! This stub does not sleep; a mock either grants immediately or refuses.

/// Timeout that means block forever. Upstream `HAL_SEMAPHORE_BLOCK_FOREVER`.
pub const BLOCK_FOREVER_MS: u32 = 0;

/// A recursive mutex. Upstream `AP_HAL::Semaphore`.
///
/// `take` / `take_nonblocking` / `give` return `bool` because that is what
/// upstream returns. Widening them to [`crate::Result`] would be a behavior
/// change (ADR-0003).
pub trait Semaphore {
    /// Take the semaphore, waiting up to `timeout_ms`.
    ///
    /// `timeout_ms == `[`BLOCK_FOREVER_MS`] blocks forever. Returns `true` if
    /// the lock was acquired. Upstream `take()`.
    fn take(&mut self, timeout_ms: u32) -> bool;

    /// Try once without waiting. Upstream `take_nonblocking()`.
    fn take_nonblocking(&mut self) -> bool;

    /// Block until taken. Upstream `take_blocking()`.
    fn take_blocking(&mut self) {
        let _taken = self.take(BLOCK_FOREVER_MS);
    }

    /// Release one take. Returns `true` if this call released a held take.
    /// Upstream `give()`.
    fn give(&mut self) -> bool;
}

/// An in-memory recursive semaphore for tests and SITL bring-up.
///
/// Single-threaded: [`take`](Semaphore::take) always grants when uncontended
/// (the mock never waits). Set [`contended`](MockSemaphore::set_contended) to
/// simulate another owner so [`take_nonblocking`](Semaphore::take_nonblocking)
/// can fail.
#[derive(Debug, Clone, Copy)]
pub struct MockSemaphore {
    depth: u32,
    contended: bool,
}

impl Default for MockSemaphore {
    fn default() -> Self {
        Self {
            depth: 0,
            contended: false,
        }
    }
}

impl MockSemaphore {
    /// A free, uncontended semaphore.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// Recursive take count. Zero means free.
    #[inline]
    pub fn depth(&self) -> u32 {
        self.depth
    }

    /// Whether another owner is simulated as holding the lock.
    #[inline]
    pub fn is_contended(&self) -> bool {
        self.contended
    }

    /// Simulate another owner (`true`) or return to uncontended (`false`).
    ///
    /// Contended only applies while this mock is free. A recursive take from
    /// the current holder still succeeds.
    #[inline]
    pub fn set_contended(&mut self, contended: bool) {
        self.contended = contended;
    }

    fn try_acquire(&mut self) -> bool {
        if self.depth == 0 && self.contended {
            return false;
        }
        self.depth = self.depth.saturating_add(1);
        true
    }
}

impl Semaphore for MockSemaphore {
    fn take(&mut self, timeout_ms: u32) -> bool {
        if self.try_acquire() {
            return true;
        }
        // Contended and free: a positive timeout still cannot wait in this
        // stub. Block-forever (`0`) also refuses — a mock has no scheduler
        // to wake it. A board backend would sleep here.
        let _ = timeout_ms;
        false
    }

    fn take_nonblocking(&mut self) -> bool {
        self.try_acquire()
    }

    fn give(&mut self) -> bool {
        if self.depth == 0 {
            return false;
        }
        self.depth -= 1;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn take_give_round_trip() {
        let mut sem = MockSemaphore::new();
        assert_eq!(sem.depth(), 0);
        assert!(sem.take(10));
        assert_eq!(sem.depth(), 1);
        assert!(sem.give());
        assert_eq!(sem.depth(), 0);
    }

    #[test]
    fn take_nonblocking_grants_when_free() {
        let mut sem = MockSemaphore::new();
        assert!(sem.take_nonblocking());
        assert_eq!(sem.depth(), 1);
        assert!(sem.give());
    }

    #[test]
    fn take_nonblocking_fails_when_contended() {
        let mut sem = MockSemaphore::new();
        sem.set_contended(true);
        assert!(sem.is_contended());
        assert!(!sem.take_nonblocking());
        assert_eq!(sem.depth(), 0);
        assert!(!sem.take(5));
        assert!(!sem.take(BLOCK_FOREVER_MS));
    }

    /// Recursive: the holder may take again and must give the same number
    /// of times. Upstream documents this on every `AP_HAL::Semaphore`.
    #[test]
    fn recursive_take_needs_matching_gives() {
        let mut sem = MockSemaphore::new();
        assert!(sem.take(1));
        assert!(sem.take_nonblocking());
        assert_eq!(sem.depth(), 2);
        assert!(sem.give());
        assert_eq!(sem.depth(), 1);
        assert!(sem.give());
        assert_eq!(sem.depth(), 0);
        assert!(!sem.give());
    }

    #[test]
    fn take_blocking_grants_when_free() {
        let mut sem = MockSemaphore::new();
        sem.take_blocking();
        assert_eq!(sem.depth(), 1);
        assert!(sem.give());
    }

    /// The trait stays object-safe, which is what allows `&dyn` in the HAL
    /// context. If a future method breaks object safety this fails to compile
    /// here rather than at some distant call site.
    #[test]
    fn semaphore_trait_is_object_safe() {
        let mut sem = MockSemaphore::new();
        let s: &mut dyn Semaphore = &mut sem;
        assert!(s.take_nonblocking());
        assert!(s.take(BLOCK_FOREVER_MS));
        assert!(s.give());
        assert!(s.give());
        assert!(!s.give());
    }
}
