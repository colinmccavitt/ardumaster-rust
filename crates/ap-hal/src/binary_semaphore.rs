//! Binary semaphore, ported from `AP_HAL/Semaphores.h` `AP_HAL::BinarySemaphore`.
//!
//! Wait/signal used to wake a waiter from another context (including ISR).
//! This is a different surface from the recursive mutex in
//! [`crate::semaphore`] (`AP_HAL::Semaphore` take/give). Upstream documents
//! that `WITH_SEMAPHORE()` cannot be used with binary semaphores.
//!
//! # Timeouts
//!
//! `wait(0)` is the non-blocking try ([`BinarySemaphore::wait_nonblocking`]).
//! That is the opposite of [`crate::semaphore::Semaphore::take`]: mutex
//! `take(0)` is block-forever. Binary `wait` takes microseconds.
//!
//! This stub does not sleep; a mock either consumes a pending signal
//! immediately or refuses.

/// A binary wait/signal. Upstream `AP_HAL::BinarySemaphore`.
///
/// `wait` / `wait_blocking` / `wait_nonblocking` return `bool` because that
/// is what upstream returns. Widening them to [`crate::Result`] would be a
/// behavior change (ADR-0003).
pub trait BinarySemaphore {
    /// Wait for a signal, up to `timeout_us` microseconds.
    ///
    /// `timeout_us == 0` is non-blocking. Returns `true` if a signal was
    /// consumed. Upstream `wait()`.
    fn wait(&mut self, timeout_us: u32) -> bool;

    /// Block until a signal arrives. Upstream `wait_blocking()`.
    fn wait_blocking(&mut self) -> bool;

    /// Try once without waiting. Upstream `wait_nonblocking()`, default
    /// `wait(0)`.
    fn wait_nonblocking(&mut self) -> bool {
        self.wait(0)
    }

    /// Post one signal. A second post while one is already pending is a
    /// no-op (binary). Upstream `signal()`.
    fn signal(&mut self);

    /// Post from interrupt context. Default is [`signal`](Self::signal).
    /// Upstream `signal_ISR()`.
    fn signal_isr(&mut self) {
        self.signal();
    }
}

/// An in-memory binary semaphore for tests and SITL bring-up.
///
/// Single-threaded: [`wait`](BinarySemaphore::wait) grants immediately when
/// a signal is pending. There is no scheduler, so an unsignaled wait
/// refuses even with a positive timeout or [`wait_blocking`](BinarySemaphore::wait_blocking).
///
/// `initial_state` matches upstream: `true` means a wait right after
/// creation does not block; `false` means it does.
#[derive(Debug, Clone, Copy)]
pub struct MockBinarySemaphore {
    pending: bool,
}

impl MockBinarySemaphore {
    /// Create with upstream `initial_state` (`false` = first wait blocks).
    #[inline]
    pub fn new(initial_state: bool) -> Self {
        Self {
            pending: initial_state,
        }
    }

    /// Whether a signal is waiting to be consumed.
    #[inline]
    pub fn is_pending(&self) -> bool {
        self.pending
    }
}

impl Default for MockBinarySemaphore {
    fn default() -> Self {
        Self::new(false)
    }
}

impl BinarySemaphore for MockBinarySemaphore {
    fn wait(&mut self, timeout_us: u32) -> bool {
        if self.pending {
            self.pending = false;
            return true;
        }
        // Unsignaled: a positive timeout still cannot wait in this stub.
        // `0` is the non-blocking path (unlike mutex take(0) = forever).
        let _ = timeout_us;
        false
    }

    fn wait_blocking(&mut self) -> bool {
        if self.pending {
            self.pending = false;
            return true;
        }
        false
    }

    fn signal(&mut self) {
        self.pending = true;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wait_signal_round_trip() {
        let mut sem = MockBinarySemaphore::new(false);
        assert!(!sem.is_pending());
        assert!(!sem.wait_nonblocking());
        sem.signal();
        assert!(sem.is_pending());
        assert!(sem.wait(10));
        assert!(!sem.is_pending());
    }

    /// Upstream: `initial_state == true` means a wait after creation does
    /// not block.
    #[test]
    fn initial_state_true_is_already_signaled() {
        let mut sem = MockBinarySemaphore::new(true);
        assert!(sem.is_pending());
        assert!(sem.wait_nonblocking());
        assert!(!sem.is_pending());
        assert!(!sem.wait(0));
    }

    /// Binary: a second signal while one is already pending is not a count.
    #[test]
    fn signal_is_binary_not_counting() {
        let mut sem = MockBinarySemaphore::new(false);
        sem.signal();
        sem.signal();
        assert!(sem.wait_blocking());
        assert!(!sem.wait_nonblocking());
    }

    #[test]
    fn wait_blocking_grants_when_pending() {
        let mut sem = MockBinarySemaphore::new(false);
        assert!(!sem.wait_blocking());
        sem.signal_isr();
        assert!(sem.wait_blocking());
    }

    /// `wait(0)` is the non-blocking try, the opposite of mutex `take(0)`.
    #[test]
    fn wait_zero_is_nonblocking() {
        let mut sem = MockBinarySemaphore::default();
        assert!(!sem.wait(0));
        sem.signal();
        assert!(sem.wait(0));
    }

    /// The trait stays object-safe, which is what allows `&dyn` in the HAL
    /// context. If a future method breaks object safety this fails to compile
    /// here rather than at some distant call site.
    #[test]
    fn binary_semaphore_trait_is_object_safe() {
        let mut sem = MockBinarySemaphore::new(false);
        let s: &mut dyn BinarySemaphore = &mut sem;
        s.signal();
        assert!(s.wait_nonblocking());
        s.signal_isr();
        assert!(s.wait(1));
        assert!(!s.wait_blocking());
    }
}
