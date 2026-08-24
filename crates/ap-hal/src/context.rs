//! The HAL context, ported from `AP_HAL/HAL.h`.
//!
//! This is the mechanism ADR-0004 decision 5 calls for: upstream reaches its
//! subsystems through 114 global accessors (`AP::foo()`), and the port replaces
//! all of them with one explicitly threaded context.
//!
//! # Why trait objects rather than generics
//!
//! `Hal` holds `&dyn Trait` rather than being generic over each subsystem.
//! Generics would mean a type parameter per subsystem — a dozen of them —
//! infecting every signature that touches the HAL.
//!
//! The usual argument for generics is avoiding virtual dispatch, and here it
//! does not apply: upstream's `AP_HAL::UARTDriver*`, `Storage*`, `RCInput*` and
//! the rest are **already abstract base classes**, so every HAL call in
//! ArduPilot is a virtual call today. `&dyn` costs exactly what upstream costs,
//! and the port pays nothing for the simpler types.
//!
//! # What this does and does not guarantee
//!
//! What it does give: the borrow checker can see the wiring. Two subsystems
//! cannot hold simultaneous mutable access to a third, aliasing is checked at
//! compile time, lifetimes tie every subsystem to a scope that must outlive its
//! users, and the set of subsystems in existence is visible in one place rather
//! than scattered across 114 accessors.
//!
//! What it does **not** give, and an earlier draft of this file wrongly claimed
//! it did: holding `&Hal` is not a read-only capability. The fields are
//! `&mut dyn Trait`, and mutating *through* a `&mut` does not require the
//! binding to be mutable — so a function taking `&Hal` can still drive servos.
//! Passing the context is passing broad authority, whether the reference is
//! shared or exclusive.
//!
//! That is why the guidance below matters rather than being a style
//! preference: **narrowing capability is done by narrowing parameters, not by
//! sharing the context.** A function that takes `&dyn Clock` genuinely cannot
//! touch anything else; a function that takes `&Hal` can touch everything.
//! Compared with `AP::foo()` this is still a large improvement — authority is
//! at least explicit in the signature and bounded by lifetimes — but it is a
//! weaker property than "the signature states what it can touch", and the
//! difference is worth being accurate about.
//!
//! # Prefer narrow parameters over the whole context
//!
//! A function that only needs the clock should take `&dyn Clock`, not
//! `&mut Hal`. That keeps its signature honest and its tests trivial — see
//! `tests::narrow_dependencies_are_expressible` below. Threading the whole
//! context is for the scheduler and vehicle layer, not for leaf algorithms.

use crate::internal_error::InternalErrors;
use crate::rc::{RcInput, RcOutput};
use crate::storage::Storage;
use crate::time::{Clock, Delay};

/// The hardware abstraction context. Upstream `AP_HAL::HAL`.
///
/// Upstream's `HAL` is a bundle of subsystem pointers constructed once per
/// board and reached globally. This holds the same bundle, but is passed
/// explicitly so ownership and aliasing stay visible.
///
/// Subsystems are added as their tickets land; this is the SITL subset needed
/// for fixed-wing bring-up, not the full board surface.
pub struct Hal<'a> {
    /// Monotonic time. Shared, so `&dyn`.
    pub clock: &'a dyn Clock,
    /// Blocking delays. Separate from `clock` so time-reading code cannot
    /// accidentally gain the ability to block.
    pub delay: &'a dyn Delay,
    /// Persistent storage, backing parameters and missions.
    pub storage: &'a mut dyn Storage,
    /// Receiver input.
    pub rcin: &'a mut dyn RcInput,
    /// Servo and motor output.
    pub rcout: &'a mut dyn RcOutput,
    /// Internal errors accumulated this run.
    ///
    /// Lives here rather than in a global because upstream's
    /// `AP::internalerror()` singleton is exactly what ADR-0004 forbids. Code
    /// that can report takes `&mut InternalErrors` directly; this is where the
    /// vehicle layer keeps the accumulated set.
    pub errors: InternalErrors,
}

impl<'a> Hal<'a> {
    /// Assemble a context from its parts.
    pub fn new(
        clock: &'a dyn Clock,
        delay: &'a dyn Delay,
        storage: &'a mut dyn Storage,
        rcin: &'a mut dyn RcInput,
        rcout: &'a mut dyn RcOutput,
    ) -> Self {
        Self {
            clock,
            delay,
            storage,
            rcin,
            rcout,
            errors: InternalErrors::none(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::internal_error::InternalError;
    use crate::rc::{MockRcInput, MockRcOutput};
    use crate::storage::RamStorage;
    use crate::time::{Micros, Millis};
    use core::cell::Cell;

    struct TestClock {
        ms: Cell<u32>,
    }

    impl TestClock {
        fn new() -> Self {
            Self { ms: Cell::new(0) }
        }
        fn advance(&self, ms: u32) {
            self.ms.set(self.ms.get().wrapping_add(ms));
        }
    }

    impl Clock for TestClock {
        fn millis(&self) -> Millis {
            Millis(self.ms.get())
        }
        fn micros(&self) -> Micros {
            Micros(self.ms.get().wrapping_mul(1000))
        }
        fn millis64(&self) -> u64 {
            self.ms.get() as u64
        }
        fn micros64(&self) -> u64 {
            self.ms.get() as u64 * 1000
        }
    }

    struct NoDelay;
    impl Delay for NoDelay {
        fn delay_ms(&self, _ms: u16) {}
        fn delay_us(&self, _us: u16) {}
    }

    /// A leaf algorithm should take only what it needs, not the whole context.
    /// This is the habit that keeps signatures honest as the port grows.
    fn elapsed_since_boot(clock: &dyn Clock) -> u32 {
        clock.millis().0
    }

    /// A stand-in for the vehicle-layer tick: reads RC, uses the clock, writes
    /// servos, and records an internal error - all through the context, with no
    /// globals anywhere.
    fn fake_scheduler_tick(hal: &mut Hal) -> Option<u16> {
        if !hal.rcin.new_input() {
            return None;
        }
        let roll_in = hal.rcin.read(0)?;
        let _now = hal.clock.millis();

        // pretend a computation produced a NaN and had to be constrained
        hal.errors.report(InternalError::ConstrainingNan);

        hal.rcout.write(0, roll_in).ok()?;
        hal.rcout.push();
        hal.rcout.read(0)
    }

    /// ADR-0004 decision 5, demonstrated end to end: a full tick with no
    /// singletons, no interior mutability in the subsystems, and no global
    /// state. This is the test that says the decision is workable.
    #[test]
    fn context_threads_through_a_tick_without_singletons() {
        let clock = TestClock::new();
        let delay = NoDelay;
        let mut storage = RamStorage::<64>::new();
        let mut rcin = MockRcInput::new();
        let mut rcout = MockRcOutput::new();

        rcin.set_input_frame(&[1600, 1500, 1000, 1500]);

        let mut hal = Hal::new(&clock, &delay, &mut storage, &mut rcin, &mut rcout);

        let out = fake_scheduler_tick(&mut hal);
        assert_eq!(out, Some(1600), "servo output should follow RC input");
        assert!(hal.errors.contains(InternalError::ConstrainingNan));

        // second tick: no new frame, so the tick declines to act
        assert_eq!(fake_scheduler_tick(&mut hal), None);

        clock.advance(20);
        assert_eq!(hal.clock.millis(), Millis(20));
    }

    /// Disjoint field borrows work, so one call can read the clock while
    /// another mutates an output. This is what makes the struct usable rather
    /// than a borrow-checker fight.
    /// Also pins the capability caveat: this binding is not `mut`, yet the
    /// subsystems are still mutated. `&Hal` is not read-only.
    #[test]
    fn disjoint_subsystem_borrows_are_allowed() {
        let clock = TestClock::new();
        let delay = NoDelay;
        let mut storage = RamStorage::<32>::new();
        let mut rcin = MockRcInput::new();
        let mut rcout = MockRcOutput::new();
        // Deliberately not `mut`: mutating through the `&mut dyn` fields does
        // not require a mutable binding. That is the caveat documented at the
        // top of this module, pinned here so it cannot regress silently.
        let hal = Hal::new(&clock, &delay, &mut storage, &mut rcin, &mut rcout);

        // immutable clock borrow held across a mutable rcout borrow
        let t = hal.clock.millis();
        hal.rcout.write(1, 1234).unwrap();
        assert_eq!(hal.rcout.read(1), Some(1234));
        assert_eq!(t, Millis(0));

        // storage and rcout mutated in the same scope
        hal.storage.write_block(0, &[1, 2, 3]).unwrap();
        hal.rcout.write(2, 1500).unwrap();
        let mut buf = [0u8; 3];
        hal.storage.read_block(&mut buf, 0).unwrap();
        assert_eq!(buf, [1, 2, 3]);
    }

    /// A leaf function declares exactly the capability it uses, and is testable
    /// with nothing but a clock.
    #[test]
    fn narrow_dependencies_are_expressible() {
        let clock = TestClock::new();
        clock.advance(4321);
        assert_eq!(elapsed_since_boot(&clock), 4321);
    }

    /// The traits stay object-safe, which is what allows `&dyn` at all. If a
    /// future method breaks object safety this fails to compile here rather
    /// than at some distant call site.
    #[test]
    fn hal_traits_are_object_safe() {
        let clock = TestClock::new();
        let mut storage = RamStorage::<8>::new();
        let mut rcin = MockRcInput::new();
        let mut rcout = MockRcOutput::new();

        let _c: &dyn Clock = &clock;
        let _d: &dyn Delay = &NoDelay;
        let _s: &mut dyn Storage = &mut storage;
        let _i: &mut dyn RcInput = &mut rcin;
        let _o: &mut dyn RcOutput = &mut rcout;
    }
}
