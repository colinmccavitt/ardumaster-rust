//! Internal error reporting, ported from `AP_InternalError`.
//!
//! Upstream raises `INTERNAL_ERROR(AP_InternalError::error_t::...)` at
//! conditions that are "shouldn't happen" but recoverable: the code carries on
//! with a safe fallback and records that it happened, so the event surfaces in
//! logs and pre-arm checks rather than vanishing.
//!
//! # Why not a global
//!
//! Upstream accumulates these in a singleton reached through `AP::internalerror()`.
//! ADR-0004 rules that out, and `no_std` without an allocator rules out anything
//! that grows. So this is a **plain bitmask value**: each error type is one bit,
//! errors are accumulated by OR-ing, and the owner decides where it lives.
//!
//! That makes reporting *pure*. A function that can report becomes
//! `fn f(..) -> (T, InternalErrors)` or takes `&mut InternalErrors`, and its
//! error behaviour is testable by inspecting the return value — no global to
//! reset between tests, no interior mutability, and it composes under `Copy`.
//!
//! # Call sites this exists for
//!
//! Four are already waiting, recorded as divergence D-004:
//!
//! | site | upstream error |
//! |---|---|
//! | `constrain_value` with NaN | `constraining_nan` |
//! | `QuaternionT::normalize` on a zero quaternion | `flow_of_control` |
//! | `calc_lowpass_alpha_dt` with negative dt or cutoff | `invalid_arg_or_result` |
//! | `Compass::consistent` (future, FW-014) | — |
//!
//! Behaviour at those sites is unchanged: they still return upstream's fallback
//! value. What changes is that the report is no longer dropped.

/// One bit per internal error condition. Upstream `AP_InternalError::error_t`.
///
/// Values are the port's own; upstream's numeric encoding is a logging detail
/// and is not part of the flight contract. Only the subset the port currently
/// raises is defined — this list grows with the port rather than being
/// speculatively complete.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum InternalError {
    /// A NaN reached a constraint. Upstream `constraining_nan`.
    ConstrainingNan = 1 << 0,
    /// Control reached a state believed unreachable. Upstream `flow_of_control`.
    FlowOfControl = 1 << 1,
    /// An argument or computed result was outside its valid domain.
    /// Upstream `invalid_arg_or_result`.
    InvalidArgOrResult = 1 << 2,
}

/// An accumulated set of internal errors.
///
/// `Copy` and allocation-free, so it can be returned by value from `no_std`
/// flight code. Accumulates rather than overwrites: upstream's semantics are
/// "record that this happened at least once", not "the most recent error".
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(transparent)]
pub struct InternalErrors(u32);

impl InternalErrors {
    /// An empty set.
    #[inline]
    pub const fn none() -> Self {
        Self(0)
    }

    /// Record that `e` occurred. Upstream `INTERNAL_ERROR(e)`.
    #[inline]
    pub fn report(&mut self, e: InternalError) {
        self.0 |= e as u32;
    }

    /// A set containing only `e`.
    #[inline]
    pub fn of(e: InternalError) -> Self {
        Self(e as u32)
    }

    /// Whether `e` has been recorded.
    #[inline]
    pub fn contains(self, e: InternalError) -> bool {
        self.0 & (e as u32) != 0
    }

    /// Whether nothing has been recorded.
    #[inline]
    pub fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// Merge another set into this one, for propagating up a call chain.
    #[inline]
    pub fn merge(&mut self, other: Self) {
        self.0 |= other.0;
    }

    /// The raw bitmask, for logging and pre-arm reporting.
    #[inline]
    pub fn bits(self) -> u32 {
        self.0
    }
}

impl core::ops::BitOr for InternalErrors {
    type Output = Self;
    #[inline]
    fn bitor(self, rhs: Self) -> Self {
        Self(self.0 | rhs.0)
    }
}

/// A value paired with any internal errors raised while producing it.
///
/// Lets a function preserve upstream's return value exactly while still
/// surfacing the report, instead of choosing between them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Reported<T> {
    /// The value upstream would have returned.
    pub value: T,
    /// Errors raised while computing it.
    pub errors: InternalErrors,
}

impl<T> Reported<T> {
    /// A value with no errors.
    #[inline]
    pub fn ok(value: T) -> Self {
        Self {
            value,
            errors: InternalErrors::none(),
        }
    }

    /// A value accompanied by one error.
    #[inline]
    pub fn with(value: T, e: InternalError) -> Self {
        Self {
            value,
            errors: InternalErrors::of(e),
        }
    }

    /// Take the value, merging any errors into `sink`.
    #[inline]
    pub fn take(self, sink: &mut InternalErrors) -> T {
        sink.merge(self.errors);
        self.value
    }
}

#[cfg(test)]
mod tests {
    // These assert exact pass-through of the upstream fallback value, which
    // is the property under test.
    #![allow(clippy::float_cmp)]

    use super::*;

    #[test]
    fn errors_accumulate_rather_than_overwrite() {
        let mut e = InternalErrors::none();
        assert!(e.is_empty());

        e.report(InternalError::ConstrainingNan);
        e.report(InternalError::FlowOfControl);
        // reporting the same condition twice is idempotent
        e.report(InternalError::ConstrainingNan);

        assert!(e.contains(InternalError::ConstrainingNan));
        assert!(e.contains(InternalError::FlowOfControl));
        assert!(!e.contains(InternalError::InvalidArgOrResult));
        assert!(!e.is_empty());
    }

    #[test]
    fn sets_merge_for_propagation_up_a_call_chain() {
        let mut outer = InternalErrors::of(InternalError::ConstrainingNan);
        let inner = InternalErrors::of(InternalError::InvalidArgOrResult);
        outer.merge(inner);
        assert!(outer.contains(InternalError::ConstrainingNan));
        assert!(outer.contains(InternalError::InvalidArgOrResult));

        let combined = InternalErrors::of(InternalError::FlowOfControl) | inner;
        assert!(combined.contains(InternalError::FlowOfControl));
        assert!(combined.contains(InternalError::InvalidArgOrResult));
    }

    /// The shape the four waiting call sites will use: upstream's fallback
    /// value is preserved, and the report travels with it.
    #[test]
    fn reported_preserves_the_upstream_return_value() {
        // e.g. constrain_value(NaN, 250, 500) returns the midpoint upstream
        let r = Reported::with(375.0_f32, InternalError::ConstrainingNan);
        let mut sink = InternalErrors::none();
        let v = r.take(&mut sink);

        assert_eq!(v, 375.0, "the upstream fallback value must be unchanged");
        assert!(sink.contains(InternalError::ConstrainingNan));

        let clean = Reported::ok(1.0_f32);
        let mut sink2 = InternalErrors::none();
        assert_eq!(clean.take(&mut sink2), 1.0);
        assert!(sink2.is_empty());
    }

    /// Allocation-free and cheap enough for a control loop.
    #[test]
    fn is_a_plain_copy_value() {
        assert_eq!(core::mem::size_of::<InternalErrors>(), 4);
        let a = InternalErrors::of(InternalError::FlowOfControl);
        let b = a; // Copy, not move
        assert_eq!(a, b);
    }
}
