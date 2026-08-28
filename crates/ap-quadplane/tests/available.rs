//! `Q_ENABLE` / `enabled()` — upstream `QuadPlane::enabled`.
//!
//! `available()` is `initialised` (set by `setup()`), not a synonym
//! for `enabled()`. These tests cover the enable parameter only.

use ap_quadplane::{QuadPlane, Q_ENABLE_DEFAULT};

#[test]
fn default_q_enable_is_zero_and_not_live() {
    let qp = QuadPlane::new();
    assert_eq!(qp.enable(), Q_ENABLE_DEFAULT);
    assert_eq!(qp.enable(), 0);
    assert!(!qp.enabled());
    assert!(!qp.initialised());
    assert!(!qp.available());
}

#[test]
fn q_enable_one_is_enabled_but_not_available_until_setup() {
    let qp = QuadPlane::with_enable(1);
    assert!(qp.enabled());
    assert!(!qp.initialised());
    assert!(!qp.available());
}

#[test]
fn q_enable_two_vtol_auto_is_still_enabled() {
    // Upstream @Values: 0:Disable,1:Enable,2:Enable VTOL AUTO.
    // `enabled()` is only `enable != 0`; value 2 is live.
    let qp = QuadPlane::with_enable(2);
    assert!(qp.enabled());
    assert!(!qp.available());
}

#[test]
fn q_enable_zero_after_enable_is_not_enabled() {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.enabled());
    qp.set_enable(0);
    assert!(!qp.enabled());
    assert!(!qp.available());
}

#[test]
fn any_nonzero_q_enable_is_enabled() {
    // Upstream compares against zero, not against the documented 1/2 values.
    let qp = QuadPlane::with_enable(-1);
    assert!(qp.enabled());
    assert!(!qp.available());
}
