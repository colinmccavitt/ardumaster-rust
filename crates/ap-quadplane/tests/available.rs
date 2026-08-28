//! `Q_ENABLE` / `available()` — upstream `QuadPlane::enabled` / `available`.
//!
//! `Q_ENABLE != 0` means the QuadPlane object is live.

use ap_quadplane::{QuadPlane, Q_ENABLE_DEFAULT};

#[test]
fn default_q_enable_is_zero_and_not_live() {
    let qp = QuadPlane::new();
    assert_eq!(qp.enable(), Q_ENABLE_DEFAULT);
    assert_eq!(qp.enable(), 0);
    assert!(!qp.enabled());
    assert!(!qp.available());
}

#[test]
fn q_enable_one_is_enabled_and_available() {
    let qp = QuadPlane::with_enable(1);
    assert!(qp.enabled());
    assert!(qp.available());
}

#[test]
fn q_enable_two_vtol_auto_is_still_enabled() {
    // Upstream @Values: 0:Disable,1:Enable,2:Enable VTOL AUTO.
    // `enabled()` is only `enable != 0`; value 2 is live.
    let qp = QuadPlane::with_enable(2);
    assert!(qp.enabled());
    assert!(qp.available());
}

#[test]
fn q_enable_zero_after_enable_is_not_live() {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.available());
    qp.set_enable(0);
    assert!(!qp.enabled());
    assert!(!qp.available());
}

#[test]
fn any_nonzero_q_enable_is_live() {
    // Upstream compares against zero, not against the documented 1/2 values.
    let qp = QuadPlane::with_enable(-1);
    assert!(qp.enabled());
    assert!(qp.available());
}
