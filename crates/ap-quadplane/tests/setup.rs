//! `QuadPlane::setup` / motors-init — upstream `QuadPlane::setup`.
//!
//! When `Q_ENABLE != 0`, setup initialises motors and sets
//! `initialised`. `available()` then returns that flag.

use ap_quadplane::QuadPlane;

#[test]
fn setup_without_q_enable_fails_and_does_not_init_motors() {
    let mut qp = QuadPlane::new();
    assert!(!qp.setup());
    assert!(!qp.initialised());
    assert!(!qp.motors_inited());
    assert!(!qp.available());
}

#[test]
fn setup_with_q_enable_inits_motors_and_becomes_available() {
    let mut qp = QuadPlane::with_enable(1);
    assert!(!qp.available());
    assert!(qp.setup());
    assert!(qp.motors_inited());
    assert!(qp.initialised());
    assert!(qp.available());
}

#[test]
fn setup_is_idempotent_once_initialised() {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    assert!(qp.setup());
    assert!(qp.initialised());
    assert!(qp.motors_inited());
    assert!(qp.available());
}

#[test]
fn q_enable_two_vtol_auto_setup_also_inits() {
    let mut qp = QuadPlane::with_enable(2);
    assert!(qp.setup());
    assert!(qp.motors_inited());
    assert!(qp.available());
}

#[test]
fn available_is_initialised_not_enabled() {
    // Upstream `available()` is only `return initialised`. Clearing
    // `Q_ENABLE` after a successful setup leaves the object available.
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp.set_enable(0);
    assert!(!qp.enabled());
    assert!(qp.initialised());
    assert!(qp.available());
    assert!(qp.motors_inited());
}

#[test]
fn setup_after_failed_enable_can_succeed_once_enabled() {
    let mut qp = QuadPlane::new();
    assert!(!qp.setup());
    qp.set_enable(1);
    assert!(qp.setup());
    assert!(qp.available());
}
