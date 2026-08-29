//! `AC_PID::set_notch_sample_rate` leftover (COP-008).

#![allow(clippy::float_cmp, reason = "exact values on exact inputs")]

use ap_pid::{AcPid, Filters, NotchFilterParams, PidGains, Scaling};

fn gains() -> PidGains {
    PidGains {
        p: 2.0,
        i: 0.0,
        d: 0.0,
        ff: 0.0,
        imax: 0.0,
        filt_t_hz: 0.0,
        filt_e_hz: 0.0,
        filt_d_hz: 0.0,
        ..PidGains::default()
    }
}

fn valid() -> NotchFilterParams {
    NotchFilterParams {
        center_freq_hz: 100.0,
        quality: 2.0,
        attenuation_db: 40.0,
    }
}

/// Stock PIDs have NTF=NEF=0. The leftover returns before allocating.
#[test]
fn zero_indices_leave_the_notches_absent() {
    let mut pid = AcPid::new(gains());
    let mut filters = Filters::new();
    filters.set(1, valid());
    pid.set_notch_sample_rate(400.0, &filters);
    assert!(pid.target_notch().is_none());
    assert!(pid.error_notch().is_none());
    assert_eq!(pid.notch_t_filter, 0);
    assert_eq!(pid.notch_e_filter, 0);
}

/// A missing filter allocates the notch but does not init or clear the index.
/// Upstream: `filter != nullptr && !setup` is the only disable path.
#[test]
fn a_null_lookup_keeps_an_uninitialised_notch() {
    let mut pid = AcPid::new(gains());
    pid.notch_t_filter = 1;
    pid.set_notch_sample_rate(400.0, &());
    let notch = pid.target_notch().expect("allocated");
    assert!(!notch.is_initialised());
    assert_eq!(pid.notch_t_filter, 1);
}

/// Zero params make setup fail, which drops the notch and clears the index.
#[test]
fn a_failed_setup_disables_the_index() {
    let mut pid = AcPid::new(gains());
    pid.notch_t_filter = 1;
    pid.notch_e_filter = 2;
    let mut filters = Filters::new();
    filters.set(
        1,
        NotchFilterParams {
            center_freq_hz: 0.0,
            quality: 2.0,
            attenuation_db: 40.0,
        },
    );
    filters.set(2, valid());
    pid.set_notch_sample_rate(400.0, &filters);
    assert!(pid.target_notch().is_none());
    assert_eq!(pid.notch_t_filter, 0);
    assert!(pid.error_notch().expect("E allocated").is_initialised());
    assert_eq!(pid.notch_e_filter, 2);
}

/// A valid lookup inits both notches at the sample rate.
#[test]
fn a_valid_lookup_inits_both_notches() {
    let mut pid = AcPid::new(gains());
    pid.notch_t_filter = 1;
    pid.notch_e_filter = 1;
    let mut filters = Filters::new();
    filters.set(1, valid());
    pid.set_notch_sample_rate(400.0, &filters);
    let t = pid.target_notch().expect("T");
    let e = pid.error_notch().expect("E");
    assert!(t.is_initialised());
    assert!(e.is_initialised());
    assert_eq!(t.sample_freq(), 400.0);
    assert_eq!(e.center_freq(), 100.0);
}

/// The target notch is applied before the target LPF. A freshly reset notch
/// passes the first sample; the next sample is filtered against that seed,
/// so the LPF sees a different input than a PID with no notch.
#[test]
fn update_all_applies_the_target_notch_before_the_lpf() {
    let mut g = gains();
    g.filt_t_hz = 1000.0;
    let mut filters = Filters::new();
    filters.set(1, valid());

    let mut notched = AcPid::new(g);
    notched.notch_t_filter = 1;
    notched.set_notch_sample_rate(400.0, &filters);
    let mut plain = AcPid::new(g);

    notched.update_all(1.0, 0.0, 0.0025, false, Scaling::default(), 0);
    plain.update_all(1.0, 0.0, 0.0025, false, Scaling::default(), 0);
    assert_eq!(
        notched.info().target,
        plain.info().target,
        "reset+apply must pass the seed through"
    );

    notched.update_all(0.0, 0.0, 0.0025, false, Scaling::default(), 3);
    plain.update_all(0.0, 0.0, 0.0025, false, Scaling::default(), 3);
    assert_ne!(
        notched.info().target,
        plain.info().target,
        "the notch must move the LPF's input"
    );
}

/// Same as the target test, on the error notch. A leftover that only wired
/// NTF would still pass the target test.
#[test]
fn update_all_applies_the_error_notch_before_the_lpf() {
    let mut g = gains();
    g.filt_e_hz = 1000.0;
    g.p = 1.0;
    let mut filters = Filters::new();
    filters.set(1, valid());

    let mut notched = AcPid::new(g);
    notched.notch_e_filter = 1;
    notched.set_notch_sample_rate(400.0, &filters);
    let mut plain = AcPid::new(g);

    notched.update_all(1.0, 0.0, 0.0025, false, Scaling::default(), 0);
    plain.update_all(1.0, 0.0, 0.0025, false, Scaling::default(), 0);
    assert_eq!(notched.info().error, plain.info().error);

    notched.update_all(1.0, 1.0, 0.0025, false, Scaling::default(), 3);
    plain.update_all(1.0, 1.0, 0.0025, false, Scaling::default(), 3);
    assert_ne!(
        notched.info().error,
        plain.info().error,
        "the error notch must move the LPF's input"
    );
}

/// An uninitialised (null-lookup) notch must pass through, so a leftover
/// that allocated and then failed to find FILT n does not change the PID.
#[test]
fn an_uninitialised_notch_does_not_change_update_all() {
    let mut with = AcPid::new(gains());
    with.notch_t_filter = 1;
    with.set_notch_sample_rate(400.0, &());
    let mut without = AcPid::new(gains());

    let out_w = with.update_all(3.0, 1.0, 0.01, false, Scaling::default(), 0);
    let out_o = without.update_all(3.0, 1.0, 0.01, false, Scaling::default(), 0);
    assert_eq!(out_w, out_o);
    assert_eq!(with.info(), without.info());
}
