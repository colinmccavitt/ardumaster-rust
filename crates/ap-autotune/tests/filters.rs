//! `AP_AutoTune::start` actuator / rate / target filter cutoff stub.

use ap_autotune::filters::{
    set_start_filter_cutoffs, start_filters, StartFilters, ACTUATOR_FILTER_HZ, RATE_FILTER_HZ,
    TARGET_FILTER_HZ,
};
use ap_filter::lowpass::LowPassFilterConstDtFloat;

fn close(got: f32, expect: f32) {
    assert!((got - expect).abs() < 1e-6, "{got} != {expect}");
}

#[test]
fn start_filter_cutoffs_match_upstream() {
    close(ACTUATOR_FILTER_HZ, 0.75);
    close(RATE_FILTER_HZ, 0.75);
    close(TARGET_FILTER_HZ, 4.0);
}

#[test]
fn configure_writes_cutoffs_at_scheduler_loop_rate() {
    for loop_rate_hz in [50.0, 400.0] {
        let filters = start_filters(loop_rate_hz);
        close(filters.actuator.get_cutoff_freq(), ACTUATOR_FILTER_HZ);
        close(filters.rate.get_cutoff_freq(), RATE_FILTER_HZ);
        close(filters.target.get_cutoff_freq(), TARGET_FILTER_HZ);
    }
}

#[test]
fn set_start_filter_cutoffs_matches_configure() {
    let mut actuator = LowPassFilterConstDtFloat::default();
    let mut rate = LowPassFilterConstDtFloat::default();
    let mut target = LowPassFilterConstDtFloat::default();
    set_start_filter_cutoffs(&mut actuator, &mut rate, &mut target, 50.0);

    let mut via_struct = StartFilters::new();
    via_struct.configure(50.0);

    close(
        actuator.get_cutoff_freq(),
        via_struct.actuator.get_cutoff_freq(),
    );
    close(rate.get_cutoff_freq(), via_struct.rate.get_cutoff_freq());
    close(
        target.get_cutoff_freq(),
        via_struct.target.get_cutoff_freq(),
    );
}

#[test]
fn start_resets_actuator_and_rate_but_not_target() {
    let mut filters = StartFilters::new();
    filters.actuator.reset_to(12.0);
    filters.rate.reset_to(-8.0);
    filters.target.reset_to(3.0);
    filters.configure(50.0);

    // reset() leaves the next sample unfiltered (first-sample seed).
    close(filters.actuator.apply(1.0), 1.0);
    close(filters.rate.apply(-2.0), -2.0);
    // target_filter is not reset in start — the seeded value stays.
    close(filters.target.get(), 3.0);
    let blended = filters.target.apply(0.0);
    assert!(
        blended < 3.0 && blended > 0.0,
        "target should stay seeded, got {blended}"
    );
}

#[test]
fn default_filters_start_unconfigured() {
    let filters = StartFilters::default();
    close(filters.actuator.get_cutoff_freq(), 0.0);
    close(filters.rate.get_cutoff_freq(), 0.0);
    close(filters.target.get_cutoff_freq(), 0.0);
}
