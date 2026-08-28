//! `AP_AutoTune` slew_limit / SlewLimiter stub (default 150 deg/s,
//! P/D slew-rate tracking).

use ap_autotune::completeness::{completeness_has, PortStatus};
use ap_autotune::slew::{
    floor_slew_limit, peak_slew_rate, scale_pd_sample, slew_limit_params, PdSlewTrackers,
    SLEW_LIMIT_DEFAULT, SLEW_LIMIT_SCALE, SLEW_LIMIT_TAU,
};
use ap_autotune::state::{AtType, AutoTune};

fn close(got: f32, expect: f32) {
    assert!((got - expect).abs() < 1e-5, "{got} != {expect}");
}

#[test]
fn slew_limit_constants_match_upstream() {
    close(SLEW_LIMIT_DEFAULT, 150.0);
    close(SLEW_LIMIT_TAU, 1.0);
    close(SLEW_LIMIT_SCALE, 45.0 / (180.0 / core::f32::consts::PI));
}

#[test]
fn start_floors_non_positive_slew_limit_to_150() {
    close(floor_slew_limit(0.0), SLEW_LIMIT_DEFAULT);
    close(floor_slew_limit(-10.0), SLEW_LIMIT_DEFAULT);
    close(floor_slew_limit(75.0), 75.0);
    close(floor_slew_limit(150.0), 150.0);

    let mut tuner = AutoTune::new(AtType::Roll);
    close(tuner.slew_limit, 0.0);
    tuner.start();
    close(tuner.slew_limit, SLEW_LIMIT_DEFAULT);
    assert!(tuner.running);

    tuner.slew_limit = 80.0;
    tuner.start();
    close(tuner.slew_limit, 80.0);
}

#[test]
fn scale_pd_sample_undoes_dmod_then_applies_45_over_degrees() {
    close(scale_pd_sample(1.0, 1.0), SLEW_LIMIT_SCALE);
    close(scale_pd_sample(0.20, 0.50), (0.20 / 0.50) * SLEW_LIMIT_SCALE);
    close(scale_pd_sample(0.10, 0.0), 0.10 * SLEW_LIMIT_SCALE);
}

#[test]
fn slew_limit_params_use_live_max_and_tau_one() {
    let params = slew_limit_params(150.0);
    close(params.slew_rate_max, 150.0);
    close(params.slew_rate_tau, SLEW_LIMIT_TAU);
}

#[test]
fn peak_slew_rate_holds_the_cycle_maximum() {
    close(peak_slew_rate(0.0, 3.0), 3.0);
    close(peak_slew_rate(3.0, 1.0), 3.0);
    close(peak_slew_rate(3.0, 5.0), 5.0);
}

#[test]
fn pd_slew_trackers_record_p_and_d_peaks() {
    let mut trackers = PdSlewTrackers::new();
    close(trackers.max_srate_p, 0.0);
    close(trackers.max_srate_d, 0.0);
    close(trackers.slew_limit_tau, SLEW_LIMIT_TAU);

    for i in 0..40u32 {
        let t = 1_000 + i * 20;
        let sample = if i % 2 == 0 { 4.0 } else { -4.0 };
        trackers.update(sample, sample * 0.25, 1.0, 0.02, t, SLEW_LIMIT_DEFAULT);
    }

    assert!(
        trackers.max_srate_p > 0.0,
        "P slew peak should rise, got {}",
        trackers.max_srate_p
    );
    assert!(
        trackers.max_srate_d > 0.0,
        "D slew peak should rise, got {}",
        trackers.max_srate_d
    );
    assert!(
        trackers.max_srate_p > trackers.max_srate_d,
        "larger P samples should peak above D"
    );
    close(trackers.slew_limit_max, SLEW_LIMIT_DEFAULT);
    close(trackers.slew_limit_tau, SLEW_LIMIT_TAU);

    let held_p = trackers.max_srate_p;
    trackers.reset_cycle_peaks();
    close(trackers.max_srate_p, 0.0);
    close(trackers.max_srate_d, 0.0);
    assert!(held_p > 0.0);
}

#[test]
fn completeness_lists_slew_limit_this_slice() {
    assert!(completeness_has(
        "slew_limit / SlewLimiter",
        PortStatus::ThisSlice
    ));
    assert!(!completeness_has(
        "slew_limit / SlewLimiter",
        PortStatus::Remaining
    ));
}
