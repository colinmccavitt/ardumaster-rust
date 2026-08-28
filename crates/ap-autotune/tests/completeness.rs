//! FW-040 completeness: AutoTune already on main vs remaining
//! `AP_AutoTune.cpp` gaps (yaw axis, IMAX on start, N-cycle save).

use ap_autotune::completeness::{
    att_limit_deg, completeness_counts, completeness_has, completeness_unique_names, on_main_items,
    remaining_items, should_save_after_cycles, start_constrain_imax, start_imax_band,
    this_slice_items, AutotunePortItem, PortStatus, AUTOTUNE_COMPLETENESS, DONE_COUNT_SAVE,
    YAW_ATT_LIMIT_DEG,
};
use ap_autotune::ff::{constrain_imax, AUTOTUNE_MAX_IMAX, AUTOTUNE_MIN_IMAX};
use ap_autotune::gains::AtGains;
use ap_autotune::options::{AutotuneAxes, AUTOTUNE_AXIS_YAW};
use ap_autotune::state::{rate_threshold1, AtType, AutoTune};

/// Surfaces already on main — do not redo these slices.
const ON_MAIN: &[&str] = &[
    "autotune_mode_hookup",
    "ATState Idle/DemandPos/DemandNeg",
    "AUTOTUNE_LEVEL aggressiveness table",
    "save_gains / restore_gains",
    "update_gains saturation/overshoot",
    "AUTOTUNE_OPTIONS FLTD/FLTT",
    "AUTOTUNE_AXES start mask",
    "I-term / FF coupling",
    "FF estimate / ff_filter",
    "start() zero-FF floor 0.01",
    "actuator/rate/target LPF cutoffs",
];

const THIS_SLICE: &[&str] = &[
    "completeness table",
    "IMAX constrain on start",
    "yaw att_limit 20 deg",
    "save_gains after N stable cycles",
    "Action / D-limit hunting",
    "slew_limit / SlewLimiter",
];

const REMAINING: &[&str] = &[
    "log_ATRP 25Hz",
    "EEPROM save_*_if_changed",
    "update_rmax FF/I inverse-tau",
    "LOW_RATE / SHORT event rejects",
    "clipped actuator without I",
];

fn close(got: f32, expect: f32) {
    assert!((got - expect).abs() < 1e-6, "{got} != {expect}");
}

fn sample_gains() -> AtGains {
    AtGains {
        tau: 0.5,
        rmax_pos: 75.0,
        rmax_neg: 75.0,
        p: 0.4,
        i: 0.3,
        d: 0.02,
    }
}

fn tuned(base: AtGains) -> AtGains {
    AtGains {
        tau: base.tau * 0.8,
        rmax_pos: base.rmax_pos + 20.0,
        rmax_neg: base.rmax_neg + 20.0,
        p: base.p * 1.2,
        i: base.i * 1.1,
        d: base.d * 0.5,
    }
}

#[test]
fn completeness_table_matches_main_versus_remaining() {
    assert!(completeness_unique_names());
    assert_eq!(
        AUTOTUNE_COMPLETENESS.len(),
        ON_MAIN.len() + THIS_SLICE.len() + REMAINING.len()
    );
    let (on_main, this_slice, remaining) = completeness_counts();
    assert_eq!(on_main, ON_MAIN.len());
    assert_eq!(this_slice, THIS_SLICE.len());
    assert_eq!(remaining, REMAINING.len());
    for name in ON_MAIN {
        assert!(
            completeness_has(name, PortStatus::OnMain),
            "{name} must stay listed as already on main"
        );
    }
    for name in THIS_SLICE {
        assert!(
            completeness_has(name, PortStatus::ThisSlice),
            "{name} must be the closing-slice row"
        );
    }
    for name in REMAINING {
        assert!(
            completeness_has(name, PortStatus::Remaining),
            "{name} is a remaining AP_AutoTune.cpp gap"
        );
    }
    assert_eq!(on_main_items().count(), ON_MAIN.len());
    assert_eq!(this_slice_items().count(), THIS_SLICE.len());
    assert_eq!(remaining_items().count(), REMAINING.len());
    for item in AUTOTUNE_COMPLETENESS {
        let AutotunePortItem { name, status, note } = item;
        assert!(!name.is_empty(), "catalog row missing a name");
        assert!(!note.is_empty(), "{name} missing an upstream note");
        let _ = status;
    }
}

#[test]
fn remaining_does_not_repeat_hooked_surfaces() {
    for item in remaining_items() {
        assert!(
            !completeness_has(item.name, PortStatus::OnMain),
            "{} listed remaining and on main",
            item.name
        );
        assert!(
            !completeness_has(item.name, PortStatus::ThisSlice),
            "{} listed remaining and this slice",
            item.name
        );
    }
}

#[test]
fn yaw_axis_start_mask_is_already_on_main() {
    assert!(completeness_has(
        "AUTOTUNE_AXES start mask",
        PortStatus::OnMain
    ));
    let yaw = AutotuneAxes::from_bits(AUTOTUNE_AXIS_YAW);
    assert!(yaw.tune_yaw());
    assert!(!yaw.tune_roll());
    assert!(!yaw.tune_pitch());
    assert!(yaw.starts_type(AtType::Yaw));
    assert!(yaw.any_selected());
}

#[test]
fn yaw_att_limit_is_the_hardcoded_twenty_degrees() {
    close(YAW_ATT_LIMIT_DEG, 20.0);
    close(att_limit_deg(AtType::Yaw, 45.0, 30.0), 20.0);
    close(att_limit_deg(AtType::Roll, 45.0, 30.0), 45.0);
    close(att_limit_deg(AtType::Pitch, 45.0, 30.0), 30.0);

    let yaw_t1 = rate_threshold1(att_limit_deg(AtType::Yaw, 45.0, 30.0), 1.0, 75.0);
    close(yaw_t1, 0.4 * 20.0);
}

#[test]
fn imax_constrain_on_start_clamps_to_0_4_through_0_9() {
    let (lo, hi) = start_imax_band();
    close(lo, 0.4);
    close(hi, 0.9);
    close(lo, AUTOTUNE_MIN_IMAX);
    close(hi, AUTOTUNE_MAX_IMAX);
    close(start_constrain_imax(0.10), AUTOTUNE_MIN_IMAX);
    close(start_constrain_imax(1.00), AUTOTUNE_MAX_IMAX);
    close(start_constrain_imax(0.60), 0.60);
    close(start_constrain_imax(0.10), constrain_imax(0.10));

    let mut tuner = AutoTune::with_gains(AtType::Yaw, sample_gains());
    tuner.imax = 0.15;
    tuner.start();
    close(tuner.imax, AUTOTUNE_MIN_IMAX);
    assert!(tuner.running);

    tuner.imax = 1.25;
    tuner.start();
    close(tuner.imax, AUTOTUNE_MAX_IMAX);

    tuner.imax = 0.55;
    tuner.start();
    close(tuner.imax, 0.55);
}

#[test]
fn save_gains_on_stop_after_n_stable_cycles() {
    assert_eq!(DONE_COUNT_SAVE, 3);
    assert!(!should_save_after_cycles(0));
    assert!(!should_save_after_cycles(2));
    assert!(should_save_after_cycles(3));

    let mut tuner = AutoTune::with_gains(AtType::Yaw, sample_gains());
    tuner.start();
    let next = tuned(sample_gains());
    tuner.current = next;
    tuner.p_limit = 0.5;
    tuner.d_limit = 0.02;

    assert!(!tuner.record_stable_cycle());
    assert_eq!(tuner.done_count, 1);
    assert!(!tuner.record_stable_cycle());
    assert_eq!(tuner.done_count, 2);
    assert!(tuner.record_stable_cycle());
    assert_eq!(tuner.done_count, 3);
    assert!(should_save_after_cycles(tuner.done_count));
    assert_eq!(tuner.last_save, next);
    assert!(!tuner.record_stable_cycle());
    assert_eq!(tuner.done_count, 3);

    tuner.stop();
    assert!(!tuner.running);
    assert_eq!(tuner.current, next);
    assert_eq!(tuner.last_save, next);
}

#[test]
fn record_stable_cycle_needs_running_and_both_limits() {
    let mut tuner = AutoTune::with_gains(AtType::Roll, sample_gains());
    tuner.p_limit = 0.5;
    tuner.d_limit = 0.02;
    assert!(!tuner.record_stable_cycle());
    assert_eq!(tuner.done_count, 0);

    tuner.start();
    tuner.p_limit = 0.0;
    tuner.d_limit = 0.02;
    assert!(!tuner.record_stable_cycle());
    assert_eq!(tuner.done_count, 0);
}

#[test]
fn start_resets_done_count() {
    let mut tuner = AutoTune::with_gains(AtType::Pitch, sample_gains());
    tuner.start();
    tuner.p_limit = 0.4;
    tuner.d_limit = 0.01;
    assert!(!tuner.record_stable_cycle());
    assert_eq!(tuner.done_count, 1);
    tuner.start();
    assert_eq!(tuner.done_count, 0);
}
