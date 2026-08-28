//! Behaviour of the battery current limiter.
//!
//! # Why these are not parity tests
//!
//! Every other file beside this one compares against fixtures dumped from the
//! compiled firmware. This one cannot, and the reason is worth stating rather
//! than leaving as an absence.
//!
//! `get_current_limit_max_throttle` reads the battery through
//! `AP_BattMonitor`, whose `state[]` is *private*, with friendship granted
//! only to the real backends. There is no injection point: a fixture cannot
//! hand it a current and a voltage. Worse, pack resistance is not a reading at
//! all — `AP_BattMonitor_Backend::update_resistance_estimate` derives it from
//! observed voltage and current *variation* over time, so it only exists in a
//! running simulation.
//!
//! So this function's inputs cannot be scripted the way throttle and time are.
//! Its correct verification is a SITL differential run — fly both, compare the
//! logged `_throttle_limit` — which is the `sitl-diff` class in the tracker,
//! not `unit-parity`. That is recorded on COP-004 rather than papered over
//! with a fixture that would only be testing a stubbed battery.
//!
//! What these tests do instead is pin the behaviour the port is claimed to
//! have, most importantly that the ceiling *accumulates* rather than lagging.

use ap_motors::current_limit::{BatteryState, CurrentLimit, CurrentLimitParams};
use ap_motors::throttle::HoverThrottle;

const DT: f32 = 0.0025;

fn params() -> CurrentLimitParams {
    CurrentLimitParams {
        batt_current_max: 60.0,
        batt_current_time_constant: 1.0,
        battery_min_voltage: 13.2,
    }
}

fn healthy(current: f32) -> BatteryState {
    BatteryState {
        current_amps: Some(current),
        resistance: 0.02,
        voltage: 15.4,
        // Not exercised by these current-limiting tests (COP-004); COP-006's
        // thrust_linearization tests cover this field.
        voltage_resting_estimate: 15.4,
    }
}

fn hover() -> HoverThrottle {
    HoverThrottle::new(0.4)
}

fn same(a: f32, b: f32) -> bool {
    (a.is_nan() && b.is_nan()) || a.to_bits() == b.to_bits()
}

/// Each disabling condition returns 1.0 *and* clears the stored limit.
///
/// Clearing matters: a limit left part-wound-down would be applied again the
/// moment the vehicle rearmed, throttling an aircraft for a current draw that
/// happened on a previous flight.
#[test]
fn every_disabling_condition_resets_the_limit() {
    let cases: [(&str, bool, CurrentLimitParams, BatteryState); 4] = [
        (
            "limiting off",
            true,
            CurrentLimitParams {
                batt_current_max: 0.0,
                ..params()
            },
            healthy(50.0),
        ),
        ("disarmed", false, params(), healthy(50.0)),
        (
            "no current telemetry",
            true,
            params(),
            BatteryState {
                current_amps: None,
                ..healthy(50.0)
            },
        ),
        (
            "unknown resistance",
            true,
            params(),
            BatteryState {
                resistance: 0.0,
                ..healthy(50.0)
            },
        ),
    ];

    for (name, armed, p, batt) in cases {
        let mut limit = CurrentLimit::new();

        // Wind the limit down first, so a failure to reset is visible.
        for _ in 0..4000 {
            limit.update(true, DT, &params(), &healthy(200.0), &hover());
        }
        assert!(
            limit.throttle_limit() < 0.5,
            "{name}: setup did not wind down"
        );

        let out = limit.update(armed, DT, &p, &batt, &hover());
        assert!(same(out, 1.0), "{name}: returned {out}, expected 1.0");
        assert!(
            same(limit.throttle_limit(), 1.0),
            "{name}: stored limit left at {}",
            limit.throttle_limit()
        );
    }
}

/// The ceiling accumulates; it does not settle at `1 - ratio`.
///
/// This is the whole point of the module docs. Written as a first-order lag —
/// which is what the expression looks like — the limit would converge on
/// `1 - ratio` and stay there. It does not: below the permissible current it
/// climbs to the ceiling, above it, it falls to the floor. A port that got
/// this wrong would agree only where the two curves happen to cross.
#[test]
fn the_ceiling_integrates_rather_than_settling() {
    // Draw well under the limit: the ceiling should climb to 1.0, not settle
    // at 1 - (10/60) = 0.833.
    let mut limit = CurrentLimit::new();
    limit.update(true, DT, &params(), &healthy(200.0), &hover()); // knock it down
    for _ in 0..20_000 {
        limit.update(true, DT, &params(), &healthy(10.0), &hover());
    }
    assert!(
        same(limit.throttle_limit(), 1.0),
        "under-draw should reach the ceiling, got {}",
        limit.throttle_limit()
    );

    // Draw well over: it should fall all the way to the floor, not settle at
    // a negative-ish steady state.
    let mut limit = CurrentLimit::new();
    for _ in 0..20_000 {
        limit.update(true, DT, &params(), &healthy(400.0), &hover());
    }
    assert!(
        same(limit.throttle_limit(), 0.2),
        "over-draw should reach the floor, got {}",
        limit.throttle_limit()
    );
}

/// The limiter never takes away the throttle needed to hover.
#[test]
fn the_ceiling_never_falls_below_hover() {
    let mut limit = CurrentLimit::new();
    let h = hover();

    let mut lowest = f32::INFINITY;
    for _ in 0..20_000 {
        let out = limit.update(true, DT, &params(), &healthy(400.0), &h);
        lowest = lowest.min(out);
    }

    assert!(
        lowest > h.get(),
        "ceiling {lowest} fell to or below hover {}",
        h.get()
    );
    // At the floor the available range above hover is a fifth of what it was.
    assert!(
        same(lowest, h.get() + (1.0 - h.get()) * 0.2),
        "got {lowest}"
    );
}

/// Above its minimum voltage, the ohmic term can never bind.
///
/// The permissible current is `min(param, draw + (V - Vmin)/R)`. While
/// `V >= Vmin` that second term is `draw` plus something non-negative, so the
/// ratio `draw / permissible` cannot exceed 1 through the ohmic path alone --
/// only the parameter limit can bind. Which means a pack that is merely
/// *close* to its minimum does not limit at all; it has to actually sag past
/// it. Worth pinning, because the opposite is the intuitive reading.
#[test]
fn a_pack_above_its_minimum_is_never_limited_by_sag() {
    let draw = 40.0; // comfortably under the 60 A parameter limit

    for volts in [13.2_f32, 13.3, 14.0, 15.4, 16.8] {
        let mut limit = CurrentLimit::new();
        for _ in 0..8000 {
            limit.update(
                true,
                DT,
                &params(),
                &BatteryState {
                    voltage: volts,
                    ..healthy(draw)
                },
                &hover(),
            );
        }
        assert!(
            same(limit.throttle_limit(), 1.0),
            "{volts} V is at or above the 13.2 V minimum, so nothing should limit: got {}",
            limit.throttle_limit()
        );
    }
}

/// Sagged below its minimum, the pack pulls the ceiling down.
///
/// Now `(V - Vmin)` is negative, the permissible current drops below the
/// measured draw, and the ratio exceeds 1 -- so the integrator winds down even
/// though the draw is well under `MOT_BAT_CURR_MAX`.
#[test]
fn a_pack_sagged_below_its_minimum_limits_the_ceiling() {
    let draw = 40.0;

    let mut limit = CurrentLimit::new();
    for _ in 0..20_000 {
        limit.update(
            true,
            DT,
            &params(),
            &BatteryState {
                voltage: 13.0,
                ..healthy(draw)
            },
            &hover(),
        );
    }

    assert!(
        same(limit.throttle_limit(), 0.2),
        "a pack 0.2 V below its minimum should wind the ceiling to the floor, got {}",
        limit.throttle_limit()
    );
}

/// At the ceiling the limiter is transparent.
#[test]
fn an_unlimited_ceiling_returns_full_throttle() {
    let mut limit = CurrentLimit::new();
    let out = limit.update(true, DT, &params(), &healthy(1.0), &hover());
    assert!(same(limit.throttle_limit(), 1.0));
    assert!(same(out, 1.0), "got {out}");
}
