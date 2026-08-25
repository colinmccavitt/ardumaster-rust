//! Does the coning correction actually recover what the naive integral loses?
//!
//! Rotations do not commute, so summing rate-times-time is not the rotation
//! that occurred. When the rate vector sweeps, the error accumulates in a
//! consistent direction instead of averaging out. The coning correction exists
//! to recover it, and this measures whether it does.
//!
//! A flight log cannot answer this. It records what upstream's accumulator
//! produced, so replaying it can only show the port loses the same rotation
//! ArduPilot loses. The truth here comes from `ap_sim`, which composes exact
//! Rodrigues rotations in `f64` -- so the true rotation over a window is the
//! logarithm of the composed matrix, computed without ever summing a rate.

#![allow(
    clippy::cast_possible_truncation,
    reason = "the simulator works in f64 and the port in f32; narrowing at the \
boundary is the interface being exercised"
)]

use ap_ins::{ImuInstance, LoopTiming};
use ap_math::vector3::Vector3f;
use ap_sim::{coning, level, AttitudeSim, RateProfile, V3};

const RATE_HZ: u16 = 8000;

fn to_port(v: V3) -> Vector3f {
    Vector3f::new(v.x as f32, v.y as f32, v.z as f32)
}

fn from_port(v: Vector3f) -> V3 {
    V3::new(f64::from(v.x), f64::from(v.y), f64::from(v.z))
}

/// How exactly truth is propagated within one sensor interval. Well past the
/// point where it stops mattering -- the residual is second order in the
/// sub-interval, so this is orders below what an f32 integrator can resolve.
const SUBSTEPS: usize = 64;

/// What one flight-loop window produced, three ways.
struct Window {
    /// The rotation that actually occurred, as a rotation vector.
    truth: V3,
    /// Trapezoidal integration with NO coning term -- exactly what the port
    /// computes minus the one thing under test. Comparing against this
    /// isolates the coning correction from the integration scheme.
    trapezoid: V3,
    /// What the port published: the same trapezoid plus coning.
    ported: V3,
}

fn run(profile: RateProfile, dt: f64, per_loop: usize, loops: usize) -> Vec<Window> {
    let mut sim = AttitudeSim::new();
    let mut imu = ImuInstance::new();
    let timing = LoopTiming::new((dt * per_loop as f64) as f32);
    let step_us = (dt * 1.0e6).round() as u64;
    let mut t_us = 1_000_000_u64;

    // Prime: the first sample is always discarded, by design.
    let first = sim.step_continuous(profile, dt, SUBSTEPS);
    t_us += step_us;
    imu.notify_gyro_raw_sample(to_port(first.gyro), t_us, RATE_HZ, t_us);
    let mut last_gyro = first.gyro;

    let mut out = Vec::with_capacity(loops);
    for _ in 0..loops {
        let start = sim.truth;
        let mut trapezoid = V3::zero();

        for _ in 0..per_loop {
            let s = sim.step_continuous(profile, dt, SUBSTEPS);
            t_us += step_us;
            imu.notify_gyro_raw_sample(to_port(s.gyro), t_us, RATE_HZ, t_us);

            // The port's integration scheme, without its coning term. Kept in
            // f64 so the comparison is not measuring f32 rounding.
            trapezoid = trapezoid.plus(s.gyro.plus(last_gyro).scaled(0.5 * dt));
            last_gyro = s.gyro;
        }

        imu.update_gyro();
        let (delta_angle, _) = imu
            .get_delta_angle(&timing)
            .expect("a sample was published this window");

        out.push(Window {
            truth: start.transposed().times(sim.truth).to_rotation_vector(),
            trapezoid,
            ported: from_port(delta_angle),
        });
    }
    out
}

/// Total error against truth over every window, as a rotation magnitude.
fn total_error(windows: &[Window], pick: fn(&Window) -> V3) -> f64 {
    windows
        .iter()
        .map(|w| {
            let e = pick(w);
            V3::new(e.x - w.truth.x, e.y - w.truth.y, e.z - w.truth.z).length()
        })
        .sum()
}

/// The headline. On a sweeping rate vector the correction should recover most
/// of what plain trapezoidal integration loses.
#[test]
fn coning_correction_recovers_what_the_naive_sum_loses() {
    // 8 kHz sensor, 400 Hz loop, one second of flight.
    let w = run(coning, 1.0 / 8000.0, 20, 400);

    let naive = total_error(&w, |w| w.trapezoid);
    let corrected = total_error(&w, |w| w.ported);

    println!(
        "over 400 windows: trapezoid alone off by {naive:.6e} rad total, corrected {corrected:.6e} rad, \
         {:.1}x better",
        naive / corrected
    );
    // Measured 8.5x at the time of writing. Losing the correction entirely
    // would miss this by an order of magnitude.
    assert!(
        corrected < naive * 0.25,
        "the correction should cut the error at least fourfold: trapezoid {naive:e}, corrected {corrected:e}"
    );
}

/// The error the correction targets is a *drift*, not noise: it accumulates in
/// one direction. That is what makes it worth correcting, and it is why a test
/// that only looked at per-window magnitudes would understate the problem.
#[test]
fn the_naive_error_accumulates_in_one_direction() {
    let w = run(coning, 1.0 / 8000.0, 20, 400);

    let sum = |pick: fn(&Window) -> V3| {
        w.iter().fold(V3::zero(), |a, win| {
            let e = pick(win);
            a.plus(V3::new(
                e.x - win.truth.x,
                e.y - win.truth.y,
                e.z - win.truth.z,
            ))
        })
    };

    let naive_drift = sum(|w| w.trapezoid).length();
    let corrected_drift = sum(|w| w.ported).length();
    println!(
        "accumulated error: trapezoid {naive_drift:.6e} rad, corrected {corrected_drift:.6e} rad, \
         {:.0}x better",
        naive_drift / corrected_drift
    );

    // If it were noise the vector sum would be far smaller than the sum of
    // magnitudes. It is not: almost all of it survives the sum.
    let naive_magnitudes = total_error(&w, |w| w.trapezoid);
    assert!(
        naive_drift > 0.5 * naive_magnitudes,
        "the naive error should be a consistent drift, not noise: vector sum \
         {naive_drift:e} against magnitude sum {naive_magnitudes:e}"
    );
    // Measured 10,500x. What survives the correction is no longer a drift at
    // all: its vector sum is three orders below its magnitude sum, which is
    // what rounding noise looks like and what a bias does not.
    assert!(
        corrected_drift < naive_drift * 0.01,
        "the correction should remove essentially all of the drift: trapezoid \
         {naive_drift:e}, corrected {corrected_drift:e}"
    );
}

/// With the rate vector held in a fixed direction there is no coning to
/// correct, and the correction must not invent any. This is the control: a
/// buggy correction that simply added something proportional to the rotation
/// would pass the headline test and fail here.
#[test]
fn a_fixed_axis_rotation_needs_no_correction() {
    fn fixed(_t: f64) -> V3 {
        V3::new(0.0, 0.0, 3.0)
    }

    let w = run(fixed as RateProfile, 1.0 / 8000.0, 20, 400);
    let err = total_error(&w, |w| w.ported);
    let naive = total_error(&w, |w| w.trapezoid);
    println!("fixed axis: naive {naive:.6e} rad, ported {err:.6e} rad over 400 windows");

    // Single-axis rotations compose exactly, so the naive sum is already
    // right and there is nothing to recover. Both should be at rounding
    // level, and the correction must not have made things worse.
    assert!(
        err < 2.0e-4,
        "a fixed axis should need no correction, got {err:e} rad"
    );
}

/// A stationary vehicle produces no rotation and no correction.
#[test]
fn a_stationary_vehicle_accumulates_nothing() {
    let w = run(level, 1.0 / 8000.0, 20, 100);
    for win in &w {
        assert_eq!(win.ported, V3::zero());
        assert_eq!(win.truth, V3::zero());
    }
}
