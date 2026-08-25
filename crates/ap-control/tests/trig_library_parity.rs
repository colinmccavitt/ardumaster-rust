//! D-017: how far apart `libm` and glibc are, measured on a real flight.
//!
//! The port calls transcendentals through `libm`, because a `no_std` binary has
//! no C library to call. The reference build calls glibc. They are not
//! bit-identical, and this is the only reason the pitch replay is not exact:
//! every quantity in it that reaches no transcendental, the measured rate
//! included, agrees to the bit.
//!
//! Rather than assert that in prose, this measures it over the same flight the
//! replay uses, on the one expression in the fixed-wing controllers that calls
//! a transcendental — the pitch turn-coordination offset. If a toolchain change
//! ever widens the gap, this fails instead of quietly enlarging the residual
//! that the replay tolerances absorb.
//!
//! # Why the calls are written `Real::tan(x)` and not `x.tan()`
//!
//! Under `cfg(test)` the harness links `std`, whose inherent `f32` math methods
//! shadow the `Real` trait — inherent methods win name resolution. Written as
//! methods, the same source compiles to glibc in unit tests and to `libm` in
//! the firmware. Both forms appear below deliberately, because measuring the
//! difference is the point; production code uses the explicit trait form only.

#![allow(
    clippy::float_cmp,
    reason = "comparing float representations exactly is the measurement"
)]

use ap_math::scalar::{degrees, Real, GRAVITY_MSS};
use ap_replay::Fixture;

fn fixtures_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures"))
        .expect("workspace root")
}

/// Upstream's pitch coordination offset, with the clamps that do not affect
/// which library is called left out.
fn offset_via_libm(pitch: f32, bank: f32, speed: f32) -> f32 {
    Real::cos(pitch) * degrees(GRAVITY_MSS / speed * Real::tan(bank) * Real::sin(bank)).abs()
}

/// The same expression written as method calls, which under `cfg(test)`
/// resolve to `std`'s inherent methods and so reach glibc.
fn offset_via_glibc(pitch: f32, bank: f32, speed: f32) -> f32 {
    pitch.cos() * degrees(GRAVITY_MSS / speed * bank.tan() * bank.sin()).abs()
}

#[test]
fn libm_and_glibc_agree_to_within_four_ulp() {
    let path = fixtures_dir().join("pitch_replay.csv");
    if !path.exists() {
        eprintln!("skipping: pitch fixture not present");
        return;
    }
    let fx = Fixture::load(&path).expect("fixture should load");

    let mut evaluated = 0usize;
    let mut differing = 0usize;
    let mut worst_ulp = 0i32;
    let mut worst_abs = 0.0_f64;

    for row in &fx.rows {
        // upstream skips the offset entirely beyond 70 degrees of pitch
        if row.input("ps").abs() > 7000.0 {
            continue;
        }
        let bank = row.input("rr") as f32;
        let pitch = row.input("pr") as f32;
        let speed = ((row.input("as") as f32) * (row.input("e2t") as f32)).max(10.0);

        let a = offset_via_libm(pitch, bank, speed);
        let b = offset_via_glibc(pitch, bank, speed);
        evaluated += 1;

        if a.to_bits() != b.to_bits() {
            differing += 1;
            let ulp = (a.to_bits() as i64 - b.to_bits() as i64).unsigned_abs() as i32;
            worst_ulp = worst_ulp.max(ulp);
            worst_abs = worst_abs.max((f64::from(a) - f64::from(b)).abs());
        }
    }

    println!("offsets evaluated:     {evaluated}");
    println!("libm and glibc differ: {differing}");
    println!("worst {worst_ulp} ulp, {worst_abs:.3e} absolute");

    assert!(evaluated > 8000, "too few samples: {evaluated}");
    assert!(
        worst_ulp <= 4,
        "libm and glibc now differ by {worst_ulp} ulp, was 4 when D-017 was \
         written; the pitch replay's residual is bounded by this"
    );
    // Not an equality assertion: that they differ at all is the point of the
    // entry, and a toolchain where they agree everywhere would be fine.
    assert!(
        differing < evaluated / 4,
        "the two libraries now disagree on {differing} of {evaluated} samples, \
         far more than the 616 measured for D-017"
    );
}
