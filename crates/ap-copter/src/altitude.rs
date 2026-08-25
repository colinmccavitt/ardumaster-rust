//! Height above ground, shared by every mode that has to know it.
//!
//! Upstream puts this on `Mode`, and four call sites reach for it: the
//! landing descent, the auto-mode landing check, RTL, and avoidance. It is
//! its own module here rather than living with the landing code because it
//! answers a question about the aircraft rather than about a manoeuvre.

/// Height above ground, upstream `Mode::get_alt_above_ground_m`.
///
/// Three sources tried in order of how much they are trusted, then an
/// assumption. The ordering is the whole content of the function, so it is
/// worth stating why each rung sits where it does.
///
/// A rangefinder measures the distance to whatever is actually below the
/// aircraft, so nothing else can beat it when it is available — it sees the
/// roof, the tree and the parked van that no database knows about.
///
/// The terrain database is next: it knows the ground's height at this
/// position, which is right about the terrain and silent about anything
/// standing on it.
///
/// The last rung is not a measurement at all. With no rangefinder and no
/// terrain data, upstream takes the altitude above home and calls it height
/// above ground — which is to say it assumes the Earth is flat and that home
/// is at ground level. It is stated as an assumption in upstream's own
/// comment rather than dressed up, and it is the right shape of fallback for
/// a landing: wrong over sloping ground, but wrong in a way that is bounded
/// by how far the aircraft has flown from home.
///
/// # The initialised check sits between the first and second rungs
///
/// Not at the top. A rangefinder reading is a distance below the aircraft and
/// means the same thing whether or not the vehicle knows where it is, so it
/// is honoured even during startup when `current_loc` holds nothing. Only the
/// two rungs that need a position are gated on having one, and an
/// uninitialised location returns zero rather than falling through to a
/// flat-earth reading of an altitude that is also meaningless.
///
/// # Parameters
///
/// Each source is `Some` when it has an answer, mirroring upstream's
/// out-parameter-and-bool pairs. `loc_alt_cm` is `current_loc.alt`, in
/// centimetres, as upstream stores it.
#[must_use]
pub fn alt_above_ground_m(
    rangefinder_m: Option<f32>,
    loc_initialised: bool,
    terrain_m: Option<f32>,
    loc_alt_cm: i32,
) -> f32 {
    if let Some(height_m) = rangefinder_m {
        return height_m;
    }

    if !loc_initialised {
        // Uninitialised during startup. Zero is not a measurement; it is the
        // least dangerous number to hand a caller that is about to decide how
        // fast to descend.
        return 0.0;
    }

    if let Some(height_m) = terrain_m {
        return height_m;
    }

    // Assume the Earth is flat.
    //
    // The multiply is done in f32 to match upstream, which stores `alt` as an
    // `int32_t` and writes `* 0.01` — a double literal, so the product is
    // computed in double and narrowed on return. Reproduced by widening
    // explicitly rather than left to whatever the expression happens to
    // promote to.
    #[allow(
        clippy::cast_lossless,
        clippy::cast_possible_truncation,
        reason = "reproduces upstream's int32 -> double -> float narrowing \
exactly; see the comment above"
    )]
    {
        (f64::from(loc_alt_cm) * 0.01) as f32
    }
}
