//! Vehicle loop go-around latch hookup, upstream the call to
//! `landing.request_go_around()` after servo override checks in `servos.cpp`.

use ap_landing::go_around::{request_go_around, LandingFlags};

/// Latch a go-around request into landing flags when the servo hookup asked for one.
///
/// Upstream `Plane::set_servos` calls `landing.request_go_around()` when deepstall
/// override fails because the elevator channel is missing.
#[must_use]
pub fn apply_landing_go_around_latch(flags: &mut LandingFlags, request: bool) -> bool {
    if request {
        request_go_around(flags)
    } else {
        false
    }
}
