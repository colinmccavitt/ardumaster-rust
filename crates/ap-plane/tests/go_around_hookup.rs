//! Vehicle loop go-around latch hookup.

use ap_landing::go_around::LandingFlags;
use ap_plane::go_around_hookup::apply_landing_go_around_latch;

#[test]
fn latch_sets_commanded_go_around_when_requested() {
    let mut flags = LandingFlags {
        in_progress: true,
        commanded_go_around: false,
    };
    assert!(apply_landing_go_around_latch(&mut flags, true));
    assert!(flags.commanded_go_around);
}

#[test]
fn latch_noop_when_not_requested() {
    let mut flags = LandingFlags::default();
    assert!(!apply_landing_go_around_latch(&mut flags, false));
    assert!(!flags.commanded_go_around);
}
