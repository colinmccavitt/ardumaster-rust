//! Compass primary instance selection stub, upstream `Compass::get_first_usable`.
//!
//! `_first_usable` is the first priority instance with `COMPASS_USE` set.
//! Frontend accessors (`get_field`, `healthy`, yaw) index through that instance.

/// First compass marked for use by `COMPASS_USE` / `USE2`.
///
/// Upstream `Compass::read` scans priority order and stores `_first_usable`.
/// An empty or all-false list leaves instance 0, matching a zeroed
/// `uint8_t _first_usable`.
#[must_use]
pub fn first_usable(use_for_yaw: &[bool]) -> u8 {
    for (i, &use_yaw) in use_for_yaw.iter().enumerate() {
        if use_yaw {
            return i as u8;
        }
    }
    0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_true_wins() {
        assert_eq!(first_usable(&[]), 0);
        assert_eq!(first_usable(&[true]), 0);
        assert_eq!(first_usable(&[true, true]), 0);
        assert_eq!(first_usable(&[false, true]), 1);
        assert_eq!(first_usable(&[false, false]), 0);
        assert_eq!(first_usable(&[false, true, true]), 1);
    }
}
