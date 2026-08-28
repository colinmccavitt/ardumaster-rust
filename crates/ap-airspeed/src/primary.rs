//! ARSPD_PRIMARY instance select, upstream `AP_Airspeed::_primary`.
//!
//! Vehicle-level preferred instance. Default 0. When the configured instance
//! is registered and healthy it is used; otherwise the first healthy instance
//! is selected (existing dual-sensor failover).

/// Upstream `ARSPD_PRIMARY` default: first airspeed instance.
pub const ARSPD_PRIMARY_DEFAULT: u8 = 0;

/// Clamp `ARSPD_PRIMARY` to a registered instance, falling back to 0.
#[must_use]
pub const fn clamp_primary(configured: u8, instance_count: u8) -> u8 {
    if instance_count > 0 && configured < instance_count {
        configured
    } else {
        0
    }
}

/// Select the live primary from `ARSPD_PRIMARY` plus per-instance health.
///
/// Prefers the clamped configured instance when that slot is healthy; otherwise
/// the first healthy instance; otherwise the clamped configured index.
#[must_use]
pub fn select_primary(configured: u8, healthy: &[bool], instance_count: u8) -> u8 {
    let preferred = clamp_primary(configured, instance_count);
    let count = core::cmp::min(instance_count as usize, healthy.len());
    if preferred as usize >= count {
        return preferred;
    }
    if healthy[preferred as usize] {
        return preferred;
    }
    for (i, &is_healthy) in healthy.iter().enumerate().take(count) {
        if is_healthy {
            return i as u8;
        }
    }
    preferred
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_first_instance() {
        assert_eq!(ARSPD_PRIMARY_DEFAULT, 0);
        assert_eq!(clamp_primary(ARSPD_PRIMARY_DEFAULT, 2), 0);
        assert_eq!(select_primary(ARSPD_PRIMARY_DEFAULT, &[true, true], 2), 0);
    }

    #[test]
    fn configured_secondary_is_used_when_healthy() {
        assert_eq!(clamp_primary(1, 2), 1);
        assert_eq!(select_primary(1, &[true, true], 2), 1);
    }

    #[test]
    fn out_of_range_clamps_to_zero() {
        assert_eq!(clamp_primary(5, 2), 0);
        assert_eq!(select_primary(5, &[true, true], 2), 0);
        assert_eq!(clamp_primary(0, 0), 0);
        assert_eq!(select_primary(1, &[], 0), 0);
    }

    #[test]
    fn unhealthy_configured_falls_back_to_first_healthy() {
        assert_eq!(select_primary(0, &[false, true], 2), 1);
        assert_eq!(select_primary(1, &[true, false], 2), 0);
        assert_eq!(select_primary(1, &[false, false], 2), 1);
    }
}
