//! Compass filter-range stub, upstream `COMPASS_FLTR_RNG`.
//!
//! `_filter_range` is a percent around the running mean field length.
//! Zero disables the filter so every finite sample is published.

use ap_compass::filter_range::filter_enabled;

use crate::sitl_compass_hookup::SitlCompassHookup;

/// Snapshot of `COMPASS_FLTR_RNG` on the frontend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompassFilterRangeOutput {
    /// Raw `COMPASS_FLTR_RNG` / `_filter_range`.
    pub filter_range: u8,
    /// True when the mean-length filter is active.
    pub filter_enabled: bool,
    /// Samples rejected by the range check on the primary backend.
    pub error_count: u32,
}

/// Report whether `COMPASS_FLTR_RNG` is gating SITL samples.
#[must_use]
pub fn compass_filter_range_tick(hookup: &SitlCompassHookup) -> CompassFilterRangeOutput {
    let filter_range = hookup.compass_params().filter_range;
    CompassFilterRangeOutput {
        filter_range,
        filter_enabled: filter_enabled(filter_range),
        error_count: hookup
            .backend()
            .map(|backend| backend.filter_state().error_count)
            .unwrap_or(0),
    }
}

/// Apply `COMPASS_FLTR_RNG` and push it onto the SITL cluster.
pub fn apply_filter_range(hookup: &mut SitlCompassHookup, filter_range: u8) {
    let mut params = *hookup.compass_params();
    params.filter_range = filter_range;
    hookup.apply_compass_params(params);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sitl_compass_hookup::SitlCompassHookup;
    use ap_compass::filter_range::COMPASS_FLTR_RNG_DEFAULT;
    use ap_compass::filter_range::COMPASS_FLTR_RNG_DEFAULT;

    #[test]
    fn default_range_keeps_filter_off() {
        let hookup = SitlCompassHookup::default();
        let out = compass_filter_range_tick(&hookup);
        assert_eq!(out.filter_range, COMPASS_FLTR_RNG_DEFAULT);
        assert!(!out.filter_enabled);
        assert_eq!(out.error_count, 0);
    }

    #[test]
    fn apply_range_enables_filter() {
        let mut hookup = SitlCompassHookup::default();
        apply_filter_range(&mut hookup, 10);
        let out = compass_filter_range_tick(&hookup);
        assert_eq!(out.filter_range, 10);
        assert!(out.filter_enabled);
        assert_eq!(hookup.backend().expect("backend").config().filter_range, 10);
    }
}
