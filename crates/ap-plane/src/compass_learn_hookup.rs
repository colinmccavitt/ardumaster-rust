//! COMPASS_LEARN mode enum stub, upstream `Compass::LearnType`.
//!
//! Reports the typed learn mode from `COMPASS_LEARN` on the SITL hookup.

use ap_compass::learn::LearnType;
use ap_compass::offset::learn_offsets_enabled;

use crate::sitl_compass_hookup::SitlCompassHookup;

/// Snapshot of the `COMPASS_LEARN` mode applied to the frontend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompassLearnOutput {
    /// Raw `COMPASS_LEARN` parameter.
    pub learn: u8,
    /// Typed `LearnType` when the value is a known variant.
    pub mode: Option<LearnType>,
    /// True when inflight offset learning is enabled.
    pub inflight_learn: bool,
    /// True when EKF or inflight offset learning is enabled.
    pub offsets_learn: bool,
}

/// Report the learn mode that offset-learn / persist will honor.
#[must_use]
pub fn compass_learn_tick(hookup: &SitlCompassHookup) -> CompassLearnOutput {
    let learn = hookup.compass_params().learn;
    let mode = LearnType::from_u8(learn);
    CompassLearnOutput {
        learn,
        mode,
        inflight_learn: mode.is_some_and(LearnType::inflight_offsets_enabled),
        offsets_learn: learn_offsets_enabled(learn),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sitl_compass_hookup::SitlCompassHookup;
    use ap_compass::offset::{COMPASS_LEARN_DEFAULT, COMPASS_LEARN_INFLIGHT};
    use ap_compass::params::CompassParams;

    #[test]
    fn default_is_none() {
        let hookup = SitlCompassHookup::default();
        let out = compass_learn_tick(&hookup);
        assert_eq!(out.learn, COMPASS_LEARN_DEFAULT);
        assert_eq!(out.mode, Some(LearnType::None));
        assert!(!out.inflight_learn);
        assert!(!out.offsets_learn);
    }

    #[test]
    fn reports_configured_inflight() {
        let mut hookup = SitlCompassHookup::default();
        let mut params = CompassParams::default();
        params.learn = COMPASS_LEARN_INFLIGHT;
        hookup.apply_compass_params(params);
        let out = compass_learn_tick(&hookup);
        assert_eq!(out.mode, Some(LearnType::Inflight));
        assert!(out.inflight_learn);
        assert!(out.offsets_learn);
    }
}
