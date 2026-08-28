//! COMPASS_LEARN mode enum stub, upstream `Compass::LearnType`. FW-014.
//!
//! The parameter is stored as `u8`; this enum is the typed `LearnType`
//! (`NONE=0`, reserved `INTERNAL=1`, `COPY_FROM_EKF=2`, `INFLIGHT=3`).

use crate::offset::{
    learn_offsets_enabled, COMPASS_LEARN_EKF, COMPASS_LEARN_INFLIGHT, COMPASS_LEARN_NONE,
};

/// Upstream `Compass::LearnType`.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LearnType {
    /// `LearnType::NONE` — offset learning disabled.
    None = 0,
    /// `LearnType::COPY_FROM_EKF` — copy EKF-learned offsets.
    CopyFromEkf = 2,
    /// `LearnType::INFLIGHT` — inflight offset learning.
    Inflight = 3,
}

impl LearnType {
    /// Decode `COMPASS_LEARN`. Reserved `INTERNAL = 1` is not a variant.
    #[must_use]
    pub const fn from_u8(raw: u8) -> Option<Self> {
        match raw {
            COMPASS_LEARN_NONE => Some(Self::None),
            COMPASS_LEARN_EKF => Some(Self::CopyFromEkf),
            COMPASS_LEARN_INFLIGHT => Some(Self::Inflight),
            _ => None,
        }
    }

    /// Encode as the `COMPASS_LEARN` parameter value.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// True when `COMPASS_LEARN` is INFLIGHT, upstream `Compass::learn_offsets_enabled`.
    #[must_use]
    pub const fn inflight_offsets_enabled(self) -> bool {
        matches!(self, Self::Inflight)
    }

    /// True when offset learning is enabled (EKF or inflight).
    #[must_use]
    pub fn offsets_learn_enabled(self) -> bool {
        learn_offsets_enabled(self.as_u8())
    }
}

impl Default for LearnType {
    fn default() -> Self {
        Self::None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::offset::COMPASS_LEARN_DEFAULT;

    #[test]
    fn default_is_none() {
        assert_eq!(LearnType::default(), LearnType::None);
        assert_eq!(LearnType::default().as_u8(), COMPASS_LEARN_DEFAULT);
        assert_eq!(
            LearnType::from_u8(COMPASS_LEARN_DEFAULT),
            Some(LearnType::None)
        );
    }

    #[test]
    fn maps_upstream_values() {
        assert_eq!(LearnType::from_u8(0), Some(LearnType::None));
        assert_eq!(LearnType::from_u8(2), Some(LearnType::CopyFromEkf));
        assert_eq!(LearnType::from_u8(3), Some(LearnType::Inflight));
        assert_eq!(LearnType::None.as_u8(), COMPASS_LEARN_NONE);
        assert_eq!(LearnType::CopyFromEkf.as_u8(), COMPASS_LEARN_EKF);
        assert_eq!(LearnType::Inflight.as_u8(), COMPASS_LEARN_INFLIGHT);
    }

    #[test]
    fn reserved_internal_is_not_a_variant() {
        assert_eq!(LearnType::from_u8(1), None);
        assert_eq!(LearnType::from_u8(4), None);
    }

    #[test]
    fn inflight_enables_offset_learn() {
        assert!(!LearnType::None.inflight_offsets_enabled());
        assert!(!LearnType::CopyFromEkf.inflight_offsets_enabled());
        assert!(LearnType::Inflight.inflight_offsets_enabled());
        assert!(!LearnType::None.offsets_learn_enabled());
        assert!(LearnType::CopyFromEkf.offsets_learn_enabled());
        assert!(LearnType::Inflight.offsets_learn_enabled());
    }
}
