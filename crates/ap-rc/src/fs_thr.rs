//! RC failsafe PWM threshold, upstream `Plane::rc_throttle_value_ok`.
//!
//! Plane names the parameter `THR_FS_VALUE`; Copter names the same PWM
//! floor `FS_THR_VALUE`. Either way a receiver that reports throttle
//! below the threshold is in failsafe. `THR_FAILSAFE` / `FS_THR_ENABLE`
//! gates the check: disabled means the PWM floor is ignored.
//!
//! The comparison is exclusive at the threshold, matching
//! `radio_in > throttle_fs_value`. A reversed throttle channel flips
//! the sense (`radio_in < throttle_fs_value` is then the healthy side).
//! Scheduler glue that already reads pulses lives in ap-plane; this
//! module is the RC_Channel-side floor.

/// Upstream Plane `THR_FS_VALUE` default.
pub const THR_FS_VALUE_DEFAULT: u16 = 950;
/// Upstream Copter `FS_THR_VALUE` default.
pub const FS_THR_VALUE_DEFAULT: u16 = 975;
/// Upstream Plane `@Range` lower bound for `THR_FS_VALUE`.
pub const THR_FS_VALUE_MIN: u16 = 925;
/// Upstream Plane `@Range` upper bound for `THR_FS_VALUE`.
pub const THR_FS_VALUE_MAX: u16 = 2200;
/// Upstream Copter `@Range` lower bound for `FS_THR_VALUE`.
pub const FS_THR_VALUE_MIN: u16 = 910;
/// Upstream Copter `@Range` upper bound for `FS_THR_VALUE`.
pub const FS_THR_VALUE_MAX: u16 = 1100;

/// Upstream `Parameters::ThrFailsafe` / Plane `THR_FAILSAFE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ThrFailsafe {
    /// `THR_FAILSAFE = 0` — do not test the PWM floor.
    Disabled = 0,
    /// `THR_FAILSAFE = 1` — test the PWM floor and take the failsafe action.
    Enabled = 1,
    /// `THR_FAILSAFE = 2` (`EnabledNoFS`) — still test the PWM floor.
    EnabledNoFs = 2,
}

impl ThrFailsafe {
    /// True when the PWM floor is consulted, upstream `!= Disabled`.
    #[must_use]
    pub const fn checks_pwm(self) -> bool {
        !matches!(self, Self::Disabled)
    }
}

/// True when throttle PWM is in the failsafe band.
///
/// Upstream `!Plane::rc_throttle_value_ok`. Disabled always returns
/// false. Otherwise a normal channel fails at or below `fs_thr_value`,
/// and a reversed channel fails at or above it.
#[must_use]
pub fn throttle_pwm_in_failsafe(
    radio_in: u16,
    fs_thr_value: u16,
    enabled: ThrFailsafe,
    reversed: bool,
) -> bool {
    if !enabled.checks_pwm() {
        return false;
    }
    if reversed {
        radio_in >= fs_thr_value
    } else {
        radio_in <= fs_thr_value
    }
}

/// Convenience: Plane defaults (`THR_FS_VALUE` = 950, enabled, not reversed).
#[must_use]
pub fn throttle_below_fs_thr_value(radio_in: u16) -> bool {
    throttle_pwm_in_failsafe(radio_in, THR_FS_VALUE_DEFAULT, ThrFailsafe::Enabled, false)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_upstream_plane_and_copter() {
        assert_eq!(THR_FS_VALUE_DEFAULT, 950);
        assert_eq!(FS_THR_VALUE_DEFAULT, 975);
        assert_eq!(THR_FS_VALUE_MIN, 925);
        assert_eq!(THR_FS_VALUE_MAX, 2200);
        assert_eq!(FS_THR_VALUE_MIN, 910);
        assert_eq!(FS_THR_VALUE_MAX, 1100);
        assert!(ThrFailsafe::Enabled.checks_pwm());
        assert!(ThrFailsafe::EnabledNoFs.checks_pwm());
        assert!(!ThrFailsafe::Disabled.checks_pwm());
    }

    #[test]
    fn throttle_below_threshold_is_failsafe() {
        assert!(throttle_below_fs_thr_value(900));
        assert!(throttle_pwm_in_failsafe(
            900,
            THR_FS_VALUE_DEFAULT,
            ThrFailsafe::Enabled,
            false
        ));
    }

    #[test]
    fn at_threshold_is_failsafe_matching_upstream_gt() {
        // `radio_in > throttle_fs_value` is the healthy test.
        assert!(throttle_below_fs_thr_value(THR_FS_VALUE_DEFAULT));
        assert!(!throttle_below_fs_thr_value(THR_FS_VALUE_DEFAULT + 1));
    }

    #[test]
    fn healthy_throttle_is_not_failsafe() {
        assert!(!throttle_below_fs_thr_value(1100));
        assert!(!throttle_pwm_in_failsafe(
            1500,
            FS_THR_VALUE_DEFAULT,
            ThrFailsafe::Enabled,
            false
        ));
    }

    #[test]
    fn disabled_ignores_the_pwm_floor() {
        assert!(!throttle_pwm_in_failsafe(
            0,
            THR_FS_VALUE_DEFAULT,
            ThrFailsafe::Disabled,
            false
        ));
    }

    #[test]
    fn reversed_flips_the_healthy_side() {
        assert!(!throttle_pwm_in_failsafe(
            900,
            THR_FS_VALUE_DEFAULT,
            ThrFailsafe::Enabled,
            true
        ));
        assert!(throttle_pwm_in_failsafe(
            1100,
            THR_FS_VALUE_DEFAULT,
            ThrFailsafe::Enabled,
            true
        ));
    }
}
