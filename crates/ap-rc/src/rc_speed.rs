//! RC output PWM update rate, upstream `RC_SPEED` / `QuadPlane::rc_speed`.
//!
//! Plane stores this as `Q_RC_SPEED`. It is the PWM refresh rate in Hz
//! for the fast RC outputs — QuadPlane motors via `AP_Motors::set_update_rate`,
//! and the `RC_Channel` UART example via `hal.rcout->set_freq`. A stored
//! Hertz value is clamped to the documented `@Range` and converted to the
//! frame period that `set_freq` consumes.
//!
//! Analog control surfaces stay on `SERVO_RATE` (50 Hz) in SRV_Channel;
//! this module is the fast-output Hertz only.

/// Upstream `QuadPlane::rc_speed` default (also `RC_Channel` example `RC_SPEED`).
pub const RC_SPEED_DEFAULT: u16 = 490;
/// Upstream QuadPlane `@Range` lower bound.
pub const RC_SPEED_MIN: u16 = 50;
/// Upstream QuadPlane `@Range` upper bound.
pub const RC_SPEED_MAX: u16 = 500;
/// Copter `@Range` upper bound / non-heli `RC_FAST_SPEED`.
pub const RC_FAST_SPEED: u16 = 490;

/// Clamp a stored `RC_SPEED` (`AP_Int16`) to the documented Hertz range.
///
/// Zero and negative values become [`RC_SPEED_MIN`] so a later period
/// conversion cannot divide by zero. Values above [`RC_SPEED_MAX`] saturate.
#[must_use]
pub fn clamp_rc_speed(hz: i16) -> u16 {
    if hz < RC_SPEED_MIN as i16 {
        RC_SPEED_MIN
    } else if hz > RC_SPEED_MAX as i16 {
        RC_SPEED_MAX
    } else {
        hz as u16
    }
}

/// PWM frame period in microseconds for a Hertz rate.
///
/// Integer division matches the HAL tick (`1_000_000 / freq_hz`).
/// `hz == 0` is treated as [`RC_SPEED_MIN`] so the period is defined.
#[must_use]
pub fn pwm_period_us(hz: u16) -> u32 {
    let hz = if hz == 0 { RC_SPEED_MIN } else { hz };
    1_000_000 / u32::from(hz)
}

/// Decoded `RC_SPEED`, upstream `QuadPlane::rc_speed` after sanity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcSpeed {
    /// Hertz passed to `set_update_rate` / `hal.rcout->set_freq`.
    pub hz: u16,
}

impl Default for RcSpeed {
    fn default() -> Self {
        Self {
            hz: RC_SPEED_DEFAULT,
        }
    }
}

impl RcSpeed {
    /// Wrap and clamp a stored `RC_SPEED` parameter.
    #[must_use]
    pub fn from_param(hz: i16) -> Self {
        Self {
            hz: clamp_rc_speed(hz),
        }
    }

    /// Frame period in microseconds.
    #[must_use]
    pub fn period_us(self) -> u32 {
        pwm_period_us(self.hz)
    }
}

/// Apply `RC_SPEED` as the fast-output Hertz, upstream `set_update_rate`.
///
/// Returns the rate `hal.rcout->set_freq(chmask, hz)` should receive.
#[must_use]
pub fn apply_rc_speed(hz: i16) -> RcSpeed {
    RcSpeed::from_param(hz)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_upstream_quadplane() {
        assert_eq!(RC_SPEED_DEFAULT, 490);
        assert_eq!(RC_SPEED_MIN, 50);
        assert_eq!(RC_SPEED_MAX, 500);
        assert_eq!(RC_FAST_SPEED, 490);
        assert_eq!(RcSpeed::default().hz, RC_SPEED_DEFAULT);
        assert_eq!(RcSpeed::default().period_us(), 1_000_000 / 490);
    }

    #[test]
    fn analog_50hz_is_20ms_frame() {
        assert_eq!(pwm_period_us(RC_SPEED_MIN), 20_000);
    }

    #[test]
    fn max_500hz_is_2ms_frame() {
        assert_eq!(pwm_period_us(RC_SPEED_MAX), 2_000);
    }

    #[test]
    fn clamp_rejects_zero_and_out_of_range() {
        assert_eq!(clamp_rc_speed(0), RC_SPEED_MIN);
        assert_eq!(clamp_rc_speed(-10), RC_SPEED_MIN);
        assert_eq!(clamp_rc_speed(49), RC_SPEED_MIN);
        assert_eq!(clamp_rc_speed(50), RC_SPEED_MIN);
        assert_eq!(clamp_rc_speed(490), RC_SPEED_DEFAULT);
        assert_eq!(clamp_rc_speed(500), RC_SPEED_MAX);
        assert_eq!(clamp_rc_speed(501), RC_SPEED_MAX);
        assert_eq!(clamp_rc_speed(i16::MAX), RC_SPEED_MAX);
    }

    #[test]
    fn apply_rc_speed_clamps_then_converts_period() {
        let applied = apply_rc_speed(0);
        assert_eq!(applied.hz, RC_SPEED_MIN);
        assert_eq!(applied.period_us(), 20_000);
        let fast = apply_rc_speed(RC_SPEED_DEFAULT as i16);
        assert_eq!(fast.hz, RC_SPEED_DEFAULT);
        assert_eq!(fast.period_us(), 2040);
    }
}
