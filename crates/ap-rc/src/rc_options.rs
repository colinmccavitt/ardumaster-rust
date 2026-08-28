//! `RC_OPTIONS` bitfield, upstream `RC_Channels::Option`.
//!
//! The `RC_OPTIONS` parameter is a bitmask that changes how radio PWM
//! and GCS overrides are consumed. Decode is `option_is_enabled`; apply
//! is the `RC_Channel::update` source pick, the protocol failsafe gate,
//! the RC arming checks, and whether aux switches honor `RCx_REVERSED`.
//!
//! Override *timeout* stays in [`crate::override_timeout`]. This module
//! only reads the already-settled `has_override` flag.

/// Upstream `RC_OPTIONS` default: `Option::ARMING_CHECK_THROTTLE`.
pub const RC_OPTIONS_DEFAULT: u32 = RcOption::ArmingCheckThrottle as u32;

/// Upstream `RC_Channels::Option` bits (`RC_OPTIONS`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum RcOption {
    /// Bit 0: ignore attached RC receiver outputs.
    IgnoreReceiver = 1 << 0,
    /// Bit 1: ignore MAVLink `RC_CHANNELS_OVERRIDE`.
    IgnoreOverrides = 1 << 1,
    /// Bit 2: ignore the receiver protocol failsafe bit.
    IgnoreFailsafe = 1 << 2,
    /// Bit 3: pad FPort telemetry output.
    FportPad = 1 << 3,
    /// Bit 4: log raw RC input bytes.
    LogRawData = 1 << 4,
    /// Bit 5: require idle throttle to arm.
    ArmingCheckThrottle = 1 << 5,
    /// Bit 6: skip roll/pitch/yaw stick-neutral arming checks.
    ArmingSkipCheckRpy = 1 << 6,
    /// Bit 7: honor `RCx_REVERSED` on aux switches.
    AllowSwitchRev = 1 << 7,
    /// Bit 8: CRSF passthrough telemetry.
    CrsfCustomTelemetry = 1 << 8,
    /// Bit 9: suppress CRSF mode/rate messages (ELRS).
    SuppressCrsfMessage = 1 << 9,
    /// Bit 10: allow multiple receivers.
    MultiReceiverSupport = 1 << 10,
    /// Bit 11: report CRSF link quality as RSSI.
    UseCrsfLqAsRssi = 1 << 11,
    /// Bit 12: annotate CRSF flight mode with `*` when disarmed.
    CrsfFmDisarmStar = 1 << 12,
    /// Bit 13: 420 kbaud ELRS protocol.
    Elrs420kbaud = 1 << 13,
}

impl RcOption {
    /// Bit value stored in `RC_OPTIONS`.
    #[must_use]
    pub const fn bit(self) -> u32 {
        self as u32
    }
}

/// Decoded `RC_OPTIONS` bitmask, upstream `RC_Channels::_options`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcOptions {
    /// Raw `RC_OPTIONS` integer.
    pub bits: u32,
}

impl Default for RcOptions {
    fn default() -> Self {
        Self {
            bits: RC_OPTIONS_DEFAULT,
        }
    }
}

impl RcOptions {
    /// Wrap a stored `RC_OPTIONS` value.
    #[must_use]
    pub const fn from_bits(bits: u32) -> Self {
        Self { bits }
    }

    /// Upstream `RC_Channels::option_is_enabled`.
    #[must_use]
    pub const fn option_is_enabled(self, option: RcOption) -> bool {
        self.bits & option as u32 != 0
    }

    /// Bit 0: do not consume receiver PWM.
    #[must_use]
    pub const fn ignore_receiver(self) -> bool {
        self.option_is_enabled(RcOption::IgnoreReceiver)
    }

    /// Bit 1: do not consume GCS overrides.
    #[must_use]
    pub const fn ignore_overrides(self) -> bool {
        self.option_is_enabled(RcOption::IgnoreOverrides)
    }

    /// Inverse of [`Self::ignore_overrides`]: apply a live GCS override.
    #[must_use]
    pub const fn honor_overrides(self) -> bool {
        !self.ignore_overrides()
    }

    /// Bit 2: drop the protocol failsafe flag.
    #[must_use]
    pub const fn ignore_failsafe(self) -> bool {
        self.option_is_enabled(RcOption::IgnoreFailsafe)
    }

    /// Bit 5: run the idle-throttle arming check.
    #[must_use]
    pub const fn arming_check_throttle(self) -> bool {
        self.option_is_enabled(RcOption::ArmingCheckThrottle)
    }

    /// Bit 6: skip roll/pitch/yaw stick-neutral arming checks.
    #[must_use]
    pub const fn arming_skip_check_rpy(self) -> bool {
        self.option_is_enabled(RcOption::ArmingSkipCheckRpy)
    }

    /// Bit 7: honor `RCx_REVERSED` on option switches.
    #[must_use]
    pub const fn allow_switch_rev(self) -> bool {
        self.option_is_enabled(RcOption::AllowSwitchRev)
    }
}

/// Which RC stick-neutral arming checks run, upstream `AP_Arming::rc_checks`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcArmingChecks {
    /// `rc().arming_check_throttle()` — throttle `control_in` must be 0.
    pub check_throttle_idle: bool,
    /// `!ARMING_SKIP_CHECK_RPY` — roll/pitch/yaw must be near trim.
    pub check_rpy_neutral: bool,
}

/// Source pick from `RC_Channel::update`.
///
/// A live override wins unless `IGNORE_OVERRIDES` is set. Otherwise a
/// receiver that has been seen wins unless `IGNORE_RECEIVER` is set.
/// Both gated off returns `None` (update fails, `radio_in` is unchanged).
#[must_use]
pub fn apply_radio_in(
    options: RcOptions,
    has_override: bool,
    has_had_rc_receiver: bool,
    override_value: u16,
    receiver_pwm: u16,
) -> Option<u16> {
    if has_override && options.honor_overrides() {
        Some(override_value)
    } else if has_had_rc_receiver && !options.ignore_receiver() {
        Some(receiver_pwm)
    } else {
        None
    }
}

/// Protocol failsafe after `IGNORE_FAILSAFE`, upstream `AP_RCProtocol_Backend`.
///
/// When the bit is set the receiver failsafe flag is forced clear so a
/// GCS can fly past RC range without an RC failsafe.
#[must_use]
pub fn apply_receiver_failsafe(options: RcOptions, protocol_failsafe: bool) -> bool {
    if options.ignore_failsafe() {
        false
    } else {
        protocol_failsafe
    }
}

/// RC stick arming checks implied by `RC_OPTIONS`.
#[must_use]
pub fn apply_arming_rc_checks(options: RcOptions) -> RcArmingChecks {
    RcArmingChecks {
        check_throttle_idle: options.arming_check_throttle(),
        check_rpy_neutral: !options.arming_skip_check_rpy(),
    }
}

/// Aux-switch reverse after `ALLOW_SWITCH_REV`.
///
/// Upstream `read_3pos_switch`: reverse only when the channel is reversed
/// *and* `RC_OPTIONS` allows it.
#[must_use]
pub fn apply_switch_reversed(options: RcOptions, channel_reversed: bool) -> bool {
    channel_reversed && options.allow_switch_rev()
}

/// Applied `RC_OPTIONS` view for one radio sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcOptionsApplied {
    /// Selected `radio_in`, or `None` when both sources are gated off.
    pub radio_in: Option<u16>,
    /// Failsafe after `IGNORE_FAILSAFE`.
    pub in_failsafe: bool,
    /// Idle-throttle arming check is live.
    pub check_throttle_idle: bool,
    /// Roll/pitch/yaw stick-neutral arming check is live.
    pub check_rpy_neutral: bool,
    /// Aux switch should flip LOW/HIGH.
    pub switch_reversed: bool,
}

/// Decode `RC_OPTIONS` and apply it to one sample.
#[must_use]
pub fn apply_rc_options(
    options: RcOptions,
    has_override: bool,
    has_had_rc_receiver: bool,
    override_value: u16,
    receiver_pwm: u16,
    protocol_failsafe: bool,
    channel_reversed: bool,
) -> RcOptionsApplied {
    let checks = apply_arming_rc_checks(options);
    RcOptionsApplied {
        radio_in: apply_radio_in(
            options,
            has_override,
            has_had_rc_receiver,
            override_value,
            receiver_pwm,
        ),
        in_failsafe: apply_receiver_failsafe(options, protocol_failsafe),
        check_throttle_idle: checks.check_throttle_idle,
        check_rpy_neutral: checks.check_rpy_neutral,
        switch_reversed: apply_switch_reversed(options, channel_reversed),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_arming_check_throttle() {
        assert_eq!(RC_OPTIONS_DEFAULT, 1 << 5);
        assert_eq!(RcOption::ArmingCheckThrottle.bit(), 32);
        let opts = RcOptions::default();
        assert_eq!(opts.bits, RC_OPTIONS_DEFAULT);
        assert!(opts.arming_check_throttle());
        assert!(!opts.ignore_receiver());
        assert!(!opts.ignore_overrides());
        assert!(opts.honor_overrides());
        assert!(!opts.ignore_failsafe());
        assert!(!opts.arming_skip_check_rpy());
        assert!(!opts.allow_switch_rev());
    }

    #[test]
    fn each_documented_bit_decodes() {
        let bits = RcOption::IgnoreReceiver.bit()
            | RcOption::IgnoreOverrides.bit()
            | RcOption::IgnoreFailsafe.bit()
            | RcOption::ArmingSkipCheckRpy.bit()
            | RcOption::AllowSwitchRev.bit();
        let opts = RcOptions::from_bits(bits);
        assert!(opts.ignore_receiver());
        assert!(opts.ignore_overrides());
        assert!(!opts.honor_overrides());
        assert!(opts.ignore_failsafe());
        assert!(opts.arming_skip_check_rpy());
        assert!(opts.allow_switch_rev());
        assert!(!opts.arming_check_throttle());
    }

    #[test]
    fn honor_overrides_selects_gcs_pwm() {
        let opts = RcOptions::default();
        assert_eq!(apply_radio_in(opts, true, true, 1650, 1500), Some(1650));
    }

    #[test]
    fn ignore_overrides_falls_back_to_receiver() {
        let opts = RcOptions::from_bits(RcOption::IgnoreOverrides.bit());
        assert_eq!(apply_radio_in(opts, true, true, 1650, 1500), Some(1500));
    }

    #[test]
    fn ignore_receiver_without_override_fails_update() {
        let opts = RcOptions::from_bits(RcOption::IgnoreReceiver.bit());
        assert_eq!(apply_radio_in(opts, false, true, 1650, 1500), None);
    }

    #[test]
    fn ignore_failsafe_clears_protocol_bit() {
        let opts = RcOptions::from_bits(RcOption::IgnoreFailsafe.bit());
        assert!(!apply_receiver_failsafe(opts, true));
        assert!(apply_receiver_failsafe(RcOptions::default(), true));
    }

    #[test]
    fn arming_bits_select_stick_checks() {
        let default_checks = apply_arming_rc_checks(RcOptions::default());
        assert!(default_checks.check_throttle_idle);
        assert!(default_checks.check_rpy_neutral);
        let skip = RcOptions::from_bits(RcOption::ArmingSkipCheckRpy.bit());
        let checks = apply_arming_rc_checks(skip);
        assert!(!checks.check_throttle_idle);
        assert!(!checks.check_rpy_neutral);
    }

    #[test]
    fn switch_reverse_requires_option_and_channel() {
        let opts = RcOptions::from_bits(RcOption::AllowSwitchRev.bit());
        assert!(apply_switch_reversed(opts, true));
        assert!(!apply_switch_reversed(opts, false));
        assert!(!apply_switch_reversed(RcOptions::default(), true));
    }
}
