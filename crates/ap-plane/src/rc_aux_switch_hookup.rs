//! RC aux-function switch latch hookup for the vehicle loop.
//!
//! Upstream `RC_Channels::read_aux_all` walks every channel whose
//! `RCn_OPTION` is not `DO_NOTHING` and asks `RC_Channel::read_aux` to
//! debounce the 3-position switch. This hookup is the vehicle-side call
//! into [`ap_rc::AuxSwitchLatch`].

use ap_rc::{AuxFunc, AuxSwitchLatch, AuxSwitchPos};

/// Frontend aux-switch hookup for one `RCn_OPTION` channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcAuxSwitchHookup {
    latch: AuxSwitchLatch,
}

impl Default for RcAuxSwitchHookup {
    fn default() -> Self {
        Self::from_option(AuxFunc::DoNothing)
    }
}

impl RcAuxSwitchHookup {
    /// Bind a vehicle aux channel to an `RCn_OPTION`.
    #[must_use]
    pub const fn from_option(option: AuxFunc) -> Self {
        Self {
            latch: AuxSwitchLatch::new(option),
        }
    }

    /// Assigned `RCn_OPTION`.
    #[must_use]
    pub const fn option(&self) -> AuxFunc {
        self.latch.option
    }

    /// Last latched 3-position value, if the switch has settled.
    #[must_use]
    pub const fn current_position(&self) -> Option<AuxSwitchPos> {
        self.latch.current_position()
    }

    /// Debounce PWM and return a newly latched position, if any.
    pub fn read(&mut self, pwm: u16, now_ms: u32) -> Option<AuxSwitchPos> {
        self.latch.read_aux(pwm, now_ms)
    }
}

/// Read one aux channel through a hookup, same as [`RcAuxSwitchHookup::read`].
#[must_use]
pub fn read_aux_option(
    hookup: &mut RcAuxSwitchHookup,
    pwm: u16,
    now_ms: u32,
) -> Option<AuxSwitchPos> {
    hookup.read(pwm, now_ms)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_option_is_disabled() {
        let mut hookup = RcAuxSwitchHookup::default();
        assert_eq!(hookup.option(), AuxFunc::DoNothing);
        assert_eq!(hookup.read(1100, 200), None);
    }

    #[test]
    fn soaring_option_latches_after_debounce() {
        let mut hookup = RcAuxSwitchHookup::from_option(AuxFunc::Soaring);
        assert_eq!(read_aux_option(&mut hookup, 1900, 0), None);
        assert_eq!(
            read_aux_option(&mut hookup, 1900, 200),
            Some(AuxSwitchPos::High)
        );
    }
}
