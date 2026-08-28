//! RC aux-function switch latch, upstream `RC_Channel::read_aux`.
//!
//! `RCn_OPTION` maps a receiver channel onto an auxiliary function. The
//! channel is not a stick: it is a 3-position switch. PWM below 1200 µs is
//! LOW, above 1800 µs is HIGH, and the band between is MIDDLE. A new
//! position is latched only after it has stayed put for
//! [`SWITCH_DEBOUNCE_TIME_MS`], matching `RC_Channel::debounce_completed`.
//! Without that latch a noisy edge would fire the function twice.

/// Upstream `RC_Channel::AUX_SWITCH_PWM_TRIGGER_LOW`.
pub const AUX_SWITCH_PWM_TRIGGER_LOW: u16 = 1200;
/// Upstream `RC_Channel::AUX_SWITCH_PWM_TRIGGER_HIGH`.
pub const AUX_SWITCH_PWM_TRIGGER_HIGH: u16 = 1800;
/// Upstream `RC_Channel::RC_MIN_LIMIT_PWM`. PWM at or below this is invalid.
pub const RC_MIN_LIMIT_PWM: u16 = 800;
/// Upstream `RC_Channel::RC_MAX_LIMIT_PWM`. PWM at or above this is invalid.
pub const RC_MAX_LIMIT_PWM: u16 = 2200;
/// Upstream `SWITCH_DEBOUNCE_TIME_MS` in `RC_Channel.cpp`.
pub const SWITCH_DEBOUNCE_TIME_MS: u32 = 200;

/// 3-position aux switch, upstream `RC_Channel::AuxSwitchPos`.
///
/// Stored as two bits upstream (`LOW` = 0, `MIDDLE` = 1, `HIGH` = 2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AuxSwitchPos {
    /// PWM `< 1200` (or reversed HIGH).
    Low = 0,
    /// PWM in `[1200, 1800]`.
    Middle = 1,
    /// PWM `> 1800` (or reversed LOW).
    High = 2,
}

/// `RCn_OPTION` aux-function codes used by this slice.
///
/// The full upstream table is hundreds of values. This stub keeps the
/// Plane-facing ones the latch must distinguish: disabled, a normal
/// 3-position function, and arm/disarm (which latches on first read
/// without firing).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum AuxFunc {
    /// Upstream `AUX_FUNC::DO_NOTHING` / `RCn_OPTION = 0`.
    DoNothing = 0,
    /// Upstream `AUX_FUNC::FENCE` / `RCn_OPTION = 11`.
    Fence = 11,
    /// Upstream `AUX_FUNC::REVERSE_THROTTLE` / `RCn_OPTION = 64`.
    ReverseThrottle = 64,
    /// Upstream `AUX_FUNC::Q_ASSIST` / `RCn_OPTION = 82`.
    QAssist = 82,
    /// Upstream `AUX_FUNC::SOARING` / `RCn_OPTION = 88`.
    Soaring = 88,
    /// Upstream `AUX_FUNC::ARMDISARM` / `RCn_OPTION = 153`.
    ArmDisarm = 153,
}

/// Map PWM microseconds to a 3-position switch, upstream `read_3pos_switch`.
///
/// Returns `None` when the pulse is outside `[RC_MIN_LIMIT_PWM,
/// RC_MAX_LIMIT_PWM)` — the same error condition as upstream. Thresholds
/// are exclusive at the LOW/HIGH edges: 1200 and 1800 are MIDDLE.
#[must_use]
pub fn read_3pos_switch(pwm: u16, reversed: bool) -> Option<AuxSwitchPos> {
    if pwm <= RC_MIN_LIMIT_PWM || pwm >= RC_MAX_LIMIT_PWM {
        return None;
    }
    let pos = if pwm < AUX_SWITCH_PWM_TRIGGER_LOW {
        AuxSwitchPos::Low
    } else if pwm > AUX_SWITCH_PWM_TRIGGER_HIGH {
        AuxSwitchPos::High
    } else {
        AuxSwitchPos::Middle
    };
    Some(if reversed { reverse_pos(pos) } else { pos })
}

/// Switch position, falling back to LOW when the pulse is invalid.
///
/// Upstream `RC_Channel::get_aux_switch_pos`.
#[must_use]
pub fn get_aux_switch_pos(pwm: u16, reversed: bool) -> AuxSwitchPos {
    read_3pos_switch(pwm, reversed).unwrap_or(AuxSwitchPos::Low)
}

fn reverse_pos(pos: AuxSwitchPos) -> AuxSwitchPos {
    match pos {
        AuxSwitchPos::Low => AuxSwitchPos::High,
        AuxSwitchPos::Middle => AuxSwitchPos::Middle,
        AuxSwitchPos::High => AuxSwitchPos::Low,
    }
}

/// True when the first valid 3-position read should latch without firing.
///
/// Upstream `RC_Channel::init_position_on_first_radio_read`. Arming from
/// a transmitter that powers up with the arm switch HIGH would be a
/// surprise, so those options record the first position and wait for a
/// later change.
#[must_use]
pub fn init_position_on_first_radio_read(func: AuxFunc) -> bool {
    matches!(func, AuxFunc::ArmDisarm)
}

/// Debounced latch for one `RCn_OPTION` channel, upstream `switch_state`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AuxSwitchLatch {
    /// Assigned `RCn_OPTION`.
    pub option: AuxFunc,
    /// Honor channel reverse on the 3-position map.
    pub reversed: bool,
    debounce_position: Option<AuxSwitchPos>,
    current_position: Option<AuxSwitchPos>,
    last_edge_time_ms: u32,
    initialised: bool,
}

impl AuxSwitchLatch {
    /// Unlatched channel with the given `RCn_OPTION`.
    #[must_use]
    pub const fn new(option: AuxFunc) -> Self {
        Self {
            option,
            reversed: false,
            debounce_position: None,
            current_position: None,
            last_edge_time_ms: 0,
            initialised: false,
        }
    }

    /// Last latched position, if any has settled.
    #[must_use]
    pub const fn current_position(&self) -> Option<AuxSwitchPos> {
        self.current_position
    }

    /// Read the channel. Returns `Some(pos)` only when a new position latches.
    ///
    /// Upstream `RC_Channel::read_aux`. `DO_NOTHING` never fires. Invalid
    /// PWM is ignored. A first-read arm option latches without returning
    /// a trigger.
    pub fn read_aux(&mut self, pwm: u16, now_ms: u32) -> Option<AuxSwitchPos> {
        if self.option == AuxFunc::DoNothing {
            return None;
        }
        let new_position = read_3pos_switch(pwm, self.reversed)?;

        if !self.initialised {
            self.initialised = true;
            if init_position_on_first_radio_read(self.option) {
                self.current_position = Some(new_position);
                self.debounce_position = Some(new_position);
                return None;
            }
        }

        if self.debounce_completed(new_position, now_ms) {
            Some(new_position)
        } else {
            None
        }
    }

    /// Upstream `RC_Channel::debounce_completed`.
    fn debounce_completed(&mut self, position: AuxSwitchPos, now_ms: u32) -> bool {
        if self.current_position == Some(position) {
            self.debounce_position = Some(position);
            return false;
        }
        if self.debounce_position != Some(position) {
            self.debounce_position = Some(position);
            self.last_edge_time_ms = now_ms;
            return false;
        }
        if now_ms.wrapping_sub(self.last_edge_time_ms) >= SWITCH_DEBOUNCE_TIME_MS {
            self.current_position = Some(position);
            return true;
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn three_pos_thresholds_match_upstream() {
        assert_eq!(read_3pos_switch(1199, false), Some(AuxSwitchPos::Low));
        assert_eq!(read_3pos_switch(1200, false), Some(AuxSwitchPos::Middle));
        assert_eq!(read_3pos_switch(1500, false), Some(AuxSwitchPos::Middle));
        assert_eq!(read_3pos_switch(1800, false), Some(AuxSwitchPos::Middle));
        assert_eq!(read_3pos_switch(1801, false), Some(AuxSwitchPos::High));
    }

    #[test]
    fn invalid_pwm_is_not_a_switch_position() {
        assert_eq!(read_3pos_switch(800, false), None);
        assert_eq!(read_3pos_switch(2200, false), None);
        assert_eq!(get_aux_switch_pos(800, false), AuxSwitchPos::Low);
    }

    #[test]
    fn reversed_flips_low_and_high() {
        assert_eq!(read_3pos_switch(1100, true), Some(AuxSwitchPos::High));
        assert_eq!(read_3pos_switch(1900, true), Some(AuxSwitchPos::Low));
        assert_eq!(read_3pos_switch(1500, true), Some(AuxSwitchPos::Middle));
    }

    #[test]
    fn do_nothing_never_latches() {
        let mut latch = AuxSwitchLatch::new(AuxFunc::DoNothing);
        assert_eq!(latch.read_aux(1100, 0), None);
        assert_eq!(latch.read_aux(1100, 500), None);
        assert_eq!(latch.current_position(), None);
    }

    #[test]
    fn new_position_latches_after_debounce() {
        let mut latch = AuxSwitchLatch::new(AuxFunc::Fence);
        assert_eq!(latch.read_aux(1100, 0), None);
        assert_eq!(latch.read_aux(1100, 199), None);
        assert_eq!(latch.read_aux(1100, 200), Some(AuxSwitchPos::Low));
        assert_eq!(latch.current_position(), Some(AuxSwitchPos::Low));
        // already latched: same position does not fire again
        assert_eq!(latch.read_aux(1100, 400), None);
    }

    #[test]
    fn bounce_resets_the_edge_timer() {
        let mut latch = AuxSwitchLatch::new(AuxFunc::Soaring);
        assert_eq!(latch.read_aux(1900, 0), None);
        assert_eq!(latch.read_aux(1500, 100), None);
        assert_eq!(latch.read_aux(1500, 299), None);
        assert_eq!(latch.read_aux(1500, 300), Some(AuxSwitchPos::Middle));
    }

    #[test]
    fn armdisarm_first_read_latches_without_firing() {
        let mut latch = AuxSwitchLatch::new(AuxFunc::ArmDisarm);
        assert!(init_position_on_first_radio_read(AuxFunc::ArmDisarm));
        assert_eq!(latch.read_aux(1900, 0), None);
        assert_eq!(latch.current_position(), Some(AuxSwitchPos::High));
        assert_eq!(latch.read_aux(1900, 200), None);
        assert_eq!(latch.read_aux(1100, 200), None);
        assert_eq!(latch.read_aux(1100, 400), Some(AuxSwitchPos::Low));
    }
}
