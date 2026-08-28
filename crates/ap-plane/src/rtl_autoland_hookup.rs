//! RTL_AUTOLAND glue: after RTL home loiter, jump to a landing sequence.
//!
//! Upstream `ModeRTL::navigate` consults `RTL_AUTOLAND` once
//! `auto_state.checked_for_autoland` is false. Immediate or then-land
//! calls `mission.jump_to_landing_sequence`; return-path calls
//! `jump_to_closest_mission_leg`. A successful jump sets force-resume and
//! switches to AUTO with `ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND`.
//! QRTL / VTOL `switch_QRTL` is deferred.

use crate::mode_table::{BuildFeatures, ModeNumber};

fn is_rtl_mode(control_mode: u8, features: &BuildFeatures) -> bool {
    ModeNumber::from_number(control_mode, features) == Some(ModeNumber::Rtl)
}

/// Upstream `RtlAutoland::RTL_DISABLE`.
pub const RTL_AUTOLAND_DISABLE: u8 = 0;
/// Upstream `RtlAutoland::RTL_THEN_DO_LAND_START`.
pub const RTL_AUTOLAND_THEN_DO_LAND_START: u8 = 1;
/// Upstream `RtlAutoland::RTL_IMMEDIATE_DO_LAND_START`.
pub const RTL_AUTOLAND_IMMEDIATE_DO_LAND_START: u8 = 2;
/// Upstream `RtlAutoland::NO_RTL_GO_AROUND` (does not change RTL).
pub const RTL_AUTOLAND_NO_RTL_GO_AROUND: u8 = 3;
/// Upstream `RtlAutoland::DO_RETURN_PATH_START`.
pub const RTL_AUTOLAND_DO_RETURN_PATH_START: u8 = 4;

/// Upstream `ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND`.
pub const MODE_REASON_RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND: u8 = 40;

/// Upstream `labs(calc_altitude_error_cm()) < 1000` gate for then-land.
pub const RTL_AUTOLAND_ALT_ERROR_CM: i32 = 1000;

/// Inputs for the RTL_AUTOLAND landing / return-path handoff.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtlAutolandInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    /// Upstream `g.rtl_autoland` (`RTL_AUTOLAND`).
    pub rtl_autoland: u8,
    /// Upstream `auto_state.checked_for_autoland`.
    pub checked_for_autoland: bool,
    /// Upstream `have_position`.
    pub have_position: bool,
    /// `mission.jump_to_landing_sequence(current_loc)` would succeed.
    pub have_landing_sequence: bool,
    /// `mission.jump_to_closest_mission_leg(current_loc)` would succeed.
    pub have_return_path: bool,
    /// Upstream `reached_loiter_target()`.
    pub reached_loiter_target: bool,
    /// Upstream `labs(calc_altitude_error_cm())`.
    pub alt_error_cm: i32,
}

/// Result of the RTL_AUTOLAND handoff tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtlAutolandOutput {
    /// `set_mode(AUTO, RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND)`.
    pub switch_to_auto: bool,
    /// `mission.set_force_resume(true)` before the mode switch.
    pub force_resume: bool,
    /// Jump via `DO_LAND_START` / landing sequence.
    pub jump_landing_sequence: bool,
    /// Jump via `DO_RETURN_PATH_START` / closest mission leg.
    pub jump_return_path: bool,
    /// Updated `auto_state.checked_for_autoland`.
    pub checked_for_autoland: bool,
    /// Mode-reason for the AUTO switch, or 0.
    pub mode_reason: u8,
    pub applied: bool,
}

fn idle(checked: bool) -> RtlAutolandOutput {
    RtlAutolandOutput {
        switch_to_auto: false,
        force_resume: false,
        jump_landing_sequence: false,
        jump_return_path: false,
        checked_for_autoland: checked,
        mode_reason: 0,
        applied: true,
    }
}

fn switch_auto(landing: bool, return_path: bool) -> RtlAutolandOutput {
    RtlAutolandOutput {
        switch_to_auto: true,
        force_resume: true,
        jump_landing_sequence: landing,
        jump_return_path: return_path,
        checked_for_autoland: true,
        mode_reason: MODE_REASON_RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND,
        applied: true,
    }
}

/// After RTL loiter, optionally jump to a landing sequence or return-path
/// and switch to AUTO, matching `ModeRTL::navigate` `RTL_AUTOLAND` handling.
#[must_use]
pub fn rtl_autoland_tick(inp: &RtlAutolandInputs) -> RtlAutolandOutput {
    if !is_rtl_mode(inp.control_mode, &inp.features) {
        return RtlAutolandOutput {
            switch_to_auto: false,
            force_resume: false,
            jump_landing_sequence: false,
            jump_return_path: false,
            checked_for_autoland: false,
            mode_reason: 0,
            applied: false,
        };
    }

    if inp.checked_for_autoland {
        return idle(true);
    }

    let then_ready = inp.reached_loiter_target && inp.alt_error_cm < RTL_AUTOLAND_ALT_ERROR_CM;
    let try_landing = inp.rtl_autoland == RTL_AUTOLAND_IMMEDIATE_DO_LAND_START
        || (inp.rtl_autoland == RTL_AUTOLAND_THEN_DO_LAND_START && then_ready);

    if try_landing {
        if inp.have_position && inp.have_landing_sequence {
            return switch_auto(true, false);
        }
        return idle(true);
    }

    if inp.rtl_autoland == RTL_AUTOLAND_DO_RETURN_PATH_START {
        if inp.have_position && inp.have_return_path {
            return switch_auto(false, true);
        }
        return idle(true);
    }

    idle(false)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rtl_inp() -> RtlAutolandInputs {
        RtlAutolandInputs {
            control_mode: ModeNumber::Rtl.as_number(),
            features: BuildFeatures::default(),
            rtl_autoland: RTL_AUTOLAND_IMMEDIATE_DO_LAND_START,
            checked_for_autoland: false,
            have_position: true,
            have_landing_sequence: true,
            have_return_path: false,
            reached_loiter_target: false,
            alt_error_cm: 0,
        }
    }

    #[test]
    fn immediate_jumps_landing_sequence_and_switches_auto() {
        let out = rtl_autoland_tick(&rtl_inp());
        assert!(out.applied);
        assert!(out.switch_to_auto);
        assert!(out.force_resume);
        assert!(out.jump_landing_sequence);
        assert!(!out.jump_return_path);
        assert!(out.checked_for_autoland);
        assert_eq!(
            out.mode_reason,
            MODE_REASON_RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND
        );
    }

    #[test]
    fn immediate_without_sequence_marks_checked() {
        let mut inp = rtl_inp();
        inp.have_landing_sequence = false;
        let out = rtl_autoland_tick(&inp);
        assert!(out.applied);
        assert!(!out.switch_to_auto);
        assert!(out.checked_for_autoland);
    }

    #[test]
    fn then_land_waits_for_loiter_and_alt() {
        let mut inp = rtl_inp();
        inp.rtl_autoland = RTL_AUTOLAND_THEN_DO_LAND_START;
        inp.reached_loiter_target = false;
        let out = rtl_autoland_tick(&inp);
        assert!(out.applied);
        assert!(!out.switch_to_auto);
        assert!(!out.checked_for_autoland);
    }

    #[test]
    fn then_land_ready_switches_auto() {
        let mut inp = rtl_inp();
        inp.rtl_autoland = RTL_AUTOLAND_THEN_DO_LAND_START;
        inp.reached_loiter_target = true;
        inp.alt_error_cm = 999;
        let out = rtl_autoland_tick(&inp);
        assert!(out.switch_to_auto);
        assert!(out.jump_landing_sequence);
        assert!(out.checked_for_autoland);
    }

    #[test]
    fn then_land_alt_error_at_threshold_waits() {
        let mut inp = rtl_inp();
        inp.rtl_autoland = RTL_AUTOLAND_THEN_DO_LAND_START;
        inp.reached_loiter_target = true;
        inp.alt_error_cm = RTL_AUTOLAND_ALT_ERROR_CM;
        let out = rtl_autoland_tick(&inp);
        assert!(!out.switch_to_auto);
        assert!(!out.checked_for_autoland);
    }

    #[test]
    fn return_path_jumps_closest_leg() {
        let mut inp = rtl_inp();
        inp.rtl_autoland = RTL_AUTOLAND_DO_RETURN_PATH_START;
        inp.have_landing_sequence = false;
        inp.have_return_path = true;
        let out = rtl_autoland_tick(&inp);
        assert!(out.switch_to_auto);
        assert!(out.jump_return_path);
        assert!(!out.jump_landing_sequence);
        assert_eq!(
            out.mode_reason,
            MODE_REASON_RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND
        );
    }

    #[test]
    fn disable_and_go_around_do_not_check() {
        for mode in [RTL_AUTOLAND_DISABLE, RTL_AUTOLAND_NO_RTL_GO_AROUND] {
            let mut inp = rtl_inp();
            inp.rtl_autoland = mode;
            let out = rtl_autoland_tick(&inp);
            assert!(out.applied);
            assert!(!out.switch_to_auto);
            assert!(!out.checked_for_autoland);
        }
    }

    #[test]
    fn already_checked_does_not_switch() {
        let mut inp = rtl_inp();
        inp.checked_for_autoland = true;
        let out = rtl_autoland_tick(&inp);
        assert!(out.applied);
        assert!(!out.switch_to_auto);
        assert!(out.checked_for_autoland);
    }

    #[test]
    fn skips_other_modes() {
        let mut inp = rtl_inp();
        inp.control_mode = ModeNumber::Auto.as_number();
        let out = rtl_autoland_tick(&inp);
        assert!(!out.applied);
        assert!(!out.switch_to_auto);
    }
}
