//! `ModeAuto::init` leftover, upstream `ArduCopter/mode_auto.cpp`.
//!
//! Tracked as **COP-016**. AUTO is the mission mode: enter it and the
//! vehicle waits for an EKF origin, then starts or resumes the plan. Copter
//! names the enter `init`, not `_enter`. This file owns that enter — the
//! two refuses, the LOITER parking submode, and the leftover that must be
//! true before `run` is allowed to start the mission. Command dispatch
//! (`start_command`) and the submode run bodies are later slices.
//!
//! # No mission is a refuse unless the caller said to ignore checks
//!
//! `mission.present()` is `_cmd_total > 1` — home plus at least one real
//! item. A bench GCS that has not loaded a plan must not put the aircraft
//! into AUTO and then wonder why nothing happens. `ignore_checks` is the
//! disarmed `set_mode` path: a vehicle on the bench is allowed in so the
//! operator can load the mission afterwards. The takeoff gate below is
//! *not* suppressed by `ignore_checks`.
//!
//! # Landed and armed without a takeoff is a flip
//!
//! The second refuse is `armed && land_complete &&
//! !starts_with_takeoff_cmd()`. A copter that tries to start a waypoint
//! from the ground with motors spinning will lift on whatever lean the
//! first WP demands. The GCS text is `"Auto: Missing Takeoff Cmd"`.
//! Disarmed, airborne, or a plan that actually starts with takeoff all
//! pass. `auto_RTL` is cleared *before* either gate, so a failed enter
//! does not leave a stale AUTO_RTL report from the previous visit.
//!
//! # Success parks in LOITER and waits
//!
//! `_mode` becomes `SubMode::LOITER` — not WP — because there is not yet
//! a destination. `waiting_to_start` is set so `run` will call
//! `mission.start_or_resume()` once `ahrs.get_origin` succeeds. Speed
//! overrides are zeroed (0 means "unset"), guided limits are cleared, and
//! a leftover ROI yaw is forced to HOLD so a previous mission's look-at
//! does not steer the first loiter. `wp_and_spline_init_m()` runs with
//! defaults; the destination is not set here.

/// `Mode::Number::AUTO`.
pub const MODE_NUMBER_AUTO: u8 = 3;

/// `Mode::Number::AUTO_RTL` — report-only; AUTO pretends this when
/// `auto_RTL` is set. `init` always clears that flag.
pub const MODE_NUMBER_AUTO_RTL: u8 = 27;

/// `ModeAuto` capability flags from `mode.h` after a successful `init`.
///
/// `requires_position` is `_mode != NAV_ATTITUDE_TIME`. Init parks in
/// LOITER, so the leftover is true. `allows_arming` is not here: it
/// depends on `auto_RTL` and `Option::AllowArming`, neither of which
/// `init` decides beyond clearing `auto_RTL`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoModeFlags {
    /// `mode_number()` with `auto_RTL == false`.
    pub mode_number: u8,
    /// `requires_position()` in the LOITER submode `init` selects.
    pub requires_position: bool,
    /// `has_manual_throttle()`.
    pub has_manual_throttle: bool,
    /// `is_autopilot()`.
    pub is_autopilot: bool,
    /// `allows_GCS_or_SCR_arming_with_throttle_high()`.
    pub allows_gcs_or_scr_arming_with_throttle_high: bool,
    /// `requires_terrain_failsafe()`.
    pub requires_terrain_failsafe: bool,
}

/// Upstream `ModeAuto` flags after `init` succeeds.
#[must_use]
pub const fn auto_mode_flags() -> AutoModeFlags {
    AutoModeFlags {
        mode_number: MODE_NUMBER_AUTO,
        requires_position: true,
        has_manual_throttle: false,
        is_autopilot: true,
        allows_gcs_or_scr_arming_with_throttle_high: true,
        requires_terrain_failsafe: true,
    }
}

/// `ModeAuto::SubMode`. Discriminants match the `uint8_t` enum in `mode.h`
/// through `LOITER_TO_ALT`. Later variants shift when payload-place is
/// compiled in; this slice only parks in [`AutoSubMode::Loiter`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AutoSubMode {
    /// `SubMode::TAKEOFF`.
    Takeoff = 0,
    /// `SubMode::WP`.
    Wp = 1,
    /// `SubMode::LAND`.
    Land = 2,
    /// `SubMode::RTL`.
    Rtl = 3,
    /// `SubMode::CIRCLE_MOVE_TO_EDGE`.
    CircleMoveToEdge = 4,
    /// `SubMode::CIRCLE`.
    Circle = 5,
    /// `SubMode::NAVGUIDED`.
    NavGuided = 6,
    /// `SubMode::LOITER` — what `init` selects.
    Loiter = 7,
    /// `SubMode::LOITER_TO_ALT`.
    LoiterToAlt = 8,
}

/// Vehicle view `ModeAuto::init` reads.
#[derive(Debug, Clone, Copy)]
pub struct AutoInitView {
    /// `mission.present()` — `_cmd_total > 1`.
    pub mission_present: bool,
    /// The `ignore_checks` argument. Disarmed `set_mode` passes true.
    pub ignore_checks: bool,
    /// `motors->armed()`.
    pub armed: bool,
    /// `copter.ap.land_complete`.
    pub land_complete: bool,
    /// `mission.starts_with_takeoff_cmd()`.
    pub starts_with_takeoff_cmd: bool,
    /// `auto_yaw.mode() == AutoYaw::Mode::ROI`.
    pub auto_yaw_is_roi: bool,
}

impl AutoInitView {
    /// Armed, airborne, a mission is loaded, yaw is not ROI.
    #[must_use]
    pub const fn airborne_with_mission() -> Self {
        Self {
            mission_present: true,
            ignore_checks: false,
            armed: true,
            land_complete: false,
            starts_with_takeoff_cmd: true,
            auto_yaw_is_roi: false,
        }
    }

    /// Armed on the ground with a plan that does not start with takeoff.
    #[must_use]
    pub const fn landed_armed_without_takeoff() -> Self {
        Self {
            mission_present: true,
            ignore_checks: false,
            armed: true,
            land_complete: true,
            starts_with_takeoff_cmd: false,
            auto_yaw_is_roi: false,
        }
    }

    /// No plan, checks enforced — the empty-mission refuse.
    #[must_use]
    pub const fn no_mission() -> Self {
        Self {
            mission_present: false,
            ignore_checks: false,
            armed: false,
            land_complete: true,
            starts_with_takeoff_cmd: false,
            auto_yaw_is_roi: false,
        }
    }
}

/// Leftover of one `ModeAuto::init` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoInit {
    /// What `init` returned.
    pub ok: bool,
    /// Always false. Cleared before either refuse.
    pub auto_rtl: bool,
    /// GCS `"Auto: Missing Takeoff Cmd"` was sent.
    pub missing_takeoff_cmd: bool,
    /// `_mode` after a successful enter. `None` on refuse.
    pub submode: Option<AutoSubMode>,
    /// `auto_yaw.set_mode(HOLD)` because the leftover mode was ROI.
    pub hold_yaw_from_roi: bool,
    /// `wp_nav->wp_and_spline_init_m()` ran.
    pub wp_and_spline_init: bool,
    /// `desired_speed_override_ms.xy` after init. Zero means unset.
    pub desired_speed_override_xy_ms: f32,
    /// `desired_speed_override_ms.up` after init.
    pub desired_speed_override_up_ms: f32,
    /// `desired_speed_override_ms.down` after init.
    pub desired_speed_override_down_ms: f32,
    /// `waiting_to_start`. `run` must not start the mission until origin.
    pub waiting_to_start: bool,
    /// `mis_change_detector.check_for_mission_change()` ran (result ignored).
    pub check_mission_change: bool,
    /// `mode_guided.limit_clear()`.
    pub guided_limit_clear: bool,
    /// `copter.ap.land_repo_active` after init. Always false on success.
    pub land_repo_active: bool,
}

impl AutoInit {
    /// Shared refuse leftover: `auto_RTL` is already clear; nothing else ran.
    #[must_use]
    const fn refused(missing_takeoff_cmd: bool) -> Self {
        Self {
            ok: false,
            auto_rtl: false,
            missing_takeoff_cmd,
            submode: None,
            hold_yaw_from_roi: false,
            wp_and_spline_init: false,
            desired_speed_override_xy_ms: 0.0,
            desired_speed_override_up_ms: 0.0,
            desired_speed_override_down_ms: 0.0,
            waiting_to_start: false,
            check_mission_change: false,
            guided_limit_clear: false,
            land_repo_active: false,
        }
    }
}

/// Upstream `ModeAuto::init`.
#[must_use]
pub fn auto_init(view: &AutoInitView) -> AutoInit {
    // `auto_RTL = false` runs before the present / ignore gate.
    if !(view.mission_present || view.ignore_checks) {
        return AutoInit::refused(false);
    }

    if view.armed && view.land_complete && !view.starts_with_takeoff_cmd {
        return AutoInit::refused(true);
    }

    AutoInit {
        ok: true,
        auto_rtl: false,
        missing_takeoff_cmd: false,
        submode: Some(AutoSubMode::Loiter),
        hold_yaw_from_roi: view.auto_yaw_is_roi,
        wp_and_spline_init: true,
        desired_speed_override_xy_ms: 0.0,
        desired_speed_override_up_ms: 0.0,
        desired_speed_override_down_ms: 0.0,
        waiting_to_start: true,
        check_mission_change: true,
        guided_limit_clear: true,
        land_repo_active: false,
    }
}

/// `mode_number()` leftover: AUTO_RTL is a report, not a mode.
#[must_use]
pub const fn auto_mode_number(auto_rtl: bool) -> u8 {
    if auto_rtl {
        MODE_NUMBER_AUTO_RTL
    } else {
        MODE_NUMBER_AUTO
    }
}
