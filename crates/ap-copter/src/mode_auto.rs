//! `ModeAuto` leftovers, upstream `ArduCopter/mode_auto.cpp`.
//!
//! Tracked as **COP-016**. AUTO is the mission mode. Copter names the
//! enter `init`, not `_enter`. [`auto_init`] is that enter — the two
//! refuses, the LOITER parking submode, and the leftover that must be
//! true before `run` starts the mission. [`auto_start_command`] is the
//! leftover switch that picks a `do_*` from `cmd.id`. [`auto_run`] is
//! the 100 Hz leftover: start-or-update the mission, call the current
//! submode body, and drop `auto_RTL` when the landing sequence is over.
//! [`auto_takeoff_start`] is the first command-handler body:
//! `ModeAuto::takeoff_start`, which `do_takeoff` calls.
//! [`auto_wp_start`] is the next: `ModeAuto::wp_start`, which
//! `do_nav_wp` and the fly-to-location arm of `do_land` call. The
//! other `do_*` bodies, `land_start`, and the `*_run` controllers
//! are later slices.
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
//!
//! # start_command is a switch, not the bodies
//!
//! Recognised ids always return true, even `DO_LAND_START` /
//! `DO_RETURN_PATH_START` which do nothing. An unknown id — or a gated
//! id whose `#if` is off — returns false so the mission may try the next
//! command. VTOL takeoff/land share the copter takeoff/land handlers.
//! Waypoint and arc-waypoint share `do_nav_wp`. The three ROI ids share
//! `do_roi`. The leftover does not run those functions.
//!
//! # run starts the mission once, then dispatches
//!
//! While `waiting_to_start`, origin is required before
//! `start_or_resume`. The current submode body still runs — after init
//! that is LOITER. Once running, a mission-file change restarts the
//! current nav command only when the submode is WP; any other submode
//! still gets `mission.update()`. `auto_RTL` expires the moment the
//! mission is no longer in a landing sequence, a return path, or
//! complete.
//!
//! # takeoff_start picks an altitude, then parks in TAKEOFF
//!
//! `current_loc` must already be initialised — the mission does not
//! start until the AHRS origin exists, so a missing current_loc is a
//! flow-of-control error and nothing else runs. Terrain-frame dests
//! with a terrain offset convert the vehicle altitude to
//! alt-above-terrain and pass `dest.alt` centimetres as metres.
//! Otherwise the leftover asks for alt-above-origin; a conversion
//! failure (terrain dest, no terrain data) logs
//! `MISSING_TERRAIN_DATA` and falls back to current plus
//! `dest.alt * 0.01`. The target is then floored at current altitude,
//! plus one metre when landed, so a grounded copter always climbs.
//! Success always HOLDs yaw, resets the D controller, starts
//! `auto_takeoff`, and sets `_mode = TAKEOFF`.
//!
//! # wp_start inits an idle wpnav, then sets the dest
//!
//! `do_nav_wp` (and the fly-to-location arm of `do_land`) call this.
//! An already-active wpnav is left alone. An idle one is re-inited;
//! a TAKEOFF leftover that still has a completion NED hands that
//! point in as the stopping origin so the first WP starts from the
//! takeoff target rather than wherever the vehicle is now. Speed
//! overrides apply only on that init: a non-positive xy override
//! becomes 0 ("unset"), and up/down overrides only fire when
//! `is_positive`. A failed `set_wp_destination_loc` (terrain /
//! rangefinder) returns false without touching yaw or `_mode` —
//! init side-effects, if they ran, stay. Success skips
//! `set_mode_to_default` only for ROI, or FIXED when
//! `WP_YAW_BEHAVIOR` is NONE, and parks in WP.

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
/// when payload-place is compiled out. Payload-place inserts a variant
/// after `LOITER_TO_ALT` and shifts [`AutoSubMode::NavScriptTime`] /
/// [`AutoSubMode::NavAttitudeTime`]; those bodies are later slices.
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
    /// `SubMode::NAV_SCRIPT_TIME` without payload-place.
    NavScriptTime = 9,
    /// `SubMode::NAV_ATTITUDE_TIME` without payload-place.
    NavAttitudeTime = 10,
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

/// `MAV_CMD_NAV_WAYPOINT` — navigate to a waypoint.
pub const MAV_CMD_NAV_WAYPOINT: u16 = 16;
/// `MAV_CMD_NAV_LOITER_UNLIM`.
pub const MAV_CMD_NAV_LOITER_UNLIM: u16 = 17;
/// `MAV_CMD_NAV_LOITER_TURNS`.
pub const MAV_CMD_NAV_LOITER_TURNS: u16 = 18;
/// `MAV_CMD_NAV_LOITER_TIME`.
pub const MAV_CMD_NAV_LOITER_TIME: u16 = 19;
/// `MAV_CMD_NAV_RETURN_TO_LAUNCH`.
pub const MAV_CMD_NAV_RETURN_TO_LAUNCH: u16 = 20;
/// `MAV_CMD_NAV_LAND`.
pub const MAV_CMD_NAV_LAND: u16 = 21;
/// `MAV_CMD_NAV_TAKEOFF`.
pub const MAV_CMD_NAV_TAKEOFF: u16 = 22;
/// `MAV_CMD_NAV_LOITER_TO_ALT`.
pub const MAV_CMD_NAV_LOITER_TO_ALT: u16 = 31;
/// `MAV_CMD_NAV_ARC_WAYPOINT`.
pub const MAV_CMD_NAV_ARC_WAYPOINT: u16 = 36;
/// `MAV_CMD_NAV_SPLINE_WAYPOINT`.
pub const MAV_CMD_NAV_SPLINE_WAYPOINT: u16 = 82;
/// `MAV_CMD_NAV_VTOL_TAKEOFF`.
pub const MAV_CMD_NAV_VTOL_TAKEOFF: u16 = 84;
/// `MAV_CMD_NAV_VTOL_LAND`.
pub const MAV_CMD_NAV_VTOL_LAND: u16 = 85;
/// `MAV_CMD_NAV_GUIDED_ENABLE` — compiled in only with `AC_NAV_GUIDED`.
pub const MAV_CMD_NAV_GUIDED_ENABLE: u16 = 92;
/// `MAV_CMD_NAV_DELAY`.
pub const MAV_CMD_NAV_DELAY: u16 = 93;
/// `MAV_CMD_NAV_PAYLOAD_PLACE` — compiled in only with payload-place.
pub const MAV_CMD_NAV_PAYLOAD_PLACE: u16 = 94;
/// `MAV_CMD_CONDITION_DELAY`.
pub const MAV_CMD_CONDITION_DELAY: u16 = 112;
/// `MAV_CMD_CONDITION_DISTANCE`.
pub const MAV_CMD_CONDITION_DISTANCE: u16 = 114;
/// `MAV_CMD_CONDITION_YAW`.
pub const MAV_CMD_CONDITION_YAW: u16 = 115;
/// `MAV_CMD_DO_CHANGE_SPEED`.
pub const MAV_CMD_DO_CHANGE_SPEED: u16 = 178;
/// `MAV_CMD_DO_SET_HOME`.
pub const MAV_CMD_DO_SET_HOME: u16 = 179;
/// `MAV_CMD_DO_RETURN_PATH_START` — recognised no-op in AUTO.
pub const MAV_CMD_DO_RETURN_PATH_START: u16 = 188;
/// `MAV_CMD_DO_LAND_START` — recognised no-op in AUTO.
pub const MAV_CMD_DO_LAND_START: u16 = 189;
/// `MAV_CMD_DO_SET_ROI_LOCATION`.
pub const MAV_CMD_DO_SET_ROI_LOCATION: u16 = 195;
/// `MAV_CMD_DO_SET_ROI_NONE`.
pub const MAV_CMD_DO_SET_ROI_NONE: u16 = 197;
/// `MAV_CMD_DO_SET_ROI`.
pub const MAV_CMD_DO_SET_ROI: u16 = 201;
/// `MAV_CMD_DO_MOUNT_CONTROL` — compiled in only with `HAL_MOUNT_ENABLED`.
pub const MAV_CMD_DO_MOUNT_CONTROL: u16 = 205;
/// `MAV_CMD_DO_GUIDED_LIMITS` — compiled in only with `AC_NAV_GUIDED`.
pub const MAV_CMD_DO_GUIDED_LIMITS: u16 = 222;
/// `MAV_CMD_DO_WINCH` — compiled in only with `AP_WINCH_ENABLED`.
pub const MAV_CMD_DO_WINCH: u16 = 42600;
/// `MAV_CMD_NAV_SCRIPT_TIME` — compiled in only with `AP_SCRIPTING_ENABLED`.
pub const MAV_CMD_NAV_SCRIPT_TIME: u16 = 42702;
/// `MAV_CMD_NAV_ATTITUDE_TIME`.
pub const MAV_CMD_NAV_ATTITUDE_TIME: u16 = 42703;

/// Compile-time feature gates `ModeAuto::start_command` wraps in `#if`.
///
/// A command whose case is compiled out falls through to `default` and
/// returns false — the vehicle is allowed to try the next command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoStartFeatures {
    /// `AC_NAV_GUIDED` — `NAV_GUIDED_ENABLE` and `DO_GUIDED_LIMITS`.
    pub nav_guided: bool,
    /// `AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED && AC_PAYLOAD_PLACE_ENABLED`.
    pub payload_place: bool,
    /// `AP_SCRIPTING_ENABLED` — `NAV_SCRIPT_TIME`.
    pub scripting: bool,
    /// `HAL_MOUNT_ENABLED` — `DO_MOUNT_CONTROL`.
    pub mount: bool,
    /// `AP_WINCH_ENABLED` — `DO_WINCH`.
    pub winch: bool,
}

impl AutoStartFeatures {
    /// Only the always-compiled cases. Gated ids return false.
    #[must_use]
    pub const fn none() -> Self {
        Self {
            nav_guided: false,
            payload_place: false,
            scripting: false,
            mount: false,
            winch: false,
        }
    }

    /// Typical large-flash / SITL leftover: every gated case is compiled in.
    #[must_use]
    pub const fn all() -> Self {
        Self {
            nav_guided: true,
            payload_place: true,
            scripting: true,
            mount: true,
            winch: true,
        }
    }
}

/// Which `do_*` `ModeAuto::start_command` selected.
///
/// The leftover is the *dispatch*, not the body. [`AutoStartHandler::Unknown`]
/// is the `default` branch that returns false so the mission can skip ahead.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutoStartHandler {
    /// `do_takeoff` — `NAV_TAKEOFF` / `NAV_VTOL_TAKEOFF`.
    DoTakeoff,
    /// `do_nav_wp` — `NAV_WAYPOINT` / `NAV_ARC_WAYPOINT`.
    DoNavWp,
    /// `do_land` — `NAV_LAND` / `NAV_VTOL_LAND`.
    DoLand,
    /// `do_loiter_unlimited`.
    DoLoiterUnlimited,
    /// `do_circle` — `NAV_LOITER_TURNS`.
    DoCircle,
    /// `do_loiter_time`.
    DoLoiterTime,
    /// `do_loiter_to_alt`.
    DoLoiterToAlt,
    /// `do_RTL` — `NAV_RETURN_TO_LAUNCH`.
    DoRtl,
    /// `do_spline_wp`.
    DoSplineWp,
    /// `do_nav_guided_enable`.
    DoNavGuidedEnable,
    /// `do_nav_delay`.
    DoNavDelay,
    /// `do_payload_place`.
    DoPayloadPlace,
    /// `do_nav_script_time`.
    DoNavScriptTime,
    /// `do_nav_attitude_time`.
    DoNavAttitudeTime,
    /// `do_wait_delay` — `CONDITION_DELAY`.
    DoWaitDelay,
    /// `do_within_distance` — `CONDITION_DISTANCE`.
    DoWithinDistance,
    /// `do_yaw` — `CONDITION_YAW`.
    DoYaw,
    /// `do_change_speed`.
    DoChangeSpeed,
    /// `do_set_home`.
    DoSetHome,
    /// `do_roi` — `DO_SET_ROI*` / `DO_SET_ROI_NONE`.
    DoRoi,
    /// `do_mount_control`.
    DoMountControl,
    /// `do_guided_limits`.
    DoGuidedLimits,
    /// `do_winch`.
    DoWinch,
    /// `DO_RETURN_PATH_START` / `DO_LAND_START` — recognised, no work.
    NoOp,
    /// `default` — command is unused; `start_command` returns false.
    Unknown,
}

/// Leftover of one `ModeAuto::start_command` call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoStartCommand {
    /// What `start_command` returned. False only on [`AutoStartHandler::Unknown`].
    pub accepted: bool,
    /// Which `do_*` the switch selected.
    pub handler: AutoStartHandler,
}

impl AutoStartCommand {
    #[must_use]
    const fn accepted(handler: AutoStartHandler) -> Self {
        Self {
            accepted: true,
            handler,
        }
    }

    #[must_use]
    const fn refused() -> Self {
        Self {
            accepted: false,
            handler: AutoStartHandler::Unknown,
        }
    }

    #[must_use]
    const fn gated(on: bool, handler: AutoStartHandler) -> Self {
        if on {
            Self::accepted(handler)
        } else {
            Self::refused()
        }
    }
}

/// Upstream `ModeAuto::start_command`.
///
/// The switch is the leftover. Recognised ids always return true, even
/// when the selected `do_*` is a no-op (`DO_LAND_START`). An unknown id
/// — or a gated id whose `#if` is off — returns false so the mission
/// may try the next command.
#[must_use]
pub const fn auto_start_command(cmd_id: u16, features: AutoStartFeatures) -> AutoStartCommand {
    match cmd_id {
        MAV_CMD_NAV_VTOL_TAKEOFF | MAV_CMD_NAV_TAKEOFF => {
            AutoStartCommand::accepted(AutoStartHandler::DoTakeoff)
        }
        MAV_CMD_NAV_WAYPOINT | MAV_CMD_NAV_ARC_WAYPOINT => {
            AutoStartCommand::accepted(AutoStartHandler::DoNavWp)
        }
        MAV_CMD_NAV_VTOL_LAND | MAV_CMD_NAV_LAND => {
            AutoStartCommand::accepted(AutoStartHandler::DoLand)
        }
        MAV_CMD_NAV_LOITER_UNLIM => AutoStartCommand::accepted(AutoStartHandler::DoLoiterUnlimited),
        MAV_CMD_NAV_LOITER_TURNS => AutoStartCommand::accepted(AutoStartHandler::DoCircle),
        MAV_CMD_NAV_LOITER_TIME => AutoStartCommand::accepted(AutoStartHandler::DoLoiterTime),
        MAV_CMD_NAV_LOITER_TO_ALT => AutoStartCommand::accepted(AutoStartHandler::DoLoiterToAlt),
        MAV_CMD_NAV_RETURN_TO_LAUNCH => AutoStartCommand::accepted(AutoStartHandler::DoRtl),
        MAV_CMD_NAV_SPLINE_WAYPOINT => AutoStartCommand::accepted(AutoStartHandler::DoSplineWp),
        MAV_CMD_NAV_GUIDED_ENABLE => {
            AutoStartCommand::gated(features.nav_guided, AutoStartHandler::DoNavGuidedEnable)
        }
        MAV_CMD_NAV_DELAY => AutoStartCommand::accepted(AutoStartHandler::DoNavDelay),
        MAV_CMD_NAV_PAYLOAD_PLACE => {
            AutoStartCommand::gated(features.payload_place, AutoStartHandler::DoPayloadPlace)
        }
        MAV_CMD_NAV_SCRIPT_TIME => {
            AutoStartCommand::gated(features.scripting, AutoStartHandler::DoNavScriptTime)
        }
        MAV_CMD_NAV_ATTITUDE_TIME => {
            AutoStartCommand::accepted(AutoStartHandler::DoNavAttitudeTime)
        }
        MAV_CMD_CONDITION_DELAY => AutoStartCommand::accepted(AutoStartHandler::DoWaitDelay),
        MAV_CMD_CONDITION_DISTANCE => {
            AutoStartCommand::accepted(AutoStartHandler::DoWithinDistance)
        }
        MAV_CMD_CONDITION_YAW => AutoStartCommand::accepted(AutoStartHandler::DoYaw),
        MAV_CMD_DO_CHANGE_SPEED => AutoStartCommand::accepted(AutoStartHandler::DoChangeSpeed),
        MAV_CMD_DO_SET_HOME => AutoStartCommand::accepted(AutoStartHandler::DoSetHome),
        MAV_CMD_DO_SET_ROI_LOCATION | MAV_CMD_DO_SET_ROI_NONE | MAV_CMD_DO_SET_ROI => {
            AutoStartCommand::accepted(AutoStartHandler::DoRoi)
        }
        MAV_CMD_DO_MOUNT_CONTROL => {
            AutoStartCommand::gated(features.mount, AutoStartHandler::DoMountControl)
        }
        MAV_CMD_DO_GUIDED_LIMITS => {
            AutoStartCommand::gated(features.nav_guided, AutoStartHandler::DoGuidedLimits)
        }
        MAV_CMD_DO_WINCH => AutoStartCommand::gated(features.winch, AutoStartHandler::DoWinch),
        MAV_CMD_DO_RETURN_PATH_START | MAV_CMD_DO_LAND_START => {
            AutoStartCommand::accepted(AutoStartHandler::NoOp)
        }
        _ => AutoStartCommand::refused(),
    }
}

/// GCS text `ModeAuto::run` may send when the mission changes mid-WP.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutoMissionChangeText {
    /// No text. Mission did not change, or the current command is not WP.
    None,
    /// `"Auto mission changed, restarted command"`.
    Restarted,
    /// `"Auto mission changed but failed to restart command"`.
    RestartFailed,
}

/// Which `*_run` body `ModeAuto::run` called this tick.
///
/// The leftover is the *choice*. The bodies themselves (`takeoff_run`,
/// `wp_run`, …) are later slices.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutoRunBody {
    /// `takeoff_run`.
    Takeoff,
    /// `wp_run` — `WP` and `CIRCLE_MOVE_TO_EDGE`.
    Wp,
    /// `land_run`.
    Land,
    /// `rtl_run`.
    Rtl,
    /// `circle_run`.
    Circle,
    /// `nav_guided_run` — `NAVGUIDED` and `NAV_SCRIPT_TIME` when compiled in.
    NavGuided,
    /// `loiter_run`.
    Loiter,
    /// `loiter_to_alt_run`.
    LoiterToAlt,
    /// `nav_attitude_time_run`.
    NavAttitudeTime,
    /// Gated body whose `#if` is off: the case matches and does nothing.
    None,
}

/// Vehicle view `ModeAuto::run` reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoRunView {
    /// `waiting_to_start` at the top of the tick.
    pub waiting_to_start: bool,
    /// `ahrs.get_origin` succeeded.
    pub has_origin: bool,
    /// `mis_change_detector.check_for_mission_change()`.
    pub mission_changed: bool,
    /// `mission.state() == MISSION_RUNNING`.
    pub mission_running: bool,
    /// `_mode` at the top of the tick.
    pub submode: AutoSubMode,
    /// `mission.restart_current_nav_cmd()` would succeed.
    pub restart_current_nav_cmd: bool,
    /// `auto_RTL` at the top of the tick.
    pub auto_rtl: bool,
    /// `mission.get_in_landing_sequence_flag()`.
    pub in_landing_sequence: bool,
    /// `mission.get_in_return_path_flag()`.
    pub in_return_path: bool,
    /// `mission.state() == MISSION_COMPLETE`.
    pub mission_complete: bool,
    /// `AC_NAV_GUIDED || AP_SCRIPTING_ENABLED`.
    pub nav_guided_or_scripting: bool,
}

impl AutoRunView {
    /// After [`auto_init`]: parked in LOITER, waiting, origin not ready.
    #[must_use]
    pub const fn waiting_no_origin() -> Self {
        Self {
            waiting_to_start: true,
            has_origin: false,
            mission_changed: false,
            mission_running: false,
            submode: AutoSubMode::Loiter,
            restart_current_nav_cmd: false,
            auto_rtl: false,
            in_landing_sequence: false,
            in_return_path: false,
            mission_complete: false,
            nav_guided_or_scripting: true,
        }
    }

    /// After [`auto_init`]: parked in LOITER, origin just arrived.
    #[must_use]
    pub const fn waiting_with_origin() -> Self {
        let mut view = Self::waiting_no_origin();
        view.has_origin = true;
        view
    }

    /// Mission already running a waypoint, no change this tick.
    #[must_use]
    pub const fn running_wp() -> Self {
        Self {
            waiting_to_start: false,
            has_origin: true,
            mission_changed: false,
            mission_running: true,
            submode: AutoSubMode::Wp,
            restart_current_nav_cmd: true,
            auto_rtl: false,
            in_landing_sequence: false,
            in_return_path: false,
            mission_complete: false,
            nav_guided_or_scripting: true,
        }
    }
}

/// Leftover of one `ModeAuto::run` tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoRun {
    /// `mission.start_or_resume()` ran (waiting + origin).
    pub start_or_resume: bool,
    /// `waiting_to_start` after the tick.
    pub waiting_to_start: bool,
    /// `mis_change_detector.check_for_mission_change()` ran.
    pub check_mission_change: bool,
    /// `mission.restart_current_nav_cmd()` ran.
    pub restart_current_nav_cmd: bool,
    /// GCS text from a mid-WP mission change.
    pub mission_change_text: AutoMissionChangeText,
    /// `mission.update()` ran. Only when not waiting.
    pub mission_update: bool,
    /// Which `*_run` body the switch selected.
    pub body: AutoRunBody,
    /// `auto_RTL` after the tick.
    pub auto_rtl: bool,
    /// `Write_Mode(..., AUTO_RTL_EXIT)` because auto-RTL expired.
    pub log_auto_rtl_exit: bool,
}

/// Upstream `ModeAuto::run`.
///
/// Waiting for origin blocks `mission.update` and `start_or_resume`. The
/// current submode body still runs — after [`auto_init`] that is LOITER.
/// `auto_RTL` is a report: it drops the moment the mission is no longer
/// in a landing sequence, a return path, or complete.
#[must_use]
pub const fn auto_run(view: &AutoRunView) -> AutoRun {
    let mut start_or_resume = false;
    let mut waiting_to_start = view.waiting_to_start;
    let mut check_mission_change = false;
    let mut restart_current_nav_cmd = false;
    let mut mission_change_text = AutoMissionChangeText::None;
    let mut mission_update = false;

    if view.waiting_to_start {
        if view.has_origin {
            start_or_resume = true;
            waiting_to_start = false;
            check_mission_change = true;
        }
    } else {
        check_mission_change = true;
        if view.mission_changed && view.mission_running && matches!(view.submode, AutoSubMode::Wp) {
            restart_current_nav_cmd = true;
            mission_change_text = if view.restart_current_nav_cmd {
                AutoMissionChangeText::Restarted
            } else {
                AutoMissionChangeText::RestartFailed
            };
        }
        mission_update = true;
    }

    let body = match view.submode {
        AutoSubMode::Takeoff => AutoRunBody::Takeoff,
        AutoSubMode::Wp | AutoSubMode::CircleMoveToEdge => AutoRunBody::Wp,
        AutoSubMode::Land => AutoRunBody::Land,
        AutoSubMode::Rtl => AutoRunBody::Rtl,
        AutoSubMode::Circle => AutoRunBody::Circle,
        AutoSubMode::NavGuided | AutoSubMode::NavScriptTime => {
            if view.nav_guided_or_scripting {
                AutoRunBody::NavGuided
            } else {
                AutoRunBody::None
            }
        }
        AutoSubMode::Loiter => AutoRunBody::Loiter,
        AutoSubMode::LoiterToAlt => AutoRunBody::LoiterToAlt,
        AutoSubMode::NavAttitudeTime => AutoRunBody::NavAttitudeTime,
    };

    let auto_rtl_active = view.in_landing_sequence || view.in_return_path || view.mission_complete;
    let log_auto_rtl_exit = view.auto_rtl && !auto_rtl_active;
    let auto_rtl = view.auto_rtl && auto_rtl_active;

    AutoRun {
        start_or_resume,
        waiting_to_start,
        check_mission_change,
        restart_current_nav_cmd,
        mission_change_text,
        mission_update,
        body,
        auto_rtl,
        log_auto_rtl_exit,
    }
}

/// Vehicle view [`auto_takeoff_start`] reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoTakeoffStartView {
    /// `copter.current_loc.initialised()`.
    pub current_loc_initialised: bool,
    /// `dest_loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN`.
    pub dest_alt_frame_terrain: bool,
    /// `wp_nav->get_terrain_U_m` — `None` when the lookup failed.
    pub terrain_u_m: Option<f32>,
    /// `pos_control->get_pos_estimate_U_m()`. Alt above EKF origin.
    pub current_alt_m: f32,
    /// `dest_loc.alt`, centimetres in the dest's own frame.
    pub dest_alt_cm: i32,
    /// `dest.get_alt_m(ABOVE_ORIGIN)` after lat/lng are copied from
    /// `current_loc`. Unused on the terrain-success path.
    pub origin_alt_m: Option<f32>,
    /// `copter.ap.land_complete`.
    pub land_complete: bool,
}

impl AutoTakeoffStartView {
    /// Airborne, origin-frame dest whose conversion succeeded.
    #[must_use]
    pub const fn origin_airborne() -> Self {
        Self {
            current_loc_initialised: true,
            dest_alt_frame_terrain: false,
            terrain_u_m: None,
            current_alt_m: 2.0,
            dest_alt_cm: 1_000,
            origin_alt_m: Some(10.0),
            land_complete: false,
        }
    }

    /// Origin not yet set — the flow-of-control refuse.
    #[must_use]
    pub const fn uninitialised() -> Self {
        let mut view = Self::origin_airborne();
        view.current_loc_initialised = false;
        view
    }

    /// On the ground. The leftover floors the target at current + 1 m.
    #[must_use]
    pub const fn landed() -> Self {
        let mut view = Self::origin_airborne();
        view.land_complete = true;
        view.current_alt_m = 0.0;
        view
    }
}

/// Leftover of one `ModeAuto::takeoff_start` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoTakeoffStart {
    /// The leftover ran past the current_loc gate.
    pub ok: bool,
    /// `INTERNAL_ERROR(flow_of_control)` — current_loc was not initialised.
    pub flow_of_control_error: bool,
    /// `LOGGER_WRITE_ERROR(TERRAIN, MISSING_TERRAIN_DATA)`.
    pub missing_terrain_data: bool,
    /// Altitude handed to `auto_takeoff.start_m`. Zero on refuse.
    pub alt_target_m: f32,
    /// Second argument to `auto_takeoff.start_m`.
    pub alt_target_terrain: bool,
    /// `auto_yaw.set_mode(HOLD)` ran.
    pub yaw_hold: bool,
    /// `pos_control->D_init_controller()` ran.
    pub d_init_controller: bool,
    /// `auto_takeoff.start_m` ran.
    pub auto_takeoff_start: bool,
    /// `_mode` after a successful start. `None` on refuse.
    pub submode: Option<AutoSubMode>,
}

impl AutoTakeoffStart {
    /// Shared refuse leftover: current_loc was missing; nothing else ran.
    #[must_use]
    const fn refused() -> Self {
        Self {
            ok: false,
            flow_of_control_error: true,
            missing_terrain_data: false,
            alt_target_m: 0.0,
            alt_target_terrain: false,
            yaw_hold: false,
            d_init_controller: false,
            auto_takeoff_start: false,
            submode: None,
        }
    }
}

/// Centimetres in a `Location::alt` to metres. Upstream `dest.alt * 0.01`.
#[must_use]
const fn loc_alt_cm_to_m(alt_cm: i32) -> f32 {
    alt_cm as f32 * 0.01
}

/// Upstream `ModeAuto::takeoff_start`.
///
/// `do_takeoff` is a one-line caller of this leftover. A missing
/// `current_loc` is a flow-of-control error: the mission does not start
/// until the AHRS origin exists. Terrain dests with a terrain offset
/// convert the vehicle to alt-above-terrain; every other dest asks for
/// alt-above-origin and falls back to current plus `dest.alt * 0.01`
/// when that conversion fails. The target is floored at current
/// altitude, plus one metre when landed.
#[must_use]
pub const fn auto_takeoff_start(view: &AutoTakeoffStartView) -> AutoTakeoffStart {
    if !view.current_loc_initialised {
        return AutoTakeoffStart::refused();
    }

    let mut current_alt_m = view.current_alt_m;
    let mut alt_target_m;
    let mut alt_target_terrain = false;
    let mut missing_terrain_data = false;

    // `ABOVE_TERRAIN && get_terrain_U_m` — both must succeed.
    let terrain_offset = if view.dest_alt_frame_terrain {
        view.terrain_u_m
    } else {
        None
    };

    if let Some(terrain_u_m) = terrain_offset {
        current_alt_m -= terrain_u_m;
        alt_target_m = loc_alt_cm_to_m(view.dest_alt_cm);
        alt_target_terrain = true;
    } else if let Some(origin) = view.origin_alt_m {
        alt_target_m = origin;
    } else {
        missing_terrain_data = true;
        alt_target_m = current_alt_m + loc_alt_cm_to_m(view.dest_alt_cm);
    }

    let alt_target_min_m = current_alt_m + if view.land_complete { 1.0 } else { 0.0 };
    if alt_target_m < alt_target_min_m {
        alt_target_m = alt_target_min_m;
    }

    AutoTakeoffStart {
        ok: true,
        flow_of_control_error: false,
        missing_terrain_data,
        alt_target_m,
        alt_target_terrain,
        yaw_hold: true,
        d_init_controller: true,
        auto_takeoff_start: true,
        submode: Some(AutoSubMode::Takeoff),
    }
}

/// Vehicle view [`auto_wp_start`] reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoWpStartView {
    /// `wp_nav->is_active()`.
    pub wp_nav_active: bool,
    /// `_mode` before the call.
    pub submode: AutoSubMode,
    /// `auto_takeoff.get_completion_pos_ned_m` succeeded.
    pub takeoff_completion_pos: bool,
    /// `desired_speed_override_ms.xy`. Zero means unset.
    pub desired_speed_override_xy_ms: f32,
    /// `desired_speed_override_ms.up`.
    pub desired_speed_override_up_ms: f32,
    /// `desired_speed_override_ms.down`.
    pub desired_speed_override_down_ms: f32,
    /// `wp_nav->set_wp_destination_loc(dest_loc)`.
    pub dest_accepted: bool,
    /// `auto_yaw.mode() == AutoYaw::Mode::ROI`.
    pub auto_yaw_is_roi: bool,
    /// `auto_yaw.mode() == AutoYaw::Mode::FIXED`.
    pub auto_yaw_is_fixed: bool,
    /// `copter.g.wp_yaw_behavior == WP_YAW_BEHAVIOR_NONE`.
    pub wp_yaw_behavior_none: bool,
}

impl AutoWpStartView {
    /// Idle wpnav after the LOITER park, dest accepted, yaw HOLD.
    #[must_use]
    pub const fn idle_loiter() -> Self {
        Self {
            wp_nav_active: false,
            submode: AutoSubMode::Loiter,
            takeoff_completion_pos: false,
            desired_speed_override_xy_ms: 0.0,
            desired_speed_override_up_ms: 0.0,
            desired_speed_override_down_ms: 0.0,
            dest_accepted: true,
            auto_yaw_is_roi: false,
            auto_yaw_is_fixed: false,
            wp_yaw_behavior_none: false,
        }
    }

    /// Leaving TAKEOFF with a completion NED.
    #[must_use]
    pub const fn from_takeoff() -> Self {
        let mut view = Self::idle_loiter();
        view.submode = AutoSubMode::Takeoff;
        view.takeoff_completion_pos = true;
        view
    }

    /// `set_wp_destination_loc` refused (terrain / rangefinder).
    #[must_use]
    pub const fn dest_refused() -> Self {
        let mut view = Self::idle_loiter();
        view.dest_accepted = false;
        view
    }
}

/// Leftover of one `ModeAuto::wp_start` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoWpStart {
    /// What `wp_start` returned.
    pub ok: bool,
    /// `wp_nav->wp_and_spline_init_m` ran.
    pub wp_and_spline_init: bool,
    /// First argument to `wp_and_spline_init_m`. Zero when init did not run.
    pub init_speed_xy_ms: f32,
    /// The TAKEOFF completion NED was the stopping point.
    pub stopping_point_from_takeoff: bool,
    /// `wp_nav->set_speed_up_ms` ran.
    pub set_speed_up: bool,
    /// `wp_nav->set_speed_down_ms` ran.
    pub set_speed_down: bool,
    /// `set_wp_destination_loc` succeeded.
    pub set_wp_destination: bool,
    /// `auto_yaw.set_mode_to_default(false)` ran.
    pub yaw_set_default: bool,
    /// `_mode` after a successful start. `None` on refuse.
    pub submode: Option<AutoSubMode>,
}

/// Upstream `is_positive` as the leftover uses it: a speed override of 0
/// means "unset".
#[must_use]
const fn speed_override_set(ms: f32) -> bool {
    ms > 0.0
}

/// Upstream `ModeAuto::wp_start`.
///
/// `do_nav_wp` is the usual caller. An idle wpnav is re-inited, and a
/// TAKEOFF leftover can hand its completion NED as the stopping origin.
/// Speed overrides apply only on that init. A dest refuse returns false
/// without touching yaw or `_mode`. Success parks in WP unless yaw is
/// already ROI, or FIXED with `WP_YAW_BEHAVIOR_NONE`.
#[must_use]
pub const fn auto_wp_start(view: &AutoWpStartView) -> AutoWpStart {
    let mut wp_and_spline_init = false;
    let mut init_speed_xy_ms = 0.0;
    let mut stopping_point_from_takeoff = false;
    let mut set_speed_up = false;
    let mut set_speed_down = false;

    if !view.wp_nav_active {
        stopping_point_from_takeoff =
            matches!(view.submode, AutoSubMode::Takeoff) && view.takeoff_completion_pos;
        init_speed_xy_ms = if speed_override_set(view.desired_speed_override_xy_ms) {
            view.desired_speed_override_xy_ms
        } else {
            0.0
        };
        wp_and_spline_init = true;
        set_speed_up = speed_override_set(view.desired_speed_override_up_ms);
        set_speed_down = speed_override_set(view.desired_speed_override_down_ms);
    }

    if !view.dest_accepted {
        return AutoWpStart {
            ok: false,
            wp_and_spline_init,
            init_speed_xy_ms,
            stopping_point_from_takeoff,
            set_speed_up,
            set_speed_down,
            set_wp_destination: false,
            yaw_set_default: false,
            submode: None,
        };
    }

    let skip_yaw = view.auto_yaw_is_roi || (view.auto_yaw_is_fixed && view.wp_yaw_behavior_none);

    AutoWpStart {
        ok: true,
        wp_and_spline_init,
        init_speed_xy_ms,
        stopping_point_from_takeoff,
        set_speed_up,
        set_speed_down,
        set_wp_destination: true,
        yaw_set_default: !skip_yaw,
        submode: Some(AutoSubMode::Wp),
    }
}
