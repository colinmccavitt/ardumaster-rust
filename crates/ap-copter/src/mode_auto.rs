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
//! `do_nav_wp` and the fly-to-location arm of `do_land` call.
//! [`auto_land_start`] is the descending arm: `ModeAuto::land_start`,
//! which `do_land` and `verify_land` call. [`auto_rtl_start`] is
//! `ModeAuto::rtl_start`, which `do_RTL` calls: it reuses
//! [`crate::mode_rtl::rtl_init`] with checks ignored and parks in RTL.
//! [`auto_spline_start`] is `ModeAuto::do_spline_wp` — C++ has no
//! separate `spline_start`; this leftover is the spline destination,
//! next-segment lookup, yaw, and WP park that `NAV_SPLINE_WAYPOINT`
//! runs. [`auto_loiter_unlimited`] is `ModeAuto::do_loiter_unlimited`:
//! the same default-loc + dest fetch as spline, then [`auto_wp_start`].
//! [`auto_loiter_time`] reuses that leftover and latches the loiter
//! timer. [`auto_loiter_to_alt`] reuses it, then reads the target
//! alt-above-home and parks in LOITER_TO_ALT (or marks both reached
//! on a bad alt). [`auto_circle`] is `ModeAuto::do_circle`: dest,
//! HIGHBYTE radius (x10 when the large-radius bit is set), then
//! `circle_movetoedge_start` or `circle_start`. [`auto_do_yaw`] is
//! `do_yaw` -- degrees to radians and `relative_angle > 0`.
//! [`auto_do_roi`] forwards to [`crate::auto_yaw::roi_action`].
//! [`auto_nav_delay`], [`auto_wait_delay`], [`auto_within_distance`],
//! [`auto_change_speed`], [`auto_set_home`], and [`auto_payload_place`]
//! are the remaining `do_*` leftovers. [`auto_verify_command`] is the
//! `verify_command` switch; the `verify_*` leftovers are the bodies.
//! The `*_run` controllers are later slices.
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
//!
//! # rtl_start reuses ModeRTL::init, then parks
//!
//! `do_RTL` is a one-line caller of this leftover. The body is
//! [`crate::mode_rtl::rtl_init`] with `ignore_checks = true`, so a
//! missing home is not a refuse. Success sets `_mode = RTL`. Failure
//! is a flow-of-control error — RTL never refuses when that argument
//! is true.
//!
//! # spline_start is do_spline_wp — dest, next, yaw, WP
//!
//! There is no `ModeAuto::spline_start`. `do_spline_wp` is the body:
//! default loc from current (or the last wp dest if wpnav is active
//! and already there), `get_spline_from_cmd` for dest and next,
//! `set_spline_destination_loc`, then the loiter delay, `set_next_wp`,
//! the same yaw skip as `wp_start`, and `_mode = WP`. A missing
//! dest loc while parked on the last WP is a flow-of-control error
//! and the leftover continues with current. Every other refuse is
//! `failsafe_terrain_on_event` and returns before yaw or `_mode`.
//!
//! # loiter_unlimited is dest then wp_start
//!
//! `do_loiter_unlimited` has no submode of its own — it flies to the
//! loiter point via [`auto_wp_start`]. Default loc is current (minus
//! offsets). If wpnav is already on its dest, that dest becomes the
//! default; a failed fetch is a flow-of-control error and the leftover
//! keeps current. `get_loc_from_cmd` then `wp_start`. Either refuse
//! is `failsafe_terrain_on_event` and returns false. Success returns
//! true with `_mode = WP`.
//!
//! # loiter_time latches the delay after that
//!
//! `do_loiter_time` is a wrapper: if unlimited refuses, the leftover
//! returns without touching the timer. Otherwise `loiter_time` is
//! zeroed and `loiter_time_max = cmd.p1` (seconds).
//!
//! # loiter_to_alt then parks in LOITER_TO_ALT
//!
//! After unlimited succeeds, a zero lat/lng copies current. A failed
//! `get_alt_m(ABOVE_HOME)` marks both reached, sends
//! `"bad do_loiter_to_alt"`, and leaves `_mode` at WP. Success
//! clears the loiter-to-alt flags, copies wpnav D limits onto the
//! position controller, and sets `_mode = LOITER_TO_ALT`.
//!
//! # do_circle is dest, radius, then edge or start
//!
//! Radius is `HIGHBYTE(p1)`. `NAV_LOITER_TURNS` with type-specific
//! bit 0 multiplies that by ten. A dest refuse is terrain failsafe
//! and returns before the last-complete reset. More than 3 m from
//! the edge flies there (`CIRCLE_MOVE_TO_EDGE`); a dest refuse on
//! that fly-to still sets yaw and the submode. Already on the edge
//! calls `circle_start` (`CIRCLE` yaw unless ROI). Then
//! `circle_last_num_complete = -1`.
//!
//! # do_yaw converts, do_roi forwards
//!
//! `CONDITION_YAW` is degrees to radians and `relative_angle > 0`.
//! The three ROI ids call `auto_yaw.set_roi`; the leftover is
//! [`roi_action`].
//!
//! # verify_command is a switch, then the bodies
//!
//! Not in AUTO is an immediate false. Recognised ids pick a
//! `verify_*`. DO commands and an unknown id complete immediately
//! (unknown also sends "Skipping invalid cmd"). `NAV_LOITER_TO_ALT`
//! returns the body leftover directly and skips the reached message.

use crate::auto_yaw::{
    reached_fixed_yaw_target, roi_action, FixedYawDirection, RoiAction, YawMode,
};
use crate::mode_rtl::{rtl_init, RtlInit, RtlInitView, RtlSubMode};

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
    /// `SubMode::NAV_PAYLOAD_PLACE` when that feature is compiled in.
    /// C++ inserts it before script/attitude and shifts those numbers;
    /// this leftover keeps the compiled-out numbering and names the
    /// variant at 11.
    NavPayloadPlace = 11,
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
    /// `payload_place.run` -- `NAV_PAYLOAD_PLACE` when compiled in.
    PayloadPlace,
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
        AutoSubMode::NavPayloadPlace => AutoRunBody::PayloadPlace,
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

/// Vehicle view [`auto_land_start`] reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoLandStartView {
    /// `pos_control->NE_is_active()`.
    pub ne_is_active: bool,
    /// `pos_control->D_is_active()`.
    pub d_is_active: bool,
    /// `wp_nav->get_default_speed_NE_ms()`.
    pub speed_ne_ms: f32,
    /// `wp_nav->get_wp_acceleration_mss()`.
    pub wp_accel_mss: f32,
    /// `wp_nav->get_default_speed_down_ms()`.
    pub speed_down_ms: f32,
    /// `wp_nav->get_default_speed_up_ms()`.
    pub speed_up_ms: f32,
    /// `wp_nav->get_accel_D_mss()`.
    pub accel_d_mss: f32,
    /// `AP_LANDINGGEAR_ENABLED` — the `#if` around `deploy_for_landing`.
    pub landing_gear: bool,
}

impl AutoLandStartView {
    /// Both controllers already running, landing gear compiled in.
    #[must_use]
    pub const fn ready() -> Self {
        Self {
            ne_is_active: true,
            d_is_active: true,
            speed_ne_ms: 5.0,
            wp_accel_mss: 1.0,
            speed_down_ms: 1.5,
            speed_up_ms: 2.5,
            accel_d_mss: 2.5,
            landing_gear: true,
        }
    }
}

/// Leftover of one `ModeAuto::land_start` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoLandStart {
    /// Horizontal max / correction speed handed to the NE controller.
    pub ne_speed_ms: f32,
    /// Horizontal max / correction accel, from `wp_nav`.
    pub ne_accel_mss: f32,
    /// `NE_init_controller` — only when NE was inactive.
    pub init_ne: bool,
    /// Vertical max / correction speed down.
    pub d_speed_down_ms: f32,
    /// Vertical max / correction speed up.
    pub d_speed_up_ms: f32,
    /// Vertical max / correction accel, from `wp_nav`.
    pub d_accel_mss: f32,
    /// `D_init_controller` — only when D was inactive.
    pub init_d: bool,
    /// `auto_yaw.set_mode(HOLD)` ran.
    pub yaw_hold: bool,
    /// `landinggear.deploy_for_landing` ran.
    pub deploy_landing_gear: bool,
    /// `copter.ap.land_repo_active` after the call. Always false.
    pub land_repo_active: bool,
    /// `copter.ap.prec_land_active` after the call. Always false.
    pub prec_land_active: bool,
    /// `_mode` after the call. Always [`AutoSubMode::Land`].
    pub submode: AutoSubMode,
}

/// Upstream `ModeAuto::land_start`.
///
/// `do_land` calls this leftover on the descending arm (zero lat/lng).
/// `verify_land` calls it again when FlyToLocation arrives. Unlike
/// [`crate::mode_land::land_init`], NE is inited whenever it is idle —
/// there is no `position_ok` gate — and there is no pause clock. Yaw
/// is HOLD, repo/prec-land flags are cleared, and `_mode` becomes LAND.
/// Landing-gear deploy is compiled out when `AP_LANDINGGEAR_ENABLED`
/// is off.
#[must_use]
pub const fn auto_land_start(view: &AutoLandStartView) -> AutoLandStart {
    AutoLandStart {
        ne_speed_ms: view.speed_ne_ms,
        ne_accel_mss: view.wp_accel_mss,
        init_ne: !view.ne_is_active,
        d_speed_down_ms: view.speed_down_ms,
        d_speed_up_ms: view.speed_up_ms,
        d_accel_mss: view.accel_d_mss,
        init_d: !view.d_is_active,
        yaw_hold: true,
        deploy_landing_gear: view.landing_gear,
        land_repo_active: false,
        prec_land_active: false,
        submode: AutoSubMode::Land,
    }
}

/// Vehicle view [`auto_rtl_start`] reads.
///
/// `rtl_start` always passes `ignore_checks = true` to `ModeRTL::init`.
/// A missing home is therefore not a refuse. [`rtl_init`] is the body;
/// this leftover is the AUTO parking around it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoRtlStartView {
    /// `AP::ahrs().home_is_set()`. Ignored because checks are ignored.
    pub home_is_set: bool,
    /// `copter.failsafe.terrain`.
    pub terrain_failsafe: bool,
    /// `speed_ms.get()`, handed to `wp_and_spline_init_m`. Zero means
    /// "use WP_SPD".
    pub speed_ms: f32,
}

impl AutoRtlStartView {
    /// Home set, no terrain failsafe, WP_SPD.
    #[must_use]
    pub const fn ready() -> Self {
        Self {
            home_is_set: true,
            terrain_failsafe: false,
            speed_ms: 0.0,
        }
    }

    /// No home. Still succeeds because `rtl_start` ignores checks.
    #[must_use]
    pub const fn no_home() -> Self {
        let mut view = Self::ready();
        view.home_is_set = false;
        view
    }
}

/// Leftover of one `ModeAuto::rtl_start` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoRtlStart {
    /// `mode_rtl.init(true)` succeeded.
    pub ok: bool,
    /// `INTERNAL_ERROR(flow_of_control)` — init failed despite
    /// `ignore_checks`. The C++ comment says this never happens.
    pub flow_of_control_error: bool,
    /// `_mode` after a successful start. `None` on refuse.
    pub submode: Option<AutoSubMode>,
    /// What [`rtl_init`] produced. Always called.
    pub rtl: RtlInit,
}

impl AutoRtlStart {
    /// Shared refuse leftover: RTL init failed; `_mode` is unchanged.
    #[must_use]
    const fn refused(rtl: RtlInit) -> Self {
        Self {
            ok: false,
            flow_of_control_error: true,
            submode: None,
            rtl,
        }
    }
}

/// Park in RTL after `mode_rtl.init(true)`, or raise the internal error.
///
/// Split out so the flow-of-control arm can be tested without rewriting
/// [`rtl_init`], which never fails when checks are ignored.
#[must_use]
pub const fn auto_rtl_from_init(rtl: RtlInit) -> AutoRtlStart {
    if rtl.ok {
        AutoRtlStart {
            ok: true,
            flow_of_control_error: false,
            submode: Some(AutoSubMode::Rtl),
            rtl,
        }
    } else {
        AutoRtlStart::refused(rtl)
    }
}

/// Upstream `ModeAuto::rtl_start`.
///
/// `do_RTL` is a one-line caller of this leftover. The body is
/// [`rtl_init`] with `ignore_checks = true`. Success parks in RTL.
/// Failure is a flow-of-control error — RTL never refuses when
/// checks are ignored.
#[must_use]
pub fn auto_rtl_start(view: &AutoRtlStartView) -> AutoRtlStart {
    auto_rtl_from_init(rtl_init(
        &RtlInitView {
            home_is_set: view.home_is_set,
            terrain_failsafe: view.terrain_failsafe,
            speed_ms: view.speed_ms,
        },
        true,
    ))
}

/// Vehicle view [`auto_spline_from_cmd`] reads.
///
/// `get_spline_from_cmd` fills dest from the current command (using
/// the caller's default loc when lat/lon/alt are zero) and then
/// decides whether the outgoing control point is the next nav command
/// or dest itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoSplineFromCmdView {
    /// `get_loc_from_cmd(cmd, default_loc, dest_loc)`.
    pub dest_loc_ok: bool,
    /// `cmd.p1` — delay at the end of this segment, seconds.
    pub delay_s: u16,
    /// `mission.get_next_nav_cmd(cmd.index+1, temp_cmd)`.
    pub next_nav_cmd: bool,
    /// `get_loc_from_cmd(temp_cmd, dest_loc, next_dest_loc)`.
    pub next_loc_ok: bool,
    /// `temp_cmd.id == MAV_CMD_NAV_SPLINE_WAYPOINT`.
    pub next_is_spline_waypoint: bool,
}

impl AutoSplineFromCmdView {
    /// Dest ok, no delay, next nav is a spline waypoint.
    #[must_use]
    pub const fn through_to_spline() -> Self {
        Self {
            dest_loc_ok: true,
            delay_s: 0,
            next_nav_cmd: true,
            next_loc_ok: true,
            next_is_spline_waypoint: true,
        }
    }

    /// Dest ok, no delay, next nav is a straight waypoint.
    #[must_use]
    pub const fn through_to_wp() -> Self {
        let mut view = Self::through_to_spline();
        view.next_is_spline_waypoint = false;
        view
    }

    /// Dest ok, no delay, this is the last nav command.
    #[must_use]
    pub const fn last_segment() -> Self {
        Self {
            dest_loc_ok: true,
            delay_s: 0,
            next_nav_cmd: false,
            next_loc_ok: false,
            next_is_spline_waypoint: false,
        }
    }

    /// Dest ok, a loiter delay — next dest is dest itself.
    #[must_use]
    pub const fn with_delay(delay_s: u16) -> Self {
        Self {
            dest_loc_ok: true,
            delay_s,
            next_nav_cmd: true,
            next_loc_ok: true,
            next_is_spline_waypoint: true,
        }
    }

    /// `get_loc_from_cmd` refused the current dest (terrain).
    #[must_use]
    pub const fn dest_refused() -> Self {
        Self {
            dest_loc_ok: false,
            delay_s: 0,
            next_nav_cmd: true,
            next_loc_ok: true,
            next_is_spline_waypoint: true,
        }
    }
}

/// Leftover of one `ModeAuto::get_spline_from_cmd` call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoSplineFromCmd {
    /// What `get_spline_from_cmd` returned.
    pub ok: bool,
    /// `next_dest_loc` was set to `dest_loc` (delay or no next cmd).
    pub next_dest_is_dest: bool,
    /// `next_dest_loc_is_spline` handed to `set_spline_destination_loc`.
    pub next_dest_is_spline: bool,
}

/// Upstream `ModeAuto::get_spline_from_cmd`.
///
/// Dest comes from the command. With no delay and a following nav
/// command, the outgoing control point is that next loc and
/// `next_dest_is_spline` is true only when the next id is
/// `NAV_SPLINE_WAYPOINT`. A delay, or the last nav command, copies
/// dest onto next and clears the spline flag so the curve stops.
#[must_use]
pub const fn auto_spline_from_cmd(view: &AutoSplineFromCmdView) -> AutoSplineFromCmd {
    if !view.dest_loc_ok {
        return AutoSplineFromCmd {
            ok: false,
            next_dest_is_dest: false,
            next_dest_is_spline: false,
        };
    }
    if view.delay_s == 0 && view.next_nav_cmd {
        if !view.next_loc_ok {
            return AutoSplineFromCmd {
                ok: false,
                next_dest_is_dest: false,
                next_dest_is_spline: false,
            };
        }
        return AutoSplineFromCmd {
            ok: true,
            next_dest_is_dest: false,
            next_dest_is_spline: view.next_is_spline_waypoint,
        };
    }
    AutoSplineFromCmd {
        ok: true,
        next_dest_is_dest: true,
        next_dest_is_spline: false,
    }
}

/// Vehicle view [`auto_spline_start`] reads.
///
/// C++ has no `spline_start`. This is `ModeAuto::do_spline_wp`, the
/// leftover `NAV_SPLINE_WAYPOINT` runs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoSplineStartView {
    /// `wp_nav->is_active()`.
    pub wp_nav_active: bool,
    /// `wp_nav->reached_wp_destination()`.
    pub reached_wp_destination: bool,
    /// `wp_nav->get_wp_destination_loc(default_loc)`.
    pub wp_dest_loc_ok: bool,
    /// Inputs to [`auto_spline_from_cmd`].
    pub spline: AutoSplineFromCmdView,
    /// `wp_nav->set_spline_destination_loc`.
    pub spline_dest_accepted: bool,
    /// `set_next_wp(cmd, dest_loc)`.
    pub next_wp_ok: bool,
    /// `auto_yaw.mode() == AutoYaw::Mode::ROI`.
    pub auto_yaw_is_roi: bool,
    /// `auto_yaw.mode() == AutoYaw::Mode::FIXED`.
    pub auto_yaw_is_fixed: bool,
    /// `copter.g.wp_yaw_behavior == WP_YAW_BEHAVIOR_NONE`.
    pub wp_yaw_behavior_none: bool,
}

impl AutoSplineStartView {
    /// Idle wpnav, dest accepted, next is a spline, yaw HOLD.
    #[must_use]
    pub const fn ready() -> Self {
        Self {
            wp_nav_active: false,
            reached_wp_destination: false,
            wp_dest_loc_ok: false,
            spline: AutoSplineFromCmdView::through_to_spline(),
            spline_dest_accepted: true,
            next_wp_ok: true,
            auto_yaw_is_roi: false,
            auto_yaw_is_fixed: false,
            wp_yaw_behavior_none: false,
        }
    }

    /// Parked on the last WP — default loc comes from wpnav.
    #[must_use]
    pub const fn from_reached_wp() -> Self {
        let mut view = Self::ready();
        view.wp_nav_active = true;
        view.reached_wp_destination = true;
        view.wp_dest_loc_ok = true;
        view
    }

    /// `get_loc_from_cmd` refused the dest.
    #[must_use]
    pub const fn dest_refused() -> Self {
        let mut view = Self::ready();
        view.spline = AutoSplineFromCmdView::dest_refused();
        view
    }

    /// `set_spline_destination_loc` refused (terrain).
    #[must_use]
    pub const fn dest_set_refused() -> Self {
        let mut view = Self::ready();
        view.spline_dest_accepted = false;
        view
    }

    /// `set_next_wp` refused after the spline dest was accepted.
    #[must_use]
    pub const fn next_wp_refused() -> Self {
        let mut view = Self::ready();
        view.next_wp_ok = false;
        view
    }
}

/// Leftover of one `ModeAuto::do_spline_wp` call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoSplineStart {
    /// The leftover reached `set_submode(WP)`. `do_spline_wp` is void;
    /// this is the no-early-return path.
    pub ok: bool,
    /// Default loc came from `get_wp_destination_loc`.
    pub default_from_wp_dest: bool,
    /// `INTERNAL_ERROR(flow_of_control)` — dest loc fetch failed while
    /// parked. The leftover continues with `current_loc`.
    pub dest_loc_flow_of_control: bool,
    /// `copter.failsafe_terrain_on_event` ran.
    pub terrain_failsafe: bool,
    /// What [`auto_spline_from_cmd`] produced.
    pub spline: AutoSplineFromCmd,
    /// `set_spline_destination_loc` succeeded.
    pub set_spline_destination: bool,
    /// `loiter_time` was zeroed.
    pub loiter_time_cleared: bool,
    /// `loiter_time_max = cmd.p1`.
    pub loiter_time_max_s: u16,
    /// `set_next_wp` succeeded.
    pub set_next_wp: bool,
    /// `auto_yaw.set_mode_to_default(false)` ran.
    pub yaw_set_default: bool,
    /// `_mode` after a successful start. `None` on refuse.
    pub submode: Option<AutoSubMode>,
}

impl AutoSplineStart {
    /// Shared refuse leftover: terrain failsafe, no yaw, `_mode` unchanged.
    #[must_use]
    const fn refused(
        default_from_wp_dest: bool,
        dest_loc_flow_of_control: bool,
        spline: AutoSplineFromCmd,
        set_spline_destination: bool,
        loiter_time_cleared: bool,
        loiter_time_max_s: u16,
        set_next_wp: bool,
    ) -> Self {
        Self {
            ok: false,
            default_from_wp_dest,
            dest_loc_flow_of_control,
            terrain_failsafe: true,
            spline,
            set_spline_destination,
            loiter_time_cleared,
            loiter_time_max_s,
            set_next_wp,
            yaw_set_default: false,
            submode: None,
        }
    }
}

/// Upstream `ModeAuto::do_spline_wp` — the spline_start leftover.
///
/// Default loc is `current_loc` (minus offsets). If wpnav is already
/// on its dest, that dest becomes the default; a failed fetch is a
/// flow-of-control error and the leftover keeps `current_loc`.
/// [`auto_spline_from_cmd`] then [`set_spline_destination_loc`]. The
/// loiter delay is latched, `set_next_wp` may add a lookahead, and
/// yaw / `_mode` match [`auto_wp_start`]: skip default yaw for ROI
/// or FIXED+NONE, park in WP. Any refuse after default-loc is
/// terrain failsafe and returns before yaw or `_mode`.
#[must_use]
pub const fn auto_spline_start(view: &AutoSplineStartView) -> AutoSplineStart {
    let mut default_from_wp_dest = false;
    let mut dest_loc_flow_of_control = false;
    if view.wp_nav_active && view.reached_wp_destination {
        if view.wp_dest_loc_ok {
            default_from_wp_dest = true;
        } else {
            dest_loc_flow_of_control = true;
        }
    }

    let spline = auto_spline_from_cmd(&view.spline);
    if !spline.ok {
        return AutoSplineStart::refused(
            default_from_wp_dest,
            dest_loc_flow_of_control,
            spline,
            false,
            false,
            0,
            false,
        );
    }

    if !view.spline_dest_accepted {
        return AutoSplineStart::refused(
            default_from_wp_dest,
            dest_loc_flow_of_control,
            spline,
            false,
            false,
            0,
            false,
        );
    }

    if !view.next_wp_ok {
        return AutoSplineStart::refused(
            default_from_wp_dest,
            dest_loc_flow_of_control,
            spline,
            true,
            true,
            view.spline.delay_s,
            false,
        );
    }

    let skip_yaw = view.auto_yaw_is_roi || (view.auto_yaw_is_fixed && view.wp_yaw_behavior_none);

    AutoSplineStart {
        ok: true,
        default_from_wp_dest,
        dest_loc_flow_of_control,
        terrain_failsafe: false,
        spline,
        set_spline_destination: true,
        loiter_time_cleared: true,
        loiter_time_max_s: view.spline.delay_s,
        set_next_wp: true,
        yaw_set_default: !skip_yaw,
        submode: Some(AutoSubMode::Wp),
    }
}

/// Vehicle view [`auto_loiter_unlimited`] reads.
///
/// `do_loiter_unlimited` fills dest from the command (using the
/// caller's default loc when lat/lon/alt are zero) and then hands
/// that dest to [`auto_wp_start`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoLoiterUnlimitedView {
    /// `wp_nav->is_active()`.
    pub wp_nav_active: bool,
    /// `wp_nav->reached_wp_destination()`.
    pub reached_wp_destination: bool,
    /// `wp_nav->get_wp_destination_loc(default_loc)`.
    pub wp_dest_loc_ok: bool,
    /// `get_loc_from_cmd(cmd, default_loc, target_loc)`.
    pub dest_loc_ok: bool,
    /// Inputs to [`auto_wp_start`].
    pub wp: AutoWpStartView,
}

impl AutoLoiterUnlimitedView {
    /// Idle wpnav, dest accepted, yaw HOLD.
    #[must_use]
    pub const fn ready() -> Self {
        Self {
            wp_nav_active: false,
            reached_wp_destination: false,
            wp_dest_loc_ok: false,
            dest_loc_ok: true,
            wp: AutoWpStartView::idle_loiter(),
        }
    }

    /// Parked on the last WP — default loc comes from wpnav.
    #[must_use]
    pub const fn from_reached_wp() -> Self {
        let mut view = Self::ready();
        view.wp_nav_active = true;
        view.reached_wp_destination = true;
        view.wp_dest_loc_ok = true;
        view.wp.wp_nav_active = true;
        view
    }

    /// `get_loc_from_cmd` refused the dest (terrain).
    #[must_use]
    pub const fn dest_refused() -> Self {
        let mut view = Self::ready();
        view.dest_loc_ok = false;
        view
    }

    /// `wp_start` refused after dest loc was accepted.
    #[must_use]
    pub const fn wp_refused() -> Self {
        let mut view = Self::ready();
        view.wp = AutoWpStartView::dest_refused();
        view
    }
}

/// Leftover of one `ModeAuto::do_loiter_unlimited` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoLoiterUnlimited {
    /// What `do_loiter_unlimited` returned.
    pub ok: bool,
    /// Default loc came from `get_wp_destination_loc`.
    pub default_from_wp_dest: bool,
    /// `INTERNAL_ERROR(flow_of_control)` — dest loc fetch failed while
    /// parked. The leftover continues with `current_loc`.
    pub dest_loc_flow_of_control: bool,
    /// `copter.failsafe_terrain_on_event` ran.
    pub terrain_failsafe: bool,
    /// What [`auto_wp_start`] produced. Zeroed when dest loc refused
    /// before `wp_start` ran.
    pub wp: AutoWpStart,
    /// `_mode` after a successful start. `None` on refuse.
    pub submode: Option<AutoSubMode>,
}

impl AutoLoiterUnlimited {
    /// Shared refuse leftover: terrain failsafe, `_mode` unchanged.
    #[must_use]
    const fn refused(
        default_from_wp_dest: bool,
        dest_loc_flow_of_control: bool,
        wp: AutoWpStart,
    ) -> Self {
        Self {
            ok: false,
            default_from_wp_dest,
            dest_loc_flow_of_control,
            terrain_failsafe: true,
            wp,
            submode: None,
        }
    }
}

/// `wp_start` was never called — dest loc refused first.
#[must_use]
const fn wp_start_not_called() -> AutoWpStart {
    AutoWpStart {
        ok: false,
        wp_and_spline_init: false,
        init_speed_xy_ms: 0.0,
        stopping_point_from_takeoff: false,
        set_speed_up: false,
        set_speed_down: false,
        set_wp_destination: false,
        yaw_set_default: false,
        submode: None,
    }
}

/// Upstream `ModeAuto::do_loiter_unlimited`.
///
/// Default loc is `current_loc` (minus offsets). If wpnav is already
/// on its dest, that dest becomes the default; a failed fetch is a
/// flow-of-control error and the leftover keeps `current_loc`.
/// `get_loc_from_cmd` then [`auto_wp_start`]. Either refuse is
/// terrain failsafe and returns false. Success parks in WP via
/// `wp_start` and returns true.
#[must_use]
pub const fn auto_loiter_unlimited(view: &AutoLoiterUnlimitedView) -> AutoLoiterUnlimited {
    let mut default_from_wp_dest = false;
    let mut dest_loc_flow_of_control = false;
    if view.wp_nav_active && view.reached_wp_destination {
        if view.wp_dest_loc_ok {
            default_from_wp_dest = true;
        } else {
            dest_loc_flow_of_control = true;
        }
    }

    if !view.dest_loc_ok {
        return AutoLoiterUnlimited::refused(
            default_from_wp_dest,
            dest_loc_flow_of_control,
            wp_start_not_called(),
        );
    }

    let wp = auto_wp_start(&view.wp);
    if !wp.ok {
        return AutoLoiterUnlimited::refused(default_from_wp_dest, dest_loc_flow_of_control, wp);
    }

    AutoLoiterUnlimited {
        ok: true,
        default_from_wp_dest,
        dest_loc_flow_of_control,
        terrain_failsafe: false,
        wp,
        submode: wp.submode,
    }
}

/// Vehicle view [`auto_loiter_time`] reads.
///
/// `do_loiter_time` reuses [`auto_loiter_unlimited`] and then latches
/// the loiter delay from `cmd.p1`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoLoiterTimeView {
    /// Inputs to [`auto_loiter_unlimited`].
    pub unlimited: AutoLoiterUnlimitedView,
    /// `cmd.p1` — delay at the loiter point, seconds.
    pub delay_s: u16,
}

impl AutoLoiterTimeView {
    /// Dest accepted, a 10 s loiter.
    #[must_use]
    pub const fn ready() -> Self {
        Self {
            unlimited: AutoLoiterUnlimitedView::ready(),
            delay_s: 10,
        }
    }
}

/// Leftover of one `ModeAuto::do_loiter_time` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoLoiterTime {
    /// What [`auto_loiter_unlimited`] produced.
    pub unlimited: AutoLoiterUnlimited,
    /// `loiter_time` was zeroed.
    pub loiter_time_cleared: bool,
    /// `loiter_time_max = cmd.p1`. Zero when unlimited refused.
    pub loiter_time_max_s: u16,
}

/// Upstream `ModeAuto::do_loiter_time`.
///
/// Reuses [`auto_loiter_unlimited`]. A refuse returns without
/// touching the timer. Success zeros `loiter_time` and stores
/// `cmd.p1` as `loiter_time_max`.
#[must_use]
pub const fn auto_loiter_time(view: &AutoLoiterTimeView) -> AutoLoiterTime {
    let unlimited = auto_loiter_unlimited(&view.unlimited);
    if !unlimited.ok {
        return AutoLoiterTime {
            unlimited,
            loiter_time_cleared: false,
            loiter_time_max_s: 0,
        };
    }
    AutoLoiterTime {
        unlimited,
        loiter_time_cleared: true,
        loiter_time_max_s: view.delay_s,
    }
}

/// Vehicle view [`auto_loiter_to_alt`] reads.
///
/// `do_loiter_to_alt` reuses [`auto_loiter_unlimited`], then reads
/// the command's alt-above-home and parks in `LOITER_TO_ALT`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoLoiterToAltView {
    /// Inputs to [`auto_loiter_unlimited`].
    pub unlimited: AutoLoiterUnlimitedView,
    /// `cmd.content.location.lat == 0 && lng == 0`.
    pub lat_lng_zero: bool,
    /// `target_loc.get_alt_m(ABOVE_HOME, loiter_to_alt.alt_m)`.
    pub alt_ok: bool,
    /// `wp_nav->get_default_speed_down_ms()`.
    pub speed_down_ms: f32,
    /// `wp_nav->get_default_speed_up_ms()`.
    pub speed_up_ms: f32,
    /// `wp_nav->get_accel_D_mss()`.
    pub accel_d_mss: f32,
}

impl AutoLoiterToAltView {
    /// Dest accepted, a real lat/lng, alt-above-home ok.
    #[must_use]
    pub const fn ready() -> Self {
        Self {
            unlimited: AutoLoiterUnlimitedView::ready(),
            lat_lng_zero: false,
            alt_ok: true,
            speed_down_ms: 1.5,
            speed_up_ms: 2.5,
            accel_d_mss: 2.5,
        }
    }

    /// Zero lat/lng — leftover copies current before the alt read.
    #[must_use]
    pub const fn current_lat_lng() -> Self {
        let mut view = Self::ready();
        view.lat_lng_zero = true;
        view
    }

    /// `get_alt_m(ABOVE_HOME)` refused.
    #[must_use]
    pub const fn bad_alt() -> Self {
        let mut view = Self::ready();
        view.alt_ok = false;
        view
    }
}

/// Leftover of one `ModeAuto::do_loiter_to_alt` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoLoiterToAlt {
    /// What [`auto_loiter_unlimited`] produced.
    pub unlimited: AutoLoiterUnlimited,
    /// Current lat/lng were copied onto a zero command loc.
    pub used_current_lat_lng: bool,
    /// `"bad do_loiter_to_alt"` — `get_alt_m` refused.
    pub bad_alt: bool,
    /// `loiter_to_alt.reached_destination_xy` after the leftover.
    pub reached_destination_xy: bool,
    /// `loiter_to_alt.loiter_start_done` after the leftover.
    pub loiter_start_done: bool,
    /// `loiter_to_alt.reached_alt` after the leftover.
    pub reached_alt: bool,
    /// `loiter_to_alt.alt_error_m` after a successful start.
    pub alt_error_m: f32,
    /// First argument to `D_set_max_speed_accel_m`. Zero when unused.
    pub d_speed_down_ms: f32,
    /// Second argument to `D_set_max_speed_accel_m`. Zero when unused.
    pub d_speed_up_ms: f32,
    /// Third argument to `D_set_max_speed_accel_m`. Zero when unused.
    pub d_accel_mss: f32,
    /// Both `D_set_max_speed_accel_m` and the correction twin ran.
    pub d_limits_set: bool,
    /// `_mode` after the leftover. `None` when unlimited refused.
    pub submode: Option<AutoSubMode>,
}

impl AutoLoiterToAlt {
    /// Shared refuse leftover: unlimited failed; nothing else ran.
    #[must_use]
    const fn refused(unlimited: AutoLoiterUnlimited) -> Self {
        Self {
            unlimited,
            used_current_lat_lng: false,
            bad_alt: false,
            reached_destination_xy: false,
            loiter_start_done: false,
            reached_alt: false,
            alt_error_m: 0.0,
            d_speed_down_ms: 0.0,
            d_speed_up_ms: 0.0,
            d_accel_mss: 0.0,
            d_limits_set: false,
            submode: None,
        }
    }
}

/// Upstream `ModeAuto::do_loiter_to_alt`.
///
/// Reuses [`auto_loiter_unlimited`]. A refuse returns without
/// touching `loiter_to_alt`. After a success, a zero lat/lng copies
/// current. A failed `get_alt_m(ABOVE_HOME)` marks both reached,
/// sends `"bad do_loiter_to_alt"`, and leaves `_mode` at WP. Success
/// clears the flags, copies wpnav D limits, and parks in
/// `LOITER_TO_ALT`.
#[must_use]
pub const fn auto_loiter_to_alt(view: &AutoLoiterToAltView) -> AutoLoiterToAlt {
    let unlimited = auto_loiter_unlimited(&view.unlimited);
    if !unlimited.ok {
        return AutoLoiterToAlt::refused(unlimited);
    }

    let used_current_lat_lng = view.lat_lng_zero;
    if !view.alt_ok {
        return AutoLoiterToAlt {
            unlimited,
            used_current_lat_lng,
            bad_alt: true,
            reached_destination_xy: true,
            loiter_start_done: false,
            reached_alt: true,
            alt_error_m: 0.0,
            d_speed_down_ms: 0.0,
            d_speed_up_ms: 0.0,
            d_accel_mss: 0.0,
            d_limits_set: false,
            submode: unlimited.submode,
        };
    }

    AutoLoiterToAlt {
        unlimited,
        used_current_lat_lng,
        bad_alt: false,
        reached_destination_xy: false,
        loiter_start_done: false,
        reached_alt: false,
        alt_error_m: 0.0,
        d_speed_down_ms: view.speed_down_ms,
        d_speed_up_ms: view.speed_up_ms,
        d_accel_mss: view.accel_d_mss,
        d_limits_set: true,
        submode: Some(AutoSubMode::LoiterToAlt),
    }
}

/// `SPEED_TYPE_AIRSPEED`.
pub const SPEED_TYPE_AIRSPEED: u8 = 0;
/// `SPEED_TYPE_GROUNDSPEED`.
pub const SPEED_TYPE_GROUNDSPEED: u8 = 1;
/// `SPEED_TYPE_CLIMB_SPEED`.
pub const SPEED_TYPE_CLIMB_SPEED: u8 = 2;
/// `SPEED_TYPE_DESCENT_SPEED`.
pub const SPEED_TYPE_DESCENT_SPEED: u8 = 3;

/// `HIGHBYTE(p1)` — circle radius lives in the high byte.
#[must_use]
pub const fn highbyte(p1: u16) -> u16 {
    p1 >> 8
}

/// `LOWBYTE(p1)` — loiter-turns lives in the low byte.
#[must_use]
pub const fn lowbyte(p1: u16) -> u16 {
    p1 & 0x00ff
}

/// `cmd.get_loiter_turns()`.
///
/// The low byte is the turn count. Type-specific bit 1 stores a
/// fractional count in 1/256ths.
#[must_use]
pub const fn loiter_turns(p1: u16, fractional: bool) -> f32 {
    let mut turns = lowbyte(p1) as f32;
    if fractional {
        turns *= 1.0 / 256.0;
    }
    turns
}

/// Radius `do_circle` hands to `circle_movetoedge_start`.
///
/// `HIGHBYTE(p1)`, times ten when the command is `NAV_LOITER_TURNS` and
/// type-specific bit 0 is set.
#[must_use]
pub const fn circle_radius_m(p1: u16, nav_loiter_turns: bool, large_radius: bool) -> u16 {
    let mut radius = highbyte(p1);
    if nav_loiter_turns && large_radius {
        radius = radius.wrapping_mul(10);
    }
    radius
}

/// Yaw `do_circle` / `circle_start` selected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutoCircleYaw {
    /// ROI was already holding the axis; neither path touched it.
    Unchanged,
    /// Outside the circle and more than 5 m from the centre: default yaw.
    Default,
    /// Inside the circle (or within 5 m of the centre): HOLD.
    Hold,
    /// `circle_start` set `CIRCLE`.
    Circle,
}

/// Vehicle view [`auto_circle`] reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoCircleView {
    /// `get_loc_from_cmd` succeeded.
    pub dest_ok: bool,
    /// `cmd.id == MAV_CMD_NAV_LOITER_TURNS`.
    pub nav_loiter_turns: bool,
    /// `cmd.p1`.
    pub p1: u16,
    /// `cmd.type_specific_bits & (1U << 0)`.
    pub large_radius: bool,
    /// `cmd.content.location.loiter_ccw`.
    pub loiter_ccw: bool,
    /// `circle_nav->get_rate_degs()` before the sign is applied.
    pub current_rate_degs: f32,
    /// `dist_to_edge_m` from `get_closest_point_on_circle_NED_m`.
    pub dist_to_edge_m: f32,
    /// `set_wp_destination_loc(circle_edge)` when flying to the edge.
    pub dest_accepted: bool,
    /// Horizontal distance from the vehicle to the circle centre, m.
    pub dist_to_center_m: f32,
    /// `auto_yaw.mode() == AutoYaw::Mode::ROI`.
    pub auto_yaw_is_roi: bool,
}

impl AutoCircleView {
    /// Dest ok, 20 m radius, 10 m from the edge, yaw HOLD.
    #[must_use]
    pub const fn ready_to_edge() -> Self {
        Self {
            dest_ok: true,
            nav_loiter_turns: true,
            p1: 0x1400,
            large_radius: false,
            loiter_ccw: false,
            current_rate_degs: 20.0,
            dist_to_edge_m: 10.0,
            dest_accepted: true,
            dist_to_center_m: 30.0,
            auto_yaw_is_roi: false,
        }
    }

    /// Already on the edge: `circle_start` runs immediately.
    #[must_use]
    pub const fn already_on_edge() -> Self {
        let mut view = Self::ready_to_edge();
        view.dist_to_edge_m = 2.0;
        view.dist_to_center_m = 18.0;
        view
    }

    /// `get_loc_from_cmd` refused.
    #[must_use]
    pub const fn dest_refused() -> Self {
        let mut view = Self::ready_to_edge();
        view.dest_ok = false;
        view
    }
}

/// Leftover of one `ModeAuto::do_circle` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoCircle {
    /// The leftover ran past the dest gate.
    pub ok: bool,
    /// `failsafe_terrain_on_event` ran.
    pub terrain_failsafe: bool,
    /// Radius handed to `circle_nav->set_radius_m`.
    pub radius_m: u16,
    /// Signed rate handed to `circle_nav->set_rate_degs`.
    pub rate_degs: f32,
    /// Flew to the edge (`dist_to_edge_m > 3`).
    pub move_to_edge: bool,
    /// `set_wp_destination_loc` ran (only on the fly-to-edge arm).
    pub set_wp_destination: bool,
    /// Yaw leftover.
    pub yaw: AutoCircleYaw,
    /// `_mode` after the leftover. `None` when dest refused.
    pub submode: Option<AutoSubMode>,
    /// `circle_last_num_complete` after the leftover. `None` when dest refused.
    pub last_num_complete: Option<f32>,
}

impl AutoCircle {
    /// Shared refuse leftover: dest failed; nothing else ran.
    #[must_use]
    const fn refused() -> Self {
        Self {
            ok: false,
            terrain_failsafe: true,
            radius_m: 0,
            rate_degs: 0.0,
            move_to_edge: false,
            set_wp_destination: false,
            yaw: AutoCircleYaw::Unchanged,
            submode: None,
            last_num_complete: None,
        }
    }
}

/// Signed circle rate: ccw is `-fabsf(current)`, cw is `fabsf(current)`.
#[must_use]
const fn circle_rate_degs(current_rate_degs: f32, ccw: bool) -> f32 {
    let mag = if current_rate_degs < 0.0 {
        -current_rate_degs
    } else {
        current_rate_degs
    };
    if ccw {
        -mag
    } else {
        mag
    }
}

/// Yaw while flying to the circle edge. ROI is left alone.
#[must_use]
const fn circle_edge_yaw(
    auto_yaw_is_roi: bool,
    dist_to_center_m: f32,
    radius_m: f32,
) -> AutoCircleYaw {
    if auto_yaw_is_roi {
        return AutoCircleYaw::Unchanged;
    }
    if dist_to_center_m > radius_m && dist_to_center_m > 5.0 {
        AutoCircleYaw::Default
    } else {
        AutoCircleYaw::Hold
    }
}

/// Upstream `ModeAuto::circle_start` yaw leftover.
#[must_use]
const fn circle_start_yaw(auto_yaw_is_roi: bool) -> AutoCircleYaw {
    if auto_yaw_is_roi {
        AutoCircleYaw::Unchanged
    } else {
        AutoCircleYaw::Circle
    }
}

/// Upstream `ModeAuto::do_circle`.
///
/// Dest first. Radius is the high byte of `p1`, times ten when the
/// large-radius bit is set on `NAV_LOITER_TURNS`. More than 3 m from the
/// edge flies there; a dest refuse on that fly-to still sets yaw and
/// `CIRCLE_MOVE_TO_EDGE`. Already on the edge calls `circle_start`.
/// Success always resets `circle_last_num_complete` to -1.
#[must_use]
pub const fn auto_circle(view: &AutoCircleView) -> AutoCircle {
    if !view.dest_ok {
        return AutoCircle::refused();
    }

    let radius_m = circle_radius_m(view.p1, view.nav_loiter_turns, view.large_radius);
    let rate_degs = circle_rate_degs(view.current_rate_degs, view.loiter_ccw);

    if view.dist_to_edge_m > 3.0 {
        let terrain_failsafe = !view.dest_accepted;
        let yaw = circle_edge_yaw(
            view.auto_yaw_is_roi,
            view.dist_to_center_m,
            radius_m as f32,
        );
        return AutoCircle {
            ok: true,
            terrain_failsafe,
            radius_m,
            rate_degs,
            move_to_edge: true,
            set_wp_destination: true,
            yaw,
            submode: Some(AutoSubMode::CircleMoveToEdge),
            last_num_complete: Some(-1.0),
        };
    }

    AutoCircle {
        ok: true,
        terrain_failsafe: false,
        radius_m,
        rate_degs,
        move_to_edge: false,
        set_wp_destination: false,
        yaw: circle_start_yaw(view.auto_yaw_is_roi),
        submode: Some(AutoSubMode::Circle),
        last_num_complete: Some(-1.0),
    }
}

/// Vehicle view [`auto_do_yaw`] reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoDoYawView {
    /// `cmd.content.yaw.angle_deg`.
    pub angle_deg: f32,
    /// `cmd.content.yaw.turn_rate_dps`.
    pub turn_rate_dps: f32,
    /// `cmd.content.yaw.direction`.
    pub direction: i8,
    /// `cmd.content.yaw.relative_angle`.
    pub relative_angle: i8,
}

impl AutoDoYawView {
    /// Absolute 90 deg at 10 deg/s, shortest.
    #[must_use]
    pub const fn absolute_90() -> Self {
        Self {
            angle_deg: 90.0,
            turn_rate_dps: 10.0,
            direction: 0,
            relative_angle: 0,
        }
    }

    /// Relative 45 deg clockwise.
    #[must_use]
    pub const fn relative_45() -> Self {
        Self {
            angle_deg: 45.0,
            turn_rate_dps: 0.0,
            direction: 1,
            relative_angle: 1,
        }
    }
}

/// Leftover of one `ModeAuto::do_yaw` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoDoYaw {
    /// `radians(angle_deg)` handed to `set_fixed_yaw_rad`.
    pub angle_rad: f32,
    /// `radians(turn_rate_dps)`.
    pub turn_rate_rads: f32,
    /// Direction after `FixedYawDirection::from_sign`.
    pub direction: FixedYawDirection,
    /// `relative_angle > 0`.
    pub relative: bool,
}

/// Upstream `ModeAuto::do_yaw`.
///
/// The leftover is the conversion: degrees to radians, and relative only
/// when `relative_angle > 0` — zero and negative are absolute.
#[must_use]
pub fn auto_do_yaw(view: &AutoDoYawView) -> AutoDoYaw {
    AutoDoYaw {
        angle_rad: view.angle_deg.to_radians(),
        turn_rate_rads: view.turn_rate_dps.to_radians(),
        direction: FixedYawDirection::from_sign(view.direction),
        relative: view.relative_angle > 0,
    }
}

/// Vehicle view [`auto_do_roi`] reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoDoRoiView {
    /// `cmd.content.location.initialised()`.
    pub location_initialised: bool,
    /// `camera_mount.has_pan_control()`.
    pub mount_has_pan_control: bool,
}

impl AutoDoRoiView {
    /// A real location, no pan — the airframe must point.
    #[must_use]
    pub const fn point_airframe() -> Self {
        Self {
            location_initialised: true,
            mount_has_pan_control: false,
        }
    }

    /// Zeros: cancel the ROI.
    #[must_use]
    pub const fn cancel() -> Self {
        Self {
            location_initialised: false,
            mount_has_pan_control: false,
        }
    }
}

/// Leftover of one `ModeAuto::do_roi` call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoDoRoi {
    /// What `auto_yaw.set_roi` decided.
    pub action: RoiAction,
}

/// Upstream `ModeAuto::do_roi`.
///
/// The body is one call to `auto_yaw.set_roi`. The leftover is
/// [`roi_action`].
#[must_use]
pub fn auto_do_roi(view: &AutoDoRoiView) -> AutoDoRoi {
    AutoDoRoi {
        action: roi_action(view.location_initialised, view.mount_has_pan_control),
    }
}

/// Vehicle view [`auto_nav_delay`] reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoNavDelayView {
    /// `cmd.content.nav_delay.seconds`.
    pub seconds: i32,
    /// `AP_RTC_ENABLED`.
    pub rtc_enabled: bool,
    /// `AP::rtc().get_time_utc(...)` when seconds is not positive.
    pub utc_delay_ms: u32,
}

impl AutoNavDelayView {
    /// Relative 5 s delay.
    #[must_use]
    pub const fn relative_5s() -> Self {
        Self {
            seconds: 5,
            rtc_enabled: true,
            utc_delay_ms: 0,
        }
    }

    /// Absolute UTC delay with RTC compiled in.
    #[must_use]
    pub const fn utc() -> Self {
        Self {
            seconds: 0,
            rtc_enabled: true,
            utc_delay_ms: 12_000,
        }
    }
}

/// Leftover of one `ModeAuto::do_nav_delay` call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoNavDelay {
    /// Relative (`seconds > 0`) rather than UTC.
    pub relative: bool,
    /// `nav_delay_time_max_ms` after the leftover.
    pub max_ms: u32,
}

/// Upstream `ModeAuto::do_nav_delay`.
///
/// A positive seconds field is a relative delay. Zero or negative is a
/// UTC time: RTC converts it, or the leftover stores 0 when RTC is off.
#[must_use]
pub const fn auto_nav_delay(view: &AutoNavDelayView) -> AutoNavDelay {
    if view.seconds > 0 {
        return AutoNavDelay {
            relative: true,
            max_ms: (view.seconds as u32).saturating_mul(1000),
        };
    }
    AutoNavDelay {
        relative: false,
        max_ms: if view.rtc_enabled {
            view.utc_delay_ms
        } else {
            0
        },
    }
}

/// Vehicle view [`auto_wait_delay`] reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoWaitDelayView {
    /// `cmd.content.delay.seconds`.
    pub seconds: f32,
}

impl AutoWaitDelayView {
    /// 3 s condition delay.
    #[must_use]
    pub const fn three_seconds() -> Self {
        Self { seconds: 3.0 }
    }
}

/// Leftover of one `ModeAuto::do_wait_delay` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoWaitDelay {
    /// `condition_value` after the leftover — seconds × 1000.
    pub condition_value: f32,
}

/// Upstream `ModeAuto::do_wait_delay`.
///
/// `condition_start` is `millis()`. The leftover stores seconds as
/// milliseconds on `condition_value`.
#[must_use]
pub const fn auto_wait_delay(view: &AutoWaitDelayView) -> AutoWaitDelay {
    AutoWaitDelay {
        condition_value: view.seconds * 1000.0,
    }
}

/// Vehicle view [`auto_within_distance`] reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoWithinDistanceView {
    /// `cmd.content.distance.meters`.
    pub meters: f32,
}

impl AutoWithinDistanceView {
    /// 10 m gate.
    #[must_use]
    pub const fn ten_metres() -> Self {
        Self { meters: 10.0 }
    }
}

/// Leftover of one `ModeAuto::do_within_distance` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoWithinDistance {
    /// `condition_value` after the leftover.
    pub condition_value: f32,
}

/// Upstream `ModeAuto::do_within_distance`.
///
/// The leftover is one assignment: `condition_value = meters`.
#[must_use]
pub const fn auto_within_distance(view: &AutoWithinDistanceView) -> AutoWithinDistance {
    AutoWithinDistance {
        condition_value: view.meters,
    }
}

/// Which speed axis `do_change_speed` wrote.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutoSpeedAxis {
    /// `target_ms` was not positive; nothing ran.
    None,
    /// `SPEED_TYPE_CLIMB_SPEED`.
    Climb,
    /// `SPEED_TYPE_DESCENT_SPEED`.
    Descent,
    /// `SPEED_TYPE_AIRSPEED` or `SPEED_TYPE_GROUNDSPEED`.
    Horizontal,
}

/// Vehicle view [`auto_change_speed`] reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoChangeSpeedView {
    /// `cmd.content.speed.target_ms`.
    pub target_ms: f32,
    /// `cmd.content.speed.speed_type`.
    pub speed_type: u8,
}

impl AutoChangeSpeedView {
    /// 8 m/s groundspeed.
    #[must_use]
    pub const fn groundspeed() -> Self {
        Self {
            target_ms: 8.0,
            speed_type: SPEED_TYPE_GROUNDSPEED,
        }
    }
}

/// Leftover of one `ModeAuto::do_change_speed` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoChangeSpeed {
    /// Which wpnav setter ran.
    pub axis: AutoSpeedAxis,
    /// Value written to the override and the setter. Zero when unused.
    pub target_ms: f32,
}

/// Upstream `ModeAuto::do_change_speed`.
///
/// A non-positive target is a no-op. Climb and descent write the D
/// setters; airspeed and groundspeed share the NE setter.
#[must_use]
pub const fn auto_change_speed(view: &AutoChangeSpeedView) -> AutoChangeSpeed {
    if view.target_ms <= 0.0 {
        return AutoChangeSpeed {
            axis: AutoSpeedAxis::None,
            target_ms: 0.0,
        };
    }
    let axis = match view.speed_type {
        SPEED_TYPE_CLIMB_SPEED => AutoSpeedAxis::Climb,
        SPEED_TYPE_DESCENT_SPEED => AutoSpeedAxis::Descent,
        SPEED_TYPE_AIRSPEED | SPEED_TYPE_GROUNDSPEED => AutoSpeedAxis::Horizontal,
        _ => AutoSpeedAxis::None,
    };
    AutoChangeSpeed {
        axis,
        target_ms: if matches!(axis, AutoSpeedAxis::None) {
            0.0
        } else {
            view.target_ms
        },
    }
}

/// Which home `do_set_home` asked for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutoSetHomeKind {
    /// `p1 == 1` or the location was uninitialised: current location.
    Current,
    /// The command's location.
    Command,
}

/// Vehicle view [`auto_set_home`] reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoSetHomeView {
    /// `cmd.p1`.
    pub p1: u16,
    /// `cmd.content.location.initialised()`.
    pub location_initialised: bool,
}

impl AutoSetHomeView {
    /// `p1 == 1`: current location regardless of the lat/lng.
    #[must_use]
    pub const fn current() -> Self {
        Self {
            p1: 1,
            location_initialised: true,
        }
    }

    /// A real location, `p1 == 0`.
    #[must_use]
    pub const fn command() -> Self {
        Self {
            p1: 0,
            location_initialised: true,
        }
    }
}

/// Leftover of one `ModeAuto::do_set_home` call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoSetHome {
    /// Which setter ran. Failures are ignored either way.
    pub kind: AutoSetHomeKind,
}

/// Upstream `ModeAuto::do_set_home`.
///
/// `p1 == 1` or an uninitialised location uses current. Anything else
/// uses the command location. Both setters ignore failure.
#[must_use]
pub const fn auto_set_home(view: &AutoSetHomeView) -> AutoSetHome {
    let kind = if view.p1 == 1 || !view.location_initialised {
        AutoSetHomeKind::Current
    } else {
        AutoSetHomeKind::Command
    };
    AutoSetHome { kind }
}

/// `PayloadPlace::State` after `do_payload_place`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PayloadPlaceState {
    /// Fly to the command location first.
    FlyToLocation,
    /// `start_descent()` — no location was provided.
    DescentStart,
    /// Descending.
    Descent,
    /// Release.
    Release,
    /// Releasing.
    Releasing,
    /// Delay after release.
    Delay,
    /// Ascent start.
    AscentStart,
    /// Ascent.
    Ascent,
    /// Done — [`auto_verify_payload_place`] returns true.
    Done,
}

/// Vehicle view [`auto_payload_place`] reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoPayloadPlaceView {
    /// `lat != 0 || lng != 0 || alt != 0`.
    pub location_provided: bool,
    /// `get_loc_from_cmd` succeeded.
    pub dest_ok: bool,
    /// Arguments to [`auto_wp_start`] on the fly-to arm.
    pub wp: AutoWpStartView,
    /// `cmd.p1` — centimetres, converted to metres.
    pub p1_cm: u16,
}

impl AutoPayloadPlaceView {
    /// No location: start descent at the current point.
    #[must_use]
    pub const fn descent_here() -> Self {
        Self {
            location_provided: false,
            dest_ok: true,
            wp: AutoWpStartView::idle_loiter(),
            p1_cm: 500,
        }
    }

    /// Fly to a location, dest and wp_start both accept.
    #[must_use]
    pub const fn fly_to() -> Self {
        Self {
            location_provided: true,
            dest_ok: true,
            wp: AutoWpStartView::idle_loiter(),
            p1_cm: 500,
        }
    }
}

/// Leftover of one `ModeAuto::do_payload_place` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoPayloadPlace {
    /// The leftover ran past the dest/wp gate (or never needed it).
    pub ok: bool,
    /// `failsafe_terrain_on_event` ran.
    pub terrain_failsafe: bool,
    /// `payload_place.state` after the leftover. `None` on refuse.
    pub state: Option<PayloadPlaceState>,
    /// `wp_start` leftover on the fly-to arm.
    pub wp: AutoWpStart,
    /// `payload_place.descent_max_m`. Zero on refuse.
    pub descent_max_m: f32,
    /// `_mode` after the leftover. `None` on refuse.
    pub submode: Option<AutoSubMode>,
}

impl AutoPayloadPlace {
    #[must_use]
    const fn refused(wp: AutoWpStart) -> Self {
        Self {
            ok: false,
            terrain_failsafe: true,
            state: None,
            wp,
            descent_max_m: 0.0,
            submode: None,
        }
    }
}

/// Upstream `ModeAuto::do_payload_place`.
///
/// A non-zero lat/lng/alt flies there via [`auto_wp_start`]. Either
/// refuse is terrain failsafe and returns before `descent_max` or
/// `_mode`. No location calls `start_descent`. Success always writes
/// `descent_max_m = p1 * 0.01` and parks in `NAV_PAYLOAD_PLACE`.
#[must_use]
pub const fn auto_payload_place(view: &AutoPayloadPlaceView) -> AutoPayloadPlace {
    if view.location_provided {
        if !view.dest_ok {
            return AutoPayloadPlace::refused(wp_start_not_called());
        }
        let wp = auto_wp_start(&view.wp);
        if !wp.ok {
            return AutoPayloadPlace::refused(wp);
        }
        return AutoPayloadPlace {
            ok: true,
            terrain_failsafe: false,
            state: Some(PayloadPlaceState::FlyToLocation),
            wp,
            descent_max_m: view.p1_cm as f32 * 0.01,
            submode: Some(AutoSubMode::NavPayloadPlace),
        };
    }

    AutoPayloadPlace {
        ok: true,
        terrain_failsafe: false,
        state: Some(PayloadPlaceState::DescentStart),
        wp: wp_start_not_called(),
        descent_max_m: view.p1_cm as f32 * 0.01,
        submode: Some(AutoSubMode::NavPayloadPlace),
    }
}

/// Which `verify_*` `ModeAuto::verify_command` selected.
///
/// The leftover is the *dispatch*, not the body — same shape as
/// [`AutoStartHandler`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutoVerifyHandler {
    /// Not in AUTO: the switch does not run.
    NotInAuto,
    /// `verify_takeoff`.
    VerifyTakeoff,
    /// `verify_nav_wp`.
    VerifyNavWp,
    /// `verify_land`.
    VerifyLand,
    /// `payload_place.verify`.
    VerifyPayloadPlace,
    /// `verify_loiter_unlimited`.
    VerifyLoiterUnlimited,
    /// `verify_circle`.
    VerifyCircle,
    /// `verify_loiter_time`.
    VerifyLoiterTime,
    /// `verify_loiter_to_alt` — early-return arm.
    VerifyLoiterToAlt,
    /// `verify_RTL`.
    VerifyRtl,
    /// `verify_spline_wp`.
    VerifySplineWp,
    /// `verify_nav_guided_enable`.
    VerifyNavGuidedEnable,
    /// `verify_nav_delay`.
    VerifyNavDelay,
    /// `verify_nav_script_time`.
    VerifyNavScriptTime,
    /// `verify_nav_attitude_time`.
    VerifyNavAttitudeTime,
    /// `verify_wait_delay`.
    VerifyWaitDelay,
    /// `verify_within_distance`.
    VerifyWithinDistance,
    /// `verify_yaw`.
    VerifyYaw,
    /// DO commands — complete immediately.
    DoAlwaysComplete,
    /// `default` — `"Skipping invalid cmd"` and complete.
    SkipInvalid,
}

/// Leftover of one `ModeAuto::verify_command` call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoVerifyCommand {
    /// The flight mode was AUTO, so the switch ran.
    pub ran: bool,
    /// Which `verify_*` the switch selected.
    pub handler: AutoVerifyHandler,
    /// Complete without calling a body (DO_* / unknown).
    pub immediate_complete: bool,
    /// `NAV_LOITER_TO_ALT` returns the body leftover and skips the
    /// reached message.
    pub early_return: bool,
    /// `"Skipping invalid cmd #%i"`.
    pub skip_invalid_text: bool,
}

impl AutoVerifyCommand {
    #[must_use]
    const fn not_in_auto() -> Self {
        Self {
            ran: false,
            handler: AutoVerifyHandler::NotInAuto,
            immediate_complete: false,
            early_return: false,
            skip_invalid_text: false,
        }
    }

    #[must_use]
    const fn body(handler: AutoVerifyHandler) -> Self {
        Self {
            ran: true,
            handler,
            immediate_complete: false,
            early_return: false,
            skip_invalid_text: false,
        }
    }

    #[must_use]
    const fn immediate(handler: AutoVerifyHandler, skip_invalid_text: bool) -> Self {
        Self {
            ran: true,
            handler,
            immediate_complete: true,
            early_return: false,
            skip_invalid_text,
        }
    }

    #[must_use]
    const fn gated(on: bool, handler: AutoVerifyHandler) -> Self {
        if on {
            Self::body(handler)
        } else {
            Self::immediate(AutoVerifyHandler::SkipInvalid, true)
        }
    }
}

/// Upstream `ModeAuto::verify_command`.
///
/// The leftover is the *switch*. Not in AUTO returns false without
/// touching a body. Recognised ids pick a `verify_*`. DO commands and
/// an unknown id complete immediately. `NAV_LOITER_TO_ALT` is the
/// early-return arm.
#[must_use]
pub const fn auto_verify_command(
    in_auto: bool,
    cmd_id: u16,
    features: AutoStartFeatures,
) -> AutoVerifyCommand {
    if !in_auto {
        return AutoVerifyCommand::not_in_auto();
    }
    match cmd_id {
        MAV_CMD_NAV_TAKEOFF | MAV_CMD_NAV_VTOL_TAKEOFF => {
            AutoVerifyCommand::body(AutoVerifyHandler::VerifyTakeoff)
        }
        MAV_CMD_NAV_WAYPOINT | MAV_CMD_NAV_ARC_WAYPOINT => {
            AutoVerifyCommand::body(AutoVerifyHandler::VerifyNavWp)
        }
        MAV_CMD_NAV_LAND | MAV_CMD_NAV_VTOL_LAND => {
            AutoVerifyCommand::body(AutoVerifyHandler::VerifyLand)
        }
        MAV_CMD_NAV_PAYLOAD_PLACE => {
            AutoVerifyCommand::gated(features.payload_place, AutoVerifyHandler::VerifyPayloadPlace)
        }
        MAV_CMD_NAV_LOITER_UNLIM => {
            AutoVerifyCommand::body(AutoVerifyHandler::VerifyLoiterUnlimited)
        }
        MAV_CMD_NAV_LOITER_TURNS => AutoVerifyCommand::body(AutoVerifyHandler::VerifyCircle),
        MAV_CMD_NAV_LOITER_TIME => AutoVerifyCommand::body(AutoVerifyHandler::VerifyLoiterTime),
        MAV_CMD_NAV_LOITER_TO_ALT => AutoVerifyCommand {
            ran: true,
            handler: AutoVerifyHandler::VerifyLoiterToAlt,
            immediate_complete: false,
            early_return: true,
            skip_invalid_text: false,
        },
        MAV_CMD_NAV_RETURN_TO_LAUNCH => AutoVerifyCommand::body(AutoVerifyHandler::VerifyRtl),
        MAV_CMD_NAV_SPLINE_WAYPOINT => AutoVerifyCommand::body(AutoVerifyHandler::VerifySplineWp),
        MAV_CMD_NAV_GUIDED_ENABLE => {
            AutoVerifyCommand::gated(features.nav_guided, AutoVerifyHandler::VerifyNavGuidedEnable)
        }
        MAV_CMD_NAV_DELAY => AutoVerifyCommand::body(AutoVerifyHandler::VerifyNavDelay),
        MAV_CMD_NAV_SCRIPT_TIME => {
            AutoVerifyCommand::gated(features.scripting, AutoVerifyHandler::VerifyNavScriptTime)
        }
        MAV_CMD_NAV_ATTITUDE_TIME => {
            AutoVerifyCommand::body(AutoVerifyHandler::VerifyNavAttitudeTime)
        }
        MAV_CMD_CONDITION_DELAY => AutoVerifyCommand::body(AutoVerifyHandler::VerifyWaitDelay),
        MAV_CMD_CONDITION_DISTANCE => {
            AutoVerifyCommand::body(AutoVerifyHandler::VerifyWithinDistance)
        }
        MAV_CMD_CONDITION_YAW => AutoVerifyCommand::body(AutoVerifyHandler::VerifyYaw),
        MAV_CMD_DO_CHANGE_SPEED
        | MAV_CMD_DO_SET_HOME
        | MAV_CMD_DO_SET_ROI_LOCATION
        | MAV_CMD_DO_SET_ROI_NONE
        | MAV_CMD_DO_SET_ROI
        | MAV_CMD_DO_RETURN_PATH_START
        | MAV_CMD_DO_LAND_START => {
            AutoVerifyCommand::immediate(AutoVerifyHandler::DoAlwaysComplete, false)
        }
        MAV_CMD_DO_MOUNT_CONTROL => AutoVerifyCommand::gated(
            features.mount,
            AutoVerifyHandler::DoAlwaysComplete,
        )
        .immediate_if_on(),
        MAV_CMD_DO_GUIDED_LIMITS => AutoVerifyCommand::gated(
            features.nav_guided,
            AutoVerifyHandler::DoAlwaysComplete,
        )
        .immediate_if_on(),
        MAV_CMD_DO_WINCH => {
            AutoVerifyCommand::gated(features.winch, AutoVerifyHandler::DoAlwaysComplete)
                .immediate_if_on()
        }
        _ => AutoVerifyCommand::immediate(AutoVerifyHandler::SkipInvalid, true),
    }
}

impl AutoVerifyCommand {
    /// Gated DO commands complete immediately when the `#if` is on.
    #[must_use]
    const fn immediate_if_on(self) -> Self {
        if matches!(self.handler, AutoVerifyHandler::DoAlwaysComplete) {
            Self {
                immediate_complete: true,
                ..self
            }
        } else {
            self
        }
    }
}

/// `ModeAuto::state` for [`auto_verify_land`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutoLandState {
    /// Flying to the land location.
    FlyToLocation,
    /// Descending.
    Descending,
}

/// Vehicle view [`auto_verify_land`] reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoVerifyLandView {
    /// `state`.
    pub state: AutoLandState,
    /// `wp_nav->reached_wp_destination()`.
    pub reached_wp: bool,
    /// `copter.ap.land_complete`.
    pub land_complete: bool,
    /// `motors->get_spool_state() == GROUND_IDLE`.
    pub ground_idle: bool,
    /// `mission.continue_after_land_check_for_takeoff()`.
    pub continue_after_land: bool,
    /// `copter.motors->armed()`.
    pub armed: bool,
}

impl AutoVerifyLandView {
    /// Still flying to the land location.
    #[must_use]
    pub const fn flying_to() -> Self {
        Self {
            state: AutoLandState::FlyToLocation,
            reached_wp: false,
            land_complete: false,
            ground_idle: false,
            continue_after_land: false,
            armed: true,
        }
    }

    /// Arrived; `land_start` should run.
    #[must_use]
    pub const fn arrived() -> Self {
        let mut view = Self::flying_to();
        view.reached_wp = true;
        view
    }

    /// On the ground, mission should stop and disarm.
    #[must_use]
    pub const fn landed() -> Self {
        Self {
            state: AutoLandState::Descending,
            reached_wp: true,
            land_complete: true,
            ground_idle: true,
            continue_after_land: false,
            armed: true,
        }
    }
}

/// Leftover of one `ModeAuto::verify_land` call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoVerifyLand {
    /// What `verify_land` returned.
    pub complete: bool,
    /// `land_start()` ran (FlyToLocation arrived).
    pub land_start: bool,
    /// `state` after the leftover.
    pub state: AutoLandState,
    /// `arming.disarm(LANDED)` ran.
    pub disarm: bool,
    /// `INTERNAL_ERROR(flow_of_control)`.
    pub flow_of_control: bool,
}

/// Upstream `ModeAuto::verify_land`.
///
/// FlyToLocation waits for the dest, then `land_start` and Descending.
/// Descending is complete only when landed and ground-idle; a completed
/// land that should not continue disarms and reports *not* complete so
/// the mission stays on NAV_LAND.
#[must_use]
pub const fn auto_verify_land(view: &AutoVerifyLandView) -> AutoVerifyLand {
    match view.state {
        AutoLandState::FlyToLocation => AutoVerifyLand {
            complete: false,
            land_start: view.reached_wp,
            state: if view.reached_wp {
                AutoLandState::Descending
            } else {
                AutoLandState::FlyToLocation
            },
            disarm: false,
            flow_of_control: false,
        },
        AutoLandState::Descending => {
            let landed = view.land_complete && view.ground_idle;
            let disarm = landed && !view.continue_after_land && view.armed;
            AutoVerifyLand {
                complete: landed && !disarm,
                land_start: false,
                state: AutoLandState::Descending,
                disarm,
                flow_of_control: false,
            }
        }
    }
}

/// Vehicle view [`auto_verify_loiter_time`] / [`auto_verify_nav_wp`] /
/// [`auto_verify_spline_wp`] share.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoVerifyWpTimerView {
    /// `wp_nav->reached_wp_destination()`.
    pub reached_wp: bool,
    /// `loiter_time == 0` at the top of the call.
    pub timer_unset: bool,
    /// Elapsed seconds since `loiter_time` was latched. Zero when unset.
    pub elapsed_s: u32,
    /// `loiter_time_max` (seconds).
    pub loiter_time_max: u16,
}

impl AutoVerifyWpTimerView {
    /// Not yet at the dest.
    #[must_use]
    pub const fn en_route() -> Self {
        Self {
            reached_wp: false,
            timer_unset: true,
            elapsed_s: 0,
            loiter_time_max: 0,
        }
    }

    /// Just arrived, no delay.
    #[must_use]
    pub const fn arrived_no_delay() -> Self {
        Self {
            reached_wp: true,
            timer_unset: true,
            elapsed_s: 0,
            loiter_time_max: 0,
        }
    }

    /// Arrived and the delay has run out.
    #[must_use]
    pub const fn delay_done(max_s: u16) -> Self {
        Self {
            reached_wp: true,
            timer_unset: false,
            elapsed_s: max_s as u32,
            loiter_time_max: max_s,
        }
    }
}

/// Leftover of one `verify_loiter_time` / `verify_nav_wp` /
/// `verify_spline_wp` call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoVerifyWpTimer {
    /// What the leftover returned.
    pub complete: bool,
    /// The timer was latched this call.
    pub timer_started: bool,
    /// `AP_Notify::events.waypoint_complete = 1` (nav_wp only).
    pub waypoint_complete_notify: bool,
    /// `"Reached command #%i"`.
    pub reached_text: bool,
}

/// Shared dest-then-timer leftover used by loiter-time, nav-wp, and spline.
#[must_use]
const fn verify_wp_timer(
    view: &AutoVerifyWpTimerView,
    notify_on_start: bool,
    notify_on_zero_max: bool,
) -> AutoVerifyWpTimer {
    if !view.reached_wp {
        return AutoVerifyWpTimer {
            complete: false,
            timer_started: false,
            waypoint_complete_notify: false,
            reached_text: false,
        };
    }
    let timer_started = view.timer_unset;
    let complete = view.elapsed_s >= view.loiter_time_max as u32;
    let notify_start = timer_started && notify_on_start && view.loiter_time_max > 0;
    let notify_done = complete && notify_on_zero_max && view.loiter_time_max == 0;
    AutoVerifyWpTimer {
        complete,
        timer_started,
        waypoint_complete_notify: notify_start || notify_done,
        reached_text: complete,
    }
}

/// Upstream `ModeAuto::verify_loiter_time`.
#[must_use]
pub const fn auto_verify_loiter_time(view: &AutoVerifyWpTimerView) -> AutoVerifyWpTimer {
    verify_wp_timer(view, false, false)
}

/// Upstream `ModeAuto::verify_nav_wp`.
///
/// Notify on timer start when there is a delay; notify on complete when
/// there is not.
#[must_use]
pub const fn auto_verify_nav_wp(view: &AutoVerifyWpTimerView) -> AutoVerifyWpTimer {
    verify_wp_timer(view, true, true)
}

/// Upstream `ModeAuto::verify_spline_wp`.
#[must_use]
pub const fn auto_verify_spline_wp(view: &AutoVerifyWpTimerView) -> AutoVerifyWpTimer {
    verify_wp_timer(view, false, false)
}

/// Upstream `ModeAuto::verify_loiter_unlimited`. Always false.
#[must_use]
pub const fn auto_verify_loiter_unlimited() -> bool {
    false
}

/// Upstream `ModeAuto::verify_loiter_to_alt`.
#[must_use]
pub const fn auto_verify_loiter_to_alt(reached_xy: bool, reached_alt: bool) -> bool {
    reached_xy && reached_alt
}

/// Upstream `ModeAuto::verify_takeoff`.
#[must_use]
pub const fn auto_verify_takeoff(complete: bool) -> bool {
    complete
}

/// Vehicle view [`auto_verify_rtl`] reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoVerifyRtlView {
    /// `mode_rtl.state_complete()`.
    pub state_complete: bool,
    /// `mode_rtl.state()`.
    pub state: RtlSubMode,
    /// `motors->get_spool_state() == GROUND_IDLE`.
    pub ground_idle: bool,
}

impl AutoVerifyRtlView {
    /// Landed in FINAL_DESCENT, ground idle.
    #[must_use]
    pub const fn landed() -> Self {
        Self {
            state_complete: true,
            state: RtlSubMode::FinalDescent,
            ground_idle: true,
        }
    }
}

/// Upstream `ModeAuto::verify_RTL`.
///
/// Complete only when RTL is done, the submode is FINAL_DESCENT or LAND,
/// and the motors are ground-idle.
#[must_use]
pub const fn auto_verify_rtl(view: &AutoVerifyRtlView) -> bool {
    view.state_complete
        && matches!(view.state, RtlSubMode::FinalDescent | RtlSubMode::Land)
        && view.ground_idle
}

/// Vehicle view [`auto_verify_wait_delay`] reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoVerifyWaitDelayView {
    /// `millis() - condition_start`.
    pub elapsed_ms: u32,
    /// `condition_value` (milliseconds, may be negative).
    pub condition_value: i32,
}

impl AutoVerifyWaitDelayView {
    /// Still waiting.
    #[must_use]
    pub const fn waiting() -> Self {
        Self {
            elapsed_ms: 500,
            condition_value: 3000,
        }
    }

    /// Delay has run out.
    #[must_use]
    pub const fn done() -> Self {
        Self {
            elapsed_ms: 3001,
            condition_value: 3000,
        }
    }
}

/// Leftover of one `ModeAuto::verify_wait_delay` call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoVerifyWaitDelay {
    /// What `verify_wait_delay` returned.
    pub complete: bool,
    /// `condition_value` was cleared.
    pub cleared: bool,
}

/// Upstream `ModeAuto::verify_wait_delay`.
///
/// Complete when elapsed is *greater than* `MAX(condition_value, 0)`.
/// Success clears `condition_value`.
#[must_use]
pub const fn auto_verify_wait_delay(view: &AutoVerifyWaitDelayView) -> AutoVerifyWaitDelay {
    let gate = if view.condition_value > 0 {
        view.condition_value as u32
    } else {
        0
    };
    let complete = view.elapsed_ms > gate;
    AutoVerifyWaitDelay {
        complete,
        cleared: complete,
    }
}

/// Vehicle view [`auto_verify_within_distance`] reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoVerifyWithinDistanceView {
    /// `wp_distance_m()`.
    pub wp_distance_m: f32,
    /// `condition_value` (metres, may be negative).
    pub condition_value: f32,
}

impl AutoVerifyWithinDistanceView {
    /// Still outside the gate.
    #[must_use]
    pub const fn outside() -> Self {
        Self {
            wp_distance_m: 12.0,
            condition_value: 10.0,
        }
    }

    /// Inside the gate.
    #[must_use]
    pub const fn inside() -> Self {
        Self {
            wp_distance_m: 9.0,
            condition_value: 10.0,
        }
    }
}

/// Leftover of one `ModeAuto::verify_within_distance` call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoVerifyWithinDistance {
    /// What `verify_within_distance` returned.
    pub complete: bool,
    /// `condition_value` was cleared.
    pub cleared: bool,
}

/// Upstream `ModeAuto::verify_within_distance`.
///
/// Complete when `wp_distance_m < MAX(condition_value, 0)`. Success
/// clears `condition_value`.
#[must_use]
pub const fn auto_verify_within_distance(
    view: &AutoVerifyWithinDistanceView,
) -> AutoVerifyWithinDistance {
    let gate = if view.condition_value > 0.0 {
        view.condition_value
    } else {
        0.0
    };
    let complete = view.wp_distance_m < gate;
    AutoVerifyWithinDistance {
        complete,
        cleared: complete,
    }
}

/// Vehicle view [`auto_verify_yaw`] reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoVerifyYawView {
    /// `auto_yaw.mode()` at the top of the call — leftover forces FIXED.
    pub mode: YawMode,
    /// `_fixed_yaw_offset_rad`.
    pub fixed_yaw_offset_rad: f32,
    /// Target yaw, rad.
    pub yaw_angle_rad: f32,
    /// Measured yaw, rad.
    pub measured_yaw_rad: f32,
}

impl AutoVerifyYawView {
    /// Already in FIXED and on the heading.
    #[must_use]
    pub const fn arrived() -> Self {
        Self {
            mode: YawMode::Hold,
            fixed_yaw_offset_rad: 0.0,
            yaw_angle_rad: 0.5,
            measured_yaw_rad: 0.5,
        }
    }
}

/// Leftover of one `ModeAuto::verify_yaw` call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoVerifyYaw {
    /// `auto_yaw.set_mode(FIXED)` always ran.
    pub set_fixed: bool,
    /// `reached_fixed_yaw_target` after that set.
    pub complete: bool,
}

/// Upstream `ModeAuto::verify_yaw`.
///
/// Forces FIXED first — wpnav often steals the axis — then asks
/// whether the slew has arrived.
#[must_use]
pub fn auto_verify_yaw(view: &AutoVerifyYawView) -> AutoVerifyYaw {
    AutoVerifyYaw {
        set_fixed: true,
        complete: reached_fixed_yaw_target(
            YawMode::Fixed,
            view.fixed_yaw_offset_rad,
            view.yaw_angle_rad,
            view.measured_yaw_rad,
        ),
    }
}

/// Vehicle view [`auto_verify_circle`] reads.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoVerifyCircleView {
    /// `_mode` at the top of the call.
    pub submode: AutoSubMode,
    /// `wp_nav->reached_wp_destination()`.
    pub reached_wp: bool,
    /// `auto_yaw.mode() == ROI` — used if `circle_start` runs.
    pub auto_yaw_is_roi: bool,
    /// `cmd.p1` for [`loiter_turns`].
    pub p1: u16,
    /// Type-specific bit 1 — fractional turns.
    pub fractional_turns: bool,
    /// `fabsf(circle_nav->get_angle_total_rad() / M_2PI)`.
    pub num_circles: f32,
    /// `circle_last_num_complete` at the top of the call.
    pub last_num_complete: f32,
}

impl AutoVerifyCircleView {
    /// Still flying to the edge.
    #[must_use]
    pub const fn moving_to_edge() -> Self {
        Self {
            submode: AutoSubMode::CircleMoveToEdge,
            reached_wp: false,
            auto_yaw_is_roi: false,
            p1: 0x0200,
            fractional_turns: false,
            num_circles: 0.0,
            last_num_complete: -1.0,
        }
    }

    /// Circling, one of two turns done.
    #[must_use]
    pub const fn circling() -> Self {
        Self {
            submode: AutoSubMode::Circle,
            reached_wp: true,
            auto_yaw_is_roi: false,
            p1: 0x0002,
            fractional_turns: false,
            num_circles: 1.0,
            last_num_complete: 0.0,
        }
    }
}

/// Leftover of one `ModeAuto::verify_circle` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutoVerifyCircle {
    /// What `verify_circle` returned.
    pub complete: bool,
    /// `circle_start()` ran (edge arrived).
    pub circle_start: bool,
    /// Yaw leftover when `circle_start` ran.
    pub yaw: AutoCircleYaw,
    /// `_mode` after the leftover.
    pub submode: AutoSubMode,
    /// `"Mission: starting circle %u/%u"` ran.
    pub starting_circle_text: bool,
    /// `circle_last_num_complete` after the leftover.
    pub last_num_complete: f32,
}

/// Truncate toward zero the way C++ `int(float)` does.
#[must_use]
const fn int_trunc(v: f32) -> i32 {
    v as i32
}

/// Upstream `ModeAuto::verify_circle`.
///
/// `CIRCLE_MOVE_TO_EDGE` waits for the dest, then `circle_start`, and
/// never completes on that tick. Once circling, complete is
/// `num_circles >= turns`.
#[must_use]
pub const fn auto_verify_circle(view: &AutoVerifyCircleView) -> AutoVerifyCircle {
    if matches!(view.submode, AutoSubMode::CircleMoveToEdge) {
        let start = view.reached_wp;
        return AutoVerifyCircle {
            complete: false,
            circle_start: start,
            yaw: if start {
                circle_start_yaw(view.auto_yaw_is_roi)
            } else {
                AutoCircleYaw::Unchanged
            },
            submode: if start {
                AutoSubMode::Circle
            } else {
                AutoSubMode::CircleMoveToEdge
            },
            starting_circle_text: false,
            last_num_complete: view.last_num_complete,
        };
    }

    let turns = loiter_turns(view.p1, view.fractional_turns);
    let starting_circle_text =
        int_trunc(view.num_circles) != int_trunc(view.last_num_complete);
    AutoVerifyCircle {
        complete: view.num_circles >= turns,
        circle_start: false,
        yaw: AutoCircleYaw::Unchanged,
        submode: view.submode,
        starting_circle_text,
        last_num_complete: if starting_circle_text {
            view.num_circles
        } else {
            view.last_num_complete
        },
    }
}

/// Vehicle view [`auto_verify_nav_delay`] reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoVerifyNavDelayView {
    /// `millis() - nav_delay_time_start_ms`.
    pub elapsed_ms: u32,
    /// `nav_delay_time_max_ms`.
    pub max_ms: u32,
}

impl AutoVerifyNavDelayView {
    /// Still delaying.
    #[must_use]
    pub const fn waiting() -> Self {
        Self {
            elapsed_ms: 1000,
            max_ms: 5000,
        }
    }

    /// Delay has run out.
    #[must_use]
    pub const fn done() -> Self {
        Self {
            elapsed_ms: 5001,
            max_ms: 5000,
        }
    }
}

/// Leftover of one `ModeAuto::verify_nav_delay` call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoVerifyNavDelay {
    /// What `verify_nav_delay` returned.
    pub complete: bool,
    /// `nav_delay_time_max_ms` was cleared.
    pub cleared: bool,
}

/// Upstream `ModeAuto::verify_nav_delay`.
///
/// Complete when elapsed is *greater than* max. Success clears max.
#[must_use]
pub const fn auto_verify_nav_delay(view: &AutoVerifyNavDelayView) -> AutoVerifyNavDelay {
    let complete = view.elapsed_ms > view.max_ms;
    AutoVerifyNavDelay {
        complete,
        cleared: complete,
    }
}

/// Vehicle view [`auto_verify_nav_guided_enable`] reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoVerifyNavGuidedView {
    /// `cmd.p1`.
    pub p1: u16,
    /// `mode_guided.limit_check()`.
    pub limit_check: bool,
}

/// Upstream `ModeAuto::verify_nav_guided_enable`.
///
/// `p1 == 0` is an immediate complete (disable). Otherwise the guided
/// limit check is the leftover.
#[must_use]
pub const fn auto_verify_nav_guided_enable(view: &AutoVerifyNavGuidedView) -> bool {
    view.p1 == 0 || view.limit_check
}

/// Vehicle view [`auto_verify_nav_script_time`] reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutoVerifyNavScriptTimeView {
    /// `nav_scripting.done`.
    pub done: bool,
    /// `nav_scripting.timeout_s`.
    pub timeout_s: u16,
    /// `millis() - nav_scripting.start_ms`.
    pub elapsed_ms: u32,
}

/// Upstream `ModeAuto::verify_nav_script_time`.
#[must_use]
pub const fn auto_verify_nav_script_time(view: &AutoVerifyNavScriptTimeView) -> bool {
    if view.done {
        return true;
    }
    view.timeout_s > 0 && view.elapsed_ms > (view.timeout_s as u32).saturating_mul(1000)
}

/// Upstream `ModeAuto::verify_nav_attitude_time`.
#[must_use]
pub const fn auto_verify_nav_attitude_time(elapsed_ms: u32, time_sec: u16) -> bool {
    elapsed_ms > (time_sec as u32).saturating_mul(1000)
}

/// Upstream `PayloadPlace::verify`.
#[must_use]
pub const fn auto_verify_payload_place(state: PayloadPlaceState) -> bool {
    matches!(state, PayloadPlaceState::Done)
}
