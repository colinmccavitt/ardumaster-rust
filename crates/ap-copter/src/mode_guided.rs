//! `ModeGuided` init / set_destination / run / set_velocity /
//! pos_control_run / set_angle / angle_control_run /
//! velaccel_control_run leftover, upstream
//! `ArduCopter/mode_guided.cpp`.
//!
//! Tracked as **COP-017**. Guided is the GCS / scripting command mode: fly
//! to a coordinate, hold a velocity, or take an attitude. The first slice
//! owns the enter and the Location dest setter. The second owns the `run`
//! dispatcher and the velocity setter. The third owns `pos_control_run`.
//! The fourth owns `set_angle` (and `angle_control_start`). The fifth owns
//! `angle_control_run` and Guided-NoGPS. This slice owns
//! `velaccel_control_run`. ModeFollow init is the companion in
//! `mode_follow`. `accel_control_run`, `posvelaccel_control_run`, the
//! accel setter, and Follow `run` are later slices.
//!
//! Upstream names the enter `init`, not `_enter`. Plane modes use `_enter`;
//! Copter modes use `init`. This is that enter.
//!
//! # Init always parks in VelAccel and always succeeds
//!
//! `ignore_checks` is unused. Guided does not refuse a missing home or a
//! missing plan — a GCS that has not yet sent a dest still needs the
//! mode to exist so the first command can land. `init` starts
//! `velaccel_control_start` (submode [`GuidedSubMode::VelAccel`], then
//! `pva_control_start`), zeroes the velocity and acceleration targets,
//! clears `send_notification`, and clears `_paused`. A leftover pause
//! from a previous visit must not freeze the vehicle the moment Guided
//! is re-entered. The position-controller leftover is the waypoint
//! navigator's default NE / D limits, both controllers re-inited, yaw
//! `set_mode_to_default(false)`, and `guided_is_terrain_alt = false`.
//!
//! # `set_destination` is a fence, then a fork
//!
//! When the fence library is compiled in, a dest outside the fence is a
//! NAK (`DEST_OUTSIDE_FENCE`) and nothing else runs. `GUID_OPTIONS` bit
//! 6 (`WPNavUsedForPosControl`) then picks the path.
//!
//! The wpnav path starts WP if the submode is not already WP
//! (`wp_and_spline_init_m`, stopping-point dest, default yaw). A failed
//! `set_wp_destination_loc` is `FAILED_TO_SET_DESTINATION` — the WP
//! start, if it ran, stays. Yaw and `send_notification` only run after
//! the dest is accepted.
//!
//! The position-controller path converts the Location through
//! `wp_nav->get_vector_NED_m` *before* switching to Pos. A conversion
//! failure leaves the submode alone. Success starts Pos if needed
//! (`pva_control_start` again), then `set_yaw_state_rad`. Terrain-frame
//! dests that cannot read `get_terrain_D_m` call `hold_position` (back
//! to VelAccel, zero vel/accel) and return false; yaw has already been
//! written. A non-terrain dest always `init_pos_terrain_D_m(0)`. A
//! terrain dest only does that when the previous dest was not terrain.
//! Success zeroes vel/accel, stamps `update_time_ms`, and sets
//! `send_notification`.
//!
//! # Yaw is a five-way leftover, not a heading
//!
//! `set_yaw_state_rad` picks an AutoYaw setter. `use_yaw && relative`
//! wins even when a rate was also supplied — the relative path is
//! `set_fixed_yaw_rad` with a zero rate. That order is the leftover.
//!
//! # `run` is a pause gate, then a seven-way leftover
//!
//! `_paused` short-circuits into `pause_control_run` and never looks at
//! the submode. A leftover pause therefore freezes the vehicle even
//! when a dest was just accepted. The unpaused leftover is the switch:
//! each `SubMode` calls one `*_run` body. Those bodies are later
//! slices; this leftover records which one was chosen.
//!
//! WP is the only arm with a side-effect. `send_notification` (set by
//! `set_destination`) is cleared and a mission-item-reached GCS
//! message is sent only when the waypoint navigator reports arrival.
//! A dest that has not been reached keeps the flag; a tick that never
//! set it never sends.
//!
//! # `set_vel_NED_ms` is zero accel, then `set_vel_accel_NED_m`
//!
//! The thin setter does not have its own leftover. It forwards a
//! zero acceleration. `set_vel_accel_NED_m` starts VelAccel if the
//! submode is not already there (`velaccel_control_start` →
//! `pva_control_start`), then `set_yaw_state_rad`, then zeroes the
//! position target and the terrain flag, stores the velocity and
//! acceleration, and stamps `update_time_ms`. A log write is compiled
//! out when logging is off, and skipped when the caller passed
//! `log_request = false`.
//!
//! # `pos_control_run` is two early exits, then a fly leftover
//!
//! `is_disarmed_or_landed` returns before terrain is consulted — a
//! landed vehicle never fires `failsafe_terrain_on_event`. The
//! `make_safe_ground_handling` argument is `tradheli && interlock`
//! only; a tradheli with motor interlock stays spooled.
//!
//! A terrain dest that cannot read `get_terrain_D_m` fires terrain
//! failsafe and returns without spooling or zeroing vel/accel. A
//! non-terrain dest never asks. The fly path always writes
//! `THROTTLE_UNLIMITED`, zeroes the guided vel/accel leftovers, and
//! may `auto_yaw.set_mode(HOLD)` when the dest is stale *and* yaw
//! is `RATE` or `ANGLE_RATE`. The timeout is `>` not `>=`, and
//! `get_timeout_ms` floors `GUID_TIMEOUT` at 0.1 s.
//!
//! Terrain margin is zero unless the dest is terrain, then
//! `MIN(wp_nav margin, 0.5 * |pos.z|)`. `input_pos_NED_m` gets that
//! margin and the terrain D (zero when not terrain). Controllers
//! and `input_thrust_vector_heading` always run on the fly path.
//!
//! # `velaccel_control_run` is one early exit, then a timeout, then a stab leftover
//!
//! `is_disarmed_or_landed` is the only early return — there is no terrain
//! gate. The `make_safe_ground_handling` argument is the same tradheli
//! interlock keep as Pos.
//!
//! The fly path always writes `THROTTLE_UNLIMITED`. Timeout (wrapping
//! unsigned subtract, `>` not `>=`) zeroes the guided vel *and* accel
//! leftovers and may `auto_yaw.set_mode(HOLD)` when yaw is `RATE` or
//! `ANGLE_RATE`. Unlike Pos, a fresh dest does not zero vel/accel — only
//! a stale one does.
//!
//! Avoidance is compiled out when `AP_AVOIDANCE_ENABLED` is off; this
//! leftover treats `adjust_velocity_NED_m` as identity and records that
//! it ran. `do_avoid` is `limits_active()` after that call, or false
//! when compiled out. A live avoid blocks both the desired-vel overwrite
//! and the stop-stabilisation.
//!
//! `!stabilizing_vel_NE && !do_avoid` overwrites the NE vel leftover
//! from `get_vel_desired_NED_ms` and then `NE_stop_vel_stabilisation`.
//! Else `!stabilizing_pos_NE && !do_avoid` only stops position
//! stabilisation. D always takes the (possibly zeroed) vel/accel z.

//! # `set_angle` starts Angle, then forks thrust vs climb
//!
//! Not already Angle calls `angle_control_start`: D limits only, D init
//! only when inactive, NE never. The start seeds a yaw-only quat that
//! `set_angle` immediately overwrites. Already Angle and switching from
//! thrust to climb is the other D-init path — an `else if`, so a start
//! that also happens to be a thrust-to-climb does not take it. Climb to
//! thrust does not re-init D.
//!
//! The unused vertical channel is zeroed. `to_euler` always runs; the
//! log write is compiled out when logging is off. There is no
//! `log_request` gate (unlike the velocity setter).
//!
//! # `angle_control_run` constrains climb, then a timeout, then three exits
//!
//! Thrust ticks never constrain or ask avoidance — `climb_rate_ms` stays
//! zero. Climb ticks constrain to `[-speed_down, speed_up]` (NaN becomes
//! the midpoint) and then call avoidance; this leftover treats avoidance
//! as identity and records that it ran.
//!
//! Timeout is wrapping unsigned subtract and `>` not `>=`, same as Pos.
//! A stale tick reseeds a yaw-only quat, zeroes body rates, and zeroes
//! climb. A stale *thrust* tick also `D_init_controller` and clears
//! `use_thrust` — so `is_positive` after a timeout always sees climb
//! zero, never the leftover thrust.
//!
//! `is_positive` is `>= FLT_EPSILON`, not `> 0`. Armed and positive
//! writes `auto_armed` *before* the exits, so a first positive command
//! can take off without a prior auto-arm. Ground is `!armed ||
//! !auto_armed || (landed && !positive)` with the same tradheli
//! interlock keep as Pos. Landed and positive is takeoff: relax,
//! unlimited spool, and `set_land_complete(false)` plus D-init only
//! when the spool is already unlimited. The fly path is a zero-quat
//! leftover: all-zero takes rate input, anything else takes the quat.

use ap_math::quaternion::Quaternion;
use ap_math::scalar::{constrain_value, is_positive};

/// `Mode::Number::GUIDED`.
pub const MODE_NUMBER_GUIDED: u8 = 4;

/// `Mode::Number::GUIDED_NOGPS` — later slice; the number is pinned here
/// so Guided and Guided-NoGPS stay in one catalog.
pub const MODE_NUMBER_GUIDED_NOGPS: u8 = 20;

/// `Mode::Number::FOLLOW` — init leftover lives in `mode_follow`.
pub const MODE_NUMBER_FOLLOW: u8 = 23;

/// `WP_SPD_DEFAULT` — `pva_control_start` NE speed when the caller has
/// not overridden `WP_SPD`.
pub const WP_SPD_DEFAULT_MS: f32 = 10.0;

/// `WPNAV_ACCELERATION_MS` — `pva_control_start` NE accel default.
pub const WPNAV_ACCELERATION_MSS: f32 = 2.5;

/// `WP_SPD_DOWN_DEFAULT` — `pva_control_start` descent default.
pub const WP_SPD_DOWN_DEFAULT_MS: f32 = 1.5;

/// `WP_SPD_UP_DEFAULT` — `pva_control_start` climb default.
pub const WP_SPD_UP_DEFAULT_MS: f32 = 2.5;

/// `WP_ACC_Z_DEFAULT` — `pva_control_start` vertical accel default.
pub const WP_ACC_Z_DEFAULT_MSS: f32 = 1.0;

/// `ModeGuided` capability flags from `mode.h`.
///
/// These are not computed. They are the leftover catalog of what the
/// class reports to `set_mode` and the arming checks. `allows_arming`
/// is not here: it depends on the arming method and
/// [`GuidedOption::AllowArmingFromTx`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GuidedModeFlags {
    /// `mode_number()`.
    pub mode_number: u8,
    /// `requires_position()`.
    pub requires_position: bool,
    /// `has_manual_throttle()`.
    pub has_manual_throttle: bool,
    /// `is_autopilot()`.
    pub is_autopilot: bool,
    /// `has_user_takeoff(...)` — Guided always allows a user takeoff.
    pub has_user_takeoff: bool,
    /// `in_guided_mode()`.
    pub in_guided_mode: bool,
    /// `requires_terrain_failsafe()`.
    pub requires_terrain_failsafe: bool,
    /// `allows_GCS_or_SCR_arming_with_throttle_high()`.
    pub allows_gcs_or_scr_arming_with_throttle_high: bool,
}

/// Upstream `ModeGuided` flags.
#[must_use]
pub const fn guided_mode_flags() -> GuidedModeFlags {
    GuidedModeFlags {
        mode_number: MODE_NUMBER_GUIDED,
        requires_position: true,
        has_manual_throttle: false,
        is_autopilot: true,
        has_user_takeoff: true,
        in_guided_mode: true,
        requires_terrain_failsafe: true,
        allows_gcs_or_scr_arming_with_throttle_high: true,
    }
}

/// `ModeGuided::SubMode`. Discriminants match the untyped enum in `mode.h`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GuidedSubMode {
    /// `SubMode::TakeOff`.
    TakeOff = 0,
    /// `SubMode::WP`.
    Wp = 1,
    /// `SubMode::Pos`.
    Pos = 2,
    /// `SubMode::PosVelAccel`.
    PosVelAccel = 3,
    /// `SubMode::VelAccel` — what `init` selects.
    VelAccel = 4,
    /// `SubMode::Accel`.
    Accel = 5,
    /// `SubMode::Angle`.
    Angle = 6,
}

/// `ModeGuided::Option` bits from `GUID_OPTIONS`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum GuidedOption {
    /// Bit 0 — arm from the transmitter.
    AllowArmingFromTx = 1 << 0,
    /// Bit 2 — ignore the pilot yaw stick. Bit 1 is unused on purpose.
    IgnorePilotYaw = 1 << 2,
    /// Bit 3 — `SET_ATTITUDE_TARGET` thrust is thrust, not climb rate.
    SetAttitudeTargetThrustAsThrust = 1 << 3,
    /// Bit 4 — do not stabilize NE position.
    DoNotStabilizePositionXy = 1 << 4,
    /// Bit 5 — do not stabilize NE velocity.
    DoNotStabilizeVelocityXy = 1 << 5,
    /// Bit 6 — `set_destination` uses wpnav instead of PosControl.
    WpNavUsedForPosControl = 1 << 6,
    /// Bit 7 — allow weathervaning.
    AllowWeatherVaning = 1 << 7,
}

/// Upstream `ModeGuided::option_is_enabled`.
#[must_use]
pub const fn option_is_enabled(guided_options: u32, option: GuidedOption) -> bool {
    guided_options & (option as u32) != 0
}

/// Upstream `ModeGuided::use_wpnav_for_position_control`.
#[must_use]
pub const fn use_wpnav_for_position_control(guided_options: u32) -> bool {
    option_is_enabled(guided_options, GuidedOption::WpNavUsedForPosControl)
}

/// Upstream `ModeGuided::stabilizing_pos_NE`.
///
/// Bit 4 (`DoNotStabilizePositionXy`) inverted. Default GUID_OPTIONS is
/// zero, so position is stabilised unless a GCS cleared the bit.
#[must_use]
pub const fn stabilizing_pos_ne(guided_options: u32) -> bool {
    !option_is_enabled(guided_options, GuidedOption::DoNotStabilizePositionXy)
}

/// Upstream `ModeGuided::stabilizing_vel_NE`.
///
/// Bit 5 (`DoNotStabilizeVelocityXy`) inverted. `velaccel_control_run`
/// consults this *and* `do_avoid` before overwriting the NE vel leftover.
#[must_use]
pub const fn stabilizing_vel_ne(guided_options: u32) -> bool {
    !option_is_enabled(guided_options, GuidedOption::DoNotStabilizeVelocityXy)
}

/// AutoYaw setter `set_yaw_state_rad` would call.
///
/// The leftover does not run AutoYaw. It records which setter the
/// five-way ladder selected. [`GuidedYawAction::NotCalled`] means the
/// dest setter returned before the ladder.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GuidedYawAction {
    /// `set_yaw_state_rad` was not reached.
    NotCalled,
    /// `auto_yaw.set_fixed_yaw_rad(yaw, 0, 0, relative)`.
    SetFixedYaw {
        /// Commanded yaw, radians.
        yaw_rad: f32,
        /// `relative_angle` passed through. Always true on this arm.
        relative: bool,
    },
    /// `auto_yaw.set_yaw_angle_and_rate_rad`.
    SetAngleAndRate {
        /// Commanded yaw, radians.
        yaw_rad: f32,
        /// Commanded rate, rad/s. Zero when `use_yaw && !use_yaw_rate`.
        yaw_rate_rads: f32,
    },
    /// `auto_yaw.set_rate_rad`.
    SetRate {
        /// Commanded rate, rad/s.
        yaw_rate_rads: f32,
    },
    /// `auto_yaw.set_mode_to_default(false)`.
    SetModeToDefault,
}

/// Upstream `ModeGuided::set_yaw_state_rad`.
///
/// `use_yaw && relative` wins over a simultaneous rate. The relative
/// setter is `set_fixed_yaw_rad`, not `set_yaw_angle_and_rate_rad`.
#[must_use]
pub fn set_yaw_state_rad(
    use_yaw: bool,
    yaw_rad: f32,
    use_yaw_rate: bool,
    yaw_rate_rads: f32,
    relative_angle: bool,
) -> GuidedYawAction {
    if use_yaw && relative_angle {
        GuidedYawAction::SetFixedYaw {
            yaw_rad,
            relative: relative_angle,
        }
    } else if use_yaw && use_yaw_rate {
        GuidedYawAction::SetAngleAndRate {
            yaw_rad,
            yaw_rate_rads,
        }
    } else if use_yaw && !use_yaw_rate {
        GuidedYawAction::SetAngleAndRate {
            yaw_rad,
            yaw_rate_rads: 0.0,
        }
    } else if use_yaw_rate {
        GuidedYawAction::SetRate { yaw_rate_rads }
    } else {
        GuidedYawAction::SetModeToDefault
    }
}

/// Vehicle view `ModeGuided::init` reads.
///
/// `ignore_checks` is accepted on the function, not here: the leftover
/// ignores it. The numbers are the waypoint navigator defaults
/// `pva_control_start` writes into PosControl.
#[derive(Debug, Clone, Copy)]
pub struct GuidedInitView {
    /// `wp_nav->get_default_speed_NE_ms()`.
    pub default_speed_ne_ms: f32,
    /// `wp_nav->get_wp_acceleration_mss()`.
    pub wp_acceleration_mss: f32,
    /// `wp_nav->get_default_speed_down_ms()`.
    pub default_speed_down_ms: f32,
    /// `wp_nav->get_default_speed_up_ms()`.
    pub default_speed_up_ms: f32,
    /// `wp_nav->get_accel_D_mss()`.
    pub accel_d_mss: f32,
}

impl GuidedInitView {
    /// Parameter defaults from `AC_WPNav`.
    #[must_use]
    pub const fn typical() -> Self {
        Self {
            default_speed_ne_ms: WP_SPD_DEFAULT_MS,
            wp_acceleration_mss: WPNAV_ACCELERATION_MSS,
            default_speed_down_ms: WP_SPD_DOWN_DEFAULT_MS,
            default_speed_up_ms: WP_SPD_UP_DEFAULT_MS,
            accel_d_mss: WP_ACC_Z_DEFAULT_MSS,
        }
    }
}

/// Leftover of one `ModeGuided::init` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GuidedInit {
    /// Always true. `ignore_checks` is unused.
    pub ok: bool,
    /// Always [`GuidedSubMode::VelAccel`].
    pub submode: GuidedSubMode,
    /// Horizontal max / correction speed from the wpnav default.
    pub ne_speed_ms: f32,
    /// Horizontal max / correction accel from the wpnav default.
    pub ne_accel_mss: f32,
    /// Vertical max / correction descent speed.
    pub d_speed_down_ms: f32,
    /// Vertical max / correction climb speed.
    pub d_speed_up_ms: f32,
    /// Vertical max / correction accel.
    pub d_accel_mss: f32,
    /// Always true: `NE_init_controller`.
    pub init_ne: bool,
    /// Always true: `D_init_controller`. Unlike Brake, Guided always
    /// re-inits both axes — `pva_control_start` does not ask `D_is_active`.
    pub init_d: bool,
    /// `auto_yaw.set_mode_to_default(false)`.
    pub yaw: GuidedYawAction,
    /// `guided_is_terrain_alt` after init. Always false.
    pub terrain_alt: bool,
    /// Velocity target was zeroed.
    pub vel_zero: bool,
    /// Acceleration target was zeroed.
    pub accel_zero: bool,
    /// `send_notification` after init. Always false.
    pub send_notification: bool,
    /// `_paused` after init. Always false.
    pub paused: bool,
}

/// Upstream `ModeGuided::init`.
///
/// `ignore_checks` is accepted and ignored, matching the unused parameter.
#[must_use]
pub fn guided_init(view: &GuidedInitView, _ignore_checks: bool) -> GuidedInit {
    GuidedInit {
        ok: true,
        submode: GuidedSubMode::VelAccel,
        ne_speed_ms: view.default_speed_ne_ms,
        ne_accel_mss: view.wp_acceleration_mss,
        d_speed_down_ms: view.default_speed_down_ms,
        d_speed_up_ms: view.default_speed_up_ms,
        d_accel_mss: view.accel_d_mss,
        init_ne: true,
        init_d: true,
        yaw: GuidedYawAction::SetModeToDefault,
        terrain_alt: false,
        vel_zero: true,
        accel_zero: true,
        send_notification: false,
        paused: false,
    }
}

/// Why `set_destination` returned false, or success.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GuidedSetDestFail {
    /// The dest was accepted.
    None,
    /// Fence compiled in and `check_destination_within_fence` refused.
    OutsideFence,
    /// `wp_nav->set_wp_destination_loc` refused (missing terrain).
    FailedToSetWpDestination,
    /// `wp_nav->get_vector_NED_m` refused.
    MissingVectorNed,
    /// Terrain-frame dest and `get_terrain_D_m` refused.
    MissingTerrainAlt,
}

/// Vehicle view `ModeGuided::set_destination` reads.
#[derive(Debug, Clone, Copy)]
pub struct GuidedSetDestView {
    /// `guided_mode` before the call.
    pub submode: GuidedSubMode,
    /// `guided_is_terrain_alt` before the call.
    pub guided_is_terrain_alt: bool,
    /// `AP_FENCE_ENABLED`. When false the fence check is compiled out.
    pub fence_enabled: bool,
    /// `copter.fence.check_destination_within_fence(dest_loc)`.
    pub within_fence: bool,
    /// `use_wpnav_for_position_control()`.
    pub use_wpnav: bool,
    /// `wp_nav->set_wp_destination_loc(dest_loc)` — wpnav path only.
    pub wp_dest_ok: bool,
    /// `wp_nav->get_vector_NED_m` — position-controller path only.
    pub vector_ned_ok: bool,
    /// NED dest from `get_vector_NED_m`, metres.
    pub pos_target_ned_m: [f32; 3],
    /// Terrain-frame flag from `get_vector_NED_m`.
    pub is_terrain_alt: bool,
    /// `wp_nav->get_terrain_D_m` — only consulted on a terrain dest.
    pub terrain_d_ok: bool,
    /// Terrain D offset, metres, when [`Self::terrain_d_ok`].
    pub terrain_d_m: f32,
    /// `millis()` stamped into `update_time_ms` on the Pos success path.
    pub now_ms: u32,
    /// `use_yaw` argument.
    pub use_yaw: bool,
    /// `yaw_rad` argument.
    pub yaw_rad: f32,
    /// `use_yaw_rate` argument.
    pub use_yaw_rate: bool,
    /// `yaw_rate_rads` argument.
    pub yaw_rate_rads: f32,
    /// `relative_yaw` argument.
    pub relative_yaw: bool,
}

impl GuidedSetDestView {
    /// After `init`: VelAccel, no terrain, fence on and inside, Pos path.
    #[must_use]
    pub const fn after_init() -> Self {
        Self {
            submode: GuidedSubMode::VelAccel,
            guided_is_terrain_alt: false,
            fence_enabled: true,
            within_fence: true,
            use_wpnav: false,
            wp_dest_ok: true,
            vector_ned_ok: true,
            pos_target_ned_m: [20.0, 10.0, -15.0],
            is_terrain_alt: false,
            terrain_d_ok: true,
            terrain_d_m: 0.0,
            now_ms: 1_000,
            use_yaw: false,
            yaw_rad: 0.0,
            use_yaw_rate: false,
            yaw_rate_rads: 0.0,
            relative_yaw: false,
        }
    }
}

/// Leftover of one `ModeGuided::set_destination` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GuidedSetDest {
    /// What `set_destination` returned.
    pub ok: bool,
    /// Why it failed, or [`GuidedSetDestFail::None`].
    pub fail: GuidedSetDestFail,
    /// `guided_mode` after the call.
    pub submode: GuidedSubMode,
    /// `wp_control_start` ran.
    pub started_wp: bool,
    /// `pos_control_start` ran.
    pub started_pos: bool,
    /// `hold_position` / `velaccel_control_start` ran.
    pub held_position: bool,
    /// `wp_nav->wp_and_spline_init_m()` ran (only on a WP start).
    pub wp_and_spline_init: bool,
    /// `set_yaw_state_rad` leftover. [`GuidedYawAction::NotCalled`]
    /// when the function returned before the ladder.
    pub yaw: GuidedYawAction,
    /// `init_pos_terrain_D_m` argument, if that call ran.
    pub init_pos_terrain_d_m: Option<f32>,
    /// `guided_pos_target_ned_m` after a Pos-path success.
    pub pos_target_ned_m: Option<[f32; 3]>,
    /// `guided_is_terrain_alt` after the call.
    pub terrain_alt: bool,
    /// Velocity target was zeroed on the success or hold path.
    pub vel_zero: bool,
    /// Acceleration target was zeroed on the success or hold path.
    pub accel_zero: bool,
    /// `update_time_ms` after a Pos-path success.
    pub update_time_ms: Option<u32>,
    /// `send_notification` after the call.
    pub send_notification: bool,
    /// `LOGGER_WRITE_ERROR(..., DEST_OUTSIDE_FENCE)`.
    pub log_dest_outside_fence: bool,
    /// `LOGGER_WRITE_ERROR(..., FAILED_TO_SET_DESTINATION)`.
    pub log_failed_to_set_destination: bool,
}

/// Upstream `ModeGuided::set_destination`.
#[must_use]
pub fn guided_set_destination(view: &GuidedSetDestView) -> GuidedSetDest {
    if view.fence_enabled && !view.within_fence {
        return GuidedSetDest {
            ok: false,
            fail: GuidedSetDestFail::OutsideFence,
            submode: view.submode,
            started_wp: false,
            started_pos: false,
            held_position: false,
            wp_and_spline_init: false,
            yaw: GuidedYawAction::NotCalled,
            init_pos_terrain_d_m: None,
            pos_target_ned_m: None,
            terrain_alt: view.guided_is_terrain_alt,
            vel_zero: false,
            accel_zero: false,
            update_time_ms: None,
            send_notification: false,
            log_dest_outside_fence: true,
            log_failed_to_set_destination: false,
        };
    }

    if view.use_wpnav {
        let started_wp = view.submode != GuidedSubMode::Wp;
        if !view.wp_dest_ok {
            return GuidedSetDest {
                ok: false,
                fail: GuidedSetDestFail::FailedToSetWpDestination,
                submode: GuidedSubMode::Wp,
                started_wp,
                started_pos: false,
                held_position: false,
                wp_and_spline_init: started_wp,
                yaw: GuidedYawAction::NotCalled,
                init_pos_terrain_d_m: None,
                pos_target_ned_m: None,
                terrain_alt: view.guided_is_terrain_alt,
                vel_zero: false,
                accel_zero: false,
                update_time_ms: None,
                send_notification: false,
                log_dest_outside_fence: false,
                log_failed_to_set_destination: true,
            };
        }
        return GuidedSetDest {
            ok: true,
            fail: GuidedSetDestFail::None,
            submode: GuidedSubMode::Wp,
            started_wp,
            started_pos: false,
            held_position: false,
            wp_and_spline_init: started_wp,
            yaw: set_yaw_state_rad(
                view.use_yaw,
                view.yaw_rad,
                view.use_yaw_rate,
                view.yaw_rate_rads,
                view.relative_yaw,
            ),
            init_pos_terrain_d_m: None,
            pos_target_ned_m: None,
            terrain_alt: view.guided_is_terrain_alt,
            vel_zero: false,
            accel_zero: false,
            update_time_ms: None,
            send_notification: true,
            log_dest_outside_fence: false,
            log_failed_to_set_destination: false,
        };
    }

    if !view.vector_ned_ok {
        return GuidedSetDest {
            ok: false,
            fail: GuidedSetDestFail::MissingVectorNed,
            submode: view.submode,
            started_wp: false,
            started_pos: false,
            held_position: false,
            wp_and_spline_init: false,
            yaw: GuidedYawAction::NotCalled,
            init_pos_terrain_d_m: None,
            pos_target_ned_m: None,
            terrain_alt: view.guided_is_terrain_alt,
            vel_zero: false,
            accel_zero: false,
            update_time_ms: None,
            send_notification: false,
            log_dest_outside_fence: false,
            log_failed_to_set_destination: false,
        };
    }

    let started_pos = view.submode != GuidedSubMode::Pos;
    let yaw = set_yaw_state_rad(
        view.use_yaw,
        view.yaw_rad,
        view.use_yaw_rate,
        view.yaw_rate_rads,
        view.relative_yaw,
    );

    if view.is_terrain_alt {
        if !view.terrain_d_ok {
            return GuidedSetDest {
                ok: false,
                fail: GuidedSetDestFail::MissingTerrainAlt,
                submode: GuidedSubMode::VelAccel,
                started_wp: false,
                started_pos,
                held_position: true,
                wp_and_spline_init: false,
                yaw,
                init_pos_terrain_d_m: None,
                pos_target_ned_m: None,
                terrain_alt: view.guided_is_terrain_alt,
                vel_zero: true,
                accel_zero: true,
                update_time_ms: None,
                send_notification: false,
                log_dest_outside_fence: false,
                log_failed_to_set_destination: false,
            };
        }
        let init_pos_terrain_d_m = if view.guided_is_terrain_alt {
            None
        } else {
            Some(view.terrain_d_m)
        };
        return GuidedSetDest {
            ok: true,
            fail: GuidedSetDestFail::None,
            submode: GuidedSubMode::Pos,
            started_wp: false,
            started_pos,
            held_position: false,
            wp_and_spline_init: false,
            yaw,
            init_pos_terrain_d_m,
            pos_target_ned_m: Some(view.pos_target_ned_m),
            terrain_alt: true,
            vel_zero: true,
            accel_zero: true,
            update_time_ms: Some(view.now_ms),
            send_notification: true,
            log_dest_outside_fence: false,
            log_failed_to_set_destination: false,
        };
    }

    GuidedSetDest {
        ok: true,
        fail: GuidedSetDestFail::None,
        submode: GuidedSubMode::Pos,
        started_wp: false,
        started_pos,
        held_position: false,
        wp_and_spline_init: false,
        yaw,
        init_pos_terrain_d_m: Some(0.0),
        pos_target_ned_m: Some(view.pos_target_ned_m),
        terrain_alt: false,
        vel_zero: true,
        accel_zero: true,
        update_time_ms: Some(view.now_ms),
        send_notification: true,
        log_dest_outside_fence: false,
        log_failed_to_set_destination: false,
    }
}

/// Which `*_run` body `ModeGuided::run` called this tick.
///
/// The leftover is the *choice*. The bodies themselves (`takeoff_run`,
/// `wp_control_run`, `pause_control_run`, …) are later slices.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GuidedRunBody {
    /// `_paused`: `pause_control_run` and return. The switch is skipped.
    Pause,
    /// `SubMode::TakeOff` → `takeoff_run`.
    TakeOff,
    /// `SubMode::WP` → `wp_control_run` (and maybe a reached-GCS).
    Wp,
    /// `SubMode::Pos` → `pos_control_run` (body is [`guided_pos_control_run`]).
    Pos,
    /// `SubMode::Accel` → `accel_control_run`.
    Accel,
    /// `SubMode::VelAccel` → `velaccel_control_run`.
    VelAccel,
    /// `SubMode::PosVelAccel` → `posvelaccel_control_run`.
    PosVelAccel,
    /// `SubMode::Angle` → `angle_control_run`.
    Angle,
}

/// Vehicle view `ModeGuided::run` reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GuidedRunView {
    /// `_paused` at the top of the tick.
    pub paused: bool,
    /// `guided_mode` at the top of the tick.
    pub submode: GuidedSubMode,
    /// `send_notification` at the top of the tick.
    pub send_notification: bool,
    /// `wp_nav->reached_wp_destination()`. Consulted only on the WP arm.
    pub wp_reached: bool,
}

impl GuidedRunView {
    /// After [`guided_init`]: VelAccel, not paused, no dest notification.
    #[must_use]
    pub const fn after_init() -> Self {
        Self {
            paused: false,
            submode: GuidedSubMode::VelAccel,
            send_notification: false,
            wp_reached: false,
        }
    }

    /// After a wpnav [`guided_set_destination`]: WP, notify armed.
    #[must_use]
    pub const fn after_wp_dest() -> Self {
        Self {
            paused: false,
            submode: GuidedSubMode::Wp,
            send_notification: true,
            wp_reached: false,
        }
    }
}

/// Leftover of one `ModeGuided::run` tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GuidedRun {
    /// Which `*_run` body the pause gate / switch selected.
    pub body: GuidedRunBody,
    /// `send_notification` after the tick.
    pub send_notification: bool,
    /// `gcs().send_mission_item_reached_message(0)` ran.
    pub mission_item_reached: bool,
}

/// Upstream `ModeGuided::run`.
///
/// `_paused` wins over every submode. The WP arm is the only one that
/// writes `send_notification`: it clears the flag and sends a GCS
/// reached message only when the waypoint navigator reports arrival.
#[must_use]
pub const fn guided_run(view: &GuidedRunView) -> GuidedRun {
    if view.paused {
        return GuidedRun {
            body: GuidedRunBody::Pause,
            send_notification: view.send_notification,
            mission_item_reached: false,
        };
    }

    match view.submode {
        GuidedSubMode::TakeOff => GuidedRun {
            body: GuidedRunBody::TakeOff,
            send_notification: view.send_notification,
            mission_item_reached: false,
        },
        GuidedSubMode::Wp => {
            let reached = view.send_notification && view.wp_reached;
            GuidedRun {
                body: GuidedRunBody::Wp,
                send_notification: view.send_notification && !view.wp_reached,
                mission_item_reached: reached,
            }
        }
        GuidedSubMode::Pos => GuidedRun {
            body: GuidedRunBody::Pos,
            send_notification: view.send_notification,
            mission_item_reached: false,
        },
        GuidedSubMode::Accel => GuidedRun {
            body: GuidedRunBody::Accel,
            send_notification: view.send_notification,
            mission_item_reached: false,
        },
        GuidedSubMode::VelAccel => GuidedRun {
            body: GuidedRunBody::VelAccel,
            send_notification: view.send_notification,
            mission_item_reached: false,
        },
        GuidedSubMode::PosVelAccel => GuidedRun {
            body: GuidedRunBody::PosVelAccel,
            send_notification: view.send_notification,
            mission_item_reached: false,
        },
        GuidedSubMode::Angle => GuidedRun {
            body: GuidedRunBody::Angle,
            send_notification: view.send_notification,
            mission_item_reached: false,
        },
    }
}

/// Vehicle view `ModeGuided::set_vel_accel_NED_m` reads.
#[derive(Debug, Clone, Copy)]
pub struct GuidedSetVelView {
    /// `guided_mode` before the call.
    pub submode: GuidedSubMode,
    /// Commanded NED velocity, m/s.
    pub vel_ned_ms: [f32; 3],
    /// Commanded NED acceleration, m/s². [`guided_set_velocity`]
    /// overwrites this with zeroes before the leftover.
    pub accel_ned_mss: [f32; 3],
    /// `millis()` stamped into `update_time_ms`.
    pub now_ms: u32,
    /// `use_yaw` argument.
    pub use_yaw: bool,
    /// `yaw_rad` argument.
    pub yaw_rad: f32,
    /// `use_yaw_rate` argument.
    pub use_yaw_rate: bool,
    /// `yaw_rate_rads` argument.
    pub yaw_rate_rads: f32,
    /// `relative_yaw` argument.
    pub relative_yaw: bool,
    /// `log_request` argument.
    pub log_request: bool,
    /// `HAL_LOGGING_ENABLED`. When false the log write is compiled out.
    pub logging_enabled: bool,
}

impl GuidedSetVelView {
    /// After [`guided_init`]: already VelAccel, logging on, no yaw.
    #[must_use]
    pub const fn after_init() -> Self {
        Self {
            submode: GuidedSubMode::VelAccel,
            vel_ned_ms: [1.5, -0.5, 0.0],
            accel_ned_mss: [0.2, 0.1, 0.0],
            now_ms: 2_000,
            use_yaw: false,
            yaw_rad: 0.0,
            use_yaw_rate: false,
            yaw_rate_rads: 0.0,
            relative_yaw: false,
            log_request: true,
            logging_enabled: true,
        }
    }
}

/// Leftover of one `ModeGuided::set_vel_accel_NED_m` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GuidedSetVel {
    /// Always [`GuidedSubMode::VelAccel`].
    pub submode: GuidedSubMode,
    /// `velaccel_control_start` ran because the submode was not VelAccel.
    pub started_velaccel: bool,
    /// `set_yaw_state_rad` leftover. Always reached.
    pub yaw: GuidedYawAction,
    /// `guided_pos_target_ned_m` after the call. Always zero.
    pub pos_target_ned_m: [f32; 3],
    /// `guided_is_terrain_alt` after the call. Always false.
    pub terrain_alt: bool,
    /// `guided_vel_target_ned_ms` after the call.
    pub vel_ned_ms: [f32; 3],
    /// `guided_accel_target_ned_mss` after the call.
    pub accel_ned_mss: [f32; 3],
    /// `update_time_ms` after the call.
    pub update_time_ms: u32,
    /// `Log_Write_Guided_Position_Target` ran.
    pub logged: bool,
}

/// Shared leftover of `set_vel_accel_NED_m`.
#[must_use]
pub fn guided_set_vel_accel(view: &GuidedSetVelView) -> GuidedSetVel {
    let started_velaccel = view.submode != GuidedSubMode::VelAccel;
    GuidedSetVel {
        submode: GuidedSubMode::VelAccel,
        started_velaccel,
        yaw: set_yaw_state_rad(
            view.use_yaw,
            view.yaw_rad,
            view.use_yaw_rate,
            view.yaw_rate_rads,
            view.relative_yaw,
        ),
        pos_target_ned_m: [0.0, 0.0, 0.0],
        terrain_alt: false,
        vel_ned_ms: view.vel_ned_ms,
        accel_ned_mss: view.accel_ned_mss,
        update_time_ms: view.now_ms,
        logged: view.logging_enabled && view.log_request,
    }
}

/// Upstream `ModeGuided::set_vel_NED_ms`.
///
/// The leftover is the zero acceleration. Everything else is
/// [`guided_set_vel_accel`].
#[must_use]
pub fn guided_set_velocity(view: &GuidedSetVelView) -> GuidedSetVel {
    let mut forwarded = *view;
    forwarded.accel_ned_mss = [0.0, 0.0, 0.0];
    guided_set_vel_accel(&forwarded)
}

/// Floor `GUID_TIMEOUT` at 0.1 s, upstream `get_timeout_ms`.
pub const GUIDED_TIMEOUT_MIN_S: f32 = 0.1;

/// `ParametersG2::guided_timeout` default.
pub const GUIDED_TIMEOUT_DEFAULT_S: f32 = 3.0;

/// Upstream `ModeGuided::get_timeout_ms`.
///
/// `MAX(guided_timeout, 0.1) * 1000` truncated toward zero into ms.
/// A zero or negative param still times out after 100 ms — Guided must
/// not wait forever for a dest that never arrives.
#[must_use]
pub fn guided_timeout_ms(guided_timeout_s: f32) -> u32 {
    let timeout_s = if guided_timeout_s > GUIDED_TIMEOUT_MIN_S {
        guided_timeout_s
    } else {
        GUIDED_TIMEOUT_MIN_S
    };
    (timeout_s * 1000.0) as u32
}

/// How `ModeGuided::pos_control_run` left the tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GuidedPosControlExit {
    /// `is_disarmed_or_landed`: `make_safe_ground_handling` and return.
    /// Terrain is never consulted.
    Disarmed {
        /// `copter.is_tradheli() && motors->get_interlock()`.
        keep_interlock: bool,
    },
    /// Terrain dest and `get_terrain_D_m` failed: `failsafe_terrain_on_event`
    /// and return. Spool / vel / accel / yaw are left alone.
    TerrainFailsafe,
    /// Flew: unlimited spool, zero vel/accel, maybe HOLD, then Pos + att.
    Flew {
        /// `auto_yaw.set_mode(HOLD)` ran (stale dest and RATE / ANGLE_RATE).
        yaw_hold: bool,
        /// `terrain_d_m` passed to `input_pos_NED_m`. Zero when not terrain.
        terrain_d_m: f32,
        /// `terrain_margin_m` passed to `input_pos_NED_m`.
        terrain_margin_m: f32,
    },
}

/// Vehicle view `ModeGuided::pos_control_run` reads.
#[derive(Debug, Clone, Copy)]
pub struct GuidedPosControlView {
    /// `Mode::is_disarmed_or_landed()`.
    pub disarmed_or_landed: bool,
    /// `copter.is_tradheli()`.
    pub is_tradheli: bool,
    /// `motors->get_interlock()`.
    pub motor_interlock: bool,
    /// `guided_is_terrain_alt`.
    pub terrain_alt: bool,
    /// `wp_nav->get_terrain_D_m` succeeded. Consulted only when terrain.
    pub terrain_d_ok: bool,
    /// Terrain D, metres, when [`Self::terrain_d_ok`].
    pub terrain_d_m: f32,
    /// `wp_nav->get_terrain_margin_m()`.
    pub wp_terrain_margin_m: f32,
    /// `guided_pos_target_ned_m`.
    pub pos_target_ned_m: [f32; 3],
    /// `millis()`.
    pub now_ms: u32,
    /// `update_time_ms` stamped by the last dest / vel setter.
    pub update_time_ms: u32,
    /// `g2.guided_timeout`, seconds.
    pub guided_timeout_s: f32,
    /// `auto_yaw.mode()` at the top of the tick.
    pub auto_yaw: crate::auto_yaw::YawMode,
}

impl GuidedPosControlView {
    /// After a non-terrain [`guided_set_destination`]: flying, dest fresh.
    #[must_use]
    pub const fn after_pos_dest() -> Self {
        Self {
            disarmed_or_landed: false,
            is_tradheli: false,
            motor_interlock: false,
            terrain_alt: false,
            terrain_d_ok: false,
            terrain_d_m: 0.0,
            wp_terrain_margin_m: 2.0,
            pos_target_ned_m: [20.0, 10.0, -15.0],
            now_ms: 1_100,
            update_time_ms: 1_000,
            guided_timeout_s: GUIDED_TIMEOUT_DEFAULT_S,
            auto_yaw: crate::auto_yaw::YawMode::Hold,
        }
    }
}

/// Leftover of one `ModeGuided::pos_control_run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GuidedPosControl {
    /// Which arm returned, or the fly leftover.
    pub exit: GuidedPosControlExit,
}

/// Upstream `ModeGuided::pos_control_run`.
///
/// Disarmed wins over terrain. Terrain fail wins over the fly path.
/// Timeout uses wrapping unsigned subtract and `>`, not `>=`. Only
/// [`crate::auto_yaw::YawMode::Rate`] and
/// [`crate::auto_yaw::YawMode::AngleRate`] go to HOLD.
#[must_use]
pub fn guided_pos_control_run(view: &GuidedPosControlView) -> GuidedPosControl {
    if view.disarmed_or_landed {
        return GuidedPosControl {
            exit: GuidedPosControlExit::Disarmed {
                keep_interlock: view.is_tradheli && view.motor_interlock,
            },
        };
    }

    if view.terrain_alt && !view.terrain_d_ok {
        return GuidedPosControl {
            exit: GuidedPosControlExit::TerrainFailsafe,
        };
    }

    let timed_out =
        view.now_ms.wrapping_sub(view.update_time_ms) > guided_timeout_ms(view.guided_timeout_s);
    let yaw_hold = timed_out
        && matches!(
            view.auto_yaw,
            crate::auto_yaw::YawMode::Rate | crate::auto_yaw::YawMode::AngleRate
        );

    let terrain_d_m = if view.terrain_alt {
        view.terrain_d_m
    } else {
        0.0
    };
    let terrain_margin_m = if view.terrain_alt {
        let half_abs_z = 0.5 * view.pos_target_ned_m[2].abs();
        if view.wp_terrain_margin_m < half_abs_z {
            view.wp_terrain_margin_m
        } else {
            half_abs_z
        }
    } else {
        0.0
    };

    GuidedPosControl {
        exit: GuidedPosControlExit::Flew {
            yaw_hold,
            terrain_d_m,
            terrain_margin_m,
        },
    }
}

/// Vehicle view `ModeGuided::angle_control_start` reads.
///
/// Unlike [`guided_init`]'s `pva_control_start`, Angle start writes only
/// the vertical limits and only inits D when it is inactive. NE is left
/// alone — a GCS that was already flying VelAccel must not have its
/// horizontal controller reset just because the next command is an
/// attitude.
#[derive(Debug, Clone, Copy)]
pub struct GuidedAngleStartView {
    /// `pos_control->D_is_active()`.
    pub d_is_active: bool,
    /// `wp_nav->get_default_speed_down_ms()`.
    pub default_speed_down_ms: f32,
    /// `wp_nav->get_default_speed_up_ms()`.
    pub default_speed_up_ms: f32,
    /// `wp_nav->get_accel_D_mss()`.
    pub accel_d_mss: f32,
    /// `attitude_control->get_att_target_euler_rad().z`.
    pub att_target_yaw_rad: f32,
    /// `millis()` stamped into `guided_angle_state.update_time_ms`.
    pub now_ms: u32,
}

impl GuidedAngleStartView {
    /// After [`guided_init`]: D is already active, wpnav defaults, north.
    #[must_use]
    pub const fn after_init() -> Self {
        Self {
            d_is_active: true,
            default_speed_down_ms: WP_SPD_DOWN_DEFAULT_MS,
            default_speed_up_ms: WP_SPD_UP_DEFAULT_MS,
            accel_d_mss: WP_ACC_Z_DEFAULT_MSS,
            att_target_yaw_rad: 0.0,
            now_ms: 2_000,
        }
    }
}

/// Leftover of one `ModeGuided::angle_control_start` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GuidedAngleStart {
    /// Always [`GuidedSubMode::Angle`].
    pub submode: GuidedSubMode,
    /// Vertical max / correction descent speed.
    pub d_speed_down_ms: f32,
    /// Vertical max / correction climb speed.
    pub d_speed_up_ms: f32,
    /// Vertical max / correction accel.
    pub d_accel_mss: f32,
    /// `D_init_controller` ran because D was inactive.
    pub init_d: bool,
    /// Always false. Angle start does not touch NE.
    pub init_ne: bool,
    /// Seeded `from_euler(0, 0, att_target_yaw)`. `set_angle` overwrites
    /// this immediately; the seed is the leftover of a start with no
    /// attitude yet.
    pub attitude_quat: [f32; 4],
    /// `guided_angle_state.ang_vel_body` after start. Always zero.
    pub ang_vel_body: [f32; 3],
    /// `guided_angle_state.climb_rate_ms` after start. Always zero.
    pub climb_rate_ms: f32,
    /// `guided_angle_state.update_time_ms` after start.
    pub update_time_ms: u32,
}

/// Upstream `ModeGuided::angle_control_start`.
///
/// D limits always. D init only when inactive. NE never. The seeded
/// quat is yaw-only; `use_thrust` / `thrust_norm` are not written.
#[must_use]
pub fn guided_angle_control_start(view: &GuidedAngleStartView) -> GuidedAngleStart {
    let q = Quaternion::from_euler(0.0, 0.0, view.att_target_yaw_rad);
    GuidedAngleStart {
        submode: GuidedSubMode::Angle,
        d_speed_down_ms: view.default_speed_down_ms,
        d_speed_up_ms: view.default_speed_up_ms,
        d_accel_mss: view.accel_d_mss,
        init_d: !view.d_is_active,
        init_ne: false,
        attitude_quat: [q.q1, q.q2, q.q3, q.q4],
        ang_vel_body: [0.0, 0.0, 0.0],
        climb_rate_ms: 0.0,
        update_time_ms: view.now_ms,
    }
}

/// Vehicle view `ModeGuided::set_angle` reads.
#[derive(Debug, Clone, Copy)]
pub struct GuidedSetAngleView {
    /// `guided_mode` before the call.
    pub submode: GuidedSubMode,
    /// `guided_angle_state.use_thrust` before the call.
    pub already_use_thrust: bool,
    /// `pos_control->D_is_active()`. Consulted only when start runs.
    pub d_is_active: bool,
    /// `wp_nav->get_default_speed_down_ms()`. Used only when start runs.
    pub default_speed_down_ms: f32,
    /// `wp_nav->get_default_speed_up_ms()`. Used only when start runs.
    pub default_speed_up_ms: f32,
    /// `wp_nav->get_accel_D_mss()`. Used only when start runs.
    pub accel_d_mss: f32,
    /// Current attitude-target yaw, used only by start's seed quat.
    pub att_target_yaw_rad: f32,
    /// Commanded attitude quaternion, scalar-first (`q1..q4`).
    pub attitude_quat: [f32; 4],
    /// Commanded body-frame angular velocity, rad/s.
    pub ang_vel_body: [f32; 3],
    /// `climb_rate_ms_or_thrust` argument.
    pub climb_rate_ms_or_thrust: f32,
    /// `use_thrust` argument.
    pub use_thrust: bool,
    /// `millis()` stamped into `guided_angle_state.update_time_ms`.
    pub now_ms: u32,
    /// `HAL_LOGGING_ENABLED`. There is no `log_request` gate.
    pub logging_enabled: bool,
}

impl GuidedSetAngleView {
    /// After [`guided_init`]: VelAccel, D already active, climb-rate command.
    #[must_use]
    pub const fn after_init() -> Self {
        Self {
            submode: GuidedSubMode::VelAccel,
            already_use_thrust: false,
            d_is_active: true,
            default_speed_down_ms: WP_SPD_DOWN_DEFAULT_MS,
            default_speed_up_ms: WP_SPD_UP_DEFAULT_MS,
            accel_d_mss: WP_ACC_Z_DEFAULT_MSS,
            att_target_yaw_rad: 0.0,
            attitude_quat: [1.0, 0.0, 0.0, 0.0],
            ang_vel_body: [0.1, -0.05, 0.2],
            climb_rate_ms_or_thrust: 1.5,
            use_thrust: false,
            now_ms: 2_000,
            logging_enabled: true,
        }
    }
}

/// Leftover of one `ModeGuided::set_angle` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GuidedSetAngle {
    /// Always [`GuidedSubMode::Angle`].
    pub submode: GuidedSubMode,
    /// `angle_control_start` ran because the submode was not Angle.
    pub started_angle: bool,
    /// `D_init_controller` ran. Start-when-inactive, or the
    /// already-Angle thrust-to-climb switch. Not both: the switch is
    /// an `else if`.
    pub init_d: bool,
    /// D limits written by start. `None` when already Angle.
    pub d_speed_down_ms: Option<f32>,
    /// D climb limit written by start. `None` when already Angle.
    pub d_speed_up_ms: Option<f32>,
    /// D accel written by start. `None` when already Angle.
    pub d_accel_mss: Option<f32>,
    /// `guided_angle_state.attitude_quat` after the call.
    pub attitude_quat: [f32; 4],
    /// `guided_angle_state.ang_vel_body` after the call.
    pub ang_vel_body: [f32; 3],
    /// `guided_angle_state.use_thrust` after the call.
    pub use_thrust: bool,
    /// `guided_angle_state.thrust_norm` after the call. Zero on climb.
    pub thrust_norm: f32,
    /// `guided_angle_state.climb_rate_ms` after the call. Zero on thrust.
    pub climb_rate_ms: f32,
    /// `guided_angle_state.update_time_ms` after the call.
    pub update_time_ms: u32,
    /// `attitude_quat.to_euler()` leftover, written even when logging
    /// is compiled out.
    pub euler_rad: [f32; 3],
    /// `Log_Write_Guided_Attitude_Target` ran.
    pub logged: bool,
}

/// Upstream `ModeGuided::set_angle`.
///
/// Not-Angle starts Angle. Already-Angle and switching from thrust to
/// climb re-inits D. Climb-to-thrust does not. The unused vertical
/// channel is zeroed. The euler conversion always runs; the log write
/// is compiled out when logging is off. There is no `log_request`.
#[must_use]
pub fn guided_set_angle(view: &GuidedSetAngleView) -> GuidedSetAngle {
    let started_angle = view.submode != GuidedSubMode::Angle;
    let init_d = if started_angle {
        !view.d_is_active
    } else {
        !view.use_thrust && view.already_use_thrust
    };

    let (thrust_norm, climb_rate_ms) = if view.use_thrust {
        (view.climb_rate_ms_or_thrust, 0.0)
    } else {
        (0.0, view.climb_rate_ms_or_thrust)
    };

    let q = Quaternion::new(
        view.attitude_quat[0],
        view.attitude_quat[1],
        view.attitude_quat[2],
        view.attitude_quat[3],
    );
    let (roll_rad, pitch_rad, yaw_rad) = q.to_euler();

    GuidedSetAngle {
        submode: GuidedSubMode::Angle,
        started_angle,
        init_d,
        d_speed_down_ms: started_angle.then_some(view.default_speed_down_ms),
        d_speed_up_ms: started_angle.then_some(view.default_speed_up_ms),
        d_accel_mss: started_angle.then_some(view.accel_d_mss),
        attitude_quat: view.attitude_quat,
        ang_vel_body: view.ang_vel_body,
        use_thrust: view.use_thrust,
        thrust_norm,
        climb_rate_ms,
        update_time_ms: view.now_ms,
        euler_rad: [roll_rad, pitch_rad, yaw_rad],
        logged: view.logging_enabled,
    }
}

/// How `ModeGuided::angle_control_run` commanded attitude on a fly tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GuidedAngleAttitude {
    /// `attitude_quat.is_zero()` then `input_rate_bf_roll_pitch_yaw_rads`.
    Rate,
    /// Non-zero quat then `input_quaternion`.
    Quaternion,
}

/// Vertical leftover of a fly tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GuidedAngleVertical {
    /// `set_throttle_out(thrust_norm, true, throttle_filt)`.
    Thrust {
        /// `guided_angle_state.thrust_norm` after the timeout gate.
        thrust_norm: f32,
    },
    /// `D_set_pos_target_from_climb_rate_ms` then `D_update_controller`.
    Climb {
        /// Climb after constrain, avoidance, and timeout.
        climb_rate_ms: f32,
    },
}

/// How `ModeGuided::angle_control_run` left the tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GuidedAngleControlExit {
    /// `!armed || !auto_armed || (land_complete && !positive)`:
    /// `make_safe_ground_handling` and return.
    Ground {
        /// `copter.is_tradheli() && motors->get_interlock()`.
        keep_interlock: bool,
    },
    /// Landed and positive: relax, unlimited spool, then maybe clear land.
    Takeoff {
        /// Current spool was already `THROTTLE_UNLIMITED`:
        /// `set_land_complete(false)` and `D_init_controller`.
        cleared_land: bool,
    },
    /// Flying: unlimited spool, then attitude + vertical leftovers.
    Flew {
        /// Rate vs quaternion leftover.
        attitude: GuidedAngleAttitude,
        /// Thrust vs climb leftover. Uses post-timeout `use_thrust`.
        vertical: GuidedAngleVertical,
    },
}

/// Vehicle view `ModeGuided::angle_control_run` reads.
#[derive(Debug, Clone, Copy)]
pub struct GuidedAngleControlView {
    /// `guided_angle_state.use_thrust` at the top of the tick.
    pub use_thrust: bool,
    /// `guided_angle_state.climb_rate_ms` at the top of the tick.
    pub climb_rate_ms: f32,
    /// `guided_angle_state.thrust_norm`.
    pub thrust_norm: f32,
    /// Commanded attitude, scalar-first (`q1..q4`).
    pub attitude_quat: [f32; 4],
    /// `guided_angle_state.ang_vel_body`.
    pub ang_vel_body: [f32; 3],
    /// `wp_nav->get_default_speed_down_ms()` — constrain lower magnitude.
    pub default_speed_down_ms: f32,
    /// `wp_nav->get_default_speed_up_ms()` — constrain upper bound.
    pub default_speed_up_ms: f32,
    /// `millis()`.
    pub now_ms: u32,
    /// `guided_angle_state.update_time_ms`.
    pub update_time_ms: u32,
    /// `g2.guided_timeout`, seconds.
    pub guided_timeout_s: f32,
    /// `attitude_control->get_att_target_euler_rad().z` — timeout seed yaw.
    pub att_target_yaw_rad: f32,
    /// `motors->armed()`.
    pub motors_armed: bool,
    /// `copter.ap.auto_armed` at the top of the tick.
    pub auto_armed: bool,
    /// `copter.ap.land_complete`.
    pub land_complete: bool,
    /// `copter.is_tradheli()`.
    pub is_tradheli: bool,
    /// `motors->get_interlock()`.
    pub motor_interlock: bool,
    /// `motors->get_spool_state() == THROTTLE_UNLIMITED`. Consulted only
    /// on the takeoff arm.
    pub spool_unlimited: bool,
}

impl GuidedAngleControlView {
    /// After a climb [`guided_set_angle`]: flying, dest fresh, identity quat.
    #[must_use]
    pub const fn after_set_angle() -> Self {
        Self {
            use_thrust: false,
            climb_rate_ms: 1.5,
            thrust_norm: 0.0,
            attitude_quat: [1.0, 0.0, 0.0, 0.0],
            ang_vel_body: [0.1, -0.05, 0.2],
            default_speed_down_ms: WP_SPD_DOWN_DEFAULT_MS,
            default_speed_up_ms: WP_SPD_UP_DEFAULT_MS,
            now_ms: 2_100,
            update_time_ms: 2_000,
            guided_timeout_s: GUIDED_TIMEOUT_DEFAULT_S,
            att_target_yaw_rad: 0.0,
            motors_armed: true,
            auto_armed: true,
            land_complete: false,
            is_tradheli: false,
            motor_interlock: false,
            spool_unlimited: false,
        }
    }
}

/// Leftover of one `ModeGuided::angle_control_run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GuidedAngleControl {
    /// Which arm returned, or the fly leftover.
    pub exit: GuidedAngleControlExit,
    /// `set_auto_armed(true)` ran (`armed && positive` after timeout).
    pub auto_armed: bool,
    /// Timeout fired. Always evaluated, even on the ground / takeoff arms.
    pub timed_out: bool,
    /// Timeout cleared thrust (`D_init_controller` + `use_thrust = false`).
    pub timeout_init_d: bool,
    /// `use_thrust` after the timeout gate.
    pub use_thrust: bool,
    /// Constrain leftover. Zero when the tick started in thrust.
    pub constrained_climb_ms: f32,
    /// Climb after constrain / avoidance / timeout. Zero on a thrust start
    /// and after any timeout.
    pub climb_rate_ms: f32,
    /// Avoidance ran because the tick started in climb, not thrust.
    pub avoidance_applied: bool,
    /// Quat after the timeout gate (yaw-only seed, or the command).
    pub attitude_quat: [f32; 4],
    /// Body rates after the timeout gate (zeroed on timeout).
    pub ang_vel_body: [f32; 3],
}

/// Upstream `ModeGuided::angle_control_run`.
///
/// Constrain and avoidance only on climb. Timeout (wrapping, `>`) can
/// convert thrust to a zero-climb hold and is evaluated *before*
/// `is_positive` and the exits. Auto-arm is written before the ground
/// check, so a first positive command is not grounded for `!auto_armed`.
#[must_use]
pub fn guided_angle_control_run(view: &GuidedAngleControlView) -> GuidedAngleControl {
    let avoidance_applied = !view.use_thrust;
    let constrained_climb_ms = if view.use_thrust {
        0.0
    } else {
        constrain_value(
            view.climb_rate_ms,
            -view.default_speed_down_ms,
            view.default_speed_up_ms,
        )
    };

    let timed_out =
        view.now_ms.wrapping_sub(view.update_time_ms) > guided_timeout_ms(view.guided_timeout_s);

    let (use_thrust, timeout_init_d, climb_rate_ms, attitude_quat, ang_vel_body) = if timed_out {
        let q = Quaternion::from_euler(0.0, 0.0, view.att_target_yaw_rad);
        let timeout_init_d = view.use_thrust;
        (
            false,
            timeout_init_d,
            0.0,
            [q.q1, q.q2, q.q3, q.q4],
            [0.0, 0.0, 0.0],
        )
    } else {
        (
            view.use_thrust,
            false,
            constrained_climb_ms,
            view.attitude_quat,
            view.ang_vel_body,
        )
    };

    let vertical_cmd = if use_thrust {
        view.thrust_norm
    } else {
        climb_rate_ms
    };
    let positive = is_positive(vertical_cmd);
    let auto_armed = view.motors_armed && positive;
    let auto_armed_now = view.auto_armed || auto_armed;

    let exit = if !view.motors_armed || !auto_armed_now || (view.land_complete && !positive) {
        GuidedAngleControlExit::Ground {
            keep_interlock: view.is_tradheli && view.motor_interlock,
        }
    } else if view.land_complete && positive {
        GuidedAngleControlExit::Takeoff {
            cleared_land: view.spool_unlimited,
        }
    } else {
        let q = Quaternion::new(
            attitude_quat[0],
            attitude_quat[1],
            attitude_quat[2],
            attitude_quat[3],
        );
        GuidedAngleControlExit::Flew {
            attitude: if q.is_zero() {
                GuidedAngleAttitude::Rate
            } else {
                GuidedAngleAttitude::Quaternion
            },
            vertical: if use_thrust {
                GuidedAngleVertical::Thrust {
                    thrust_norm: view.thrust_norm,
                }
            } else {
                GuidedAngleVertical::Climb { climb_rate_ms }
            },
        }
    };

    GuidedAngleControl {
        exit,
        auto_armed,
        timed_out,
        timeout_init_d,
        use_thrust,
        constrained_climb_ms,
        climb_rate_ms,
        avoidance_applied,
        attitude_quat,
        ang_vel_body,
    }
}

/// How `ModeGuided::velaccel_control_run` stopped NE stabilisation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GuidedVelAccelStop {
    /// Both axes still stabilising, or `do_avoid` blocked the stops.
    None,
    /// `!stabilizing_vel_NE && !do_avoid`: `NE_stop_vel_stabilisation`.
    StopVel,
    /// `stabilizing_vel_NE && !stabilizing_pos_NE && !do_avoid`:
    /// `NE_stop_pos_stabilisation`.
    StopPos,
}

/// How `ModeGuided::velaccel_control_run` left the tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GuidedVelAccelExit {
    /// `is_disarmed_or_landed`: `make_safe_ground_handling` and return.
    Disarmed {
        /// `copter.is_tradheli() && motors->get_interlock()`.
        keep_interlock: bool,
    },
    /// Flying: unlimited spool, then timeout / avoid / stab leftovers.
    Flew {
        /// `auto_yaw.set_mode(HOLD)` ran (stale dest and RATE / ANGLE_RATE).
        yaw_hold: bool,
        /// Timeout zeroed `guided_vel_target_ned_ms`.
        vel_zeroed: bool,
        /// Timeout zeroed `guided_accel_target_ned_mss`.
        accel_zeroed: bool,
        /// `AP_AVOIDANCE_ENABLED` compiled in: `adjust_velocity_NED_m` ran.
        avoidance_applied: bool,
        /// `copter.avoid.limits_active()` after that call. False when
        /// avoidance is compiled out.
        do_avoid: bool,
        /// NE vel leftover overwritten from `get_vel_desired_NED_ms`.
        vel_from_desired_ne: bool,
        /// Which stop-stabilisation ran.
        stop: GuidedVelAccelStop,
        /// Vel leftover after timeout and the desired-NE overwrite.
        vel_ned_ms: [f32; 3],
        /// Accel leftover after timeout.
        accel_ned_mss: [f32; 3],
    },
}

/// Vehicle view `ModeGuided::velaccel_control_run` reads.
#[derive(Debug, Clone, Copy)]
pub struct GuidedVelAccelView {
    /// `Mode::is_disarmed_or_landed()`.
    pub disarmed_or_landed: bool,
    /// `copter.is_tradheli()`.
    pub is_tradheli: bool,
    /// `motors->get_interlock()`.
    pub motor_interlock: bool,
    /// `millis()`.
    pub now_ms: u32,
    /// `update_time_ms` stamped by the last dest / vel setter.
    pub update_time_ms: u32,
    /// `g2.guided_timeout`, seconds.
    pub guided_timeout_s: f32,
    /// `auto_yaw.mode()` at the top of the tick.
    pub auto_yaw: crate::auto_yaw::YawMode,
    /// `guided_vel_target_ned_ms` at the top of the tick.
    pub vel_target_ned_ms: [f32; 3],
    /// `guided_accel_target_ned_mss` at the top of the tick.
    pub accel_target_ned_mss: [f32; 3],
    /// `pos_control->get_vel_desired_NED_ms()` — NE overwrite source.
    pub vel_desired_ned_ms: [f32; 3],
    /// `stabilizing_vel_NE()`.
    pub stabilizing_vel_ne: bool,
    /// `stabilizing_pos_NE()`.
    pub stabilizing_pos_ne: bool,
    /// `AP_AVOIDANCE_ENABLED`.
    pub avoidance_enabled: bool,
    /// `copter.avoid.limits_active()`. Consulted only when avoidance.
    pub avoidance_limits_active: bool,
}

impl GuidedVelAccelView {
    /// After [`guided_set_velocity`]: flying, dest fresh, both axes stab.
    #[must_use]
    pub const fn after_set_vel() -> Self {
        Self {
            disarmed_or_landed: false,
            is_tradheli: false,
            motor_interlock: false,
            now_ms: 2_100,
            update_time_ms: 2_000,
            guided_timeout_s: GUIDED_TIMEOUT_DEFAULT_S,
            auto_yaw: crate::auto_yaw::YawMode::Hold,
            vel_target_ned_ms: [1.5, -0.5, 0.0],
            accel_target_ned_mss: [0.0, 0.0, 0.0],
            vel_desired_ned_ms: [0.3, 0.1, -0.2],
            stabilizing_vel_ne: true,
            stabilizing_pos_ne: true,
            avoidance_enabled: true,
            avoidance_limits_active: false,
        }
    }
}

/// Leftover of one `ModeGuided::velaccel_control_run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GuidedVelAccel {
    /// Which arm returned, or the fly leftover.
    pub exit: GuidedVelAccelExit,
}

/// Upstream `ModeGuided::velaccel_control_run`.
///
/// Disarmed is the only early exit. Timeout uses wrapping unsigned
/// subtract and `>`, not `>=`. Avoidance is identity on the numbers
/// and only `do_avoid` changes the leftover. `!stabilizing_vel_NE &&
/// !do_avoid` overwrites NE vel from the position controller's desired
/// and stops vel stab; else `!stabilizing_pos_NE && !do_avoid` stops
/// pos stab. D always takes the post-timeout vel/accel z.
#[must_use]
pub fn guided_velaccel_control_run(view: &GuidedVelAccelView) -> GuidedVelAccel {
    if view.disarmed_or_landed {
        return GuidedVelAccel {
            exit: GuidedVelAccelExit::Disarmed {
                keep_interlock: view.is_tradheli && view.motor_interlock,
            },
        };
    }

    let timed_out =
        view.now_ms.wrapping_sub(view.update_time_ms) > guided_timeout_ms(view.guided_timeout_s);
    let yaw_hold = timed_out
        && matches!(
            view.auto_yaw,
            crate::auto_yaw::YawMode::Rate | crate::auto_yaw::YawMode::AngleRate
        );

    let mut vel_ned_ms = if timed_out {
        [0.0, 0.0, 0.0]
    } else {
        view.vel_target_ned_ms
    };
    let accel_ned_mss = if timed_out {
        [0.0, 0.0, 0.0]
    } else {
        view.accel_target_ned_mss
    };

    let avoidance_applied = view.avoidance_enabled;
    let do_avoid = view.avoidance_enabled && view.avoidance_limits_active;
    let vel_from_desired_ne = !view.stabilizing_vel_ne && !do_avoid;
    if vel_from_desired_ne {
        vel_ned_ms[0] = view.vel_desired_ned_ms[0];
        vel_ned_ms[1] = view.vel_desired_ned_ms[1];
    }

    let stop = if !view.stabilizing_vel_ne && !do_avoid {
        GuidedVelAccelStop::StopVel
    } else if !view.stabilizing_pos_ne && !do_avoid {
        GuidedVelAccelStop::StopPos
    } else {
        GuidedVelAccelStop::None
    };

    GuidedVelAccel {
        exit: GuidedVelAccelExit::Flew {
            yaw_hold,
            vel_zeroed: timed_out,
            accel_zeroed: timed_out,
            avoidance_applied,
            do_avoid,
            vel_from_desired_ne,
            stop,
            vel_ned_ms,
            accel_ned_mss,
        },
    }
}
