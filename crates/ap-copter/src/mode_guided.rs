//! `ModeGuided` init / set_destination leftover, upstream `ArduCopter/mode_guided.cpp`.
//!
//! Tracked as **COP-017**. Guided is the GCS / scripting command mode: fly
//! to a coordinate, hold a velocity, or take an attitude. This first slice
//! owns the enter and the Location dest setter. The `*_run` controllers,
//! velocity / accel / angle setters, Guided-NoGPS, and Follow are later
//! slices.
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

/// `Mode::Number::GUIDED`.
pub const MODE_NUMBER_GUIDED: u8 = 4;

/// `Mode::Number::GUIDED_NOGPS` — later slice; the number is pinned here
/// so Guided and Guided-NoGPS stay in one catalog.
pub const MODE_NUMBER_GUIDED_NOGPS: u8 = 20;

/// `Mode::Number::FOLLOW` — later slice; same catalog reason.
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
