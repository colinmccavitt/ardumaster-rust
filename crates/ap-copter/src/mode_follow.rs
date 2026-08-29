//! `ModeFollow` init leftover, upstream `ArduCopter/mode_follow.cpp`.
//!
//! Tracked as **COP-017**. Follow is Guided's companion that tracks another
//! MAVLink vehicle by sysid. It inherits `ModeGuided` but its `init` does
//! **not** call `ModeGuided::init`. There is no VelAccel park, no dest
//! notification, no pause clear. The leftover is an enable gate, an
//! optional mount hand-off, then the same wpnav-sized PosControl start
//! Guided uses in `pva_control_start`, then default yaw.
//!
//! # Enable is `FOLL_ENABLE`, not `ignore_checks`
//!
//! `enabled()` is `g2.follow.enabled()` — `FOLL_ENABLE`, default 0. A
//! disabled library sends `Set FOLL_ENABLE = 1` and returns false.
//! `ignore_checks` is unused. Mount and PosControl never run on that
//! arm.
//!
//! # Mount is compiled out, then a two-way leftover
//!
//! `HAL_MOUNT_ENABLED` compiled out never calls `set_target_sysid`.
//! Compiled in, the call needs both `MOUNT_FOLLOW_ON_ENTER` (`FOLL_OPTIONS`
//! bit 0) and a live `AP_Mount` singleton. A missing mount is silent.
//!
//! # The PosControl start is `pva` without a submode
//!
//! NE and D max / correction limits come from the waypoint navigator
//! defaults. Both controllers always re-init — Follow does not ask
//! `D_is_active`. Yaw is `set_mode_to_default(false)`. The Guided
//! submode, vel/accel leftovers, `send_notification`, `_paused`, and
//! `guided_is_terrain_alt` are left alone. `run` is a later slice.

use crate::mode_guided::{
    GuidedModeFlags, GuidedYawAction, MODE_NUMBER_FOLLOW, WPNAV_ACCELERATION_MSS,
    WP_ACC_Z_DEFAULT_MSS, WP_SPD_DEFAULT_MS, WP_SPD_DOWN_DEFAULT_MS, WP_SPD_UP_DEFAULT_MS,
};

/// `AP_Follow::Option::MOUNT_FOLLOW_ON_ENTER` — `FOLL_OPTIONS` bit 0.
pub const FOLLOW_OPTION_MOUNT_FOLLOW_ON_ENTER: u16 = 1;

/// Upstream `AP_Follow::option_is_enabled`.
#[must_use]
pub const fn follow_option_is_enabled(follow_options: u16, option: u16) -> bool {
    follow_options & option != 0
}

/// `ModeFollow` capability flags from `mode.h`.
///
/// Follow inherits `ModeGuided` then overrides `allows_arming` to
/// always false. The rest match Guided except the mode number.
/// `enabled()` is not here: it is the runtime `FOLL_ENABLE` leftover.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FollowModeFlags {
    /// Shared Guided catalog plus the Follow-only arming leftover.
    pub guided: GuidedModeFlags,
    /// `allows_arming(...)` — always false, regardless of method.
    pub allows_arming: bool,
}

/// Upstream `ModeFollow` flags.
#[must_use]
pub const fn follow_mode_flags() -> FollowModeFlags {
    FollowModeFlags {
        guided: GuidedModeFlags {
            mode_number: MODE_NUMBER_FOLLOW,
            requires_position: true,
            has_manual_throttle: false,
            is_autopilot: true,
            has_user_takeoff: true,
            in_guided_mode: true,
            requires_terrain_failsafe: true,
            allows_gcs_or_scr_arming_with_throttle_high: true,
        },
        allows_arming: false,
    }
}

/// Vehicle view `ModeFollow::init` reads.
///
/// `ignore_checks` is accepted on the function, not here: the leftover
/// ignores it.
#[derive(Debug, Clone, Copy)]
pub struct FollowInitView {
    /// `g2.follow.enabled()` / `FOLL_ENABLE`.
    pub follow_enabled: bool,
    /// `HAL_MOUNT_ENABLED`.
    pub mount_enabled: bool,
    /// `AP_Mount::get_singleton() != nullptr`.
    pub mount_present: bool,
    /// `g2.follow.option_is_enabled(MOUNT_FOLLOW_ON_ENTER)`.
    pub mount_follow_on_enter: bool,
    /// `g2.follow.get_target_sysid()`.
    pub target_sysid: u8,
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

impl FollowInitView {
    /// Parameter defaults: `FOLL_ENABLE` off, wpnav defaults, no mount.
    #[must_use]
    pub const fn typical() -> Self {
        Self {
            follow_enabled: false,
            mount_enabled: true,
            mount_present: true,
            mount_follow_on_enter: false,
            target_sysid: 2,
            default_speed_ne_ms: WP_SPD_DEFAULT_MS,
            wp_acceleration_mss: WPNAV_ACCELERATION_MSS,
            default_speed_down_ms: WP_SPD_DOWN_DEFAULT_MS,
            default_speed_up_ms: WP_SPD_UP_DEFAULT_MS,
            accel_d_mss: WP_ACC_Z_DEFAULT_MSS,
        }
    }

    /// Enabled Follow with wpnav defaults and no mount hand-off.
    #[must_use]
    pub const fn enabled() -> Self {
        let mut view = Self::typical();
        view.follow_enabled = true;
        view
    }
}

/// Leftover of one `ModeFollow::init` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FollowInit {
    /// `FOLL_ENABLE` was on. `ignore_checks` is unused.
    pub ok: bool,
    /// GCS warning `Set FOLL_ENABLE = 1` ran.
    pub gcs_enable_warning: bool,
    /// `mount->set_target_sysid` ran with this sysid.
    pub mount_sysid: Option<u8>,
    /// Horizontal max / correction speed. Zero when disabled.
    pub ne_speed_ms: f32,
    /// Horizontal max / correction accel. Zero when disabled.
    pub ne_accel_mss: f32,
    /// Vertical max / correction descent speed. Zero when disabled.
    pub d_speed_down_ms: f32,
    /// Vertical max / correction climb speed. Zero when disabled.
    pub d_speed_up_ms: f32,
    /// Vertical max / correction accel. Zero when disabled.
    pub d_accel_mss: f32,
    /// `NE_init_controller` ran.
    pub init_ne: bool,
    /// `D_init_controller` ran. Follow always inits both when enabled.
    pub init_d: bool,
    /// `auto_yaw.set_mode_to_default(false)` on success.
    pub yaw: GuidedYawAction,
}

/// Upstream `ModeFollow::init`.
///
/// Disabled is a GCS warning and an immediate false. Enabled writes the
/// wpnav-sized PosControl limits, inits both axes, and defaults yaw.
/// Mount is a compiled-out / option / singleton leftover and never
/// blocks init.
#[must_use]
pub fn follow_init(view: &FollowInitView, _ignore_checks: bool) -> FollowInit {
    if !view.follow_enabled {
        return FollowInit {
            ok: false,
            gcs_enable_warning: true,
            mount_sysid: None,
            ne_speed_ms: 0.0,
            ne_accel_mss: 0.0,
            d_speed_down_ms: 0.0,
            d_speed_up_ms: 0.0,
            d_accel_mss: 0.0,
            init_ne: false,
            init_d: false,
            yaw: GuidedYawAction::NotCalled,
        };
    }

    let mount_sysid = if view.mount_enabled && view.mount_follow_on_enter && view.mount_present {
        Some(view.target_sysid)
    } else {
        None
    };

    FollowInit {
        ok: true,
        gcs_enable_warning: false,
        mount_sysid,
        ne_speed_ms: view.default_speed_ne_ms,
        ne_accel_mss: view.wp_acceleration_mss,
        d_speed_down_ms: view.default_speed_down_ms,
        d_speed_up_ms: view.default_speed_up_ms,
        d_accel_mss: view.accel_d_mss,
        init_ne: true,
        init_d: true,
        yaw: GuidedYawAction::SetModeToDefault,
    }
}
