//! `ModeFollow` init / run leftover, upstream `ArduCopter/mode_follow.cpp`.
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
//! `guided_is_terrain_alt` are left alone.
//!
//! # `run` is one early exit, then offsets, then a target leftover
//!
//! `is_disarmed_or_landed` is the only early return. Unlike Guided's
//! `*_control_run` bodies, Follow calls `make_safe_ground_handling()`
//! with no argument — tradheli interlock is not a keep. Offsets,
//! spool, and yaw never run on that arm.
//!
//! The fly path always `init_offsets_if_required` (so the vehicle does
//! not start on top of the lead) and writes `THROTTLE_UNLIMITED`. Yaw
//! seeds from the attitude target; the rate starts at zero.
//!
//! A valid `get_ofs_pos_vel_accel_NED_m` feeds `input_pos_vel_accel`
//! on both axes. Yaw then forks on `FOLL_YAW_BEHAVE`: face-lead uses
//! the lead *without* offset minus `get_pos_target` and only writes
//! when `length_squared > 1`; same-as-lead takes `radians` of the
//! lead heading and rate; dir-of-flight uses the offset velocity NE
//! and the same `> 1` gate; none / default leaves the seeded yaw.
//!
//! Invalid target data holds with zero `input_vel_accel` on both axes
//! and a zero yaw rate. Controllers and
//! `input_thrust_vector_heading_rad` always run on the fly path.

use ap_math::scalar::radians;
use ap_math::vector2::Vector2f;

use crate::mode_guided::{
    GuidedModeFlags, GuidedYawAction, MODE_NUMBER_FOLLOW, WPNAV_ACCELERATION_MSS,
    WP_ACC_Z_DEFAULT_MSS, WP_SPD_DEFAULT_MS, WP_SPD_DOWN_DEFAULT_MS, WP_SPD_UP_DEFAULT_MS,
};

/// `AP_Follow::Option::MOUNT_FOLLOW_ON_ENTER` — `FOLL_OPTIONS` bit 0.
pub const FOLLOW_OPTION_MOUNT_FOLLOW_ON_ENTER: u16 = 1;

/// `length_squared > 1` gate on face-lead and dir-of-flight yaw.
pub const FOLLOW_YAW_LENGTH_SQ_MIN: f32 = 1.0;

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

/// `AP_Follow::YawBehave` / `FOLL_YAW_BEHAVE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FollowYawBehave {
    /// `YAW_BEHAVE_NONE` — leave the seeded attitude-target yaw.
    None = 0,
    /// `YAW_BEHAVE_FACE_LEAD_VEHICLE`.
    FaceLeadVehicle = 1,
    /// `YAW_BEHAVE_SAME_AS_LEAD_VEHICLE`.
    SameAsLeadVehicle = 2,
    /// `YAW_BEHAVE_DIR_OF_FLIGHT`.
    DirOfFlight = 3,
}

/// What `ModeFollow::run` fed the position controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FollowPosInput {
    /// Valid ofs: `input_pos_vel_accel` on NE and D.
    PosVelAccel,
    /// Invalid ofs: zero `input_vel_accel` on NE and D.
    VelAccelHold,
}

/// Where the fly-path yaw leftover came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FollowYawSource {
    /// Seeded `get_att_target_euler_rad().z`, never overwritten.
    AttTarget,
    /// Face-lead: `vec_to_lead.angle()` after `length_squared > 1`.
    FaceLead,
    /// Same-as-lead: `radians` of the lead heading and rate.
    SameAsLead,
    /// Dir-of-flight: offset-velocity NE `angle()` after `length_squared > 1`.
    DirOfFlight,
}

/// How `ModeFollow::run` left the tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FollowRunExit {
    /// `is_disarmed_or_landed`: `make_safe_ground_handling()` and return.
    /// No tradheli-interlock keep — unlike Guided's `*_control_run`.
    Disarmed,
    /// Flying: offsets, unlimited spool, then target / yaw leftovers.
    Flew {
        /// `g2.follow.init_offsets_if_required` ran.
        init_offsets: bool,
        /// PosControl input leftover.
        input: FollowPosInput,
        /// Which yaw arm wrote `yaw_rad` / `yaw_rate_rads`.
        yaw_source: FollowYawSource,
        /// Heading passed to `input_thrust_vector_heading_rad`.
        yaw_rad: f32,
        /// Yaw rate passed to `input_thrust_vector_heading_rad`.
        yaw_rate_rads: f32,
        /// Offset pos fed on a valid target; zeros on hold.
        pos_ned_m: [f32; 3],
        /// Offset vel fed on a valid target; zeros on hold.
        vel_ned_ms: [f32; 3],
        /// Offset accel fed on a valid target; zeros on hold.
        accel_ned_mss: [f32; 3],
    },
}

/// Vehicle view `ModeFollow::run` reads.
#[derive(Debug, Clone, Copy)]
pub struct FollowRunView {
    /// `Mode::is_disarmed_or_landed()`.
    pub disarmed_or_landed: bool,
    /// `attitude_control->get_att_target_euler_rad().z`.
    pub att_yaw_rad: f32,
    /// `g2.follow.get_ofs_pos_vel_accel_NED_m` succeeded.
    pub ofs_valid: bool,
    /// Offset pos (lead + offset), metres NED.
    pub pos_ofs_ned_m: [f32; 3],
    /// Offset vel, m/s NED.
    pub vel_ofs_ned_ms: [f32; 3],
    /// Offset accel, m/s² NED.
    pub accel_ofs_ned_mss: [f32; 3],
    /// `g2.follow.get_target_heading_deg()`.
    pub target_heading_deg: f32,
    /// `g2.follow.get_target_heading_rate_degs()`.
    pub target_heading_rate_degs: f32,
    /// `g2.follow.get_yaw_behave()`.
    pub yaw_behave: FollowYawBehave,
    /// `g2.follow.get_target_pos_vel_accel_NED_m` succeeded.
    /// Consulted only on face-lead.
    pub target_pva_ok: bool,
    /// Lead vehicle pos *without* offset. Face-lead only.
    pub lead_pos_ned_m: [f32; 3],
    /// `pos_control->get_pos_target_NED_m()` after the ofs input.
    /// Face-lead subtracts this from [`Self::lead_pos_ned_m`].
    pub pos_target_ned_m: [f32; 3],
}

impl FollowRunView {
    /// Flying, valid ofs, `YAW_BEHAVE_NONE`, identity pos-target.
    #[must_use]
    pub const fn flying() -> Self {
        Self {
            disarmed_or_landed: false,
            att_yaw_rad: 0.3,
            ofs_valid: true,
            pos_ofs_ned_m: [8.0, 4.0, -12.0],
            vel_ofs_ned_ms: [1.5, -0.5, 0.0],
            accel_ofs_ned_mss: [0.2, 0.1, 0.0],
            target_heading_deg: 90.0,
            target_heading_rate_degs: 10.0,
            yaw_behave: FollowYawBehave::None,
            target_pva_ok: true,
            lead_pos_ned_m: [10.0, 0.0, -12.0],
            pos_target_ned_m: [8.0, 4.0, -12.0],
        }
    }
}

/// Leftover of one `ModeFollow::run` tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FollowRun {
    /// Which arm returned, or the fly leftover.
    pub exit: FollowRunExit,
}

/// Upstream `ModeFollow::run`.
///
/// Disarmed is the only early exit and has no tradheli keep. The fly
/// path always inits offsets and spools unlimited. A valid ofs feeds
/// pos+vel+accel; a miss holds on zero vel/accel. Face-lead and
/// dir-of-flight use `length_squared > 1`, not `>=`. Same-as-lead is
/// `radians` of the lead heading and rate. Controllers always update
/// on the fly path.
#[must_use]
pub fn follow_run(view: &FollowRunView) -> FollowRun {
    if view.disarmed_or_landed {
        return FollowRun {
            exit: FollowRunExit::Disarmed,
        };
    }

    let mut yaw_rad = view.att_yaw_rad;
    let mut yaw_rate_rads = 0.0;
    let mut yaw_source = FollowYawSource::AttTarget;

    let (input, pos_ned_m, vel_ned_ms, accel_ned_mss) = if view.ofs_valid {
        match view.yaw_behave {
            FollowYawBehave::FaceLeadVehicle => {
                if view.target_pva_ok {
                    let vec_n = view.lead_pos_ned_m[0] - view.pos_target_ned_m[0];
                    let vec_e = view.lead_pos_ned_m[1] - view.pos_target_ned_m[1];
                    if vec_n * vec_n + vec_e * vec_e > FOLLOW_YAW_LENGTH_SQ_MIN {
                        yaw_rad = Vector2f::new(vec_n, vec_e).angle();
                        yaw_source = FollowYawSource::FaceLead;
                    }
                }
            }
            FollowYawBehave::SameAsLeadVehicle => {
                yaw_rad = radians(view.target_heading_deg);
                yaw_rate_rads = radians(view.target_heading_rate_degs);
                yaw_source = FollowYawSource::SameAsLead;
            }
            FollowYawBehave::DirOfFlight => {
                let vel_n = view.vel_ofs_ned_ms[0];
                let vel_e = view.vel_ofs_ned_ms[1];
                if vel_n * vel_n + vel_e * vel_e > FOLLOW_YAW_LENGTH_SQ_MIN {
                    yaw_rad = Vector2f::new(vel_n, vel_e).angle();
                    yaw_source = FollowYawSource::DirOfFlight;
                }
            }
            FollowYawBehave::None => {}
        }
        (
            FollowPosInput::PosVelAccel,
            view.pos_ofs_ned_m,
            view.vel_ofs_ned_ms,
            view.accel_ofs_ned_mss,
        )
    } else {
        (
            FollowPosInput::VelAccelHold,
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        )
    };

    FollowRun {
        exit: FollowRunExit::Flew {
            init_offsets: true,
            input,
            yaw_source,
            yaw_rad,
            yaw_rate_rads,
            pos_ned_m,
            vel_ned_ms,
            accel_ned_mss,
        },
    }
}
