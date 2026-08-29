//! `AC_Fence` type bits, enable leftover, circle / alt-max / alt-min checks, and `check()` orchestration.

use ap_math::location::AltFrame;
use ap_math::scalar::{is_positive, is_zero};
use ap_math::vector2::Vector2f;

/// High-alt fence. Upstream `AC_FENCE_TYPE_ALT_MAX`.
pub const TYPE_ALT_MAX: u8 = 1;
/// Circular horizontal fence centered on home. Upstream `AC_FENCE_TYPE_CIRCLE`.
pub const TYPE_CIRCLE: u8 = 2;
/// Polygon inclusion / exclusion. Upstream `AC_FENCE_TYPE_POLYGON`.
pub const TYPE_POLYGON: u8 = 4;
/// Low-alt / floor fence. Upstream `AC_FENCE_TYPE_ALT_MIN`.
pub const TYPE_ALT_MIN: u8 = 8;

/// Fences enabled on arming. Upstream `AC_FENCE_ARMING_FENCES`.
pub const ARMING_FENCES: u8 = TYPE_ALT_MAX | TYPE_CIRCLE | TYPE_POLYGON;
/// Every fence type bit. Upstream `AC_FENCE_ALL_FENCES`.
pub const TYPE_ALL: u8 = ARMING_FENCES | TYPE_ALT_MIN;

/// Copter / Sub `FENCE_TYPE` default. Upstream `AC_FENCE_TYPE_DEFAULT` else-arm.
pub const FENCE_TYPE_DEFAULT_COPTER: u8 = TYPE_ALT_MAX | TYPE_CIRCLE | TYPE_POLYGON;
/// Plane `FENCE_TYPE` default.
pub const FENCE_TYPE_DEFAULT_PLANE: u8 = TYPE_POLYGON;
/// Rover `FENCE_TYPE` default.
pub const FENCE_TYPE_DEFAULT_ROVER: u8 = TYPE_CIRCLE | TYPE_POLYGON;

/// Default max altitude, metres. Upstream `AC_FENCE_ALT_MAX_DEFAULT`.
pub const ALT_MAX_DEFAULT_M: f32 = 100.0;
/// Default min altitude, metres. Upstream `AC_FENCE_ALT_MIN_DEFAULT`.
pub const ALT_MIN_DEFAULT_M: f32 = -10.0;
/// Default circle radius, metres. Upstream `AC_FENCE_CIRCLE_RADIUS_DEFAULT`.
pub const CIRCLE_RADIUS_DEFAULT_M: f32 = 300.0;
/// Default margin, metres. Upstream `AC_FENCE_MARGIN_DEFAULT`.
pub const MARGIN_DEFAULT_M: f32 = 2.0;
/// After an alt-max breach, rebuild the backup this many metres higher.
pub const ALT_MAX_BACKUP_DISTANCE_M: f32 = 20.0;
/// After an alt-min breach, rebuild the backup this many metres lower.
pub const ALT_MIN_BACKUP_DISTANCE_M: f32 = 20.0;
/// Copter / Sub circle backup step. Upstream `AC_FENCE_CIRCLE_RADIUS_BACKUP_DISTANCE`.
pub const CIRCLE_RADIUS_BACKUP_DISTANCE_COPTER_M: f32 = 20.0;
/// Plane circle backup step.
pub const CIRCLE_RADIUS_BACKUP_DISTANCE_PLANE_M: f32 = 100.0;
/// Distance outside the fence at which the vehicle should give up.
///
/// Upstream `AC_FENCE_GIVE_UP_DISTANCE`. The library does not consume this;
/// the vehicle does.
pub const GIVE_UP_DISTANCE_M: f32 = 100.0;
/// Pilot recovery window. Upstream `AC_FENCE_MANUAL_RECOVERY_TIME_MIN`.
pub const MANUAL_RECOVERY_TIME_MIN_MS: u32 = 10_000;

/// Upstream `AC_Fence::Action`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Action {
    /// 0 — report to GCS, no mode change.
    ReportOnly = 0,
    /// 1 — RTL, and land if that fails.
    RtlAndLand = 1,
    /// 2 — always land. Copter / Rover; not a Plane `@Values` token.
    AlwaysLand = 2,
    /// 3 — SmartRTL, else RTL, else land.
    SmartRtl = 3,
    /// 4 — brake, else land.
    Brake = 4,
    /// 5 — SmartRTL, else land.
    SmartRtlOrLand = 5,
    /// 6 — Guided to the fence return point.
    Guided = 6,
    /// 7 — Guided, pilot keeps throttle.
    GuidedThrottlePass = 7,
    /// 8 — fixed-wing autoland if it can start, else RTL.
    AutolandOrRtl = 8,
}

impl Action {
    /// Decode `FENCE_ACTION`. Unknown values are `None`.
    #[must_use]
    pub const fn from_param(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::ReportOnly),
            1 => Some(Self::RtlAndLand),
            2 => Some(Self::AlwaysLand),
            3 => Some(Self::SmartRtl),
            4 => Some(Self::Brake),
            5 => Some(Self::SmartRtlOrLand),
            6 => Some(Self::Guided),
            7 => Some(Self::GuidedThrottlePass),
            8 => Some(Self::AutolandOrRtl),
            _ => None,
        }
    }

    /// Upstream `AP_GROUPINFO` default, `Action::RTL_AND_LAND`.
    #[must_use]
    pub const fn default_param() -> Self {
        Self::RtlAndLand
    }
}

/// Upstream `AC_Fence::AutoEnable`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AutoEnable {
    /// 0 — never auto-enable.
    AlwaysDisabled = 0,
    /// 1 — enable after auto takeoff.
    EnableOnAutoTakeoff = 1,
    /// 2 — enable on takeoff, disable the floor on landing.
    EnableDisableFloorOnly = 2,
    /// 3 — enable on arming.
    OnlyWhenArmed = 3,
}

/// Manual min-alt leftover. Upstream `AC_Fence::MinAltState`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MinAltState {
    /// No manual override. Upstream `DEFAULT`.
    Default,
    /// User enabled the floor. Upstream `MANUALLY_ENABLED`.
    ManuallyEnabled,
    /// User disabled the floor. Upstream `MANUALLY_DISABLED`.
    ManuallyDisabled,
}

/// What [`Fence::enable`] stored and asked the logger for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EnableLeftover {
    /// Bits that flipped. The C++ return value.
    pub changed_mask: u8,
    /// `_enabled_fences` after the call.
    pub enabled_fences: u8,
    /// `_min_alt_state` after the call.
    pub min_alt_state: MinAltState,
    /// `clear_breach` leftover when disabling. Zero when enabling.
    pub clear_breach_mask: u8,
    /// Explicit user disable resets `_manual_recovery_start_ms`.
    pub reset_manual_recovery: bool,
    /// Leftover of `Write_Event(FENCE_ENABLE)` / `FENCE_DISABLE`.
    pub log_enable: Option<bool>,
    /// Leftover of `FENCE_ALT_MAX_ENABLE` / `DISABLE`.
    pub log_alt_max: Option<bool>,
    /// Leftover of `FENCE_CIRCLE_ENABLE` / `DISABLE`.
    pub log_circle: Option<bool>,
    /// Leftover of `FENCE_ALT_MIN_ENABLE` / `DISABLE`.
    pub log_alt_min: Option<bool>,
    /// Leftover of `FENCE_POLYGON_ENABLE` / `DISABLE`.
    pub log_polygon: Option<bool>,
}

/// Inputs [`Fence::check_fence_circle`] reads from AHRS.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CheckCircleContext {
    /// `ahrs.get_relative_position_NE_home`. `None` keeps the last home
    /// distance — "we (may) remain breached if we can't update home".
    pub ne_home_m: Option<(f32, f32)>,
    /// `AP_HAL::millis()` leftover of `record_breach`.
    pub now_ms: u32,
}

impl Default for CheckCircleContext {
    fn default() -> Self {
        Self {
            ne_home_m: Some((0.0, 0.0)),
            now_ms: 1_001,
        }
    }
}

/// Circle-check leftover, upstream `AC_Fence::check_fence_circle`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CheckCircleLeftover {
    /// Fresh breach (or backup-fence re-breach). The C++ return.
    pub newly_breached: bool,
    /// Whether the type bit was in [`Fence::get_enabled_fences`].
    pub enabled: bool,
    /// Leftover of `ahrs.get_relative_position_NE_home`.
    pub need_ne_home: bool,
    /// `_home_distance_m` after the call.
    pub home_distance_m: f32,
    /// `_circle_breach_distance_m` after the call.
    pub breach_distance_m: f32,
    /// `_circle_radius_backup_m` after the call.
    pub backup_radius_m: f32,
    /// `_circle_breach_direction` after the call.
    pub breach_direction_ne_m: Vector2f,
    /// Leftover of `record_breach`.
    pub recorded_breach: bool,
    /// Leftover of `GCS_SEND_MESSAGE(MSG_FENCE_STATUS)`.
    pub need_gcs_fence_status: bool,
    /// Margin bit set this call.
    pub margin_breached: bool,
    /// `clear_breach` ran because the vehicle came back inside.
    pub cleared_breach: bool,
}

/// Inputs [`Fence::check_fence_alt_max`] reads from AHRS / location.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CheckAltMaxContext {
    /// `get_alt_in_alt_max_frame_m`. `None` is a fresh breach without
    /// `record_breach`.
    pub alt_u_m: Option<f32>,
    /// Leftover of `ahrs.get_home().get_alt_m(ABSOLUTE)` when the max
    /// frame is [`AltFrame::Absolute`].
    pub home_alt_amsl_m: f32,
    /// `AP_HAL::millis()` leftover of `record_breach`.
    pub now_ms: u32,
}

impl Default for CheckAltMaxContext {
    fn default() -> Self {
        Self {
            alt_u_m: Some(0.0),
            home_alt_amsl_m: 0.0,
            now_ms: 1_001,
        }
    }
}

/// Alt-max leftover, upstream `AC_Fence::check_fence_alt_max`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CheckAltMaxLeftover {
    /// Fresh breach (or backup-fence re-breach), or alt unavailable.
    pub newly_breached: bool,
    /// Whether the type bit was in [`Fence::get_enabled_fences`].
    pub enabled: bool,
    /// Altitude frame was unavailable. Returns true without `record_breach`.
    pub alt_unavailable: bool,
    /// Leftover of `get_alt_in_alt_max_frame_m`.
    pub need_alt_in_frame: bool,
    /// Leftover of home AMSL when [`Fence::alt_max_type`] is Absolute.
    pub need_home_alt: bool,
    /// `_alt_max_breach_distance_m` after the call.
    pub breach_distance_m: f32,
    /// `_safe_relhome_alt_max_m` after the call.
    pub safe_relhome_alt_max_m: f32,
    /// `_alt_max_backup_m` after the call.
    pub backup_alt_m: f32,
    /// Leftover of `record_breach`.
    pub recorded_breach: bool,
    /// Leftover of `GCS_SEND_MESSAGE(MSG_FENCE_STATUS)`.
    pub need_gcs_fence_status: bool,
    /// Margin bit set this call.
    pub margin_breached: bool,
    /// `clear_breach` ran because the vehicle came back under the ceiling.
    pub cleared_breach: bool,
}

/// Inputs [`Fence::check_fence_alt_min`] reads from AHRS / location.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CheckAltMinContext {
    /// `get_alt_in_alt_min_frame_m`. `None` is a fresh breach without
    /// `record_breach`.
    pub alt_u_m: Option<f32>,
    /// Leftover of `ahrs.get_home().get_alt_m(ABSOLUTE)` when the min
    /// frame is [`AltFrame::Absolute`].
    pub home_alt_amsl_m: f32,
    /// `AP_HAL::millis()` leftover of `record_breach`.
    pub now_ms: u32,
}

impl Default for CheckAltMinContext {
    fn default() -> Self {
        Self {
            alt_u_m: Some(0.0),
            home_alt_amsl_m: 0.0,
            now_ms: 1_001,
        }
    }
}

/// Alt-min leftover, upstream `AC_Fence::check_fence_alt_min`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CheckAltMinLeftover {
    /// Fresh breach (or backup-fence re-breach), or alt unavailable.
    pub newly_breached: bool,
    /// Whether the type bit was in [`Fence::get_enabled_fences`].
    pub enabled: bool,
    /// Altitude frame was unavailable. Returns true without `record_breach`.
    pub alt_unavailable: bool,
    /// Leftover of `get_alt_in_alt_min_frame_m`.
    pub need_alt_in_frame: bool,
    /// Leftover of home AMSL when [`Fence::alt_min_type`] is Absolute.
    pub need_home_alt: bool,
    /// `_alt_min_breach_distance_m` after the call.
    pub breach_distance_m: f32,
    /// `_safe_relhome_alt_min_m` after the call.
    pub safe_relhome_alt_min_m: f32,
    /// `_alt_min_backup_m` after the call.
    pub backup_alt_m: f32,
    /// Leftover of `record_breach`.
    pub recorded_breach: bool,
    /// Leftover of `GCS_SEND_MESSAGE(MSG_FENCE_STATUS)`.
    pub need_gcs_fence_status: bool,
    /// Margin bit set this call.
    pub margin_breached: bool,
    /// `clear_breach` ran because the vehicle climbed back above the floor.
    pub cleared_breach: bool,
}


/// Inputs [`Fence::check`] reads from the vehicle / AHRS.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CheckContext {
    /// `disable_auto_fences` — landing / auto-disable request.
    pub disable_auto_fences: bool,
    /// `AP_HAL::millis()` leftover of `record_breach` and recovery.
    pub now_ms: u32,
    /// Leftover of `ahrs.get_location(_last_fence_check_loc)`.
    pub location_valid: bool,
    /// `ahrs.get_relative_position_NE_home` leftover for the circle checker.
    pub ne_home_m: Option<(f32, f32)>,
    /// `get_alt_in_alt_max_frame_m` leftover.
    pub alt_max_u_m: Option<f32>,
    /// `get_alt_in_alt_min_frame_m` leftover, also the floor auto-enable alt.
    pub alt_min_u_m: Option<f32>,
    /// Home AMSL when an alt frame is [`AltFrame::Absolute`].
    pub home_alt_amsl_m: f32,
}

impl Default for CheckContext {
    fn default() -> Self {
        Self {
            disable_auto_fences: false,
            now_ms: 1_001,
            location_valid: true,
            ne_home_m: Some((0.0, 0.0)),
            alt_max_u_m: Some(0.0),
            alt_min_u_m: Some(0.0),
            home_alt_amsl_m: 0.0,
        }
    }
}

/// `AC_Fence::check` leftover. Polygon EEPROM stays later, so the poly
/// checker is not invoked.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CheckLeftover {
    /// Newly breached types. The C++ return. Zero during recovery.
    pub new_breaches: u8,
    /// `get_auto_disable_fences` when `disable_auto_fences` is set.
    pub disabled_fences: u8,
    /// `disabled_fences & _enabled_fences`, then alt-min stripped if
    /// the floor was manually enabled. Used for the GCS print leftover.
    pub fences_to_disable: u8,
    /// Bits `clear_breach(~_configured_fences)` dropped.
    pub cleared_unconfigured_breach: u8,
    /// Bits `clear_breach(fences_to_disable)` dropped.
    pub cleared_disabled_breach: u8,
    /// Leftover of `print_fence_message("auto-disabled", ...)`.
    pub auto_disabled_message: Option<u8>,
    /// Early return: nothing enabled / auto / alt-min, or no `FENCE_TYPE`.
    pub skipped: bool,
    /// Leftover of `ahrs.get_location`.
    pub need_location: bool,
    /// `_last_fence_check_loc_valid` after the call.
    pub last_check_loc_valid: bool,
    /// `enable(false, disabled_fences, false)` leftover.
    pub disable_changed_mask: u8,
    /// Alt-max checker leftover. Idle when [`Self::skipped`].
    pub alt_max: CheckAltMaxLeftover,
    /// Alt-min checker leftover. Idle when [`Self::skipped`].
    pub alt_min: CheckAltMinLeftover,
    /// Circle checker leftover. Idle when [`Self::skipped`].
    pub circle: CheckCircleLeftover,
    /// Polygon checker is the loader leftover. Always false this slice.
    pub polygon_checked: bool,
    /// `auto_enable_fence_floor` actually flipped the floor on.
    pub floor_auto_enabled: bool,
    /// Floor leftover could not get an altitude. C++ returns true then.
    pub floor_alt_unavailable: bool,
    /// Leftover of `GCS_SEND_TEXT` "Min Alt fence enabled (auto enable)".
    pub need_gcs_floor_notice: bool,
    /// Recovery window is still open; return is forced to 0.
    pub manual_recovery_active: bool,
    /// Recovery window expired this call; `_manual_recovery_start_ms` reset.
    pub manual_recovery_expired: bool,
}

/// Geofence state. Upstream `AC_Fence` without the poly loader.
#[derive(Debug, Clone, PartialEq)]
pub struct Fence {
    enabled_fences: u8,
    configured_fences: u8,
    /// Inclusion / exclusion vertex count. Zero until the loader leftover.
    poly_fence_count: u8,
    action: Action,
    alt_max_m: f32,
    alt_min_m: f32,
    circle_radius_m: f32,
    margin_m: f32,
    margin_ne_m: f32,
    alt_max_type: AltFrame,
    alt_min_type: AltFrame,
    circle_backup_step_m: f32,
    alt_max_backup_m: f32,
    alt_min_backup_m: f32,
    circle_radius_backup_m: f32,
    alt_max_breach_distance_m: f32,
    alt_min_breach_distance_m: f32,
    circle_breach_distance_m: f32,
    circle_breach_direction: Vector2f,
    home_distance_m: f32,
    safe_relhome_alt_max_m: f32,
    safe_relhome_alt_min_m: f32,
    breached_fences: u8,
    breached_fence_margins: u8,
    breach_time_ms: u32,
    margin_breach_time_ms: u32,
    breach_count: u16,
    last_breach_notify_sent_ms: u32,
    min_alt_state: MinAltState,
    manual_recovery_start_ms: u32,
    auto_enabled: AutoEnable,
    /// `FENCE_ENABLE` param leftover. Distinct from [`Self::enabled`].
    enable_param: bool,
    last_fence_check_loc_valid: bool,
}

impl Default for Fence {
    fn default() -> Self {
        Self::new()
    }
}

impl Fence {
    /// Constructor leftover with Copter defaults and `FENCE_ENABLE` off.
    ///
    /// Upstream writes `_enabled_fences` from the configured mask only
    /// when the ENABLE param is already true. The default param is 0.
    #[must_use]
    pub fn new() -> Self {
        Self::from_params(false, FENCE_TYPE_DEFAULT_COPTER)
    }

    /// Constructor leftover of `_enabled` / `_configured_fences`.
    ///
    /// When `enable_param` is set, `_enabled_fences` is the configured
    /// mask with alt-min stripped — the floor waits for auto-enable.
    #[must_use]
    pub fn from_params(enable_param: bool, configured_fences: u8) -> Self {
        let enabled_fences = if enable_param {
            configured_fences & !TYPE_ALT_MIN
        } else {
            0
        };
        Self {
            enabled_fences,
            configured_fences,
            poly_fence_count: 0,
            action: Action::default_param(),
            alt_max_m: ALT_MAX_DEFAULT_M,
            alt_min_m: ALT_MIN_DEFAULT_M,
            circle_radius_m: CIRCLE_RADIUS_DEFAULT_M,
            margin_m: MARGIN_DEFAULT_M,
            margin_ne_m: 0.0,
            alt_max_type: AltFrame::AboveHome,
            alt_min_type: AltFrame::AboveHome,
            circle_backup_step_m: CIRCLE_RADIUS_BACKUP_DISTANCE_COPTER_M,
            alt_max_backup_m: 0.0,
            alt_min_backup_m: 0.0,
            circle_radius_backup_m: 0.0,
            alt_max_breach_distance_m: 0.0,
            alt_min_breach_distance_m: 0.0,
            circle_breach_distance_m: 0.0,
            circle_breach_direction: Vector2f::zero(),
            home_distance_m: 0.0,
            safe_relhome_alt_max_m: 0.0,
            safe_relhome_alt_min_m: 0.0,
            breached_fences: 0,
            breached_fence_margins: 0,
            breach_time_ms: 0,
            margin_breach_time_ms: 0,
            breach_count: 0,
            last_breach_notify_sent_ms: 0,
            min_alt_state: MinAltState::Default,
            manual_recovery_start_ms: 0,
            auto_enabled: AutoEnable::AlwaysDisabled,
            enable_param,
            last_fence_check_loc_valid: false,
        }
    }

    /// `_configured_fences`.
    #[must_use]
    pub const fn configured_fences(&self) -> u8 {
        self.configured_fences
    }

    /// Set `FENCE_TYPE`. Does not change `_enabled_fences`.
    pub fn set_configured_fences(&mut self, mask: u8) {
        self.configured_fences = mask;
    }

    /// `_enabled_fences` raw bits, including types `present()` would hide.
    #[must_use]
    pub const fn enabled_fences_raw(&self) -> u8 {
        self.enabled_fences
    }

    /// `enabled()` — any raw enabled bit.
    #[must_use]
    pub const fn enabled(&self) -> bool {
        self.enabled_fences != 0
    }

    /// `present()` without the poly loader: circle / alt bits always, plus
    /// polygon only when [`Self::poly_fence_count`] is non-zero.
    #[must_use]
    pub const fn present(&self) -> u8 {
        let mut mask = TYPE_CIRCLE | TYPE_ALT_MIN | TYPE_ALT_MAX;
        if self.poly_fence_count > 0 {
            mask |= TYPE_POLYGON;
        }
        self.configured_fences & mask
    }

    /// `get_enabled_fences` — `_enabled_fences & present()`.
    #[must_use]
    pub const fn get_enabled_fences(&self) -> u8 {
        self.enabled_fences & self.present()
    }

    /// Inclusion / exclusion vertex count leftover. Stays 0 this slice.
    #[must_use]
    pub const fn poly_fence_count(&self) -> u8 {
        self.poly_fence_count
    }

    /// `get_action`.
    #[must_use]
    pub const fn action(&self) -> Action {
        self.action
    }

    /// `get_breaches`.
    #[must_use]
    pub const fn get_breaches(&self) -> u8 {
        self.breached_fences
    }

    /// `get_margin_breaches`.
    #[must_use]
    pub const fn get_margin_breaches(&self) -> u8 {
        self.breached_fence_margins
    }

    /// `get_breach_count`.
    #[must_use]
    pub const fn get_breach_count(&self) -> u16 {
        self.breach_count
    }

    /// `get_breach_time`.
    #[must_use]
    pub const fn get_breach_time(&self) -> u32 {
        self.breach_time_ms
    }

    /// `get_radius_m`.
    #[must_use]
    pub const fn circle_radius_m(&self) -> f32 {
        self.circle_radius_m
    }

    /// Set `FENCE_RADIUS`.
    pub fn set_circle_radius_m(&mut self, radius_m: f32) {
        self.circle_radius_m = radius_m;
    }

    /// `get_safe_alt_max_m` — `_alt_max_m - _margin_m`.
    #[must_use]
    pub fn get_safe_alt_max_m(&self) -> f32 {
        self.alt_max_m - self.margin_m
    }

    /// `_alt_max_m`.
    #[must_use]
    pub const fn alt_max_m(&self) -> f32 {
        self.alt_max_m
    }

    /// Set `FENCE_ALT_MAX`.
    pub fn set_alt_max_m(&mut self, alt_m: f32) {
        self.alt_max_m = alt_m;
    }

    /// `FENCE_ALT_MAX_TP`.
    #[must_use]
    pub const fn alt_max_type(&self) -> AltFrame {
        self.alt_max_type
    }

    /// Set `FENCE_ALT_MAX_TP`.
    pub fn set_alt_max_type(&mut self, frame: AltFrame) {
        self.alt_max_type = frame;
    }

    /// `get_safe_alt_min_m` — `_alt_min_m + _margin_m`.
    #[must_use]
    pub fn get_safe_alt_min_m(&self) -> f32 {
        self.alt_min_m + self.margin_m
    }

    /// `_alt_min_m`.
    #[must_use]
    pub const fn alt_min_m(&self) -> f32 {
        self.alt_min_m
    }

    /// Set `FENCE_ALT_MIN`.
    pub fn set_alt_min_m(&mut self, alt_m: f32) {
        self.alt_min_m = alt_m;
    }

    /// `FENCE_ALT_MIN_TP`.
    #[must_use]
    pub const fn alt_min_type(&self) -> AltFrame {
        self.alt_min_type
    }

    /// Set `FENCE_ALT_MIN_TP`.
    pub fn set_alt_min_type(&mut self, frame: AltFrame) {
        self.alt_min_type = frame;
    }

    /// `get_margin_ne_m` — `FENCE_MARGIN_XY` when positive, else `FENCE_MARGIN`.
    #[must_use]
    pub fn get_margin_ne_m(&self) -> f32 {
        if is_positive(self.margin_ne_m) {
            self.margin_ne_m
        } else {
            self.margin_m
        }
    }

    /// Set `FENCE_MARGIN`.
    pub fn set_margin_m(&mut self, margin_m: f32) {
        self.margin_m = margin_m;
    }

    /// Set `FENCE_MARGIN_XY`.
    pub fn set_margin_ne_m(&mut self, margin_m: f32) {
        self.margin_ne_m = margin_m;
    }

    /// Plane vs Copter circle backup step.
    pub fn set_circle_backup_step_m(&mut self, step_m: f32) {
        self.circle_backup_step_m = step_m;
    }

    /// `_min_alt_state`.
    #[must_use]
    pub const fn min_alt_state(&self) -> MinAltState {
        self.min_alt_state
    }

    /// `_manual_recovery_start_ms`.
    #[must_use]
    pub const fn manual_recovery_start_ms(&self) -> u32 {
        self.manual_recovery_start_ms
    }

    /// Seed a recovery window so disable leftover can clear it.
    pub fn set_manual_recovery_start_ms(&mut self, now_ms: u32) {
        self.manual_recovery_start_ms = now_ms;
    }

    /// `FENCE_ENABLE` param leftover.
    #[must_use]
    pub const fn enable_param(&self) -> bool {
        self.enable_param
    }

    /// `auto_enabled()`.
    #[must_use]
    pub const fn auto_enabled(&self) -> AutoEnable {
        self.auto_enabled
    }

    /// Set `FENCE_AUTOENABLE`.
    pub fn set_auto_enabled(&mut self, value: AutoEnable) {
        self.auto_enabled = value;
    }

    /// `_last_fence_check_loc_valid`.
    #[must_use]
    pub const fn last_fence_check_loc_valid(&self) -> bool {
        self.last_fence_check_loc_valid
    }

    /// `floor_enabled` — `_enabled_fences & TYPE_ALT_MIN`.
    #[must_use]
    pub const fn floor_enabled(&self) -> bool {
        self.enabled_fences & TYPE_ALT_MIN != 0
    }

    /// `AC_Fence::get_auto_disable_fences`.
    #[must_use]
    pub const fn get_auto_disable_fences(&self) -> u8 {
        let mut auto_disable = match self.auto_enabled {
            AutoEnable::EnableOnAutoTakeoff => TYPE_ALL,
            AutoEnable::EnableDisableFloorOnly
            | AutoEnable::OnlyWhenArmed
            | AutoEnable::AlwaysDisabled => TYPE_ALT_MIN,
        };
        if matches!(self.min_alt_state, MinAltState::ManuallyEnabled) {
            auto_disable &= !TYPE_ALT_MIN;
        }
        auto_disable
    }

    /// `enable_configured` leftover.
    pub fn enable_configured(&mut self, value: bool) -> EnableLeftover {
        self.enable(value, self.configured_fences, true)
    }

    /// `AC_Fence::enable` leftover.
    ///
    /// Only configured bits in `fence_types` move. The min-alt manual
    /// state is written *before* the no-change early return.
    pub fn enable(
        &mut self,
        value: bool,
        fence_types: u8,
        update_auto_enable: bool,
    ) -> EnableLeftover {
        let fences = self.configured_fences & fence_types;
        let mut enabled_fences = self.enabled_fences;
        if value {
            enabled_fences |= fences;
        } else {
            enabled_fences &= !fences;
        }

        if update_auto_enable && (fences & TYPE_ALT_MIN) != 0 {
            self.min_alt_state = if value {
                MinAltState::ManuallyEnabled
            } else {
                MinAltState::ManuallyDisabled
            };
        }

        let fences_to_change = self.enabled_fences ^ enabled_fences;
        if fences_to_change == 0 {
            return EnableLeftover {
                changed_mask: 0,
                enabled_fences: self.enabled_fences,
                min_alt_state: self.min_alt_state,
                clear_breach_mask: 0,
                reset_manual_recovery: false,
                log_enable: None,
                log_alt_max: None,
                log_circle: None,
                log_alt_min: None,
                log_polygon: None,
            };
        }

        self.enabled_fences = enabled_fences;

        let mut clear_breach_mask = 0;
        let mut reset_manual_recovery = false;
        if !value {
            self.clear_breach(fences_to_change);
            clear_breach_mask = fences_to_change;
            if update_auto_enable {
                self.manual_recovery_start_ms = 0;
                reset_manual_recovery = true;
            }
        }

        EnableLeftover {
            changed_mask: fences_to_change,
            enabled_fences: self.enabled_fences,
            min_alt_state: self.min_alt_state,
            clear_breach_mask,
            reset_manual_recovery,
            log_enable: Some(value),
            log_alt_max: log_bit(fences_to_change, TYPE_ALT_MAX, value),
            log_circle: log_bit(fences_to_change, TYPE_CIRCLE, value),
            log_alt_min: log_bit(fences_to_change, TYPE_ALT_MIN, value),
            log_polygon: log_bit(fences_to_change, TYPE_POLYGON, value),
        }
    }

    /// `AC_Fence::check_fence_circle` leftover.
    pub fn check_fence_circle(&mut self, ctx: CheckCircleContext) -> CheckCircleLeftover {
        if self.get_enabled_fences() & TYPE_CIRCLE == 0 {
            return CheckCircleLeftover {
                newly_breached: false,
                enabled: false,
                need_ne_home: false,
                home_distance_m: self.home_distance_m,
                breach_distance_m: self.circle_breach_distance_m,
                backup_radius_m: self.circle_radius_backup_m,
                breach_direction_ne_m: self.circle_breach_direction,
                recorded_breach: false,
                need_gcs_fence_status: false,
                margin_breached: (self.breached_fence_margins & TYPE_CIRCLE) != 0,
                cleared_breach: false,
            };
        }

        let mut need_ne_home = false;
        if let Some((n_m, e_m)) = ctx.ne_home_m {
            need_ne_home = true;
            let home = Vector2f::new(n_m, e_m);
            self.home_distance_m = home.length();
            if is_zero(home.length_squared()) {
                self.circle_breach_direction = Vector2f::new(self.circle_radius_m, 0.0);
            } else if let Some(unit) = home.normalized() {
                self.circle_breach_direction = unit * self.circle_radius_m - home;
            }
        }

        self.circle_breach_distance_m = self.home_distance_m - self.circle_radius_m;

        if self.home_distance_m >= self.circle_radius_m {
            let already = (self.breached_fences & TYPE_CIRCLE) != 0;
            let backup_hit = !is_zero(self.circle_radius_backup_m)
                && self.home_distance_m >= self.circle_radius_backup_m;
            if !already || backup_hit {
                let gcs = self.record_breach(TYPE_CIRCLE, ctx.now_ms);
                self.circle_radius_backup_m = self.home_distance_m + self.circle_backup_step_m;
                return CheckCircleLeftover {
                    newly_breached: true,
                    enabled: true,
                    need_ne_home,
                    home_distance_m: self.home_distance_m,
                    breach_distance_m: self.circle_breach_distance_m,
                    backup_radius_m: self.circle_radius_backup_m,
                    breach_direction_ne_m: self.circle_breach_direction,
                    recorded_breach: true,
                    need_gcs_fence_status: gcs,
                    margin_breached: (self.breached_fence_margins & TYPE_CIRCLE) != 0,
                    cleared_breach: false,
                };
            }
            return CheckCircleLeftover {
                newly_breached: false,
                enabled: true,
                need_ne_home,
                home_distance_m: self.home_distance_m,
                breach_distance_m: self.circle_breach_distance_m,
                backup_radius_m: self.circle_radius_backup_m,
                breach_direction_ne_m: self.circle_breach_direction,
                recorded_breach: false,
                need_gcs_fence_status: false,
                margin_breached: (self.breached_fence_margins & TYPE_CIRCLE) != 0,
                cleared_breach: false,
            };
        }

        if self.home_distance_m >= self.circle_radius_m - self.get_margin_ne_m() {
            self.record_margin_breach(TYPE_CIRCLE, ctx.now_ms);
        } else {
            self.clear_margin_breach(TYPE_CIRCLE);
        }

        let mut cleared_breach = false;
        if self.breached_fences & TYPE_CIRCLE != 0 {
            self.clear_breach(TYPE_CIRCLE);
            self.circle_radius_backup_m = 0.0;
            cleared_breach = true;
        }

        CheckCircleLeftover {
            newly_breached: false,
            enabled: true,
            need_ne_home,
            home_distance_m: self.home_distance_m,
            breach_distance_m: self.circle_breach_distance_m,
            backup_radius_m: self.circle_radius_backup_m,
            breach_direction_ne_m: self.circle_breach_direction,
            recorded_breach: false,
            need_gcs_fence_status: false,
            margin_breached: (self.breached_fence_margins & TYPE_CIRCLE) != 0,
            cleared_breach,
        }
    }

    /// `AC_Fence::check_fence_alt_max` leftover.
    pub fn check_fence_alt_max(&mut self, ctx: CheckAltMaxContext) -> CheckAltMaxLeftover {
        if self.get_enabled_fences() & TYPE_ALT_MAX == 0 {
            return CheckAltMaxLeftover {
                newly_breached: false,
                enabled: false,
                alt_unavailable: false,
                need_alt_in_frame: false,
                need_home_alt: false,
                breach_distance_m: self.alt_max_breach_distance_m,
                safe_relhome_alt_max_m: self.safe_relhome_alt_max_m,
                backup_alt_m: self.alt_max_backup_m,
                recorded_breach: false,
                need_gcs_fence_status: false,
                margin_breached: (self.breached_fence_margins & TYPE_ALT_MAX) != 0,
                cleared_breach: false,
            };
        }

        let Some(curr_alt_u_m) = ctx.alt_u_m else {
            return CheckAltMaxLeftover {
                newly_breached: true,
                enabled: true,
                alt_unavailable: true,
                need_alt_in_frame: true,
                need_home_alt: false,
                breach_distance_m: self.alt_max_breach_distance_m,
                safe_relhome_alt_max_m: self.safe_relhome_alt_max_m,
                backup_alt_m: self.alt_max_backup_m,
                recorded_breach: false,
                need_gcs_fence_status: false,
                margin_breached: (self.breached_fence_margins & TYPE_ALT_MAX) != 0,
                cleared_breach: false,
            };
        };

        self.alt_max_breach_distance_m = curr_alt_u_m - self.alt_max_m;

        let need_home_alt = self.alt_max_type == AltFrame::Absolute;
        if need_home_alt {
            self.safe_relhome_alt_max_m = self.alt_max_m - ctx.home_alt_amsl_m - self.margin_m;
        } else {
            self.safe_relhome_alt_max_m = self.alt_max_m - self.margin_m;
        }

        if curr_alt_u_m >= self.alt_max_m {
            let already = (self.breached_fences & TYPE_ALT_MAX) != 0;
            let backup_hit =
                !is_zero(self.alt_max_backup_m) && curr_alt_u_m >= self.alt_max_backup_m;
            if !already || backup_hit {
                let gcs = self.record_breach(TYPE_ALT_MAX, ctx.now_ms);
                self.alt_max_backup_m = curr_alt_u_m + ALT_MAX_BACKUP_DISTANCE_M;
                return CheckAltMaxLeftover {
                    newly_breached: true,
                    enabled: true,
                    alt_unavailable: false,
                    need_alt_in_frame: true,
                    need_home_alt,
                    breach_distance_m: self.alt_max_breach_distance_m,
                    safe_relhome_alt_max_m: self.safe_relhome_alt_max_m,
                    backup_alt_m: self.alt_max_backup_m,
                    recorded_breach: true,
                    need_gcs_fence_status: gcs,
                    margin_breached: (self.breached_fence_margins & TYPE_ALT_MAX) != 0,
                    cleared_breach: false,
                };
            }
            return CheckAltMaxLeftover {
                newly_breached: false,
                enabled: true,
                alt_unavailable: false,
                need_alt_in_frame: true,
                need_home_alt,
                breach_distance_m: self.alt_max_breach_distance_m,
                safe_relhome_alt_max_m: self.safe_relhome_alt_max_m,
                backup_alt_m: self.alt_max_backup_m,
                recorded_breach: false,
                need_gcs_fence_status: false,
                margin_breached: (self.breached_fence_margins & TYPE_ALT_MAX) != 0,
                cleared_breach: false,
            };
        }

        if curr_alt_u_m >= self.get_safe_alt_max_m() {
            self.record_margin_breach(TYPE_ALT_MAX, ctx.now_ms);
        } else {
            self.clear_margin_breach(TYPE_ALT_MAX);
        }

        let mut cleared_breach = false;
        if self.breached_fences & TYPE_ALT_MAX != 0 {
            self.clear_breach(TYPE_ALT_MAX);
            self.alt_max_backup_m = 0.0;
            cleared_breach = true;
        }

        CheckAltMaxLeftover {
            newly_breached: false,
            enabled: true,
            alt_unavailable: false,
            need_alt_in_frame: true,
            need_home_alt,
            breach_distance_m: self.alt_max_breach_distance_m,
            safe_relhome_alt_max_m: self.safe_relhome_alt_max_m,
            backup_alt_m: self.alt_max_backup_m,
            recorded_breach: false,
            need_gcs_fence_status: false,
            margin_breached: (self.breached_fence_margins & TYPE_ALT_MAX) != 0,
            cleared_breach,
        }
    }

    /// `AC_Fence::check_fence_alt_min` leftover.
    pub fn check_fence_alt_min(&mut self, ctx: CheckAltMinContext) -> CheckAltMinLeftover {
        if self.get_enabled_fences() & TYPE_ALT_MIN == 0 {
            return CheckAltMinLeftover {
                newly_breached: false,
                enabled: false,
                alt_unavailable: false,
                need_alt_in_frame: false,
                need_home_alt: false,
                breach_distance_m: self.alt_min_breach_distance_m,
                safe_relhome_alt_min_m: self.safe_relhome_alt_min_m,
                backup_alt_m: self.alt_min_backup_m,
                recorded_breach: false,
                need_gcs_fence_status: false,
                margin_breached: (self.breached_fence_margins & TYPE_ALT_MIN) != 0,
                cleared_breach: false,
            };
        }

        let Some(curr_alt_u_m) = ctx.alt_u_m else {
            return CheckAltMinLeftover {
                newly_breached: true,
                enabled: true,
                alt_unavailable: true,
                need_alt_in_frame: true,
                need_home_alt: false,
                breach_distance_m: self.alt_min_breach_distance_m,
                safe_relhome_alt_min_m: self.safe_relhome_alt_min_m,
                backup_alt_m: self.alt_min_backup_m,
                recorded_breach: false,
                need_gcs_fence_status: false,
                margin_breached: (self.breached_fence_margins & TYPE_ALT_MIN) != 0,
                cleared_breach: false,
            };
        };

        self.alt_min_breach_distance_m = self.alt_min_m - curr_alt_u_m;

        let need_home_alt = self.alt_min_type == AltFrame::Absolute;
        if need_home_alt {
            self.safe_relhome_alt_min_m = self.alt_min_m - ctx.home_alt_amsl_m - self.margin_m;
        } else {
            self.safe_relhome_alt_min_m = self.alt_min_m - self.margin_m;
        }

        if curr_alt_u_m <= self.alt_min_m {
            let already = (self.breached_fences & TYPE_ALT_MIN) != 0;
            let backup_hit =
                !is_zero(self.alt_min_backup_m) && curr_alt_u_m <= self.alt_min_backup_m;
            if !already || backup_hit {
                let gcs = self.record_breach(TYPE_ALT_MIN, ctx.now_ms);
                self.alt_min_backup_m = curr_alt_u_m - ALT_MIN_BACKUP_DISTANCE_M;
                return CheckAltMinLeftover {
                    newly_breached: true,
                    enabled: true,
                    alt_unavailable: false,
                    need_alt_in_frame: true,
                    need_home_alt,
                    breach_distance_m: self.alt_min_breach_distance_m,
                    safe_relhome_alt_min_m: self.safe_relhome_alt_min_m,
                    backup_alt_m: self.alt_min_backup_m,
                    recorded_breach: true,
                    need_gcs_fence_status: gcs,
                    margin_breached: (self.breached_fence_margins & TYPE_ALT_MIN) != 0,
                    cleared_breach: false,
                };
            }
            return CheckAltMinLeftover {
                newly_breached: false,
                enabled: true,
                alt_unavailable: false,
                need_alt_in_frame: true,
                need_home_alt,
                breach_distance_m: self.alt_min_breach_distance_m,
                safe_relhome_alt_min_m: self.safe_relhome_alt_min_m,
                backup_alt_m: self.alt_min_backup_m,
                recorded_breach: false,
                need_gcs_fence_status: false,
                margin_breached: (self.breached_fence_margins & TYPE_ALT_MIN) != 0,
                cleared_breach: false,
            };
        }

        if curr_alt_u_m <= self.get_safe_alt_min_m() {
            self.record_margin_breach(TYPE_ALT_MIN, ctx.now_ms);
        } else {
            self.clear_margin_breach(TYPE_ALT_MIN);
        }

        let mut cleared_breach = false;
        if self.breached_fences & TYPE_ALT_MIN != 0 {
            self.clear_breach(TYPE_ALT_MIN);
            self.alt_min_backup_m = 0.0;
            cleared_breach = true;
        }

        CheckAltMinLeftover {
            newly_breached: false,
            enabled: true,
            alt_unavailable: false,
            need_alt_in_frame: true,
            need_home_alt,
            breach_distance_m: self.alt_min_breach_distance_m,
            safe_relhome_alt_min_m: self.safe_relhome_alt_min_m,
            backup_alt_m: self.alt_min_backup_m,
            recorded_breach: false,
            need_gcs_fence_status: false,
            margin_breached: (self.breached_fence_margins & TYPE_ALT_MIN) != 0,
            cleared_breach,
        }
    }


    /// `AC_Fence::check` leftover.
    ///
    /// Clears stale breaches, optionally auto-disables the landing
    /// floor, then runs the alt / circle checkers already on this
    /// crate. The polygon checker stays with the loader leftover.
    /// A live manual-recovery window records breaches but returns 0.
    pub fn check(&mut self, ctx: CheckContext) -> CheckLeftover {
        let disabled_fences = if ctx.disable_auto_fences {
            self.get_auto_disable_fences()
        } else {
            0
        };
        let mut fences_to_disable = disabled_fences & self.enabled_fences;

        let before_breach = self.breached_fences;
        self.clear_breach(!self.configured_fences);
        self.clear_breach(fences_to_disable);
        self.clear_margin_breach(!self.configured_fences);
        self.clear_margin_breach(fences_to_disable);
        let cleared_unconfigured_breach = before_breach & !self.configured_fences;
        let cleared_disabled_breach = before_breach & fences_to_disable;

        if matches!(self.min_alt_state, MinAltState::ManuallyEnabled) {
            fences_to_disable &= !TYPE_ALT_MIN;
        }

        let auto_disabled_message = if fences_to_disable != 0 {
            Some(fences_to_disable)
        } else {
            None
        };

        let idle_circle = self.idle_circle_leftover();
        let idle_alt_max = self.idle_alt_max_leftover();
        let idle_alt_min = self.idle_alt_min_leftover();

        if (!self.enabled()
            && matches!(self.auto_enabled, AutoEnable::AlwaysDisabled)
            && self.configured_fences & TYPE_ALT_MIN == 0)
            || self.configured_fences == 0
        {
            return CheckLeftover {
                new_breaches: 0,
                disabled_fences,
                fences_to_disable,
                cleared_unconfigured_breach,
                cleared_disabled_breach,
                auto_disabled_message,
                skipped: true,
                need_location: false,
                last_check_loc_valid: self.last_fence_check_loc_valid,
                disable_changed_mask: 0,
                alt_max: idle_alt_max,
                alt_min: idle_alt_min,
                circle: idle_circle,
                polygon_checked: false,
                floor_auto_enabled: false,
                floor_alt_unavailable: false,
                need_gcs_floor_notice: false,
                manual_recovery_active: false,
                manual_recovery_expired: false,
            };
        }

        let disable_leftover = self.enable(false, disabled_fences, false);
        self.last_fence_check_loc_valid = ctx.location_valid;

        let mut new_breaches = 0;
        let alt_max = if disabled_fences & TYPE_ALT_MAX == 0 {
            let leftover = self.check_fence_alt_max(CheckAltMaxContext {
                alt_u_m: ctx.alt_max_u_m,
                home_alt_amsl_m: ctx.home_alt_amsl_m,
                now_ms: ctx.now_ms,
            });
            if leftover.newly_breached {
                new_breaches |= TYPE_ALT_MAX;
            }
            leftover
        } else {
            idle_alt_max
        };

        let alt_min = if disabled_fences & TYPE_ALT_MIN == 0 {
            let leftover = self.check_fence_alt_min(CheckAltMinContext {
                alt_u_m: ctx.alt_min_u_m,
                home_alt_amsl_m: ctx.home_alt_amsl_m,
                now_ms: ctx.now_ms,
            });
            if leftover.newly_breached {
                new_breaches |= TYPE_ALT_MIN;
            }
            leftover
        } else {
            idle_alt_min
        };

        let mut floor_auto_enabled = false;
        let mut floor_alt_unavailable = false;
        let mut need_gcs_floor_notice = false;
        if disabled_fences & TYPE_ALT_MIN == 0 {
            let floor = self.auto_enable_fence_floor(ctx);
            floor_auto_enabled = floor.0;
            floor_alt_unavailable = floor.1;
            need_gcs_floor_notice = floor.2;
        }

        let circle = if disabled_fences & TYPE_CIRCLE == 0 {
            let leftover = self.check_fence_circle(CheckCircleContext {
                ne_home_m: ctx.ne_home_m,
                now_ms: ctx.now_ms,
            });
            if leftover.newly_breached {
                new_breaches |= TYPE_CIRCLE;
            }
            leftover
        } else {
            idle_circle
        };

        let mut manual_recovery_active = false;
        let mut manual_recovery_expired = false;
        if self.manual_recovery_start_ms != 0 {
            if ctx
                .now_ms
                .wrapping_sub(self.manual_recovery_start_ms)
                < MANUAL_RECOVERY_TIME_MIN_MS
            {
                manual_recovery_active = true;
                new_breaches = 0;
            } else {
                self.manual_recovery_start_ms = 0;
                manual_recovery_expired = true;
            }
        }

        CheckLeftover {
            new_breaches,
            disabled_fences,
            fences_to_disable,
            cleared_unconfigured_breach,
            cleared_disabled_breach,
            auto_disabled_message,
            skipped: false,
            need_location: true,
            last_check_loc_valid: self.last_fence_check_loc_valid,
            disable_changed_mask: disable_leftover.changed_mask,
            alt_max,
            alt_min,
            circle,
            polygon_checked: false,
            floor_auto_enabled,
            floor_alt_unavailable,
            need_gcs_floor_notice,
            manual_recovery_active,
            manual_recovery_expired,
        }
    }

    /// `AC_Fence::auto_enable_fence_floor` leftover. Arm / takeoff
    /// auto-enable stays a later slice; `check()` still calls this.
    fn auto_enable_fence_floor(&mut self, ctx: CheckContext) -> (bool, bool, bool) {
        if self.configured_fences & TYPE_ALT_MIN == 0
            || self.get_enabled_fences() & TYPE_ALT_MIN != 0
            || matches!(self.min_alt_state, MinAltState::ManuallyDisabled)
            || (!self.enable_param
                && matches!(
                    self.auto_enabled,
                    AutoEnable::AlwaysDisabled | AutoEnable::EnableOnAutoTakeoff
                ))
        {
            return (false, false, false);
        }

        let Some(curr_alt_u_m) = ctx.alt_min_u_m else {
            return (false, true, false);
        };

        if self.alt_min_type == AltFrame::Absolute {
            self.safe_relhome_alt_min_m = self.alt_min_m - ctx.home_alt_amsl_m - self.margin_m;
        } else {
            self.safe_relhome_alt_min_m = self.alt_min_m - self.margin_m;
        }

        if !self.floor_enabled() && curr_alt_u_m >= self.get_safe_alt_min_m() {
            self.enable(true, TYPE_ALT_MIN, false);
            return (true, false, true);
        }

        (false, false, false)
    }

    fn idle_circle_leftover(&self) -> CheckCircleLeftover {
        CheckCircleLeftover {
            newly_breached: false,
            enabled: false,
            need_ne_home: false,
            home_distance_m: self.home_distance_m,
            breach_distance_m: self.circle_breach_distance_m,
            backup_radius_m: self.circle_radius_backup_m,
            breach_direction_ne_m: self.circle_breach_direction,
            recorded_breach: false,
            need_gcs_fence_status: false,
            margin_breached: (self.breached_fence_margins & TYPE_CIRCLE) != 0,
            cleared_breach: false,
        }
    }

    fn idle_alt_max_leftover(&self) -> CheckAltMaxLeftover {
        CheckAltMaxLeftover {
            newly_breached: false,
            enabled: false,
            alt_unavailable: false,
            need_alt_in_frame: false,
            need_home_alt: false,
            breach_distance_m: self.alt_max_breach_distance_m,
            safe_relhome_alt_max_m: self.safe_relhome_alt_max_m,
            backup_alt_m: self.alt_max_backup_m,
            recorded_breach: false,
            need_gcs_fence_status: false,
            margin_breached: (self.breached_fence_margins & TYPE_ALT_MAX) != 0,
            cleared_breach: false,
        }
    }

    fn idle_alt_min_leftover(&self) -> CheckAltMinLeftover {
        CheckAltMinLeftover {
            newly_breached: false,
            enabled: false,
            alt_unavailable: false,
            need_alt_in_frame: false,
            need_home_alt: false,
            breach_distance_m: self.alt_min_breach_distance_m,
            safe_relhome_alt_min_m: self.safe_relhome_alt_min_m,
            backup_alt_m: self.alt_min_backup_m,
            recorded_breach: false,
            need_gcs_fence_status: false,
            margin_breached: (self.breached_fence_margins & TYPE_ALT_MIN) != 0,
            cleared_breach: false,
        }
    }

    fn record_breach(&mut self, fence_type: u8, now_ms: u32) -> bool {
        let mut need_gcs = false;
        if self.breached_fences == 0 {
            self.breach_time_ms = now_ms;
            if now_ms.wrapping_sub(self.last_breach_notify_sent_ms) > 1_000 {
                self.last_breach_notify_sent_ms = now_ms;
                need_gcs = true;
            }
        }
        if self.breach_count < 65_500 {
            self.breach_count += 1;
        }
        self.breached_fences |= fence_type;
        need_gcs
    }

    fn record_margin_breach(&mut self, fence_type: u8, now_ms: u32) {
        if self.breached_fence_margins & fence_type == 0 {
            self.margin_breach_time_ms = now_ms;
        }
        self.breached_fence_margins |= fence_type;
    }

    fn clear_breach(&mut self, fence_type: u8) {
        self.breached_fences &= !fence_type;
    }

    fn clear_margin_breach(&mut self, fence_type: u8) {
        self.breached_fence_margins &= !fence_type;
    }
}

const fn log_bit(changed: u8, bit: u8, value: bool) -> Option<bool> {
    if changed & bit != 0 {
        Some(value)
    } else {
        None
    }
}
