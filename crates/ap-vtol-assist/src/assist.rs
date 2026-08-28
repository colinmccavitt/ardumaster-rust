//! Enable / check gate for VTOL assistance. Upstream `VTOL_Assist`.
//!
//! [`VtolAssist`] holds the `Q_ASSIST_*` parameters and the pilot latch
//! (`STATE::{ASSIST_DISABLED, ASSIST_ENABLED, FORCE_ENABLED}`). The
//! interesting rule is the speed gate: `Q_ASSIST_SPEED <= 0` turns off
//! *every* speed / alt / angle check, including `Q_ASSIST_ALT`. Force
//! enable (aux HIGH, or `Q_OPTIONS` bit
//! [`Q_ASSIST_FORCE_ENABLE`]) still assists.

/// Default `Q_ASSIST_SPEED` (m/s). Zero disables speed / alt / angle checks.
pub const ASSIST_SPEED_DEFAULT: f32 = 0.0;

/// Default `Q_ASSIST_ANGLE` (deg).
pub const ASSIST_ANGLE_DEFAULT: i8 = 30;

/// Default `Q_ASSIST_ALT` (m). Zero disables altitude assist.
pub const ASSIST_ALT_DEFAULT: i16 = 0;

/// Default `Q_ASSIST_DELAY` (s).
pub const ASSIST_DELAY_DEFAULT: f32 = 0.5;

/// Default `Q_ASSIST_OPTIONS`.
pub const ASSIST_OPTIONS_DEFAULT: i16 = 0;

/// `Q_OPTIONS` bit 7, upstream `QuadPlane::Option::Q_ASSIST_FORCE_ENABLE`.
pub const Q_ASSIST_FORCE_ENABLE: u32 = 1 << 7;

/// `Q_OPTIONS` bit 12, upstream `DISABLE_SYNTHETIC_AIRSPEED_ASSIST`.
pub const DISABLE_SYNTHETIC_AIRSPEED_ASSIST: u32 = 1 << 12;

/// Whether `Q_OPTIONS` has `Q_ASSIST_FORCE_ENABLE` set.
#[must_use]
pub const fn q_assist_force_enable_set(q_options: u32) -> bool {
    (q_options & Q_ASSIST_FORCE_ENABLE) != 0
}

/// Special options on `Q_ASSIST_OPTIONS`, upstream `VTOL_Assist::OPTION`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i16)]
pub enum AssistOption {
    /// Bit 0: disable force fixed-wing controller recovery.
    FwForceDisabled = 1 << 0,
    /// Bit 1: disable quadplane spin recovery.
    SpinDisabled = 1 << 1,
}

impl AssistOption {
    /// Upstream discriminant (`1U<<0` / `1U<<1`).
    #[must_use]
    pub const fn as_i16(self) -> i16 {
        self as i16
    }
}

/// Pilot / option-bit latch, upstream `VTOL_Assist::STATE`.
///
/// Defaults to [`AssistState::AssistEnabled`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AssistState {
    /// Aux LOW: assist is off.
    AssistDisabled,
    /// Aux MIDDLE (and the constructor default): checks may run.
    AssistEnabled,
    /// Aux HIGH or `Q_ASSIST_FORCE_ENABLE`: assist regardless of speed.
    ForceEnabled,
}

impl AssistState {
    /// Upstream declaration order (`ASSIST_DISABLED`, `ASSIST_ENABLED`,
    /// `FORCE_ENABLED`).
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        match self {
            Self::AssistDisabled => 0,
            Self::AssistEnabled => 1,
            Self::ForceEnabled => 2,
        }
    }

    /// Inverse of [`Self::as_u8`].
    #[must_use]
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::AssistDisabled),
            1 => Some(Self::AssistEnabled),
            2 => Some(Self::ForceEnabled),
            _ => None,
        }
    }

    /// Three-position `Q_ASSIST` aux, upstream
    /// `RC_Channel_Plane::do_aux_function_q_assist_state`.
    #[must_use]
    pub const fn from_aux(pos: AuxSwitchPos) -> Self {
        match pos {
            AuxSwitchPos::Low => Self::AssistDisabled,
            AuxSwitchPos::Middle => Self::AssistEnabled,
            AuxSwitchPos::High => Self::ForceEnabled,
        }
    }
}

/// Three-position `Q_ASSIST` switch, upstream `RC_Channel::AuxSwitchPos`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuxSwitchPos {
    /// Low: `ASSIST_DISABLED`.
    Low,
    /// Middle: `ASSIST_ENABLED`.
    Middle,
    /// High: `FORCE_ENABLED`.
    High,
}

/// VTOL assistance state. Upstream `VTOL_Assist`.
///
/// Holds the `Q_ASSIST_*` parameters and the enable latch. Does not
/// own a `QuadPlane`.
#[derive(Debug, Clone, Copy)]
pub struct VtolAssist {
    /// Speed below which assistance is given, m/s. `Q_ASSIST_SPEED`.
    speed: f32,
    /// Angular error that triggers assistance, deg. `Q_ASSIST_ANGLE`.
    angle: i8,
    /// Altitude below which assistance is given, m. `Q_ASSIST_ALT`.
    alt: i16,
    /// Trigger hysteresis, s. `Q_ASSIST_DELAY`.
    delay: f32,
    /// `Q_ASSIST_OPTIONS` bitmask.
    options: i16,
    /// Pilot / `Q_OPTIONS` latch. Defaults to enabled.
    state: AssistState,
}

impl Default for VtolAssist {
    fn default() -> Self {
        Self::new()
    }
}

impl VtolAssist {
    /// Parameter defaults from `AP_GROUPINFO` (`ASSIST_SPEED` 0,
    /// `ASSIST_ANGLE` 30, `ASSIST_ALT` 0, `ASSIST_DELAY` 0.5,
    /// `ASSIST_OPTIONS` 0) and `STATE::ASSIST_ENABLED`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            speed: ASSIST_SPEED_DEFAULT,
            angle: ASSIST_ANGLE_DEFAULT,
            alt: ASSIST_ALT_DEFAULT,
            delay: ASSIST_DELAY_DEFAULT,
            options: ASSIST_OPTIONS_DEFAULT,
            state: AssistState::AssistEnabled,
        }
    }

    /// `Q_ASSIST_SPEED` (m/s).
    #[must_use]
    pub const fn speed(&self) -> f32 {
        self.speed
    }

    /// `Q_ASSIST_ANGLE` (deg).
    #[must_use]
    pub const fn angle(&self) -> i8 {
        self.angle
    }

    /// `Q_ASSIST_ALT` (m).
    #[must_use]
    pub const fn alt(&self) -> i16 {
        self.alt
    }

    /// `Q_ASSIST_DELAY` (s).
    #[must_use]
    pub const fn delay(&self) -> f32 {
        self.delay
    }

    /// `Q_ASSIST_OPTIONS`.
    #[must_use]
    pub const fn options(&self) -> i16 {
        self.options
    }

    /// Current latch, upstream `state`.
    #[must_use]
    pub const fn state(&self) -> AssistState {
        self.state
    }

    /// Poke `Q_ASSIST_SPEED`.
    pub fn set_speed(&mut self, speed: f32) {
        self.speed = speed;
    }

    /// Poke `Q_ASSIST_ANGLE`.
    pub fn set_angle(&mut self, angle: i8) {
        self.angle = angle;
    }

    /// Poke `Q_ASSIST_ALT`.
    pub fn set_alt(&mut self, alt: i16) {
        self.alt = alt;
    }

    /// Poke `Q_ASSIST_DELAY`.
    pub fn set_delay(&mut self, delay: f32) {
        self.delay = delay;
    }

    /// Poke `Q_ASSIST_OPTIONS`.
    pub fn set_options(&mut self, options: i16) {
        self.options = options;
    }

    /// Upstream `VTOL_Assist::set_state`.
    pub fn set_state(&mut self, state: AssistState) {
        self.state = state;
    }

    /// Apply the `Q_ASSIST` aux, upstream
    /// `do_aux_function_q_assist_state`.
    pub fn set_state_from_aux(&mut self, pos: AuxSwitchPos) {
        self.state = AssistState::from_aux(pos);
    }

    /// Default QAssist state as set with `Q_OPTIONS` at QuadPlane setup.
    ///
    /// Upstream: if `option_is_set(Q_ASSIST_FORCE_ENABLE)` then
    /// `assist.set_state(FORCE_ENABLED)`. Other bits are left for
    /// later slices.
    pub fn apply_q_options(&mut self, q_options: u32) {
        if q_assist_force_enable_set(q_options) {
            self.state = AssistState::ForceEnabled;
        }
    }

    /// Upstream `VTOL_Assist::option_is_set`.
    #[must_use]
    pub const fn option_is_set(&self, option: AssistOption) -> bool {
        (self.options & option.as_i16()) != 0
    }

    /// `Q_ASSIST_SPEED > 0` unlocks speed, alt, and angle checks.
    ///
    /// Upstream `should_assist`: `if (speed <= 0)` every check is
    /// skipped (`speed_assist` cleared, alt / angle hysteresis reset)
    /// and only force-enable still returns true. `-1` is the documented
    /// "disable all Q_ASSIST features except during transitions" value
    /// and takes the same path.
    #[must_use]
    pub fn speed_checks_enabled(&self) -> bool {
        self.speed > 0.0
    }

    /// Altitude assist is configured *and* not gated off by speed.
    ///
    /// `Q_ASSIST_ALT > 0` is not enough: the speed gate runs first.
    #[must_use]
    pub fn alt_check_enabled(&self) -> bool {
        self.speed_checks_enabled() && self.alt > 0
    }

    /// Whether `should_assist` should evaluate speed / alt / angle.
    ///
    /// False when the aux has disabled assist, or when
    /// `Q_ASSIST_SPEED <= 0`. Force-enable still assists, but does not
    /// by itself run those checks.
    #[must_use]
    pub fn should_check(&self) -> bool {
        self.state != AssistState::AssistDisabled && self.speed_checks_enabled()
    }

    /// Whether assist may run at all.
    ///
    /// Matches the enable half of `should_assist`:
    /// - `ASSIST_DISABLED` → off
    /// - `FORCE_ENABLED` → on, even when `Q_ASSIST_SPEED <= 0`
    /// - `ASSIST_ENABLED` → on only when the speed gate is open
    #[must_use]
    pub fn is_enabled(&self) -> bool {
        match self.state {
            AssistState::AssistDisabled => false,
            AssistState::ForceEnabled => true,
            AssistState::AssistEnabled => self.speed_checks_enabled(),
        }
    }
}
