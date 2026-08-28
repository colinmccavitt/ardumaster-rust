//! `ARMING_RUDDER` / rudder-stick arm-disarm gate. FW-026.
//!
//! Upstream `AP_Arming::RudderArming` / `ARMING_RUDDER`:
//! * 0 — disabled: rudder stick cannot arm or disarm
//! * 1 — arm only: right rudder can arm; left rudder cannot disarm
//! * 2 — arm-or-disarm: right rudder can arm; left rudder can disarm
//!
//! Default on Plane is 1 (`ARMONLY`). The stick-hold FSM (throttle at
//! zero, yaw-channel extreme) is a later slice; this is the parameter
//! gate that decides whether a rudder-stick arm or disarm is allowed.

/// Default `ARMING_RUDDER` on Plane, upstream `RudderArming::ARMONLY`.
pub const ARMING_RUDDER_DEFAULT: RudderArming = RudderArming::ArmOnly;

/// Upstream `AP_Arming::RudderArming`.
///
/// Plane stores 0 / 1 / 2. The names match the parameter docs
/// (`Disabled` / `ArmingOnly` / `ArmOrDisarm`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RudderArming {
    /// 0 — rudder stick cannot arm or disarm.
    Disabled = 0,
    /// 1 — rudder stick can arm; cannot disarm.
    ArmOnly = 1,
    /// 2 — rudder stick can arm or disarm.
    ArmOrDisarm = 2,
}

impl RudderArming {
    /// Decode a stored `ARMING_RUDDER` value.
    #[must_use]
    pub const fn from_u8(raw: u8) -> Option<Self> {
        match raw {
            0 => Some(Self::Disabled),
            1 => Some(Self::ArmOnly),
            2 => Some(Self::ArmOrDisarm),
            _ => None,
        }
    }

    /// The stored parameter value.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Upstream rudder-method arm: refused only when the param is disabled.
    #[must_use]
    pub const fn allows_rudder_arm(self) -> bool {
        !matches!(self, Self::Disabled)
    }

    /// Upstream rudder-method disarm: allowed only for `ArmOrDisarm`.
    #[must_use]
    pub const fn allows_rudder_disarm(self) -> bool {
        matches!(self, Self::ArmOrDisarm)
    }
}

/// A rudder-stick request: arm (right rudder) or disarm (left rudder).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RudderStickAction {
    /// Right-rudder hold, requesting arm.
    Arm,
    /// Left-rudder hold, requesting disarm.
    Disarm,
}

/// Whether `ARMING_RUDDER` allows this rudder-stick action.
///
/// Non-rudder methods (GCS, aux switch) do not go through this gate.
#[must_use]
pub const fn rudder_stick_allowed(rudder: RudderArming, action: RudderStickAction) -> bool {
    match action {
        RudderStickAction::Arm => rudder.allows_rudder_arm(),
        RudderStickAction::Disarm => rudder.allows_rudder_disarm(),
    }
}
