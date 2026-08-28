//! Vehicle arm/disarm gate. FW-026.
//!
//! Upstream `AP_Arming::arm` / `AP_Arming::disarm`: the vehicle asks to
//! flip `armed`. When `do_arming_checks` is true the registry must allow;
//! otherwise this is a force-arm (`arm_force`). Already-armed arm and
//! already-disarmed disarm refuse. Rudder-method requests still go
//! through [`crate::rudder_arming`]. Mandatory-only force-arm and
//! vehicle `arm_checks` (RC / logging / estop) are later slices.

use crate::rudder_arming::RudderArming;
use crate::{Arming, Check, NamedCheck, PreArmOutcome};

/// Upstream `AP_Arming::Method` — who asked to arm or disarm.
///
/// Only the methods this gate needs to distinguish are named. The rest
/// of the upstream enum is a later slice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Method {
    /// 0 — rudder-stick request; gated by `ARMING_RUDDER`.
    Rudder = 0,
    /// 1 — GCS / MAVLink.
    Mavlink = 1,
    /// 2 — auxiliary switch.
    AuxSwitch = 2,
    /// 4 — scripting.
    Scripting = 4,
    /// 100 — unknown / unspecified method.
    Unknown = 100,
}

impl Method {
    /// Whether this method is a rudder-stick request.
    #[must_use]
    pub const fn is_rudder(self) -> bool {
        matches!(self, Self::Rudder)
    }
}

/// What [`Arming::arm`] decided.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArmOutcome {
    /// `armed` is now true.
    Armed {
        /// Who asked.
        method: Method,
    },
    /// Already armed; state unchanged. Upstream returns false.
    AlreadyArmed,
    /// Rudder method but `ARMING_RUDDER` disables rudder arm.
    RudderRefused,
    /// Registry refused; still disarmed.
    ChecksFailed {
        /// Which check refused.
        check: Check,
        /// The name from the registry entry.
        name: &'static str,
    },
}

impl ArmOutcome {
    /// Whether the vehicle is now armed because of this call.
    #[must_use]
    pub const fn succeeded(self) -> bool {
        matches!(self, Self::Armed { .. })
    }
}

/// What [`Arming::disarm`] decided.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisarmOutcome {
    /// `armed` is now false.
    Disarmed {
        /// Who asked.
        method: Method,
    },
    /// Already disarmed; state unchanged. Upstream returns false.
    AlreadyDisarmed,
    /// Rudder method but `ARMING_RUDDER` is not `ArmOrDisarm`.
    RudderRefused,
}

impl DisarmOutcome {
    /// Whether the vehicle is now disarmed because of this call.
    #[must_use]
    pub const fn succeeded(self) -> bool {
        matches!(self, Self::Disarmed { .. })
    }
}

impl Arming {
    /// Upstream `AP_Arming::is_armed`.
    #[must_use]
    pub const fn is_armed(self) -> bool {
        self.armed
    }

    /// Upstream `AP_Arming::arm`.
    ///
    /// Already-armed refuses. A rudder-method request is refused when
    /// `ARMING_RUDDER` is disabled. When `do_arming_checks` is true the
    /// registry must allow; a force-arm skips it. On success `armed` is
    /// set. Vehicle-specific `arm_checks` are a later slice.
    pub fn arm(
        &mut self,
        method: Method,
        do_arming_checks: bool,
        checks: &[NamedCheck],
        rudder: RudderArming,
    ) -> ArmOutcome {
        if self.armed {
            return ArmOutcome::AlreadyArmed;
        }
        if method.is_rudder() && !rudder.allows_rudder_arm() {
            return ArmOutcome::RudderRefused;
        }
        if do_arming_checks {
            if let PreArmOutcome::Refused { check, name } = self.pre_arm_checks(checks) {
                return ArmOutcome::ChecksFailed { check, name };
            }
        }
        self.armed = true;
        ArmOutcome::Armed { method }
    }

    /// Upstream `AP_Arming::arm_force` — skip the registry.
    pub fn arm_force(&mut self, method: Method, rudder: RudderArming) -> ArmOutcome {
        self.arm(method, false, &[], rudder)
    }

    /// Upstream `AP_Arming::disarm`.
    ///
    /// Already-disarmed refuses. A rudder-method request is refused
    /// unless `ARMING_RUDDER` is `ArmOrDisarm`. On success `armed` is
    /// cleared. Throttle-down and other vehicle disarm checks are a
    /// later slice.
    pub fn disarm(&mut self, method: Method, rudder: RudderArming) -> DisarmOutcome {
        if !self.armed {
            return DisarmOutcome::AlreadyDisarmed;
        }
        if method.is_rudder() && !rudder.allows_rudder_disarm() {
            return DisarmOutcome::RudderRefused;
        }
        self.armed = false;
        DisarmOutcome::Disarmed { method }
    }
}
