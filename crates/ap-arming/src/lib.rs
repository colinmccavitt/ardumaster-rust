//! Shared AP_Arming check registry. FW-026.
//!
//! Upstream `libraries/AP_Arming`: `ARMING_REQUIRE` decides whether the
//! vehicle may skip the gate entirely; `ARMING_SKIPCHK` (`checks_to_skip`)
//! is the bitmask of named checks the operator has switched off. The
//! registry walks those named checks and [`Arming::pre_arm_checks`] fails
//! on the first enabled one that is not ok.
//!
//! This slice is the gate, not the sensor bodies. AHRS / compass /
//! airspeed already have their own plane hookups; they plug into the
//! registry later.

#![no_std]

/// Default `ARMING_REQUIRE` on Plane, upstream `Required::YES_MIN_PWM`.
pub const ARMING_REQUIRE_DEFAULT: Required = Required::YesMinPwm;

/// Default `ARMING_SKIPCHK`: skip nothing, so every named check runs.
pub const ARMING_SKIPCHK_DEFAULT: u32 = 0;

/// Upstream `Check::CHECK_LAST` — one past the last named bit.
pub const CHECK_LAST: u32 = 1 << 21;

/// Bits that are real named checks, upstream
/// `(Check::CHECK_LAST - 1) & (~1)`. Bit 0 was the former ALL value and
/// is not a check.
pub const CHECK_MASK: u32 = (CHECK_LAST - 1) & !1;

/// `ARMING_REQUIRE` values, upstream `AP_Arming::Required`.
///
/// Plane stores 0 / 1 / 2. 3 and 4 are Rover auto-arm variants kept
/// because the shared library owns the enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Required {
    /// 0 — no arming requirement; the gate is skipped.
    No = 0,
    /// 1 — arming required; disarmed throttle is `THR_MIN` PWM.
    YesMinPwm = 1,
    /// 2 — arming required; disarmed throttle is 0 PWM.
    YesZeroPwm = 2,
    /// 3 — Rover: auto-arm once after checks pass (`THR_MIN` PWM).
    YesAutoArmMinPwm = 3,
    /// 4 — Rover: auto-arm once after checks pass (0 PWM).
    YesAutoArmZeroPwm = 4,
}

impl Required {
    /// Decode a stored `ARMING_REQUIRE` value.
    #[must_use]
    pub const fn from_u8(raw: u8) -> Option<Self> {
        match raw {
            0 => Some(Self::No),
            1 => Some(Self::YesMinPwm),
            2 => Some(Self::YesZeroPwm),
            3 => Some(Self::YesAutoArmMinPwm),
            4 => Some(Self::YesAutoArmZeroPwm),
            _ => None,
        }
    }

    /// The stored parameter value.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// Named `ARMING_CHECK` / `ARMING_SKIPCHK` bits, upstream `AP_Arming::Check`.
///
/// A bit set in `checks_to_skip` *disables* that check. The former ALL
/// value occupied bit 0 and is not a member.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum Check {
    /// Bit 1 — barometer.
    Baro = 1 << 1,
    /// Bit 2 — compass.
    Compass = 1 << 2,
    /// Bit 3 — GPS.
    Gps = 1 << 3,
    /// Bit 4 — inertial sensors.
    Ins = 1 << 4,
    /// Bit 5 — parameters.
    Parameters = 1 << 5,
    /// Bit 6 — RC.
    Rc = 1 << 6,
    /// Bit 7 — board voltage.
    Voltage = 1 << 7,
    /// Bit 8 — battery.
    Battery = 1 << 8,
    /// Bit 9 — airspeed.
    Airspeed = 1 << 9,
    /// Bit 10 — logging.
    Logging = 1 << 10,
    /// Bit 11 — arming switch.
    Switch = 1 << 11,
    /// Bit 12 — GPS configuration.
    GpsConfig = 1 << 12,
    /// Bit 13 — system.
    System = 1 << 13,
    /// Bit 14 — mission.
    Mission = 1 << 14,
    /// Bit 15 — rangefinder.
    Rangefinder = 1 << 15,
    /// Bit 16 — camera.
    Camera = 1 << 16,
    /// Bit 17 — auxiliary authorisation.
    AuxAuth = 1 << 17,
    /// Bit 18 — visual odometry.
    Vision = 1 << 18,
    /// Bit 19 — FFT.
    Fft = 1 << 19,
    /// Bit 20 — OSD.
    Osd = 1 << 20,
}

impl Check {
    /// The skip-mask bit for this named check.
    #[must_use]
    pub const fn as_u32(self) -> u32 {
        self as u32
    }
}

/// One named check in the registry: which bit, what to call it, whether
/// it currently passes.
///
/// The name is what [`Arming::pre_arm_checks`] reports when this check
/// refuses. Sensor hookups own the real health test; they fill `ok`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NamedCheck {
    /// Which `ARMING_SKIPCHK` bit gates this entry.
    pub check: Check,
    /// Short name reported on refusal, e.g. `"BARO"`.
    pub name: &'static str,
    /// Whether the check currently passes.
    pub ok: bool,
}

/// What walking the registry decided.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PreArmOutcome {
    /// The gate did not refuse.
    Allowed,
    /// An enabled named check failed.
    Refused {
        /// Which check refused.
        check: Check,
        /// The name from the registry entry.
        name: &'static str,
    },
}

impl PreArmOutcome {
    /// Whether the gate allowed arming.
    #[must_use]
    pub const fn allowed(self) -> bool {
        matches!(self, Self::Allowed)
    }
}

/// Shared `AP_Arming` state for the check-registry stub.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Arming {
    /// `ARMING_REQUIRE`.
    pub require: Required,
    /// `ARMING_SKIPCHK` — bits set here are *not* run.
    pub checks_to_skip: u32,
    /// Soft-armed flag, upstream `AP_Arming::armed`.
    pub armed: bool,
}

impl Default for Arming {
    fn default() -> Self {
        Self::new()
    }
}

impl Arming {
    /// Plane defaults: require `YES_MIN_PWM`, skip nothing, disarmed.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            require: ARMING_REQUIRE_DEFAULT,
            checks_to_skip: ARMING_SKIPCHK_DEFAULT,
            armed: false,
        }
    }

    /// Upstream `AP_Arming::arming_required`.
    ///
    /// OpenDroneID can force a YES value later; this slice returns the
    /// stored parameter.
    #[must_use]
    pub const fn arming_required(self) -> Required {
        self.require
    }

    /// Upstream `AP_Arming::check_enabled`: a bit set in
    /// `checks_to_skip` disables that named check.
    #[must_use]
    pub const fn check_enabled(self, check: Check) -> bool {
        (self.checks_to_skip & check.as_u32()) == 0
    }

    /// Upstream `AP_Arming::get_enabled_checks`.
    #[must_use]
    pub const fn get_enabled_checks(self) -> u32 {
        (!self.checks_to_skip) & CHECK_MASK
    }

    /// Upstream `AP_Arming::should_skip_all_checks`.
    #[must_use]
    pub const fn should_skip_all_checks(self) -> bool {
        self.get_enabled_checks() == 0
    }

    /// Upstream `AP_Arming::pre_arm_checks` gate (non-Copter).
    ///
    /// Already-armed or `ARMING_REQUIRE=NO` skips the registry. Skipping
    /// every named check also allows — Plane still runs mandatory checks
    /// in that case, which is a later slice. Otherwise the first enabled
    /// named check that is not ok refuses.
    #[must_use]
    pub fn pre_arm_checks(self, checks: &[NamedCheck]) -> PreArmOutcome {
        if self.armed || self.arming_required() == Required::No {
            return PreArmOutcome::Allowed;
        }
        if self.should_skip_all_checks() {
            return PreArmOutcome::Allowed;
        }
        for named in checks {
            if !self.check_enabled(named.check) {
                continue;
            }
            if !named.ok {
                return PreArmOutcome::Refused {
                    check: named.check,
                    name: named.name,
                };
            }
        }
        PreArmOutcome::Allowed
    }
}
