//! Battery failsafe action stub, `FS_BATT_ENABLE` / `FS_BATT_VOLTAGE` / `FS_BATT_MAH`.
//!
//! Historic ArduPlane vehicle-level battery failsafe (`FS_BATT_*` in
//! `Parameters.cpp`, later moved to `AP_BattMonitor` as `BATT_LOWVOLT` /
//! `BATT_LOWMAH` / `BATT_FS_LOW_ACT`). This stub keeps the original
//! `FS_BATT_ENABLE` action table: Disabled / Land / RTL / Terminate.
//!
//! A trip is a voltage drop below `FS_BATT_VOLTAGE` or remaining capacity
//! below `FS_BATT_MAH` (either threshold `<= 0` disables that check). Landing
//! sequence / QLand / AUTOLAND fallbacks in `Plane::handle_battery_failsafe`
//! are left for a later slice.

/// Upstream historic `FS_BATT_VOLTAGE` default, volts.
pub const FS_BATT_VOLTAGE_DEFAULT: f32 = 10.5;
/// Upstream `FS_BATT_MAH` default (0 disables the capacity check).
pub const FS_BATT_MAH_DEFAULT: f32 = 0.0;

/// Upstream historic `FS_BATT_ENABLE`.
///
/// Default is [`Self::Disabled`]. Values match the old vehicle parameter
/// (`0:Disabled, 1:Land, 2:RTL, 3:Terminate`), not the later Plane
/// `Failsafe_Action` numbering used by `BATT_FS_LOW_ACT`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BatteryFailsafeEnable {
    /// 0 — no battery failsafe action.
    Disabled = 0,
    /// 1 — start a landing (`handle_battery_failsafe` Land).
    Land = 1,
    /// 2 — RTL.
    Rtl = 2,
    /// 3 — terminate (`afs.gcs_terminate` / disarm).
    Terminate = 3,
}

impl BatteryFailsafeEnable {
    /// Decode `FS_BATT_ENABLE`. Unknown values are `None`.
    #[must_use]
    pub const fn from_param(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Disabled),
            1 => Some(Self::Land),
            2 => Some(Self::Rtl),
            3 => Some(Self::Terminate),
            _ => None,
        }
    }

    /// Upstream `FS_BATT_ENABLE` default, disabled.
    #[must_use]
    pub const fn default_param() -> Self {
        Self::Disabled
    }

    /// Whether any battery-failsafe action is armed.
    #[must_use]
    pub const fn is_enabled(self) -> bool {
        !matches!(self, Self::Disabled)
    }
}

/// What the battery action table asks the vehicle to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BatteryFailsafeResult {
    /// Stay put (`FS_BATT_ENABLE` disabled, or thresholds not crossed).
    None,
    /// Request a landing.
    Land,
    /// Request RTL.
    Rtl,
    /// Request terminate.
    Terminate,
}

/// Inputs for the battery failsafe stub.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BatteryFailsafeInputs {
    /// `FS_BATT_ENABLE`.
    pub enable: BatteryFailsafeEnable,
    /// Latest pack voltage. `<= 0` means no reading (unhealthy / unset).
    pub voltage_v: f32,
    /// Remaining capacity from current integration. `None` if no current monitor.
    pub remaining_mah: Option<f32>,
    /// `FS_BATT_VOLTAGE`. `<= 0` disables the voltage check.
    pub fs_batt_voltage: f32,
    /// `FS_BATT_MAH`. `<= 0` disables the remaining-capacity check.
    pub fs_batt_mah: f32,
}

impl Default for BatteryFailsafeInputs {
    fn default() -> Self {
        Self {
            enable: BatteryFailsafeEnable::default_param(),
            voltage_v: 0.0,
            remaining_mah: None,
            fs_batt_voltage: FS_BATT_VOLTAGE_DEFAULT,
            fs_batt_mah: FS_BATT_MAH_DEFAULT,
        }
    }
}

/// Whether voltage or remaining capacity has crossed its configured threshold.
///
/// Matches `AP_BattMonitor_Backend::check_failsafe_types`: a non-positive
/// reading or a non-positive threshold does not trip that check. Comparisons
/// are strict (`voltage < FS_BATT_VOLTAGE`, `remaining < FS_BATT_MAH`).
#[must_use]
pub fn battery_failsafe_thresholds_crossed(inp: &BatteryFailsafeInputs) -> bool {
    let voltage_trip =
        inp.voltage_v > 0.0 && inp.fs_batt_voltage > 0.0 && inp.voltage_v < inp.fs_batt_voltage;
    let mah_trip = match inp.remaining_mah {
        Some(remaining) if inp.fs_batt_mah > 0.0 => remaining < inp.fs_batt_mah,
        _ => false,
    };
    voltage_trip || mah_trip
}

/// Resolve `FS_BATT_ENABLE` once a voltage / remaining-mAh threshold is crossed.
#[must_use]
pub fn battery_failsafe_action(inp: &BatteryFailsafeInputs) -> BatteryFailsafeResult {
    if !inp.enable.is_enabled() || !battery_failsafe_thresholds_crossed(inp) {
        return BatteryFailsafeResult::None;
    }
    match inp.enable {
        BatteryFailsafeEnable::Disabled => BatteryFailsafeResult::None,
        BatteryFailsafeEnable::Land => BatteryFailsafeResult::Land,
        BatteryFailsafeEnable::Rtl => BatteryFailsafeResult::Rtl,
        BatteryFailsafeEnable::Terminate => BatteryFailsafeResult::Terminate,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn param_values_match_upstream_fs_batt_enable() {
        assert_eq!(
            BatteryFailsafeEnable::from_param(0),
            Some(BatteryFailsafeEnable::Disabled)
        );
        assert_eq!(
            BatteryFailsafeEnable::from_param(1),
            Some(BatteryFailsafeEnable::Land)
        );
        assert_eq!(
            BatteryFailsafeEnable::from_param(2),
            Some(BatteryFailsafeEnable::Rtl)
        );
        assert_eq!(
            BatteryFailsafeEnable::from_param(3),
            Some(BatteryFailsafeEnable::Terminate)
        );
        assert_eq!(BatteryFailsafeEnable::from_param(4), None);
        assert_eq!(
            BatteryFailsafeEnable::default_param(),
            BatteryFailsafeEnable::Disabled
        );
        assert!(!BatteryFailsafeEnable::Disabled.is_enabled());
        assert!(BatteryFailsafeEnable::Land.is_enabled());
        assert!((FS_BATT_VOLTAGE_DEFAULT - 10.5).abs() < 1e-6);
        assert!((FS_BATT_MAH_DEFAULT - 0.0).abs() < 1e-6);
    }
}
