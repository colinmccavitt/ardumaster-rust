//! Analog airspeed backend, upstream `AP_Airspeed_Analog`. FW-010.
//!
//! Reads `ARSPD_PIN` through [`AnalogSource`] and converts ratiometric voltage
//! to differential pressure: `V * VOLTS_TO_PASCAL / PSI_RANGE`.

use ap_hal::analog::AnalogSource;

use crate::psi_range::clamp_psi_range;

/// 3DR analog airspeed sensor scale, upstream `VOLTS_TO_PASCAL`.
pub const VOLTS_TO_PASCAL: f32 = 819.0;

/// Upstream `PSI_RANGE_DEFAULT` / `ARSPD_PSI_RANGE` (owned by `psi_range`).
pub use crate::psi_range::ARSPD_PSI_RANGE_DEFAULT;

/// Upstream `ARSPD_PIN` param-table default.
pub const ARSPD_PIN_DEFAULT: i8 = 0;

/// `ARSPD_PIN = -1` disables the analog source.
pub const ARSPD_PIN_DISABLED: i8 = -1;

/// Upstream `AP_Airspeed::TYPE_ANALOG`.
pub const ARSPD_TYPE_ANALOG: u8 = 2;

/// Per-instance analog pin / range, upstream `ARSPD_PIN` / `ARSPD_PSI_RANGE`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AnalogAirspeedConfig {
    /// Analog input pin, upstream `ARSPD_PIN` (`-1` disables).
    pub pin: i8,
    /// Sensor PSI range, upstream `ARSPD_PSI_RANGE`.
    pub psi_range: f32,
}

impl Default for AnalogAirspeedConfig {
    fn default() -> Self {
        Self {
            pin: ARSPD_PIN_DEFAULT,
            psi_range: ARSPD_PSI_RANGE_DEFAULT,
        }
    }
}

/// Differential pressure (Pa) from ratiometric volts, upstream
/// `AP_Airspeed_Analog::get_differential_pressure`.
#[must_use]
pub fn differential_pressure_pa(voltage: f32, psi_range: f32) -> f32 {
    voltage * VOLTS_TO_PASCAL / clamp_psi_range(psi_range)
}

/// Analog pitot backend, upstream `AP_Airspeed_Analog`.
#[derive(Debug, Clone)]
pub struct AnalogAirspeedBackend<S: AnalogSource> {
    source: S,
    config: AnalogAirspeedConfig,
    pressure_pa: f32,
    have_pressure: bool,
}

impl<S: AnalogSource> AnalogAirspeedBackend<S> {
    /// Bind a HAL analog source to `ARSPD_PIN` / `ARSPD_PSI_RANGE`.
    #[must_use]
    pub fn new(source: S, config: AnalogAirspeedConfig) -> Self {
        Self {
            source,
            config,
            pressure_pa: 0.0,
            have_pressure: false,
        }
    }

    /// Analog pin and PSI-range config.
    #[must_use]
    pub const fn config(&self) -> &AnalogAirspeedConfig {
        &self.config
    }

    /// Last differential pressure in Pascal.
    #[must_use]
    pub const fn pressure_pa(&self) -> f32 {
        self.pressure_pa
    }

    /// Whether `get_differential_pressure` succeeded on the last read.
    #[must_use]
    pub const fn have_pressure(&self) -> bool {
        self.have_pressure
    }

    /// Replace pin / PSI-range without dropping the analog source.
    pub fn set_config(&mut self, config: AnalogAirspeedConfig) {
        self.config = config;
    }

    /// Bind `ARSPD_PIN`, upstream constructor `hal.analogin->channel(get_pin())`.
    #[must_use]
    pub fn init(&mut self) -> bool {
        if self.config.pin < 0 {
            return false;
        }
        self.source.set_pin(self.config.pin as u8).is_ok()
    }

    /// Temperature is not available on the analog backend.
    #[must_use]
    pub fn get_temperature(&self) -> Option<f32> {
        None
    }

    /// Read differential pressure, upstream `get_differential_pressure()`.
    pub fn get_differential_pressure(&mut self) -> Option<f32> {
        if self.config.pin < 0 {
            self.have_pressure = false;
            return None;
        }
        if self.source.set_pin(self.config.pin as u8).is_err() {
            self.have_pressure = false;
            return None;
        }
        let voltage = self.source.voltage_average_ratiometric();
        let pressure = differential_pressure_pa(voltage, self.config.psi_range);
        self.pressure_pa = pressure;
        self.have_pressure = true;
        Some(pressure)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_hal::analog::MockAnalogSource;

    fn source_at_volts(volts: f32) -> MockAnalogSource {
        let mut source = MockAnalogSource::new();
        source.set_counts(volts * 4095.0 / 3.3);
        source
    }

    #[test]
    fn default_pin_and_psi_range_match_upstream() {
        let cfg = AnalogAirspeedConfig::default();
        assert_eq!(cfg.pin, 0);
        assert!((cfg.psi_range - 1.0).abs() < 1e-6);
        assert_eq!(ARSPD_TYPE_ANALOG, 2);
        assert!((VOLTS_TO_PASCAL - 819.0).abs() < 1e-6);
    }

    #[test]
    fn one_volt_is_819_pa_at_default_psi_range() {
        assert!((differential_pressure_pa(1.0, 1.0) - 819.0).abs() < 1e-4);
        assert!((differential_pressure_pa(1.0, 2.0) - 409.5).abs() < 1e-4);
        assert!((differential_pressure_pa(1.0, 0.0) - 819.0).abs() < 1e-4);
    }

    #[test]
    fn init_fails_when_pin_disabled() {
        let mut backend = AnalogAirspeedBackend::new(
            MockAnalogSource::new(),
            AnalogAirspeedConfig {
                pin: ARSPD_PIN_DISABLED,
                ..AnalogAirspeedConfig::default()
            },
        );
        assert!(!backend.init());
        assert!(backend.get_differential_pressure().is_none());
        assert!(!backend.have_pressure());
        assert!(backend.get_temperature().is_none());
    }

    #[test]
    fn reads_ratiometric_voltage_as_pressure() {
        let mut backend = AnalogAirspeedBackend::new(
            source_at_volts(1.0),
            AnalogAirspeedConfig::default(),
        );
        assert!(backend.init());
        let pressure = backend.get_differential_pressure().unwrap();
        assert!((pressure - 819.0).abs() < 1e-2);
        assert!(backend.have_pressure());
        assert_eq!(backend.config().pin, ARSPD_PIN_DEFAULT);
    }

    #[test]
    fn pin_rebind_follows_arspd_pin() {
        let mut source = source_at_volts(0.5);
        source.set_pin(15).unwrap();
        let mut backend = AnalogAirspeedBackend::new(
            source,
            AnalogAirspeedConfig {
                pin: 13,
                psi_range: 1.0,
            },
        );
        assert!(backend.init());
        let pressure = backend.get_differential_pressure().unwrap();
        assert!((pressure - 409.5).abs() < 1e-2);
        assert_eq!(backend.config().pin, 13);
    }
}
