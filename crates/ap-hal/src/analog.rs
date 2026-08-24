//! Analog input, ported from `AP_HAL/AnalogIn.h`.
//!
//! Feeds battery monitoring and the analog airspeed sensor — both on the
//! fixed-wing path, which is why this is in the FW-001 slice while GPIO and the
//! device buses are not.
//!
//! Upstream splits this in two: `AnalogSource` is one configured channel, and
//! `AnalogIn` is the manager that hands them out. That split is kept, because
//! it is what lets a subsystem hold its own channel without reaching back
//! through a manager singleton on every read.
//!
//! # Averaged versus latest
//!
//! Both variants are ported deliberately. `read_average`/`voltage_average`
//! return an accumulated mean since the last read, which is what battery
//! monitoring wants; `read_latest`/`voltage_latest` return the most recent
//! sample, which is what a fast loop wants. Collapsing them would silently
//! change filter behaviour at every call site.

use crate::Result;

/// One configured analog channel. Upstream `AP_HAL::AnalogSource`.
pub trait AnalogSource {
    /// Mean of samples accumulated since the last call, in ADC counts.
    /// Upstream `read_average()`.
    fn read_average(&mut self) -> f32;

    /// Most recent sample, in ADC counts. Upstream `read_latest()`.
    fn read_latest(&mut self) -> f32;

    /// Mean since the last call, scaled to volts. Upstream `voltage_average()`.
    fn voltage_average(&mut self) -> f32;

    /// Most recent sample, scaled to volts. Upstream `voltage_latest()`.
    fn voltage_latest(&mut self) -> f32;

    /// Mean scaled to volts and corrected against the board's 5V rail.
    ///
    /// Upstream `voltage_average_ratiometric()`, used where a sensor's output
    /// scales with its supply so rail sag would otherwise read as signal.
    fn voltage_average_ratiometric(&mut self) -> f32;

    /// Point this source at a different pin. Upstream `set_pin()`.
    fn set_pin(&mut self, pin: u8) -> Result<()>;
}

/// The analog input manager. Upstream `AP_HAL::AnalogIn`.
pub trait AnalogIn {
    /// Board supply voltage. Upstream `board_voltage()`.
    fn board_voltage(&self) -> f32;

    /// Servo rail voltage, or `None` if the board cannot measure it.
    ///
    /// Upstream `servorail_voltage()` returns `0` for "not measurable", which
    /// collides with a genuine reading of zero volts — a real value, and the
    /// one that matters, since it means the rail has collapsed.
    fn servorail_voltage(&self) -> Option<f32> {
        None
    }

    /// Whether `pin` can be used as an analog input. Upstream
    /// `valid_analog_pin()`.
    fn valid_pin(&self, pin: u16) -> bool;
}

/// A fixed-reading [`AnalogSource`] for tests and SITL bring-up.
#[derive(Debug, Clone, Copy)]
pub struct MockAnalogSource {
    counts: f32,
    scale_volts_per_count: f32,
    board_voltage: f32,
    pin: u8,
    reads: u32,
}

impl Default for MockAnalogSource {
    fn default() -> Self {
        Self {
            counts: 0.0,
            // 12-bit ADC over a 3.3V reference, a common board configuration
            scale_volts_per_count: 3.3 / 4095.0,
            board_voltage: 5.0,
            pin: 0,
            reads: 0,
        }
    }
}

impl MockAnalogSource {
    /// A source reading zero.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the value subsequent reads will return, in ADC counts.
    pub fn set_counts(&mut self, counts: f32) {
        self.counts = counts;
    }

    /// Set the board rail voltage used by the ratiometric conversion.
    pub fn set_board_voltage(&mut self, v: f32) {
        self.board_voltage = v;
    }

    /// The pin this source is bound to.
    pub fn pin(&self) -> u8 {
        self.pin
    }

    /// How many reads have been taken, so a test can show that averaged and
    /// latest reads are distinct calls rather than aliases.
    pub fn read_count(&self) -> u32 {
        self.reads
    }
}

impl AnalogSource for MockAnalogSource {
    fn read_average(&mut self) -> f32 {
        self.reads += 1;
        self.counts
    }

    fn read_latest(&mut self) -> f32 {
        self.reads += 1;
        self.counts
    }

    fn voltage_average(&mut self) -> f32 {
        self.reads += 1;
        self.counts * self.scale_volts_per_count
    }

    fn voltage_latest(&mut self) -> f32 {
        self.reads += 1;
        self.counts * self.scale_volts_per_count
    }

    fn voltage_average_ratiometric(&mut self) -> f32 {
        self.reads += 1;
        // scale the reading by how far the rail has drifted from nominal 5V
        let v = self.counts * self.scale_volts_per_count;
        if self.board_voltage > 0.0 {
            v * (5.0 / self.board_voltage)
        } else {
            v
        }
    }

    fn set_pin(&mut self, pin: u8) -> Result<()> {
        self.pin = pin;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;

    fn near(a: f32, b: f32) {
        assert!((a - b).abs() < 1.0e-4, "expected {b}, got {a}");
    }

    #[test]
    fn converts_counts_to_volts() {
        let mut s = MockAnalogSource::new();
        s.set_counts(4095.0);
        near(s.voltage_latest(), 3.3);
        s.set_counts(2047.5);
        near(s.voltage_latest(), 1.65);
    }

    /// Ratiometric conversion corrects for rail sag, which is the whole reason
    /// upstream keeps it as a separate call.
    #[test]
    fn ratiometric_corrects_for_rail_sag() {
        let mut s = MockAnalogSource::new();
        s.set_counts(2047.5);

        // nominal rail: ratiometric matches the plain reading
        s.set_board_voltage(5.0);
        near(s.voltage_average_ratiometric(), 1.65);

        // rail sagged 10%: the same counts mean a proportionally higher signal
        s.set_board_voltage(4.5);
        near(s.voltage_average_ratiometric(), 1.65 * (5.0 / 4.5));

        // a collapsed rail must not divide by zero
        s.set_board_voltage(0.0);
        near(s.voltage_average_ratiometric(), 1.65);
    }

    #[test]
    fn averaged_and_latest_are_distinct_calls() {
        let mut s = MockAnalogSource::new();
        s.set_counts(100.0);
        let _ = s.read_average();
        let _ = s.read_latest();
        assert_eq!(s.read_count(), 2, "each is its own read, not an alias");
    }

    #[test]
    fn pin_can_be_rebound() {
        let mut s = MockAnalogSource::new();
        assert_eq!(s.pin(), 0);
        s.set_pin(13).unwrap();
        assert_eq!(s.pin(), 13);
    }
}
