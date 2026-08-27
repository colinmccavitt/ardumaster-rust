//! AP_Baro frontend: ground pressure calibration and altitude, upstream `AP_Baro.cpp`.
//! FW-013.

use ap_math::scalar::is_positive;

use crate::sitl::SITL_BARO_MAX_INSTANCES;
use crate::{altitude_difference, sealevel_pressure};

/// Per-instance sensor state after frontend update, upstream `AP_Baro::sensor`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct BaroInstanceState {
    /// Calibrated sea-level reference pressure, upstream `ground_pressure`.
    pub ground_pressure_pa: f32,
    pub calibrated: bool,
    pub altitude_m: f32,
    pub alt_ok: bool,
}

/// Validate stored ground pressure; fall back to current reading, upstream `update()`.
#[must_use]
pub fn ensure_ground_pressure(ground_pressure_pa: f32, current_pressure_pa: f32) -> f32 {
    if is_positive(ground_pressure_pa) && ground_pressure_pa.is_finite() {
        ground_pressure_pa
    } else {
        current_pressure_pa
    }
}

/// Sea-level ground pressure from averaged sample at boot, upstream `calibrate()`.
#[must_use]
pub fn calibrate_ground_pressure(avg_pressure_pa: f32, field_elevation_m: f32) -> f32 {
    sealevel_pressure(avg_pressure_pa, field_elevation_m)
}

/// Refresh ground pressure from current reading, upstream `update_calibration()`.
#[must_use]
pub fn update_calibration_ground_pressure(
    corrected_pressure_pa: f32,
    field_elevation_m: f32,
) -> f32 {
    sealevel_pressure(corrected_pressure_pa, field_elevation_m)
}

/// Barometric altitude relative to calibration, upstream altitude calc in `update()`.
#[must_use]
pub fn baro_altitude_m(
    ground_pressure_pa: f32,
    corrected_pressure_pa: f32,
    field_elevation_m: f32,
    alt_offset_m: f32,
) -> Option<f32> {
    let ground = ensure_ground_pressure(ground_pressure_pa, corrected_pressure_pa);
    let altitude =
        altitude_difference(ground, corrected_pressure_pa)? - field_elevation_m + alt_offset_m;
    altitude.is_finite().then_some(altitude)
}

/// Frontend holding per-instance calibration and computed altitude.
#[derive(Debug, Clone)]
pub struct BaroFrontend {
    pub instances: [BaroInstanceState; SITL_BARO_MAX_INSTANCES],
    pub field_elevation_m: f32,
    pub alt_offset_m: f32,
}

impl Default for BaroFrontend {
    fn default() -> Self {
        Self::new()
    }
}

impl BaroFrontend {
    #[must_use]
    pub fn new() -> Self {
        Self {
            instances: [BaroInstanceState::default(); SITL_BARO_MAX_INSTANCES],
            field_elevation_m: 0.0,
            alt_offset_m: 0.0,
        }
    }

    /// Boot calibration for one instance, upstream `calibrate()`.
    pub fn calibrate_instance(&mut self, instance: u8, avg_pressure_pa: f32) {
        let Some(inst) = self.instances.get_mut(instance as usize) else {
            return;
        };
        if is_positive(avg_pressure_pa) && avg_pressure_pa.is_finite() {
            inst.ground_pressure_pa =
                calibrate_ground_pressure(avg_pressure_pa, self.field_elevation_m);
            inst.calibrated = true;
            inst.alt_ok = true;
        } else {
            inst.calibrated = false;
            inst.alt_ok = false;
        }
    }

    /// Boot calibration from averaged pressure samples, upstream `calibrate()`.
    pub fn calibrate(&mut self, avg_pressures: &[f32]) {
        for (i, &pressure) in avg_pressures.iter().enumerate().take(SITL_BARO_MAX_INSTANCES) {
            self.calibrate_instance(i as u8, pressure);
        }
    }

    /// Pre-arm refresh, upstream `update_calibration()`.
    pub fn update_calibration(&mut self, corrected_pressures: &[f32]) {
        for (i, &pressure) in corrected_pressures
            .iter()
            .enumerate()
            .take(SITL_BARO_MAX_INSTANCES)
        {
            if is_positive(pressure) && pressure.is_finite() {
                self.instances[i].ground_pressure_pa =
                    update_calibration_ground_pressure(pressure, self.field_elevation_m);
                self.instances[i].calibrated = true;
            }
        }
    }

    /// Compute altitude for one instance from corrected pressure.
    pub fn update_instance_altitude(&mut self, instance: u8, corrected_pressure_pa: f32) {
        let Some(inst) = self.instances.get_mut(instance as usize) else {
            return;
        };
        let ground = inst.ground_pressure_pa;
        match baro_altitude_m(
            ground,
            corrected_pressure_pa,
            self.field_elevation_m,
            self.alt_offset_m,
        ) {
            Some(alt) => {
                inst.altitude_m = alt;
                inst.alt_ok = true;
            }
            None => inst.alt_ok = false,
        }
    }

    #[must_use]
    pub fn is_calibrated(&self, instance: u8) -> bool {
        self.instances
            .get(instance as usize)
            .is_some_and(|inst| inst.calibrated)
    }

    #[must_use]
    pub fn get_altitude(&self, instance: u8) -> f32 {
        self.instances
            .get(instance as usize)
            .map(|inst| inst.altitude_m)
            .unwrap_or(0.0)
    }

    #[must_use]
    pub fn get_ground_pressure(&self, instance: u8) -> f32 {
        self.instances
            .get(instance as usize)
            .map(|inst| inst.ground_pressure_pa)
            .unwrap_or(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ensure_ground_pressure_falls_back_to_current_reading() {
        assert_eq!(ensure_ground_pressure(0.0, 99_000.0), 99_000.0);
        assert_eq!(ensure_ground_pressure(f32::NAN, 99_000.0), 99_000.0);
        assert_eq!(ensure_ground_pressure(101_325.0, 99_000.0), 101_325.0);
    }

    #[test]
    fn calibrated_altitude_is_zero_at_reference_pressure() {
        let ground = calibrate_ground_pressure(101_325.0, 0.0);
        let alt = baro_altitude_m(ground, 101_325.0, 0.0, 0.0).expect("finite altitude");
        assert!(alt.abs() < 0.5, "expected ~0 m at calibration point, got {alt}");
    }

    #[test]
    fn calibrated_altitude_tracks_climb_from_reference() {
        let (p0, _) = crate::pressure_temperature_for_alt_amsl(0.0);
        let (p500, _) = crate::pressure_temperature_for_alt_amsl(500.0);
        let mut frontend = BaroFrontend::new();
        frontend.calibrate(&[p0]);
        frontend.update_instance_altitude(0, p500);
        let alt = frontend.get_altitude(0);
        assert!(
            (alt - 500.0).abs() < 2.0,
            "expected ~500 m after climb, got {alt}"
        );
    }

    #[test]
    fn update_calibration_resets_altitude_reference() {
        let (p0, _) = crate::pressure_temperature_for_alt_amsl(0.0);
        let (p500, _) = crate::pressure_temperature_for_alt_amsl(500.0);
        let mut frontend = BaroFrontend::new();
        frontend.calibrate(&[p0]);
        frontend.update_instance_altitude(0, p500);
        assert!((frontend.get_altitude(0) - 500.0).abs() < 2.0);

        frontend.update_calibration(&[p500]);
        frontend.update_instance_altitude(0, p500);
        assert!(
            frontend.get_altitude(0).abs() < 0.5,
            "update_calibration should zero altitude at current pressure"
        );
    }

    #[test]
    fn field_elevation_shifts_reported_altitude() {
        let (p100, _) = crate::pressure_temperature_for_alt_amsl(100.0);
        let mut frontend = BaroFrontend::new();
        frontend.field_elevation_m = 100.0;
        frontend.calibrate(&[p100]);
        frontend.update_instance_altitude(0, p100);
        assert!(
            frontend.get_altitude(0).abs() < 0.5,
            "field elevation cancels AMSL offset at calibration"
        );
    }

    #[test]
    fn calibrate_instance_targets_failover_primary() {
        let (p200, _) = crate::pressure_temperature_for_alt_amsl(200.0);
        let (p300, _) = crate::pressure_temperature_for_alt_amsl(300.0);
        let mut frontend = BaroFrontend::new();
        frontend.calibrate_instance(1, p200);
        assert!(frontend.is_calibrated(1));
        assert!(!frontend.is_calibrated(0));
        frontend.update_instance_altitude(1, p200);
        assert!(frontend.get_altitude(1).abs() < 0.5);
        frontend.update_instance_altitude(1, p300);
        assert!(
            (frontend.get_altitude(1) - 100.0).abs() < 2.0,
            "secondary instance should track climb after its own calibration"
        );
    }
}
