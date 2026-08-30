//! Port of libraries/SITL/SIM_Battery.h + SIM_Battery.cpp (Copter-4.7.0).
//! C++ counterpart `sim_battery.hpp` (CCP-046). SoC table, IR sag, 10 Hz
//! voltage filter, first-order temperature model. `consume_energy` takes
//! explicit `now_us` (ADR-0012, no AP_HAL::micros64).

#![allow(missing_docs)]

/// SITL battery parameter surface, C++ `SitlParams`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SitlParams {
    pub batt_voltage: f32,
    pub batt_capacity_ah: f32,
    /// Negative means "leave resistance unchanged" in `maybe_reset`.
    pub batt_resistance: f32,
    pub vibe_motor: f32,
}

impl Default for SitlParams {
    fn default() -> Self {
        Self {
            batt_voltage: 12.6,
            batt_capacity_ah: 0.0,
            batt_resistance: -1.0,
            vibe_motor: 0.0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct SocRow {
    volt_per_cell: f32,
    soc_pct: f32,
}

const SOC_TABLE: [SocRow; 40] = [
    SocRow { volt_per_cell: 4.173, soc_pct: 100.0 },
    SocRow { volt_per_cell: 4.112, soc_pct: 96.15 },
    SocRow { volt_per_cell: 4.085, soc_pct: 92.31 },
    SocRow { volt_per_cell: 4.071, soc_pct: 88.46 },
    SocRow { volt_per_cell: 4.039, soc_pct: 84.62 },
    SocRow { volt_per_cell: 3.987, soc_pct: 80.77 },
    SocRow { volt_per_cell: 3.943, soc_pct: 76.92 },
    SocRow { volt_per_cell: 3.908, soc_pct: 73.08 },
    SocRow { volt_per_cell: 3.887, soc_pct: 69.23 },
    SocRow { volt_per_cell: 3.854, soc_pct: 65.38 },
    SocRow { volt_per_cell: 3.833, soc_pct: 61.54 },
    SocRow { volt_per_cell: 3.801, soc_pct: 57.69 },
    SocRow { volt_per_cell: 3.783, soc_pct: 53.85 },
    SocRow { volt_per_cell: 3.742, soc_pct: 50.0 },
    SocRow { volt_per_cell: 3.715, soc_pct: 46.15 },
    SocRow { volt_per_cell: 3.679, soc_pct: 42.31 },
    SocRow { volt_per_cell: 3.636, soc_pct: 38.46 },
    SocRow { volt_per_cell: 3.588, soc_pct: 34.62 },
    SocRow { volt_per_cell: 3.543, soc_pct: 30.77 },
    SocRow { volt_per_cell: 3.503, soc_pct: 26.92 },
    SocRow { volt_per_cell: 3.462, soc_pct: 23.08 },
    SocRow { volt_per_cell: 3.379, soc_pct: 19.23 },
    SocRow { volt_per_cell: 3.296, soc_pct: 15.38 },
    SocRow { volt_per_cell: 3.218, soc_pct: 11.54 },
    SocRow { volt_per_cell: 3.165, soc_pct: 7.69 },
    SocRow { volt_per_cell: 3.091, soc_pct: 3.85 },
    SocRow { volt_per_cell: 2.977, soc_pct: 2.0 },
    SocRow { volt_per_cell: 2.8, soc_pct: 1.5 },
    SocRow { volt_per_cell: 2.7, soc_pct: 1.3 },
    SocRow { volt_per_cell: 2.5, soc_pct: 1.2 },
    SocRow { volt_per_cell: 2.3, soc_pct: 1.1 },
    SocRow { volt_per_cell: 2.1, soc_pct: 1.0 },
    SocRow { volt_per_cell: 1.9, soc_pct: 0.9 },
    SocRow { volt_per_cell: 1.6, soc_pct: 0.8 },
    SocRow { volt_per_cell: 1.3, soc_pct: 0.7 },
    SocRow { volt_per_cell: 1.0, soc_pct: 0.6 },
    SocRow { volt_per_cell: 0.6, soc_pct: 0.4 },
    SocRow { volt_per_cell: 0.3, soc_pct: 0.2 },
    SocRow { volt_per_cell: 0.01, soc_pct: 0.01 },
    SocRow { volt_per_cell: 0.001, soc_pct: 0.001 },
];

/// First-order low-pass matching Filter/LowPassFilter `apply(sample, dt)`.
fn calc_lowpass_alpha_dt(dt: f32, cutoff_hz: f32) -> f32 {
    if dt <= 0.0 || cutoff_hz <= 0.0 {
        return 1.0;
    }
    let rc = 1.0 / (core::f32::consts::TAU * cutoff_hz);
    dt / (dt + rc)
}

/// Simulated battery. Upstream `SITL::Battery`.
#[derive(Debug, Clone)]
pub struct Battery {
    capacity_ah: f32,
    resistance_ohm: f32,
    max_voltage: f32,
    ambient_temperature_degc: f32,
    voltage_set: f32,
    remaining_ah: f32,
    last_us: u64,
    temperature_degc: f32,
    cutoff_hz: f32,
    filtered_voltage: f32,
    filter_seeded: bool,
}

impl Default for Battery {
    fn default() -> Self {
        Self::new(10.0)
    }
}

impl Battery {
    pub fn new(cutoff_hz: f32) -> Self {
        Self {
            capacity_ah: 0.0,
            resistance_ohm: 0.01,
            max_voltage: 12.6,
            ambient_temperature_degc: 25.0,
            voltage_set: 12.6,
            remaining_ah: 0.0,
            last_us: 0,
            temperature_degc: 0.0,
            cutoff_hz,
            filtered_voltage: 12.6,
            filter_seeded: false,
        }
    }

    fn reset_filter(&mut self, v: f32) {
        self.filtered_voltage = v;
        self.filter_seeded = true;
    }

    fn apply_filter(&mut self, sample: f32, dt: f32) -> f32 {
        if !self.filter_seeded {
            self.reset_filter(sample);
            return sample;
        }
        let a = calc_lowpass_alpha_dt(dt, self.cutoff_hz);
        self.filtered_voltage += (sample - self.filtered_voltage) * a;
        self.filtered_voltage
    }

    pub fn setup(
        &mut self,
        capacity_ah: f32,
        resistance_ohm: f32,
        max_voltage: f32,
        ambient_temperature_degc: f32,
    ) {
        self.capacity_ah = capacity_ah;
        self.resistance_ohm = resistance_ohm;
        self.max_voltage = max_voltage;
        self.ambient_temperature_degc = ambient_temperature_degc;
        self.voltage_set = max_voltage;
        self.reset_filter(self.voltage_set);
        self.remaining_ah = self.compute_remaining_ah(self.voltage_set);
        self.last_us = 0;
        self.temperature_degc = 0.0;
    }

    pub fn maybe_reset(
        &mut self,
        desired_voltage: f32,
        desired_capacity_ah: f32,
        desired_resistance_ohm: f32,
    ) {
        if desired_resistance_ohm >= 0.0 {
            self.resistance_ohm = desired_resistance_ohm;
        }
        let reset_not_needed = (self.voltage_set - desired_voltage).abs() < 1.0e-6
            && (self.capacity_ah - desired_capacity_ah).abs() < 1.0e-6;
        if reset_not_needed {
            return;
        }
        self.capacity_ah = desired_capacity_ah;
        self.voltage_set = desired_voltage.min(self.max_voltage);
        self.reset_filter(self.voltage_set);
        self.remaining_ah = self.compute_remaining_ah(self.voltage_set);
    }

    pub fn consume_energy(&mut self, attempted_current_amp: f32, now_us: u64) {
        const MICROSEC_TO_SEC: f32 = 1.0e-6;
        const MAX_DT: f32 = 0.1;
        let dt = (now_us.saturating_sub(self.last_us) as f32) * MICROSEC_TO_SEC;
        if dt <= 0.0 {
            return;
        }
        self.last_us = now_us;
        if dt > MAX_DT {
            return;
        }
        const HOURS_PER_SECOND: f32 = 1.0 / 3600.0;
        let dt_hr = dt * HOURS_PER_SECOND;
        let delta_ah = (attempted_current_amp * dt_hr).min(self.remaining_ah);
        if !self.capacity_is_unlimited() {
            self.remaining_ah -= delta_ah;
        }
        let current_amp = if dt_hr > 0.0 { delta_ah / dt_hr } else { 0.0 };
        let voltage_delta = current_amp * self.resistance_ohm;
        let sagged_voltage = self.get_resting_voltage() - voltage_delta;
        self.apply_filter(sagged_voltage, dt);
        self.update_temperature(current_amp, dt);
    }

    pub fn get_voltage(&self) -> f32 {
        self.filtered_voltage
    }
    pub fn get_capacity(&self) -> f32 {
        self.capacity_ah
    }
    pub fn get_temperature_degc(&self) -> f32 {
        self.temperature_degc
    }
    pub fn remaining_ah(&self) -> f32 {
        self.remaining_ah
    }
    pub fn capacity_is_unlimited(&self) -> bool {
        !(self.capacity_ah > 0.0)
    }

    fn get_resting_voltage(&self) -> f32 {
        if self.capacity_is_unlimited() {
            return self.voltage_set;
        }
        let charge_pct = 100.0 * self.remaining_ah / self.capacity_ah;
        let max_cell_voltage = SOC_TABLE[0].volt_per_cell;
        let min_cell_voltage = SOC_TABLE[SOC_TABLE.len() - 1].volt_per_cell;
        for i in 1..SOC_TABLE.len() {
            if charge_pct >= SOC_TABLE[i].soc_pct {
                let dv1 = charge_pct - SOC_TABLE[i].soc_pct;
                let dv2 = SOC_TABLE[i - 1].soc_pct - SOC_TABLE[i].soc_pct;
                let vpc1 = SOC_TABLE[i].volt_per_cell;
                let vpc2 = SOC_TABLE[i - 1].volt_per_cell;
                let cell_volt = vpc1 + (dv1 / dv2) * (vpc2 - vpc1);
                return (cell_volt / max_cell_voltage) * self.max_voltage;
            }
        }
        min_cell_voltage
    }

    fn compute_remaining_ah(&self, voltage: f32) -> f32 {
        if self.capacity_is_unlimited() {
            return f32::MAX;
        }
        let max_cell_voltage = SOC_TABLE[0].volt_per_cell;
        let cell_volt = (voltage / self.max_voltage) * max_cell_voltage;
        for i in 1..SOC_TABLE.len() {
            if cell_volt >= SOC_TABLE[i].volt_per_cell {
                let dv1 = cell_volt - SOC_TABLE[i].volt_per_cell;
                let dv2 = SOC_TABLE[i - 1].volt_per_cell - SOC_TABLE[i].volt_per_cell;
                let soc1 = SOC_TABLE[i].soc_pct;
                let soc2 = SOC_TABLE[i - 1].soc_pct;
                let soc = soc1 + (dv1 / dv2) * (soc2 - soc1);
                return self.capacity_ah * (soc * 0.01);
            }
        }
        0.0
    }

    fn update_temperature(&mut self, current_amp: f32, dt: f32) {
        const INVERSE_THERMAL_CAPACITY: f32 = 1.0 / 500.0;
        const TEMPERATURE_DECAY: f32 = 5.6e-4;
        let temp_increase =
            (current_amp * current_amp) * self.resistance_ohm * INVERSE_THERMAL_CAPACITY * dt;
        let temp_decrease =
            (self.temperature_degc - self.ambient_temperature_degc) * TEMPERATURE_DECAY * dt;
        self.temperature_degc += temp_increase - temp_decrease;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sag_under_load_with_unlimited_capacity() {
        let mut b = Battery::new(10.0);
        b.setup(0.0, 0.01, 12.6, 25.0);
        let v0 = b.get_voltage();
        b.consume_energy(30.0, 1_000);
        b.consume_energy(30.0, 1_000 + 20_000);
        let v1 = b.get_voltage();
        assert!(v1 < v0, "v0={v0} v1={v1}");
        assert!(v1 > 11.5, "v1={v1}");
    }

    #[test]
    fn finite_capacity_loses_amp_hours() {
        let mut b = Battery::new(10.0);
        b.setup(1.0, 0.01, 12.6, 25.0);
        let start = b.remaining_ah();
        let mut t = 1_000u64;
        for _ in 0..200 {
            t += 20_000;
            b.consume_energy(10.0, t);
        }
        assert!(b.remaining_ah() < start);
        assert!(b.get_voltage().is_finite());
    }
}
