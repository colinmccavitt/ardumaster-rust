//! Auto ground-pressure latch on arm, upstream `AP_Baro::update_calibration()`.
//!
//! When the vehicle arms, refresh each baro instance ground-pressure calibration
//! from its latest corrected sample. Dual-instance setups calibrate every healthy
//! backend, not only the selected primary.

use ap_baro::frontend::BaroFrontend;
use ap_baro::sitl::{SitlBaroCluster, SITL_BARO_MAX_INSTANCES};
use ap_math::scalar::is_positive;

/// Inputs for one arm-calibration tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BaroArmCalibrationInputs {
    /// `hal.util->get_soft_armed()` this tick.
    pub soft_armed: bool,
    /// Previous tick armed state tracked by the vehicle loop.
    pub was_soft_armed: bool,
}

/// Result of one arm-calibration tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BaroArmCalibrationOutput {
    /// Ground pressure was latched this tick.
    pub latched: bool,
    /// Updated armed-state memory for the next tick.
    pub was_soft_armed: bool,
}

/// Collect corrected pressures from instances with valid samples.
#[must_use]
pub fn baro_instance_pressures(cluster: &SitlBaroCluster) -> ([f32; SITL_BARO_MAX_INSTANCES], u8) {
    let mut pressures = [0.0_f32; SITL_BARO_MAX_INSTANCES];
    let mut count = 0_u8;
    for i in 0..cluster.instance_count() {
        if let Some(backend) = cluster.backend(i) {
            let sample = backend.state();
            if sample.have_sample && is_positive(sample.pressure_pa) && sample.pressure_pa.is_finite() {
                if let Some(slot) = pressures.get_mut(count as usize) {
                    *slot = sample.pressure_pa;
                    count += 1;
                }
            }
        }
    }
    (pressures, count)
}

/// Latch ground pressure on arm rising edge, upstream `update_calibration()`.
#[must_use]
pub fn baro_arm_calibration_tick(
    frontend: &mut BaroFrontend,
    cluster: &SitlBaroCluster,
    inp: BaroArmCalibrationInputs,
) -> BaroArmCalibrationOutput {
    let mut latched = false;
    if inp.soft_armed && !inp.was_soft_armed {
        let (pressures, count) = baro_instance_pressures(cluster);
        if count > 0 {
            frontend.update_calibration(&pressures[..count as usize]);
            latched = true;
        }
    }
    BaroArmCalibrationOutput {
        latched,
        was_soft_armed: inp.soft_armed,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_baro::frontend::BaroFrontend;
    use ap_baro::sitl::SitlBaroCluster;
    use ap_math::vector3::Vector3f;

    #[test]
    fn arm_rising_edge_latches_ground_pressure() {
        let mut frontend = BaroFrontend::new();
        let mut cluster = SitlBaroCluster::default();
        cluster.timer_tick_all(0.0, Vector3f::zero(), 1000, 0.0);
        let out = baro_arm_calibration_tick(
            &mut frontend,
            &cluster,
            BaroArmCalibrationInputs {
                soft_armed: true,
                was_soft_armed: false,
            },
        );
        assert!(out.latched);
        assert!(frontend.is_calibrated(0));
        assert!(out.was_soft_armed);
    }

    #[test]
    fn no_latch_when_already_armed_or_disarmed() {
        let mut frontend = BaroFrontend::new();
        let cluster = SitlBaroCluster::default();
        let held = baro_arm_calibration_tick(
            &mut frontend,
            &cluster,
            BaroArmCalibrationInputs {
                soft_armed: true,
                was_soft_armed: true,
            },
        );
        assert!(!held.latched);
        let disarm = baro_arm_calibration_tick(
            &mut frontend,
            &cluster,
            BaroArmCalibrationInputs {
                soft_armed: false,
                was_soft_armed: true,
            },
        );
        assert!(!disarm.latched);
        assert!(!disarm.was_soft_armed);
    }
}
