//! Inertial sensor frontend: instance registration and sensor-rate hooks.
//! Upstream `AP_InertialSensor` register_gyro/register_accel and
//! `_notify_new_*_sensor_rate_sample`. FW-011.

use ap_math::vector3::Vector3f;

use crate::ImuInstance;

/// Maximum IMU instances, upstream `INS_MAX_INSTANCES` for Plane.
pub const INS_MAX_INSTANCES: usize = 3;

/// SITL bus type nibble for synthetic device IDs, upstream `BUS_TYPE_SITL`.
const SITL_BUS_TYPE: u32 = 3;

/// SITL device type, upstream `DEVTYPE_SITL`.
const DEVTYPE_SITL: u32 = 0x53;

/// Build a synthetic bus ID like upstream `AP_HAL::Device::make_bus_id`.
#[must_use]
pub const fn sitl_bus_id(bus_id: u8, devnum: u8) -> u32 {
    (SITL_BUS_TYPE << 24) | ((bus_id as u32) << 16) | ((devnum as u32) << 8) | DEVTYPE_SITL
}

/// Gyro device number on a SITL bus, upstream `start()` registration.
pub const SITL_GYRO_DEVNUM: u8 = 1;
/// Accel device number on a SITL bus.
pub const SITL_ACCEL_DEVNUM: u8 = 2;

/// Optional callbacks for per-sub-sample delivery at sensor rate, upstream
/// `_notify_new_gyro_sensor_rate_sample` / `_notify_new_accel_sensor_rate_sample`.
#[derive(Debug, Clone, Copy, Default)]
pub struct InsSensorRateHooks {
    /// Called once per kinematic sub-sample before averaging (fast sampling).
    pub on_gyro: Option<fn(u8, Vector3f)>,
    pub on_accel: Option<fn(u8, Vector3f)>,
}

impl InsSensorRateHooks {
    /// Invoke the gyro hook if configured.
    pub fn notify_gyro(&self, instance: u8, gyro: Vector3f) {
        if let Some(hook) = self.on_gyro {
            hook(instance, gyro);
        }
    }

    /// Invoke the accel hook if configured.
    pub fn notify_accel(&self, instance: u8, accel: Vector3f) {
        if let Some(hook) = self.on_accel {
            hook(instance, accel);
        }
    }
}

/// Frontend state shared by backends, upstream `AP_InertialSensor`.
#[derive(Debug, Clone)]
pub struct InertialSensorFrontend {
    /// Per-instance accumulation; SITL uses the same slot for paired gyro/accel.
    pub instances: [ImuInstance; INS_MAX_INSTANCES],
    gyro_count: u8,
    accel_count: u8,
    gyro_raw_sample_rates: [u16; INS_MAX_INSTANCES],
    accel_raw_sample_rates: [u16; INS_MAX_INSTANCES],
    gyro_ids: [u32; INS_MAX_INSTANCES],
    accel_ids: [u32; INS_MAX_INSTANCES],
    /// Sensor-rate sample hooks (batch logging, FFT prep).
    pub sensor_rate_hooks: InsSensorRateHooks,
    next_sitl_bus_id: u8,
}

impl Default for InertialSensorFrontend {
    fn default() -> Self {
        Self::new()
    }
}

impl InertialSensorFrontend {
    #[must_use]
    pub fn new() -> Self {
        Self {
            instances: [ImuInstance::new(), ImuInstance::new(), ImuInstance::new()],
            gyro_count: 0,
            accel_count: 0,
            gyro_raw_sample_rates: [0; INS_MAX_INSTANCES],
            accel_raw_sample_rates: [0; INS_MAX_INSTANCES],
            gyro_ids: [0; INS_MAX_INSTANCES],
            accel_ids: [0; INS_MAX_INSTANCES],
            sensor_rate_hooks: InsSensorRateHooks::default(),
            next_sitl_bus_id: 0,
        }
    }

    /// Peek at the next gyro instance number, upstream `get_gyro_instance`.
    #[must_use]
    pub fn get_gyro_instance(&self) -> Option<u8> {
        if self.gyro_count as usize >= INS_MAX_INSTANCES {
            return None;
        }
        Some(self.gyro_count)
    }

    /// Peek at the next accel instance number, upstream `get_accel_instance`.
    #[must_use]
    pub fn get_accel_instance(&self) -> Option<u8> {
        if self.accel_count as usize >= INS_MAX_INSTANCES {
            return None;
        }
        Some(self.accel_count)
    }

    /// Register a gyro, upstream `register_gyro`. Returns the instance or None if full/duplicate.
    pub fn register_gyro(&mut self, raw_sample_rate_hz: u16, id: u32) -> Option<u8> {
        if self.gyro_count as usize >= INS_MAX_INSTANCES {
            return None;
        }
        for i in 0..self.gyro_count as usize {
            if self.gyro_ids[i] == id {
                return None;
            }
        }
        let instance = self.gyro_count;
        self.gyro_raw_sample_rates[instance as usize] = raw_sample_rate_hz;
        self.gyro_ids[instance as usize] = id;
        self.gyro_count += 1;
        Some(instance)
    }

    /// Register an accelerometer, upstream `register_accel`.
    pub fn register_accel(&mut self, raw_sample_rate_hz: u16, id: u32) -> Option<u8> {
        if self.accel_count as usize >= INS_MAX_INSTANCES {
            return None;
        }
        for i in 0..self.accel_count as usize {
            if self.accel_ids[i] == id {
                return None;
            }
        }
        let instance = self.accel_count;
        self.accel_raw_sample_rates[instance as usize] = raw_sample_rate_hz;
        self.accel_ids[instance as usize] = id;
        self.accel_count += 1;
        Some(instance)
    }

    /// Effective gyro rate including oversampling (oversampling is 1 here).
    #[must_use]
    pub fn get_gyro_rate_hz(&self, instance: u8) -> u16 {
        self.gyro_raw_sample_rates
            .get(instance as usize)
            .copied()
            .unwrap_or(0)
    }

    /// Effective accel rate including oversampling (oversampling is 1 here).
    #[must_use]
    pub fn get_accel_rate_hz(&self, instance: u8) -> u16 {
        self.accel_raw_sample_rates
            .get(instance as usize)
            .copied()
            .unwrap_or(0)
    }

    #[must_use]
    pub fn gyro_count(&self) -> u8 {
        self.gyro_count
    }

    #[must_use]
    pub fn accel_count(&self) -> u8 {
        self.accel_count
    }

    /// Register both sensors for one SITL backend, upstream `AP_InertialSensor_SITL::start`.
    ///
    /// Returns `(gyro_instance, accel_instance)` or None on failure.
    pub fn register_sitl_backend(
        &mut self,
        gyro_rate_hz: u16,
        accel_rate_hz: u16,
    ) -> Option<(u8, u8)> {
        let bus_id = self.next_sitl_bus_id;
        let gyro_id = sitl_bus_id(bus_id, SITL_GYRO_DEVNUM);
        let accel_id = sitl_bus_id(bus_id, SITL_ACCEL_DEVNUM);
        let gyro = self.register_gyro(gyro_rate_hz, gyro_id)?;
        let accel = self.register_accel(accel_rate_hz, accel_id)?;
        self.next_sitl_bus_id = self.next_sitl_bus_id.wrapping_add(1);
        Some((gyro, accel))
    }

    /// Deliver a sensor-rate gyro sample, upstream `_notify_new_gyro_sensor_rate_sample`.
    pub fn notify_gyro_sensor_rate(&self, instance: u8, gyro: Vector3f) {
        self.sensor_rate_hooks.notify_gyro(instance, gyro);
    }

    /// Deliver a sensor-rate accel sample, upstream `_notify_new_accel_sensor_rate_sample`.
    pub fn notify_accel_sensor_rate(&self, instance: u8, accel: Vector3f) {
        self.sensor_rate_hooks.notify_accel(instance, accel);
    }

    /// Mutable access to the IMU instance for a slot.
    pub fn imu_mut(&mut self, instance: u8) -> Option<&mut ImuInstance> {
        self.instances.get_mut(instance as usize)
    }

    /// Read-only access to the IMU instance for a slot.
    #[must_use]
    pub fn imu(&self, instance: u8) -> Option<&ImuInstance> {
        self.instances.get(instance as usize)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_gyro_assigns_sequential_instances() {
        let mut fe = InertialSensorFrontend::new();
        let id0 = sitl_bus_id(0, SITL_GYRO_DEVNUM);
        let id1 = sitl_bus_id(1, SITL_GYRO_DEVNUM);
        assert_eq!(fe.register_gyro(8000, id0), Some(0));
        assert_eq!(fe.register_gyro(1000, id1), Some(1));
        assert_eq!(fe.gyro_count(), 2);
        assert_eq!(fe.get_gyro_rate_hz(0), 8000);
        assert_eq!(fe.get_gyro_rate_hz(1), 1000);
    }

    #[test]
    fn duplicate_gyro_id_is_rejected() {
        let mut fe = InertialSensorFrontend::new();
        let id = sitl_bus_id(0, SITL_GYRO_DEVNUM);
        assert_eq!(fe.register_gyro(8000, id), Some(0));
        assert_eq!(fe.register_gyro(1000, id), None);
    }

    #[test]
    fn register_sitl_backend_increments_bus_id() {
        let mut fe = InertialSensorFrontend::new();
        let (g0, a0) = fe.register_sitl_backend(8000, 1000).unwrap();
        let (g1, a1) = fe.register_sitl_backend(8000, 1000).unwrap();
        assert_eq!((g0, a0), (0, 0));
        assert_eq!((g1, a1), (1, 1));
        assert_eq!(fe.gyro_count(), 2);
        assert_eq!(fe.accel_count(), 2);
    }

    #[test]
    fn sensor_rate_hooks_fire() {
        static mut GYRO_SEEN: u32 = 0;
        fn hook(_inst: u8, _v: Vector3f) {
            unsafe {
                GYRO_SEEN += 1;
            }
        }
        let mut fe = InertialSensorFrontend::new();
        fe.sensor_rate_hooks.on_gyro = Some(hook);
        fe.notify_gyro_sensor_rate(0, Vector3f::new(1.0, 0.0, 0.0));
        unsafe {
            assert_eq!(GYRO_SEEN, 1);
        }
    }
}
