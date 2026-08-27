//! Inertial sensor frontend: instance registration and sensor-rate hooks.
//! Upstream `AP_InertialSensor` register_gyro/register_accel and
//! `_notify_new_*_sensor_rate_sample`, primary instance selection, and
//! the flight-loop `update()` pass. FW-011.

use ap_math::vector3::Vector3f;

use crate::hntch_params::{InsHntchParams, INS_HNTCH_MAX_MOTORS};
use crate::{ImuInstance, LoopTiming, DEFAULT_GYRO_FILTER_HZ};

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
    /// INS_USE / INS_USE2 / INS_USE3 — which instances participate.
    ins_use: [bool; INS_MAX_INSTANCES],
    /// Primary IMU for AHRS/FFT, upstream `_primary`.
    primary: u8,
    /// First healthy enabled gyro, upstream `_first_usable_gyro`.
    first_usable_gyro: u8,
    /// First healthy enabled accel, upstream `_first_usable_accel`.
    first_usable_accel: u8,
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
            ins_use: [true, true, true],
            primary: 0,
            first_usable_gyro: 0,
            first_usable_accel: 0,
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

    /// Whether instance `i` is enabled, upstream `_use(i)`.
    #[must_use]
    pub fn ins_use(&self, instance: u8) -> bool {
        self.ins_use
            .get(instance as usize)
            .copied()
            .unwrap_or(false)
    }

    /// Enable or disable an instance (INS_USE parameters).
    pub fn set_ins_use(&mut self, instance: u8, enabled: bool) {
        if let Some(slot) = self.ins_use.get_mut(instance as usize) {
            *slot = enabled;
        }
    }

    /// Force INS_USE3 on when INS_USE and INS_USE2 are set and INS_USE3 is off,
    /// upstream init safety for boards that require three IMUs.
    pub fn apply_ins_use_safety(&mut self) {
        if self.ins_use[0] && self.ins_use[1] && !self.ins_use[2] {
            self.ins_use[2] = true;
        }
    }

    /// Primary IMU index, upstream `_primary`.
    #[must_use]
    pub const fn primary(&self) -> u8 {
        self.primary
    }

    /// Notify AHRS of a new primary, upstream `set_primary`.
    pub fn set_primary(&mut self, instance: u8) {
        self.primary = instance;
    }

    /// First healthy enabled gyro, upstream `get_first_usable_gyro`.
    #[must_use]
    pub const fn first_usable_gyro(&self) -> u8 {
        self.first_usable_gyro
    }

    /// First healthy enabled accel, upstream `get_first_usable_accel`.
    #[must_use]
    pub const fn first_usable_accel(&self) -> u8 {
        self.first_usable_accel
    }

    /// Gyro health for an enabled instance, upstream `get_gyro_health(instance) && _use(instance)`.
    #[must_use]
    pub fn gyro_usable(&self, instance: u8) -> bool {
        self.ins_use(instance)
            && self
                .instances
                .get(instance as usize)
                .is_some_and(|imu| imu.gyro_healthy())
    }

    /// Accel health for an enabled instance.
    #[must_use]
    pub fn accel_usable(&self, instance: u8) -> bool {
        self.ins_use(instance)
            && self
                .instances
                .get(instance as usize)
                .is_some_and(|imu| imu.accel_healthy())
    }

    /// Primary IMU accumulation state.
    #[must_use]
    pub fn primary_imu(&self) -> Option<&ImuInstance> {
        self.imu(self.primary)
    }

    /// Mutable primary IMU.
    pub fn primary_imu_mut(&mut self) -> Option<&mut ImuInstance> {
        self.imu_mut(self.primary)
    }


    /// Copy a backend's accumulated IMU state into an instance slot before
    /// publish, upstream the SITL backend handoff into `_update`.
    pub fn receive_backend_imu(&mut self, instance: u8, imu: &ImuInstance) {
        if let Some(slot) = self.imu_mut(instance) {
            *slot = imu.clone();
        }
    }

    /// Published gyro from the primary IMU, upstream `get_gyro()`.
    #[must_use]
    pub fn get_gyro(&self) -> Vector3f {
        self.primary_imu()
            .map(ImuInstance::gyro)
            .unwrap_or(Vector3f::zero())
    }

    /// Published accelerometer from the primary IMU, upstream `get_accel()`.
    #[must_use]
    pub fn get_accel(&self) -> Vector3f {
        self.primary_imu()
            .map(ImuInstance::accel)
            .unwrap_or(Vector3f::zero())
    }

    /// Delta angle from the primary IMU since the last publish, upstream
    /// `get_delta_angle`.
    #[must_use]
    pub fn get_delta_angle(&self, timing: &LoopTiming) -> Option<(Vector3f, f32)> {
        self.primary_imu()?.get_delta_angle(timing)
    }

    /// Delta velocity from the primary IMU since the last publish, upstream
    /// `get_delta_velocity`.
    #[must_use]
    pub fn get_delta_velocity(&self, timing: &LoopTiming) -> Option<(Vector3f, f32)> {
        self.primary_imu()?.get_delta_velocity(timing)
    }

    /// Primary gyro health, upstream `get_gyro_health()`.
    #[must_use]
    pub fn get_gyro_health(&self) -> bool {
        self.gyro_usable(self.primary)
    }

    /// Primary accelerometer health, upstream `get_accel_health()`.
    #[must_use]
    pub fn get_accel_health(&self) -> bool {
        self.accel_usable(self.primary)
    }


    /// Retune gyro filters from INS_HNTCH_* and INS_GYRO_FILTER, upstream
    /// `AP_InertialSensor::update_gyro_filters`.
    pub fn update_gyro_filters(
        &mut self,
        hntch: &InsHntchParams,
        gyro_filter_hz: f32,
        motor_count: u8,
    ) {
        let apply_all = hntch.enable_on_all_imus();
        for i in 0..self.gyro_count as usize {
            let instance = i as u8;
            let apply_notch = hntch.enable && (apply_all || instance == self.primary);
            let rate = f32::from(self.get_gyro_rate_hz(instance));
            if let Some(imu) = self.imu_mut(instance) {
                if apply_notch && hntch.num_notches(motor_count) > 0 {
                    imu.set_gyro_notch(
                        rate,
                        hntch.harmonic_notch_params(),
                        hntch.num_notches(motor_count),
                    );
                }
                imu.set_gyro_filter(rate, gyro_filter_hz);
            }
        }
    }

    /// Update harmonic notch centre frequencies from throttle/RPM, upstream
    /// `AP_Vehicle::update_dynamic_notch` + `HarmonicNotch::update_params`.
    pub fn update_dynamic_notch(
        &mut self,
        hntch: &InsHntchParams,
        throttle: f32,
        motor_rpm: &[f32; INS_HNTCH_MAX_MOTORS],
        motor_count: u8,
    ) {
        if !hntch.enable || hntch.tracking_mode() == ap_filter::harmonic::TrackingMode::Fixed {
            return;
        }
        let apply_all = hntch.enable_on_all_imus();
        for i in 0..self.gyro_count as usize {
            let instance = i as u8;
            if !apply_all && instance != self.primary {
                continue;
            }
            if let Some(imu) = self.imu_mut(instance) {
                hntch.update_notch_centers(imu, throttle, motor_rpm, motor_count);
            }
        }
    }

    /// Mark every instance unhealthy before backends publish, upstream
    /// `AP_InertialSensor::update`.
    pub fn begin_update(&mut self) {
        for imu in self.instances.iter_mut().take(self.gyro_count as usize) {
            imu.clear_health();
        }
    }

    /// Publish accumulated samples and recompute primary selection, upstream
    /// `AP_InertialSensor::update` after backend `update()`.
    pub fn update(&mut self) {
        for i in 0..self.gyro_count as usize {
            self.instances[i].update_gyro();
            self.instances[i].update_accel();
        }
        self.select_primary();
    }

    fn select_primary(&mut self) {
        self.apply_ins_use_safety();

        self.first_usable_gyro = 0;
        for i in 0..INS_MAX_INSTANCES {
            if self.gyro_usable(i as u8) {
                self.first_usable_gyro = i as u8;
                break;
            }
        }

        self.first_usable_accel = 0;
        for i in 0..INS_MAX_INSTANCES {
            if self.accel_usable(i as u8) {
                self.first_usable_accel = i as u8;
                break;
            }
        }

        // Upstream sets `_primary = _first_usable_gyro` when AP_AHRS is disabled;
        // Plane always has AHRS but AHRS may override via `set_primary` later.
        self.primary = self.first_usable_gyro;
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

    fn feed_gyro(imu: &mut ImuInstance, t: &mut u64) {
        *t += 125;
        imu.notify_gyro_raw_sample(Vector3f::new(0.5, 0.0, 0.0), *t, 8000, *t);
    }

    fn feed_accel(imu: &mut ImuInstance, t: &mut u64) {
        *t += 125;
        imu.notify_accel_raw_sample(Vector3f::new(0.0, 0.0, -9.80665), *t, 8000, *t);
    }

    #[test]
    fn primary_skips_disabled_instance() {
        let mut fe = InertialSensorFrontend::new();
        fe.register_sitl_backend(8000, 1000).unwrap();
        fe.register_sitl_backend(8000, 1000).unwrap();
        fe.set_ins_use(0, false);

        let mut t = 1_000_000_u64;
        for _ in 0..801 {
            feed_gyro(&mut fe.instances[1], &mut t);
            feed_accel(&mut fe.instances[1], &mut t);
        }
        fe.begin_update();
        fe.update();

        assert_eq!(fe.first_usable_gyro(), 1);
        assert_eq!(fe.first_usable_accel(), 1);
        assert_eq!(fe.primary(), 1);
        assert!(!fe.gyro_usable(0));
        assert!(fe.gyro_usable(1));
    }

    #[test]
    fn ins_use_safety_forces_third_imu() {
        let mut fe = InertialSensorFrontend::new();
        fe.set_ins_use(0, true);
        fe.set_ins_use(1, true);
        fe.set_ins_use(2, false);
        fe.apply_ins_use_safety();
        assert!(fe.ins_use(2));
    }


    #[test]
    fn publish_accessors_read_primary_after_update() {
        let mut fe = InertialSensorFrontend::new();
        fe.register_sitl_backend(8000, 1000).unwrap();

        let mut t = 1_000_000_u64;
        for _ in 0..801 {
            feed_gyro(&mut fe.instances[0], &mut t);
            feed_accel(&mut fe.instances[0], &mut t);
        }
        fe.begin_update();
        fe.update();

        let timing = LoopTiming::new(0.0025);
        assert!(fe.get_gyro_health());
        assert!(fe.get_accel_health());
        assert!(fe.get_gyro().x > 0.0);
        assert!(fe.get_accel().z < 0.0);
        assert!(fe.get_delta_angle(&timing).is_some());
        assert!(fe.get_delta_velocity(&timing).is_some());
    }

    #[test]
    fn update_gyro_filters_applies_notch_on_primary() {
        let mut fe = InertialSensorFrontend::new();
        fe.register_sitl_backend(8000, 1000).unwrap();
        let hntch = InsHntchParams {
            enable: true,
            freq_hz: 80.0,
            bandwidth_hz: 40.0,
            attenuation_db: 40.0,
            harmonics: 1,
            ..InsHntchParams::default()
        };
        fe.update_gyro_filters(&hntch, DEFAULT_GYRO_FILTER_HZ, 1);
        assert!(fe.imu(0).unwrap().gyro_notch_is_initialised());
    }

    #[test]
    fn update_dynamic_notch_retunes_from_throttle() {
        let mut fe = InertialSensorFrontend::new();
        fe.register_sitl_backend(1000, 1000).unwrap();
        let hntch = InsHntchParams {
            enable: true,
            freq_hz: 100.0,
            bandwidth_hz: 40.0,
            attenuation_db: 40.0,
            harmonics: 1,
            reference: 1.0,
            mode: 1,
            freq_min_ratio: 0.5,
            ..InsHntchParams::default()
        };
        fe.update_gyro_filters(&hntch, DEFAULT_GYRO_FILTER_HZ, 1);
        fe.update_dynamic_notch(&hntch, 1.0, &[0.0; INS_HNTCH_MAX_MOTORS], 1);
        for _ in 0..32 {
            fe.update_dynamic_notch(&hntch, 0.25, &[0.0; INS_HNTCH_MAX_MOTORS], 1);
        }
        let center = fe.imu(0).unwrap().gyro_notch_center(0).expect("notch");
        assert!((center - 50.0).abs() < 1.0, "quarter throttle -> 50 Hz, got {center}");
    }

    #[test]
    fn set_primary_overrides_auto_selection() {
        let mut fe = InertialSensorFrontend::new();
        fe.register_sitl_backend(8000, 1000).unwrap();
        fe.register_sitl_backend(8000, 1000).unwrap();

        let mut t = 1_000_000_u64;
        for _ in 0..801 {
            feed_gyro(&mut fe.instances[0], &mut t);
            feed_accel(&mut fe.instances[0], &mut t);
        }
        fe.begin_update();
        fe.update();
        assert_eq!(fe.primary(), 0);

        fe.set_primary(1);
        assert_eq!(fe.primary(), 1);
        assert_eq!(fe.primary_imu().unwrap().gyro().x, 0.0);
    }
}
