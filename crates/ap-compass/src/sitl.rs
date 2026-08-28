//! SITL compass backend, upstream `AP_Compass_SITL::_timer`. FW-014.
//!
//! Rotates the WMM earth-frame field into body frame using true attitude.
//! Applies `COMPASS_OFS` (`mag += offsets`) then `COMPASS_MOT * thr_or_curr`
//! when `COMPASS_MOTCT` is current or throttle. Optional SITL hard-iron bias is
//! added before offsets so learn-offsets can cancel metal in the frame.
//! No noise or delay ring in this slice.

use ap_declination::get_mag_field_ef;
use ap_math::matrix3::Matrix3f;
use ap_math::scalar::{radians, Real};
use ap_math::vector3::Vector3f;

use crate::motor_comp::apply_motor_compensation;
use crate::offset::{apply_offsets, learn_offsets, offsets_within_max};

/// Minimum interval between compass updates, upstream `_timer` at 100 Hz.
pub const SITL_COMPASS_UPDATE_MS: u32 = 10;

/// Maximum SITL compass instances registered in the cluster.
pub const SITL_COMPASS_MAX_INSTANCES: usize = 2;

/// Body-frame magnetic sample from one backend read, upstream `get_field()`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct MagSampleState {
    pub mag_body: Vector3f,
    pub declination_rad: f32,
    pub have_sample: bool,
    pub last_sample_time_ms: u32,
}

/// Per-instance SITL compass parameters, upstream `SITL::MagParams` subset.
#[derive(Debug, Clone, Copy)]
pub struct SitlCompassConfig {
    pub disabled: bool,
    /// Hard-iron offset added to the sample, upstream `COMPASS_OFS`.
    pub offset: Vector3f,
    /// SITL-injected metal bias added before offsets, for learn tests.
    pub hardiron_bias: Vector3f,
    /// Motor compensation factors, upstream `COMPASS_MOT`.
    pub motor_compensation: Vector3f,
    /// Motor compensation type, upstream `COMPASS_MOTCT`.
    pub motor_comp_type: u8,
}

impl Default for SitlCompassConfig {
    fn default() -> Self {
        Self {
            disabled: false,
            offset: Vector3f::zero(),
            hardiron_bias: Vector3f::zero(),
            motor_compensation: Vector3f::zero(),
            motor_comp_type: 0,
        }
    }
}

/// Earth-frame NED field and declination at `lat`/`lon`, upstream
/// `update_mag_field_bf()` before body rotation.
#[must_use]
pub fn mag_field_ef_ned(latitude_deg: f32, longitude_deg: f32) -> (Vector3f, f32) {
    let (field, _coverage) = get_mag_field_ef(latitude_deg, longitude_deg);
    let declination_rad = radians(field.declination_deg);
    let inclination_rad = radians(field.inclination_deg);
    let intensity = field.intensity_gauss;
    let horizontal = intensity * Real::cos(inclination_rad);
    let mag_ef = Vector3f::new(
        horizontal * Real::cos(declination_rad),
        horizontal * Real::sin(declination_rad),
        intensity * Real::sin(inclination_rad),
    );
    (mag_ef, declination_rad)
}

/// Rotate the WMM earth-frame field into body frame, upstream
/// `dcm.transposed() * mag_ef` in `update_mag_field_bf()`.
#[must_use]
pub fn mag_field_body_ned(
    latitude_deg: f32,
    longitude_deg: f32,
    attitude: Matrix3f,
) -> (Vector3f, f32) {
    let (mag_ef, declination_rad) = mag_field_ef_ned(latitude_deg, longitude_deg);
    let mag_body = attitude.transposed() * mag_ef;
    (mag_body, declination_rad)
}

/// SITL compass backend producer, upstream `AP_Compass_SITL::_timer` / `update`.
#[derive(Debug, Clone)]
pub struct SitlCompassBackend {
    config: SitlCompassConfig,
    last_sample_time_ms: u32,
    pending: MagSampleState,
    has_pending: bool,
    raw_mag_body: Vector3f,
    thr_or_curr: f32,
    last_latitude_deg: f32,
    last_longitude_deg: f32,
    last_attitude: Matrix3f,
}

impl Default for SitlCompassBackend {
    fn default() -> Self {
        Self {
            config: SitlCompassConfig::default(),
            last_sample_time_ms: 0,
            pending: MagSampleState::default(),
            has_pending: false,
            raw_mag_body: Vector3f::zero(),
            thr_or_curr: 0.0,
            last_latitude_deg: 0.0,
            last_longitude_deg: 0.0,
            last_attitude: Matrix3f::identity(),
        }
    }
}

impl SitlCompassBackend {
    #[must_use]
    pub fn with_config(config: SitlCompassConfig) -> Self {
        Self {
            config,
            ..Self::default()
        }
    }

    #[must_use]
    pub const fn config(&self) -> &SitlCompassConfig {
        &self.config
    }

    #[must_use]
    pub const fn state(&self) -> &MagSampleState {
        &self.pending
    }

    /// Upstream `healthy()`: seen at least one tick and compass not disabled.
    #[must_use]
    pub fn healthy(&self) -> bool {
        self.last_sample_time_ms != 0 && !self.config.disabled
    }

    /// Raw body-frame field before `COMPASS_OFS`, including SITL hard-iron.
    #[must_use]
    pub const fn raw_mag_body(&self) -> Vector3f {
        self.raw_mag_body
    }

    pub fn set_config(&mut self, config: SitlCompassConfig) {
        self.config = config;
    }

    /// Throttle `0..1` or battery current in amps, upstream `_thr` / battery.
    pub fn set_thr_or_curr(&mut self, value: f32) {
        self.thr_or_curr = value;
    }

    #[must_use]
    pub const fn thr_or_curr(&self) -> f32 {
        self.thr_or_curr
    }

    /// Latch `COMPASS_OFS` so the corrected field matches WMM, upstream learn.
    #[must_use]
    pub fn learn_offset(&mut self, offsets_max: f32) -> bool {
        if self.config.disabled || self.last_sample_time_ms == 0 {
            return false;
        }
        let (expected, _) = mag_field_body_ned(
            self.last_latitude_deg,
            self.last_longitude_deg,
            self.last_attitude,
        );
        let ofs = learn_offsets(self.raw_mag_body, expected);
        if !offsets_within_max(ofs, offsets_max) {
            return false;
        }
        self.config.offset = ofs;
        true
    }

    /// Run the 100 Hz timer path from sim truth and true attitude.
    pub fn timer_tick(
        &mut self,
        latitude_deg: f32,
        longitude_deg: f32,
        attitude: Matrix3f,
        now_ms: u32,
    ) -> bool {
        if now_ms.wrapping_sub(self.last_sample_time_ms) < SITL_COMPASS_UPDATE_MS {
            return false;
        }
        self.last_sample_time_ms = now_ms;

        if self.config.disabled {
            return false;
        }

        self.last_latitude_deg = latitude_deg;
        self.last_longitude_deg = longitude_deg;
        self.last_attitude = attitude;
        let (wmm, declination_rad) = mag_field_body_ned(latitude_deg, longitude_deg, attitude);
        self.raw_mag_body = wmm + self.config.hardiron_bias;
        let mag_body = apply_offsets(self.raw_mag_body, self.config.offset);
        let mag_body = apply_motor_compensation(
            mag_body,
            self.config.motor_compensation,
            self.config.motor_comp_type,
            self.thr_or_curr,
        );
        self.pending = MagSampleState {
            mag_body,
            declination_rad,
            have_sample: true,
            last_sample_time_ms: now_ms,
        };
        self.has_pending = true;
        true
    }

    /// Copy the pending sample to the frontend, upstream `update()`.
    #[must_use]
    pub fn update(&mut self) -> Option<MagSampleState> {
        if !self.has_pending {
            return None;
        }
        self.has_pending = false;
        Some(self.pending)
    }
}

/// Per-instance compass health, upstream `AP_Compass` frontend flags.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CompassHealthFlags {
    pub instance_count: u8,
    pub healthy: [bool; SITL_COMPASS_MAX_INSTANCES],
    pub have_sample: [bool; SITL_COMPASS_MAX_INSTANCES],
    pub primary: u8,
}

impl CompassHealthFlags {
    #[must_use]
    pub fn any_healthy(&self) -> bool {
        self.healthy[..self.instance_count as usize]
            .iter()
            .any(|&healthy| healthy)
    }

    #[must_use]
    pub fn primary_healthy(&self) -> bool {
        let i = self.primary as usize;
        i < self.instance_count as usize && self.healthy[i] && self.have_sample[i]
    }
}

/// Multi-instance SITL compass cluster, upstream `AP_Compass` instance list.
#[derive(Debug, Clone)]
pub struct SitlCompassCluster {
    backends: [SitlCompassBackend; SITL_COMPASS_MAX_INSTANCES],
    instance_count: u8,
    primary: u8,
}

impl Default for SitlCompassCluster {
    fn default() -> Self {
        let mut cluster = Self {
            backends: [SitlCompassBackend::default(), SitlCompassBackend::default()],
            instance_count: 0,
            primary: 0,
        };
        let _ = cluster.register(SitlCompassBackend::default());
        cluster
    }
}

impl SitlCompassCluster {
    #[must_use]
    pub const fn instance_count(&self) -> u8 {
        self.instance_count
    }

    #[must_use]
    pub const fn primary(&self) -> u8 {
        self.primary
    }

    pub fn set_primary(&mut self, index: u8) {
        if (index as usize) < self.instance_count as usize {
            self.primary = index;
        }
    }

    /// Same throttle/current on every registered instance.
    pub fn set_thr_or_curr(&mut self, value: f32) {
        for i in 0..self.instance_count as usize {
            self.backends[i].set_thr_or_curr(value);
        }
    }

    pub fn register(&mut self, backend: SitlCompassBackend) -> Result<u8, ()> {
        let idx = self.instance_count;
        if idx as usize >= SITL_COMPASS_MAX_INSTANCES {
            return Err(());
        }
        self.backends[idx as usize] = backend;
        self.instance_count = idx.saturating_add(1);
        Ok(idx)
    }

    pub fn select_primary_healthy(&mut self) {
        for i in 0..self.instance_count as usize {
            if self.backends[i].healthy() && self.backends[i].state().have_sample {
                self.primary = i as u8;
                return;
            }
        }
    }

    #[must_use]
    pub fn backend(&self, index: u8) -> Option<&SitlCompassBackend> {
        (index < self.instance_count).then(|| &self.backends[index as usize])
    }

    pub fn backend_mut(&mut self, index: u8) -> Option<&mut SitlCompassBackend> {
        (index < self.instance_count).then(|| &mut self.backends[index as usize])
    }

    pub fn timer_tick_all(
        &mut self,
        latitude_deg: f32,
        longitude_deg: f32,
        attitude: Matrix3f,
        now_ms: u32,
    ) {
        for i in 0..self.instance_count as usize {
            self.backends[i].timer_tick(latitude_deg, longitude_deg, attitude, now_ms);
        }
    }

    #[must_use]
    pub fn primary_sample(&mut self) -> Option<MagSampleState> {
        let primary = self.primary as usize;
        self.backends[primary]
            .update()
            .or_else(|| {
                let sample = *self.backends[primary].state();
                sample.have_sample.then_some(sample)
            })
    }

    /// Learn offsets on every enabled instance, upstream `Compass::learn_offsets`.
    #[must_use]
    pub fn learn_offsets(&mut self, offsets_max: f32) -> bool {
        let mut any = false;
        for i in 0..self.instance_count as usize {
            if self.backends[i].learn_offset(offsets_max) {
                any = true;
            }
        }
        any
    }

    #[must_use]
    pub fn health_flags(&self) -> CompassHealthFlags {
        let mut flags = CompassHealthFlags {
            instance_count: self.instance_count,
            primary: self.primary,
            ..CompassHealthFlags::default()
        };
        for i in 0..self.instance_count as usize {
            flags.healthy[i] = self.backends[i].healthy();
            flags.have_sample[i] = self.backends[i].state().have_sample;
        }
        flags
    }

    #[must_use]
    pub fn cluster_with_disabled_primary() -> Self {
        Self {
            backends: [
                SitlCompassBackend::with_config(SitlCompassConfig {
                    disabled: true,
                    ..SitlCompassConfig::default()
                }),
                SitlCompassBackend::default(),
            ],
            instance_count: 2,
            primary: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_math::scalar::degrees;

    fn level_attitude() -> Matrix3f {
        Matrix3f::identity()
    }

    #[test]
    fn rate_limit_gates_first_sample_until_10ms() {
        let mut compass = SitlCompassBackend::default();
        assert!(!compass.timer_tick(51.875, -0.154, level_attitude(), 0));
        assert!(!compass.state().have_sample);
        assert!(compass.timer_tick(51.875, -0.154, level_attitude(), 10));
        assert!(compass.state().have_sample);
    }

    #[test]
    fn producer_emits_nonzero_body_field_at_default_location() {
        let mut compass = SitlCompassBackend::default();
        assert!(compass.timer_tick(51.875, -0.154, level_attitude(), 10));
        let sample = compass.update().expect("pending sample");
        assert!(sample.have_sample);
        assert!(sample.mag_body.length() > 0.1);
        assert!(sample.declination_rad.abs() > 0.0);
    }

    #[test]
    fn producer_unhealthy_when_disabled() {
        let mut compass = SitlCompassBackend::with_config(SitlCompassConfig {
            disabled: true,
            ..SitlCompassConfig::default()
        });
        assert!(!compass.timer_tick(51.875, -0.154, level_attitude(), 10));
        assert!(!compass.healthy());
    }

    #[test]
    fn yaw_rotation_changes_horizontal_components() {
        let yaw90 = Matrix3f::from_euler(0.0, 0.0, degrees(90.0));
        let (level, _) = mag_field_body_ned(51.875, -0.154, level_attitude());
        let (rotated, _) = mag_field_body_ned(51.875, -0.154, yaw90);
        assert!((level.x - rotated.y).abs() < 0.05 || (level.y - rotated.x).abs() < 0.05);
    }

    #[test]
    fn cluster_health_flags_track_disabled_primary() {
        let mut cluster = SitlCompassCluster::cluster_with_disabled_primary();
        cluster.timer_tick_all(51.875, -0.154, level_attitude(), 10);
        let flags = cluster.health_flags();
        assert_eq!(flags.instance_count, 2);
        assert!(!flags.healthy[0]);
        assert!(flags.healthy[1]);
    }
}
