//! SITL airspeed backend, upstream `AP_Airspeed_SITL::_timer`. FW-010.
//!
//! Pitot true airspeed comes from the forward body-frame air-relative velocity.
//! Equivalent airspeed divides by the baro EAS2TAS ratio passed in from the
//! vehicle loop.

use ap_math::scalar::{is_positive, Real};
use ap_math::vector3::Vector3f;

/// Minimum interval between airspeed updates, upstream `_timer` at 100 Hz.
pub const SITL_AIRSPEED_UPDATE_MS: u32 = 10;

/// Maximum SITL airspeed instances registered in the cluster.
pub const SITL_AIRSPEED_MAX_INSTANCES: usize = 2;

/// Pitot sample from one backend read, upstream `get_airspeed()`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct AirspeedSampleState {
    pub tas_mps: f32,
    pub eas_mps: f32,
    pub have_sample: bool,
    pub last_sample_time_ms: u32,
}

/// Per-instance SITL airspeed parameters, upstream `SITL::AirspeedParams` subset.
#[derive(Debug, Clone, Copy)]
pub struct SitlAirspeedConfig {
    pub disabled: bool,
    /// Latched pitot TAS offset, upstream `ARSPD_OFFSET` (m/s stub).
    pub offset_mps: f32,
    /// Skip startup / requested calibration, upstream `ARSPD_SKIP_CAL`.
    pub skip_cal: bool,
}

impl Default for SitlAirspeedConfig {
    fn default() -> Self {
        Self {
            disabled: false,
            offset_mps: 0.0,
            skip_cal: false,
        }
    }
}

/// Pitot true airspeed from body-frame air-relative velocity, upstream
/// `AP_Airspeed_SITL` reading the forward pitot tube.
#[must_use]
pub fn pitot_tas_from_body(airspeed_bf: Vector3f) -> f32 {
    if is_positive(airspeed_bf.x) {
        airspeed_bf.x
    } else {
        0.0
    }
}

/// Equivalent airspeed from true and EAS2TAS, upstream `get_airspeed()`.
#[must_use]
pub fn eas_from_tas(tas_mps: f32, eas2tas: f32) -> f32 {
    if eas2tas > 0.0 {
        tas_mps / eas2tas
    } else {
        tas_mps
    }
}

/// SITL airspeed backend producer, upstream `AP_Airspeed_SITL::_timer` / `update`.
#[derive(Debug, Clone)]
pub struct SitlAirspeedBackend {
    config: SitlAirspeedConfig,
    last_sample_time_ms: u32,
    pending: AirspeedSampleState,
    has_pending: bool,
    /// Raw pitot TAS before offset, used by `calibrate()`.
    raw_tas_mps: f32,
}

impl Default for SitlAirspeedBackend {
    fn default() -> Self {
        Self {
            config: SitlAirspeedConfig::default(),
            last_sample_time_ms: 0,
            pending: AirspeedSampleState::default(),
            has_pending: false,
            raw_tas_mps: 0.0,
        }
    }
}

impl SitlAirspeedBackend {
    #[must_use]
    pub fn with_config(config: SitlAirspeedConfig) -> Self {
        Self {
            config,
            ..Self::default()
        }
    }

    #[must_use]
    pub const fn config(&self) -> &SitlAirspeedConfig {
        &self.config
    }

    #[must_use]
    pub const fn state(&self) -> &AirspeedSampleState {
        &self.pending
    }

    /// Upstream `healthy()`: seen at least one tick and sensor not disabled.
    #[must_use]
    pub fn healthy(&self) -> bool {
        self.last_sample_time_ms != 0 && !self.config.disabled
    }

    /// Raw pitot TAS before offset subtraction.
    #[must_use]
    pub const fn raw_tas_mps(&self) -> f32 {
        self.raw_tas_mps
    }

    pub fn set_config(&mut self, config: SitlAirspeedConfig) {
        self.config = config;
    }

    /// Latch current raw TAS as the pitot offset, upstream `AP_Airspeed::calibrate()`.
    #[must_use]
    pub fn calibrate_offset(&mut self) -> bool {
        if self.config.skip_cal || self.config.disabled || self.last_sample_time_ms == 0 {
            return false;
        }
        self.config.offset_mps = self.raw_tas_mps;
        true
    }

    /// Run the 100 Hz timer path from sim truth and EAS2TAS.
    pub fn timer_tick(
        &mut self,
        airspeed_bf: Vector3f,
        eas2tas: f32,
        now_ms: u32,
    ) -> bool {
        if now_ms.wrapping_sub(self.last_sample_time_ms) < SITL_AIRSPEED_UPDATE_MS {
            return false;
        }
        self.last_sample_time_ms = now_ms;

        if self.config.disabled {
            return false;
        }

        let raw_tas = pitot_tas_from_body(airspeed_bf);
        self.raw_tas_mps = raw_tas;
        let tas_mps = (raw_tas - self.config.offset_mps).max(0.0);
        let eas_mps = eas_from_tas(tas_mps, eas2tas);
        self.pending = AirspeedSampleState {
            tas_mps,
            eas_mps,
            have_sample: true,
            last_sample_time_ms: now_ms,
        };
        self.has_pending = true;
        true
    }

    /// Copy the pending sample to the frontend, upstream `update()`.
    #[must_use]
    pub fn update(&mut self) -> Option<AirspeedSampleState> {
        if !self.has_pending {
            return None;
        }
        self.has_pending = false;
        Some(self.pending)
    }
}

/// Per-instance airspeed health, upstream `AP_Airspeed` frontend flags.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct AirspeedHealthFlags {
    pub instance_count: u8,
    pub healthy: [bool; SITL_AIRSPEED_MAX_INSTANCES],
    pub have_sample: [bool; SITL_AIRSPEED_MAX_INSTANCES],
    pub primary: u8,
}

impl AirspeedHealthFlags {
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

/// Multi-instance SITL airspeed cluster, upstream `AP_Airspeed` instance list.
#[derive(Debug, Clone)]
pub struct SitlAirspeedCluster {
    backends: [SitlAirspeedBackend; SITL_AIRSPEED_MAX_INSTANCES],
    instance_count: u8,
    primary: u8,
}

impl Default for SitlAirspeedCluster {
    fn default() -> Self {
        let mut cluster = Self {
            backends: [SitlAirspeedBackend::default(), SitlAirspeedBackend::default()],
            instance_count: 0,
            primary: 0,
        };
        let _ = cluster.register(SitlAirspeedBackend::default());
        cluster
    }
}

impl SitlAirspeedCluster {
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

    pub fn register(&mut self, backend: SitlAirspeedBackend) -> Result<u8, ()> {
        let idx = self.instance_count;
        if idx as usize >= SITL_AIRSPEED_MAX_INSTANCES {
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
    pub fn backend(&self, index: u8) -> Option<&SitlAirspeedBackend> {
        (index < self.instance_count).then(|| &self.backends[index as usize])
    }

    pub fn timer_tick_all(
        &mut self,
        airspeed_bf: Vector3f,
        eas2tas: f32,
        now_ms: u32,
    ) {
        for i in 0..self.instance_count as usize {
            self.backends[i].timer_tick(airspeed_bf, eas2tas, now_ms);
        }
    }

    #[must_use]
    pub fn primary_sample(&mut self) -> Option<AirspeedSampleState> {
        let primary = self.primary as usize;
        self.backends[primary]
            .update()
            .or_else(|| {
                let sample = *self.backends[primary].state();
                sample.have_sample.then_some(sample)
            })
    }

    /// Calibrate every enabled instance, upstream `AP_Airspeed::calibrate()`.
    #[must_use]
    pub fn calibrate_offsets(&mut self) -> bool {
        let mut any = false;
        for i in 0..self.instance_count as usize {
            if self.backends[i].calibrate_offset() {
                any = true;
            }
        }
        any
    }

    #[must_use]
    pub fn health_flags(&self) -> AirspeedHealthFlags {
        let mut flags = AirspeedHealthFlags {
            instance_count: self.instance_count,
            primary: self.primary,
            ..AirspeedHealthFlags::default()
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
                SitlAirspeedBackend::with_config(SitlAirspeedConfig {
                    disabled: true,
                    ..SitlAirspeedConfig::default()
                }),
                SitlAirspeedBackend::default(),
            ],
            instance_count: 2,
            primary: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pitot_uses_forward_body_component() {
        let tas = pitot_tas_from_body(Vector3f::new(22.0, 3.0, 1.0));
        assert!((tas - 22.0).abs() < 1e-6);
    }

    #[test]
    fn pitot_clamps_negative_forward_to_zero() {
        let tas = pitot_tas_from_body(Vector3f::new(-5.0, 10.0, 0.0));
        assert_eq!(tas, 0.0);
    }

    #[test]
    fn eas_from_tas_divides_by_eas2tas() {
        assert!((eas_from_tas(20.0, 1.25) - 16.0).abs() < 1e-6);
    }

    #[test]
    fn rate_limit_gates_first_sample_until_10ms() {
        let mut backend = SitlAirspeedBackend::default();
        assert!(!backend.timer_tick(Vector3f::new(15.0, 0.0, 0.0), 1.0, 0));
        assert!(!backend.state().have_sample);
        assert!(backend.timer_tick(Vector3f::new(15.0, 0.0, 0.0), 1.0, 10));
        assert!(backend.state().have_sample);
    }

    #[test]
    fn producer_unhealthy_when_disabled() {
        let mut backend = SitlAirspeedBackend::with_config(SitlAirspeedConfig {
            disabled: true,
            ..SitlAirspeedConfig::default()
        });
        assert!(!backend.timer_tick(Vector3f::new(15.0, 0.0, 0.0), 1.0, 10));
        assert!(!backend.healthy());
    }

    #[test]
    fn cluster_health_flags_track_disabled_primary() {
        let mut cluster = SitlAirspeedCluster::cluster_with_disabled_primary();
        cluster.timer_tick_all(Vector3f::new(18.0, 0.0, 0.0), 1.0, 10);
        let flags = cluster.health_flags();
        assert_eq!(flags.instance_count, 2);
        assert!(!flags.healthy[0]);
        assert!(flags.healthy[1]);
    }

    #[test]
    fn calibrate_latches_raw_tas_as_offset() {
        let mut backend = SitlAirspeedBackend::default();
        assert!(backend.timer_tick(Vector3f::new(3.0, 0.0, 0.0), 1.0, 10));
        assert!((backend.state().tas_mps - 3.0).abs() < 1e-6);
        assert!(backend.calibrate_offset());
        assert!((backend.config().offset_mps - 3.0).abs() < 1e-6);
        assert!(backend.timer_tick(Vector3f::new(3.0, 0.0, 0.0), 1.0, 20));
        assert!(backend.state().tas_mps.abs() < 1e-6);
        assert!(backend.timer_tick(Vector3f::new(23.0, 0.0, 0.0), 1.0, 30));
        assert!((backend.state().tas_mps - 20.0).abs() < 1e-6);
    }

    #[test]
    fn skip_cal_leaves_offset_unchanged() {
        let mut backend = SitlAirspeedBackend::with_config(SitlAirspeedConfig {
            skip_cal: true,
            ..SitlAirspeedConfig::default()
        });
        assert!(backend.timer_tick(Vector3f::new(4.0, 0.0, 0.0), 1.0, 10));
        assert!(!backend.calibrate_offset());
        assert_eq!(backend.config().offset_mps, 0.0);
    }

    #[test]
    fn cluster_calibrate_offsets_both_instances() {
        let mut cluster = SitlAirspeedCluster::default();
        let _ = cluster.register(SitlAirspeedBackend::default());
        cluster.timer_tick_all(Vector3f::new(2.5, 0.0, 0.0), 1.0, 10);
        assert!(cluster.calibrate_offsets());
        assert!((cluster.backend(0).unwrap().config().offset_mps - 2.5).abs() < 1e-6);
        assert!((cluster.backend(1).unwrap().config().offset_mps - 2.5).abs() < 1e-6);
    }
}
