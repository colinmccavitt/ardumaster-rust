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

/// Upstream `ARSPD_RATIO` default (pitot tube ratio).
pub const ARSPD_RATIO_DEFAULT: f32 = 2.0;

/// SITL plane default for `ARSPD_USE` (param table default is 0; SITL enables Use).
pub const ARSPD_USE_DEFAULT: u8 = 1;

/// ISA sea-level temperature (deg C). SITL `get_temperature` uses
/// `AP_Baro::get_temperatureC_for_alt_amsl`.
pub const ARSPD_TEMP_REF_C: f32 = 15.0;

/// ISA troposphere lapse rate (K/m) used by the SITL temperature stub.
pub const ISA_LAPSE_K_PER_M: f32 = 0.0065;

/// Default temperature-compensation coefficient (identity).
pub const ARSPD_TEMP_COEFF_DEFAULT: f32 = 0.0;

/// Upstream `ARSPD_AUTOCAL` / `ARSPD2_AUTOCAL` default (disabled).
pub const ARSPD_AUTOCAL_DEFAULT: u8 = 0;

/// Upstream `ARSPD_SKIP_CAL` / `ARSPD2_SKIP_CAL` default (run startup / requested cal).
pub const ARSPD_SKIP_CAL_DEFAULT: bool = false;

/// Pitot sample from one backend read, upstream `get_airspeed()`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct AirspeedSampleState {
    pub tas_mps: f32,
    pub eas_mps: f32,
    pub have_sample: bool,
    pub last_sample_time_ms: u32,
    /// Last pitot / ISA temperature (deg C), upstream `get_temperature()`.
    pub temperature_c: f32,
}

/// Per-instance SITL airspeed parameters, upstream `SITL::AirspeedParams` subset.
#[derive(Debug, Clone, Copy)]
pub struct SitlAirspeedConfig {
    pub disabled: bool,
    /// Latched pitot TAS offset, upstream `ARSPD_OFFSET` (m/s stub).
    pub offset_mps: f32,
    /// Skip startup / requested calibration, upstream `ARSPD_SKIP_CAL`.
    pub skip_cal: bool,
    /// Pitot tube ratio, upstream `ARSPD_RATIO`.
    pub ratio: f32,
    /// Use TAS for TECS/nav, upstream `ARSPD_USE` (0=DoNotUse, 1=Use).
    pub use_airspeed: u8,
    /// Sensor / ISA temperature (deg C), upstream SITL `get_temperature`.
    pub temperature_c: f32,
    /// Linear TAS temperature-compensation coefficient (1/deg C). Zero is identity.
    pub temp_coeff: f32,
    /// Automatic pitot-ratio calibration, upstream `ARSPD_AUTOCAL` (0=off).
    pub autocal: u8,
}

impl Default for SitlAirspeedConfig {
    fn default() -> Self {
        Self {
            disabled: false,
            offset_mps: 0.0,
            skip_cal: ARSPD_SKIP_CAL_DEFAULT,
            ratio: ARSPD_RATIO_DEFAULT,
            use_airspeed: ARSPD_USE_DEFAULT,
            temperature_c: ARSPD_TEMP_REF_C,
            temp_coeff: ARSPD_TEMP_COEFF_DEFAULT,
            autocal: ARSPD_AUTOCAL_DEFAULT,
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

/// Scale pitot TAS by `ARSPD_RATIO` / default, leaving default ratio at unity.
#[must_use]
pub fn apply_pitot_ratio(tas_mps: f32, ratio: f32) -> f32 {
    if ratio > 0.0 {
        (tas_mps * (ratio / ARSPD_RATIO_DEFAULT)).max(0.0)
    } else {
        0.0
    }
}

/// Upstream `AP_Airspeed::use()`: enabled instance with `ARSPD_USE != 0`.
#[must_use]
pub fn use_airspeed_for_control(disabled: bool, use_airspeed: u8) -> bool {
    !disabled && use_airspeed != 0
}

/// ISA temperature at AMSL, upstream SITL `AP_Airspeed_SITL::get_temperature`.
#[must_use]
pub fn sitl_airspeed_temperature_c(alt_amsl_m: f32) -> f32 {
    ARSPD_TEMP_REF_C - ISA_LAPSE_K_PER_M * alt_amsl_m
}

/// Linear temperature compensation of pitot TAS.
/// `tas * (1 + coeff * (temp_c - T_ref))`. Coeff 0 leaves TAS unchanged.
#[must_use]
pub fn apply_temp_compensation(tas_mps: f32, temp_c: f32, coeff: f32) -> f32 {
    (tas_mps * (1.0 + coeff * (temp_c - ARSPD_TEMP_REF_C))).max(0.0)
}

/// One-step `ARSPD_AUTOCAL` ratio update from GPS groundspeed vs pitot TAS.
/// Upstream `AP_Airspeed::update_calibration`: disabled when `autocal == 0`.
/// Enabled: `ratio *= gps_gs / tas`, constrained to `[1, 4]`.
#[must_use]
pub fn apply_autocal_ratio(ratio: f32, gps_gs_mps: f32, tas_mps: f32, autocal: u8) -> f32 {
    if autocal == 0 || !is_positive(gps_gs_mps) || !is_positive(tas_mps) {
        return ratio;
    }
    (ratio * (gps_gs_mps / tas_mps)).clamp(1.0, 4.0)
}

/// TAS consumed by TECS/AHRS nav: zero when `ARSPD_USE` is disabled.
#[must_use]
pub fn tas_for_nav(tas_mps: f32, use_for_control: bool) -> f32 {
    if use_for_control {
        tas_mps
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

    pub fn set_ratio(&mut self, ratio: f32) {
        self.config.ratio = ratio;
    }

    pub fn set_use_airspeed(&mut self, use_airspeed: u8) {
        self.config.use_airspeed = use_airspeed;
    }

    pub fn set_temperature_c(&mut self, temperature_c: f32) {
        self.config.temperature_c = temperature_c;
    }

    pub fn set_temp_coeff(&mut self, temp_coeff: f32) {
        self.config.temp_coeff = temp_coeff;
    }

    pub fn set_autocal(&mut self, autocal: u8) {
        self.config.autocal = autocal;
    }

    pub fn set_skip_cal(&mut self, skip_cal: bool) {
        self.config.skip_cal = skip_cal;
    }

    /// Learn pitot ratio from GPS groundspeed, upstream `update_calibration`.
    pub fn update_autocal(&mut self, gps_gs_mps: f32) {
        if !self.pending.have_sample {
            return;
        }
        let old_ratio = self.config.ratio;
        let new_ratio = apply_autocal_ratio(
            old_ratio,
            gps_gs_mps,
            self.pending.tas_mps,
            self.config.autocal,
        );
        if (new_ratio - old_ratio).abs() <= f32::EPSILON {
            return;
        }
        if is_positive(old_ratio) {
            let scale = new_ratio / old_ratio;
            self.pending.tas_mps = (self.pending.tas_mps * scale).max(0.0);
            self.pending.eas_mps = (self.pending.eas_mps * scale).max(0.0);
        }
        self.config.ratio = new_ratio;
    }

    /// Upstream `AP_Airspeed::use()` for this instance.
    #[must_use]
    pub fn use_for_control(&self) -> bool {
        use_airspeed_for_control(self.config.disabled, self.config.use_airspeed)
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
        let tas_mps = apply_pitot_ratio((raw_tas - self.config.offset_mps).max(0.0), self.config.ratio);
        let tas_mps = apply_temp_compensation(
            tas_mps,
            self.config.temperature_c,
            self.config.temp_coeff,
        );
        let eas_mps = eas_from_tas(tas_mps, eas2tas);
        self.pending = AirspeedSampleState {
            tas_mps,
            eas_mps,
            have_sample: true,
            last_sample_time_ms: now_ms,
            temperature_c: self.config.temperature_c,
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

    pub fn backend_mut(&mut self, index: u8) -> Option<&mut SitlAirspeedBackend> {
        (index < self.instance_count).then(|| &mut self.backends[index as usize])
    }

    pub fn set_ratio_all(&mut self, ratio: f32) {
        for i in 0..self.instance_count as usize {
            self.backends[i].set_ratio(ratio);
        }
    }

    pub fn set_use_airspeed_all(&mut self, use_airspeed: u8) {
        for i in 0..self.instance_count as usize {
            self.backends[i].set_use_airspeed(use_airspeed);
        }
    }

    pub fn set_temperature_all(&mut self, temperature_c: f32) {
        for i in 0..self.instance_count as usize {
            self.backends[i].set_temperature_c(temperature_c);
        }
    }

    pub fn set_temp_coeff_all(&mut self, temp_coeff: f32) {
        for i in 0..self.instance_count as usize {
            self.backends[i].set_temp_coeff(temp_coeff);
        }
    }

    pub fn set_autocal_all(&mut self, autocal: u8) {
        for i in 0..self.instance_count as usize {
            self.backends[i].set_autocal(autocal);
        }
    }

    pub fn set_skip_cal_all(&mut self, skip_cal: bool) {
        for i in 0..self.instance_count as usize {
            self.backends[i].set_skip_cal(skip_cal);
        }
    }

    /// Apply `ARSPD_AUTOCAL` to every enabled instance.
    pub fn update_autocal_all(&mut self, gps_gs_mps: f32) {
        for i in 0..self.instance_count as usize {
            self.backends[i].update_autocal(gps_gs_mps);
        }
    }

    /// Primary instance `AP_Airspeed::use()`.
    #[must_use]
    pub fn primary_use_for_control(&self) -> bool {
        self.backend(self.primary)
            .map(SitlAirspeedBackend::use_for_control)
            .unwrap_or(false)
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

    #[test]
    fn default_ratio_leaves_tas_unchanged() {
        let mut backend = SitlAirspeedBackend::default();
        assert!((backend.config().ratio - ARSPD_RATIO_DEFAULT).abs() < 1e-6);
        assert!(backend.timer_tick(Vector3f::new(16.0, 0.0, 0.0), 1.0, 10));
        assert!((backend.state().tas_mps - 16.0).abs() < 1e-6);
    }

    #[test]
    fn half_ratio_halves_pitot_tas() {
        let mut backend = SitlAirspeedBackend::with_config(SitlAirspeedConfig {
            ratio: 1.0,
            ..SitlAirspeedConfig::default()
        });
        assert!(backend.timer_tick(Vector3f::new(20.0, 0.0, 0.0), 1.0, 10));
        assert!((backend.state().tas_mps - 10.0).abs() < 1e-6);
        assert!((backend.state().eas_mps - 10.0).abs() < 1e-6);
    }

    #[test]
    fn use_airspeed_zero_disables_control_gate() {
        assert!(use_airspeed_for_control(false, 1));
        assert!(!use_airspeed_for_control(false, 0));
        assert!(!use_airspeed_for_control(true, 1));
        assert_eq!(tas_for_nav(20.0, false), 0.0);
        assert!((tas_for_nav(20.0, true) - 20.0).abs() < 1e-6);
        let mut backend = SitlAirspeedBackend::default();
        assert!(backend.use_for_control());
        backend.set_use_airspeed(0);
        assert!(!backend.use_for_control());
        assert!(backend.timer_tick(Vector3f::new(16.0, 0.0, 0.0), 1.0, 10));
        assert!(backend.healthy());
        assert!((backend.state().tas_mps - 16.0).abs() < 1e-6);
    }

    #[test]
    fn sitl_temperature_follows_isa_lapse() {
        assert!((sitl_airspeed_temperature_c(0.0) - ARSPD_TEMP_REF_C).abs() < 1e-6);
        assert!((sitl_airspeed_temperature_c(1000.0) - 8.5).abs() < 1e-6);
    }

    #[test]
    fn default_temp_coeff_leaves_tas_unchanged() {
        let mut backend = SitlAirspeedBackend::default();
        assert!((backend.config().temperature_c - ARSPD_TEMP_REF_C).abs() < 1e-6);
        assert!((backend.config().temp_coeff - ARSPD_TEMP_COEFF_DEFAULT).abs() < 1e-6);
        assert!(backend.timer_tick(Vector3f::new(18.0, 0.0, 0.0), 1.0, 10));
        assert!((backend.state().tas_mps - 18.0).abs() < 1e-6);
        assert!((backend.state().temperature_c - ARSPD_TEMP_REF_C).abs() < 1e-6);
    }

    #[test]
    fn temp_coeff_scales_tas_with_temperature_delta() {
        let mut backend = SitlAirspeedBackend::with_config(SitlAirspeedConfig {
            temperature_c: 25.0,
            temp_coeff: 0.01,
            ..SitlAirspeedConfig::default()
        });
        assert!(backend.timer_tick(Vector3f::new(20.0, 0.0, 0.0), 1.0, 10));
        // 20 * (1 + 0.01 * (25 - 15)) = 22
        assert!((backend.state().tas_mps - 22.0).abs() < 1e-6);
        assert!((backend.state().eas_mps - 22.0).abs() < 1e-6);
        assert!((backend.state().temperature_c - 25.0).abs() < 1e-6);
        assert!((apply_temp_compensation(20.0, 15.0, 0.01) - 20.0).abs() < 1e-6);
    }

    #[test]
    fn default_autocal_leaves_ratio_unchanged() {
        let mut backend = SitlAirspeedBackend::default();
        assert_eq!(backend.config().autocal, ARSPD_AUTOCAL_DEFAULT);
        assert!(backend.timer_tick(Vector3f::new(20.0, 0.0, 0.0), 1.0, 10));
        backend.update_autocal(25.0);
        assert!((backend.config().ratio - ARSPD_RATIO_DEFAULT).abs() < 1e-6);
        assert!((backend.state().tas_mps - 20.0).abs() < 1e-6);
        assert!((apply_autocal_ratio(2.0, 25.0, 20.0, 0) - 2.0).abs() < 1e-6);
    }

    #[test]
    fn autocal_scales_ratio_toward_gps_groundspeed() {
        let mut backend = SitlAirspeedBackend::with_config(SitlAirspeedConfig {
            autocal: 1,
            ..SitlAirspeedConfig::default()
        });
        assert!(backend.timer_tick(Vector3f::new(20.0, 0.0, 0.0), 1.0, 10));
        backend.update_autocal(25.0);
        // ratio *= 25/20 = 2.5; TAS *= 2.5/2.0 = 25
        assert!((backend.config().ratio - 2.5).abs() < 1e-6);
        assert!((backend.state().tas_mps - 25.0).abs() < 1e-6);
        assert!((backend.state().eas_mps - 25.0).abs() < 1e-6);
        assert!((apply_autocal_ratio(2.0, 30.0, 15.0, 1) - 4.0).abs() < 1e-6);
    }
}
