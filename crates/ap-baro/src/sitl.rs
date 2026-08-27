//! SITL barometer backend, upstream `AP_Baro_SITL`. FW-013.
//!
//! Pure transforms from simulator altitude and configuration to the pressure
//! and temperature the frontend receives. Random noise uses an injected sample
//! so tests stay deterministic.

use ap_filter::derivative::DerivativeFilter;
use ap_math::scalar::{is_positive, Real};
use ap_math::vector3::Vector3f;

use crate::{air_density_for_alt_amsl, pressure_temperature_for_alt_amsl, SSL_AIR_DENSITY};

/// Board warm-up and temperature-dependent pressure offset, upstream
/// `AP_Baro_SITL::temperature_adjustment`.
#[must_use]
pub fn temperature_adjustment(
    p: f32,
    t_c: f32,
    tsec: f32,
    temp_start_c: f32,
    temp_board_offset_c: f32,
    temp_tconst: f32,
    temp_baro_factor: f32,
) -> (f32, f32) {
    let t_sensor = t_c + temp_board_offset_c;
    let t = if tsec < 23.0 * temp_tconst {
        let t0 = temp_start_c;
        t_sensor - (t_sensor - t0) * Real::exp(-tsec / temp_tconst)
    } else {
        t_sensor
    };

    let mut p_out = p;
    const TZERO: f32 = 30.0;
    if is_positive(temp_baro_factor) {
        let delta = (t - TZERO).max(0.0);
        p_out -= Real::powf(delta, temp_baro_factor);
    }
    (p_out, t)
}

/// Static-pressure position error from body-frame airspeed, upstream
/// `AP_Baro_SITL::wind_pressure_correction`.
#[must_use]
pub fn wind_pressure_correction(
    airspeed_bf: Vector3f,
    wcof_xp: f32,
    wcof_xn: f32,
    wcof_yp: f32,
    wcof_yn: f32,
    wcof_zp: f32,
    wcof_zn: f32,
    air_density_ratio: f32,
) -> f32 {
    let sqx = airspeed_bf.x * airspeed_bf.x;
    let sqy = airspeed_bf.y * airspeed_bf.y;
    let sqz = airspeed_bf.z * airspeed_bf.z;

    let error = if is_positive(airspeed_bf.x) {
        wcof_xp * sqx
    } else {
        wcof_xn * sqx
    } + if is_positive(airspeed_bf.y) {
        wcof_yp * sqy
    } else {
        wcof_yn * sqy
    } + if is_positive(airspeed_bf.z) {
        wcof_zp * sqz
    } else {
        wcof_zn * sqz
    };

    error * 0.5 * SSL_AIR_DENSITY * air_density_ratio
}

/// Accumulate altitude drift from a constant rate, upstream the drift term in
/// `_timer`.
#[must_use]
pub fn accumulate_altitude_drift(total_drift_m: f32, drift_rate_mps: f32, dt_ms: u32) -> f32 {
    #[allow(clippy::cast_precision_loss, reason = "milliseconds fit in f32 for SITL dt")]
    let dt = dt_ms as f32 * 0.001;
    total_drift_m + drift_rate_mps * dt
}

/// One slot in the SITL baro delay ring buffer.
#[derive(Debug, Clone, Copy, Default)]
pub struct DelayedSample {
    pub altitude_m: f32,
    pub time_ms: u32,
}

/// Store a sample and pick the buffer entry closest to `delayed_time_ms`.
///
/// Upstream runs at 100 Hz with a 10 ms store cadence and returns the delayed
/// sample only when the best match is within 200 ms.
#[must_use]
pub fn delay_buffer_sample(
    buffer: &mut [DelayedSample],
    store_index: &mut u8,
    now_ms: u32,
    altitude_m: f32,
    delay_ms: u32,
    last_store_ms: &mut u32,
) -> Option<f32> {
    if now_ms.wrapping_sub(*last_store_ms) >= 10 {
        *last_store_ms = now_ms;
        let idx = (*store_index as usize) % buffer.len();
        buffer[idx] = DelayedSample {
            altitude_m,
            time_ms: now_ms,
        };
        *store_index = store_index.wrapping_add(1);
    }

    let delayed_time = now_ms.wrapping_sub(delay_ms);
    let mut best_index = 0_u8;
    let mut best_delta = 200_u32;

    for (i, entry) in buffer.iter().enumerate() {
        let delta = delayed_time.abs_diff(entry.time_ms);
        if delta < best_delta {
            best_index = i as u8;
            best_delta = delta;
        }
    }

    if best_delta < 200 {
        Some(buffer[best_index as usize].altitude_m)
    } else {
        None
    }
}

/// Convert simulator altitude to pressure and temperature before corrections.
#[must_use]
pub fn pressure_temperature_from_altitude(altitude_m: f32) -> (f32, f32) {
    let (p, t_k) = pressure_temperature_for_alt_amsl(altitude_m);
    (p, t_k - 273.15)
}

/// Full fixed-wing sample path without noise, upstream `_timer` body.
#[must_use]
pub fn sitl_baro_sample(
    sim_altitude_m: f32,
    tsec: f32,
    temp_start_c: f32,
    temp_board_offset_c: f32,
    temp_tconst: f32,
    temp_baro_factor: f32,
    airspeed_bf: Vector3f,
    wcof_xp: f32,
    wcof_xn: f32,
    wcof_yp: f32,
    wcof_yn: f32,
    wcof_zp: f32,
    wcof_zn: f32,
    air_density_ratio: f32,
    glitch_m: f32,
) -> (f32, f32) {
    let alt = sim_altitude_m + glitch_m;
    let (p, t_c) = pressure_temperature_from_altitude(alt);
    let (p, t_c) = temperature_adjustment(
        p,
        t_c,
        tsec,
        temp_start_c,
        temp_board_offset_c,
        temp_tconst,
        temp_baro_factor,
    );
    let p = p + wind_pressure_correction(
        airspeed_bf,
        wcof_xp,
        wcof_xn,
        wcof_yp,
        wcof_yn,
        wcof_zp,
        wcof_zn,
        air_density_ratio,
    );
    (p, t_c)
}

/// Minimum interval between baro timer ticks, upstream `_timer` at 100 Hz.
pub const SITL_BARO_UPDATE_MS: u32 = 10;

/// Delay ring capacity, upstream `_buffer_length`.
pub const SITL_BARO_BUFFER_LEN: usize = 50;

/// Per-instance SITL baro parameters, upstream `SITL::BaroParams`.
#[derive(Debug, Clone, Copy)]
pub struct SitlBaroConfig {
    pub disabled: bool,
    pub drift_rate_mps: f32,
    pub noise_scale: f32,
    pub glitch_m: f32,
    pub delay_ms: u32,
    pub freeze: bool,
    pub wcof_xp: f32,
    pub wcof_xn: f32,
    pub wcof_yp: f32,
    pub wcof_yn: f32,
    pub wcof_zp: f32,
    pub wcof_zn: f32,
    pub temp_start_c: f32,
    pub temp_board_offset_c: f32,
    pub temp_tconst: f32,
    pub temp_baro_factor: f32,
}

impl Default for SitlBaroConfig {
    fn default() -> Self {
        Self {
            disabled: false,
            drift_rate_mps: 0.0,
            noise_scale: 0.0,
            glitch_m: 0.0,
            delay_ms: 0,
            freeze: false,
            wcof_xp: 0.0,
            wcof_xn: 0.0,
            wcof_yp: 0.0,
            wcof_yn: 0.0,
            wcof_zp: 0.0,
            wcof_zn: 0.0,
            temp_start_c: 20.0,
            temp_board_offset_c: 0.0,
            temp_tconst: 30.0,
            temp_baro_factor: 0.0,
        }
    }
}

/// Pressure and temperature from one backend timer tick, upstream `_recent_*`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct BaroSampleState {
    pub pressure_pa: f32,
    pub temp_c: f32,
    pub altitude_m: f32,
    pub have_sample: bool,
    pub last_sample_time_ms: u32,
}

/// SITL barometer backend producer, upstream `AP_Baro_SITL::_timer` / `update`.
#[derive(Debug, Clone)]
pub struct SitlBaroBackend {
    config: SitlBaroConfig,
    last_sample_time_ms: u32,
    last_drift_delta_ms: u32,
    total_alt_drift_m: f32,
    last_altitude_m: f32,
    store_index: u8,
    last_store_ms: u32,
    buffer: [DelayedSample; SITL_BARO_BUFFER_LEN],
    pending: BaroSampleState,
    has_pending: bool,
}

impl Default for SitlBaroBackend {
    fn default() -> Self {
        Self {
            config: SitlBaroConfig::default(),
            last_sample_time_ms: 0,
            last_drift_delta_ms: 0,
            total_alt_drift_m: 0.0,
            last_altitude_m: 0.0,
            store_index: 0,
            last_store_ms: 0,
            buffer: [DelayedSample::default(); SITL_BARO_BUFFER_LEN],
            pending: BaroSampleState::default(),
            has_pending: false,
        }
    }
}

impl SitlBaroBackend {
    #[must_use]
    pub fn with_config(config: SitlBaroConfig) -> Self {
        Self {
            config,
            ..Self::default()
        }
    }

    #[must_use]
    pub const fn config(&self) -> &SitlBaroConfig {
        &self.config
    }

    #[must_use]
    pub const fn state(&self) -> &BaroSampleState {
        &self.pending
    }

    /// Upstream `healthy()`: seen at least one tick and baro not disabled.
    #[must_use]
    pub fn healthy(&self) -> bool {
        self.last_sample_time_ms != 0 && !self.config.disabled
    }

    /// Run the 100 Hz timer path. `noise_sample` stands in for `rand_float()`.
    pub fn timer_tick(
        &mut self,
        sim_altitude_m: f32,
        airspeed_bf: Vector3f,
        now_ms: u32,
        noise_sample: f32,
    ) -> bool {
        if now_ms.wrapping_sub(self.last_sample_time_ms) < SITL_BARO_UPDATE_MS {
            return false;
        }
        self.last_sample_time_ms = now_ms;

        if self.config.disabled {
            return false;
        }

        let drift_dt = now_ms.wrapping_sub(self.last_drift_delta_ms);
        self.last_drift_delta_ms = now_ms;
        self.total_alt_drift_m =
            accumulate_altitude_drift(self.total_alt_drift_m, self.config.drift_rate_mps, drift_dt);

        let mut alt = sim_altitude_m + self.total_alt_drift_m;
        alt += self.config.noise_scale * noise_sample;
        alt += self.config.glitch_m;

        if now_ms.wrapping_sub(self.last_store_ms) >= 10 {
            self.last_store_ms = now_ms;
            if self.config.freeze {
                alt = self.last_altitude_m;
            } else {
                self.last_altitude_m = alt;
            }
            let idx = (self.store_index as usize) % SITL_BARO_BUFFER_LEN;
            self.buffer[idx] = DelayedSample {
                altitude_m: alt,
                time_ms: now_ms,
            };
            self.store_index = self.store_index.wrapping_add(1);
        }

        let delayed_time = now_ms.wrapping_sub(self.config.delay_ms);
        let mut best_index = 0_u8;
        let mut best_delta = 200_u32;
        for (i, entry) in self.buffer.iter().enumerate() {
            let delta = delayed_time.abs_diff(entry.time_ms);
            if delta < best_delta {
                best_index = i as u8;
                best_delta = delta;
            }
        }
        if best_delta < 200 {
            alt = self.buffer[best_index as usize].altitude_m;
        }

        let air_density_ratio = air_density_for_alt_amsl(sim_altitude_m) / SSL_AIR_DENSITY;
        #[allow(clippy::cast_precision_loss, reason = "milliseconds fit in f32 for SITL dt")]
        let tsec = now_ms as f32 * 0.001;

        let (p, t_c) = sitl_baro_sample(
            alt,
            tsec,
            self.config.temp_start_c,
            self.config.temp_board_offset_c,
            self.config.temp_tconst,
            self.config.temp_baro_factor,
            airspeed_bf,
            self.config.wcof_xp,
            self.config.wcof_xn,
            self.config.wcof_yp,
            self.config.wcof_yn,
            self.config.wcof_zp,
            self.config.wcof_zn,
            air_density_ratio,
            0.0,
        );

        self.pending = BaroSampleState {
            pressure_pa: p,
            temp_c: t_c,
            altitude_m: alt,
            have_sample: true,
            last_sample_time_ms: now_ms,
        };
        self.has_pending = true;
        true
    }

    /// Copy the pending sample to the frontend, upstream `update()`.
    #[must_use]
    pub fn update(&mut self) -> Option<BaroSampleState> {
        if !self.has_pending {
            return None;
        }
        self.has_pending = false;
        Some(self.pending)
    }
}


/// Climb-rate estimate from primary altitude, upstream `AP_Baro::_climb_rate_filter`.
#[derive(Debug, Clone, Copy, Default)]
pub struct BaroClimbRate {
    filter: DerivativeFilter<7>,
}

impl BaroClimbRate {
    /// Feed primary altitude when healthy, upstream `read()` climb filter update.
    pub fn update_primary(&mut self, altitude_m: f32, last_update_ms: u32, healthy: bool) {
        if healthy {
            self.filter.update(altitude_m, last_update_ms);
        }
    }

    /// Current climb rate in m/s, upstream `AP_Baro::get_climb_rate()`.
    ///
    /// Returns zero when the primary baro is unhealthy, matching upstream.
    #[must_use]
    pub fn climb_rate_mps(&mut self, healthy: bool) -> f32 {
        if !healthy {
            return 0.0;
        }
        self.filter.slope() * 1.0e3
    }
}

/// Dual-instance capacity, upstream `BARO_MAX_INSTANCES`.
pub const SITL_BARO_MAX_INSTANCES: usize = 2;

/// Per-instance health aggregation, upstream `AP_Baro` frontend flags.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BaroHealthFlags {
    /// Instance has published at least one tick and is not disabled.
    pub healthy: [bool; SITL_BARO_MAX_INSTANCES],
    /// Instance currently holds a valid sample.
    pub have_sample: [bool; SITL_BARO_MAX_INSTANCES],
    pub instance_count: u8,
    /// Selected primary baro index, upstream `_primary`.
    pub primary: u8,
}

impl BaroHealthFlags {
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

/// Multi-instance SITL baro cluster stub, upstream dual `AP_Baro_SITL` drivers.
#[derive(Debug, Clone)]
pub struct SitlBaroCluster {
    backends: [SitlBaroBackend; SITL_BARO_MAX_INSTANCES],
    instance_count: u8,
    primary: u8,
}

impl Default for SitlBaroCluster {
    fn default() -> Self {
        let mut cluster = Self {
            backends: [SitlBaroBackend::default(), SitlBaroBackend::default()],
            instance_count: 0,
            primary: 0,
        };
        let _ = cluster.register(SitlBaroBackend::default());
        cluster
    }
}

impl SitlBaroCluster {
    #[must_use]
    pub const fn instance_count(&self) -> u8 {
        self.instance_count
    }

    #[must_use]
    pub const fn primary(&self) -> u8 {
        self.primary
    }

    pub fn register(&mut self, backend: SitlBaroBackend) -> Result<u8, ()> {
        if self.instance_count as usize >= SITL_BARO_MAX_INSTANCES {
            return Err(());
        }
        let idx = self.instance_count as usize;
        self.backends[idx] = backend;
        self.instance_count += 1;
        Ok(self.instance_count - 1)
    }

    pub fn set_primary(&mut self, index: u8) {
        if index < self.instance_count {
            self.primary = index;
        }
    }

    /// Re-select primary to the first healthy instance with a sample, upstream
    /// `AP_Baro` frontend failover after backend updates.
    pub fn select_primary_healthy(&mut self) {
        for i in 0..self.instance_count as usize {
            if self.backends[i].healthy() && self.backends[i].state().have_sample {
                self.primary = i as u8;
                return;
            }
        }
    }

    #[must_use]
    pub fn backend(&self, index: u8) -> Option<&SitlBaroBackend> {
        (index < self.instance_count).then(|| &self.backends[index as usize])
    }

    #[must_use]
    pub fn backend_mut(&mut self, index: u8) -> Option<&mut SitlBaroBackend> {
        (index < self.instance_count).then(|| &mut self.backends[index as usize])
    }

    pub fn timer_tick_all(
        &mut self,
        sim_altitude_m: f32,
        airspeed_bf: Vector3f,
        now_ms: u32,
        noise_sample: f32,
    ) {
        for i in 0..self.instance_count {
            let idx = i as usize;
            let noise = if i == 0 { noise_sample } else { 0.0 };
            let _ = self.backends[idx].timer_tick(
                sim_altitude_m,
                airspeed_bf,
                now_ms,
                noise,
            );
        }
    }

    #[must_use]
    pub fn health_flags(&self) -> BaroHealthFlags {
        let mut flags = BaroHealthFlags {
            instance_count: self.instance_count,
            primary: self.primary,
            ..BaroHealthFlags::default()
        };
        for i in 0..self.instance_count as usize {
            flags.healthy[i] = self.backends[i].healthy();
            flags.have_sample[i] = self.backends[i].state().have_sample;
        }
        flags
    }

    /// Cluster with disabled primary and healthy secondary for failover tests.
    #[must_use]
    pub fn cluster_with_disabled_primary() -> Self {
        Self {
            backends: [
                SitlBaroBackend::with_config(SitlBaroConfig {
                    disabled: true,
                    ..SitlBaroConfig::default()
                }),
                SitlBaroBackend::default(),
            ],
            instance_count: 2,
            primary: 0,
        }
    }

    /// Primary instance sample after `update()`, upstream `_primary_baro`.
    #[must_use]
    pub fn primary_sample(&mut self) -> Option<BaroSampleState> {
        let primary = self.primary as usize;
        self.backends[primary]
            .update()
            .or_else(|| {
                let sample = *self.backends[primary].state();
                sample.have_sample.then_some(sample)
            })
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn temperature_warmup_starts_at_temp_start_and_rises() {
        let (_, t_early) = temperature_adjustment(
            101_325.0,
            15.0,
            0.0,
            20.0,
            0.0,
            10.0,
            0.0,
        );
        let (_, t_late) = temperature_adjustment(
            101_325.0,
            15.0,
            500.0,
            20.0,
            0.0,
            10.0,
            0.0,
        );
        assert!((t_early - 20.0).abs() < 0.01, "starts at temp_start, got {t_early}");
        assert!((t_late - 15.0).abs() < 0.01, "settles at sensor temp, got {t_late}");
    }

    #[test]
    fn wind_correction_is_zero_with_no_airspeed() {
        assert_eq!(
            wind_pressure_correction(Vector3f::zero(), 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
            0.0
        );
    }

    #[test]
    fn drift_accumulates_linearly() {
        assert!((accumulate_altitude_drift(0.0, 0.1, 1000) - 0.1).abs() < 1e-6);
    }

    #[test]
    fn the_delay_buffer_returns_a_nearby_sample() {
        let mut buffer = [DelayedSample::default(); 4];
        let mut idx = 0_u8;
        let mut last = 0_u32;
        delay_buffer_sample(&mut buffer, &mut idx, 0, 100.0, 50, &mut last);
        delay_buffer_sample(&mut buffer, &mut idx, 10, 100.0, 50, &mut last);
        let got = delay_buffer_sample(&mut buffer, &mut idx, 60, 200.0, 50, &mut last);
        assert_eq!(got, Some(100.0));
    }

    #[test]
    fn sea_level_pressure_is_about_standard() {
        let (p, t) = sitl_baro_sample(
            0.0,
            1000.0,
            20.0,
            0.0,
            30.0,
            0.0,
            Vector3f::zero(),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        );
        assert!((p - 101_325.0).abs() < 500.0, "pressure {p}");
        assert!(t > 10.0 && t < 30.0, "temperature {t}");
    }
    #[test]
    fn producer_rate_limit_gates_until_10ms() {
        let mut baro = SitlBaroBackend::default();
        assert!(!baro.timer_tick(100.0, Vector3f::zero(), 0, 0.0));
        assert!(!baro.state().have_sample);
        assert!(baro.timer_tick(100.0, Vector3f::zero(), 10, 0.0));
        assert!(baro.state().have_sample);
    }

    #[test]
    fn producer_emits_sea_level_pressure() {
        let mut baro = SitlBaroBackend::default();
        assert!(baro.timer_tick(0.0, Vector3f::zero(), 10, 0.0));
        let sample = baro.update().expect("pending sample");
        assert!((sample.pressure_pa - 101_325.0).abs() < 500.0);
        assert!(sample.temp_c > 10.0 && sample.temp_c < 30.0);
    }

    #[test]
    fn producer_drift_shifts_reported_altitude() {
        let mut baro = SitlBaroBackend::default();
        baro.config = SitlBaroConfig {
            drift_rate_mps: 0.1,
            ..SitlBaroConfig::default()
        };
        assert!(baro.timer_tick(100.0, Vector3f::zero(), 10, 0.0));
        assert!(baro.timer_tick(100.0, Vector3f::zero(), 1010, 0.0));
        assert!((baro.state().altitude_m - 100.1).abs() < 0.05);
    }

    #[test]
    fn producer_unhealthy_when_disabled() {
        let mut baro = SitlBaroBackend::default();
        baro.config = SitlBaroConfig {
            disabled: true,
            ..SitlBaroConfig::default()
        };
        assert!(!baro.timer_tick(0.0, Vector3f::zero(), 10, 0.0));
        assert!(!baro.healthy());
    }

    #[test]
    fn update_consumes_pending_sample_once() {
        let mut baro = SitlBaroBackend::default();
        assert!(baro.timer_tick(0.0, Vector3f::zero(), 10, 0.0));
        assert!(baro.update().is_some());
        assert!(baro.update().is_none());
    }
    #[test]
    fn cluster_registers_two_instances() {
        let mut cluster = SitlBaroCluster::default();
        assert_eq!(cluster.instance_count(), 1);
        assert!(cluster.register(SitlBaroBackend::default()).is_ok());
        assert_eq!(cluster.instance_count(), 2);
        assert!(cluster.register(SitlBaroBackend::default()).is_err());
    }

    #[test]
    fn cluster_health_flags_track_primary_and_secondary() {
        let mut cluster = SitlBaroCluster::default();
        let secondary = SitlBaroBackend::with_config(SitlBaroConfig {
            disabled: true,
            ..SitlBaroConfig::default()
        });
        let _ = cluster.register(secondary);
        cluster.timer_tick_all(100.0, Vector3f::zero(), 10, 0.0);
        let flags = cluster.health_flags();
        assert_eq!(flags.instance_count, 2);
        assert!(flags.healthy[0]);
        assert!(flags.have_sample[0]);
        assert!(!flags.healthy[1]);
    }

    #[test]
    fn cluster_primary_sample_comes_from_selected_index() {
        let mut cluster = SitlBaroCluster::default();
        let _ = cluster.register(SitlBaroBackend::default());
        cluster.set_primary(1);
        cluster.timer_tick_all(250.0, Vector3f::zero(), 10, 0.0);
        let sample = cluster.primary_sample().expect("primary sample");
        assert!((sample.altitude_m - 250.0).abs() < 1.0);
    }

    #[test]
    fn cluster_failover_selects_secondary_when_primary_disabled() {
        let mut cluster = SitlBaroCluster::cluster_with_disabled_primary();
        cluster.timer_tick_all(180.0, Vector3f::zero(), 10, 0.0);
        cluster.select_primary_healthy();
        assert_eq!(cluster.primary(), 1);
        let flags = cluster.health_flags();
        assert!(!flags.healthy[0]);
        assert!(flags.healthy[1]);
        assert!(flags.primary_healthy());
    }

    #[test]
    fn primary_healthy_requires_have_sample() {
        let flags = BaroHealthFlags {
            instance_count: 1,
            healthy: [true, false],
            have_sample: [false, false],
            primary: 0,
        };
        assert!(!flags.primary_healthy());
    }

    #[test]
    fn climb_rate_tracks_constant_ascent() {
        let mut climb = BaroClimbRate::default();
        let rate_mps = 2.0_f32;
        let dt_ms = 10_u32;
        let step_m = rate_mps * dt_ms as f32 * 0.001;
        let mut alt = 100.0_f32;
        let mut t = 0_u32;
        for _ in 0..20 {
            t += dt_ms;
            alt += step_m;
            climb.update_primary(alt, t, true);
        }
        let got = climb.climb_rate_mps(true);
        assert!(
            (got - rate_mps).abs() < 0.15,
            "expected ~{rate_mps} m/s, got {got}"
        );
    }


    #[test]
    fn climb_rate_zero_when_primary_unhealthy() {
        let mut climb = BaroClimbRate::default();
        let rate_mps = 2.0_f32;
        let dt_ms = 10_u32;
        let step_m = rate_mps * dt_ms as f32 * 0.001;
        let mut alt = 100.0_f32;
        let mut t = 0_u32;
        for _ in 0..20 {
            t += dt_ms;
            alt += step_m;
            climb.update_primary(alt, t, true);
        }
        assert!(
            (climb.climb_rate_mps(true) - rate_mps).abs() < 0.15,
            "expected ~{rate_mps} m/s while healthy"
        );
        assert_eq!(
            climb.climb_rate_mps(false),
            0.0,
            "unhealthy primary must publish zero climb rate"
        );
    }

}
