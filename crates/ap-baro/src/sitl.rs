//! SITL barometer backend, upstream `AP_Baro_SITL`. FW-013.
//!
//! Pure transforms from simulator altitude and configuration to the pressure
//! and temperature the frontend receives. Random noise uses an injected sample
//! so tests stay deterministic.

use ap_math::scalar::{is_positive, Real};
use ap_math::vector3::Vector3f;

use crate::{pressure_temperature_for_alt_amsl, SSL_AIR_DENSITY};

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
}
