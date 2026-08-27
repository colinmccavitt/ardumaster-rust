//! Altitude target and baro feed into TECS update_pitch_throttle.

use ap_tecs::params::FlightStage;
use ap_tecs::tecs::{Tecs, TecsInputs};

use crate::target_altitude::TargetAltitude;
use crate::tecs_baro_hookup::{tecs_baro_feed_tick, TecsBaroInputs};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AltitudeTecsFeedInputs {
    pub relative_altitude_m: f32,
    pub baro_climb_rate_mps: f32,
    pub have_baro_sample: bool,
    pub baro_healthy: bool,
    pub home_altitude_m: f32,
    pub next_wp_alt_m: f32,
    pub mission_alt_offset_cm: i32,
    pub rangefinder_correction_m: f32,
    pub target: TargetAltitude,
    pub throttle_suppressed: bool,
    pub throttle_nudge: i16,
    pub target_airspeed_cm: f32,
    pub flight_stage: FlightStage,
    pub pitch_rad: f32,
    pub cos_roll: f32,
    pub use_airspeed: bool,
    pub pitch_trim_deg: f32,
    pub now_ms: ap_hal::time::Millis,
    pub dt: f32,
}

impl Default for AltitudeTecsFeedInputs {
    fn default() -> Self {
        Self {
            relative_altitude_m: 0.0,
            baro_climb_rate_mps: 0.0,
            have_baro_sample: false,
            baro_healthy: false,
            home_altitude_m: 0.0,
            next_wp_alt_m: 0.0,
            mission_alt_offset_cm: 0,
            rangefinder_correction_m: 0.0,
            target: TargetAltitude::FromNextWaypoint,
            throttle_suppressed: false,
            throttle_nudge: 0,
            target_airspeed_cm: 1500.0,
            flight_stage: FlightStage::Normal,
            pitch_rad: 0.0,
            cos_roll: 1.0,
            use_airspeed: true,
            pitch_trim_deg: 0.0,
            now_ms: ap_hal::time::Millis(0),
            dt: 0.02,
        }
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct AltitudeTecsFeedOutput {
    pub tecs_throttle_demand: f32,
    pub tecs_pitch_demand_rad: f32,
    pub tecs_target_alt_cm: f32,
    pub ran: bool,
}

#[must_use]
pub fn relative_target_altitude_cm(inp: &AltitudeTecsFeedInputs) -> f32 {
    if matches!(inp.target, TargetAltitude::HoldCurrentAndResetOffset) {
        return inp.relative_altitude_m * 100.0;
    }
    let home_cm = inp.home_altitude_m * 100.0;
    let target_cm = inp.next_wp_alt_m * 100.0 - home_cm;
    target_cm + inp.mission_alt_offset_cm as f32 + inp.rangefinder_correction_m * 100.0
}

#[must_use]
pub fn altitude_tecs_feed_tick(
    tecs: &mut Tecs,
    inp: &AltitudeTecsFeedInputs,
) -> AltitudeTecsFeedOutput {
    if inp.throttle_suppressed {
        return AltitudeTecsFeedOutput::default();
    }
    let baro = tecs_baro_feed_tick(TecsBaroInputs {
        relative_altitude_m: inp.relative_altitude_m,
        baro_climb_rate_mps: inp.baro_climb_rate_mps,
        have_baro_sample: inp.have_baro_sample,
        baro_healthy: inp.baro_healthy,
    });
    if baro.height_m == 0.0 && !inp.have_baro_sample {
        return AltitudeTecsFeedOutput::default();
    }
    let hgt_dem_cm = relative_target_altitude_cm(inp);
    let eas_dem_cm = inp.target_airspeed_cm;
    let tas = eas_dem_cm * 0.01;
    let tecs_inp = TecsInputs {
        hgt_dem_cm,
        eas_dem_cm,
        flight_stage: inp.flight_stage,
        distance_beyond_land_wp: 0.0,
        pitch_min_climbout_cd: 0.0,
        throttle_nudge: inp.throttle_nudge,
        hgt_afe: baro.hgt_afe_m,
        load_factor: 1.0,
        pitch_trim_deg: inp.pitch_trim_deg,
        height: baro.height_m,
        climb_rate: baro.climb_rate_mps,
        tas_state: tas,
        vel_dot: 0.0,
        vel_dot_lpf: 0.0,
        tas_min: tas * 0.6,
        tas_max: tas * 1.5,
        tas_dem: tas,
        tas_cruise: tas,
        pitch_measured: inp.pitch_rad,
        cos_roll: inp.cos_roll,
        use_airspeed: inp.use_airspeed,
        gliding_requested: false,
        is_flaring: false,
        is_on_approach: false,
        landing_pitch_cd: 0.0,
        land_throttle_slewrate: 0,
        throttle_slewrate: 0,
        path_proportion: 0.0,
        thr_max_ext: 1.0,
        thr_min_ext: 0.0,
        pitch_max_ext: 90.0,
        pitch_min_ext: -90.0,
        now_ms: inp.now_ms,
    };
    tecs.update_pitch_throttle(&tecs_inp, inp.dt);
    AltitudeTecsFeedOutput {
        tecs_throttle_demand: tecs.throttle_demand(),
        tecs_pitch_demand_rad: tecs.pitch_demand(),
        tecs_target_alt_cm: hgt_dem_cm,
        ran: true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn alt_hold_uses_current_relative_altitude() {
        let cm = relative_target_altitude_cm(&AltitudeTecsFeedInputs {
            relative_altitude_m: 42.0,
            target: TargetAltitude::HoldCurrentAndResetOffset,
            ..Default::default()
        });
        assert!((cm - 4200.0).abs() < 1e-6);
    }
}
