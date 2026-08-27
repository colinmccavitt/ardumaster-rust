//! TECS height/climb feed from baro cluster publish.
//!
//! Upstream AP_TECS::update_50hz falls back to baro altitude in its
//! complementary filter when EKF velocity is unavailable, and
//! Plane::tecs_hgt_afe() reads relative_altitude during normal flight.

/// Baro outputs for one TECS feed tick.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct TecsBaroInputs {
    pub relative_altitude_m: f32,
    pub baro_climb_rate_mps: f32,
    pub have_baro_sample: bool,
    pub baro_healthy: bool,
}

/// Height/climb values consumed by TECS 50 Hz and pitch/throttle update.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct TecsBaroFeed {
    pub height_m: f32,
    pub climb_rate_mps: f32,
    pub hgt_afe_m: f32,
}

#[must_use]
pub fn tecs_baro_feed_tick(inp: TecsBaroInputs) -> TecsBaroFeed {
    if !inp.have_baro_sample || !inp.baro_healthy {
        return TecsBaroFeed::default();
    }
    TecsBaroFeed {
        height_m: inp.relative_altitude_m,
        climb_rate_mps: inp.baro_climb_rate_mps,
        hgt_afe_m: inp.relative_altitude_m,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn healthy_baro_feeds_relative_altitude_to_tecs() {
        let feed = tecs_baro_feed_tick(TecsBaroInputs {
            relative_altitude_m: 120.0,
            baro_climb_rate_mps: 2.5,
            have_baro_sample: true,
            baro_healthy: true,
        });
        assert!((feed.height_m - 120.0).abs() < 1e-6);
        assert!((feed.hgt_afe_m - 120.0).abs() < 1e-6);
    }
}
