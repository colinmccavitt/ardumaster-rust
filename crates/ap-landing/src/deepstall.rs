//! Deepstall landing, upstream `AP_Landing_Deepstall`. FW-029.
//!
//! First slice: travel-distance prediction and loiter breakout predicate.

use ap_math::scalar::{constrain_value, is_positive, radians, Real};
use ap_math::vector2::Vector2f;

/// Loiter altitude tolerance for breakout, upstream
/// `DEEPSTALL_LOITER_ALT_TOLERANCE`.
pub const LOITER_ALT_TOLERANCE_M: f32 = 5.0;

/// Breakout heading margin, upstream the 10° check in `verify_breakout`.
pub const BREAKOUT_HEADING_MARGIN_DEG: f32 = 10.0;

/// Deepstall tuning parameters for travel prediction.
#[derive(Debug, Clone, Copy)]
pub struct DeepstallPredictParams {
    pub forward_speed_ms: f32,
    pub slope_a: f32,
    pub slope_b: f32,
    pub down_speed_ms: f32,
    /// Target heading for the stall, degrees.
    pub target_heading_deg: f32,
}

/// Predict how far the aircraft travels during deepstall entry, upstream
/// `AP_Landing_Deepstall::predict_travel_distance`.
#[must_use]
pub fn predict_travel_distance(
    params: &DeepstallPredictParams,
    wind_ne: Vector2f,
    height_m: f32,
) -> f32 {
    let course = radians(params.target_heading_deg);
    let forward_speed_ms = params.forward_speed_ms.max(0.1);

    let wind_length = wind_ne.length().max(0.05);
    let course_vec = Vector2f::new(Real::cos(course), Real::sin(course));

    let offset = course - libm::atan2f(-wind_ne.y, -wind_ne.x);
    let stall_distance = params.slope_a * wind_length * Real::cos(offset) + params.slope_b;

    let cos_theta = constrain_value((wind_ne.dot(course_vec)) / wind_length, -1.0, 1.0);
    let mut theta = Real::acos(cos_theta);
    let reverse = wind_ne.cross(course_vec) > 0.0;
    if reverse {
        theta = -theta;
    }

    let cross_component = Real::sin(theta) * wind_length;
    let mut estimated_crab_angle =
        Real::asin(constrain_value(cross_component / forward_speed_ms, -1.0, 1.0));
    if reverse {
        estimated_crab_angle = -estimated_crab_angle;
    }

    let estimated_forward =
        Real::cos(estimated_crab_angle) * forward_speed_ms + Real::cos(theta) * wind_length;

    if is_positive(params.down_speed_ms) {
        estimated_forward * height_m / params.down_speed_ms + stall_distance
    } else {
        stall_distance
    }
}

/// Whether the aircraft may break out of the loiter to approach, upstream
/// `AP_Landing_Deepstall::verify_breakout`.
#[must_use]
pub fn verify_breakout(heading_error_deg: f32, height_error_m: f32) -> bool {
    heading_error_deg <= BREAKOUT_HEADING_MARGIN_DEG
        && libm::fabsf(height_error_m) < LOITER_ALT_TOLERANCE_M
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn breakout_requires_heading_and_altitude() {
        assert!(verify_breakout(5.0, 2.0));
        assert!(!verify_breakout(15.0, 2.0));
        assert!(!verify_breakout(5.0, 6.0));
    }

    #[test]
    fn zero_down_speed_returns_stall_distance_only() {
        let p = DeepstallPredictParams {
            forward_speed_ms: 10.0,
            slope_a: 1.0,
            slope_b: 5.0,
            down_speed_ms: 0.0,
            target_heading_deg: 0.0,
        };
        // Headwind along course: cos(offset) = -1, so stall_distance = 1*2*(-1)+5 = 3.
        let d = predict_travel_distance(&p, Vector2f::new(2.0, 0.0), 100.0);
        assert!((d - 3.0).abs() < 0.5, "got {d}");
    }
}
