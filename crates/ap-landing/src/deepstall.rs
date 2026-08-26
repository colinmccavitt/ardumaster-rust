//! Deepstall landing, upstream `AP_Landing_Deepstall`. FW-029.
//!
//! Slices: travel prediction, loiter breakout, L1 crosstrack steering.

use ap_math::scalar::{constrain_value, degrees, is_positive, radians, wrap_pi, Real};
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

/// Heading error in degrees for breakout, upstream the angle between groundspeed
/// and the vector to the target.
#[must_use]
pub fn heading_error_deg(groundspeed_ne: Vector2f, to_target_ne: Vector2f) -> f32 {
    degrees(groundspeed_ne.angle_to(to_target_ne))
}

/// Breakout check from NE vectors, upstream `verify_breakout` with AHRS inputs.
#[must_use]
pub fn verify_breakout_vectors(
    groundspeed_ne: Vector2f,
    to_target_ne: Vector2f,
    height_error_m: f32,
) -> bool {
    verify_breakout(heading_error_deg(groundspeed_ne, to_target_ne), height_error_m)
}

/// L1 crosstrack steering tuning, upstream deepstall L1 parameters.
#[derive(Debug, Clone, Copy)]
pub struct DeepstallSteeringParams {
    pub target_heading_deg: f32,
    pub l1_period: f32,
    pub l1_i: f32,
    pub time_constant: f32,
    pub yaw_rate_limit_deg: f32,
}

/// Persistent L1 integrator state for deepstall steering.
#[derive(Debug, Clone, Copy, Default)]
pub struct DeepstallSteeringState {
    pub l1_xtrack_i: f32,
    pub crosstrack_error: f32,
}

/// One steering update's geometry, upstream `update_steering` inputs.
#[derive(Debug, Clone, Copy)]
pub struct DeepstallSteeringInputs {
    /// NE vector from extended approach to arc exit.
    pub approach_to_arc_ne: Vector2f,
    /// NE vector from current position to arc exit.
    pub current_to_arc_ne: Vector2f,
    pub yaw_rad: f32,
    pub yaw_rate_rps: f32,
    pub dt_s: f32,
    pub hold_level: bool,
}

/// Target approach heading from wind or current yaw, upstream
/// `build_approach_path`.
#[must_use]
pub fn deepstall_target_heading_deg(
    wind_ne: Vector2f,
    use_current_heading: bool,
    current_yaw_deg: f32,
) -> f32 {
    if use_current_heading {
        current_yaw_deg
    } else {
        degrees(libm::atan2f(-wind_ne.y, -wind_ne.x))
    }
}

/// Approach extension distance along final bearing, upstream the `MAX(...)` in
/// `build_approach_path`.
#[must_use]
pub fn deepstall_approach_extension_m(
    expected_travel_m: f32,
    approach_extension_m: f32,
    loiter_radius_m: f32,
) -> f32 {
    let base = expected_travel_m + approach_extension_m;
    let min_ext = libm::fabsf(loiter_radius_m) * 0.5;
    base.max(min_ext)
}

/// Tangent heading for the turnaround arc, upstream `arc_heading_deg`.
#[must_use]
pub fn deepstall_arc_heading_deg(target_heading_deg: f32, loiter_ccw: bool) -> f32 {
    if loiter_ccw {
        target_heading_deg - 90.0
    } else {
        target_heading_deg + 90.0
    }
}

/// Crosstrack error for the deepstall arc, upstream the `% ab` in `update_steering`.
#[must_use]
pub fn deepstall_crosstrack_error(
    approach_to_arc_ne: Vector2f,
    current_to_arc_ne: Vector2f,
) -> f32 {
    let ab = approach_to_arc_ne.normalized().unwrap_or_else(Vector2f::zero);
    current_to_arc_ne.cross(ab)
}

/// Yaw-rate PID error for deepstall steering, upstream `update_steering` before
/// `ds_PID.get_pid`.
#[must_use]
pub fn deepstall_steering_pid_error(
    params: &DeepstallSteeringParams,
    state: &mut DeepstallSteeringState,
    inp: &DeepstallSteeringInputs,
) -> f32 {
    let mut desired_change = 0.0_f32;

    if !inp.hold_level {
        state.crosstrack_error =
            deepstall_crosstrack_error(inp.approach_to_arc_ne, inp.current_to_arc_ne);
        let l1_period = params.l1_period.max(0.1);
        let sine_nu1 = constrain_value(state.crosstrack_error / l1_period, -0.7071, 0.7107);
        let mut nu1 = Real::asin(sine_nu1);

        if params.l1_i > 0.0 && inp.dt_s > 0.0 {
            state.l1_xtrack_i += nu1 * params.l1_i / inp.dt_s;
            state.l1_xtrack_i = constrain_value(state.l1_xtrack_i, -0.5, 0.5);
            nu1 += state.l1_xtrack_i;
        }

        desired_change = wrap_pi(
            radians(params.target_heading_deg) + nu1 - inp.yaw_rad,
        ) / params.time_constant;
    }

    let yaw_rate_limit_rps = radians(params.yaw_rate_limit_deg);
    let limited = constrain_value(desired_change, -yaw_rate_limit_rps, yaw_rate_limit_rps);
    wrap_pi(limited - inp.yaw_rate_rps)
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

    #[test]
    fn steering_on_track_has_near_zero_crosstrack() {
        let err = deepstall_crosstrack_error(
            Vector2f::new(100.0, 0.0),
            Vector2f::new(50.0, 0.0),
        );
        assert!(err.abs() < 1e-3);
    }

    #[test]
    fn hold_level_seeks_zero_yaw_rate() {
        let params = DeepstallSteeringParams {
            target_heading_deg: 0.0,
            l1_period: 20.0,
            l1_i: 0.1,
            time_constant: 2.0,
            yaw_rate_limit_deg: 30.0,
        };
        let mut state = DeepstallSteeringState::default();
        let err = deepstall_steering_pid_error(
            &params,
            &mut state,
            &DeepstallSteeringInputs {
                approach_to_arc_ne: Vector2f::new(100.0, 0.0),
                current_to_arc_ne: Vector2f::new(50.0, 10.0),
                yaw_rad: 0.0,
                yaw_rate_rps: 0.1,
                dt_s: 0.05,
                hold_level: true,
            },
        );
        assert!((err - wrap_pi(-0.1)).abs() < 1e-4);
    }

    #[test]
    fn target_heading_honours_current_yaw_override() {
        let h = deepstall_target_heading_deg(Vector2f::new(10.0, 10.0), true, 42.0);
        assert!((h - 42.0).abs() < 1e-6);
    }

    #[test]
    fn approach_extension_respects_half_loiter_radius() {
        let ext = deepstall_approach_extension_m(10.0, 5.0, 100.0);
        assert!((ext - 50.0).abs() < 1e-3);
    }

    #[test]
    fn arc_heading_offsets_by_quarter_turn() {
        assert!((deepstall_arc_heading_deg(0.0, false) - 90.0).abs() < 1e-3);
        assert!((deepstall_arc_heading_deg(0.0, true) + 90.0).abs() < 1e-3);
    }
}
