//! Lean-angle leftover `AC_Avoid::adjust_roll_pitch_rad`. Tracked as **COP-026**.
//!
//! This is the last Copter leftover in `libraries/AC_Avoidance`. Rover-only
//! `adjust_speed` stays out of scope.

use ap_avoidance::{
    Avoid, LeanAngleContext, LeanProximityObject, ANGLE_MAX_DEG_DEFAULT, ANGLE_MAX_PERCENT,
    DISABLED, NONGPS_DIST_MAX_DEFAULT, USE_PROXIMITY_SENSOR,
};
use ap_math::scalar::{is_equal, radians};
use ap_math::vector2::Vector2f;

fn almost(a: f32, b: f32) {
    assert!(is_equal(a, b), "{a} != {b}");
}

fn ahead(dist_m: f32) -> LeanAngleContext {
    LeanAngleContext::from_objects(&[LeanProximityObject {
        angle_deg: 0.0,
        dist_m,
    }])
}

fn right(dist_m: f32) -> LeanAngleContext {
    LeanAngleContext::from_objects(&[LeanProximityObject {
        angle_deg: 90.0,
        dist_m,
    }])
}

fn left(dist_m: f32) -> LeanAngleContext {
    LeanAngleContext::from_objects(&[LeanProximityObject {
        angle_deg: -90.0,
        dist_m,
    }])
}

fn behind(dist_m: f32) -> LeanAngleContext {
    LeanAngleContext::from_objects(&[LeanProximityObject {
        angle_deg: 180.0,
        dist_m,
    }])
}

#[test]
fn defaults_match_upstream_ang_max_and_dist_max() {
    let avoid = Avoid::new();
    almost(avoid.angle_max_deg(), ANGLE_MAX_DEG_DEFAULT);
    almost(avoid.dist_max_m(), NONGPS_DIST_MAX_DEFAULT);
    almost(ANGLE_MAX_PERCENT, 0.75);
}

#[test]
fn distance_to_lean_is_inverted_linear() {
    let avoid = Avoid::new();
    almost(avoid.distance_m_to_lean_norm(0.0), 1.0);
    almost(avoid.distance_m_to_lean_norm(2.5), 0.5);
    almost(avoid.distance_m_to_lean_norm(NONGPS_DIST_MAX_DEFAULT), 0.0);
    almost(avoid.distance_m_to_lean_norm(6.0), 0.0);
    almost(avoid.distance_m_to_lean_norm(-0.1), 0.0);
}

#[test]
fn distance_to_lean_is_zero_when_dist_max_is_non_positive() {
    let mut avoid = Avoid::new();
    avoid.set_dist_max_m(0.0);
    almost(avoid.distance_m_to_lean_norm(1.0), 0.0);
    avoid.set_dist_max_m(-1.0);
    almost(avoid.distance_m_to_lean_norm(1.0), 0.0);
}

#[test]
fn skipped_when_proximity_bit_is_off() {
    let mut avoid = Avoid::new();
    avoid.set_enabled(DISABLED);
    let leftover = avoid.adjust_roll_pitch_rad(0.1, -0.2, 0.5, ahead(1.0));
    assert!(leftover.skipped);
    almost(leftover.roll_rad, 0.1);
    almost(leftover.pitch_rad, -0.2);
}

#[test]
fn skipped_when_runtime_proximity_is_disabled() {
    let mut avoid = Avoid::new();
    avoid.proximity_avoidance_enable(false);
    assert!(avoid.enabled_bits() & USE_PROXIMITY_SENSOR > 0);
    let leftover = avoid.adjust_roll_pitch_rad(0.1, 0.0, 0.5, ahead(1.0));
    assert!(leftover.skipped);
    almost(leftover.roll_rad, 0.1);
    almost(leftover.pitch_rad, 0.0);
}

#[test]
fn skipped_when_ang_max_is_non_positive() {
    let mut avoid = Avoid::new();
    avoid.set_angle_max_deg(0.0);
    let leftover = avoid.adjust_roll_pitch_rad(0.2, 0.1, 0.5, ahead(1.0));
    assert!(leftover.skipped);
    almost(leftover.roll_rad, 0.2);
    almost(leftover.pitch_rad, 0.1);
}

#[test]
fn skipped_when_vehicle_angle_max_is_non_positive() {
    let avoid = Avoid::new();
    let leftover = avoid.adjust_roll_pitch_rad(0.2, 0.1, 0.0, ahead(1.0));
    assert!(leftover.skipped);
    almost(leftover.roll_rad, 0.2);
    almost(leftover.pitch_rad, 0.1);
}

#[test]
fn object_ahead_adds_positive_pitch() {
    let avoid = Avoid::new();
    let leftover = avoid.adjust_roll_pitch_rad(0.0, 0.0, 0.8, ahead(0.0));
    assert!(!leftover.skipped);
    almost(leftover.roll_rad, 0.0);
    assert!(leftover.pitch_rad > 0.0, "pitch {}", leftover.pitch_rad);
}

#[test]
fn object_behind_adds_negative_pitch() {
    let avoid = Avoid::new();
    let leftover = avoid.adjust_roll_pitch_rad(0.0, 0.0, 0.8, behind(0.0));
    assert!(!leftover.skipped);
    almost(leftover.roll_rad, 0.0);
    assert!(leftover.pitch_rad < 0.0, "pitch {}", leftover.pitch_rad);
}

#[test]
fn object_to_the_right_adds_negative_roll() {
    let avoid = Avoid::new();
    let leftover = avoid.adjust_roll_pitch_rad(0.0, 0.0, 0.8, right(0.0));
    assert!(!leftover.skipped);
    assert!(leftover.roll_rad < 0.0, "roll {}", leftover.roll_rad);
    almost(leftover.pitch_rad, 0.0);
}

#[test]
fn object_to_the_left_adds_positive_roll() {
    let avoid = Avoid::new();
    let leftover = avoid.adjust_roll_pitch_rad(0.0, 0.0, 0.8, left(0.0));
    assert!(!leftover.skipped);
    assert!(leftover.roll_rad > 0.0, "roll {}", leftover.roll_rad);
    almost(leftover.pitch_rad, 0.0);
}

#[test]
fn object_beyond_dist_max_is_ignored() {
    let avoid = Avoid::new();
    let leftover = avoid.adjust_roll_pitch_rad(0.05, -0.04, 0.8, ahead(5.0));
    assert!(!leftover.skipped);
    almost(leftover.roll_rad, 0.05);
    almost(leftover.pitch_rad, -0.04);
    assert!(!leftover.avoidance_limited);
    assert!(!leftover.total_limited);
}

#[test]
fn closer_object_leans_harder_than_farther() {
    let mut avoid = Avoid::new();
    // Default ANG_MAX is 10 deg; 45 deg * lean_norm stays under that only
    // when lean_norm < 10/45. Use far ranges so the linear response shows.
    avoid.set_angle_max_deg(45.0);
    let close = avoid.adjust_roll_pitch_rad(0.0, 0.0, 0.8, ahead(4.0));
    let far = avoid.adjust_roll_pitch_rad(0.0, 0.0, 0.8, ahead(4.6));
    assert!(
        close.pitch_rad > far.pitch_rad,
        "close {} far {}",
        close.pitch_rad,
        far.pitch_rad
    );
    assert!(!close.avoidance_limited);
    assert!(!far.avoidance_limited);
    assert!(far.pitch_rad > 0.0);
}

#[test]
fn same_side_objects_take_the_larger_norm() {
    let avoid = Avoid::new();
    let ctx = LeanAngleContext::from_objects(&[
        LeanProximityObject {
            angle_deg: 0.0,
            dist_m: 3.0,
        },
        LeanProximityObject {
            angle_deg: 0.0,
            dist_m: 1.0,
        },
    ]);
    let both = avoid.adjust_roll_pitch_rad(0.0, 0.0, 0.8, ctx);
    let closer_only = avoid.adjust_roll_pitch_rad(0.0, 0.0, 0.8, ahead(1.0));
    almost(both.pitch_rad, closer_only.pitch_rad);
}

#[test]
fn opposite_side_objects_cancel() {
    let avoid = Avoid::new();
    let ctx = LeanAngleContext::from_objects(&[
        LeanProximityObject {
            angle_deg: 90.0,
            dist_m: 1.0,
        },
        LeanProximityObject {
            angle_deg: -90.0,
            dist_m: 1.0,
        },
    ]);
    let leftover = avoid.adjust_roll_pitch_rad(0.0, 0.0, 0.8, ctx);
    almost(leftover.roll_rad, 0.0);
    almost(leftover.pitch_rad, 0.0);
}

#[test]
fn avoidance_lean_is_capped_at_seventy_five_percent() {
    let mut avoid = Avoid::new();
    avoid.set_angle_max_deg(45.0);
    let veh = radians(40.0);
    // Dist 0 → lean_norm 1 → raw avoidance is radians(45) before the cap.
    let leftover = avoid.adjust_roll_pitch_rad(0.0, 0.0, veh, ahead(0.0));
    assert!(leftover.avoidance_limited);
    let cap = veh * ANGLE_MAX_PERCENT;
    almost(leftover.pitch_rad, cap);
    almost(leftover.roll_rad, 0.0);
}

#[test]
fn ang_max_below_seventy_five_percent_is_the_cap() {
    let mut avoid = Avoid::new();
    avoid.set_angle_max_deg(8.0);
    let veh = radians(40.0);
    let leftover = avoid.adjust_roll_pitch_rad(0.0, 0.0, veh, ahead(0.0));
    assert!(leftover.avoidance_limited);
    almost(leftover.pitch_rad, radians(8.0));
}

#[test]
fn combined_lean_is_capped_at_vehicle_max() {
    let avoid = Avoid::new();
    let veh = radians(12.0);
    // Pilot already at the vehicle max, plus an object ahead.
    let leftover = avoid.adjust_roll_pitch_rad(0.0, veh, veh, ahead(0.0));
    assert!(leftover.total_limited);
    let len = Vector2f::new(leftover.roll_rad, leftover.pitch_rad).length();
    almost(len, veh);
    assert!(leftover.pitch_rad > 0.0);
}

#[test]
fn missing_proximity_still_clamps_total_lean() {
    let avoid = Avoid::new();
    let veh = radians(15.0);
    let leftover = avoid.adjust_roll_pitch_rad(0.0, veh * 2.0, veh, LeanAngleContext::empty());
    assert!(!leftover.skipped);
    assert!(leftover.total_limited);
    almost(leftover.roll_rad, 0.0);
    almost(leftover.pitch_rad, veh);
}

#[test]
fn proximity_norms_match_distance_and_bearing() {
    let avoid = Avoid::new();
    let lean = avoid.distance_m_to_lean_norm(1.0);
    let (roll_pos, roll_neg, pitch_pos, pitch_neg) =
        avoid.get_proximity_roll_pitch_norm(ahead(1.0));
    almost(roll_pos, 0.0);
    almost(roll_neg, 0.0);
    almost(pitch_pos, lean);
    almost(pitch_neg, 0.0);

    let (roll_pos, roll_neg, pitch_pos, pitch_neg) =
        avoid.get_proximity_roll_pitch_norm(right(1.0));
    almost(roll_pos, 0.0);
    almost(roll_neg, -lean);
    almost(pitch_pos, 0.0);
    almost(pitch_neg, 0.0);
}

#[test]
fn adds_pilot_lean_then_avoids() {
    let avoid = Avoid::new();
    let none = avoid.adjust_roll_pitch_rad(0.05, 0.04, 0.8, LeanAngleContext::empty());
    let with = avoid.adjust_roll_pitch_rad(0.05, 0.04, 0.8, ahead(1.0));
    almost(none.roll_rad, 0.05);
    almost(none.pitch_rad, 0.04);
    almost(with.roll_rad, 0.05);
    assert!(with.pitch_rad > none.pitch_rad);
}
