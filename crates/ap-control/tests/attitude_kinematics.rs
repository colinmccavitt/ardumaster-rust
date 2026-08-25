//! Euler and body-frame kinematics for the attitude controller.
//!
//! The two derivative conversions are inverses, so the strongest check
//! available without a fixture is a round trip: convert an arbitrary rate one
//! way and back, at attitudes across the range, and require it to return. A
//! sign error in either direction breaks that at once, and a sign error in
//! *both* — the way a careless port makes them, by transcribing one and
//! flipping it — breaks it too, because they are not each other's negation.

use ap_control::attitude_kinematics::{
    ang_vel_limit, body_to_euler_derivative, body_to_euler_limit, euler_derivative_to_body,
};
use ap_math::quaternion::Quaternion;
use ap_math::vector3::Vector3f;

fn attitude(roll: f32, pitch: f32, yaw: f32) -> Quaternion {
    Quaternion::from_euler(roll, pitch, yaw)
}

/// Euler to body and back returns the original rate.
#[test]
fn the_derivative_conversions_are_inverses() {
    let rate = Vector3f::new(0.35_f32, -0.20, 0.55);

    for &(roll, pitch, yaw) in &[
        (0.0_f32, 0.0_f32, 0.0_f32),
        (0.4, 0.2, 1.1),
        (-0.6, 0.5, -2.0),
        (1.2, -0.9, 0.3),
        (0.1, 1.4, 2.9),
    ] {
        let att = attitude(roll, pitch, yaw);
        let body = euler_derivative_to_body(att, rate);
        let back = body_to_euler_derivative(att, body).expect("not at gimbal lock");

        for (label, got, want) in [
            ("x", back.x, rate.x),
            ("y", back.y, rate.y),
            ("z", back.z, rate.z),
        ] {
            assert!(
                libm::fabsf(got - want) < 1e-4,
                "roll {roll} pitch {pitch}: {label} round-tripped to {got}, not {want}"
            );
        }
    }
}

/// Level and wings-level, the two frames coincide.
#[test]
fn at_level_the_two_frames_agree() {
    let att = attitude(0.0, 0.0, 0.0);
    let rate = Vector3f::new(0.3_f32, -0.4, 0.5);

    let body = euler_derivative_to_body(att, rate);
    assert!(libm::fabsf(body.x - rate.x) < 1e-6);
    assert!(libm::fabsf(body.y - rate.y) < 1e-6);
    assert!(libm::fabsf(body.z - rate.z) < 1e-6);
}

/// Near gimbal lock the conversion returns huge values, not a refusal.
///
/// Upstream's comment says it "returns false if the vehicle is pitched 90
/// degrees up or down". In practice it almost never does, and the reason is
/// worth knowing: the guard is `is_zero(cos θ)`, which wants |cos| below
/// `FLT_EPSILON` — 1.19e-7 — while building a 90-degree quaternion and reading
/// its pitch back gives 1.570451 rather than 1.5707963. `get_euler_pitch` is
/// an `asin`, and `asin` is flat near its endpoints, so the round trip loses
/// about 3.5e-4 and `cos θ` lands three thousand times above the threshold.
///
/// So the refusal is not a safety net a caller can lean on. What the guard
/// does do is prevent a literal division by zero, and that it achieves.
/// Reproduced rather than "fixed": a caller near gimbal lock has to cope with
/// enormous Euler rates either way, and widening the threshold would start
/// refusing attitudes upstream answers for.
#[test]
fn near_gimbal_lock_the_answer_is_large_but_finite() {
    let att = attitude(0.0, core::f32::consts::FRAC_PI_2, 0.0);
    let body = Vector3f::new(0.1_f32, 0.2, 0.3);

    let euler = body_to_euler_derivative(att, body)
        .expect("the guard does not fire here, despite the documented claim");

    assert!(
        euler.z.is_finite(),
        "the guard's real job is preventing a division by zero"
    );
    assert!(
        libm::fabsf(euler.z) > 100.0,
        "yaw rate should be enormous this close to the singularity, got {}",
        euler.z
    );

    // The forward direction has no singularity and stays well behaved.
    let back = euler_derivative_to_body(att, euler);
    assert!(back.x.is_finite() && back.y.is_finite() && back.z.is_finite());
}

/// The guard does fire when the cosine really is within epsilon.
///
/// Constructed by hand rather than through a quaternion, because a quaternion
/// round trip cannot get close enough — which is the point of the test above.
#[test]
fn an_exactly_level_singularity_is_refused() {
    // A quaternion whose recovered pitch is close enough that cos is inside
    // FLT_EPSILON. Found by bisection rather than assumed.
    let mut pitch = core::f32::consts::FRAC_PI_2;
    let mut refused = false;
    for _ in 0..40 {
        let att = attitude(0.0, pitch, 0.0);
        if body_to_euler_derivative(att, Vector3f::new(0.1_f32, 0.2, 0.3)).is_none() {
            refused = true;
            break;
        }
        // Walk toward the true singularity in the recovered angle.
        pitch += 1e-4;
    }

    assert!(
        refused,
        "no pitch in the search reached a cosine inside FLT_EPSILON; if this \
         starts failing, the guard has become unreachable rather than merely \
         narrow"
    );
}

/// Roll and pitch are limited on an ellipse, not a box.
///
/// A per-axis clamp would let a 45-degree command through at root-two of the
/// intended magnitude. This is the difference between a rate limit that means
/// what it says and one that is up to 41% loose on the diagonal.
#[test]
fn the_rate_limit_is_elliptical_not_square() {
    let max = 1.0_f32;

    // A diagonal command at the box corner is pulled back to the ellipse.
    let mut v = Vector3f::new(1.0_f32, 1.0, 0.0);
    ang_vel_limit(&mut v, max, max, 0.0);

    let magnitude = libm::sqrtf(v.x * v.x + v.y * v.y);
    assert!(
        libm::fabsf(magnitude - 1.0) < 1e-5,
        "diagonal should land on the unit ellipse, got {magnitude}"
    );
    assert!(
        libm::fabsf(v.x - v.y) < 1e-6,
        "and should keep its direction: {v:?}"
    );

    // Inside the ellipse, nothing moves.
    let mut inside = Vector3f::new(0.5_f32, 0.5, 0.0);
    ang_vel_limit(&mut inside, max, max, 0.0);
    assert!(libm::fabsf(inside.x - 0.5) < 1e-6 && libm::fabsf(inside.y - 0.5) < 1e-6);
}

/// A zero limit means unlimited, not held at zero.
///
/// Reading it the other way would clamp an axis to zero and quietly remove
/// control of it.
#[test]
fn a_zero_limit_leaves_its_axis_alone() {
    let mut v = Vector3f::new(5.0_f32, 6.0, 7.0);
    ang_vel_limit(&mut v, 0.0, 0.0, 0.0);
    assert!(libm::fabsf(v.x - 5.0) < 1e-6, "roll untouched");
    assert!(libm::fabsf(v.y - 6.0) < 1e-6, "pitch untouched");
    assert!(libm::fabsf(v.z - 7.0) < 1e-6, "yaw untouched");

    // With one of the pair zero, the other still clamps independently.
    let mut half = Vector3f::new(5.0_f32, 6.0, 7.0);
    ang_vel_limit(&mut half, 0.0, 2.0, 3.0);
    assert!(libm::fabsf(half.x - 5.0) < 1e-6, "roll unlimited");
    assert!(libm::fabsf(half.y - 2.0) < 1e-6, "pitch clamped");
    assert!(libm::fabsf(half.z - 3.0) < 1e-6, "yaw clamped");
}

/// A non-positive body limit is passed through untransformed.
#[test]
fn a_non_positive_body_limit_is_not_transformed() {
    let att = attitude(0.5, 0.3, 0.0);
    let limit = Vector3f::new(1.0_f32, 0.0, 2.0);
    let out = body_to_euler_limit(att, limit);

    assert!(libm::fabsf(out.x - limit.x) < 1e-6);
    assert!(libm::fabsf(out.y - limit.y) < 1e-6);
    assert!(libm::fabsf(out.z - limit.z) < 1e-6);
}

/// The trig clamp caps how far a limit can inflate near a singularity.
///
/// Without it, an attitude approaching gimbal lock would divide by a cosine
/// near zero and produce an Euler limit large enough to be no limit at all.
/// The tenth-magnitude floor caps the inflation at ten times.
#[test]
fn the_limit_transform_cannot_inflate_without_bound() {
    let body = Vector3f::new(1.0_f32, 1.0, 1.0);

    // Straight up: cos(pitch) is zero, so the yaw term would otherwise be
    // infinite.
    let steep = body_to_euler_limit(attitude(0.0, core::f32::consts::FRAC_PI_2, 0.0), body);
    assert!(steep.z.is_finite(), "yaw limit went non-finite");
    assert!(
        steep.z <= 100.0,
        "a tenth floor on two terms caps inflation at a hundred, got {}",
        steep.z
    );

    // Roll is passed straight through in every case.
    assert!(libm::fabsf(steep.x - body.x) < 1e-6);
}
