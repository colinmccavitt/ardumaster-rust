//! The roll and pitch stick conversions, upstream `ArduCopter/mode.cpp`.
//!
//! Both take the same two normalised stick inputs and produce a demand, but
//! for different controllers: one an attitude the aircraft should hold, the
//! other a velocity over the ground it should fly. They are kept apart because
//! the second is not the first with a different scale — it turns the stick
//! into an earth-frame command and reshapes its range on the way.

use ap_math::control::rc_input_to_roll_pitch_rad;

use ap_math::vector2::Vector2f;

/// Pilot roll and pitch sticks to a lean angle, upstream
/// `Mode::get_pilot_desired_lean_angles_rad`.
///
/// Returns `(roll_rad, pitch_rad)`, and neutral when the radio has no valid
/// input — a failsafe holding the last stick position is worse than one
/// holding none.
///
/// The shaping itself is [`rc_input_to_roll_pitch_rad`], which is shared with
/// the libraries; what is here is the failsafe decision.
#[must_use]
pub fn pilot_desired_lean_angles_rad(
    roll_in_norm: f32,
    pitch_in_norm: f32,
    angle_max_rad: f32,
    angle_limit_rad: f32,
    has_valid_input: bool,
) -> (f32, f32) {
    if !has_valid_input {
        return (0.0, 0.0);
    }
    let mut roll_out_rad = 0.0;
    let mut pitch_out_rad = 0.0;
    rc_input_to_roll_pitch_rad(
        roll_in_norm,
        pitch_in_norm,
        angle_max_rad,
        angle_limit_rad,
        &mut roll_out_rad,
        &mut pitch_out_rad,
    );
    (roll_out_rad, pitch_out_rad)
}

/// Pilot roll and pitch sticks to an earth-frame velocity, upstream
/// `Mode::get_pilot_desired_velocity`.
///
/// `cos_yaw` and `sin_yaw` are the AHRS's own, taken rather than derived from
/// a heading: recomputing them from an angle would agree to about six digits
/// and differ in the last bits, and there is no reason to introduce that when
/// the values already exist.
///
/// # Why the sticks map to `(-pitch, roll)`
///
/// Forward on the pitch stick is a *negative* deflection — pushing the nose
/// down is how a pilot asks to go forward — so the body-frame x component,
/// which points out of the nose, is its negation. Roll goes to y unchanged.
///
/// # Square stick, round envelope
///
/// The stick's travel is a square: full deflection on both axes is a corner,
/// √2 times further from centre than full deflection on one. Left alone that
/// would make the aircraft fly half again as fast diagonally as straight
/// ahead. The scaling divides by the distance to the edge of that square in
/// the direction the stick is pushed, so every direction reaches `vel_max` and
/// no more.
///
/// # The square is the compass's, not the stick's
///
/// The reshaping happens *after* the rotation to earth frame, so the square
/// being normalised against is axis-aligned to north and east rather than to
/// the sticks. The two coincide only when the aircraft points along a
/// cardinal. Everywhere else a pilot holding full stick gets less than
/// `vel_max` — exactly `vel_max`/√2 at 45 degrees, a 29% loss from yaw alone.
///
/// Upstream's comment says the transform turns a "square input range" into a
/// "circular output", and with these two steps in this order it does not.
///
/// The port reproduces it deliberately. Correcting the frame is two lines
/// swapped, and it was considered and declined: an aircraft that flies
/// differently from every other ArduCopter is a worse outcome than one
/// carrying a known and bounded quirk. See DIVERGENCES.md D-027, which has
/// the measurements and both sides of the argument.
#[must_use]
pub fn pilot_desired_velocity_ne(
    roll_in_norm: f32,
    pitch_in_norm: f32,
    vel_max: f32,
    cos_yaw: f32,
    sin_yaw: f32,
    has_valid_input: bool,
) -> Vector2f {
    if !has_valid_input {
        return Vector2f::zero();
    }

    let stick = Vector2f::new(-pitch_in_norm, roll_in_norm);

    // Tested before the rotation, on the sticks themselves. A rotation cannot
    // turn a zero vector into a non-zero one, so the order does not change the
    // answer — but it does mean the guard is asking about the pilot's input
    // rather than about a computed quantity, which is what it is for: without
    // it, a centred stick would divide by zero twice over.
    if stick.is_zero() {
        return Vector2f::zero();
    }

    let vel = Vector2f::new(
        stick.x * cos_yaw - stick.y * sin_yaw,
        stick.x * sin_yaw + stick.y * cos_yaw,
    );

    // The vector from centre to the edge of the ±1 square in this direction.
    let to_edge = vel / libm::fmaxf(libm::fabsf(vel.x), libm::fabsf(vel.y));
    vel * (vel_max / to_edge.length())
}
