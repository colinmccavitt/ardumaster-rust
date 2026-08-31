//! CCP-045 / COP-031: port of libraries/SITL/SIM_Frame.h + SIM_Frame.cpp
//! (Copter-4.7.0). Frame templates (motor angle / yaw_factor / servo index)
//! and `Frame::init` / `calculate_forces` transcribed from original source.
//!
//! Disclosed leftovers vs original, matching C++ `sim_frame.hpp`:
//! JSON `load_frame_params` is ported (COP-032). Battery voltage is owned by
//! the vehicle (`Battery::consume_energy`); Frame still sums motor current.
//!   - `AP::sitl()->vibe_motor` RPM coupling is omitted (no SITL singleton).
//!   - `get_air_density` uses troposphere ISA (0-11 km).

#![allow(missing_docs)]

use crate::sim_motor::{
    is_negative, is_zero, radians, vec3_is_zero, Motor, SitlInput, MOTORS_YAW_FACTOR_CCW,
    MOTORS_YAW_FACTOR_CW, SITL_SERVO_CHANNELS,
};
use crate::sim_plane::{Mat3, Vec3, GRAVITY_MSS, SSL_AIR_DENSITY};

pub const SIM_FRAME_MAX_ACTUATORS: usize = 32;

/// Troposphere ISA density, matching C++ `get_air_density_for_alt_amsl`
/// for the 0–11 km layer used at SITL hover `refAlt = 593 m`.
pub fn air_density_for_alt_amsl(alt_amsl: f32) -> f32 {
    const SSL_AIR_TEMPERATURE: f32 = 288.15;
    const SSL_AIR_PRESSURE: f32 = 101_325.0;
    const LAPSE_RATE: f32 = 0.0065;
    const ISA_GAS: f32 = 287.052_8;
    let temp = SSL_AIR_TEMPERATURE - LAPSE_RATE * alt_amsl;
    if temp <= 1.0 {
        return SSL_AIR_DENSITY;
    }
    let pressure =
        SSL_AIR_PRESSURE * (temp / SSL_AIR_TEMPERATURE).powf(GRAVITY_MSS / (ISA_GAS * LAPSE_RATE));
    pressure / (ISA_GAS * temp)
}

fn frame_name_matches(name: &str, prefix: &str) -> bool {
    let mut nchars = name.chars();
    for b in prefix.chars() {
        let Some(a) = nchars.next() else {
            return false;
        };
        if a.to_ascii_lowercase() != b.to_ascii_lowercase() {
            return false;
        }
    }
    true
}

fn quad_plus_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 90.0, MOTORS_YAW_FACTOR_CCW, 2),
        Motor::new(1, -90.0, MOTORS_YAW_FACTOR_CCW, 4),
        Motor::new(2, 0.0, MOTORS_YAW_FACTOR_CW, 1),
        Motor::new(3, 180.0, MOTORS_YAW_FACTOR_CW, 3),
    ]
}
fn quad_x_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 45.0, MOTORS_YAW_FACTOR_CCW, 1),
        Motor::new(1, -135.0, MOTORS_YAW_FACTOR_CCW, 3),
        Motor::new(2, -45.0, MOTORS_YAW_FACTOR_CW, 4),
        Motor::new(3, 135.0, MOTORS_YAW_FACTOR_CW, 2),
    ]
}
fn quad_bf_x_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 135.0, MOTORS_YAW_FACTOR_CW, 2),
        Motor::new(1, 45.0, MOTORS_YAW_FACTOR_CCW, 1),
        Motor::new(2, -135.0, MOTORS_YAW_FACTOR_CCW, 3),
        Motor::new(3, -45.0, MOTORS_YAW_FACTOR_CW, 4),
    ]
}
fn quad_bf_x_rev_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 135.0, MOTORS_YAW_FACTOR_CCW, 2),
        Motor::new(1, 45.0, MOTORS_YAW_FACTOR_CW, 1),
        Motor::new(2, -135.0, MOTORS_YAW_FACTOR_CW, 3),
        Motor::new(3, -45.0, MOTORS_YAW_FACTOR_CCW, 4),
    ]
}
fn quad_dji_x_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 45.0, MOTORS_YAW_FACTOR_CCW, 1),
        Motor::new(1, -45.0, MOTORS_YAW_FACTOR_CW, 4),
        Motor::new(2, -135.0, MOTORS_YAW_FACTOR_CCW, 3),
        Motor::new(3, 135.0, MOTORS_YAW_FACTOR_CW, 2),
    ]
}
fn quad_cw_x_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 45.0, MOTORS_YAW_FACTOR_CCW, 1),
        Motor::new(1, 135.0, MOTORS_YAW_FACTOR_CW, 2),
        Motor::new(2, -135.0, MOTORS_YAW_FACTOR_CCW, 3),
        Motor::new(3, -45.0, MOTORS_YAW_FACTOR_CW, 4),
    ]
}
fn dotriaconta_octaquad_x_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 45.0, MOTORS_YAW_FACTOR_CCW, 1),
        Motor::new(1, -135.0, MOTORS_YAW_FACTOR_CCW, 17),
        Motor::new(2, -45.0, MOTORS_YAW_FACTOR_CW, 25),
        Motor::new(3, 135.0, MOTORS_YAW_FACTOR_CW, 9),
        Motor::new(4, 45.0, MOTORS_YAW_FACTOR_CW, 2),
        Motor::new(5, -135.0, MOTORS_YAW_FACTOR_CW, 18),
        Motor::new(6, -45.0, MOTORS_YAW_FACTOR_CCW, 26),
        Motor::new(7, 135.0, MOTORS_YAW_FACTOR_CCW, 10),
        Motor::new(8, 45.0, MOTORS_YAW_FACTOR_CCW, 3),
        Motor::new(9, -135.0, MOTORS_YAW_FACTOR_CCW, 19),
        Motor::new(10, -45.0, MOTORS_YAW_FACTOR_CW, 27),
        Motor::new(11, 135.0, MOTORS_YAW_FACTOR_CW, 11),
        Motor::new(12, 45.0, MOTORS_YAW_FACTOR_CW, 4),
        Motor::new(13, -135.0, MOTORS_YAW_FACTOR_CW, 20),
        Motor::new(14, -45.0, MOTORS_YAW_FACTOR_CCW, 28),
        Motor::new(15, 135.0, MOTORS_YAW_FACTOR_CCW, 12),
        Motor::new(16, 45.0, MOTORS_YAW_FACTOR_CCW, 5),
        Motor::new(17, -135.0, MOTORS_YAW_FACTOR_CCW, 21),
        Motor::new(18, -45.0, MOTORS_YAW_FACTOR_CW, 29),
        Motor::new(19, 135.0, MOTORS_YAW_FACTOR_CW, 13),
        Motor::new(20, 45.0, MOTORS_YAW_FACTOR_CW, 6),
        Motor::new(21, -135.0, MOTORS_YAW_FACTOR_CW, 22),
        Motor::new(22, -45.0, MOTORS_YAW_FACTOR_CCW, 30),
        Motor::new(23, 135.0, MOTORS_YAW_FACTOR_CCW, 14),
        Motor::new(24, 45.0, MOTORS_YAW_FACTOR_CCW, 7),
        Motor::new(25, -135.0, MOTORS_YAW_FACTOR_CCW, 23),
        Motor::new(26, -45.0, MOTORS_YAW_FACTOR_CW, 31),
        Motor::new(27, 135.0, MOTORS_YAW_FACTOR_CW, 15),
        Motor::new(28, 45.0, MOTORS_YAW_FACTOR_CW, 8),
        Motor::new(29, -135.0, MOTORS_YAW_FACTOR_CW, 24),
        Motor::new(30, -45.0, MOTORS_YAW_FACTOR_CCW, 32),
        Motor::new(31, 135.0, MOTORS_YAW_FACTOR_CCW, 16),
    ]
}
fn tiltquad_h_vectored_motors() -> Vec<Motor> {
    vec![
        Motor::with_tilt(
            0,
            45.0,
            MOTORS_YAW_FACTOR_CW,
            1,
            -1,
            0.0,
            0.0,
            7,
            10.0,
            -90.0,
        ),
        Motor::with_tilt(
            1,
            -135.0,
            MOTORS_YAW_FACTOR_CW,
            3,
            -1,
            0.0,
            0.0,
            8,
            10.0,
            -90.0,
        ),
        Motor::with_tilt(
            2,
            -45.0,
            MOTORS_YAW_FACTOR_CCW,
            4,
            -1,
            0.0,
            0.0,
            8,
            10.0,
            -90.0,
        ),
        Motor::with_tilt(
            3,
            135.0,
            MOTORS_YAW_FACTOR_CCW,
            2,
            -1,
            0.0,
            0.0,
            7,
            10.0,
            -90.0,
        ),
    ]
}
fn tiltquad_motors() -> Vec<Motor> {
    vec![
        Motor::with_tilt(
            0,
            45.0,
            MOTORS_YAW_FACTOR_CCW,
            1,
            -1,
            0.0,
            0.0,
            7,
            10.0,
            -90.0,
        ),
        Motor::new(1, -135.0, MOTORS_YAW_FACTOR_CCW, 3),
        Motor::with_tilt(
            2,
            -45.0,
            MOTORS_YAW_FACTOR_CW,
            4,
            -1,
            0.0,
            0.0,
            8,
            10.0,
            -90.0,
        ),
        Motor::new(3, 135.0, MOTORS_YAW_FACTOR_CW, 2),
    ]
}
fn hexa_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 0.0, MOTORS_YAW_FACTOR_CW, 1),
        Motor::new(1, 180.0, MOTORS_YAW_FACTOR_CCW, 4),
        Motor::new(2, -120.0, MOTORS_YAW_FACTOR_CW, 5),
        Motor::new(3, 60.0, MOTORS_YAW_FACTOR_CCW, 2),
        Motor::new(4, -60.0, MOTORS_YAW_FACTOR_CCW, 6),
        Motor::new(5, 120.0, MOTORS_YAW_FACTOR_CW, 3),
    ]
}
fn hexax_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 90.0, MOTORS_YAW_FACTOR_CW, 2),
        Motor::new(1, -90.0, MOTORS_YAW_FACTOR_CCW, 5),
        Motor::new(2, -30.0, MOTORS_YAW_FACTOR_CW, 6),
        Motor::new(3, 150.0, MOTORS_YAW_FACTOR_CCW, 3),
        Motor::new(4, 30.0, MOTORS_YAW_FACTOR_CCW, 1),
        Motor::new(5, -150.0, MOTORS_YAW_FACTOR_CW, 4),
    ]
}
fn hexa_dji_x_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 30.0, MOTORS_YAW_FACTOR_CCW, 1),
        Motor::new(1, -30.0, MOTORS_YAW_FACTOR_CW, 6),
        Motor::new(2, -90.0, MOTORS_YAW_FACTOR_CCW, 5),
        Motor::new(3, -150.0, MOTORS_YAW_FACTOR_CW, 4),
        Motor::new(4, 150.0, MOTORS_YAW_FACTOR_CCW, 3),
        Motor::new(5, 90.0, MOTORS_YAW_FACTOR_CW, 2),
    ]
}
fn hexa_cw_x_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 30.0, MOTORS_YAW_FACTOR_CCW, 1),
        Motor::new(1, 90.0, MOTORS_YAW_FACTOR_CW, 2),
        Motor::new(2, 150.0, MOTORS_YAW_FACTOR_CCW, 3),
        Motor::new(3, -150.0, MOTORS_YAW_FACTOR_CW, 4),
        Motor::new(4, -90.0, MOTORS_YAW_FACTOR_CCW, 5),
        Motor::new(5, -30.0, MOTORS_YAW_FACTOR_CW, 6),
    ]
}
fn octa_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 0.0, MOTORS_YAW_FACTOR_CW, 1),
        Motor::new(1, 180.0, MOTORS_YAW_FACTOR_CW, 5),
        Motor::new(2, 45.0, MOTORS_YAW_FACTOR_CCW, 2),
        Motor::new(3, 135.0, MOTORS_YAW_FACTOR_CCW, 4),
        Motor::new(4, -45.0, MOTORS_YAW_FACTOR_CCW, 8),
        Motor::new(5, -135.0, MOTORS_YAW_FACTOR_CCW, 6),
        Motor::new(6, -90.0, MOTORS_YAW_FACTOR_CW, 7),
        Motor::new(7, 90.0, MOTORS_YAW_FACTOR_CW, 3),
    ]
}
fn octa_dji_x_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 22.5, MOTORS_YAW_FACTOR_CCW, 1),
        Motor::new(1, -22.5, MOTORS_YAW_FACTOR_CW, 8),
        Motor::new(2, -67.5, MOTORS_YAW_FACTOR_CCW, 7),
        Motor::new(3, -112.5, MOTORS_YAW_FACTOR_CW, 6),
        Motor::new(4, -157.5, MOTORS_YAW_FACTOR_CCW, 5),
        Motor::new(5, 157.5, MOTORS_YAW_FACTOR_CW, 4),
        Motor::new(6, 112.5, MOTORS_YAW_FACTOR_CCW, 3),
        Motor::new(7, 67.5, MOTORS_YAW_FACTOR_CW, 2),
    ]
}
fn octa_cw_x_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 22.5, MOTORS_YAW_FACTOR_CCW, 1),
        Motor::new(1, 67.5, MOTORS_YAW_FACTOR_CW, 2),
        Motor::new(2, 112.5, MOTORS_YAW_FACTOR_CCW, 3),
        Motor::new(3, 157.5, MOTORS_YAW_FACTOR_CW, 4),
        Motor::new(4, -157.5, MOTORS_YAW_FACTOR_CCW, 5),
        Motor::new(5, -112.5, MOTORS_YAW_FACTOR_CW, 6),
        Motor::new(6, -67.5, MOTORS_YAW_FACTOR_CCW, 7),
        Motor::new(7, -22.5, MOTORS_YAW_FACTOR_CW, 8),
    ]
}
fn octa_quad_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 45.0, MOTORS_YAW_FACTOR_CCW, 1),
        Motor::new(1, -45.0, MOTORS_YAW_FACTOR_CW, 7),
        Motor::new(2, -135.0, MOTORS_YAW_FACTOR_CCW, 5),
        Motor::new(3, 135.0, MOTORS_YAW_FACTOR_CW, 3),
        Motor::new(4, -45.0, MOTORS_YAW_FACTOR_CCW, 8),
        Motor::new(5, 45.0, MOTORS_YAW_FACTOR_CW, 2),
        Motor::new(6, 135.0, MOTORS_YAW_FACTOR_CCW, 4),
        Motor::new(7, -135.0, MOTORS_YAW_FACTOR_CW, 6),
    ]
}
fn octa_quad_corotating_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 45.0, MOTORS_YAW_FACTOR_CCW, 1),
        Motor::new(1, -45.0, MOTORS_YAW_FACTOR_CW, 7),
        Motor::new(2, -135.0, MOTORS_YAW_FACTOR_CCW, 5),
        Motor::new(3, 135.0, MOTORS_YAW_FACTOR_CW, 3),
        Motor::new(4, -45.0, MOTORS_YAW_FACTOR_CCW, 8),
        Motor::new(5, 45.0, MOTORS_YAW_FACTOR_CW, 2),
        Motor::new(6, 135.0, MOTORS_YAW_FACTOR_CCW, 4),
        Motor::new(7, -135.0, MOTORS_YAW_FACTOR_CW, 6),
    ]
}
fn octa_quad_cw_corotating_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 45.0, MOTORS_YAW_FACTOR_CCW, 1),
        Motor::new(1, 45.0, MOTORS_YAW_FACTOR_CCW, 2),
        Motor::new(2, 135.0, MOTORS_YAW_FACTOR_CW, 3),
        Motor::new(3, 135.0, MOTORS_YAW_FACTOR_CW, 4),
        Motor::new(4, -135.0, MOTORS_YAW_FACTOR_CCW, 5),
        Motor::new(5, -135.0, MOTORS_YAW_FACTOR_CCW, 6),
        Motor::new(6, -45.0, MOTORS_YAW_FACTOR_CW, 7),
        Motor::new(7, -45.0, MOTORS_YAW_FACTOR_CW, 8),
    ]
}
fn octa_quad_cw_x_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 45.0, MOTORS_YAW_FACTOR_CCW, 1),
        Motor::new(1, 45.0, MOTORS_YAW_FACTOR_CW, 2),
        Motor::new(2, 135.0, MOTORS_YAW_FACTOR_CCW, 3),
        Motor::new(3, 135.0, MOTORS_YAW_FACTOR_CW, 4),
        Motor::new(4, -135.0, MOTORS_YAW_FACTOR_CCW, 5),
        Motor::new(5, -135.0, MOTORS_YAW_FACTOR_CW, 6),
        Motor::new(6, -45.0, MOTORS_YAW_FACTOR_CCW, 7),
        Motor::new(7, -45.0, MOTORS_YAW_FACTOR_CW, 8),
    ]
}
fn dodeca_hexa_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 30.0, MOTORS_YAW_FACTOR_CCW, 1),
        Motor::new(1, 30.0, MOTORS_YAW_FACTOR_CW, 2),
        Motor::new(2, 90.0, MOTORS_YAW_FACTOR_CW, 3),
        Motor::new(3, 90.0, MOTORS_YAW_FACTOR_CCW, 4),
        Motor::new(4, 150.0, MOTORS_YAW_FACTOR_CCW, 5),
        Motor::new(5, 150.0, MOTORS_YAW_FACTOR_CW, 6),
        Motor::new(6, -150.0, MOTORS_YAW_FACTOR_CW, 7),
        Motor::new(7, -150.0, MOTORS_YAW_FACTOR_CCW, 8),
        Motor::new(8, -90.0, MOTORS_YAW_FACTOR_CCW, 9),
        Motor::new(9, -90.0, MOTORS_YAW_FACTOR_CW, 10),
        Motor::new(10, -30.0, MOTORS_YAW_FACTOR_CW, 11),
        Motor::new(11, -30.0, MOTORS_YAW_FACTOR_CCW, 12),
    ]
}
fn hexadeca_octa_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 0.0, MOTORS_YAW_FACTOR_CW, 1),
        Motor::new(1, 0.0, MOTORS_YAW_FACTOR_CCW, 2),
        Motor::new(2, 45.0, MOTORS_YAW_FACTOR_CCW, 3),
        Motor::new(3, 45.0, MOTORS_YAW_FACTOR_CW, 4),
        Motor::new(4, 90.0, MOTORS_YAW_FACTOR_CW, 5),
        Motor::new(5, 90.0, MOTORS_YAW_FACTOR_CCW, 6),
        Motor::new(6, 135.0, MOTORS_YAW_FACTOR_CCW, 7),
        Motor::new(7, 135.0, MOTORS_YAW_FACTOR_CW, 8),
        Motor::new(8, 180.0, MOTORS_YAW_FACTOR_CW, 9),
        Motor::new(9, 180.0, MOTORS_YAW_FACTOR_CCW, 10),
        Motor::new(10, -135.0, MOTORS_YAW_FACTOR_CCW, 11),
        Motor::new(11, -135.0, MOTORS_YAW_FACTOR_CW, 12),
        Motor::new(12, -90.0, MOTORS_YAW_FACTOR_CCW, 13),
        Motor::new(13, -90.0, MOTORS_YAW_FACTOR_CW, 14),
        Motor::new(14, -45.0, MOTORS_YAW_FACTOR_CCW, 15),
        Motor::new(15, -45.0, MOTORS_YAW_FACTOR_CW, 16),
    ]
}
fn hexadeca_octa_cw_x_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 22.5, MOTORS_YAW_FACTOR_CW, 1),
        Motor::new(1, 22.5, MOTORS_YAW_FACTOR_CCW, 2),
        Motor::new(2, 67.5, MOTORS_YAW_FACTOR_CCW, 3),
        Motor::new(3, 67.5, MOTORS_YAW_FACTOR_CW, 4),
        Motor::new(4, 112.5, MOTORS_YAW_FACTOR_CW, 5),
        Motor::new(5, 112.5, MOTORS_YAW_FACTOR_CCW, 6),
        Motor::new(6, 157.5, MOTORS_YAW_FACTOR_CCW, 7),
        Motor::new(7, 157.5, MOTORS_YAW_FACTOR_CW, 8),
        Motor::new(8, -157.5, MOTORS_YAW_FACTOR_CW, 9),
        Motor::new(9, -157.5, MOTORS_YAW_FACTOR_CCW, 10),
        Motor::new(10, -112.5, MOTORS_YAW_FACTOR_CCW, 11),
        Motor::new(11, -112.5, MOTORS_YAW_FACTOR_CW, 12),
        Motor::new(12, -67.5, MOTORS_YAW_FACTOR_CCW, 13),
        Motor::new(13, -67.5, MOTORS_YAW_FACTOR_CW, 14),
        Motor::new(14, -22.5, MOTORS_YAW_FACTOR_CCW, 15),
        Motor::new(15, -22.5, MOTORS_YAW_FACTOR_CW, 16),
    ]
}
fn deca_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 0.0, MOTORS_YAW_FACTOR_CCW, 1),
        Motor::new(1, 36.0, MOTORS_YAW_FACTOR_CW, 2),
        Motor::new(2, 72.0, MOTORS_YAW_FACTOR_CCW, 3),
        Motor::new(3, 108.0, MOTORS_YAW_FACTOR_CW, 4),
        Motor::new(4, 144.0, MOTORS_YAW_FACTOR_CCW, 5),
        Motor::new(5, 180.0, MOTORS_YAW_FACTOR_CW, 6),
        Motor::new(6, -144.0, MOTORS_YAW_FACTOR_CCW, 7),
        Motor::new(7, -108.0, MOTORS_YAW_FACTOR_CW, 8),
        Motor::new(8, -72.0, MOTORS_YAW_FACTOR_CCW, 9),
        Motor::new(9, -36.0, MOTORS_YAW_FACTOR_CW, 10),
    ]
}
fn deca_cw_x_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 18.0, MOTORS_YAW_FACTOR_CCW, 1),
        Motor::new(1, 54.0, MOTORS_YAW_FACTOR_CW, 2),
        Motor::new(2, 90.0, MOTORS_YAW_FACTOR_CCW, 3),
        Motor::new(3, 126.0, MOTORS_YAW_FACTOR_CW, 4),
        Motor::new(4, 162.0, MOTORS_YAW_FACTOR_CCW, 5),
        Motor::new(5, -162.0, MOTORS_YAW_FACTOR_CW, 6),
        Motor::new(6, -126.0, MOTORS_YAW_FACTOR_CCW, 7),
        Motor::new(7, -90.0, MOTORS_YAW_FACTOR_CW, 8),
        Motor::new(8, -54.0, MOTORS_YAW_FACTOR_CCW, 9),
        Motor::new(9, -18.0, MOTORS_YAW_FACTOR_CW, 10),
    ]
}
fn tri_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 60.0, MOTORS_YAW_FACTOR_CCW, 1),
        Motor::new(1, -60.0, MOTORS_YAW_FACTOR_CW, 3),
        Motor::with_tilt(
            3,
            180.0,
            MOTORS_YAW_FACTOR_CCW,
            2,
            6,
            60.0,
            -60.0,
            -1,
            0.0,
            0.0,
        ),
    ]
}
fn tilttri_motors() -> Vec<Motor> {
    vec![
        Motor::with_tilt(
            0,
            60.0,
            MOTORS_YAW_FACTOR_CCW,
            1,
            -1,
            0.0,
            0.0,
            7,
            0.0,
            -90.0,
        ),
        Motor::with_tilt(
            1,
            -60.0,
            MOTORS_YAW_FACTOR_CW,
            3,
            -1,
            0.0,
            0.0,
            7,
            0.0,
            -90.0,
        ),
        Motor::with_tilt(
            3,
            180.0,
            MOTORS_YAW_FACTOR_CCW,
            2,
            6,
            60.0,
            -60.0,
            -1,
            0.0,
            0.0,
        ),
    ]
}
fn tilttri_vectored_motors() -> Vec<Motor> {
    vec![
        Motor::with_tilt(
            0,
            60.0,
            MOTORS_YAW_FACTOR_CCW,
            1,
            -1,
            0.0,
            0.0,
            7,
            10.0,
            -90.0,
        ),
        Motor::with_tilt(
            1,
            -60.0,
            MOTORS_YAW_FACTOR_CW,
            3,
            -1,
            0.0,
            0.0,
            8,
            10.0,
            -90.0,
        ),
        Motor::new(3, 180.0, MOTORS_YAW_FACTOR_CCW, 2),
    ]
}
fn y6_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 60.0, MOTORS_YAW_FACTOR_CCW, 2),
        Motor::new(1, -60.0, MOTORS_YAW_FACTOR_CW, 5),
        Motor::new(2, -60.0, MOTORS_YAW_FACTOR_CCW, 6),
        Motor::new(3, 180.0, MOTORS_YAW_FACTOR_CW, 4),
        Motor::new(4, 60.0, MOTORS_YAW_FACTOR_CW, 1),
        Motor::new(5, 180.0, MOTORS_YAW_FACTOR_CCW, 3),
    ]
}
fn firefly_motors() -> Vec<Motor> {
    vec![
        Motor::new(0, 180.0, MOTORS_YAW_FACTOR_CCW, 3),
        Motor::with_tilt(
            1,
            60.0,
            MOTORS_YAW_FACTOR_CCW,
            1,
            -1,
            0.0,
            0.0,
            6,
            0.0,
            -90.0,
        ),
        Motor::with_tilt(
            2,
            -60.0,
            MOTORS_YAW_FACTOR_CCW,
            5,
            -1,
            0.0,
            0.0,
            6,
            0.0,
            -90.0,
        ),
        Motor::new(3, 180.0, MOTORS_YAW_FACTOR_CW, 4),
        Motor::with_tilt(
            4,
            60.0,
            MOTORS_YAW_FACTOR_CW,
            2,
            -1,
            0.0,
            0.0,
            6,
            0.0,
            -90.0,
        ),
        Motor::with_tilt(
            5,
            -60.0,
            MOTORS_YAW_FACTOR_CW,
            6,
            -1,
            0.0,
            0.0,
            6,
            0.0,
            -90.0,
        ),
    ]
}

type FrameFactory = fn() -> Vec<Motor>;

const SUPPORTED_FRAME_TEMPLATES: &[(&str, FrameFactory, u8)] = &[
    ("+", quad_plus_motors, 4),
    ("quad", quad_plus_motors, 4),
    ("copter", quad_plus_motors, 4),
    ("x", quad_x_motors, 4),
    ("bfxrev", quad_bf_x_rev_motors, 4),
    ("bfx", quad_bf_x_motors, 4),
    ("dotriaconta", dotriaconta_octaquad_x_motors, 32),
    ("djix", quad_dji_x_motors, 4),
    ("cwx", quad_cw_x_motors, 4),
    ("tilthvec", tiltquad_h_vectored_motors, 4),
    ("hexadeca-octa", hexadeca_octa_motors, 16),
    ("hexadeca-octa-cwx", hexadeca_octa_cw_x_motors, 16),
    ("hexax", hexax_motors, 6),
    ("hexa-cwx", hexa_cw_x_motors, 6),
    ("hexa-dji", hexa_dji_x_motors, 6),
    ("hexa", hexa_motors, 6),
    ("octa-cwx", octa_cw_x_motors, 8),
    ("octa-dji", octa_dji_x_motors, 8),
    ("octa-quad-cwx", octa_quad_cw_x_motors, 8),
    ("octa-quad-cor", octa_quad_corotating_motors, 8),
    ("octa-quad-cw-cor", octa_quad_cw_corotating_motors, 8),
    ("octa-quad", octa_quad_motors, 8),
    ("octa", octa_motors, 8),
    ("deca", deca_motors, 10),
    ("deca-cwx", deca_cw_x_motors, 10),
    ("dodeca-hexa", dodeca_hexa_motors, 12),
    ("tri", tri_motors, 3),
    ("tilttrivec", tilttri_vectored_motors, 3),
    ("tilttri", tilttri_motors, 3),
    ("y6", y6_motors, 6),
    ("firefly", firefly_motors, 6),
    ("tilt", tiltquad_motors, 4),
];

#[derive(Debug, Clone)]
pub struct FrameModel {
    pub mass: f32,
    pub diagonal_size: f32,
    pub ref_spd: f32,
    pub ref_angle: f32,
    pub ref_voltage: f32,
    pub ref_current: f32,
    pub ref_alt: f32,
    pub ref_temp_c: f32,
    pub ref_bat_res: f32,
    pub max_voltage: f32,
    pub batt_capacity_ah: f32,
    pub hover_thr_out: f32,
    pub prop_expo: f32,
    pub ref_rot_rate: f32,
    pub pwm_min: f32,
    pub pwm_max: f32,
    pub spin_min: f32,
    pub spin_max: f32,
    pub slew_max: f32,
    pub disc_area: f32,
    pub mdrag_coef: f32,
    pub bbdrag_coef: f32,
    pub moment_of_inertia: Vec3,
    pub motor_pos: [Vec3; SIM_FRAME_MAX_ACTUATORS],
    pub motor_thrust_vec: [Vec3; SIM_FRAME_MAX_ACTUATORS],
    pub yaw_factor: [f32; SIM_FRAME_MAX_ACTUATORS],
    pub num_motors: f32,
}

impl Default for FrameModel {
    fn default() -> Self {
        Self {
            mass: 3.0,
            diagonal_size: 0.35,
            ref_spd: 15.08,
            ref_angle: 45.0,
            ref_voltage: 12.09,
            ref_current: 29.3,
            ref_alt: 593.0,
            ref_temp_c: 25.0,
            ref_bat_res: 0.01,
            max_voltage: 4.2 * 3.0,
            batt_capacity_ah: 0.0,
            hover_thr_out: 0.39,
            prop_expo: 0.65,
            ref_rot_rate: 120.0,
            pwm_min: 1000.0,
            pwm_max: 2000.0,
            spin_min: 0.15,
            spin_max: 0.95,
            slew_max: 150.0,
            disc_area: 0.385,
            mdrag_coef: 0.2,
            bbdrag_coef: 1.0,
            moment_of_inertia: Vec3::zero(),
            motor_pos: [Vec3::zero(); SIM_FRAME_MAX_ACTUATORS],
            motor_thrust_vec: [Vec3::zero(); SIM_FRAME_MAX_ACTUATORS],
            yaw_factor: [0.0; SIM_FRAME_MAX_ACTUATORS],
            num_motors: 4.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Frame {
    pub name: &'static str,
    pub num_motors: u8,
    pub motor_offset: u8,
    pub terminal_velocity: f32,
    pub terminal_rotation_rate: f32,
    motors: [Motor; SIM_FRAME_MAX_ACTUATORS],
    model: FrameModel,
    area_cd: f32,
    mass: f32,
    battery_voltage: f32,
    pub mass_scale: f32,
    pub battery_dirty: bool,
}

impl Default for Frame {
    fn default() -> Self {
        Self {
            name: "",
            num_motors: 0,
            motor_offset: 0,
            terminal_velocity: 0.0,
            terminal_rotation_rate: 0.0,
            motors: core::array::from_fn(|_| Motor::default()),
            model: FrameModel::default(),
            area_cd: 0.0,
            mass: 3.0,
            battery_voltage: 12.6,
            mass_scale: 1.0,
            battery_dirty: false,
        }
    }
}

impl Frame {
    pub fn from_template(name: &'static str, src: Vec<Motor>) -> Self {
        let nmot = src.len().min(SIM_FRAME_MAX_ACTUATORS) as u8;
        let mut motors = core::array::from_fn(|_| Motor::default());
        for (i, m) in src.into_iter().take(SIM_FRAME_MAX_ACTUATORS).enumerate() {
            motors[i] = m;
        }
        Self {
            name,
            num_motors: nmot,
            motors,
            ..Self::default()
        }
    }

    pub fn create_frame(frame_name: &str) -> Self {
        for &(n, factory, nmot) in SUPPORTED_FRAME_TEMPLATES {
            if frame_name_matches(frame_name, n) {
                let mut f = Self::from_template(n, factory());
                f.num_motors = nmot;
                return f;
            }
        }
        Self::default()
    }

    pub fn valid(&self) -> bool {
        self.num_motors > 0
    }

    pub fn motors(&self) -> &[Motor] {
        &self.motors[..self.num_motors as usize]
    }

    pub fn motors_mut(&mut self) -> &mut [Motor] {
        let n = self.num_motors as usize;
        &mut self.motors[..n]
    }

    pub fn get_model(&self) -> &FrameModel {
        &self.model
    }

    pub fn load_frame_params(&mut self, model_json: &str) -> bool {
        use crate::sim_json::{json_get_float, json_get_vector3, load_json_file};
        let obj = match load_json_file(std::path::Path::new(model_json)) {
            Ok(v) => v,
            Err(_) => return false,
        };
        json_get_float(&obj, "mass", &mut self.model.mass);
        json_get_float(&obj, "diagonal_size", &mut self.model.diagonal_size);
        json_get_float(&obj, "refSpd", &mut self.model.ref_spd);
        json_get_float(&obj, "refAngle", &mut self.model.ref_angle);
        json_get_float(&obj, "refVoltage", &mut self.model.ref_voltage);
        json_get_float(&obj, "refCurrent", &mut self.model.ref_current);
        json_get_float(&obj, "refAlt", &mut self.model.ref_alt);
        json_get_float(&obj, "maxVoltage", &mut self.model.max_voltage);
        json_get_float(&obj, "battCapacityAh", &mut self.model.batt_capacity_ah);
        json_get_float(&obj, "refBatRes", &mut self.model.ref_bat_res);
        json_get_float(&obj, "propExpo", &mut self.model.prop_expo);
        json_get_float(&obj, "refRotRate", &mut self.model.ref_rot_rate);
        json_get_float(&obj, "hoverThrOut", &mut self.model.hover_thr_out);
        json_get_float(&obj, "pwmMin", &mut self.model.pwm_min);
        json_get_float(&obj, "pwmMax", &mut self.model.pwm_max);
        json_get_float(&obj, "spin_min", &mut self.model.spin_min);
        json_get_float(&obj, "spin_max", &mut self.model.spin_max);
        json_get_float(&obj, "slew_max", &mut self.model.slew_max);
        json_get_float(&obj, "disc_area", &mut self.model.disc_area);
        json_get_float(&obj, "mdrag_coef", &mut self.model.mdrag_coef);
        json_get_float(&obj, "bbdrag_coef", &mut self.model.bbdrag_coef);
        json_get_float(&obj, "refTempC", &mut self.model.ref_temp_c);
        json_get_float(&obj, "num_motors", &mut self.model.num_motors);
        json_get_vector3(&obj, "moment_inertia", &mut self.model.moment_of_inertia);
        for j in 0..SIM_FRAME_MAX_ACTUATORS {
            let n = j + 1;
            json_get_vector3(&obj, &format!("motor{n}_position"), &mut self.model.motor_pos[j]);
            json_get_vector3(&obj, &format!("motor{n}_vector"), &mut self.model.motor_thrust_vec[j]);
            json_get_float(&obj, &format!("motor{n}_yaw"), &mut self.model.yaw_factor[j]);
        }
        true
    }

    pub fn init(&mut self, frame_str: &str) {
        self.model = FrameModel::default();
        if let Some(colon) = frame_str.find(':') {
            let path = &frame_str[colon + 1..];
            if path.ends_with(".json") {
                let _ = self.load_frame_params(path);
            }
        }
        self.mass = self.model.mass * self.mass_scale;

        let drag_force = self.model.mass * GRAVITY_MSS * radians(self.model.ref_angle).tan();
        let cos_tilt = radians(self.model.ref_angle).cos();
        let airspeed_bf = self.model.ref_spd * cos_tilt;
        let ref_thrust = self.model.mass * GRAVITY_MSS / cos_tilt;
        let ref_air_density = air_density_for_alt_amsl(self.model.ref_alt);

        let momentum_drag = cos_tilt
            * self.model.mdrag_coef
            * airspeed_bf
            * (ref_thrust * ref_air_density * self.model.disc_area).sqrt();

        if momentum_drag > drag_force {
            self.model.mdrag_coef *= drag_force / momentum_drag;
            self.area_cd = 0.0;
        } else {
            self.area_cd = self.model.bbdrag_coef * (drag_force - momentum_drag)
                / (0.5 * ref_air_density * self.model.ref_spd * self.model.ref_spd);
        }

        self.terminal_rotation_rate = self.model.ref_rot_rate;

        let hover_thrust = self.mass * GRAVITY_MSS;
        let hover_power = self.model.ref_current * self.model.ref_voltage;
        let hover_velocity_out = 2.0 * hover_power / hover_thrust;
        let effective_disc_area =
            hover_thrust / (0.5 * ref_air_density * hover_velocity_out * hover_velocity_out);
        let velocity_max = hover_velocity_out / self.model.hover_thr_out.sqrt();
        let n = self.num_motors as f32;
        let effective_prop_area = effective_disc_area / n;
        let true_prop_area = self.model.disc_area / n;
        let power_factor = hover_power / hover_thrust;

        self.battery_voltage = self.model.max_voltage;
        self.battery_dirty = true;

        let model = self.model.clone();
        let nmot = self.num_motors as usize;
        for i in 0..nmot {
            self.motors[i].setup_params(
                model.pwm_min as u16,
                model.pwm_max as u16,
                model.spin_min,
                model.spin_max,
                model.prop_expo,
                model.slew_max,
                model.diagonal_size,
                power_factor,
                model.max_voltage,
                effective_prop_area,
                velocity_max,
                model.motor_pos[i],
                model.motor_thrust_vec[i],
                model.yaw_factor[i],
                true_prop_area,
                model.mdrag_coef,
            );
        }

        if is_zero(self.model.moment_of_inertia.x)
            || is_zero(self.model.moment_of_inertia.y)
            || is_zero(self.model.moment_of_inertia.z)
        {
            let half = self.model.diagonal_size * 0.5;
            self.model.moment_of_inertia.x = self.model.mass * 0.25 * half * half;
            self.model.moment_of_inertia.y = self.model.moment_of_inertia.x;
            self.model.moment_of_inertia.z = self.model.mass * 0.5 * half * half;
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn calculate_forces(
        &mut self,
        dcm: Mat3,
        velocity_ef: Vec3,
        gyro: Vec3,
        alt_amsl: f32,
        input: &SitlInput,
        gross_mass: f32,
        use_drag: bool,
        time_us: u64,
    ) -> (Vec3, Vec3) {
        let mut thrust = Vec3::zero();
        let mut torque = Vec3::zero();
        let air_density = air_density_for_alt_amsl(alt_amsl);
        let vel_air_bf = dcm.transposed().apply(velocity_ef);
        let nmot = self.num_motors as usize;
        let motor_offset = self.motor_offset;
        let battery_voltage = self.battery_voltage;
        for i in 0..nmot {
            let (mtorque, mthrust) = self.motors[i].calculate_forces(
                input,
                motor_offset,
                vel_air_bf,
                gyro,
                air_density,
                battery_voltage,
                use_drag,
                time_us,
            );
            torque = torque.plus(mtorque);
            thrust = thrust.plus(mthrust);
        }

        let mut rot_accel = Vec3::new(
            torque.x / self.model.moment_of_inertia.x,
            torque.y / self.model.moment_of_inertia.y,
            torque.z / self.model.moment_of_inertia.z,
        );

        if self.terminal_rotation_rate > 0.0 {
            let damp = radians(400.0) / self.terminal_rotation_rate;
            rot_accel.x -= gyro.x * damp;
            rot_accel.y -= gyro.y * damp;
            rot_accel.z -= gyro.z * damp;
        }

        if use_drag {
            let mut drag_bf = Vec3::new(
                self.area_cd * 0.5 * air_density * vel_air_bf.x * vel_air_bf.x,
                self.area_cd * 0.5 * air_density * vel_air_bf.y * vel_air_bf.y,
                self.area_cd * 0.5 * air_density * vel_air_bf.z * vel_air_bf.z,
            );
            if is_negative(vel_air_bf.x) {
                drag_bf.x = -drag_bf.x;
            }
            if is_negative(vel_air_bf.y) {
                drag_bf.y = -drag_bf.y;
            }
            if is_negative(vel_air_bf.z) {
                drag_bf.z = -drag_bf.z;
            }
            thrust = thrust.minus(drag_bf);
        }

        let body_accel = thrust.scaled(1.0 / gross_mass);
        (rot_accel, body_accel)
    }

    pub fn get_mass(&self) -> f32 {
        self.mass
    }
    pub fn hover_thr_out(&self) -> f32 {
        self.model.hover_thr_out
    }
    pub fn hover_command(&self) -> f32 {
        let e = self.model.prop_expo;
        let h = self.model.hover_thr_out;
        if e <= 1.0e-6 {
            return h;
        }
        let disc = (1.0 - e) * (1.0 - e) + 4.0 * e * h;
        (-(1.0 - e) + disc.sqrt()) / (2.0 * e)
    }
    pub fn battery_voltage(&self) -> f32 {
        self.battery_voltage
    }
    pub fn set_battery_voltage(&mut self, v: f32) {
        self.battery_voltage = v;
    }
    pub fn command_to_pwm(&self, command: f32) -> u16 {
        if self.num_motors == 0 {
            1000
        } else {
            self.motors[0].command_to_pwm(command)
        }
    }
    pub fn set_equal_command(&self, input: &mut SitlInput, command: f32) {
        let pwm = self.command_to_pwm(command);
        for i in 0..self.num_motors as usize {
            let idx = self.motor_offset as usize + self.motors[i].servo as usize;
            if let Some(slot) = input.servos.get_mut(idx) {
                *slot = pwm;
            }
        }
    }
    pub fn get_model_batt_max_voltage(&self) -> f32 {
        self.model.max_voltage
    }
    pub fn get_current_amp(&self) -> f32 {
        self.motors().iter().map(Motor::get_current).sum()
    }
    pub fn get_model_batt_capacity_ah(&self) -> f32 {
        self.model.batt_capacity_ah
    }
    pub fn get_model_batt_resistance_ohm(&self) -> f32 {
        self.model.ref_bat_res
    }
    pub fn set_mass_scale(&mut self, scale: f32) {
        self.mass_scale = scale;
    }
    pub fn battery_changed(&mut self) -> bool {
        let ret = self.battery_dirty;
        self.battery_dirty = false;
        ret
    }
}

// Silence unused template-helper imports used only by generated motor tables.
#[allow(dead_code)]
fn _yaw_consts() -> (f32, f32) {
    (MOTORS_YAW_FACTOR_CCW, MOTORS_YAW_FACTOR_CW)
}
#[allow(dead_code)]
fn _channels() -> usize {
    SITL_SERVO_CHANNELS
}
#[allow(dead_code)]
fn _vec3_zero_check(v: Vec3) -> bool {
    vec3_is_zero(v)
}

#[cfg(test)]
mod json_tests {
    use super::*;

    #[test]
    fn json_frame_overrides_mass() {
        let dir = std::env::temp_dir();
        let path = dir.join("ardumaster_frame_model.json");
        std::fs::write(
            &path,
            r#"{"mass": 7.5, "maxVoltage": 16.8, "battCapacityAh": 5.0, "hoverThrOut": 0.4}"#,
        )
        .unwrap();
        let mut f = Frame::create_frame("x");
        let arg = format!("x:{}", path.display());
        f.init(&arg);
        assert!((f.get_mass() - 7.5).abs() < 1e-4, "mass={}", f.get_mass());
        assert!((f.get_model_batt_max_voltage() - 16.8).abs() < 1e-4);
        assert!((f.get_model_batt_capacity_ah() - 5.0).abs() < 1e-4);
        let _ = std::fs::remove_file(&path);
    }

    // COP-035: real, byte-for-byte upstream `Tools/autotest/models/
    // Callisto.json` / `Tools/autotest/models/freestyle.json`
    // (Copter-4.7.0). Rotor-side sibling of FW-050's own
    // `load_coeffs_reproduces_real_skywalker_2013_fixture`. Real upstream
    // `libraries/SITL/SIM_Frame.cpp`: `load_frame_params` at real line
    // 458, its `json_search vars[]` field table at real line 489,
    // `json_search per_motor_vars[]` at real line 531, `Frame::init`'s
    // `:model.json` suffix check + call site at real lines 580/588 --
    // all re-verified directly against the pinned upstream tree before
    // writing these tests. `Callisto.json` and `freestyle.json` are both
    // confirmed (by the upstream field names inside them) to be inputs to
    // THIS function -- not `SimPlane::load_coeffs`'s `skywalker_2013.json`
    // (aerodynamic coefficients) nor the X-Plane DREF configs
    // (`xplane_plane.json`/`xplane_heli.json`), neither of which this
    // function, or `load_coeffs`, accepts.

    #[test]
    fn load_frame_params_reproduces_real_callisto_fixture() {
        // Callisto: an 8-motor coaxial-octocopter research drone
        // (https://www.freespaceoperations.com.au/). Values below are
        // read directly from the real fixture file, not copied from the
        // ticket text.
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .and_then(|p| p.parent())
            .map(|p| p.join("fixtures/Callisto.json"))
            .expect("workspace root");
        let mut f = Frame::default();
        assert!(
            f.load_frame_params(path.to_str().expect("utf8 path")),
            "failed to load real fixture {}",
            path.display()
        );
        let m = f.get_model();
        assert!((m.mass - 32.5).abs() < 1e-4, "mass={}", m.mass);
        assert!((m.diagonal_size - 1.325).abs() < 1e-4);
        assert!((m.ref_spd - 25.0).abs() < 1e-4);
        assert!((m.ref_angle - 30.0).abs() < 1e-4);
        assert!((m.ref_voltage - 46.9).abs() < 1e-4);
        assert!((m.ref_current - 65.36).abs() < 1e-4);
        assert!((m.ref_alt - 26.0).abs() < 1e-4);
        assert!((m.ref_temp_c - 25.0).abs() < 1e-4);
        assert!((m.ref_bat_res - 0.024).abs() < 1e-5);
        assert!((m.max_voltage - 50.4).abs() < 1e-4);
        assert!((m.batt_capacity_ah - 44.0).abs() < 1e-4);
        assert!((m.prop_expo - 0.5).abs() < 1e-4);
        assert!((m.ref_rot_rate - 120.0).abs() < 1e-4);
        assert!(
            (m.hover_thr_out - 0.36).abs() < 1e-4,
            "hoverThrOut={}",
            m.hover_thr_out
        );
        assert!((m.pwm_min - 1000.0).abs() < 1e-4);
        assert!((m.pwm_max - 1940.0).abs() < 1e-4);
        assert!((m.spin_min - 0.2).abs() < 1e-4);
        assert!((m.spin_max - 0.975).abs() < 1e-4);
        assert!((m.slew_max - 75.0).abs() < 1e-4);
        assert!(
            (m.disc_area - 1.82).abs() < 1e-4,
            "disc_area={}",
            m.disc_area
        );
        assert!((m.mdrag_coef - 0.10).abs() < 1e-4);
        assert!(
            (m.num_motors - 8.0).abs() < 1e-4,
            "num_motors={}",
            m.num_motors
        );
    }

    #[test]
    fn load_frame_params_reproduces_real_freestyle_fixture() {
        // freestyle: a small 5-inch FPV racing quad. Values below are
        // read directly from the real fixture file, not copied from the
        // ticket text.
        let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .and_then(|p| p.parent())
            .map(|p| p.join("fixtures/freestyle.json"))
            .expect("workspace root");
        let mut f = Frame::default();
        assert!(
            f.load_frame_params(path.to_str().expect("utf8 path")),
            "failed to load real fixture {}",
            path.display()
        );
        let m = f.get_model();
        assert!((m.mass - 0.8).abs() < 1e-4, "mass={}", m.mass);
        assert!((m.diagonal_size - 0.25).abs() < 1e-4);
        assert!((m.ref_spd - 20.0).abs() < 1e-4);
        assert!((m.ref_angle - 45.0).abs() < 1e-4);
        assert!((m.ref_voltage - 23.2).abs() < 1e-4);
        assert!((m.ref_current - 5.0).abs() < 1e-4);
        assert!((m.ref_alt - 607.0).abs() < 1e-4);
        assert!((m.ref_temp_c - 25.0).abs() < 1e-4);
        assert!((m.ref_bat_res - 0.0226).abs() < 1e-5);
        assert!((m.max_voltage - 25.2).abs() < 1e-4);
        assert!((m.batt_capacity_ah - 0.0).abs() < 1e-4);
        assert!((m.prop_expo - 0.7).abs() < 1e-4);
        assert!((m.ref_rot_rate - 700.0).abs() < 1e-4);
        assert!(
            (m.hover_thr_out - 0.125).abs() < 1e-4,
            "hoverThrOut={}",
            m.hover_thr_out
        );
        assert!((m.pwm_min - 1000.0).abs() < 1e-4);
        assert!((m.pwm_max - 2000.0).abs() < 1e-4);
        assert!((m.spin_min - 0.01).abs() < 1e-4);
        assert!((m.spin_max - 0.95).abs() < 1e-4);
        assert!((m.slew_max - 0.0).abs() < 1e-4);
        assert!(
            (m.disc_area - 0.204).abs() < 1e-4,
            "disc_area={}",
            m.disc_area
        );
        assert!((m.mdrag_coef - 0.10).abs() < 1e-4);
        assert!(
            (m.num_motors - 4.0).abs() < 1e-4,
            "num_motors={}",
            m.num_motors
        );
    }

    #[test]
    fn callisto_and_freestyle_differ_from_each_other_and_from_default() {
        // Proves `load_frame_params` isn't silently falling through to the
        // plain hardcoded default (same discipline as VT-012/VCP-012's own
        // tailsitter/tiltrotor work): Callisto (32.5 kg, 8 motors,
        // 1.82 sq m disc, 0.36 hover throttle) and freestyle (0.8 kg,
        // 4 motors, 0.204 sq m disc, 0.125 hover throttle) are both
        // genuinely different from each other and from
        // `FrameModel::default()` (3.0 kg, 4 motors, 0.385 sq m disc,
        // 0.39 hover throttle -- the plain frame config this port's own
        // harnesses fall back to for an unrecognised/non-json frame name).
        let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .and_then(|p| p.parent())
            .map(|p| p.to_path_buf())
            .expect("workspace root");

        let mut callisto = Frame::default();
        assert!(callisto.load_frame_params(
            root.join("fixtures/Callisto.json")
                .to_str()
                .expect("utf8 path")
        ));
        let mut freestyle = Frame::default();
        assert!(freestyle.load_frame_params(
            root.join("fixtures/freestyle.json")
                .to_str()
                .expect("utf8 path")
        ));
        let default_model = FrameModel::default();
        let c = callisto.get_model();
        let fs = freestyle.get_model();

        // Genuinely different from each other.
        assert!(
            (c.mass - fs.mass).abs() > 1.0,
            "mass: {} vs {}",
            c.mass,
            fs.mass
        );
        assert!(
            (c.num_motors - fs.num_motors).abs() > 0.5,
            "num_motors: {} vs {}",
            c.num_motors,
            fs.num_motors
        );
        assert!(
            (c.disc_area - fs.disc_area).abs() > 0.5,
            "disc_area: {} vs {}",
            c.disc_area,
            fs.disc_area
        );
        assert!(
            (c.hover_thr_out - fs.hover_thr_out).abs() > 0.1,
            "hoverThrOut: {} vs {}",
            c.hover_thr_out,
            fs.hover_thr_out
        );

        // Callisto genuinely differs from the plain default on all four.
        assert!((c.mass - default_model.mass).abs() > 1.0);
        assert!((c.num_motors - default_model.num_motors).abs() > 0.5);
        assert!((c.disc_area - default_model.disc_area).abs() > 0.5);
        assert!((c.hover_thr_out - default_model.hover_thr_out).abs() > 0.01);

        // freestyle genuinely differs from the plain default on mass,
        // disc_area and hoverThrOut. Its num_motors (4) legitimately
        // coincides with the default's num_motors (4) -- both are
        // 4-motor quads -- so that field alone doesn't distinguish them;
        // the other three do.
        assert!((fs.mass - default_model.mass).abs() > 1.0);
        assert!((fs.disc_area - default_model.disc_area).abs() > 0.1);
        assert!((fs.hover_thr_out - default_model.hover_thr_out).abs() > 0.2);
    }
}
