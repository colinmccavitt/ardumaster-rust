//! Port of ArduPilot `libraries/AP_Math`, pinned to `Plane-4.7.0`.
//!
//! Tracked as **FW-002**. Verification is `unit-parity`: upstream ships 18 test
//! files under `libraries/AP_Math/tests/`, which are the oracle for this crate.
//!
//! Conventions are set by ADR-0004:
//! - Precision follows the `ekf-double` feature, mirroring upstream's global
//!   `HAL_WITH_EKF_DOUBLE` switch, rather than being generic over the scalar.
//! - Type aliases keep upstream's names (`Vector3f`, `Vector3d`) so call sites
//!   outside this crate stay diffable against the C++ line by line.
//! - Bit-exact float parity is explicitly not a goal; upstream itself builds
//!   with `-fno-signed-zeros -fno-trapping-math -fsingle-precision-constant`.

#![no_std]

pub mod crc;
/// Generated CRC lookup tables. See `tools/parity/gen_crc_tables.py`.
pub mod crc_tables;
pub mod matrix3;
pub mod quaternion;
pub mod scalar;
pub mod vector2;
pub mod vector3;

/// The chosen float type, mirroring upstream's `ftype` in `AP_Math/ftype.h`.
///
/// Upstream writes this as a `typedef` switched on `HAL_WITH_EKF_DOUBLE`, with
/// capital-F macro wrappers (`sinF`, `sqrtF`, ...) selecting the matching libm
/// entry point. Here the alias carries the choice and callers use [`Ftype`]
/// methods directly.
#[cfg(feature = "ekf-double")]
pub type Ftype = f64;

/// The chosen float type, mirroring upstream's `ftype` in `AP_Math/ftype.h`.
#[cfg(not(feature = "ekf-double"))]
pub type Ftype = f32;

/// Whether this build selected double precision. Mirrors `HAL_WITH_EKF_DOUBLE`.
pub const EKF_DOUBLE: bool = cfg!(feature = "ekf-double");

#[cfg(test)]
mod tests {
    use super::*;

    /// Guards ADR-0004 decision 3: precision is a build-wide choice, so exactly
    /// one of the two `Ftype` definitions may ever be active.
    #[test]
    fn ftype_matches_feature() {
        if EKF_DOUBLE {
            assert_eq!(core::mem::size_of::<Ftype>(), 8);
        } else {
            assert_eq!(core::mem::size_of::<Ftype>(), 4);
        }
    }
}
