//! Port of ArduPilot `libraries/Filter`, pinned to `Plane-4.7.0`.
//!
//! Tracked as **FW-003**. Scope is the filters the fixed-wing path actually
//! uses: `LowPassFilter` and its const-dt variant, and the two-pole
//! `LowPassFilter2p` that `AP_InertialSensor` runs every gyro and accelerometer
//! sample through. Notch and harmonic notch filters are still deferred.
//!
//! Conventions follow ADR-0004; divergences are registered in DIVERGENCES.md.

#![no_std]

pub mod average;
pub mod biquad;
pub mod buffer;
pub mod derivative;
pub mod harmonic;
pub mod lowpass;
pub mod mode;
pub mod notch;
pub mod slew;
