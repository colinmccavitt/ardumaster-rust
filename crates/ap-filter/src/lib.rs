//! Port of ArduPilot `libraries/Filter`, pinned to `Plane-4.7.0`.
//!
//! Tracked as **FW-003**. Scope is the filters the fixed-wing path actually
//! uses: `LowPassFilter` and its const-dt variant first. Notch and harmonic
//! notch filters are gyro-side (`AP_InertialSensor`) and are deferred.
//!
//! Conventions follow ADR-0004; divergences are registered in DIVERGENCES.md.

#![no_std]

pub mod average;
pub mod buffer;
pub mod lowpass;
pub mod mode;
