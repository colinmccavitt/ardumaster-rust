//! Port of ArduPilot `libraries/Filter`, pinned to `Plane-4.7.0`.
//!
//! Tracked as **FW-003**. Scope is the filters the fixed-wing path actually
//! uses: `LowPassFilter` and its const-dt variant, and the two-pole
//! `LowPassFilter2p` that `AP_InertialSensor` runs every gyro and accelerometer
//! sample through. [`notch`] is the biquad; [`ap_filter`] is the COP-008 hook
//! `AC_PID::set_notch_sample_rate` uses to look one up. Harmonic notch and the
//! `AP_Filters` parameter table stay on FW-003.
//!
//! Conventions follow ADR-0004; divergences are registered in DIVERGENCES.md.

#![no_std]

pub mod ap_filter;
pub mod average;
pub mod biquad;
pub mod buffer;
pub mod derivative;
pub mod harmonic;
pub mod lowpass;
pub mod mode;
pub mod notch;
pub mod slew;
