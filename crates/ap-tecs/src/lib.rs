//! Port of ArduPilot `libraries/AP_TECS`, pinned to `Plane-4.7.0`.
//!
//! Total Energy Control System: the fixed-wing coordinator that trades kinetic
//! against potential energy to produce throttle and pitch demands.
//!
//! Tracked as **FW-015**. Verification is log replay (ADR-0008) against
//! `fixtures/tecs_replay.csv`, which holds 1,914 records of real flight -
//! every `update_pitch_throttle` argument paired with upstream own throttle
//! and pitch outputs at the same instant.

#![no_std]

pub mod params;
