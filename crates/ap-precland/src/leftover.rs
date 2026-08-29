//! Remaining `AC_PrecLand` leftovers after `init` + `update` +
//! `handle_msg` + the estimator frontend + `PosVelEKF` + output
//! prediction and the getters / target-status leftover + the Backend
//! / MAVLink sensor path.
//!
//! Tracked as **COP-028**. [`PrecLand::init`](crate::PrecLand::init),
//! [`PrecLand::update`](crate::PrecLand::update),
//! [`PrecLand::handle_msg`](crate::PrecLand::handle_msg),
//! [`PrecLand::run_estimator`](crate::PrecLand::run_estimator),
//! [`PrecLand::check_ekf_init_timeout`](crate::PrecLand::check_ekf_init_timeout),
//! [`PrecLand::construct_pos_meas_using_rangefinder`](crate::PrecLand::construct_pos_meas_using_rangefinder),
//! [`PrecLand::retrieve_los_meas`](crate::PrecLand::retrieve_los_meas),
//! [`PosVelEKF`](crate::PosVelEKF),
//! [`PrecLand::run_output_prediction`](crate::PrecLand::run_output_prediction),
//! [`Backend`](crate::Backend), and [`MavlinkBackend`](crate::MavlinkBackend)
//! are the contiguous leftovers so far. Everything listed here is
//! still later.

/// Remaining upstream symbols this crate has not ported yet.
///
/// Weights are the ticket's 1,133 LOC (`AC_PrecLand.cpp` + `.h`). The
/// Backend getters and MAVLink `LANDING_TARGET` path write `_los_meas`.
/// Logging, the inertial ring, IRLock / SITL drivers, and the retry
/// state machine stay here.
pub const REMAINING: &[&str] = &[
    "AC_PrecLand::Write_Precland",
    // inertial history consumed by update / estimator / prediction
    "inertial_data_frame_s",
    "ObjectArray<inertial_data_frame_s>",
    // remaining backends (IRLock / SITL talk to forbidden drivers)
    "AC_PrecLand_IRLock::init(irlock)",
    "AC_PrecLand_IRLock::update",
    "AC_PrecLand_SITL::init(AP::sitl)",
    "AC_PrecLand_SITL::update",
    "AC_PrecLand_SITL_Gazebo::init(irlock)",
    "AC_PrecLand_SITL_Gazebo::update",
    // retry machine
    "AC_PrecLand_StateMachine::init",
    "AC_PrecLand_StateMachine::update",
    "AC_PrecLand_StateMachine::get_target_lost_actions",
    "AC_PrecLand_StateMachine::retry_landing",
    "AC_PrecLand_StateMachine::get_failsafe_actions",
];
