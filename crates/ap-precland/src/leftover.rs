//! Remaining `AC_PrecLand` leftovers after the `init` slice.
//!
//! Tracked as **COP-028**. [`PrecLand::init`](crate::PrecLand::init) is
//! the first contiguous leftover. Everything listed here is still later.

/// Remaining upstream symbols this crate has not ported yet.
///
/// Weights are the ticket's 1,133 LOC (`AC_PrecLand.cpp` + `.h`). The
/// `init` leftover is the constructor follow-on plus backend dispatch;
/// the rest of the frontend, both EKFs, the four sensor `update` paths,
/// and the retry state machine stay here.
pub const REMAINING: &[&str] = &[
    // AC_PrecLand.cpp frontend after init
    "AC_PrecLand::update",
    "AC_PrecLand::handle_msg",
    "AC_PrecLand::get_target_position_m",
    "AC_PrecLand::get_target_position_measurement_NED_m",
    "AC_PrecLand::get_target_position_relative_NE_m",
    "AC_PrecLand::get_target_velocity_relative_NE_ms",
    "AC_PrecLand::get_target_velocity_ms",
    "AC_PrecLand::get_target_velocity",
    "AC_PrecLand::target_acquired",
    "AC_PrecLand::get_target_location",
    "AC_PrecLand::check_target_status",
    "AC_PrecLand::check_if_sensor_in_range",
    "AC_PrecLand::check_ekf_init_timeout",
    "AC_PrecLand::run_estimator",
    "AC_PrecLand::construct_pos_meas_using_rangefinder",
    "AC_PrecLand::retrieve_los_meas",
    "AC_PrecLand::run_output_prediction",
    "AC_PrecLand::Write_Precland",
    // inertial history consumed by update / estimator
    "inertial_data_frame_s",
    "ObjectArray<inertial_data_frame_s>",
    // backends
    "AC_PrecLand_Backend::update",
    "AC_PrecLand_Backend::handle_msg",
    "AC_PrecLand_MAVLink::update",
    "AC_PrecLand_MAVLink::handle_msg",
    "AC_PrecLand_IRLock::init(irlock)",
    "AC_PrecLand_IRLock::update",
    "AC_PrecLand_SITL::init(AP::sitl)",
    "AC_PrecLand_SITL::update",
    "AC_PrecLand_SITL_Gazebo::init(irlock)",
    "AC_PrecLand_SITL_Gazebo::update",
    // EKF + retry machine
    "PosVelEKF",
    "AC_PrecLand_StateMachine::init",
    "AC_PrecLand_StateMachine::update",
    "AC_PrecLand_StateMachine::get_target_lost_actions",
    "AC_PrecLand_StateMachine::retry_landing",
    "AC_PrecLand_StateMachine::get_failsafe_actions",
];
