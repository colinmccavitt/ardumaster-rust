//! Remaining `AC_PrecLand` leftovers.
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
//! [`Backend`](crate::Backend), [`MavlinkBackend`](crate::MavlinkBackend),
//! [`IrlockBackend`](crate::IrlockBackend),
//! [`SitlBackend`](crate::SitlBackend),
//! [`PrecLand::write_precland`](crate::PrecLand::write_precland),
//! [`InertialHistory`](crate::InertialHistory), and
//! [`StateMachine`](crate::StateMachine) are the contiguous leftovers.
//! IRLock / SITL driver `init` is recorded on
//! [`crate::InitLeftover`] (`irlock_bus` / `need_sitl`) because ADR-0004
//! forbids `AP_IRLock` and `AP::sitl()`.

/// Remaining upstream symbols this crate has not ported yet.
///
/// Empty: the retry state machine and the driver-`init` leftover
/// records close COP-028. `GCS_SEND_TEXT` / `WriteBlock` stay vehicle
/// leftovers, not catalog entries.
pub const REMAINING: &[&str] = &[];
