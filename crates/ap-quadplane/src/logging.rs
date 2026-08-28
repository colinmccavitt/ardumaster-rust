//! QuadPlane leftover logging, upstream `QuadPlane::Log_Write_QControl_Tuning`
//! / `log_QPOS` / `Log_Write_AttRate` (Plane-4.7.0 `quadplane.cpp`).
//!
//! Tracked as **VT-001**. This is the leftover logger surface: pack a
//! QTUN block (assist bitmask + 25 Hz `update()` gate), stream QPOS
//! (state-change and 40 ms period), and record ANG/RATE via
//! [`QuadPlane::log_write_att_rate`]. It does not rewrite
//! [`crate::auto_vtol`], [`crate::poscontrol`], [`crate::air_mode`],
//! or the leftover catalog helpers in [`crate::quadplane_completeness`].

use crate::poscontrol::PositionControlState;
use crate::quadplane_completeness::{
    qtun_assist_flags, QTUN_ASSIST_ALT, QTUN_ASSIST_ANGLE, QTUN_ASSIST_FORCED,
    QTUN_ASSIST_FW_FORCE, QTUN_ASSIST_IN_ASSISTED_FLIGHT, QTUN_ASSIST_SPEED,
    QTUN_ASSIST_SPIN_RECOVERY, QTUN_PERIOD_MS,
};
use crate::QuadPlane;

/// QPOS write period inside `vtol_position_controller`, upstream `>= 40`.
pub const QPOS_PERIOD_MS: u32 = 40;

/// QTUN keep-alive after motors go quiet, upstream `< 250` ms.
pub const QTUN_ACTIVE_HOLD_MS: u32 = 250;

/// MOTB period in `update()`, upstream `> 100` ms (10 Hz).
pub const MOTB_PERIOD_MS: u32 = 100;

/// `log_QControl_Tuning` / `LOG_QTUN_MSG` payload (after TimeUS).
///
/// Field order matches upstream `struct log_QControl_Tuning`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QControlTuning {
    /// `attitude_control->get_throttle_in()`.
    pub throttle_in: f32,
    /// `attitude_control->angle_boost()`.
    pub angle_boost: f32,
    /// `motors->get_throttle()`.
    pub throttle_out: f32,
    /// `motors->get_throttle_hover()`.
    pub throttle_hover: f32,
    /// `pos_control->get_pos_desired_U_m()`, or 0 in QSTABILIZE.
    pub desired_alt: f32,
    /// `inertial_nav.get_position_z_up_cm() * 0.01`.
    pub inav_alt: f32,
    /// `plane.barometer.get_altitude() * 100`.
    pub baro_alt: i32,
    /// `pos_control->get_vel_target_U_ms() * 100`, or 0 in QSTABILIZE.
    pub target_climb_rate: i16,
    /// `inertial_nav.get_velocity_z_up_cms()`.
    pub climb_rate: i16,
    /// `attitude_control->get_throttle_mix()`.
    pub throttle_mix: f32,
    /// `transition->get_log_transition_state()`.
    pub transition_state: u8,
    /// Packed `log_assistance_flags`.
    pub assist: u8,
}

impl QControlTuning {
    /// Zeroed QTUN row.
    #[must_use]
    pub const fn empty() -> Self {
        Self {
            throttle_in: 0.0,
            angle_boost: 0.0,
            throttle_out: 0.0,
            throttle_hover: 0.0,
            desired_alt: 0.0,
            inav_alt: 0.0,
            baro_alt: 0,
            target_climb_rate: 0,
            climb_rate: 0,
            throttle_mix: 0.0,
            transition_state: 0,
            assist: 0,
        }
    }
}

/// `log_QPOS` / `WriteStreaming("QPOS", ...)` payload (after TimeUS).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QPos {
    /// `poscontrol.get_state()`.
    pub state: PositionControlState,
    /// `plane.auto_state.wp_distance`.
    pub dist: f32,
    /// `poscontrol.target_speed_ms`.
    pub target_speed_ms: f32,
    /// `poscontrol.target_accel_mss`.
    pub target_accel_mss: f32,
    /// `poscontrol.overshoot`.
    pub overshoot: bool,
}

impl QPos {
    /// Empty QPOS row, `QPOS_NONE`.
    #[must_use]
    pub const fn empty() -> Self {
        Self {
            state: PositionControlState::None,
            dist: 0.0,
            target_speed_ms: 0.0,
            target_accel_mss: 0.0,
            overshoot: false,
        }
    }
}

/// Inputs [`QuadPlane::log_write_qcontrol_tuning`] reads from Plane / COP.
///
/// This crate does not own `AC_AttitudeControl`, `AP_Motors`, or the
/// logger backend.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QTunView {
    /// `plane.control_mode == mode_qstabilize`.
    pub qstabilize: bool,
    /// `attitude_control->get_throttle_in()`.
    pub throttle_in: f32,
    /// `attitude_control->angle_boost()`.
    pub angle_boost: f32,
    /// `motors->get_throttle()`.
    pub throttle_out: f32,
    /// `motors->get_throttle_hover()`.
    pub throttle_hover: f32,
    /// `pos_control->get_pos_desired_U_m()`.
    pub desired_alt_m: f32,
    /// Inertial height, metres.
    pub inav_alt_m: f32,
    /// Baro altitude, centimetres.
    pub baro_alt_cm: i32,
    /// `pos_control->get_vel_target_U_ms()`.
    pub target_climb_rate_ms: f32,
    /// `inertial_nav.get_velocity_z_up_cms()`.
    pub climb_rate_cms: i16,
    /// `attitude_control->get_throttle_mix()`.
    pub throttle_mix: f32,
    /// `transition->get_log_transition_state()`.
    pub transition_state: u8,
    /// `assist.in_force_assist()`.
    pub force_assist: bool,
    /// `assist.in_speed_assist()`.
    pub speed_assist: bool,
    /// `assist.in_alt_assist()`.
    pub alt_assist: bool,
    /// `assist.in_angle_assist()`.
    pub angle_assist: bool,
    /// Leftover `force_fw_control_recovery` QTUN bit.
    pub fw_force_recovery: bool,
    /// Leftover `in_spin_recovery` QTUN bit.
    pub spin_recovery: bool,
}

impl QTunView {
    /// Hovering VTOL sample (not QSTABILIZE), no assist bits.
    #[must_use]
    pub const fn hover() -> Self {
        Self {
            qstabilize: false,
            throttle_in: 0.4,
            angle_boost: 0.0,
            throttle_out: 0.45,
            throttle_hover: 0.35,
            desired_alt_m: 10.0,
            inav_alt_m: 9.5,
            baro_alt_cm: 950,
            target_climb_rate_ms: 0.5,
            climb_rate_cms: 40,
            throttle_mix: 0.5,
            transition_state: 2,
            force_assist: false,
            speed_assist: false,
            alt_assist: false,
            angle_assist: false,
            fw_force_recovery: false,
            spin_recovery: false,
        }
    }

    /// QSTABILIZE sample — desired alt / climb target stay zero.
    #[must_use]
    pub const fn qstabilize() -> Self {
        let mut view = Self::hover();
        view.qstabilize = true;
        view
    }
}

/// Inputs [`QuadPlane::log_qpos`] reads from Plane / poscontrol extras.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QPosView {
    /// `plane.auto_state.wp_distance`.
    pub wp_distance: f32,
    /// `poscontrol.target_speed_ms`.
    pub target_speed_ms: f32,
    /// `poscontrol.target_accel_mss`.
    pub target_accel_mss: f32,
    /// `poscontrol.overshoot`.
    pub overshoot: bool,
}

impl QPosView {
    /// Approach sample, no overshoot.
    #[must_use]
    pub const fn approach(wp_distance: f32, target_speed_ms: f32) -> Self {
        Self {
            wp_distance,
            target_speed_ms,
            target_accel_mss: 0.0,
            overshoot: false,
        }
    }
}

/// Inputs the leftover `update()` logging gate reads.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LogUpdateView {
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
    /// `in_vtol_mode()`.
    pub in_vtol_mode: bool,
    /// `motors->get_spool_state() == SHUT_DOWN`.
    pub spool_shutdown: bool,
    /// `show_vtol_view()`.
    pub show_vtol_view: bool,
    /// `plane.g2.systemid.is_running()`.
    pub sysid_running: bool,
    /// `last_motors_active_ms` (owned by motors_output leftover).
    pub last_motors_active_ms: u32,
    /// QTUN field snapshot for a write this tick.
    pub qtun: QTunView,
}

impl LogUpdateView {
    /// Armed VTOL hover at `now_ms`, motors recently active.
    #[must_use]
    pub const fn vtol_hover(now_ms: u32) -> Self {
        Self {
            now_ms,
            in_vtol_mode: true,
            spool_shutdown: false,
            show_vtol_view: true,
            sysid_running: false,
            last_motors_active_ms: now_ms,
            qtun: QTunView::hover(),
        }
    }
}

/// Side-effects of [`QuadPlane::maybe_log_update`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LogUpdateResult {
    /// `attitude_control->Write_ANG()` (loop rate, `show_vtol_view`).
    pub wrote_ang: bool,
    /// `attitude_control->Write_Rate(*pos_control)`.
    pub wrote_rate: bool,
    /// `Log_Write_QControl_Tuning()` this tick.
    pub wrote_qtun: bool,
}

/// Leftover logger block, upstream `last_qtun_log_ms` plus last packets.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QLogging {
    last_qtun_log_ms: u32,
    last_qpos_log_ms: u32,
    qtun_writes: u32,
    qpos_writes: u32,
    att_rate_writes: u32,
    last_qtun: QControlTuning,
    last_qpos: QPos,
}

impl Default for QLogging {
    fn default() -> Self {
        Self::new()
    }
}

impl QLogging {
    /// Empty logger block — no writes yet.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            last_qtun_log_ms: 0,
            last_qpos_log_ms: 0,
            qtun_writes: 0,
            qpos_writes: 0,
            att_rate_writes: 0,
            last_qtun: QControlTuning::empty(),
            last_qpos: QPos::empty(),
        }
    }

    /// `last_qtun_log_ms`.
    #[must_use]
    pub const fn last_qtun_log_ms(&self) -> u32 {
        self.last_qtun_log_ms
    }

    /// `poscontrol.last_log_ms` equivalent for the QPOS period gate.
    #[must_use]
    pub const fn last_qpos_log_ms(&self) -> u32 {
        self.last_qpos_log_ms
    }

    /// How many QTUN blocks have been packed.
    #[must_use]
    pub const fn qtun_writes(&self) -> u32 {
        self.qtun_writes
    }

    /// How many QPOS streams have been packed.
    #[must_use]
    pub const fn qpos_writes(&self) -> u32 {
        self.qpos_writes
    }

    /// How many `Log_Write_AttRate` calls ran.
    #[must_use]
    pub const fn att_rate_writes(&self) -> u32 {
        self.att_rate_writes
    }

    /// Last packed QTUN row.
    #[must_use]
    pub const fn last_qtun(&self) -> QControlTuning {
        self.last_qtun
    }

    /// Last packed QPOS row.
    #[must_use]
    pub const fn last_qpos(&self) -> QPos {
        self.last_qpos
    }

    /// Test poke for the QTUN 25 Hz gate.
    pub fn set_last_qtun_log_ms(&mut self, last_qtun_log_ms: u32) {
        self.last_qtun_log_ms = last_qtun_log_ms;
    }

    /// Test poke for the QPOS 40 ms gate.
    pub fn set_last_qpos_log_ms(&mut self, last_qpos_log_ms: u32) {
        self.last_qpos_log_ms = last_qpos_log_ms;
    }
}

/// Pack leftover QTUN `assist` from the flight latch plus the view.
///
/// Upstream `Log_Write_QControl_Tuning` `log_assistance_flags`.
#[must_use]
pub const fn pack_qtun_assist(assisted_flight: bool, view: QTunView) -> u8 {
    qtun_assist_flags(
        assisted_flight,
        view.force_assist,
        view.speed_assist,
        view.alt_assist,
        view.angle_assist,
        view.fw_force_recovery,
        view.spin_recovery,
    )
}

/// Assemble a QTUN row. QSTABILIZE zeros desired alt / climb target.
#[must_use]
pub const fn assemble_qtun(assisted_flight: bool, view: QTunView) -> QControlTuning {
    let (desired_alt, target_climb_rate) = if view.qstabilize {
        (0.0, 0)
    } else {
        (
            view.desired_alt_m,
            (view.target_climb_rate_ms * 100.0) as i16,
        )
    };
    QControlTuning {
        throttle_in: view.throttle_in,
        angle_boost: view.angle_boost,
        throttle_out: view.throttle_out,
        throttle_hover: view.throttle_hover,
        desired_alt,
        inav_alt: view.inav_alt_m,
        baro_alt: view.baro_alt_cm,
        target_climb_rate,
        climb_rate: view.climb_rate_cms,
        throttle_mix: view.throttle_mix,
        transition_state: view.transition_state,
        assist: pack_qtun_assist(assisted_flight, view),
    }
}

/// `now - last_qtun_log_ms > 40` — the `update()` QTUN gate.
#[must_use]
pub const fn qtun_period_elapsed(now_ms: u32, last_qtun_log_ms: u32) -> bool {
    now_ms.wrapping_sub(last_qtun_log_ms) > QTUN_PERIOD_MS
}

/// `now - last_log_ms >= 40` — the `vtol_position_controller` QPOS gate.
#[must_use]
pub const fn qpos_period_elapsed(now_ms: u32, last_qpos_log_ms: u32) -> bool {
    now_ms.wrapping_sub(last_qpos_log_ms) >= QPOS_PERIOD_MS
}

/// QTUN keep-alive: motors active, or active within 250 ms.
#[must_use]
pub const fn qtun_motors_recent(
    now_ms: u32,
    motors_active: bool,
    last_motors_active_ms: u32,
) -> bool {
    motors_active || now_ms.wrapping_sub(last_motors_active_ms) < QTUN_ACTIVE_HOLD_MS
}

impl QuadPlane {
    /// Leftover logger block.
    #[must_use]
    pub const fn logging(&self) -> &QLogging {
        &self.logging
    }

    /// Mutable leftover logger block (period poke / tests).
    pub fn logging_mut(&mut self) -> &mut QLogging {
        &mut self.logging
    }

    /// Upstream `QuadPlane::Log_Write_QControl_Tuning`.
    ///
    /// Packs QTUN (QSTABILIZE zeros desired alt / climb) and records
    /// the write. The COP `pos_control->write_log()` / tiltrotor log
    /// follow-ups are later slices.
    pub fn log_write_qcontrol_tuning(&mut self, view: QTunView) -> QControlTuning {
        let pkt = assemble_qtun(self.assisted_flight, view);
        self.logging.last_qtun = pkt;
        self.logging.qtun_writes = self.logging.qtun_writes.saturating_add(1);
        pkt
    }

    /// Upstream `QuadPlane::log_QPOS`.
    ///
    /// `WriteStreaming("QPOS", TimeUS, State, Dist, TSpd, TAcc, OShoot)`.
    pub fn log_qpos(&mut self, view: QPosView) -> QPos {
        let pkt = QPos {
            state: self.poscontrol.state(),
            dist: view.wp_distance,
            target_speed_ms: view.target_speed_ms,
            target_accel_mss: view.target_accel_mss,
            overshoot: view.overshoot,
        };
        self.logging.last_qpos = pkt;
        self.logging.qpos_writes = self.logging.qpos_writes.saturating_add(1);
        pkt
    }

    /// Upstream `vtol_position_controller` QPOS period (`>= 40` ms).
    ///
    /// State-change double-log lives on `PosControlState::set_state`
    /// and is a later poscontrol slice.
    pub fn maybe_log_qpos(&mut self, now_ms: u32, view: QPosView) -> bool {
        if !qpos_period_elapsed(now_ms, self.logging.last_qpos_log_ms) {
            return false;
        }
        self.logging.last_qpos_log_ms = now_ms;
        let _ = self.log_qpos(view);
        true
    }

    /// Upstream `QuadPlane::Log_Write_AttRate`.
    ///
    /// `attitude_control->Write_ANG()` then `Write_Rate(*pos_control)`.
    pub fn log_write_att_rate(&mut self) {
        self.logging.att_rate_writes = self.logging.att_rate_writes.saturating_add(1);
    }

    /// Leftover `update()` logging gate (armed / VTOL / 25 Hz QTUN).
    ///
    /// ANG+RATE run at loop rate while motors are active and not shut
    /// down (ANG also needs `show_vtol_view`; both skip when sysid is
    /// running). QTUN writes at `> 40` ms while motors are active or
    /// were active in the last 250 ms.
    pub fn maybe_log_update(&mut self, view: LogUpdateView) -> LogUpdateResult {
        if !self.motors_armed {
            return LogUpdateResult {
                wrote_ang: false,
                wrote_rate: false,
                wrote_qtun: false,
            };
        }
        let motors_active = view.in_vtol_mode || self.assisted_flight;
        let mut wrote_ang = false;
        let mut wrote_rate = false;
        if motors_active && !view.spool_shutdown && !view.sysid_running {
            wrote_ang = view.show_vtol_view;
            wrote_rate = true;
            if wrote_ang || wrote_rate {
                self.log_write_att_rate();
            }
        }
        let wrote_qtun = qtun_motors_recent(view.now_ms, motors_active, view.last_motors_active_ms)
            && qtun_period_elapsed(view.now_ms, self.logging.last_qtun_log_ms);
        if wrote_qtun {
            self.logging.last_qtun_log_ms = view.now_ms;
            let _ = self.log_write_qcontrol_tuning(view.qtun);
        }
        LogUpdateResult {
            wrote_ang,
            wrote_rate,
            wrote_qtun,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn enabled() -> QuadPlane {
        let mut qp = QuadPlane::with_enable(1);
        assert!(qp.setup());
        qp
    }

    #[test]
    fn qtun_qstabilize_zeros_desired_alt_and_climb() {
        let mut qp = enabled();
        let pkt = qp.log_write_qcontrol_tuning(QTunView::qstabilize());
        assert_eq!(pkt.desired_alt, 0.0);
        assert_eq!(pkt.target_climb_rate, 0);
        assert_eq!(
            pkt.throttle_out as i32,
            QTunView::hover().throttle_out as i32
        );
        assert_eq!(qp.logging().qtun_writes(), 1);
    }

    #[test]
    fn qtun_assist_packs_flight_and_leftover_bits() {
        let mut qp = enabled();
        qp.set_assisted_flight(true);
        let mut view = QTunView::hover();
        view.fw_force_recovery = true;
        view.spin_recovery = true;
        view.speed_assist = true;
        let pkt = qp.log_write_qcontrol_tuning(view);
        assert_eq!(
            pkt.assist,
            QTUN_ASSIST_IN_ASSISTED_FLIGHT
                | QTUN_ASSIST_SPEED
                | QTUN_ASSIST_FW_FORCE
                | QTUN_ASSIST_SPIN_RECOVERY
        );
        assert_eq!(pkt.target_climb_rate, 50);
        assert_eq!(pkt.desired_alt as i32, 10);
        let _ = QTUN_ASSIST_FORCED;
        let _ = QTUN_ASSIST_ALT;
        let _ = QTUN_ASSIST_ANGLE;
    }

    #[test]
    fn qpos_streams_state_and_period_gate() {
        let mut qp = enabled();
        qp.poscontrol_mut()
            .set_state(PositionControlState::Position2);
        let pkt = qp.log_qpos(QPosView::approach(12.0, 8.0));
        assert_eq!(pkt.state, PositionControlState::Position2);
        assert_eq!(pkt.dist as i32, 12);
        assert_eq!(qp.logging().qpos_writes(), 1);

        assert!(qp.maybe_log_qpos(40, QPosView::approach(11.0, 7.0)));
        assert!(!qp.maybe_log_qpos(79, QPosView::approach(10.0, 6.0)));
        assert!(qp.maybe_log_qpos(80, QPosView::approach(9.0, 5.0)));
        assert_eq!(qp.logging().qpos_writes(), 3);
        assert_eq!(qp.logging().last_qpos().dist as i32, 9);
    }

    #[test]
    fn update_gate_needs_armed_and_qtun_period() {
        let mut qp = enabled();
        let idle = qp.maybe_log_update(LogUpdateView::vtol_hover(100));
        assert!(!idle.wrote_qtun);
        assert!(!idle.wrote_ang);

        qp.set_motors_armed(true);
        let r = qp.maybe_log_update(LogUpdateView::vtol_hover(100));
        assert!(r.wrote_ang);
        assert!(r.wrote_rate);
        assert!(r.wrote_qtun);
        assert_eq!(qp.logging().qtun_writes(), 1);
        assert_eq!(qp.logging().att_rate_writes(), 1);

        let too_soon = qp.maybe_log_update(LogUpdateView::vtol_hover(130));
        assert!(too_soon.wrote_ang);
        assert!(!too_soon.wrote_qtun);

        let next = qp.maybe_log_update(LogUpdateView::vtol_hover(141));
        assert!(next.wrote_qtun);
        assert_eq!(qp.logging().last_qtun_log_ms(), 141);
    }
}
