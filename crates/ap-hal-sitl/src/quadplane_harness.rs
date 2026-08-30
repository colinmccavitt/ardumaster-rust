//! VT-011: SitlQuadPlaneHarness — QuadPlane analogue of SitlHarness.
//!
//! RC/sensors from SimQuadPlane truth -> Plane tick + QuadPlane::update /
//! QHover / hold_hover leftover -> FW servos + leftover VTOL MotorsMatrix
//! at Frame::motor_offset into original-source SimQuadPlane (VT-010).
//!
//! Matches C++ `sitl_quadplane_harness.hpp` / `sitl/quadplane_main.cpp`
//! (VCP-011). Plane tick does not own QuadPlane (same disclosed gap);
//! this harness ticks both in the SITL HAL role. VTOL PWM is leftover
//! MotorsMatrix Quad-X (same leftover_apply_collective path as copter),
//! not a new mixer.

#![allow(missing_docs)]

use ap_motors::armed::{output_armed_stabilizing, ArmedDemand};
use ap_motors::MotorMatrix;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::ModeNumber;
use ap_quadplane::mode_q::{
    qhover_enter, qhover_run, QHoverEnterState, QHoverEnterView, QManualRun, QManualRunAction,
    QManualRunView,
};
use ap_quadplane::motors_output::{
    DesiredSpoolState, MotorsOutputAction, MotorsOutputTick, MotorsOutputView,
};
use ap_quadplane::transition_fsm::{SltTransition, TransitionPhase};
use ap_quadplane::QuadPlane;
use ap_sim::sim_motor::SitlInput;
use ap_sim::sim_quadplane::SimQuadPlane;

use crate::harness::{SitlHarness, SERVO_MAX};

/// Leftover MotorsMatrix Quad-X into `SitlInput` at `Frame::motor_offset`.
///
/// C++ `leftover_apply_vtol_motors`: attitude P on roll/pitch, rate P on
/// yaw, command as throttle. Does not overwrite FW servos 0-3.
pub fn leftover_apply_vtol_motors(
    input: &mut SitlInput,
    sim: &SimQuadPlane,
    mixer: &mut MotorMatrix,
    mixer_inited: &mut bool,
    command: f32,
    armed: bool,
) {
    if !*mixer_inited {
        let ok = mixer.setup_motors(1, 1);
        debug_assert!(ok, "QUAD X");
        *mixer_inited = true;
    }
    if !armed {
        return;
    }
    let (roll, pitch, _yaw) = sim.plane.dcm.to_euler();
    let demand = ArmedDemand {
        roll: (-0.5 * roll).clamp(-1.0, 1.0),
        pitch: (-0.5 * pitch).clamp(-1.0, 1.0),
        yaw: (-0.2 * sim.plane.gyro.z).clamp(-1.0, 1.0),
        throttle: command,
        throttle_avg_max: command,
        throttle_thrust_max: 1.0,
        compensation_gain: 1.0,
        yaw_headroom: 200,
        thrust_boost: false,
        thrust_boost_ratio: 0.0,
        motor_lost_index: 0,
    };
    let out = output_armed_stabilizing(mixer, &demand);
    let n = sim.frame.num_motors as usize;
    let offset = sim.frame.motor_offset as usize;
    for i in 0..n {
        let pwm = (1000.0 + out.get_thrust_rpyt_out(i as u8) * 1000.0)
            .clamp(1000.0, 2000.0)
            .round() as u16;
        let servo = sim.frame.motors()[i].servo as usize;
        if let Some(slot) = input.servos.get_mut(offset + servo) {
            *slot = pwm;
        }
    }
}

fn fw_servos_from_plane(plane: &PlaneMainLoop) -> SitlInput {
    let aileron = (plane.servos.aileron_scaled / SERVO_MAX).clamp(-1.0, 1.0);
    let elevator = (plane.stabilize_servos.elevator_scaled / SERVO_MAX).clamp(-1.0, 1.0);
    let rudder = (plane.servos.rudder_scaled / SERVO_MAX).clamp(-1.0, 1.0);
    let throttle = (plane.servos.throttle_scaled / 100.0).clamp(0.0, 1.0);
    let mut input = SitlInput::default();
    input.servos[0] = (1500.0 + aileron * 500.0).clamp(1000.0, 2000.0) as u16;
    input.servos[1] = (1500.0 + elevator * 500.0).clamp(1000.0, 2000.0) as u16;
    input.servos[2] = (1000.0 + throttle * 1000.0).clamp(1000.0, 2000.0) as u16;
    input.servos[3] = (1500.0 + rudder * 500.0).clamp(1000.0, 2000.0) as u16;
    input
}

/// FBWA + armed Plane, QuadPlane setup + QHover enter.
///
/// C++ quadplane_main: `QuadPlane qp{1}; qp.setup(); qp.mode_enter();
/// qhover_enter(); ModeFBWA; armed; airspeed calibrate`.
pub fn setup_hover_transition(plane: &mut PlaneMainLoop, qp: &mut QuadPlane) {
    SitlHarness::configure_vehicle(plane);
    plane.mode.control_mode = ModeNumber::FlyByWireA.as_number();
    plane.soft_armed = true;
    plane.airspeed_calibrate_requested = true;
    let _ = qp.setup();
    let mut enter = QHoverEnterState::new();
    let _ = qhover_enter(qp, QHoverEnterView::parked_idle(), &mut enter);
}

/// Reusable QuadPlane SITL closed-loop driver. Does not own Plane /
/// QuadPlane / SimQuadPlane — same as C++ SitlQuadPlaneHarness.
pub struct SitlQuadPlaneHarness {
    plane_harness: SitlHarness,
    mixer: MotorMatrix,
    mixer_inited: bool,
    slt: SltTransition,
    last_qhover: QManualRun,
    last_motors: MotorsOutputTick,
    last_input: SitlInput,
    tick_count: u32,
}

impl Default for SitlQuadPlaneHarness {
    fn default() -> Self {
        Self::new()
    }
}

impl SitlQuadPlaneHarness {
    pub fn new() -> Self {
        Self {
            plane_harness: SitlHarness::new(),
            mixer: MotorMatrix::new(),
            mixer_inited: false,
            slt: SltTransition::new(),
            last_qhover: qhover_run(&QuadPlane::with_enable(1), &QManualRunView::flying()),
            last_motors: MotorsOutputTick {
                action: MotorsOutputAction::Disarmed,
                desired_spool: DesiredSpoolState::ShutDown,
                motors_output_ran: false,
                rate_controller_ran: false,
                attitude_relaxed: false,
                motors_inactive: true,
            },
            last_input: SitlInput::default(),
            tick_count: 0,
        }
    }

    pub fn tick_count(&self) -> u32 {
        self.tick_count
    }

    pub fn last_input(&self) -> &SitlInput {
        &self.last_input
    }

    pub fn last_qhover(&self) -> QManualRun {
        self.last_qhover
    }

    pub fn last_motors(&self) -> MotorsOutputTick {
        self.last_motors
    }

    /// Leftover `QuadPlane::update` (C++ `run_quadplane_update`).
    fn leftover_quadplane_update(
        &mut self,
        qp: &QuadPlane,
        now_ms: u32,
        armed: bool,
        in_vtol_mode: bool,
        airspeed_ms: f32,
    ) {
        if !qp.available() {
            return;
        }
        self.slt.set_q_options(qp.options());
        self.slt.reset_fail_timer_if_disarmed(now_ms, armed);
        let assist = qp.assisted_flight();
        let run_transition = !in_vtol_mode || self.slt.in_transition();
        if run_transition {
            self.slt
                .update_forward_timing(now_ms, true, airspeed_ms, 0.0, assist, false);
            let _ = self.slt.apply_transition_fail(now_ms, false);
        }
        let _phase = self.slt.phase(in_vtol_mode);
        let _ = matches!(_phase, TransitionPhase::Transition);
    }

    /// One closed-loop tick. Caller must have called [`crate::set_sticks`]
    /// first (binary mission) or left sticks at failsafe-idle (unit tests).
    ///
    /// Order matches C++ `SitlQuadPlaneHarness::step`: sensors from
    /// SimQuadPlane, Plane tick, QuadPlane update + QHover leftover,
    /// FW servos + leftover VTOL PWM, then `SimQuadPlane::update`.
    pub fn step(
        &mut self,
        plane: &mut PlaneMainLoop,
        qp: &mut QuadPlane,
        sim: &mut SimQuadPlane,
        now_ms: u32,
        dt: f32,
        vtol_command: f32,
        vtol_armed: bool,
    ) {
        let _ = self.plane_harness.tick_vehicle(plane, &sim.plane, now_ms);

        self.leftover_quadplane_update(qp, now_ms, plane.soft_armed, true, sim.plane.airspeed);

        // C++ `qh.throttle_wait = !vtol_armed` each tick, not the
        // QHover-enter parked latch.
        qp.set_throttle_wait(!vtol_armed);
        self.last_qhover = qhover_run(qp, &QManualRunView::flying());
        if vtol_armed && self.last_qhover.action == QManualRunAction::HoldHover {
            let _ = qp.hold_hover(0.0);
        }

        let mut mo = MotorsOutputView::armed_output(now_ms);
        mo.armed_and_safety_off = plane.soft_armed && vtol_armed;
        mo.motors_throttle = vtol_command;
        mo.run_rate_controller = false;
        self.last_motors = qp.motors_output(mo);

        let mut input = fw_servos_from_plane(plane);
        leftover_apply_vtol_motors(
            &mut input,
            sim,
            &mut self.mixer,
            &mut self.mixer_inited,
            vtol_command,
            vtol_armed && self.last_motors.action == MotorsOutputAction::Output,
        );
        self.last_input = input;
        sim.update(&input, dt);
        self.tick_count = self.tick_count.saturating_add(1);
    }
}

pub mod sitl_quadplane {
    //! Leftover catalog for VT-011 SitlQuadPlaneHarness.

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub enum PortStatus {
        OnMain = 0,
        ThisSlice = 1,
        Remaining = 2,
        OutOfScope = 3,
    }

    pub struct PortItem {
        pub name: &'static str,
        pub status: PortStatus,
        pub note: &'static str,
    }

    pub const COMPLETENESS: &[PortItem] = &[
        PortItem {
            name: "leftover catalog",
            status: PortStatus::ThisSlice,
            note: "this table",
        },
        PortItem {
            name: "SitlQuadPlaneHarness scaffold",
            status: PortStatus::ThisSlice,
            note: "Plane + QuadPlane + SimQuadPlane; sensors then tick then SitlInput",
        },
        PortItem {
            name: "sensor synthesis",
            status: PortStatus::ThisSlice,
            note: "SitlHarness pattern from SimQuadPlane truth",
        },
        PortItem {
            name: "Plane::tick",
            status: PortStatus::ThisSlice,
            note: "SitlHarness::tick_vehicle like sitl_run",
        },
        PortItem {
            name: "QuadPlane::update",
            status: PortStatus::ThisSlice,
            note: "SltTransition leftover (run_quadplane_update)",
        },
        PortItem {
            name: "QHover leftover",
            status: PortStatus::ThisSlice,
            note: "qhover_run + QuadPlane::hold_hover",
        },
        PortItem {
            name: "VTOL motors leftover",
            status: PortStatus::ThisSlice,
            note: "MotorsMatrix Quad-X at Frame::motor_offset into SitlInput",
        },
        PortItem {
            name: "SIM_QuadPlane plant",
            status: PortStatus::OnMain,
            note: "sim_quadplane.rs VT-010 original-source",
        },
        PortItem {
            name: "SitlHarness Plane path (FW-046)",
            status: PortStatus::OnMain,
            note: "harness.rs",
        },
        PortItem {
            name: "GCS / MAVLink / interactive run",
            status: PortStatus::OutOfScope,
            note: "bounded duration like FW-046",
        },
        PortItem {
            name: "AP:: / HAL SITL singletons",
            status: PortStatus::OutOfScope,
            note: "ADR-0012 explicit refs",
        },
        PortItem {
            name: "C++ SitlQuadPlaneHarness copy",
            status: PortStatus::OutOfScope,
            note: "Do not copy C++",
        },
    ];

    #[must_use]
    pub const fn completeness_size() -> usize {
        COMPLETENESS.len()
    }

    #[must_use]
    pub fn count_status(status: PortStatus) -> usize {
        COMPLETENESS.iter().filter(|i| i.status == status).count()
    }

    #[must_use]
    pub fn completeness_has(name: &str, status: PortStatus) -> bool {
        COMPLETENESS
            .iter()
            .any(|i| i.name == name && i.status == status)
    }

    #[must_use]
    pub fn on_main_count() -> usize {
        count_status(PortStatus::OnMain)
    }
    #[must_use]
    pub fn this_slice_count() -> usize {
        count_status(PortStatus::ThisSlice)
    }
    #[must_use]
    pub fn remaining_count() -> usize {
        count_status(PortStatus::Remaining)
    }
    #[must_use]
    pub fn out_of_scope_count() -> usize {
        count_status(PortStatus::OutOfScope)
    }
}

#[cfg(test)]
mod tests {
    use super::sitl_quadplane::{
        completeness_has, completeness_size, on_main_count, out_of_scope_count, remaining_count,
        this_slice_count, PortStatus,
    };
    use super::*;
    use crate::set_sticks;
    use ap_plane::main_loop::PlaneMainLoop;
    use ap_quadplane::QuadPlane;
    use ap_sim::sim_quadplane::SimQuadPlane;

    #[test]
    fn step_ticks_plane_and_quadplane_into_sim() {
        let mut plane = PlaneMainLoop::default();
        let mut qp = QuadPlane::with_enable(1);
        setup_hover_transition(&mut plane, &mut qp);
        let mut sim = SimQuadPlane::new("quadplane");
        let mut harness = SitlQuadPlaneHarness::new();
        assert_eq!(harness.tick_count(), 0);
        let hover = sim.frame.hover_command();
        set_sticks(&mut plane, 1500, 1500, 1000, 1500);
        harness.step(&mut plane, &mut qp, &mut sim, 20, 0.0025, hover, true);
        assert_eq!(harness.tick_count(), 1);
        assert!(qp.available());
    }

    #[test]
    fn climb_command_leaves_the_ground() {
        let mut plane = PlaneMainLoop::default();
        let mut qp = QuadPlane::with_enable(1);
        setup_hover_transition(&mut plane, &mut qp);
        let mut sim = SimQuadPlane::new("quadplane");
        let mut harness = SitlQuadPlaneHarness::new();
        let climb = sim.frame.hover_command() + 0.20;
        let dt = 0.0025_f32;
        let mut now_ms = 0_u32;
        for _ in 0..1600 {
            now_ms = now_ms.saturating_add(3);
            set_sticks(&mut plane, 1500, 1500, 1000, 1500);
            harness.step(&mut plane, &mut qp, &mut sim, now_ms, dt, climb, true);
        }
        assert!(-sim.plane.position.z > 2.0, "alt={}", -sim.plane.position.z);
        assert!(sim.plane.airspeed.is_finite());
    }

    #[test]
    fn leftover_catalog_remaining_count() {
        assert_eq!(remaining_count(), 0);
        assert_eq!(this_slice_count(), 7);
        assert_eq!(on_main_count(), 2);
        assert_eq!(out_of_scope_count(), 3);
        assert_eq!(
            completeness_size(),
            on_main_count() + this_slice_count() + remaining_count() + out_of_scope_count()
        );
        assert!(completeness_has(
            "SitlQuadPlaneHarness scaffold",
            PortStatus::ThisSlice
        ));
        assert!(completeness_has("Plane::tick", PortStatus::ThisSlice));
        assert!(completeness_has("QuadPlane::update", PortStatus::ThisSlice));
        assert!(completeness_has("SIM_QuadPlane plant", PortStatus::OnMain));
    }
}
