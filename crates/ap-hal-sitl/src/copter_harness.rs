//! COP-034: Copter analogue of SitlHarness (FW-046) / C++ SitlCopterHarness
//! (CCP-043/045). Sensors from SimMulticopter truth, leftover Copter tick,
//! motor PWM back into the Frame/Motor plant.
//!
//! Mission: arm / takeoff ~10 m / fly 1 mile NE / RTL / land original spot.
//! Horizontal motion is original-source AC_PosControl NE + WPNav destination
//! + ModeRTL leftover, mixed as differential MotorsMatrix PWM. Not leftover
//! equal-collective. Vertical throttle is AC_PosControl D (CCP-064 counterpart).

#![allow(missing_docs)]

use ap_control::pos_control_ne::{
    AttitudeCapability, DEstimates, DLimits, DOffsets, DTerrain, DUpdateInputs, NeDisturbance,
    NeEstimates, NeLimits, NeOffsets, NeUpdateInputs, PosControlD, PosControlNe, JERK_NE_MSSS,
    NE_POS_P,
};
use ap_copter::mode_rtl::{rtl_build_path, rtl_init, RtlInitView, RtlPathView};
use ap_copter::mode_stabilize::{stabilize_run, StabilizeRunView};
use ap_copter::vehicle_loop::{
    copter_first_fast_tasks, copter_later_fast_tasks, copter_next_fast_tasks, run_scheduler_tick,
    update_flight_mode, CopterVehicleLoop, COPTER_LOOP_RATE_HZ,
};
use ap_hal::time::{Clock, Micros, Millis};
use ap_math::vector2::{Vector2, Vector2f};
use ap_math::vector3::Vector3f;
use ap_motors::armed::{output_armed_stabilizing, ArmedDemand};
use ap_motors::spool::SpoolState;
use ap_motors::MotorMatrix;
use ap_pid::{AcP1d, AcP2d, AcPid, AcPid2d, AcPidBasic, PidGains};
use ap_scheduler::scheduler::Scheduler;
use ap_sim::sim_motor::SitlInput;
use ap_sim::sim_multicopter::SimMulticopter;
use ap_sim::sim_plane::Vec3;
use ap_wpnav::{
    AttitudeJerkLimits, SetWpDestinationContext, UpdateWpNavContext, WpNav, WPNAV_ACCELERATION_MS,
    WP_RADIUS_M_DEFAULT, WP_SPD_DEFAULT, WP_SPD_DOWN_DEFAULT, WP_SPD_UP_DEFAULT,
};
use core::cell::Cell;

/// Statute mile used by the copter_sitl_run fly-out.
pub const MILE_M: f32 = 1609.34;
/// Default fly-out heading: North (C++ sibling worktree is still altitude-only).
pub const FLY_HEADING_NORTH_RAD: f32 = 0.0;
const ATT_ANGLE_P: f32 = 1.2;
const ATT_RATE_D: f32 = 0.20;
const LEAN_ANGLE_MAX_RAD: f32 = 0.523_598_8;
const REACH_SPEED_MS: f32 = 1.5;

/// C++ leftover Copter vehicle shell for SitlCopterHarness.
#[derive(Debug, Clone)]
pub struct LeftoverCopter {
    pub gyro_buffer: Vec3,
    pub accel_buffer: Vec3,
    pub baro_altitude_m: f32,
    pub gps_lat: i32,
    pub gps_lng: i32,
    pub compass_field_bf: Vec3,
    pub gyro_injected: bool,
    pub accel_injected: bool,
    pub baro_injected: bool,
    pub gps_injected: bool,
    pub compass_injected: bool,
    pub motors_armed: bool,
    pub motors_armed_injected: bool,
    pub spool_unlimited: bool,
    pub spool_injected: bool,
    pub attitude_hold: bool,
    pub attitude_hold_injected: bool,
    pub motor_pwm: [u16; 32],
    pub home_lat: i32,
    pub home_lng: i32,
    pub tick_count: u32,
    pub land_complete: bool,
    pub collective_command: f32,
    pub last_flightmode_run: bool,
    pub loop_dt: f32,
    pub now_ms: u32,
    pub pos_d: PosControlD,
    pub p_pos_d: AcP1d,
    pub pid_vel_d: AcPidBasic,
    pub pid_accel_d: AcPid,
    pub d_limits: DLimits,
    pub pos_ne: PosControlNe,
    pub p_pos_ne: AcP2d,
    pub pid_vel_ne: AcPid2d,
    pub ne_limits: NeLimits,
    pub pos_inited: bool,
    pub roll_target_rad: f32,
    pub pitch_target_rad: f32,
    pub throttle_out: f32,
    mixer: MotorMatrix,
    mixer_inited: bool,
}

impl Default for LeftoverCopter {
    fn default() -> Self {
        Self {
            gyro_buffer: Vec3::zero(),
            accel_buffer: Vec3::zero(),
            baro_altitude_m: 0.0,
            gps_lat: 0,
            gps_lng: 0,
            compass_field_bf: Vec3::zero(),
            gyro_injected: false,
            accel_injected: false,
            baro_injected: false,
            gps_injected: false,
            compass_injected: false,
            motors_armed: false,
            motors_armed_injected: false,
            spool_unlimited: false,
            spool_injected: false,
            attitude_hold: false,
            attitude_hold_injected: false,
            motor_pwm: [0; 32],
            home_lat: -353_632_621,
            home_lng: 1_491_652_374,
            tick_count: 0,
            land_complete: false,
            collective_command: 0.0,
            last_flightmode_run: false,
            loop_dt: 0.0025,
            now_ms: 0,
            pos_d: PosControlD::new(),
            p_pos_d: AcP1d::new(1.0),
            pid_vel_d: AcPidBasic::new(5.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0),
            pid_accel_d: AcPid::new(PidGains {
                p: 0.5,
                i: 1.0,
                d: 0.0,
                ff: 0.0,
                dff: 0.0,
                imax: 1.0,
                pdmax: 0.0,
                filt_t_hz: 20.0,
                filt_e_hz: 20.0,
                filt_d_hz: 0.0,
                srmax: 0.0,
                srtau: 1.0,
            }),
            d_limits: DLimits {
                vel_max_down_ms: WP_SPD_DOWN_DEFAULT,
                vel_max_up_ms: WP_SPD_UP_DEFAULT,
                accel_max_d_mss: 2.5,
                jerk_max_d_msss: 5.0,
            },
            pos_ne: PosControlNe::new(),
            p_pos_ne: AcP2d::new(NE_POS_P),
            pid_vel_ne: AcPid2d::new(2.0, 1.0, 0.5, 0.0, 10.0, 5.0, 5.0),
            ne_limits: NeLimits {
                vel_max_ne_ms: WP_SPD_DEFAULT,
                accel_max_ne_mss: WPNAV_ACCELERATION_MS,
                jerk_max_ne_msss: JERK_NE_MSSS,
            },
            pos_inited: false,
            roll_target_rad: 0.0,
            pitch_target_rad: 0.0,
            throttle_out: 0.0,
            mixer: MotorMatrix::new(),
            mixer_inited: false,
        }
    }
}

pub fn leftover_copter_tick(copter: &mut LeftoverCopter) {
    copter.tick_count = copter.tick_count.saturating_add(1);
    let leftover = update_flight_mode(ap_copter::vehicle_loop::UpdateFlightModeInputs {
        land_complete: copter.land_complete,
        move_vehicle_on_ekf_reset: false,
    });
    copter.last_flightmode_run = leftover.flightmode_run;
}

struct CopterStepClock {
    us: Cell<u32>,
}

impl CopterStepClock {
    fn from_dt_ticks(ticks: u32, dt: f32) -> Self {
        Self {
            us: Cell::new(((ticks as f32) * dt * 1.0e6) as u32),
        }
    }
}

impl Clock for CopterStepClock {
    fn millis(&self) -> Millis {
        Millis(self.us.get() / 1000)
    }
    fn micros(&self) -> Micros {
        Micros(self.us.get())
    }
    fn millis64(&self) -> u64 {
        u64::from(self.us.get()) / 1000
    }
    fn micros64(&self) -> u64 {
        u64::from(self.us.get())
    }
}

fn copter_sitl_tasks() -> [ap_scheduler::scheduler::Task<CopterVehicleLoop>; 9] {
    let [a0, a1, a2, a3] = copter_first_fast_tasks();
    let [b0, b1, b2, b3] = copter_next_fast_tasks();
    let [c0] = copter_later_fast_tasks();
    [a0, a1, a2, a3, b0, b1, b2, b3, c0]
}

/// Leftover mission phases. CRUISE is the 1-mile fly-out; RTL returns to origin.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissionPhase {
    Disarmed,
    Takeoff,
    Cruise,
    Rtl,
    Land,
    Landed,
}

impl MissionPhase {
    pub fn name(self) -> &'static str {
        match self {
            Self::Disarmed => "DISARMED",
            Self::Takeoff => "TAKEOFF",
            Self::Cruise => "CRUISE",
            Self::Rtl => "RTL",
            Self::Land => "LAND",
            Self::Landed => "LANDED",
        }
    }
}

#[derive(Debug, Clone)]
pub struct LeftoverMission {
    pub phase: MissionPhase,
    pub takeoff_alt_m: f32,
    pub climb_command: f32,
    pub land_command: f32,
    pub hold_s: f32,
    pub hold_elapsed_s: f32,
    pub command: f32,
    pub fly_distance_m: f32,
    pub fly_heading_rad: f32,
    pub origin_n: f32,
    pub origin_e: f32,
    pub origin_captured: bool,
    pub cruise_n: f32,
    pub cruise_e: f32,
    pub wpnav: WpNav,
    pub wp_inited: bool,
}

impl Default for LeftoverMission {
    fn default() -> Self {
        Self {
            phase: MissionPhase::Disarmed,
            takeoff_alt_m: 10.0,
            climb_command: 0.70,
            land_command: 0.20,
            hold_s: 2.0,
            hold_elapsed_s: 0.0,
            command: 0.0,
            fly_distance_m: MILE_M,
            fly_heading_rad: FLY_HEADING_NORTH_RAD,
            origin_n: 0.0,
            origin_e: 0.0,
            origin_captured: false,
            cruise_n: 0.0,
            cruise_e: 0.0,
            wpnav: WpNav::new(),
            wp_inited: false,
        }
    }
}

fn attitude_jerk_limits() -> AttitudeJerkLimits {
    AttitudeJerkLimits {
        ang_vel_roll_max_rads: 3.5,
        ang_vel_pitch_max_rads: 3.5,
        accel_roll_max_radss: 20.0,
        accel_pitch_max_radss: 20.0,
        input_tc: 0.2,
    }
}

fn leftover_init_poscontrol(copter: &mut LeftoverCopter) {
    if copter.pos_inited {
        return;
    }
    let attitude = AttitudeCapability {
        ang_vel_roll_max_rads: 3.5,
        ang_vel_pitch_max_rads: 3.5,
        accel_roll_max_radss: 20.0,
        accel_pitch_max_radss: 20.0,
        bf_feedforward: true,
    };
    copter.ne_limits = NeLimits::derive(
        WP_SPD_DEFAULT,
        WPNAV_ACCELERATION_MS,
        JERK_NE_MSSS,
        &attitude,
    );
    copter.p_pos_ne.set_limits(
        copter.ne_limits.vel_max_ne_ms,
        copter.ne_limits.accel_max_ne_mss,
        copter.ne_limits.jerk_max_ne_msss,
    );
    copter.pos_inited = true;
}

fn horiz_speed_ms(sim: &SimMulticopter) -> f32 {
    sim.velocity_ef.x.hypot(sim.velocity_ef.y)
}

fn ned_pos(sim: &SimMulticopter) -> Vector3f {
    Vector3f::new(sim.position.x, sim.position.y, sim.position.z)
}

/// Vertical AC_PosControl D cascade. NED +z down. Counterpart of C++
/// leftover_poscontrol_throttle (CCP-064).
pub fn leftover_poscontrol_throttle(
    copter: &mut LeftoverCopter,
    sim: &SimMulticopter,
    pos_d_target_m: f32,
    vel_d_desired_ms: f32,
) -> f32 {
    leftover_init_poscontrol(copter);
    copter.pos_d.pos_desired_m = f64::from(pos_d_target_m);
    copter.pos_d.vel_desired_ms = vel_d_desired_ms;
    let dt = if copter.loop_dt > 0.0 {
        copter.loop_dt
    } else {
        0.0025
    };
    let inp = DUpdateInputs {
        dt,
        now_ms: copter.now_ms,
        ahrs_control_scale_z: 1.0,
        estimates: DEstimates {
            pos_m: f64::from(sim.position.z),
            vel_ms: sim.velocity_ef.z,
        },
        offsets: DOffsets::default(),
        terrain: DTerrain::default(),
        estimated_accel_d_mss: 0.0,
        throttle_lower: false,
        throttle_upper: false,
        throttle_hover: sim.hover_command(),
        vibe_comp_enabled: false,
        vel_max_down_ms: copter.d_limits.vel_max_down_ms,
    };
    let out = copter.pos_d.update_controller(
        &mut copter.p_pos_d,
        &mut copter.pid_vel_d,
        &mut copter.pid_accel_d,
        &inp,
    );
    copter.throttle_out = out.throttle_out.clamp(0.0, 1.0);
    copter.throttle_out
}

/// Hold altitude with AC_PosControl D (not the leftover vz damper).
pub fn leftover_hold_command(
    copter: &mut LeftoverCopter,
    sim: &SimMulticopter,
    hold_alt_m: f32,
) -> f32 {
    leftover_poscontrol_throttle(copter, sim, -hold_alt_m, 0.0)
}

/// NE position controller: shape toward (target_n, target_e), then PID path
/// to lean angles. Those angles become differential MotorsMatrix PWM.
fn leftover_poscontrol_ne(
    copter: &mut LeftoverCopter,
    sim: &SimMulticopter,
    target_n: f32,
    target_e: f32,
) {
    leftover_init_poscontrol(copter);
    let dt = if copter.loop_dt > 0.0 {
        copter.loop_dt
    } else {
        0.0025
    };
    let mut dest = Vector2::new(f64::from(target_n), f64::from(target_e));
    let mut vel = Vector2f::zero();
    let pos_err = Vector2f::new(
        copter.pos_ne.pos_desired_m.x as f32 - sim.position.x,
        copter.pos_ne.pos_desired_m.y as f32 - sim.position.y,
    );
    let vel_err = Vector2f::new(
        copter.pos_ne.vel_desired_ms.x - sim.velocity_ef.x,
        copter.pos_ne.vel_desired_ms.y - sim.velocity_ef.y,
    );
    copter.pos_ne.input_pos_vel_accel(
        &mut dest,
        &mut vel,
        Vector2f::zero(),
        &copter.ne_limits,
        dt,
        true,
        pos_err,
        vel_err,
    );
    let (_roll, _pitch, yaw) = sim.dcm.to_euler();
    let inp = NeUpdateInputs {
        dt,
        ahrs_control_scale_xy: 1.0,
        ne_control_scale_factor: 1.0,
        vel_max_ne_ms: copter.ne_limits.vel_max_ne_ms,
        estimates: NeEstimates {
            pos_m: Vector2::new(f64::from(sim.position.x), f64::from(sim.position.y)),
            vel_ms: Vector2f::new(sim.velocity_ef.x, sim.velocity_ef.y),
        },
        offsets: NeOffsets::default(),
        lean_angle_max_rad: LEAN_ANGLE_MAX_RAD,
        cos_yaw: yaw.cos(),
        sin_yaw: yaw.sin(),
        att_yaw_target_rad: yaw,
    };
    let mut disturb = NeDisturbance::default();
    let out = copter.pos_ne.update_controller(
        &mut copter.p_pos_ne,
        &mut copter.pid_vel_ne,
        &inp,
        &mut disturb,
    );
    copter.roll_target_rad = out.roll_target_rad;
    copter.pitch_target_rad = out.pitch_target_rad;
}

fn ensure_mixer(copter: &mut LeftoverCopter) {
    if copter.mixer_inited {
        return;
    }
    // FRAME_CLASS=1 QUAD, FRAME_TYPE=1 X — same as C++ leftover_apply_collective.
    let ok = copter.mixer.setup_motors(1, 1);
    debug_assert!(ok, "QUAD X");
    copter.mixer_inited = true;
}

/// Collective throttle + NE lean targets → per-motor PWM through AP_MotorsMatrix.
/// Roll/pitch are attitude error to PosControl NE lean, not leftover equal mix.
pub fn leftover_apply_collective(copter: &mut LeftoverCopter, sim: &SimMulticopter, command: f32) {
    copter.collective_command = command;
    ensure_mixer(copter);
    for slot in &mut copter.motor_pwm {
        *slot = 0;
    }
    if !copter.motors_armed {
        return;
    }
    let (roll, pitch, _yaw) = sim.dcm.to_euler();
    let roll_in =
        (ATT_ANGLE_P * (copter.roll_target_rad - roll) - ATT_RATE_D * sim.gyro.x).clamp(-1.0, 1.0);
    let pitch_in = (ATT_ANGLE_P * (copter.pitch_target_rad - pitch) - ATT_RATE_D * sim.gyro.y)
        .clamp(-1.0, 1.0);
    let yaw_in = (-0.2 * sim.gyro.z).clamp(-1.0, 1.0);
    let demand = ArmedDemand {
        roll: roll_in,
        pitch: pitch_in,
        yaw: yaw_in,
        throttle: command,
        throttle_avg_max: command,
        throttle_thrust_max: 1.0,
        compensation_gain: 1.0,
        yaw_headroom: 200,
        thrust_boost: false,
        thrust_boost_ratio: 0.0,
        motor_lost_index: 0,
    };
    let out = output_armed_stabilizing(&copter.mixer, &demand);
    let n = sim.frame.num_motors as usize;
    for i in 0..n {
        let pwm = (1000.0 + out.get_thrust_rpyt_out(i as u8) * 1000.0)
            .clamp(1000.0, 2000.0)
            .round() as u16;
        let servo = sim.frame.motors()[i].servo as usize;
        if let Some(slot) = copter.motor_pwm.get_mut(servo) {
            *slot = pwm;
        }
    }
}

fn capture_origin(mission: &mut LeftoverMission, sim: &SimMulticopter) {
    if mission.origin_captured {
        return;
    }
    mission.origin_n = sim.position.x;
    mission.origin_e = sim.position.y;
    mission.origin_captured = true;
    mission.cruise_n = mission.origin_n + mission.fly_distance_m * mission.fly_heading_rad.cos();
    mission.cruise_e = mission.origin_e + mission.fly_distance_m * mission.fly_heading_rad.sin();
}

fn wp_set_destination(
    mission: &mut LeftoverMission,
    copter: &LeftoverCopter,
    sim: &SimMulticopter,
    dest_n: f32,
    dest_e: f32,
) {
    let att = attitude_jerk_limits();
    let stopping = ned_pos(sim);
    if !mission.wp_inited {
        mission
            .wpnav
            .wp_and_spline_init_m(WP_SPD_DEFAULT, stopping, copter.now_ms, att);
        mission.wp_inited = true;
    }
    let ctx = SetWpDestinationContext {
        now_ms: copter.now_ms,
        attitude: att,
        stopping_point_ned_m: stopping,
        terrain_d_m: None,
    };
    let _ = mission.wpnav.set_wp_destination_ned_m(
        Vector3f::new(dest_n, dest_e, -mission.takeoff_alt_m),
        false,
        0.0,
        ctx,
    );
}

fn wp_tick_and_distance(
    mission: &mut LeftoverMission,
    copter: &LeftoverCopter,
    sim: &SimMulticopter,
) -> f32 {
    let _ = mission.wpnav.update_wpnav(UpdateWpNavContext {
        now_ms: copter.now_ms,
        dt_s: copter.loop_dt,
        terrain_d_m: None,
    });
    mission.wpnav.get_wp_distance_to_destination_m(ned_pos(sim))
}

fn wp_reached(dist_m: f32, sim: &SimMulticopter) -> bool {
    dist_m <= WP_RADIUS_M_DEFAULT && horiz_speed_ms(sim) < REACH_SPEED_MS
}

fn seat_ne_at_current(copter: &mut LeftoverCopter, sim: &SimMulticopter) {
    copter.pos_ne.pos_desired_m =
        Vector2::new(f64::from(sim.position.x), f64::from(sim.position.y));
    copter.pos_ne.vel_desired_ms = Vector2f::new(sim.velocity_ef.x, sim.velocity_ef.y);
    copter.pos_ne.accel_desired_mss = Vector2f::zero();
    copter.pid_vel_ne.reset_i();
}

fn begin_rtl_leftover(mission: &LeftoverMission, sim: &SimMulticopter) {
    let _ = rtl_init(&RtlInitView::ready(), true);
    let mut view = RtlPathView::ready();
    view.current_alt_m = -sim.position.z;
    view.return_dist_m =
        (sim.position.x - mission.origin_n).hypot(sim.position.y - mission.origin_e);
    view.altitude_m = mission.takeoff_alt_m;
    view.alt_final_m = 0.0;
    let _path = rtl_build_path(&view);
}

pub fn leftover_mission_begin_takeoff(mission: &mut LeftoverMission) {
    mission.phase = MissionPhase::Takeoff;
    mission.hold_elapsed_s = 0.0;
}

pub fn leftover_mission_advance(
    copter: &mut LeftoverCopter,
    sim: &SimMulticopter,
    mission: &mut LeftoverMission,
    dt: f32,
) {
    copter.loop_dt = dt;
    copter.now_ms = (copter.tick_count as f32 * dt * 1000.0) as u32;
    let alt_m = -sim.position.z;
    match mission.phase {
        MissionPhase::Disarmed => {
            copter.motors_armed = false;
            copter.land_complete = true;
            mission.command = 0.0;
            copter.roll_target_rad = 0.0;
            copter.pitch_target_rad = 0.0;
        }
        MissionPhase::Takeoff => {
            copter.motors_armed = true;
            if copter.land_complete {
                copter.land_complete = false;
            }
            capture_origin(mission, sim);
            leftover_poscontrol_ne(copter, sim, mission.origin_n, mission.origin_e);
            mission.command =
                leftover_poscontrol_throttle(copter, sim, -mission.takeoff_alt_m, -2.5);
            if alt_m >= mission.takeoff_alt_m {
                seat_ne_at_current(copter, sim);
                wp_set_destination(mission, copter, sim, mission.cruise_n, mission.cruise_e);
                mission.phase = MissionPhase::Cruise;
                mission.command = leftover_hold_command(copter, sim, mission.takeoff_alt_m);
            }
        }
        MissionPhase::Cruise => {
            copter.motors_armed = true;
            leftover_poscontrol_ne(copter, sim, mission.cruise_n, mission.cruise_e);
            mission.command = leftover_hold_command(copter, sim, mission.takeoff_alt_m);
            let dist = wp_tick_and_distance(mission, copter, sim);
            if wp_reached(dist, sim) {
                seat_ne_at_current(copter, sim);
                begin_rtl_leftover(mission, sim);
                wp_set_destination(mission, copter, sim, mission.origin_n, mission.origin_e);
                mission.phase = MissionPhase::Rtl;
            }
        }
        MissionPhase::Rtl => {
            copter.motors_armed = true;
            leftover_poscontrol_ne(copter, sim, mission.origin_n, mission.origin_e);
            mission.command = leftover_hold_command(copter, sim, mission.takeoff_alt_m);
            let dist = wp_tick_and_distance(mission, copter, sim);
            if wp_reached(dist, sim) {
                mission.phase = MissionPhase::Land;
            }
        }
        MissionPhase::Land => {
            copter.motors_armed = true;
            leftover_poscontrol_ne(copter, sim, mission.origin_n, mission.origin_e);
            mission.command = leftover_poscontrol_throttle(copter, sim, 0.25, 1.5);
            if sim.on_ground() {
                copter.land_complete = true;
                copter.motors_armed = false;
                mission.command = 0.0;
                copter.roll_target_rad = 0.0;
                copter.pitch_target_rad = 0.0;
                mission.phase = MissionPhase::Landed;
            }
        }
        MissionPhase::Landed => {
            copter.motors_armed = false;
            copter.land_complete = true;
            mission.command = 0.0;
            copter.roll_target_rad = 0.0;
            copter.pitch_target_rad = 0.0;
        }
    }
    copter.collective_command = mission.command;
}

/// SitlCopterHarness: sensors from sim, leftover tick, PWM into plant.
pub struct SitlCopterHarness {
    tick_count: u32,
    pub vehicle: CopterVehicleLoop,
    tasks: [ap_scheduler::scheduler::Task<CopterVehicleLoop>; 9],
    last_run: [u16; 9],
}

impl Default for SitlCopterHarness {
    fn default() -> Self {
        Self::new()
    }
}

impl SitlCopterHarness {
    pub fn new() -> Self {
        Self {
            tick_count: 0,
            vehicle: CopterVehicleLoop::typical(),
            tasks: copter_sitl_tasks(),
            last_run: [0; 9],
        }
    }

    pub fn tick_count(&self) -> u32 {
        self.tick_count
    }

    pub fn step(&mut self, copter: &mut LeftoverCopter, sim: &mut SimMulticopter, dt: f32) {
        copter.gyro_buffer = sim.gyro;
        copter.accel_buffer = sim.accel_body;
        copter.gyro_injected = true;
        copter.accel_injected = true;

        sim.update_position();
        sim.update_mag_field_bf();
        copter.baro_altitude_m = -sim.position.z;
        copter.baro_injected = true;

        copter.gps_lat = sim.location_lat_e7;
        copter.gps_lng = sim.location_lng_e7;
        copter.gps_injected = true;

        copter.compass_field_bf = sim.get_mag_field_bf();
        copter.compass_injected = true;

        copter.motors_armed_injected = true;
        if copter.motors_armed {
            copter.spool_unlimited = true;
            copter.attitude_hold = true;
        } else {
            copter.spool_unlimited = false;
            copter.attitude_hold = false;
        }
        copter.spool_injected = true;
        copter.attitude_hold_injected = true;

        leftover_copter_tick(copter);
        self.tick_count = copter.tick_count;

        self.vehicle.flight_mode.land_complete = copter.land_complete;
        self.vehicle.motors.armed = copter.motors_armed;
        self.vehicle.auto_armed.motors_armed = copter.motors_armed;
        self.vehicle.auto_armed.has_valid_input = true;
        self.vehicle.auto_disarm.motors_armed = copter.motors_armed;
        self.vehicle.auto_disarm.land_complete = copter.land_complete;
        let clock = CopterStepClock::from_dt_ticks(self.tick_count, dt);
        let mut scheduler =
            Scheduler::new(&self.tasks, &[], &mut self.last_run, COPTER_LOOP_RATE_HZ);
        let _stats = run_scheduler_tick(&mut self.vehicle, &mut scheduler, &clock, 2_500);

        if copter.last_flightmode_run {
            let throttle_control = (copter.collective_command * 1000.0).clamp(0.0, 1000.0) as i16;
            let view = StabilizeRunView {
                throttle_control,
                throttle_zero: !copter.motors_armed || copter.collective_command < 0.05,
                spool_state: if copter.motors_armed {
                    SpoolState::ThrottleUnlimited
                } else {
                    SpoolState::ShutDown
                },
                ..StabilizeRunView::flying()
            };
            let run = stabilize_run(&view);
            if run.clear_land_complete {
                copter.land_complete = false;
            }
        }
        leftover_apply_collective(copter, sim, copter.collective_command);

        let mut input = SitlInput::default();
        input.servos = copter.motor_pwm;
        sim.update(&input, dt);
    }
}

/// Mission leftover then harness (sensors + Frame/Motor plant).
pub fn leftover_copter_sitl_step(
    harness: &mut SitlCopterHarness,
    copter: &mut LeftoverCopter,
    sim: &mut SimMulticopter,
    mission: &mut LeftoverMission,
    dt: f32,
) {
    leftover_mission_advance(copter, sim, mission, dt);
    harness.step(copter, sim, dt);
}

/// Horizontal miss from the captured origin, metres.
pub fn leftover_touchdown_miss_m(mission: &LeftoverMission, sim: &SimMulticopter) -> f32 {
    (sim.position.x - mission.origin_n).hypot(sim.position.y - mission.origin_e)
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.0025;

    #[test]
    fn zero_command_stays_on_ground() {
        let mut copter = LeftoverCopter::default();
        let mut sim = SimMulticopter::new("x");
        let mut harness = SitlCopterHarness::new();
        let mut mission = LeftoverMission::default();
        leftover_copter_sitl_step(&mut harness, &mut copter, &mut sim, &mut mission, DT);
        assert!(sim.on_ground());
        assert!((-sim.position.z).abs() < 0.01);
    }

    #[test]
    fn climb_command_via_leftover_pwm_leaves_the_ground() {
        let mut copter = LeftoverCopter::default();
        copter.motors_armed = true;
        let mut sim = SimMulticopter::new("x");
        let mut harness = SitlCopterHarness::new();
        let mut mission = LeftoverMission::default();
        leftover_mission_begin_takeoff(&mut mission);
        for _ in 0..1200 {
            leftover_copter_sitl_step(&mut harness, &mut copter, &mut sim, &mut mission, DT);
        }
        assert!(-sim.position.z > 2.0, "alt={}", -sim.position.z);
        assert!(!sim.on_ground());
    }

    #[test]
    fn hover_thr_out_holds_altitude_on_the_real_plant() {
        let mut sim = SimMulticopter::new("x");
        sim.position.z = -10.0;
        sim.velocity_ef = Vec3::zero();
        let mut copter = LeftoverCopter::default();
        copter.motors_armed = true;
        let mut harness = SitlCopterHarness::new();
        let hover = sim.hover_command();
        for _ in 0..400 {
            leftover_apply_collective(&mut copter, &sim, hover);
            harness.step(&mut copter, &mut sim, DT);
        }
        let alt = -sim.position.z;
        assert!((alt - 10.0).abs() < 1.5, "alt={alt}");
    }

    #[test]
    fn leftover_copter_sitl_mission_takeoff_cruise_rtl_land() {
        let mut copter = LeftoverCopter::default();
        let mut sim = SimMulticopter::new("x");
        let mut harness = SitlCopterHarness::new();
        let mut mission = LeftoverMission {
            fly_distance_m: 8.0,
            ..LeftoverMission::default()
        };
        leftover_mission_begin_takeoff(&mut mission);

        let mut max_alt_m = 0.0f32;
        let mut max_n = 0.0f32;
        let mut saw_cruise = false;
        let mut saw_rtl = false;
        let mut saw_differential_pwm = false;
        let max_ticks = 40 * 400;
        for _ in 0..max_ticks {
            leftover_copter_sitl_step(&mut harness, &mut copter, &mut sim, &mut mission, DT);
            let alt = -sim.position.z;
            if alt > max_alt_m {
                max_alt_m = alt;
            }
            if sim.position.x > max_n {
                max_n = sim.position.x;
            }
            if mission.phase == MissionPhase::Cruise {
                saw_cruise = true;
                let n = sim.frame.num_motors as usize;
                let mut lo = u16::MAX;
                let mut hi = 0u16;
                for i in 0..n {
                    let servo = sim.frame.motors()[i].servo as usize;
                    let pwm = copter.motor_pwm[servo];
                    lo = lo.min(pwm);
                    hi = hi.max(pwm);
                }
                if hi.saturating_sub(lo) > 20 {
                    saw_differential_pwm = true;
                }
            }
            if mission.phase == MissionPhase::Rtl {
                saw_rtl = true;
            }
            if mission.phase == MissionPhase::Landed {
                break;
            }
        }

        assert!(
            saw_cruise,
            "never reached CRUISE, phase={:?}",
            mission.phase
        );
        assert!(saw_rtl, "never reached RTL, phase={:?}", mission.phase);
        assert!(max_alt_m >= 9.0, "max_alt={max_alt_m}");
        assert!(
            max_n >= 6.0,
            "never flew out, max_n={max_n} phase={:?}",
            mission.phase
        );
        assert!(
            saw_differential_pwm,
            "cruise PWM was equal-collective; NE lean never mixed"
        );
        assert_eq!(mission.phase, MissionPhase::Landed);
        assert!(copter.land_complete);
        assert!(!copter.motors_armed);
        assert!(sim.on_ground());
        let miss = leftover_touchdown_miss_m(&mission, &sim);
        assert!(
            miss < 3.0,
            "touchdown miss {miss} m from origin ({}, {}) pos=({}, {})",
            mission.origin_n,
            mission.origin_e,
            sim.position.x,
            sim.position.y
        );
        assert!(harness.tick_count() > 0);
        assert!(copter.gyro_injected);
        assert!(copter.baro_injected);
        assert!(
            harness.vehicle.ticks.update_flight_mode > 0,
            "CopterVehicleLoop update_flight_mode never ran"
        );
        assert!(copter.last_flightmode_run);
    }

    #[test]
    fn harness_synthesizes_gyro_accel_baro_gps_compass() {
        let mut copter = LeftoverCopter::default();
        let mut sim = SimMulticopter::new("x");
        sim.position.z = -5.0;
        sim.gyro = Vec3::new(0.1, 0.0, 0.0);
        let mut harness = SitlCopterHarness::new();
        harness.step(&mut copter, &mut sim, DT);
        assert!(copter.gyro_injected);
        assert!(copter.accel_injected);
        assert!(copter.baro_injected);
        assert!(copter.gps_injected);
        assert!(copter.compass_injected);
        assert!(copter.tick_count > 0);
    }
}
