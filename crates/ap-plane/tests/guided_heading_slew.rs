//! GUIDED offboard heading-slew PID — `g2.guidedHeading` (FW-044).
//!
//! Covers both heading-error modes, bank-limit MIN against `roll_limit_cd`,
//! the `target_heading_time_ms` time-delta update, saturation writing
//! `target_heading_limit`, and that the controller is the real [`ap_pid::AcPid`].

use ap_math::scalar::{constrain_int32, degrees, wrap_pi, GRAVITY_MSS};
use ap_pid::AcPid;
use ap_plane::guided_heading_slew::{
    guided_heading_pid, guided_heading_slew, GuidedHeadingSlewInputs, GuidedHeadingType,
};

fn base_inp() -> GuidedHeadingSlewInputs {
    GuidedHeadingSlewInputs {
        now_ms: 10_500,
        target_heading_time_ms: 10_000,
        target_heading: 0.1,
        target_heading_type: GuidedHeadingType::Heading,
        yaw_rad: 0.0,
        groundspeed_x: 10.0,
        groundspeed_y: 0.0,
        target_heading_accel_limit: 5.0,
        roll_limit_cd: 4500,
        target_heading_limit: false,
    }
}

fn expected_error(inp: &GuidedHeadingSlewInputs) -> f32 {
    if inp.target_heading_type == GuidedHeadingType::Heading {
        wrap_pi(inp.target_heading - inp.yaw_rad)
    } else {
        wrap_pi(
            inp.target_heading - f32::atan2(-inp.groundspeed_y, -inp.groundspeed_x)
                + core::f32::consts::PI,
        )
    }
}

fn expected_bank_limit(inp: &GuidedHeadingSlewInputs) -> f32 {
    let bank = degrees((inp.target_heading_accel_limit / GRAVITY_MSS).atan()) * 1e2_f32;
    bank.min(inp.roll_limit_cd as f32)
}

fn expected_from(inp: &GuidedHeadingSlewInputs, pid: &mut AcPid) -> (i32, bool, u32) {
    let delta = inp.now_ms.wrapping_sub(inp.target_heading_time_ms) as f32 * 1e-3_f32;
    let desired = pid.update_error(
        expected_error(inp),
        delta,
        inp.target_heading_limit,
        inp.now_ms,
    );
    let bank_limit = expected_bank_limit(inp);
    (
        constrain_int32(desired as i32, (-bank_limit) as i32, bank_limit as i32),
        desired.abs() >= bank_limit,
        inp.now_ms,
    )
}

// --- Both heading-error modes ------------------------------------------------

#[test]
fn heading_mode_uses_yaw_and_ignores_groundspeed() {
    let inp = base_inp();
    assert_eq!(inp.target_heading_type, GuidedHeadingType::Heading);

    let mut pid = guided_heading_pid();
    let out = guided_heading_slew(&inp, &mut pid);

    let mut expected_pid = guided_heading_pid();
    let (nav, limit, tnow) = expected_from(&inp, &mut expected_pid);
    assert_eq!(out.nav_roll_cd, nav);
    assert_eq!(out.target_heading_limit, limit);
    assert_eq!(out.target_heading_time_ms, tnow);

    // First-call seed: P * wrap_pi(0.1 - 0) = 500, well under the accel bank.
    assert_eq!(out.nav_roll_cd, 500);
    assert!(!out.target_heading_limit);

    // Groundspeed must not change HEADING-mode error. Flip gs to east.
    let mut east = inp;
    east.groundspeed_x = 0.0;
    east.groundspeed_y = 10.0;
    let mut pid_east = guided_heading_pid();
    let out_east = guided_heading_slew(&east, &mut pid_east);
    assert_eq!(out_east.nav_roll_cd, out.nav_roll_cd);
}

#[test]
fn cog_mode_uses_groundspeed_course_and_ignores_yaw() {
    let mut inp = base_inp();
    inp.target_heading_type = GuidedHeadingType::Cog;
    // Flying east, target almost north: large course error, not the 0.1 yaw error.
    inp.groundspeed_x = 0.0;
    inp.groundspeed_y = 10.0;
    inp.yaw_rad = 0.0;

    let mut pid = guided_heading_pid();
    let out = guided_heading_slew(&inp, &mut pid);

    let mut expected_pid = guided_heading_pid();
    let (nav, limit, tnow) = expected_from(&inp, &mut expected_pid);
    assert_eq!(out.nav_roll_cd, nav);
    assert_eq!(out.target_heading_limit, limit);
    assert_eq!(out.target_heading_time_ms, tnow);

    // Course of (0, 10) is +pi/2; error = wrap_pi(0.1 - pi/2) is large enough
    // that P*error saturates against the accel-derived bank.
    assert!(out.target_heading_limit);
    assert_ne!(out.nav_roll_cd, 500);

    // Yaw must not change COG-mode error.
    let mut yawed = inp;
    yawed.yaw_rad = 1.2;
    let mut pid_yawed = guided_heading_pid();
    let out_yawed = guided_heading_slew(&yawed, &mut pid_yawed);
    assert_eq!(out_yawed.nav_roll_cd, out.nav_roll_cd);
}

#[test]
fn heading_and_cog_diverge_on_the_same_sensors() {
    let heading_inp = base_inp();
    let mut cog_inp = heading_inp;
    cog_inp.target_heading_type = GuidedHeadingType::Cog;
    cog_inp.groundspeed_x = 0.0;
    cog_inp.groundspeed_y = 10.0;

    let heading = guided_heading_slew(&heading_inp, &mut guided_heading_pid());
    let cog = guided_heading_slew(&cog_inp, &mut guided_heading_pid());
    assert_ne!(heading.nav_roll_cd, cog.nav_roll_cd);
}

// --- Bank-limit MIN against roll_limit_cd ------------------------------------

#[test]
fn bank_limit_min_takes_roll_limit_when_accel_is_large() {
    let mut inp = base_inp();
    // atan(inf) * 180/pi * 100 → 9000 cd, MIN with 2500 → 2500.
    inp.target_heading_accel_limit = 1.0e6;
    inp.roll_limit_cd = 2500;
    // Large heading error so desired saturates against the MIN'd bank.
    inp.target_heading = core::f32::consts::PI;
    inp.yaw_rad = 0.0;

    let out = guided_heading_slew(&inp, &mut guided_heading_pid());
    assert!(out.target_heading_limit);
    assert_eq!(out.nav_roll_cd, 2500);
}

#[test]
fn bank_limit_min_takes_accel_when_roll_limit_is_wider() {
    let mut inp = base_inp();
    inp.target_heading_accel_limit = 1.0;
    inp.roll_limit_cd = 4500;
    inp.target_heading = core::f32::consts::PI;
    inp.yaw_rad = 0.0;

    let bank = expected_bank_limit(&inp);
    // accel=1 → ~583 cd, well below 4500.
    assert!(bank < 1000.0);
    assert!(bank < inp.roll_limit_cd as f32);

    let out = guided_heading_slew(&inp, &mut guided_heading_pid());
    assert!(out.target_heading_limit);
    assert_eq!(out.nav_roll_cd, bank as i32);
    assert_ne!(out.nav_roll_cd, inp.roll_limit_cd);
}

// --- Time-delta update of target_heading_time_ms -----------------------------

#[test]
fn stores_tnow_into_target_heading_time_ms() {
    let inp = base_inp();
    assert_ne!(inp.now_ms, inp.target_heading_time_ms);

    let out = guided_heading_slew(&inp, &mut guided_heading_pid());
    assert_eq!(out.target_heading_time_ms, inp.now_ms);

    // A later tick must publish the new tnow, not keep the previous one.
    let mut later = inp;
    later.target_heading_time_ms = out.target_heading_time_ms;
    later.now_ms = 11_250;
    let out_later = guided_heading_slew(&later, &mut guided_heading_pid());
    assert_eq!(out_later.target_heading_time_ms, 11_250);
}

#[test]
fn time_delta_is_fed_to_the_pid() {
    // After the first (reset) call, dt changes the error-filter step. Two
    // otherwise identical second ticks with different dt must diverge.
    let mut first = base_inp();
    first.now_ms = 10_020;
    first.target_heading_time_ms = 10_000;

    let mut pid_fast = guided_heading_pid();
    let mut pid_slow = guided_heading_pid();
    let _ = guided_heading_slew(&first, &mut pid_fast);
    let _ = guided_heading_slew(&first, &mut pid_slow);

    let mut second = first;
    second.target_heading = 1.0;
    second.target_heading_time_ms = first.now_ms;

    let mut fast = second;
    fast.now_ms = first.now_ms + 20;
    let mut slow = second;
    slow.now_ms = first.now_ms + 500;

    let out_fast = guided_heading_slew(&fast, &mut pid_fast);
    let out_slow = guided_heading_slew(&slow, &mut pid_slow);
    assert_ne!(out_fast.nav_roll_cd, out_slow.nav_roll_cd);
}

// --- Saturation sets target_heading_limit ------------------------------------

#[test]
fn saturation_sets_target_heading_limit() {
    let mut inp = base_inp();
    inp.target_heading_accel_limit = 1.0e6;
    inp.roll_limit_cd = 4500;

    // Small error: |desired| = 500 < 4500.
    let unsaturated = guided_heading_slew(&inp, &mut guided_heading_pid());
    assert!(!unsaturated.target_heading_limit);
    assert_eq!(unsaturated.nav_roll_cd, 500);

    // Large error: |P * pi| >> 4500.
    inp.target_heading = core::f32::consts::PI;
    let saturated = guided_heading_slew(&inp, &mut guided_heading_pid());
    assert!(saturated.target_heading_limit);
    assert_eq!(saturated.nav_roll_cd, 4500);
}

#[test]
fn equality_with_bank_limit_counts_as_saturated() {
    // First-call seed: desired = p * error. Choose error so |desired| equals
    // the MIN'd bank (4500) exactly: error = 4500 / 5000 = 0.9.
    let mut inp = base_inp();
    inp.target_heading_accel_limit = 1.0e6;
    inp.roll_limit_cd = 4500;
    inp.target_heading = 0.9;
    inp.yaw_rad = 0.0;

    let out = guided_heading_slew(&inp, &mut guided_heading_pid());
    assert!(out.target_heading_limit);
    assert_eq!(out.nav_roll_cd, 4500);
}

#[test]
fn previous_limit_is_what_the_pid_sees_this_tick() {
    // update_error is called with the *incoming* target_heading_limit; the
    // outgoing flag is computed after. I=0 so the integrator path is quiet,
    // but PidInfo.limit still records the flag the PID was given.
    let mut inp = base_inp();
    inp.target_heading_limit = true;

    let mut pid = guided_heading_pid();
    let out = guided_heading_slew(&inp, &mut pid);
    assert!(!out.target_heading_limit);
    assert!(pid.info().limit);
}

// --- PID is the real AcPid ---------------------------------------------------

#[test]
fn uses_real_ac_pid_update_error_not_a_second_controller() {
    let inp = base_inp();
    let mut live = guided_heading_pid();
    let out = guided_heading_slew(&inp, &mut live);

    let mut independent = guided_heading_pid();
    let delta = inp.now_ms.wrapping_sub(inp.target_heading_time_ms) as f32 * 1e-3_f32;
    let desired = independent.update_error(
        wrap_pi(inp.target_heading - inp.yaw_rad),
        delta,
        inp.target_heading_limit,
        inp.now_ms,
    );
    let bank = expected_bank_limit(&inp);
    assert_eq!(
        out.nav_roll_cd,
        constrain_int32(desired as i32, (-bank) as i32, bank as i32)
    );
    // Same controller state: the live PID must have recorded the same error
    // the independent AcPid did, not a hand-rolled P term.
    assert_eq!(live.info().error, independent.info().error);
    assert_eq!(live.info().p, independent.info().p);
}

#[test]
fn default_gains_match_parameters_h() {
    let pid = guided_heading_pid();
    assert_eq!(pid.gains.p, 5000.0);
    assert_eq!(pid.gains.i, 0.0);
    assert_eq!(pid.gains.d, 0.0);
    assert_eq!(pid.gains.ff, 0.0);
    assert_eq!(pid.gains.imax, 10.0);
    assert_eq!(pid.gains.filt_t_hz, 5.0);
    assert_eq!(pid.gains.filt_e_hz, 5.0);
    assert_eq!(pid.gains.filt_d_hz, 5.0);
    assert_eq!(pid.gains.srmax, 0.0);
}
