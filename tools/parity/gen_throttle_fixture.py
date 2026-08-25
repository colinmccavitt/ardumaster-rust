"""Dump the multicopter throttle and lean-angle logic from the real firmware.

Its own binary. These functions read the motors object and the AHRS through
accessors that the attitude sequences never touch, and several of them latch
state (the mix slews, the lean limit filters), so sharing a probe with the
attitude harness would mean each sequence inherited whatever the previous one
left behind.

`get_throttle_boosted` reads `_ahrs.cos_pitch()` and `cos_roll()`, which cannot
be scripted on an uninitialised AHRS. So the sweep drives `_thrust_angle_rad`,
which is a member, and records the AHRS-derived cos_tilt alongside each row
rather than assuming it: whatever the AHRS reports, both sides see the same
number.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/plane-fw-rust")
OUT = ROOT / "fixtures/throttle_mix.csv"
BUILD = Path("/tmp/throttle_parity/harness")

HARNESS = r'''
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Motors/AP_MotorsMatrix.h>
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h>
#include <AC_PID/AC_PID.h>
#include <cstdarg>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace AP_HAL {
void panic(const char *errormsg, ...)
{
    va_list ap;
    va_start(ap, errormsg);
    vfprintf(stderr, errormsg, ap);
    va_end(ap);
    fprintf(stderr, "\n");
    abort();
}
}


static uint32_t fbits(float f)
{
    uint32_t u;
    memcpy(&u, &f, sizeof(u));
    return u;
}

extern const AP_HAL::HAL &hal;

static void harness_set_time_us(uint64_t t)
{
    hal.scheduler->stop_clock(t);
}

class MotorProbe : public AP_MotorsMatrix {
public:
    using AP_MotorsMatrix::AP_MotorsMatrix;
    using AP_MotorsMulticopter::_throttle_thrust_max;
    using AP_Motors::_throttle_out;
    using AP_Motors::_throttle_filter;
    using AP_Motors::_throttle_slew_rate;
    using AP_Motors::_throttle_avg_max;
    using AP_Motors::_throttle_in;
};

class Probe : public AC_AttitudeControl_Multi {
public:
    using AC_AttitudeControl_Multi::AC_AttitudeControl_Multi;
    using AC_AttitudeControl_Multi::update_althold_lean_angle_max;
    using AC_AttitudeControl_Multi::get_throttle_boosted;
    using AC_AttitudeControl_Multi::get_throttle_avg_max;
    using AC_AttitudeControl_Multi::update_throttle_rpy_mix;
    using AC_AttitudeControl::_dt_s;
    using AC_AttitudeControl::_thrust_angle_rad;
    using AC_AttitudeControl::_angle_boost;
    using AC_AttitudeControl::_throttle_rpy_mix;
    using AC_AttitudeControl::_throttle_rpy_mix_desired;
    using AC_AttitudeControl::_althold_lean_angle_max_rad;
    using AC_AttitudeControl::_angle_limit_tc;
    using AC_AttitudeControl_Multi::_thr_mix_min;
    using AC_AttitudeControl_Multi::_thr_mix_max;
    using AC_AttitudeControl_Multi::_throttle_gain_boost;
    using AC_AttitudeControl::_ang_vel_body_rads;
    using AC_AttitudeControl::_feedforward_scalar;
    using AC_AttitudeControl::_sysid_ang_vel_body_rads;
    using AC_AttitudeControl::_actuator_sysid;
    using AC_AttitudeControl::_rate_gyro_rads;
    using AC_AttitudeControl::_pd_scale_used;
    using AC_AttitudeControl::_pd_scale;
    using AC_AttitudeControl::_throttle_in;
    using AC_AttitudeControl::_euler_angle_target_rad;
    using AC_AttitudeControl::_euler_rate_target_rads;
    using AC_AttitudeControl::_ang_vel_target_rads;
    using AC_AttitudeControl::_attitude_ang_error;
    using AC_AttitudeControl::_attitude_target;
    using AC_AttitudeControl::_thrust_error_angle_rad;
    using AC_AttitudeControl::_i_scale;
};

int main()
{
    AP::scheduler().init(nullptr, 0, 0);

    static MotorProbe motors(400.0f);
    motors.init(AP_Motors::MOTOR_FRAME_QUAD, AP_Motors::MOTOR_FRAME_TYPE_X);
    static AP_AHRS_View view(AP::ahrs(), ROTATION_NONE);
    static Probe att(view, motors);
    att._dt_s = 0.0025f;

    // Spool-driven in flight; set here so the lean-angle filter runs at
    // all rather than every row taking the zero-thrust guard.
    motors._throttle_thrust_max = 0.95f;


    // The config the port must be handed. cos_tilt comes from the AHRS and is
    // recorded rather than assumed.
    // Zero by default, which makes the gain boost a no-op however fast
    // the throttle slews. Set before the dump, so the port is handed the
    // same value rather than its own default.
    att._throttle_gain_boost.set(0.6f);

    printf("#config\n");
    printf("dt,angle_limit_tc,thr_mix_min,thr_mix_max,throttle_hover,"
           "throttle_thrust_max,cos_tilt,angle_boost_enabled,throttle_gain_boost\n");
    printf("%u,%u,%u,%u,%u,%u,%u,%d,%u\n",
           fbits(att._dt_s),
           fbits(att._angle_limit_tc),
           fbits(att._thr_mix_min),
           fbits(att._thr_mix_max),
           fbits(motors.get_throttle_hover()),
           fbits(motors.get_throttle_thrust_max()),
           fbits(AP::ahrs().cos_pitch() * AP::ahrs().cos_roll()),
           1,
           fbits(att._throttle_gain_boost));

    // ---- the lean-angle limit, filtered ----
    //
    // Stepped rather than swept: the result is a first-order filter, so what
    // matters is the trajectory and not any single value.
    printf("#leanangle\n");
    printf("step,throttle_in,lean_max\n");
    {
        att._althold_lean_angle_max_rad = 0.0f;
        for (int i = 0; i < 600; i++) {
            const float ts = i * 0.0025f;
            // Across the whole range, including past the 0.8 knee where the
            // permitted lean is pinned at zero.
            const float thr = ts < 0.5f ? 0.05f + 1.6f * ts : 0.95f - 0.9f * (ts - 0.5f);
            att.update_althold_lean_angle_max(thr);
            printf("%d,%u,%u\n", i, fbits(thr),
                   fbits(att._althold_lean_angle_max_rad));
        }
    }

    // ---- tilt compensation ----
    printf("#boost\n");
    printf("idx,throttle_in,thrust_angle,boosted,angle_boost\n");
    {
        const float throttles[] = {0.0f, 0.15f, 0.5f, 0.85f, 1.0f};
        // Past 84 degrees the 1/cos clamp binds, so the sweep goes there.
        const float angles[] = {0.0f, 0.2f, 0.6f, 1.0f, 1.4f, 1.5f, 1.57f};
        int idx = 0;
        for (unsigned a = 0; a < sizeof(throttles)/sizeof(throttles[0]); a++) {
            for (unsigned b = 0; b < sizeof(angles)/sizeof(angles[0]); b++) {
                att._thrust_angle_rad = angles[b];
                const float out = att.get_throttle_boosted(throttles[a]);
                printf("%d,%u,%u,%u,%u\n", idx++,
                       fbits(throttles[a]), fbits(angles[b]),
                       fbits(out), fbits(att._angle_boost));
            }
        }
    }

    // ---- the average-maximum throttle ----
    printf("#avgmax\n");
    printf("idx,mix,throttle_in,avg_max\n");
    {
        const float mixes[] = {0.1f, 0.5f, 0.9f, 1.0f, 2.5f, 5.0f};
        const float throttles[] = {-0.2f, 0.0f, 0.1f, 0.35f, 0.7f, 1.0f, 1.3f};
        int idx = 0;
        for (unsigned a = 0; a < sizeof(mixes)/sizeof(mixes[0]); a++) {
            for (unsigned b = 0; b < sizeof(throttles)/sizeof(throttles[0]); b++) {
                att._throttle_rpy_mix = mixes[a];
                const float out = att.get_throttle_avg_max(throttles[b]);
                printf("%d,%u,%u,%u\n", idx++,
                       fbits(mixes[a]), fbits(throttles[b]), fbits(out));
            }
        }
    }

    // ---- the mix slew ----
    //
    // Rising and falling are different code with different rates, and the
    // falling branch has the mix_used snap-down inside it, so the sequence
    // has to go both ways.
    printf("#mixslew\n");
    printf("step,desired,throttle_in,throttle_out,mix\n");
    {
        att._throttle_rpy_mix = 0.1f;
        for (int i = 0; i < 1200; i++) {
            const float ts = i * 0.0025f;
            // Below 0.1 in the falling phase, so the final clamp binds too.
            att._throttle_rpy_mix_desired = ts < 1.0f ? 0.9f : 0.05f;

            // The mixer's output must spend the falling phase BELOW hover,
            // or mix_used exceeds the mix and the snap-down never binds --
            // which is exactly what the first version of this did.
            const float thr_in = 0.10f + 0.05f * ts;
            const float thr_out = thr_in + 0.02f * (1.0f + ts);
            motors._throttle_filter.reset(thr_in);
            motors._throttle_out = thr_out;

            att.update_throttle_rpy_mix();
            printf("%d,%u,%u,%u,%u\n", i,
                   fbits(att._throttle_rpy_mix_desired),
                   fbits(motors.get_throttle()),
                   fbits(motors.get_throttle_out()),
                   fbits(att._throttle_rpy_mix));
        }
    }

    // ---- the rate loop ----
    //
    // The PID gains are recorded so the port runs the same controller; without
    // them a match would only prove both sides read the same defaults.
    // kff defaults to zero, which would leave every ff column zero and
    // the whole feed-forward path -- including the yaw scaling, the one
    // asymmetry in this function -- recorded as nothing. Set BEFORE the
    // dump below, or the recorded gains describe a controller the
    // sequence never ran. Different per axis so a port that crossed two
    // of them is caught.
    att.get_rate_roll_pid().ff().set(0.050f);
    att.get_rate_pitch_pid().ff().set(0.080f);
    att.get_rate_yaw_pid().ff().set(0.120f);

    printf("#pidgains\n");
    printf("axis,kp,ki,kd,kff,kimax,filt_t,filt_e,filt_d,slew\n");
    {
        AC_PID *pids[3] = {&att.get_rate_roll_pid(), &att.get_rate_pitch_pid(),
                           &att.get_rate_yaw_pid()};
        for (int a = 0; a < 3; a++) {
            printf("%d,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", a,
                   fbits(pids[a]->kP()), fbits(pids[a]->kI()),
                   fbits(pids[a]->kD()), fbits(pids[a]->ff()),
                   fbits(pids[a]->imax()),
                   fbits(pids[a]->filt_T_hz()), fbits(pids[a]->filt_E_hz()),
                   fbits(pids[a]->filt_D_hz()), fbits(pids[a]->slew_limit()));
        }
    }

    printf("#rateloop\n");
    printf("step,tgt_x,tgt_y,tgt_z,gyro_x,gyro_y,gyro_z,ff_scalar,"
           "lim_r,lim_p,lim_y,now_ms,slew_rate,"
           "sysid_x,sysid_y,sysid_z,act_x,act_y,act_z,"
           "pd_x,pd_y,pd_z,i_x,i_y,i_z,"
           "roll,pitch,yaw,roll_ff,pitch_ff,yaw_ff,mix,pd_used,angle_p_used\n");
    {
        att.reset_rate_controller_I_terms();
        att.get_rate_roll_pid().reset_filter();
        att.get_rate_pitch_pid().reset_filter();
        att.get_rate_yaw_pid().reset_filter();

        att._throttle_rpy_mix = 0.3f;
        att._throttle_rpy_mix_desired = 0.6f;
        att._sysid_ang_vel_body_rads.zero();
        att._actuator_sysid.zero();

        const int STEPS = 800;
        for (int i = 0; i < STEPS; i++) {
            const float ts = i * 0.0025f;

            const Vector3f target{1.2f * sinf(4.0f * ts),
                                  -0.9f * cosf(3.0f * ts),
                                  ts < 1.0f ? 0.6f : -0.5f};
            const Vector3f gyro{0.7f * target.x, 0.5f * target.y, 0.85f * target.z};

            // Saturation on one axis at a time, so the integrator freeze is
            // recorded per axis rather than all three at once.
            const bool lim_r = (i / 100) % 3 == 0;
            const bool lim_p = (i / 100) % 3 == 1;
            const bool lim_y = (i / 100) % 3 == 2;

            // Zero by default, which leaves both injection paths untested.
            const Vector3f sysid{0.05f * sinf(9.0f * ts), -0.04f * cosf(7.0f * ts),
                                 0.03f * sinf(5.0f * ts)};
            const Vector3f actuator{0.01f * cosf(6.0f * ts), 0.02f * sinf(8.0f * ts),
                                    -0.015f * cosf(4.0f * ts)};
            att._sysid_ang_vel_body_rads = sysid;
            att._actuator_sysid = actuator;

            // The gain boost writes roll and pitch the same, so on its own it
            // cannot distinguish an axis mix-up. These deliberately differ.
            const Vector3f pd_scale{1.0f + 0.20f * sinf(3.0f * ts),
                                    1.0f + 0.35f * cosf(2.0f * ts),
                                    1.0f + 0.10f * sinf(5.0f * ts)};
            const Vector3f i_scale{1.0f + 0.15f * cosf(3.5f * ts),
                                   1.0f + 0.25f * sinf(2.5f * ts),
                                   1.0f + 0.05f * cosf(4.5f * ts)};
            att._pd_scale = pd_scale;
            att._i_scale = i_scale;

            att._ang_vel_body_rads = target;
            att._feedforward_scalar = 0.4f + 0.6f * (ts < 1.0f ? ts : 1.0f);

            // Driven by a derivative filter over the motor output that a
            // harness never spins up, so it reads zero and the gain boost
            // never fires. Set directly, and swept across the 1.0 threshold
            // so both branches are recorded.
            motors._throttle_slew_rate = 2.5f * sinf(2.5f * ts);

            motors._throttle_filter.reset(0.2f + 0.3f * ts);
            motors._throttle_out = 0.25f + 0.3f * ts;
            motors.limit.roll = lim_r;
            motors.limit.pitch = lim_p;
            motors.limit.yaw = lim_y;

            harness_set_time_us((uint64_t)(ts * 1e6f) + 1000000ULL);

            att.rate_controller_run_dt(gyro, 0.0025f);

            printf("%d,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%u,%u,"
                   "%u,%u,%u,%u,%u,%u,"
                   "%u,%u,%u,%u,%u,%u,"
                   "%u,%u,%u,%u,%u,%u,%u,%u,%u\n", i,
                   fbits(target.x), fbits(target.y), fbits(target.z),
                   fbits(gyro.x), fbits(gyro.y), fbits(gyro.z),
                   fbits(att._feedforward_scalar),
                   lim_r ? 1 : 0, lim_p ? 1 : 0, lim_y ? 1 : 0,
                   (unsigned)(AP_HAL::millis()),
                   fbits(motors.get_throttle_slew_rate()),
                   fbits(sysid.x), fbits(sysid.y), fbits(sysid.z),
                   fbits(actuator.x), fbits(actuator.y), fbits(actuator.z),
                   fbits(pd_scale.x), fbits(pd_scale.y), fbits(pd_scale.z),
                   fbits(i_scale.x), fbits(i_scale.y), fbits(i_scale.z),
                   fbits(motors.get_roll()), fbits(motors.get_pitch()),
                   fbits(motors.get_yaw()), fbits(motors.get_roll_ff()),
                   fbits(motors.get_pitch_ff()), fbits(motors.get_yaw_ff()),
                   fbits(att._throttle_rpy_mix),
                   fbits(att._pd_scale_used.x),
                   fbits(att.get_last_angle_P_scale().x));

            // The vehicle calls this every loop, immediately after the rate
            // controller (ArduCopter/Attitude.cpp:23). Without it the gain
            // boost's multiply compounds every cycle and the scales run to
            // infinity within a second -- which is what the first recording
            // of this sequence did.
            att.rate_controller_target_reset();
        }
    }

    // ---- set_throttle_out ----
    //
    // Swept over the boost flag as well as the throttle, because the flag does
    // not merely skip the boost: it also clears the logged angle_boost, and a
    // port that left it stale would report a boost that did not happen.
    //
    // The lean-angle limit is filtered state, so this runs as a sequence
    // rather than independent rows.
    printf("#throttleout\n");
    printf("step,throttle_in,apply_boost,thrust_angle,"
           "throttle_out,avg_max,angle_boost,lean_max\n");
    {
        att._althold_lean_angle_max_rad = 0.0f;
        att._throttle_rpy_mix = 0.45f;
        att._throttle_rpy_mix_desired = 0.45f;

        const int STEPS = 500;
        for (int i = 0; i < STEPS; i++) {
            const float ts = i * 0.0025f;
            const float thr = 0.05f + 0.85f * (ts < 0.625f ? ts / 0.625f : 1.0f);
            const bool boost = (i / 125) % 2 == 0;

            // Past 84 degrees the fade would matter, but the AHRS reports
            // level here; the target lean still drives boost_factor.
            att._thrust_angle_rad = 1.4f * (ts / 1.25f);

            att.set_throttle_out(thr, boost, 10.0f);

            printf("%d,%u,%d,%u,%u,%u,%u,%u\n", i,
                   fbits(thr), boost ? 1 : 0, fbits(att._thrust_angle_rad),
                   fbits(motors._throttle_in),
                   fbits(motors._throttle_avg_max),
                   fbits(att._angle_boost),
                   fbits(att._althold_lean_angle_max_rad));
        }
    }

    // ---- relax_attitude_controllers ----
    //
    // State assignment rather than arithmetic, so what is recorded is the
    // state afterwards. Run from a controller carrying real state, or the
    // reset would be indistinguishable from construction.
    printf("#relax\n");
    printf("idx,gyro_x,gyro_y,gyro_z,body_r,body_p,body_y,"
           "targ_r,targ_p,targ_y,err_w,err_x,err_y,err_z,"
           "avt_x,avt_y,avt_z,ert_x,ert_y,ert_z,thrust_err\n");
    {
        for (int k = 0; k < 6; k++) {
            const float s = 0.2f * (k + 1);

            // Put the controller somewhere first, so the relax has something
            // to undo.
            Quaternion offset;
            offset.from_euler(0.3f * s, -0.25f * s, 0.9f * s);
            att._attitude_target = offset;
            att._ang_vel_target_rads = Vector3f{1.5f * s, -1.1f * s, 0.7f * s};
            att._euler_rate_target_rads = Vector3f{0.9f * s, 0.4f * s, -0.6f * s};
            att._thrust_error_angle_rad = 0.5f * s;
            Quaternion err;
            err.from_euler(0.1f * s, 0.2f * s, -0.15f * s);
            att._attitude_ang_error = err;

            const Vector3f gyro{0.35f * s, -0.22f * s, 0.11f * s};
            att._rate_gyro_rads = gyro;

            att.relax_attitude_controllers();

            Quaternion body_now;
            view.get_quat_body_to_ned(body_now);
            Vector3f body_euler;
            body_now.to_euler(body_euler);

            const Vector3f te = att._euler_angle_target_rad;
            const Quaternion ae = att._attitude_ang_error;
            const Vector3f avt = att._ang_vel_target_rads;
            const Vector3f ert = att._euler_rate_target_rads;

            printf("%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,"
                   "%u,%u,%u,%u,%u,%u,%u\n", k,
                   fbits(gyro.x), fbits(gyro.y), fbits(gyro.z),
                   fbits(body_euler.x), fbits(body_euler.y), fbits(body_euler.z),
                   fbits(te.x), fbits(te.y), fbits(te.z),
                   fbits(ae.q1), fbits(ae.q2), fbits(ae.q3), fbits(ae.q4),
                   fbits(avt.x), fbits(avt.y), fbits(avt.z),
                   fbits(ert.x), fbits(ert.y), fbits(ert.z),
                   fbits(att._thrust_error_angle_rad));
        }
    }

    return 0;
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/throttle_parity/vehicle")
    build(HARNESS, objects, BUILD,
          "AC_AttitudeControl/AC_AttitudeControl_Multi.cpp",
          link_flags=vehicle_link.LINK_FLAGS)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines() if l and not l[0].isalpha() and not l.startswith("#"))
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
