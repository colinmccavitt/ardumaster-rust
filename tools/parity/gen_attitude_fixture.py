"""Dump the attitude controller's error decomposition from the real firmware.

`thrust_heading_rotation_angles` is public and takes both attitudes as
arguments, so it can be driven directly with scripted quaternions -- no AHRS
state to set up, no vehicle loop to run. `update_ang_vel_target_from_att_error`
is protected and reachable through a probe subclass, and it too reads only its
argument and the controller's own gains.

Constructing the controller is the only real obstacle, and it turns out to be
cheap: AP_AHRS_View just stores a reference in its constructor, and the vehicle
already has an AP_AHRS built at load time. Nothing dereferences the AHRS on the
paths below.

Its own binary, per the pattern established for the servo sweeps: this needs a
controller nothing else has configured.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/plane-fw-rust")
OUT = ROOT / "fixtures/attitude_error.csv"
BUILD = Path("/tmp/attitude_parity/harness")

HARNESS = r'''
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Motors/AP_MotorsMatrix.h>
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h>
#include <cstdarg>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace AP_HAL {
    void panic(const char *m, ...)
    {
        va_list ap;
        va_start(ap, m);
        vfprintf(stderr, m, ap);
        va_end(ap);
        fputc('\n', stderr);
        abort();
    }
}

extern const AP_HAL::HAL &hal;

static uint32_t fbits(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }

// The motors object the controller wants. A quad, so the frame is valid.
static AP_MotorsMatrix motors(400.0f);

// update_ang_vel_target_from_att_error is protected; the rest is public.
class Probe : public AC_AttitudeControl_Multi {
public:
    using AC_AttitudeControl_Multi::AC_AttitudeControl_Multi;
    using AC_AttitudeControl::update_ang_vel_target_from_att_error;
    using AC_AttitudeControl::_dt_s;
    using AC_AttitudeControl::_attitude_target;
    using AC_AttitudeControl::_input_tc;
    using AC_AttitudeControl::_rate_y_tc;
    using AC_AttitudeControl::_rate_rp_tc;
    using AC_AttitudeControl::_ang_vel_roll_max_degs;
    using AC_AttitudeControl::_ang_vel_pitch_max_degs;
    using AC_AttitudeControl::_ang_vel_yaw_max_degs;
    using AC_AttitudeControl::_euler_angle_target_rad;
    using AC_AttitudeControl::_rate_gyro_rads;
    using AC_AttitudeControl::_attitude_ang_error;
};

int main(void)
{
    // Before anything asks for the loop period: get_loop_period_us raises
    // flow_of_control when the scheduler has never been initialised.
    AP::scheduler().init(nullptr, 0, 0);

    motors.init(AP_Motors::MOTOR_FRAME_QUAD, AP_Motors::MOTOR_FRAME_TYPE_X);

    // AP_AHRS_View only stores the reference; nothing below dereferences it.
    static AP_AHRS_View view(AP::ahrs(), ROTATION_NONE);
    static Probe att(view, motors);

    att._dt_s = 0.0025f;

    // _rate_rp_tc and _rate_y_tc are plain floats with no initializer -- the
    // vehicle assigns them at mode entry from the AC_CommandModel parameters,
    // so a controller built outside a vehicle reads whatever was on the stack.
    // It read zero, which happens to equal Copter's shipped default, but the
    // fixture cannot rest on that: set them, and set them apart, so the two
    // constants are distinguishable and neither is the no-op value.
    //
    // The velocity limits default to 0, meaning unlimited, so every sequence
    // recorded so far compared the limiter as a no-op on both sides. These
    // bite on pitch and yaw and not on roll, so both branches get recorded.
    att._rate_rp_tc = 0.15f;
    att._rate_y_tc = 0.25f;
    att._ang_vel_roll_max_degs.set(220.0f);
    att._ang_vel_pitch_max_degs.set(140.0f);
    att._ang_vel_yaw_max_degs.set(120.0f);

    // The gains this controller is running, so the port can be driven with
    // exactly the same ones rather than with a hand-copied guess.
    printf("#gains\n");
    printf("angle_p_roll,angle_p_pitch,angle_p_yaw,accel_roll,accel_pitch,"
           "accel_yaw,rate_yaw_kp,use_sqrt,dt,"
           "input_tc,rate_y_tc,ff_enabled,vel_roll,vel_pitch,vel_yaw,slew_yaw,rate_rp_tc\n");
    printf("%u,%u,%u,%u,%u,%u,%u,%d,%u,%u,%u,%d,%u,%u,%u,%u,%u\n",
           fbits(att.get_angle_roll_p().kP()),
           fbits(att.get_angle_pitch_p().kP()),
           fbits(att.get_angle_yaw_p().kP()),
           fbits(att.get_accel_roll_max_radss()),
           fbits(att.get_accel_pitch_max_radss()),
           fbits(att.get_accel_yaw_max_radss()),
           fbits(att.get_rate_yaw_pid().kP()),
           att.get_bf_feedforward() ? 1 : 0,
           fbits(0.0025f),
           fbits(att._input_tc),
           fbits(att._rate_y_tc),
           att.get_bf_feedforward() ? 1 : 0,
           fbits(att._ang_vel_roll_max_degs),
           fbits(att._ang_vel_pitch_max_degs),
           fbits(att._ang_vel_yaw_max_degs),
           fbits(att.get_slew_yaw_max_rads()),
           fbits(att._rate_rp_tc));

    printf("#rows\n");
    printf("body_r,body_p,body_y,targ_r,targ_p,targ_y,"
           "err_x,err_y,err_z,thrust_angle,thrust_err,"
           "rate_x,rate_y,rate_z\n");

    // Attitudes chosen to be neither level nor north-facing wherever possible:
    // that is where motor numbers, frames and axes stop coinciding.
    static const float ANGLES[][3] = {
        {  0.0f,  0.0f,  0.0f },
        {  0.2f, -0.3f,  0.7f },
        { -0.5f,  0.4f, -1.2f },
        {  0.8f,  0.1f,  2.9f },
        {  0.05f, 0.05f, 0.1f },
        { -0.9f, -0.7f,  1.6f },
        {  1.2f,  0.0f, -2.5f },
    };
    const unsigned n = sizeof(ANGLES) / sizeof(ANGLES[0]);

    for (unsigned b = 0; b < n; b++) {
        for (unsigned t = 0; t < n; t++) {
            Quaternion body, target;
            body.from_euler(ANGLES[b][0], ANGLES[b][1], ANGLES[b][2]);
            target.from_euler(ANGLES[t][0], ANGLES[t][1], ANGLES[t][2]);

            Vector3f err;
            float thrust_angle = 0.0f;
            float thrust_err = 0.0f;
            // Takes the target by reference and may rebuild it.
            Quaternion target_inout = target;
            att.thrust_heading_rotation_angles(target_inout, body, err,
                                               thrust_angle, thrust_err);

            const Vector3f rate = att.update_ang_vel_target_from_att_error(err);

            printf("%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
                   fbits(ANGLES[b][0]), fbits(ANGLES[b][1]), fbits(ANGLES[b][2]),
                   fbits(ANGLES[t][0]), fbits(ANGLES[t][1]), fbits(ANGLES[t][2]),
                   fbits(err.x), fbits(err.y), fbits(err.z),
                   fbits(thrust_angle), fbits(thrust_err),
                   fbits(rate.x), fbits(rate.y), fbits(rate.z));
        }
    }

    // ---- a scripted stick sequence through the euler entry point ----
    printf("#sticks\n");
    printf("step,roll_cmd,pitch_cmd,yaw_rate_cmd,"
           "body_r,body_p,body_y,"
           "targ_r,targ_p,targ_y,ang_vel_x,ang_vel_y,ang_vel_z,"
           "rate_x,rate_y,rate_z\n");
    {
        // A fresh controller: the state is what is being tested, so it must
        // start somewhere both sides can reach.
        static Probe seq(view, motors);
        seq._dt_s = 0.0025f;
        seq._rate_rp_tc = 0.15f;
        seq._rate_y_tc = 0.25f;
        seq._ang_vel_roll_max_degs.set(220.0f);
        seq._ang_vel_pitch_max_degs.set(140.0f);
        seq._ang_vel_yaw_max_degs.set(120.0f);
        seq.reset_target_and_rate(true);

        // The controller reads the body attitude from the AHRS itself, so
        // there is nothing to set here -- only something to record. Whatever
        // the uninitialised AHRS reports is deterministic, which is all the
        // comparison needs; the port is then driven with the same value rather
        // than with an assumption about it.

        const int STEPS = 400;
        for (int i = 0; i < STEPS; i++) {
            const float ts = i * 0.0025f;
            // A step in roll, a ramp in pitch, and a yaw rate that reverses:
            // between them these exercise the shaper settling, tracking and
            // turning around.
            const float roll_cmd = ts < 0.2f ? 0.0f : 0.35f;
            const float pitch_cmd = -0.4f + ts * 0.5f;
            const float yaw_rate_cmd = ts < 0.5f ? 0.6f : -0.6f;

            seq.input_euler_angle_roll_pitch_euler_rate_yaw_rad(
                roll_cmd, pitch_cmd, yaw_rate_cmd);

            Quaternion body_now;
            view.get_quat_body_to_ned(body_now);
            Vector3f body_euler;
            body_now.to_euler(body_euler);

            const Vector3f euler = seq._euler_angle_target_rad;
            const Vector3f ang_vel = seq.get_attitude_target_ang_vel();
            const Vector3f rate = seq.rate_bf_targets();

            printf("%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", i,
                   fbits(roll_cmd), fbits(pitch_cmd), fbits(yaw_rate_cmd),
                   fbits(body_euler.x), fbits(body_euler.y), fbits(body_euler.z),
                   fbits(euler.x), fbits(euler.y), fbits(euler.z),
                   fbits(ang_vel.x), fbits(ang_vel.y), fbits(ang_vel.z),
                   fbits(rate.x), fbits(rate.y), fbits(rate.z));
        }
    }

    // ---- a heading command, with and without slew limiting ----
    printf("#heading\n");
    printf("slew,step,roll_cmd,pitch_cmd,yaw_cmd,body_r,body_p,body_y,"
           "targ_r,targ_p,targ_y,ang_vel_x,ang_vel_y,ang_vel_z,"
           "rate_x,rate_y,rate_z\n");
    {
        for (int slew = 0; slew <= 1; slew++) {
            static Probe head(view, motors);
            head._dt_s = 0.0025f;
            head._rate_rp_tc = 0.15f;
            head._rate_y_tc = 0.25f;
            head._ang_vel_roll_max_degs.set(220.0f);
            head._ang_vel_pitch_max_degs.set(140.0f);
            head._ang_vel_yaw_max_degs.set(120.0f);
            head.reset_target_and_rate(true);

            // Start the target away from the body. With target == body the
            // attitude error begins at zero and creeps up through the region
            // where 1 - cos(theta) is too small to represent next to 1.0 --
            // acos returns exactly zero there and the error contribution
            // disappears. Stepping across that boundary is a discontinuity,
            // not a controller behaviour, and comparing two implementations
            // across it measures which side of an ulp each landed on.
            Quaternion offset;
            offset.from_euler(0.30f, -0.20f, 0.50f);
            head._attitude_target = offset;

            const int STEPS = 400;
            for (int i = 0; i < STEPS; i++) {
                const float ts = i * 0.0025f;
                // Hold a lean, then command a large heading change part-way:
                // the slew limit only shows on a change big enough to hit it.
                const float roll_cmd = 0.2f;
                const float pitch_cmd = -0.15f;
                const float yaw_cmd = ts < 0.25f ? 0.0f : 2.0f;

                head.input_euler_angle_roll_pitch_yaw_rad(
                    roll_cmd, pitch_cmd, yaw_cmd, slew != 0);

                Quaternion body_now;
                view.get_quat_body_to_ned(body_now);
                Vector3f body_euler;
                body_now.to_euler(body_euler);

                // The member, not the quaternion's euler angles: the two
                // differ whenever the yaw cap rebuilds the target, and the
                // port stores this one.
                const Vector3f euler = head._euler_angle_target_rad;
                const Vector3f ang_vel = head.get_attitude_target_ang_vel();
                const Vector3f rate = head.rate_bf_targets();

                printf("%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
                       slew, i,
                       fbits(roll_cmd), fbits(pitch_cmd), fbits(yaw_cmd),
                       fbits(body_euler.x), fbits(body_euler.y), fbits(body_euler.z),
                       fbits(euler.x), fbits(euler.y), fbits(euler.z),
                       fbits(ang_vel.x), fbits(ang_vel.y), fbits(ang_vel.z),
                       fbits(rate.x), fbits(rate.y), fbits(rate.z));
            }
        }
    }

    // ---- an euler-rate sequence ----
    printf("#eulerrate\n");
    printf("step,roll_rate,pitch_rate,yaw_rate,body_r,body_p,body_y,"
           "targ_r,targ_p,targ_y,ang_vel_x,ang_vel_y,ang_vel_z,"
           "rate_x,rate_y,rate_z\n");
    {
        static Probe er(view, motors);
        er._dt_s = 0.0025f;
        er._rate_rp_tc = 0.15f;
        er._rate_y_tc = 0.25f;
        er._ang_vel_roll_max_degs.set(220.0f);
        er._ang_vel_pitch_max_degs.set(140.0f);
        er._ang_vel_yaw_max_degs.set(120.0f);
        er.reset_target_and_rate(true);

        // Offset, for the reason on the heading sequence: starting at zero
        // attitude error spends the first steps where acos has no precision.
        Quaternion offset;
        offset.from_euler(-0.25f, 0.35f, -0.40f);
        er._attitude_target = offset;

        const int STEPS = 400;
        for (int i = 0; i < STEPS; i++) {
            const float ts = i * 0.0025f;
            // A step, a ramp, and a reversal again -- the same three shapes,
            // now as rates on all three axes.
            const float roll_rate = ts < 0.2f ? 0.0f : 0.8f;
            const float pitch_rate = -0.5f + ts * 1.2f;
            const float yaw_rate = ts < 0.5f ? 0.4f : -0.4f;

            er.input_euler_rate_roll_pitch_yaw_rads(roll_rate, pitch_rate, yaw_rate);

            Quaternion body_now;
            view.get_quat_body_to_ned(body_now);
            Vector3f body_euler;
            body_now.to_euler(body_euler);

            const Vector3f euler = er._euler_angle_target_rad;
            const Vector3f ang_vel = er.get_attitude_target_ang_vel();
            const Vector3f rate = er.rate_bf_targets();

            printf("%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", i,
                   fbits(roll_rate), fbits(pitch_rate), fbits(yaw_rate),
                   fbits(body_euler.x), fbits(body_euler.y), fbits(body_euler.z),
                   fbits(euler.x), fbits(euler.y), fbits(euler.z),
                   fbits(ang_vel.x), fbits(ang_vel.y), fbits(ang_vel.z),
                   fbits(rate.x), fbits(rate.y), fbits(rate.z));
        }
    }

    // ---- a body-frame rate sequence ----
    printf("#bfrate\n");
    printf("step,roll_rate,pitch_rate,yaw_rate,body_r,body_p,body_y,"
           "targ_r,targ_p,targ_y,ang_vel_x,ang_vel_y,ang_vel_z,"
           "rate_x,rate_y,rate_z\n");
    {
        static Probe bf(view, motors);
        bf._dt_s = 0.0025f;
        bf._rate_rp_tc = 0.15f;
        bf._rate_y_tc = 0.25f;
        bf._ang_vel_roll_max_degs.set(220.0f);
        bf._ang_vel_pitch_max_degs.set(140.0f);
        bf._ang_vel_yaw_max_degs.set(120.0f);
        bf.reset_target_and_rate(true);

        // Offset, per the note on the heading sequence.
        Quaternion offset;
        offset.from_euler(0.40f, 0.30f, 1.10f);
        bf._attitude_target = offset;

        const int STEPS = 400;
        for (int i = 0; i < STEPS; i++) {
            const float ts = i * 0.0025f;
            const float roll_rate = ts < 0.3f ? 0.0f : -0.9f;
            const float pitch_rate = 0.6f - ts * 1.0f;
            const float yaw_rate = ts < 0.6f ? -0.3f : 0.7f;

            bf.input_rate_bf_roll_pitch_yaw_rads(roll_rate, pitch_rate, yaw_rate);

            Quaternion body_now;
            view.get_quat_body_to_ned(body_now);
            Vector3f body_euler;
            body_now.to_euler(body_euler);

            const Vector3f euler = bf._euler_angle_target_rad;
            const Vector3f ang_vel = bf.get_attitude_target_ang_vel();
            const Vector3f rate = bf.rate_bf_targets();

            printf("%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", i,
                   fbits(roll_rate), fbits(pitch_rate), fbits(yaw_rate),
                   fbits(body_euler.x), fbits(body_euler.y), fbits(body_euler.z),
                   fbits(euler.x), fbits(euler.y), fbits(euler.z),
                   fbits(ang_vel.x), fbits(ang_vel.y), fbits(ang_vel.z),
                   fbits(rate.x), fbits(rate.y), fbits(rate.z));
        }
    }

    // ---- a thrust vector with a heading RATE, both slew settings ----
    //
    // The heading rate reaches 1.5 rad/s, above the slew limit (~1.05 rad/s)
    // but below ATC_RATE_Y_MAX (120 deg/s = 2.09 rad/s), so the slew flag
    // decides whether the command is clipped. Anything smaller records the
    // same numbers twice.
    printf("#thrustrate\n");
    printf("slew,step,tx,ty,tz,hrate,body_r,body_p,body_y,"
           "targ_r,targ_p,targ_y,av_x,av_y,av_z,rate_x,rate_y,rate_z\n");
    {
        for (int slew = 0; slew <= 1; slew++) {
            static Probe tr(view, motors);
            tr._dt_s = 0.0025f;
            tr._rate_rp_tc = 0.15f;
            tr._rate_y_tc = 0.25f;
            tr._ang_vel_roll_max_degs.set(220.0f);
            tr._ang_vel_pitch_max_degs.set(140.0f);
            tr._ang_vel_yaw_max_degs.set(120.0f);
            tr.reset_target_and_rate(true);

            Quaternion offset;
            offset.from_euler(0.10f, -0.15f, 0.60f);
            tr._attitude_target = offset;

            const int STEPS = 400;
            for (int i = 0; i < STEPS; i++) {
                const float ts = i * 0.0025f;

                // A lean that opens up, swung around the compass, so roll and
                // pitch are both driven and neither dominates.
                const float lean = 0.05f + 0.30f * ts;
                const float azim = 2.0f * ts;
                Vector3f thrust{sinf(lean) * cosf(azim),
                                sinf(lean) * sinf(azim),
                                -cosf(lean)};

                const float hrate = ts < 0.4f ? 1.5f : -1.5f;

                tr.input_thrust_vector_rate_heading_rads(thrust, hrate, slew != 0);

                Quaternion body_now;
                view.get_quat_body_to_ned(body_now);
                Vector3f body_euler;
                body_now.to_euler(body_euler);

                const Vector3f euler = tr._euler_angle_target_rad;
                const Vector3f ang_vel = tr.get_attitude_target_ang_vel();
                const Vector3f rate = tr.rate_bf_targets();

                printf("%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
                       slew, i,
                       fbits(thrust.x), fbits(thrust.y), fbits(thrust.z), fbits(hrate),
                       fbits(body_euler.x), fbits(body_euler.y), fbits(body_euler.z),
                       fbits(euler.x), fbits(euler.y), fbits(euler.z),
                       fbits(ang_vel.x), fbits(ang_vel.y), fbits(ang_vel.z),
                       fbits(rate.x), fbits(rate.y), fbits(rate.z));
            }
        }
    }

    // ---- a thrust vector with a heading ANGLE and a feedforward rate ----
    //
    // The only path where the yaw shaper gets an angle error and a rate at
    // once, so the sequence keeps both non-zero rather than exercising them
    // one at a time.
    printf("#thrustangle\n");
    printf("step,tx,ty,tz,hangle,hrate,body_r,body_p,body_y,"
           "targ_r,targ_p,targ_y,av_x,av_y,av_z,rate_x,rate_y,rate_z\n");
    {
        static Probe ta(view, motors);
        ta._dt_s = 0.0025f;
        ta._rate_rp_tc = 0.15f;
        ta._rate_y_tc = 0.25f;
        ta._ang_vel_roll_max_degs.set(220.0f);
        ta._ang_vel_pitch_max_degs.set(140.0f);
        ta._ang_vel_yaw_max_degs.set(120.0f);
        ta.reset_target_and_rate(true);

        Quaternion offset;
        offset.from_euler(-0.20f, 0.25f, -0.50f);
        ta._attitude_target = offset;

        const int STEPS = 400;
        for (int i = 0; i < STEPS; i++) {
            const float ts = i * 0.0025f;

            const float lean = 0.30f - 0.25f * ts;
            const float azim = -1.5f * ts;
            Vector3f thrust{sinf(lean) * cosf(azim),
                            sinf(lean) * sinf(azim),
                            -cosf(lean)};

            // The commanded heading walks away from the target's own, so the
            // error stays live instead of collapsing in the first few steps.
            const float hangle = 0.9f + 0.8f * ts;
            const float hrate = ts < 0.5f ? 0.4f : -0.6f;

            ta.input_thrust_vector_heading_rad(thrust, hangle, hrate);

            Quaternion body_now;
            view.get_quat_body_to_ned(body_now);
            Vector3f body_euler;
            body_now.to_euler(body_euler);

            const Vector3f euler = ta._euler_angle_target_rad;
            const Vector3f ang_vel = ta.get_attitude_target_ang_vel();
            const Vector3f rate = ta.rate_bf_targets();

            printf("%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", i,
                   fbits(thrust.x), fbits(thrust.y), fbits(thrust.z),
                   fbits(hangle), fbits(hrate),
                   fbits(body_euler.x), fbits(body_euler.y), fbits(body_euler.z),
                   fbits(euler.x), fbits(euler.y), fbits(euler.z),
                   fbits(ang_vel.x), fbits(ang_vel.y), fbits(ang_vel.z),
                   fbits(rate.x), fbits(rate.y), fbits(rate.z));
        }
    }

    // ---- a full quaternion demand with a body-frame rate ----
    //
    // The desired quaternion is advanced by the call, so it is recorded after
    // each step: the mutation is part of what the port has to reproduce, not
    // an incidental detail of the caller.
    printf("#quatinput\n");
    printf("step,wx,wy,wz,des_w,des_x,des_y,des_z,body_r,body_p,body_y,"
           "targ_r,targ_p,targ_y,av_x,av_y,av_z,rate_x,rate_y,rate_z\n");
    {
        static Probe qi(view, motors);
        qi._dt_s = 0.0025f;
        qi._rate_rp_tc = 0.15f;
        qi._rate_y_tc = 0.25f;
        qi._ang_vel_roll_max_degs.set(220.0f);
        qi._ang_vel_pitch_max_degs.set(140.0f);
        qi._ang_vel_yaw_max_degs.set(120.0f);
        qi.reset_target_and_rate(true);

        Quaternion offset;
        offset.from_euler(0.15f, -0.30f, 0.80f);
        qi._attitude_target = offset;

        Quaternion desired;
        desired.from_euler(-0.10f, 0.20f, -0.35f);

        const int STEPS = 400;
        for (int i = 0; i < STEPS; i++) {
            const float ts = i * 0.0025f;

            // The yaw component exceeds ATC_RATE_Y_MAX (120 deg/s = 2.09) for
            // part of the run, and roll and pitch together exceed the
            // elliptical roll/pitch bound, so ang_vel_limit does real work on
            // both of its branches rather than passing the input through.
            Vector3f w{3.0f * sinf(6.0f * ts),
                       2.5f * cosf(4.0f * ts),
                       ts < 0.5f ? 2.6f : -1.0f};

            qi.input_quaternion(desired, w);

            Quaternion body_now;
            view.get_quat_body_to_ned(body_now);
            Vector3f body_euler;
            body_now.to_euler(body_euler);

            const Vector3f euler = qi._euler_angle_target_rad;
            const Vector3f ang_vel = qi.get_attitude_target_ang_vel();
            const Vector3f rate = qi.rate_bf_targets();

            printf("%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", i,
                   fbits(w.x), fbits(w.y), fbits(w.z),
                   fbits(desired.q1), fbits(desired.q2),
                   fbits(desired.q3), fbits(desired.q4),
                   fbits(body_euler.x), fbits(body_euler.y), fbits(body_euler.z),
                   fbits(euler.x), fbits(euler.y), fbits(euler.z),
                   fbits(ang_vel.x), fbits(ang_vel.y), fbits(ang_vel.z),
                   fbits(rate.x), fbits(rate.y), fbits(rate.z));
        }
    }

    // ---- the roll/pitch rate predictor ----
    //
    // A pure function of its arguments, so this is a sweep rather than a
    // sequence: each row carries the state in and the state out. The dt column
    // records what the caller passed, which upstream ignores -- see D-025.
    printf("#predictor\n");
    printf("idx,err_x,err_y,in_vel_x,in_vel_y,in_acc_x,in_acc_y,dt,"
           "out_vel_x,out_vel_y,out_acc_x,out_acc_y\n");
    {
        static Probe pr(view, motors);
        pr._dt_s = 0.0025f;
        pr._rate_rp_tc = 0.15f;
        pr._rate_y_tc = 0.25f;
        pr._ang_vel_roll_max_degs.set(220.0f);
        pr._ang_vel_pitch_max_degs.set(140.0f);
        pr._ang_vel_yaw_max_degs.set(120.0f);

        // Errors from below the shaper's linear region out past the rate
        // limit, and starting states both with and against the error.
        const float errors[] = {0.0f, 0.002f, -0.05f, 0.4f, -1.2f, 3.0f, -3.3f};
        const float vels[] = {0.0f, 0.8f, -1.5f};
        const float accels[] = {0.0f, 5.0f, -12.0f};

        int idx = 0;
        for (unsigned a = 0; a < sizeof(errors)/sizeof(errors[0]); a++) {
            for (unsigned b = 0; b < sizeof(errors)/sizeof(errors[0]); b++) {
                for (unsigned c = 0; c < sizeof(vels)/sizeof(vels[0]); c++) {
                    for (unsigned d = 0; d < sizeof(accels)/sizeof(accels[0]); d++) {
                        const Vector2f err{errors[a], errors[b]};
                        Vector2f vel{vels[c], vels[(c + 1) % 3]};
                        Vector2f acc{accels[d], accels[(d + 2) % 3]};
                        const Vector2f in_vel = vel;
                        const Vector2f in_acc = acc;

                        pr.command_model_rate_predictor(err, vel, acc, 0.0025f);

                        printf("%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", idx++,
                               fbits(err.x), fbits(err.y),
                               fbits(in_vel.x), fbits(in_vel.y),
                               fbits(in_acc.x), fbits(in_acc.y),
                               fbits(0.0025f),
                               fbits(vel.x), fbits(vel.y),
                               fbits(acc.x), fbits(acc.y));
                    }
                }
            }
        }
    }

    // ---- rate-only acro ----
    //
    // Never runs the attitude controller: the shaped rate goes straight to the
    // rate loop and the target is dragged to wherever the aircraft is. The
    // recorded output is _ang_vel_body_rads, not a controller result.
    printf("#acro2\n");
    printf("step,roll_rate,pitch_rate,yaw_rate,body_r,body_p,body_y,"
           "targ_r,targ_p,targ_y,av_x,av_y,av_z,out_x,out_y,out_z\n");
    {
        static Probe a2(view, motors);
        a2._dt_s = 0.0025f;
        a2._rate_rp_tc = 0.15f;
        a2._rate_y_tc = 0.25f;
        a2._ang_vel_roll_max_degs.set(220.0f);
        a2._ang_vel_pitch_max_degs.set(140.0f);
        a2._ang_vel_yaw_max_degs.set(120.0f);
        a2.reset_target_and_rate(true);

        const int STEPS = 400;
        for (int i = 0; i < STEPS; i++) {
            const float ts = i * 0.0025f;
            const float rr = ts < 0.35f ? 1.1f : -0.7f;
            const float pr = -0.5f + 1.4f * ts;
            const float yr = 0.8f * sinf(5.0f * ts);

            a2.input_rate_bf_roll_pitch_yaw_2_rads(rr, pr, yr);

            Quaternion body_now;
            view.get_quat_body_to_ned(body_now);
            Vector3f body_euler;
            body_now.to_euler(body_euler);

            const Vector3f euler = a2._euler_angle_target_rad;
            const Vector3f ang_vel = a2.get_attitude_target_ang_vel();
            const Vector3f out = a2.rate_bf_targets();

            printf("%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", i,
                   fbits(rr), fbits(pr), fbits(yr),
                   fbits(body_euler.x), fbits(body_euler.y), fbits(body_euler.z),
                   fbits(euler.x), fbits(euler.y), fbits(euler.z),
                   fbits(ang_vel.x), fbits(ang_vel.y), fbits(ang_vel.z),
                   fbits(out.x), fbits(out.y), fbits(out.z));
        }
    }

    // ---- acro with integrated rate error ----
    //
    // The gyro is the integrator's other input, so it is scripted rather than
    // left at zero: with a zero gyro the rate error equals the commanded rate
    // and the anti-windup clamp is the only thing that ever bounds it, which
    // exercises far less than the real loop does.
    printf("#acro3\n");
    printf("step,roll_rate,pitch_rate,yaw_rate,gx,gy,gz,body_r,body_p,body_y,"
           "targ_r,targ_p,targ_y,err_w,err_x,err_y,err_z,out_x,out_y,out_z\n");
    {
        static Probe a3(view, motors);
        a3._dt_s = 0.0025f;
        a3._rate_rp_tc = 0.15f;
        a3._rate_y_tc = 0.25f;
        a3._ang_vel_roll_max_degs.set(220.0f);
        a3._ang_vel_pitch_max_degs.set(140.0f);
        a3._ang_vel_yaw_max_degs.set(120.0f);
        a3.reset_target_and_rate(true);
        a3._attitude_ang_error.initialise();

        const int STEPS = 500;
        for (int i = 0; i < STEPS; i++) {
            const float ts = i * 0.0025f;
            const float rr = ts < 1.0f ? 1.3f : -0.9f;
            const float pr = 0.7f * cosf(3.0f * ts);
            const float yr = ts < 0.8f ? 0.5f : -0.4f;

            // A gyro well short of the command, held one-signed long enough
            // that the integrated error runs past the 30-degree anti-windup
            // clamp instead of merely approaching it.
            const Vector3f gyro{0.15f * rr, 0.80f * pr, 0.30f * yr};
            a3._rate_gyro_rads = gyro;

            a3.input_rate_bf_roll_pitch_yaw_3_rads(rr, pr, yr);

            Quaternion body_now;
            view.get_quat_body_to_ned(body_now);
            Vector3f body_euler;
            body_now.to_euler(body_euler);

            const Vector3f euler = a3._euler_angle_target_rad;
            const Quaternion err = a3._attitude_ang_error;
            const Vector3f out = a3.rate_bf_targets();

            printf("%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", i,
                   fbits(rr), fbits(pr), fbits(yr),
                   fbits(gyro.x), fbits(gyro.y), fbits(gyro.z),
                   fbits(body_euler.x), fbits(body_euler.y), fbits(body_euler.z),
                   fbits(euler.x), fbits(euler.y), fbits(euler.z),
                   fbits(err.q1), fbits(err.q2), fbits(err.q3), fbits(err.q4),
                   fbits(out.x), fbits(out.y), fbits(out.z));
        }
    }

    return 0;
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/attitude_parity/vehicle")
    build(HARNESS, objects, BUILD,
          "AC_AttitudeControl/AC_AttitudeControl.cpp",
          link_flags=vehicle_link.LINK_FLAGS)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines() if l and not l.startswith("body_r"))
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
