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

    // The gains this controller is running, so the port can be driven with
    // exactly the same ones rather than with a hand-copied guess.
    printf("#gains\n");
    printf("angle_p_roll,angle_p_pitch,angle_p_yaw,accel_roll,accel_pitch,"
           "accel_yaw,rate_yaw_kp,use_sqrt,dt\n");
    printf("%u,%u,%u,%u,%u,%u,%u,%d,%u\n",
           fbits(att.get_angle_roll_p().kP()),
           fbits(att.get_angle_pitch_p().kP()),
           fbits(att.get_angle_yaw_p().kP()),
           fbits(att.get_accel_roll_max_radss()),
           fbits(att.get_accel_pitch_max_radss()),
           fbits(att.get_accel_yaw_max_radss()),
           fbits(att.get_rate_yaw_pid().kP()),
           att.get_bf_feedforward() ? 1 : 0,
           fbits(0.0025f));

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
