"""Dump AC_PosControl's NE kinematic layer from the real firmware.

A probe over AC_PosControl, so the controller's own state is driven rather
than reconstructed. The three input entry points carry state forward, so they
are recorded as trajectories; the limit derivation and the stopping point are
pure and are swept.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/plane-fw-rust")
OUT = ROOT / "fixtures/pos_control_ne.csv"
BUILD = Path("/tmp/posne_parity/harness")

HARNESS = r'''
#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_AHRS/AP_AHRS_View.h>
#include <AP_Motors/AP_MotorsMatrix.h>
#include <AC_AttitudeControl/AC_AttitudeControl_Multi.h>
#include <AC_AttitudeControl/AC_PosControl.h>
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

extern const AP_HAL::HAL &hal;

static uint32_t fbits(float f)
{
    uint32_t u;
    memcpy(&u, &f, sizeof(u));
    return u;
}

class AttProbe : public AC_AttitudeControl_Multi {
public:
    using AC_AttitudeControl_Multi::AC_AttitudeControl_Multi;
    using AC_AttitudeControl::_ang_vel_roll_max_degs;
    using AC_AttitudeControl::_ang_vel_pitch_max_degs;
};

class PosProbe : public AC_PosControl {
public:
    using AC_PosControl::AC_PosControl;
    using AC_PosControl::_dt_s;
    using AC_PosControl::_pos_desired_ned_m;
    using AC_PosControl::_vel_desired_ned_ms;
    using AC_PosControl::_accel_desired_ned_mss;
    using AC_PosControl::_limit_vector_ned;
    using AC_PosControl::_pos_estimate_ned_m;
    using AC_PosControl::_vel_estimate_ned_ms;
    using AC_PosControl::_pos_offset_ned_m;
    using AC_PosControl::_vel_offset_ned_ms;
    using AC_PosControl::_vel_max_ne_ms;
    using AC_PosControl::_accel_max_ne_mss;
    using AC_PosControl::_jerk_max_ne_msss;
    using AC_PosControl::_shaping_jerk_ne_msss;
    using AC_PosControl::_p_pos_ne_m;
    using AC_PosControl::_p_pos_d_m;
    using AC_PosControl::_pid_accel_d_m;
    using AC_PosControl::_vel_max_down_ms;
    using AC_PosControl::_vel_max_up_ms;
    using AC_PosControl::_accel_max_d_mss;
    using AC_PosControl::_jerk_max_d_msss;
    using AC_PosControl::_shaping_jerk_d_msss;
    using AC_PosControl::calculate_overspeed_gain;
};

int main()
{
    AP::scheduler().init(nullptr, 0, 0);

    static AP_MotorsMatrix motors(400.0f);
    motors.init(AP_Motors::MOTOR_FRAME_QUAD, AP_Motors::MOTOR_FRAME_TYPE_X);
    static AP_AHRS_View view(AP::ahrs(), ROTATION_NONE);
    static AttProbe att(view, motors);
    static PosProbe pos(view, motors, att);

    pos._dt_s = 0.0025f;

    // ---- the limit derivation ----
    //
    // Swept over the attitude controller's capability, because the jerk limit
    // is derived from it: a multirotor changes horizontal acceleration by
    // changing lean, so the rate it can lean at bounds the jerk it can make.
    printf("#limits\n");
    printf("idx,speed,accel,shaping_jerk,ang_vel_r,ang_vel_p,accel_r,accel_p,ff,"
           "vel_max,accel_max,jerk_max\n");
    {
        const float speeds[] = {-7.5f, 0.0f, 5.0f, 20.0f};
        const float accels[] = {-2.5f, 0.0f, 2.5f, 9.0f};
        const float jerks[] = {0.0f, 5.0f, 50.0f};
        const float rates[] = {0.0f, 0.35f, 3.0f};
        const float angaccels[] = {0.0f, 2.0f, 18.0f};
        int idx = 0;
        for (unsigned a = 0; a < 4; a++)
          for (unsigned b = 0; b < 4; b++)
            for (unsigned c = 0; c < 3; c++)
              for (unsigned d = 0; d < 3; d++)
                for (unsigned e = 0; e < 3; e++)
                  for (int ff = 0; ff <= 1; ff++) {
                    // The attitude limits the derivation reads. No setter
                    // exists for the rate maxima, so the probe exposes them.
                    att.bf_feedforward(ff != 0);
                    att._ang_vel_roll_max_degs.set(degrees(rates[d]));
                    att._ang_vel_pitch_max_degs.set(degrees(rates[d] * 1.2f));
                    att.set_accel_roll_max_radss(angaccels[e]);
                    att.set_accel_pitch_max_radss(angaccels[e] * 0.8f);

                    pos._shaping_jerk_ne_msss.set(jerks[c]);
                    pos.NE_set_max_speed_accel_m(speeds[a], accels[b]);

                    printf("%d,%u,%u,%u,%u,%u,%u,%u,%d,%u,%u,%u\n", idx++,
                           fbits(speeds[a]), fbits(accels[b]), fbits(jerks[c]),
                           fbits(att.get_ang_vel_roll_max_rads()),
                           fbits(att.get_ang_vel_pitch_max_rads()),
                           fbits(att.get_accel_roll_max_radss()),
                           fbits(att.get_accel_pitch_max_radss()),
                           ff,
                           fbits(pos._vel_max_ne_ms),
                           fbits(pos._accel_max_ne_mss),
                           fbits(pos._jerk_max_ne_msss));
                  }
    }

    // ---- the vertical limit derivation ----
    //
    // Zero means leave unchanged here, the opposite of the horizontal setter,
    // so the sweep passes zeros deliberately and carries state between rows.
    // The jerk bound comes from the acceleration PID's filter cutoffs, so
    // those are swept too.
    printf("#dlimits\n");
    printf("idx,descent,climb,accel,shaping_jerk,filt_t,filt_e,"
           "vel_down,vel_up,accel_max,jerk_max\n");
    {
        pos._vel_max_down_ms = 2.0f;
        pos._vel_max_up_ms = 2.5f;
        pos._accel_max_d_mss = 2.5f;

        const float speeds[] = {0.0f, -3.5f, 6.0f};
        const float accels[] = {0.0f, -1.5f, 12.0f};
        const float jerks[] = {1.0f, 5.0f, 100.0f};
        const float filts[] = {0.0f, 0.5f, 5.0f, 40.0f};
        int idx = 0;
        for (unsigned a = 0; a < 3; a++)
          for (unsigned b = 0; b < 3; b++)
            for (unsigned c = 0; c < 3; c++)
              for (unsigned j = 0; j < 3; j++)
                for (unsigned ft = 0; ft < 4; ft++)
                  for (unsigned fe = 0; fe < 4; fe++) {
                    pos._shaping_jerk_d_msss.set(jerks[j]);
                    pos._pid_accel_d_m.filt_T_hz().set(filts[ft]);
                    pos._pid_accel_d_m.filt_E_hz().set(filts[fe]);

                    const float in_down = pos._vel_max_down_ms;
                    const float in_up = pos._vel_max_up_ms;
                    const float in_accel = pos._accel_max_d_mss;

                    pos.D_set_max_speed_accel_m(speeds[a], speeds[b], accels[c]);

                    printf("%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", idx++,
                           fbits(speeds[a]), fbits(speeds[b]), fbits(accels[c]),
                           fbits(jerks[j]), fbits(filts[ft]), fbits(filts[fe]),
                           fbits(pos._vel_max_down_ms), fbits(pos._vel_max_up_ms),
                           fbits(pos._accel_max_d_mss), fbits(pos._jerk_max_d_msss));

                    // Put them back, so each row starts from the same state
                    // and the leave-unchanged behaviour stays legible.
                    pos._vel_max_down_ms = in_down;
                    pos._vel_max_up_ms = in_up;
                    pos._accel_max_d_mss = in_accel;
                  }
    }

    // ---- the overspeed gain ----
    printf("#overspeed\n");
    printf("idx,vel_desired,vel_down,vel_up,gain\n");
    {
        const float downs[] = {0.0f, 1.5f, 4.0f};
        const float ups[] = {0.0f, 2.0f, 5.0f};
        int idx = 0;
        for (unsigned a = 0; a < 3; a++)
          for (unsigned b = 0; b < 3; b++)
            for (int v = -20; v <= 20; v++) {
                const float vd = v * 0.75f;
                pos._vel_max_down_ms = downs[a];
                pos._vel_max_up_ms = ups[b];
                pos._vel_desired_ned_ms.z = vd;
                printf("%d,%u,%u,%u,%u\n", idx++,
                       fbits(vd), fbits(downs[a]), fbits(ups[b]),
                       fbits(pos.calculate_overspeed_gain()));
            }
    }

    // ---- the vertical stopping point ----
    printf("#dstop\n");
    printf("idx,pos,pos_off,vel,vel_off,kp,accel_max,stop\n");
    {
        const float vels[] = {-9.0f, -2.0f, 0.0f, 1.0f, 9.0f};
        const float kps[] = {0.0f, 1.0f, 3.0f};
        const float accels[] = {0.0f, 2.5f, 10.0f};
        int idx = 0;
        for (unsigned a = 0; a < 5; a++)
          for (unsigned b = 0; b < 3; b++)
            for (unsigned c = 0; c < 3; c++) {
                pos._pos_estimate_ned_m.z = -12.5f;
                pos._pos_offset_ned_m.z = 1.5f;
                pos._vel_estimate_ned_ms.z = vels[a];
                pos._vel_offset_ned_ms.z = 0.25f;
                pos._p_pos_d_m.kP().set(kps[b]);
                pos._accel_max_d_mss = accels[c];

                postype_t stop = 0.0;
                pos.get_stopping_point_D_m(stop);

                printf("%d,%u,%u,%u,%u,%u,%u,%.17g\n", idx++,
                       fbits(-12.5f), fbits(1.5f), fbits(vels[a]), fbits(0.25f),
                       fbits(kps[b]), fbits(accels[c]), (double)stop);
            }
    }

    return 0;
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/posne_parity/vehicle")
    build(HARNESS, objects, BUILD, "AC_AttitudeControl/AC_PosControl.cpp",
          link_flags=vehicle_link.LINK_FLAGS)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
