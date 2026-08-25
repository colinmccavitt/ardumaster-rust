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

class MotorProbe : public AP_MotorsMatrix {
public:
    using AP_MotorsMatrix::AP_MotorsMatrix;
    using AP_MotorsMulticopter::_throttle_thrust_max;
    using AP_Motors::_throttle_out;
    using AP_Motors::_throttle_filter;
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
    printf("#config\n");
    printf("dt,angle_limit_tc,thr_mix_min,thr_mix_max,throttle_hover,"
           "throttle_thrust_max,cos_tilt,angle_boost_enabled\n");
    printf("%u,%u,%u,%u,%u,%u,%u,%d\n",
           fbits(att._dt_s),
           fbits(att._angle_limit_tc),
           fbits(att._thr_mix_min),
           fbits(att._thr_mix_max),
           fbits(motors.get_throttle_hover()),
           fbits(motors.get_throttle_thrust_max()),
           fbits(AP::ahrs().cos_pitch() * AP::ahrs().cos_roll()),
           1);

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
