"""Record AutoYaw's fixed-yaw command, look-ahead and arrival test.

set_fixed_yaw_rad is the interesting one. Relative and absolute commands mean
different things, and the direction argument overrides the shorter way round
for an absolute command -- so the sweep covers both forms, all three
directions, and angles either side of the wrap.

set_rate_rad is recorded because its ordering is silent if got wrong:
set_mode(RATE) zeroes the stored rate and the assignment follows it. A port
doing those in the other order leaves the aircraft turning at zero, and
nothing in the return value would say so. The recorded rate after the call is
what catches it.

look_ahead_yaw_rad needs a velocity the AHRS will report, so
get_velocity_NED is wrapped -- it is an undefined reference in autoyaw.cpp.o
(checked first). position_ok is wrapped too, for the same reason as in
gen_mode_entry.py.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/ardumaster-rust")
OUT = ROOT / "fixtures/copter_auto_yaw3.csv"
BUILD = Path("/tmp/ay3_parity/harness")

POSITION_OK = "_ZNK6Copter11position_okEv"
VELOCITY_NED = "_ZNK7AP_AHRS16get_velocity_NEDER7Vector3IfE"

HARNESS = r'''
#include <AP_HAL/AP_HAL.h>

#define private public
#define protected public
#include "/srv/ardumaster/upstream/plane-4.7.0/ArduCopter/Copter.h"
#undef private
#undef protected

#include <cstdarg>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

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

static bool g_position_ok = true;

extern "C" bool __wrap__ZNK6Copter11position_okEv(const void *self);
extern "C" bool __wrap__ZNK6Copter11position_okEv(const void *self)
{ (void)self; return g_position_ok; }

// The velocity comes from the EKF, so there is no member to assign. Injected
// the same way the rangefinder and terrain are in gen_alt_above_ground.py.
static bool g_vel_valid = true;
static float g_vel_n = 0.0f, g_vel_e = 0.0f;

extern "C" bool __wrap__ZNK7AP_AHRS16get_velocity_NEDER7Vector3IfE(
    const void *self, Vector3f &vec);
extern "C" bool __wrap__ZNK7AP_AHRS16get_velocity_NEDER7Vector3IfE(
    const void *self, Vector3f &vec)
{
    (void)self;
    if (!g_vel_valid) {
        return false;
    }
    vec.x = g_vel_n;
    vec.y = g_vel_e;
    vec.z = 0.0f;
    return true;
}

int main()
{
    AP::scheduler().init(nullptr, 0, 0);
    copter.allocate_motors();
    copter.motors->_throttle_hover_learn.set(0);
    setvbuf(stdout, NULL, _IOLBF, 0);

    Mode::AutoYaw &ay = Mode::auto_yaw;
    copter.flightmode = (Mode *)&copter.mode_auto;

    // ---- set_fixed_yaw_rad ----
    //
    // Both command forms, all three directions, and angles either side of the
    // wrap. The current target angle is set explicitly because an absolute
    // command is relative to it.
    printf("#fixed_yaw\n");
    printf("idx,angle,current,direction,relative,mode_before,"
           "slew_req,slew_max,offset_out,slew_out,angle_out\n");
    {
        const float angles[] = {0.0f, 0.5f, 3.0f, -0.5f, -3.0f, 6.0f};
        const float currents[] = {0.0f, 1.5f, -2.5f};
        const int8_t dirs[] = {-1, 0, 1};
        const float rates[] = {-1.0f, 0.0f, 0.05f, 100.0f};
        int idx = 0;
        for (unsigned a = 0; a < 6; a++)
          for (unsigned c = 0; c < 3; c++)
            for (unsigned d = 0; d < 3; d++)
              for (unsigned rel = 0; rel < 2; rel++)
                for (unsigned r = 0; r < 4; r++) {
                    // Start from a mode that is not HOLD, so the relative
                    // branch does not re-seed the angle from the AHRS -- that
                    // seeding is recorded separately below.
                    ay._mode = (Mode::AutoYaw::Mode)2;   // ROI
                    ay._yaw_angle_rad = currents[c];
                    ay._fixed_yaw_offset_rad = 999.0f;
                    ay._fixed_yaw_slewrate_rads = 999.0f;

                    ay.set_fixed_yaw_rad(angles[a], rates[r], dirs[d], rel != 0);

                    printf("%d,%u,%u,%d,%d,%d,%u,%u,%u,%u,%u\n", idx++,
                           fbits(angles[a]), fbits(currents[c]),
                           (int)dirs[d], (int)rel, 2,
                           fbits(rates[r]),
                           fbits(copter.attitude_control->get_slew_yaw_max_rads()),
                           fbits(ay._fixed_yaw_offset_rad),
                           fbits(ay._fixed_yaw_slewrate_rads),
                           fbits(ay._yaw_angle_rad));
                }
    }

    // ---- set_rate_rad's ordering ----
    //
    // set_mode(RATE) zeroes the stored rate and the assignment follows. If a
    // port does those in the other order the rate is lost, and only the value
    // after the call shows it.
    printf("#set_rate\n");
    printf("from_mode,requested,rate_after,mode_after\n");
    {
        const float rates[] = {0.0f, 0.75f, -0.75f};
        for (int m = 0; m <= 10; m++)
          for (unsigned r = 0; r < 3; r++) {
              ay._mode = (Mode::AutoYaw::Mode)m;
              ay._yaw_rate_rads = -9.5f;
              ay.set_rate_rad(rates[r]);
              printf("%d,%u,%u,%d\n", m, fbits(rates[r]),
                     fbits(ay._yaw_rate_rads), (int)ay._mode);
          }
    }

    // ---- reached_fixed_yaw_target ----
    printf("#reached\n");
    printf("mode,offset,angle,measured_yaw,reached\n");
    {
        const float offsets[] = {0.0f, 0.001f, -0.5f};
        const float angles[] = {0.0f, 0.03f, 0.04f, 1.0f};
        for (int m = 0; m <= 10; m++)
          for (unsigned o = 0; o < 3; o++)
            for (unsigned a = 0; a < 4; a++) {
                ay._mode = (Mode::AutoYaw::Mode)m;
                ay._fixed_yaw_offset_rad = offsets[o];
                ay._yaw_angle_rad = angles[a];
                // The measured heading is zero: cos_yaw 1, sin_yaw 0.
                copter.ahrs._cos_yaw = 1.0f;
                copter.ahrs._sin_yaw = 0.0f;

                printf("%d,%u,%u,%u,%d\n", m, fbits(offsets[o]),
                       fbits(angles[a]), fbits(0.0f),
                       (int)ay.reached_fixed_yaw_target());
            }
    }

    // ---- look_ahead_yaw_rad ----
    printf("#look_ahead\n");
    printf("idx,held,position_ok,vel_n,vel_e,out\n");
    {
        const float vels[][2] = {
            {0.0f, 0.0f}, {0.5f, 0.5f}, {0.7071f, 0.7071f},
            {1.0f, 0.0f}, {0.0f, -2.0f}, {-3.0f, 4.0f},
        };
        int idx = 0;
        for (unsigned v = 0; v < 6; v++)
          for (unsigned ok = 0; ok < 2; ok++) {
              g_position_ok = (ok != 0);
              ay._look_ahead_yaw_rad = -1.25f;
              g_vel_valid = true;
              g_vel_n = vels[v][0];
              g_vel_e = vels[v][1];

              const float out = ay.look_ahead_yaw_rad();

              printf("%d,%u,%d,%u,%u,%u\n", idx++,
                     fbits(-1.25f), (int)(ok != 0),
                     fbits(vels[v][0]), fbits(vels[v][1]), fbits(out));
          }
    }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/ay3_parity/vehicle")
    flags = list(vehicle_link.LINK_FLAGS) + [
        "-Wl,--wrap=" + POSITION_OK,
        "-Wl,--wrap=" + VELOCITY_NED,
    ]
    build(HARNESS, objects, BUILD, "ArduCopter/Copter.cpp", link_flags=flags)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
