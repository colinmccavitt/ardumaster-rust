"""Record AutoYaw's remaining setters and the ROI bearing.

The two setters differ in one way that matters: set_yaw_angle_offset_deg
wraps to 0..2pi and the fixed-yaw path wraps to -pi..pi, so the same physical
heading comes out as different numbers. Both are swept across the wrap.

roi_yaw_rad needs the vehicle's position relative to the EKF origin, which
comes through AP_AHRS::get_relative_position_NE_origin_float -- an undefined
reference in autoyaw.cpp.o, so it is wrapped and answered by the sweep. Its
no-position branch returns the attitude controller's standing target, which
is set explicitly so it is distinguishable from zero.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/plane-fw-rust")
OUT = ROOT / "fixtures/copter_auto_yaw4.csv"
BUILD = Path("/tmp/ay4_parity/harness")

REL_POS = "_ZNK7AP_AHRS37get_relative_position_NE_origin_floatER7Vector2IfE"

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

static bool g_pos_valid = true;
static float g_pos_n = 0.0f, g_pos_e = 0.0f;

extern "C" bool __wrap__ZNK7AP_AHRS37get_relative_position_NE_origin_floatER7Vector2IfE(
    const void *self, Vector2f &vec);
extern "C" bool __wrap__ZNK7AP_AHRS37get_relative_position_NE_origin_floatER7Vector2IfE(
    const void *self, Vector2f &vec)
{
    (void)self;
    if (!g_pos_valid) {
        return false;
    }
    vec.x = g_pos_n;
    vec.y = g_pos_e;
    return true;
}

int main()
{
    AP::scheduler().init(nullptr, 0, 0);
    copter.allocate_motors();
    copter.motors->_throttle_hover_learn.set(0);
    setvbuf(stdout, NULL, _IOLBF, 0);

    Mode::AutoYaw &ay = Mode::auto_yaw;

    // ---- set_yaw_angle_and_rate_rad ----
    printf("#angle_and_rate\n");
    printf("idx,angle_in,rate_in,angle_out,rate_out,mode_out\n");
    {
        const float angles[] = {0.0f, 1.5f, -1.5f, 4.0f, -4.0f, 7.0f};
        const float rates[] = {0.0f, 0.5f, -0.5f};
        int idx = 0;
        for (unsigned a = 0; a < 6; a++)
          for (unsigned r = 0; r < 3; r++) {
              ay._mode = (Mode::AutoYaw::Mode)0;   // HOLD
              ay._yaw_angle_rad = -9.0f;
              ay._yaw_rate_rads = -9.0f;

              ay.set_yaw_angle_and_rate_rad(angles[a], rates[r]);

              printf("%d,%u,%u,%u,%u,%d\n", idx++,
                     fbits(angles[a]), fbits(rates[r]),
                     fbits(ay._yaw_angle_rad), fbits(ay._yaw_rate_rads),
                     (int)ay._mode);
          }
    }

    // ---- set_yaw_angle_offset_deg ----
    //
    // Across the wrap in both directions: this one wraps to 0..2pi where the
    // fixed-yaw path wraps to -pi..pi.
    printf("#angle_offset\n");
    printf("idx,current,offset_deg,angle_out,rate_out,mode_out\n");
    {
        const float currents[] = {0.0f, 3.0f, 6.0f, -1.0f};
        const float offsets[] = {0.0f, 45.0f, 180.0f, 350.0f, -45.0f, -400.0f};
        int idx = 0;
        for (unsigned c = 0; c < 4; c++)
          for (unsigned o = 0; o < 6; o++) {
              ay._mode = (Mode::AutoYaw::Mode)0;
              ay._yaw_angle_rad = currents[c];
              ay._yaw_rate_rads = -9.0f;

              ay.set_yaw_angle_offset_deg(offsets[o]);

              printf("%d,%u,%u,%u,%u,%d\n", idx++,
                     fbits(currents[c]), fbits(offsets[o]),
                     fbits(ay._yaw_angle_rad), fbits(ay._yaw_rate_rads),
                     (int)ay._mode);
          }
    }

    // ---- roi_yaw_rad ----
    printf("#roi\n");
    printf("idx,pos_valid,pos_n,pos_e,roi_n,roi_e,att_target,out\n");
    {
        const float positions[][2] = {
            {0.0f, 0.0f}, {10.0f, 0.0f}, {-5.0f, 7.5f},
        };
        const float rois[][2] = {
            {0.0f, 0.0f}, {100.0f, 0.0f}, {0.0f, 100.0f},
            {-50.0f, -50.0f}, {10.0f, 0.0f},
        };
        int idx = 0;
        for (unsigned p = 0; p < 3; p++)
          for (unsigned r = 0; r < 5; r++)
            for (unsigned v = 0; v < 2; v++) {
                g_pos_valid = (v != 0);
                g_pos_n = positions[p][0];
                g_pos_e = positions[p][1];
                ay.roi_ned_m.x = rois[r][0];
                ay.roi_ned_m.y = rois[r][1];
                ay.roi_ned_m.z = 0.0f;

                const float att = copter.attitude_control->get_att_target_euler_rad().z;
                const float out = ay.roi_yaw_rad();

                printf("%d,%d,%u,%u,%u,%u,%u,%u\n", idx++, (int)(v != 0),
                       fbits(positions[p][0]), fbits(positions[p][1]),
                       fbits(rois[r][0]), fbits(rois[r][1]),
                       fbits(att), fbits(out));
            }
    }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/ay4_parity/vehicle")
    flags = list(vehicle_link.LINK_FLAGS) + ["-Wl,--wrap=" + REL_POS]
    build(HARNESS, objects, BUILD, "ArduCopter/Copter.cpp", link_flags=flags)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
