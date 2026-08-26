"""Record Copter's AutoYaw mode selection, entry effects and rate source.

Three things are observed, each where it actually leaves the firmware:

  * default_mode(rtl) is a pure query and is called directly over every
    WP_YAW_BEHAVIOR value including out-of-range ones -- upstream's switch
    shares its default arm with LOOK_AT_NEXT_WP, so what an unrecognised
    value does is a fact about the code.

  * set_mode's per-mode initialisation is invisible in any return value, so
    the two fields it can touch are sentinel-filled and read back:
    _look_ahead_yaw_rad and _yaw_rate_rads. Every transition between all
    eleven modes is swept, which also captures the early return when the mode
    is unchanged -- a case that matters because it means re-selecting RATE
    does NOT re-zero the rate.

  * rate_rads assigns from a different source per mode, and three modes fall
    through without assigning at all. A sentinel rate distinguishes "assigned
    zero" from "left alone", which counting or reading zero could not.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/plane-fw-rust")
OUT = ROOT / "fixtures/copter_auto_yaw.csv"
BUILD = Path("/tmp/auto_yaw_parity/harness")

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

int main()
{
    AP::scheduler().init(nullptr, 0, 0);
    copter.allocate_motors();
    copter.motors->_throttle_hover_learn.set(0);
    setvbuf(stdout, NULL, _IOLBF, 0);

    Mode::AutoYaw &ay = Mode::auto_yaw;

    // ---- default_mode ----
    printf("#default_mode\n");
    printf("wp_yaw_behavior,rtl,mode\n");
    for (int b = -1; b < 6; b++) {
        for (unsigned rtl = 0; rtl < 2; rtl++) {
            copter.g.wp_yaw_behavior.set((int8_t)b);
            printf("%d,%d,%d\n", b, (int)rtl,
                   (int)ay.default_mode(rtl != 0));
        }
    }

    // ---- set_mode's initialisation ----
    //
    // Both fields it can touch are set to a sentinel first, so a field left
    // alone is distinguishable from one assigned the same value.
    printf("#set_mode\n");
    printf("from,to,look_ahead_changed,rate_changed,last_mode\n");
    {
        // A yaw the look-ahead seed cannot coincidentally equal.
        copter.ahrs._cos_yaw = 0.6f;
        copter.ahrs._sin_yaw = 0.8f;

        for (int from = 0; from <= 10; from++) {
            for (int to = 0; to <= 10; to++) {
                ay._mode = (Mode::AutoYaw::Mode)from;
                ay._last_mode = (Mode::AutoYaw::Mode)0;
                ay._look_ahead_yaw_rad = -7.5f;
                ay._yaw_rate_rads = -3.25f;

                ay.set_mode((Mode::AutoYaw::Mode)to);

                printf("%d,%d,%d,%d,%d\n", from, to,
                       fbits(ay._look_ahead_yaw_rad) == fbits(-7.5f) ? 0 : 1,
                       fbits(ay._yaw_rate_rads) == fbits(-3.25f) ? 0 : 1,
                       (int)ay._last_mode);
            }
        }
    }

    // ---- rate_rads ----
    //
    // The sentinel separates "assigned zero" from "left alone": three modes
    // fall through the switch without assigning.
    printf("#rate\n");
    printf("mode,rate_out,was_assigned\n");
    {
        // A distinctive rate on the position controller, so the mode that
        // reads it is separable from the modes that assign zero. Without
        // this both come out as zero and a port confusing them would pass.
        copter.pos_control->_yaw_rate_target_rads = 0.875f;

        for (int m = 0; m <= 10; m++) {
            ay._mode = (Mode::AutoYaw::Mode)m;
            ay._yaw_rate_rads = -3.25f;
            ay._pilot_yaw_rate_rads = 1.75f;

            const float out = ay.rate_rads();

            printf("%d,%u,%d\n", m, fbits(out),
                   fbits(out) == fbits(-3.25f) ? 0 : 1);
        }
    }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/auto_yaw_parity/vehicle")
    build(HARNESS, objects, BUILD, "ArduCopter/Copter.cpp",
          link_flags=vehicle_link.LINK_FLAGS)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
