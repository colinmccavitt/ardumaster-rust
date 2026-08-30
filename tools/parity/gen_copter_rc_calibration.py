"""Record the RC calibration check and the altitude disparity check.

rc_calibration_checks is recorded through the real firmware because the thing
worth pinning is not the returned bool -- it is how many messages arrive. A
channel that was never calibrated is wrong at both ends and upstream reports
both; a port returning at the first fault gives the same bool and half the
messages. The count is the observation.

The altitude disparity check needs the EKF's prediction-status flags and its
relative height, none of which a harness can produce, so it is covered by
reasoning in the test rather than recorded. Stated here so the fixture is not
read as covering it.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/ardumaster-rust")
OUT = ROOT / "fixtures/copter_rc_calibration.csv"
BUILD = Path("/tmp/rccal_parity/harness")

# The message crosses the boundary from AP_Arming.cpp as send_textv, not
# as check_failed -- see the module docstring.
SEND_TEXTV = "_ZN3GCS10send_textvE12MAV_SEVERITYPKcP13__va_list_tag"
SEND_TEXT = "_ZN3GCS9send_textE12MAV_SEVERITYPKcz"

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

static char g_first[160];
static int g_calls = 0;

static void record(const char *fmt, va_list ap)
{
    char buf[160];
    vsnprintf(buf, sizeof(buf), fmt, ap);
    if (g_calls == 0) {
        // check_failed prefixes "PreArm: "; the reason is what follows.
        const char *tail = strstr(buf, ": ");
        snprintf(g_first, sizeof(g_first), "%s", tail ? tail + 2 : buf);
    }
    g_calls++;
}

extern "C" void __wrap__ZN3GCS10send_textvE12MAV_SEVERITYPKcP13__va_list_tag(
    void *self, int severity, const char *fmt, va_list ap);
extern "C" void __wrap__ZN3GCS10send_textvE12MAV_SEVERITYPKcP13__va_list_tag(
    void *self, int severity, const char *fmt, va_list ap)
{ (void)self; (void)severity; record(fmt, ap); }

extern "C" void __wrap__ZN3GCS9send_textE12MAV_SEVERITYPKcz(
    void *self, int severity, const char *fmt, ...);
extern "C" void __wrap__ZN3GCS9send_textE12MAV_SEVERITYPKcz(
    void *self, int severity, const char *fmt, ...)
{ (void)self; (void)severity; va_list ap; va_start(ap, fmt); record(fmt, ap); va_end(ap); }

int main()
{
    AP::scheduler().init(nullptr, 0, 0);
    copter.allocate_motors();
    copter.motors->_throttle_hover_learn.set(0);
    setvbuf(stdout, NULL, _IOLBF, 0);

    copter.channel_roll = rc().channel(0);
    copter.channel_pitch = rc().channel(1);
    copter.channel_throttle = rc().channel(2);
    copter.channel_yaw = rc().channel(3);

    AP_Arming_Copter &arming = copter.arming;

    printf("#rc_calibration\n");
    printf("idx,enabled,roll_min,roll_max,pitch_min,pitch_max,"
           "throttle_min,throttle_max,yaw_min,yaw_max,passed,calls,first\n");

    // Either end of each limit, and a channel that is wrong at both ends.
    const uint16_t mins[] = {1000, 1300, 1301};
    const uint16_t maxs[] = {2000, 1700, 1699};

    int idx = 0;
    for (unsigned en = 0; en < 2; en++)
      for (unsigned rmin = 0; rmin < 3; rmin++)
        for (unsigned rmax = 0; rmax < 3; rmax++)
          for (unsigned pmin = 0; pmin < 3; pmin++)
            for (unsigned tmax = 0; tmax < 3; tmax++) {
                // Check::RC is 1<<6, and checks_to_skip is a SKIP mask.
                arming.checks_to_skip.set(en ? 0 : (1 << 6));

                copter.channel_roll->radio_min.set(mins[rmin]);
                copter.channel_roll->radio_max.set(maxs[rmax]);
                copter.channel_pitch->radio_min.set(mins[pmin]);
                copter.channel_pitch->radio_max.set(2000);
                copter.channel_throttle->radio_min.set(1000);
                copter.channel_throttle->radio_max.set(maxs[tmax]);
                copter.channel_yaw->radio_min.set(1000);
                copter.channel_yaw->radio_max.set(2000);

                g_first[0] = '\0';
                g_calls = 0;
                const bool ok = arming.rc_calibration_checks(true);

                printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s\n", idx++,
                       (int)arming.check_enabled(AP_Arming::Check::RC),
                       (int)mins[rmin], (int)maxs[rmax],
                       (int)mins[pmin], 2000,
                       1000, (int)maxs[tmax],
                       1000, 2000,
                       ok ? 1 : 0, g_calls,
                       g_calls ? g_first : "-");
            }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/rccal_parity/vehicle")
    flags = list(vehicle_link.LINK_FLAGS) + [
        "-Wl,--wrap=" + SEND_TEXTV,
        "-Wl,--wrap=" + SEND_TEXT,
    ]
    build(HARNESS, objects, BUILD, "ArduCopter/Copter.cpp", link_flags=flags)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
