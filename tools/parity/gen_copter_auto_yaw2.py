"""Record AutoYaw's heading mode, fixed slew, and angle-rate integration.

The heading mode is a pure function of the yaw mode and is read straight out
of get_heading's return value, so all eleven are swept.

The fixed slew and the angle-rate integration both read millis() for their
timestep, which a harness cannot set. They are driven instead by seeding
_last_update_ms relative to the clock the firmware will read, and the dt the
firmware actually used is recorded alongside the result rather than assumed --
the port is then compared against upstream's own dt, not against one the
harness hoped for.

get_heading also runs the pilot-override and weathervane paths. Both are
neutralised here rather than driven: the RC has never been seen, so the pilot
branch takes its else, and no weathervane is configured. Those two decisions
need their own harness with RC plumbing and are pinned by reasoning in the
test meanwhile, which says so.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/ardumaster-rust")
OUT = ROOT / "fixtures/copter_auto_yaw2.csv"
BUILD = Path("/tmp/ay2_parity/harness")

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
    copter.flightmode = (Mode *)&copter.mode_auto;

    // The RC has never been seen, so has_valid_input() is false and
    // get_heading takes the else of its pilot branch. That path is recorded
    // too: a mode of PILOT_RATE must come back as HOLD.
    printf("#heading_mode\n");
    printf("mode,mode_after,heading_mode,wp_yaw_behavior\n");
    // Two mode transitions are visible here besides the heading mode: the
    // pilot-release path (the RC has never been seen, so PILOT_RATE must come
    // back as HOLD) and the weathervane release, whose destination depends on
    // WP_YAW_BEHAVIOR. That parameter is recorded so the test can compute the
    // destination rather than assume it.
    for (int m = 0; m <= 10; m++) {
        ay._mode = (Mode::AutoYaw::Mode)m;
        ay._last_mode = (Mode::AutoYaw::Mode)0;
        const AC_AttitudeControl::HeadingCommand h = ay.get_heading();
        printf("%d,%d,%d,%d\n", m, (int)ay._mode, (int)h.heading_mode,
               (int)copter.g.wp_yaw_behavior.get());
    }

    // ---- the fixed-yaw slew ----
    //
    // dt comes from millis() inside yaw_rad, so it cannot be dictated. It is
    // recorded instead: _last_update_ms is seeded a known number of
    // milliseconds behind the clock the firmware is about to read, and the
    // resulting angle and remaining offset are compared against the port
    // driven with that same dt.
    printf("#fixed_slew\n");
    printf("idx,angle_in,offset_in,slew_rads,dt_ms,angle_out,offset_out\n");
    {
        const float offsets[] = {0.0f, 0.05f, -0.05f, 1.5f, -1.5f};
        const float slews[] = {0.0f, 0.1f, 2.0f};
        const uint32_t lags[] = {0, 20, 250};
        int idx = 0;
        for (unsigned o = 0; o < 5; o++)
          for (unsigned s = 0; s < 3; s++)
            for (unsigned l = 0; l < 3; l++) {
                ay._mode = (Mode::AutoYaw::Mode)3;   // FIXED
                ay._yaw_angle_rad = 0.25f;
                ay._fixed_yaw_offset_rad = offsets[o];
                ay._fixed_yaw_slewrate_rads = slews[s];

                const uint32_t now = AP_HAL::millis();
                ay._last_update_ms = now - lags[l];

                const float angle_in = ay._yaw_angle_rad;
                const float offset_in = ay._fixed_yaw_offset_rad;
                const float out = ay.yaw_rad();
                const uint32_t dt_ms = ay._last_update_ms - (now - lags[l]);

                printf("%d,%u,%u,%u,%u,%u,%u\n", idx++,
                       fbits(angle_in), fbits(offset_in), fbits(slews[s]),
                       (unsigned)dt_ms,
                       fbits(out), fbits(ay._fixed_yaw_offset_rad));
            }
    }

    // ---- the angle-rate integration ----
    printf("#angle_rate\n");
    printf("idx,angle_in,rate_in,dt_ms,angle_out\n");
    {
        const float rates[] = {0.0f, 0.35f, -0.35f, 3.0f};
        const uint32_t lags[] = {0, 20, 250};
        int idx = 0;
        for (unsigned r = 0; r < 4; r++)
          for (unsigned l = 0; l < 3; l++) {
              ay._mode = (Mode::AutoYaw::Mode)6;   // ANGLE_RATE
              ay._yaw_angle_rad = -0.5f;
              ay._yaw_rate_rads = rates[r];

              const uint32_t now = AP_HAL::millis();
              ay._last_update_ms = now - lags[l];

              const float angle_in = ay._yaw_angle_rad;
              const float out = ay.yaw_rad();
              const uint32_t dt_ms = ay._last_update_ms - (now - lags[l]);

              printf("%d,%u,%u,%u,%u\n", idx++,
                     fbits(angle_in), fbits(rates[r]),
                     (unsigned)dt_ms, fbits(out));
          }
    }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/ay2_parity/vehicle")
    build(HARNESS, objects, BUILD, "ArduCopter/Copter.cpp",
          link_flags=vehicle_link.LINK_FLAGS)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
