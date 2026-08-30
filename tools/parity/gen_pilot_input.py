"""Dump the pilot stick conversions from the real ArduCopter firmware.

# Bringing the vehicle up far enough

The obstacle was `copter.motors`: a pointer assigned during `setup()`, which a
parity harness does not run, so reading the hover throttle through it crashed.

`Copter::allocate_motors()` is the single function in `setup()` that assigns
it — along with the attitude and position controllers, which the whole Mode
layer needs. Calling it directly, after the scheduler is up so it can read the
loop rate, gives a vehicle whose controllers are the firmware's own.

That is the Copter counterpart of what `plane_link` did for `AP_Landing`, and
it unblocks parity testing for `Mode` generally rather than only for these two
functions.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/ardumaster-rust")
OUT = ROOT / "fixtures/pilot_input.csv"
BUILD = Path("/tmp/pilot_parity/harness")

HARNESS = r'''
#include <AP_HAL/AP_HAL.h>

// Visibility only, for this translation unit; the firmware objects linked
// against are untouched. See tools/parity/gen_slope_stage.py.
#define private public
#define protected public
#include "/srv/ardumaster/upstream/plane-4.7.0/ArduCopter/Copter.h"
#undef private
#undef protected

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

int main()
{
    // The scheduler first: allocate_motors reads its loop rate.
    AP::scheduler().init(nullptr, 0, 0);
    copter.allocate_motors();

    if (copter.motors == nullptr) {
        fprintf(stderr, "motors still null after allocate_motors\n");
        return 1;
    }

    // setup() never runs, so the channel pointers are null. They are assigned
    // from the RC singleton's own array -- the same objects init_rc_in points
    // them at.
    copter.channel_roll = rc().channel(0);
    copter.channel_pitch = rc().channel(1);
    copter.channel_throttle = rc().channel(2);
    copter.channel_yaw = rc().channel(3);
    if (copter.channel_throttle == nullptr || copter.channel_yaw == nullptr) {
        fprintf(stderr, "RC channels unavailable\n");
        return 1;
    }
    copter.channel_throttle->set_range(1000);
    copter.channel_yaw->set_angle(4500);

    // A conventional radio range on both, so radio_in has something to be
    // measured against.
    for (RC_Channel *ch : {copter.channel_throttle, copter.channel_yaw}) {
        ch->radio_min.set(1000);
        ch->radio_trim.set(1500);
        ch->radio_max.set(2000);
        ch->dead_zone.set(20);
    }

    // Both conversions return neutral unless the radio has been seen at
    // least once -- a failsafe with a stale stick position is worse than one
    // with none. Without this every recorded row is the failsafe path rather
    // than the conversion, which is what the first recording showed.
    rc()._has_ever_seen_rc_input = true;

    Mode *mode = &copter.mode_stabilize;

    // ---- pilot throttle ----
    printf("#throttle\n");
    printf("idx,control,mid_stick,thr_hover,out\n");
    {
        const int16_t controls[] = {-200, 0, 1, 250, 499, 500, 501, 750, 1000, 1200};
        // get_control_mid is derived from the radio range and the dead zone,
        // so moving the maximum moves the mid. Setting a trim would not: the
        // mid reads min, max and dead zone only.
        const int16_t radio_maxes[] = {1600, 2000, 2400};
        const float hovers[] = {0.0f, 0.125f, 0.35f, 0.5f, 0.6875f, 1.0f};
        int idx = 0;
        for (unsigned a = 0; a < 10; a++)
          for (unsigned b = 0; b < 3; b++)
            for (unsigned c = 0; c < 6; c++) {
                copter.channel_throttle->radio_max.set(radio_maxes[b]);
                copter.channel_throttle->set_control_in(controls[a]);
                copter.motors->_throttle_hover.set(hovers[c]);

                const float out = mode->get_pilot_desired_throttle();

                printf("%d,%d,%d,%u,%u\n", idx++,
                       (int)controls[a],
                       (int)copter.get_throttle_mid(),
                       fbits(copter.motors->get_throttle_hover()),
                       fbits(out));
            }
    }

    // ---- a collapsed throttle calibration ----
    //
    // With radio_min == radio_max the channel has no travel, and
    // get_control_mid returns the full 1000 rather than something near the
    // middle. That makes the upper branch's divisor (1000 - mid_stick) zero.
    // Recorded rather than argued about: see DIVERGENCES.md D-026.
    printf("#throttle_degenerate\n");
    printf("idx,control,mid_stick,thr_hover,out\n");
    {
        copter.channel_throttle->radio_min.set(2000);
        copter.channel_throttle->radio_max.set(2000);
        const int16_t controls[] = {0, 500, 998, 999, 1000, 1200};
        const float hovers[] = {0.25f, 0.5f, 0.75f};
        int idx = 0;
        for (unsigned a = 0; a < 6; a++)
          for (unsigned c = 0; c < 3; c++) {
              copter.channel_throttle->set_control_in(controls[a]);
              copter.motors->_throttle_hover.set(hovers[c]);

              const float out = mode->get_pilot_desired_throttle();

              printf("%d,%d,%d,%u,%u\n", idx++,
                     (int)controls[a],
                     (int)copter.get_throttle_mid(),
                     fbits(copter.motors->get_throttle_hover()),
                     fbits(out));
          }
    }

    // ---- pilot yaw rate ----
    printf("#yaw\n");
    printf("idx,stick_norm,rate_degs,expo,out\n");
    {
        // radio_in, which is what norm_input_dz reads. 1500 is centre and
        // 1480 to 1520 is inside the dead zone.
        const int16_t sticks[] = {1000, 1200, 1490, 1500, 1510, 1800, 2000};
        const float rates[] = {0.0f, 45.0f, 202.5f};
        const float expos[] = {-0.5f, 0.0f, 0.35f, 0.94f, 0.95f, 1.0f};
        int idx = 0;
        for (unsigned a = 0; a < 7; a++)
          for (unsigned b = 0; b < 3; b++)
            for (unsigned c = 0; c < 6; c++) {
                copter.channel_yaw->set_radio_in(sticks[a]);
                copter.g2.command_model_pilot_y.rate.set(rates[b]);
                copter.g2.command_model_pilot_y.expo.set(expos[c]);

                const float out = mode->get_pilot_desired_yaw_rate_rads();

                printf("%d,%u,%u,%u,%u\n", idx++,
                       fbits(copter.channel_yaw->norm_input_dz()),
                       fbits(copter.g2.command_model_pilot_y.get_rate()), fbits(copter.g2.command_model_pilot_y.get_expo()), fbits(out));
            }
    }

    return 0;
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/pilot_parity/vehicle")
    build(HARNESS, objects, BUILD, "ArduCopter/Copter.cpp",
          link_flags=vehicle_link.LINK_FLAGS)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
