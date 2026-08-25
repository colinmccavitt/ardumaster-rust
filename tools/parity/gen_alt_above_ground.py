"""Record Mode::get_alt_above_ground_m's fallback ladder from the firmware.

# Injecting inputs rather than stubbing the code under test

The ladder tries three sources in order and falls back to a flat-earth
assumption. Two of those sources cannot be brought up in a harness: the
interpolated rangefinder height needs the AHRS to have an origin and a running
estimator, and the above-terrain altitude needs a terrain database with data
loaded for the location.

So they are injected. `-Wl,--wrap` on each source replaces what the ladder
calls with a harness-controlled answer, sweeping both "this source has an
answer" and "it does not". The ladder itself -- the order the sources are
tried, the initialised check between them, and the flat-earth fallback -- is
the real firmware code, unmodified, and it is what is being recorded.

That is the distinction that matters: wrapping the *inputs* of the function
under test is supplying it with data, which is what a harness is for.
Wrapping the function under test, or transcribing its body, would be
comparing the port against a copy of itself.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/plane-fw-rust")
OUT = ROOT / "fixtures/alt_above_ground.csv"
BUILD = Path("/tmp/agl_parity/harness")

RANGEFINDER = "_ZNK6Copter37get_rangefinder_height_interpolated_mERf"
GET_ALT = "_ZNK8Location9get_alt_mENS_8AltFrameERf"

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

// ---- injected inputs ----
//
// The values the two unavailable sources should report this row. Set by the
// sweep, read by the wrappers.
static bool g_rf_ok = false;
static float g_rf_value = 0.0f;
static bool g_terrain_ok = false;
static float g_terrain_value = 0.0f;
static int g_terrain_calls = 0;

extern "C" bool __wrap__ZNK6Copter37get_rangefinder_height_interpolated_mERf(
    const void *self, float &height_m);
extern "C" bool __wrap__ZNK6Copter37get_rangefinder_height_interpolated_mERf(
    const void *self, float &height_m)
{
    (void)self;
    if (!g_rf_ok) {
        return false;
    }
    height_m = g_rf_value;
    return true;
}

extern "C" bool __real__ZNK8Location9get_alt_mENS_8AltFrameERf(
    const void *self, Location::AltFrame frame, float &ret_alt_m);

extern "C" bool __wrap__ZNK8Location9get_alt_mENS_8AltFrameERf(
    const void *self, Location::AltFrame frame, float &ret_alt_m);
extern "C" bool __wrap__ZNK8Location9get_alt_mENS_8AltFrameERf(
    const void *self, Location::AltFrame frame, float &ret_alt_m)
{
    // Only the above-terrain query is injected. Every other frame -- and
    // set_alt_cm's own internal use -- goes to the real implementation, so
    // the Location under test behaves normally in every other respect.
    if (frame != Location::AltFrame::ABOVE_TERRAIN) {
        return __real__ZNK8Location9get_alt_mENS_8AltFrameERf(self, frame, ret_alt_m);
    }
    g_terrain_calls++;
    if (!g_terrain_ok) {
        return false;
    }
    ret_alt_m = g_terrain_value;
    return true;
}

int main()
{
    AP::scheduler().init(nullptr, 0, 0);
    copter.allocate_motors();
    copter.motors->_throttle_hover_learn.set(0);

    Mode *mode = &copter.mode_land;

    printf("#agl\n");
    printf("idx,rf_ok,rf_value,initialised,terrain_ok,terrain_value,"
           "loc_alt_cm,out\n");

    const float rf_values[] = {-2.5f, 0.0f, 7.25f};
    const float terrain_values[] = {-1.5f, 0.0f, 12.5f};
    // Altitudes in centimetres, including a fractional metre so the flat-earth
    // fallback's 0.01 scaling is visible, and a negative one.
    const int32_t alt_cms[] = {-450, 0, 37, 1200, 250000};

    int idx = 0;
    for (unsigned r = 0; r < 2; r++)
      for (unsigned rv = 0; rv < 3; rv++)
        for (unsigned i = 0; i < 2; i++)
          for (unsigned t = 0; t < 2; t++)
            for (unsigned tv = 0; tv < 3; tv++)
              for (unsigned a = 0; a < 5; a++) {
                  g_rf_ok = (r != 0);
                  g_rf_value = rf_values[rv];
                  g_terrain_ok = (t != 0);
                  g_terrain_value = terrain_values[tv];

                  if (i != 0) {
                      copter.current_loc.lat = -353632621;
                      copter.current_loc.lng = 1491652374;
                  } else {
                      // An uninitialised Location is one with no position.
                      copter.current_loc.lat = 0;
                      copter.current_loc.lng = 0;
                  }
                  copter.current_loc.set_alt_cm(alt_cms[a],
                                                Location::AltFrame::ABOVE_HOME);

                  const bool initialised = copter.current_loc.initialised();
                  g_terrain_calls = 0;
                  const float out = mode->get_alt_above_ground_m();

                  printf("%d,%d,%u,%d,%d,%u,%d,%u\n", idx++,
                         (int)g_rf_ok, fbits(g_rf_value),
                         (int)initialised,
                         (int)g_terrain_ok, fbits(g_terrain_value),
                         (int)copter.current_loc.alt,
                         fbits(out));
              }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/agl_parity/vehicle")
    flags = list(vehicle_link.LINK_FLAGS) + [
        "-Wl,--wrap=" + RANGEFINDER,
        "-Wl,--wrap=" + GET_ALT,
    ]
    build(HARNESS, objects, BUILD, "ArduCopter/Copter.cpp", link_flags=flags)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
