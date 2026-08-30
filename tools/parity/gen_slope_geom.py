"""Dump the glide-slope geometry from the real ArduPlane firmware.

This closes the gap the first FW-029 slice recorded honestly: the function
writes through the vehicle's callbacks rather than returning, so verifying it
needs a real vehicle -- which plane_link now provides.

# Observing rather than stubbing

The two callbacks are where the answer comes out. The harness rebinds them to
recorders, which is not stubbing the function under test: the slope
calculation itself is entirely the firmware's, and what is replaced is only
the vehicle's reaction to its result. The port returns those same values in a
SlopeResult, so the recorders capture exactly what the port must produce.

The alternative -- reading the vehicle's target_altitude afterwards -- would
verify Plane's altitude bookkeeping instead of the slope, and would miss the
aim point entirely.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import plane_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/ardumaster-rust")
OUT = ROOT / "fixtures/slope_geometry.csv"
BUILD = Path("/tmp/slopegeom_parity/harness")

HARNESS = r'''
#include <AP_HAL/AP_HAL.h>

// Visibility only, for this translation unit; see gen_slope_stage.py.
#define private public
#define protected public
#include "/srv/ardumaster/upstream/plane-4.7.0/ArduPlane/Plane.h"
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

// Records what the vehicle would have been told.
struct Recorder {
    Location aim;
    float altitude_proportion;
    Location constrain_a;
    Location constrain_b;
    bool got_proportion;
    bool got_constrain;

    void set_target_altitude_proportion(const Location &loc, float proportion) {
        aim = loc;
        altitude_proportion = proportion;
        got_proportion = true;
    }
    void constrain_target_altitude_location(const Location &loc1, const Location &loc2) {
        constrain_a = loc1;
        constrain_b = loc2;
        got_constrain = true;
    }
};

static Recorder recorder;

int main()
{
    AP_Landing *land = &plane.landing;

    land->set_target_altitude_proportion_fn =
        FUNCTOR_BIND(&recorder, &Recorder::set_target_altitude_proportion, void,
                     const Location &, float);
    land->constrain_target_altitude_location_fn =
        FUNCTOR_BIND(&recorder, &Recorder::constrain_target_altitude_location, void,
                     const Location &, const Location &);

    printf("#geometry\n");
    printf("idx,flare_sec,flare_alt,flare_effect,groundspeed,land_sinkrate,"
           "prev_lat,prev_lng,prev_alt,next_lat,next_lng,next_alt,"
           "cur_lat,cur_lng,cur_alt,"
           "slope,offset_cm,aim_lat,aim_lng,aim_alt,alt_proportion\n");
    {
        // A landing at a fixed point, approached from a few bearings and
        // distances, with the flare parameters swept. Altitudes are absolute
        // so no terrain or origin resolution is involved.
        const int32_t base_lat = -353632621;
        const int32_t base_lng = 1491652374;

        const float flare_secs[] = {0.0f, 2.0f, 6.0f};
        const float flare_alts[] = {0.0f, 3.0f, 10.0f};
        const int flare_effects[] = {0, 60, 100};
        const float speeds[] = {0.2f, 12.0f, 35.0f};
        const float sinkrates[] = {0.5f, 2.5f};
        const int32_t distances_m[] = {50, 400, 2000};

        int idx = 0;
        for (unsigned a = 0; a < 3; a++)
          for (unsigned b = 0; b < 3; b++)
            for (unsigned c = 0; c < 3; c++)
              for (unsigned s = 0; s < 3; s++)
                for (unsigned k = 0; k < 2; k++)
                  for (unsigned dd = 0; dd < 3; dd++) {
                      land->flare_sec.set(flare_secs[a]);
                      land->flare_alt.set(flare_alts[b]);
                      land->flare_effectivness_pct.set(flare_effects[c]);

                      // Ground speed and land sink rate come from the AHRS and
                      // TECS, which cannot be driven here, so they are read
                      // and recorded rather than set.
                      (void)speeds[s];
                      (void)sinkrates[k];

                      Location next(base_lat, base_lng, 4000, Location::AltFrame::ABSOLUTE);
                      Location prev = next;
                      prev.offset_bearing(45.0f, -(float)distances_m[dd]);
                      prev.alt = 4000 + distances_m[dd] * 5;

                      Location cur = next;
                      cur.offset_bearing(45.0f, -(float)distances_m[dd] * 0.4f);
                      cur.alt = 4000 + distances_m[dd] * 2;

                      land->type_slope_stage = AP_Landing::SlopeStage::APPROACH;
                      land->slope = 0.0f;   // force the first-calculation path
                      recorder.got_proportion = false;
                      recorder.got_constrain = false;

                      int32_t offset_cm = 0;
                      land->type_slope_setup_landing_glide_slope(prev, next, cur, offset_cm);

                      if (!recorder.got_proportion || !recorder.got_constrain) {
                          fprintf(stderr, "callbacks not invoked at idx %d\n", idx);
                          return 1;
                      }

                      printf("%d,%u,%u,%d,%u,%u,"
                             "%d,%d,%d,%d,%d,%d,%d,%d,%d,"
                             "%u,%d,%d,%d,%d,%u\n", idx++,
                             fbits(land->flare_sec), fbits(land->flare_alt),
                             (int)land->flare_effectivness_pct,
                             fbits(plane.ahrs.groundspeed()),
                             fbits(plane.TECS_controller.get_land_sinkrate()),
                             prev.lat, prev.lng, prev.alt,
                             next.lat, next.lng, next.alt,
                             cur.lat, cur.lng, cur.alt,
                             fbits(land->slope), (int)offset_cm,
                             recorder.aim.lat, recorder.aim.lng, recorder.aim.alt,
                             fbits(recorder.altitude_proportion));
                  }
    }

    return 0;
}
'''


def main():
    objects = plane_link.objects(stage_dir="/tmp/slopegeom_parity/vehicle")
    build(HARNESS, objects, BUILD, "ArduPlane/Plane.cpp",
          link_flags=plane_link.LINK_FLAGS)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
