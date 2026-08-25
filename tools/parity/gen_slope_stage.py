"""Dump the slope landing's stage predicates from the real firmware.

Linked against ArduPlane, not ArduCopter. AP_Landing exists in only one of the
two binaries -- nothing in Copter ever constructs one -- so a harness for it
built against Copter finds no singleton to call. This is the first harness to
use plane_link.

# Reaching the stage

Everything below AP_Landing.h:124 is private, including the stage member and
the type_slope_* predicates, and a subclass cannot reopen private access. The
harness relaxes access checking for its own translation unit instead.

That is a compiler-visibility change, not a behavioural one: the firmware
objects this links against are untouched, every function called is the
firmware's own, and access specifiers do not affect layout for a class whose
members are declared in sequence. The alternative was to drive the stage
through the real state machine, which needs a fully constructed vehicle and
would test the transitions rather than the predicates.

Both the private predicate and its public wrapper are recorded for each stage,
so the type dispatch is covered as well as the predicate.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import plane_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/plane-fw-rust")
OUT = ROOT / "fixtures/slope_stage.csv"
BUILD = Path("/tmp/slope_parity/harness")

HARNESS = r'''
#include <AP_HAL/AP_HAL.h>

// See the module docstring: visibility only, for this translation unit.
#define private public
#define protected public
#include <AP_Landing/AP_Landing.h>
#undef private
#undef protected

#define private public
#define protected public
// Absolute: waf compiles Plane.cpp from its own directory, so the quoted
// include resolves there. This harness is generated elsewhere.
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

int main()
{
    // AP_Landing has no singleton accessor -- the vehicle owns it, and the
    // vehicle object is one of the firmware objects this links against, so
    // this is the instance the firmware would fly with.
    AP_Landing *land = &plane.landing;

    // The public wrappers dispatch on the configured type; make sure it is
    // the slope type so both spellings are answering about the same thing.
    land->type.set((int8_t)AP_Landing::TYPE_STANDARD_GLIDE_SLOPE);

    // Every public wrapper except is_complete gates on this first and returns
    // false when no landing is running, so without it the recording shows the
    // gate rather than the predicate. is_complete does NOT check it -- it
    // dispatches straight to the stage -- which is the asymmetry the two
    // recorded columns exist to pin.
    land->flags.in_progress = true;

    printf("#predicates\n");
    printf("stage,flaring,on_final,on_approach,expecting_impact,complete,"
           "pub_flaring,pub_on_final,pub_on_approach,pub_expecting_impact,pub_complete\n");
    for (int s = 0; s <= 3; s++) {
        land->type_slope_stage = (decltype(land->type_slope_stage))s;
        printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", s,
               land->type_slope_is_flaring() ? 1 : 0,
               land->type_slope_is_on_final() ? 1 : 0,
               land->type_slope_is_on_approach() ? 1 : 0,
               land->type_slope_is_expecting_impact() ? 1 : 0,
               land->type_slope_is_complete() ? 1 : 0,
               land->is_flaring() ? 1 : 0,
               land->is_on_final() ? 1 : 0,
               land->is_on_approach() ? 1 : 0,
               land->is_expecting_impact() ? 1 : 0,
               land->is_complete() ? 1 : 0);
    }

    printf("#roll\n");
    printf("stage,desired,limit,out\n");
    {
        const int32_t desireds[] = {-9000, -2500, 0, 1500, 9000};
        const int32_t limits[] = {0, 500, 3000};
        for (int s = 0; s <= 3; s++) {
            land->type_slope_stage = (decltype(land->type_slope_stage))s;
            for (unsigned a = 0; a < 5; a++)
                for (unsigned b = 0; b < 3; b++) {
                    printf("%d,%d,%d,%d\n", s, desireds[a], limits[b],
                           (int)land->constrain_roll(desireds[a], limits[b]));
                }
        }
    }

    return 0;
}
'''


def main():
    objects = plane_link.objects(stage_dir="/tmp/slope_parity/vehicle")
    build(HARNESS, objects, BUILD, "AP_Landing/AP_Landing_Slope.cpp",
          link_flags=plane_link.LINK_FLAGS)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
