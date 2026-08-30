"""Record Mode::use_throttle_limits and Mode::use_battery_compensation.

Both are const predicates over vehicle state, so they are called directly on
each mode with the state driven around them. The manual-throttle set is
identified by *which* mode the predicate is called on -- upstream compares
`this` against five specific modes -- so every mode in the table is swept
rather than a flag being injected.

THR_PASS_STAB and guided_throttle_passthru are plain fields and are set
directly. The VTOL and scripting branches are not reachable here: no
quadplane is configured, and nav_scripting_active needs a running script. The
recording therefore covers the manual-throttle divergence, which is the one
that differs between the two functions on a fixed wing, and the test says
what is left to reasoning.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import plane_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/ardumaster-rust")
OUT = ROOT / "fixtures/plane_throttle_rules.csv"
BUILD = Path("/tmp/plane_thr_parity/harness")

HARNESS = r'''
#include <AP_HAL/AP_HAL.h>

#define private public
#define protected public
#include "/srv/ardumaster/upstream/plane-4.7.0/ArduPlane/Plane.h"
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

int main()
{
    AP::scheduler().init(nullptr, 0, 0);
    setvbuf(stdout, NULL, _IOLBF, 0);

    // Every fixed-wing mode, so the five-mode manual-throttle set is
    // identified by the firmware rather than asserted by the harness.
    Mode *modes[] = {
        (Mode *)&plane.mode_manual,
        (Mode *)&plane.mode_circle,
        (Mode *)&plane.mode_stabilize,
        (Mode *)&plane.mode_training,
        (Mode *)&plane.mode_acro,
        (Mode *)&plane.mode_fbwa,
        (Mode *)&plane.mode_fbwb,
        (Mode *)&plane.mode_cruise,
        (Mode *)&plane.mode_autotune,
        (Mode *)&plane.mode_auto,
        (Mode *)&plane.mode_rtl,
        (Mode *)&plane.mode_loiter,
        (Mode *)&plane.mode_takeoff,
        (Mode *)&plane.mode_guided,
    };

    printf("#throttle_rules\n");
    printf("idx,mode,is_guided,thr_pass_stab,guided_passthru,"
           "use_limits,use_battery_comp\n");

    int idx = 0;
    for (unsigned m = 0; m < 14; m++)
      for (unsigned ps = 0; ps < 2; ps++)
        for (unsigned gp = 0; gp < 2; gp++) {
            plane.g.throttle_passthru_stabilize.set(ps != 0);
            plane.guided_throttle_passthru = (gp != 0);

            printf("%d,%d,%d,%d,%d,%d,%d\n", idx++,
                   (int)modes[m]->mode_number(),
                   (int)modes[m]->is_guided_mode(),
                   ps, gp,
                   (int)modes[m]->use_throttle_limits(),
                   (int)modes[m]->use_battery_compensation());
        }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = plane_link.objects(stage_dir="/tmp/plane_thr_parity/vehicle")
    build(HARNESS, objects, BUILD, "ArduPlane/Plane.cpp",
          link_flags=plane_link.LINK_FLAGS)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
