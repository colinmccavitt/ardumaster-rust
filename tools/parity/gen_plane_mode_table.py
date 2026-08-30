"""Record Plane's mode-number table and the mode-exit autotune decision.

# Exhaustive, because it can be

The table takes one byte. Every possible input is recorded -- all 256 -- so
there is no question of which numbers were sampled. That also settles the
feature gates empirically: a number whose case is compiled out returns null in
this build, and the fixture says which ones did rather than the port having to
be trusted about #ifs it cannot see.

It is worth the exhaustiveness for another reason. The numbers were guessed
wrong twice while writing this slice -- TAKEOFF is 13 and GUIDED is 15, not
what a reading of the switch's order suggests -- and a sampled recording would
have agreed with a wrong table on the samples it took.

# The exit decision

Mode::exit restores the autotuned gains unless the mode is autotune. The
subtlety is which mode it means: set_mode assigns control_mode before calling
old_mode.exit(), so the comparison is against the mode being *entered*.
Reading it the other way would restore the gains on every exit from autotune,
discarding a tune the moment it completed.

Observed by wrapping Plane::autotune_restore, which mode.cpp.o references as
undefined (checked with nm -u first).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import plane_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/ardumaster-rust")
OUT = ROOT / "fixtures/plane_mode_table.csv"
BUILD = Path("/tmp/plane_table_parity/harness")

AUTOTUNE_RESTORE = "_ZN5Plane16autotune_restoreEv"

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

static int g_restore_calls = 0;

extern "C" void __wrap__ZN5Plane16autotune_restoreEv(void *self);
extern "C" void __wrap__ZN5Plane16autotune_restoreEv(void *self)
{
    (void)self;
    g_restore_calls++;
    // Not forwarded: it writes gains into three attitude controllers and
    // sends a GCS message, and whether it was called is the observation.
}

int main()
{
    AP::scheduler().init(nullptr, 0, 0);
    setvbuf(stdout, NULL, _IOLBF, 0);

    // Which optional modes this build has, reported by the firmware itself
    // rather than assumed from the #ifs.
    printf("#features\n");
    printf("name,enabled\n");
#if HAL_ADSB_ENABLED
    printf("adsb,1\n");
#else
    printf("adsb,0\n");
#endif
#if HAL_QUADPLANE_ENABLED
    printf("quadplane,1\n");
#else
    printf("quadplane,0\n");
#endif
#if HAL_QUADPLANE_ENABLED && QAUTOTUNE_ENABLED
    printf("qautotune,1\n");
#else
    printf("qautotune,0\n");
#endif
#if HAL_SOARING_ENABLED
    printf("soaring,1\n");
#else
    printf("soaring,0\n");
#endif
#if MODE_AUTOLAND_ENABLED
    printf("autoland,1\n");
#else
    printf("autoland,0\n");
#endif

    // ---- the whole table ----
    printf("#table\n");
    printf("number,mode\n");
    for (unsigned n = 0; n < 256; n++) {
        Mode *m = plane.mode_from_mode_num((Mode::Number)n);
        printf("%u,%d\n", n, m == nullptr ? -1 : (int)m->mode_number());
    }

    // ---- Mode::exit's autotune restore ----
    //
    // set_mode assigns control_mode before calling old_mode.exit(), so what
    // exit() compares against is the mode being entered. The harness
    // reproduces that ordering explicitly.
    printf("#exit\n");
    printf("idx,entered_mode,left_mode,restored\n");
    {
        Mode *entered[] = {
            (Mode *)&plane.mode_autotune,
            (Mode *)&plane.mode_manual,
            (Mode *)&plane.mode_fbwa,
            (Mode *)&plane.mode_cruise,
        };
        Mode *left[] = {
            (Mode *)&plane.mode_autotune,
            (Mode *)&plane.mode_manual,
        };
        int idx = 0;
        for (unsigned e = 0; e < 4; e++)
          for (unsigned l = 0; l < 2; l++) {
              plane.control_mode = entered[e];
              g_restore_calls = 0;
              left[l]->exit();
              printf("%d,%d,%d,%d\n", idx++,
                     (int)entered[e]->mode_number(),
                     (int)left[l]->mode_number(),
                     g_restore_calls > 0 ? 1 : 0);
          }
    }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = plane_link.objects(stage_dir="/tmp/plane_table_parity/vehicle")
    flags = list(plane_link.LINK_FLAGS) + ["-Wl,--wrap=" + AUTOTUNE_RESTORE]
    build(HARNESS, objects, BUILD, "ArduPlane/Plane.cpp", link_flags=flags)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
