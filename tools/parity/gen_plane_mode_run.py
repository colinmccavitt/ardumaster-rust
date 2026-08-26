"""Record Mode::run's stick mixing, reset_controllers, and pre_arm_checks.

Each decision is observed where it leaves the firmware:

  * stick mixing, by wrapping Plane::stabilize_stick_mixing_fbw and counting
    calls across every STICK_MIXING value including out-of-range ones, since
    upstream's switch has no default case.
  * reset_controllers, by sentinel-filling the steering state and reading it
    back, the same technique gen_plane_mode_entry.py uses.
  * pre_arm_checks, by reading the buffer the firmware wrote -- which is the
    only way to see the generic-message substitution, because the return
    value is false either way.

All three wrapped symbols were checked against nm -u on mode.cpp.o first.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import plane_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/plane-fw-rust")
OUT = ROOT / "fixtures/plane_mode_run.csv"
BUILD = Path("/tmp/plane_run_parity/harness")

MIX = "_ZN5Plane26stabilize_stick_mixing_fbwEv"
ROLL = "_ZN5Plane14stabilize_rollEv"
PITCH = "_ZN5Plane15stabilize_pitchEv"
YAW = "_ZN5Plane13stabilize_yawEv"

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

static uint32_t fbits(float f)
{
    uint32_t u;
    memcpy(&u, &f, sizeof(u));
    return u;
}

static int g_mix_calls = 0;
static int g_roll_calls = 0;
static int g_pitch_calls = 0;
static int g_yaw_calls = 0;

extern "C" void __wrap__ZN5Plane26stabilize_stick_mixing_fbwEv(void *self);
extern "C" void __wrap__ZN5Plane26stabilize_stick_mixing_fbwEv(void *self)
{
    (void)self;
    g_mix_calls++;
    // Not forwarded: it drives the attitude controllers, and whether it ran
    // is the observation.
}

// All three stabilise calls are wrapped, not just roll. They run against
// controllers a harness has not brought up, and leaving two of them live
// segfaulted on the first row. What is being recorded is the dispatch --
// which calls run() makes for a given STICK_MIXING -- and their bodies have
// their own slices. Counting them also makes "no mixing" distinguishable
// from "run() never got there".
extern "C" void __wrap__ZN5Plane14stabilize_rollEv(void *self);
extern "C" void __wrap__ZN5Plane14stabilize_rollEv(void *self)
{
    (void)self;
    g_roll_calls++;
}

extern "C" void __wrap__ZN5Plane15stabilize_pitchEv(void *self);
extern "C" void __wrap__ZN5Plane15stabilize_pitchEv(void *self)
{
    (void)self;
    g_pitch_calls++;
}

extern "C" void __wrap__ZN5Plane13stabilize_yawEv(void *self);
extern "C" void __wrap__ZN5Plane13stabilize_yawEv(void *self)
{
    (void)self;
    g_yaw_calls++;
}

int main()
{
    AP::scheduler().init(nullptr, 0, 0);
    setvbuf(stdout, NULL, _IOLBF, 0);

    Mode *mode = (Mode *)&plane.mode_fbwa;

    // ---- stick mixing ----
    //
    // Every value the parameter can hold, not only the five the enum names:
    // upstream's switch has no default case, so what an out-of-range value
    // does is a fact about the code rather than an impossibility.
    printf("#stick_mixing\n");
    printf("stick_mixing,mixed,stabilized_roll,stabilized_pitch,stabilized_yaw\n");
    for (int v = -1; v < 8; v++) {
        plane.g.stick_mixing.set((StickMixing)v);
        g_mix_calls = 0;
        g_roll_calls = 0;
        g_pitch_calls = 0;
        g_yaw_calls = 0;
        mode->run();
        printf("%d,%d,%d,%d,%d\n", v,
               g_mix_calls > 0 ? 1 : 0,
               g_roll_calls > 0 ? 1 : 0,
               g_pitch_calls > 0 ? 1 : 0,
               g_yaw_calls > 0 ? 1 : 0);
    }

    // ---- reset_controllers ----
    printf("#reset\n");
    printf("idx,locked_course,locked_course_err\n");
    {
        int idx = 0;
        for (unsigned a = 0; a < 2; a++) {
            plane.steer_state.locked_course = (a != 0);
            plane.steer_state.locked_course_err = 2.75f;
            mode->reset_controllers();
            printf("%d,%d,%u\n", idx++,
                   (int)plane.steer_state.locked_course,
                   fbits(plane.steer_state.locked_course_err));
        }
    }

    // ---- pre_arm_checks ----
    //
    // The generic substitution is invisible in the return value, which is
    // false either way; the buffer is where it shows.
    printf("#pre_arm\n");
    printf("idx,mode,allowed,message\n");
    {
        Mode *candidates[] = {
            (Mode *)&plane.mode_manual,
            (Mode *)&plane.mode_fbwa,
            (Mode *)&plane.mode_auto,
            (Mode *)&plane.mode_takeoff,
            (Mode *)&plane.mode_qhover,
        };
        int idx = 0;
        for (unsigned c = 0; c < 5; c++) {
            char buf[128];
            buf[0] = '\0';
            const bool ok = candidates[c]->pre_arm_checks(sizeof(buf), buf);
            printf("%d,%d,%d,%s\n", idx++,
                   (int)candidates[c]->mode_number(),
                   ok ? 1 : 0,
                   buf[0] == '\0' ? "-" : buf);
        }
    }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = plane_link.objects(stage_dir="/tmp/plane_run_parity/vehicle")
    flags = list(plane_link.LINK_FLAGS) + [
        "-Wl,--wrap=" + MIX,
        "-Wl,--wrap=" + ROLL,
        "-Wl,--wrap=" + PITCH,
        "-Wl,--wrap=" + YAW,
    ]
    build(HARNESS, objects, BUILD, "ArduPlane/Plane.cpp", link_flags=flags)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
