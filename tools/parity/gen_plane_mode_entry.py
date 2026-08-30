"""Record what Mode::enter clears, by filling every field with a sentinel.

The function is a reset list, so the failure mode being tested for is omission
rather than arithmetic. A sweep of plausible values would not find a missing
field -- the field would just keep whatever the sweep happened to put there
and no assertion would notice.

So every field is written with a distinct non-default value before the real
enter() runs, and every field is read back afterwards. A field upstream clears
comes back cleared; a field it seeds comes back seeded; anything the port
forgets shows up on the Rust side as a sentinel that survived.

Five modes are used, with different answers for does_auto_throttle, and the
pitch is set to several values because initial_pitch_cd is seeded from the
attitude rather than zeroed -- a field a port might flatten.

No VTOL mode is among them. Calling enter() on one with no quadplane
available segfaults: Plane::set_mode's VTOL guard is what normally stands
between a Q mode request and that null dereference, and a harness calling
enter() directly walks straight past it. So auto_state.vtol_mode is recorded
only in its false case, and the test says so rather than implying otherwise.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import plane_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/ardumaster-rust")
OUT = ROOT / "fixtures/plane_mode_entry.csv"
BUILD = Path("/tmp/plane_entry_parity/harness")

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

int main()
{
    AP::scheduler().init(nullptr, 0, 0);

    // Line buffered, so a crash mid-sweep does not swallow the rows
    // that already succeeded -- which is the only way to see where it
    // happened. The first attempt segfaulted with an empty file.
    setvbuf(stdout, NULL, _IOLBF, 0);

    printf("#entry\n");
    printf("idx,mode,pitch_cd,is_vtol,ok,"
           "inverted_flight,waiting_rudder,next_wp_crosstrack,"
           "checked_for_autoland,locked_course,locked_course_err,"
           "is_crashed,impact_detected,highest_airspeed,initial_pitch_cd,"
           "fbwa_tdrag,rotation_complete,loiter_start_ms,vtol_mode,"
           "vtol_loiter,new_airspeed_cm,long_fs_pending,idle_mode,"
           "throttle_suppressed,does_auto_throttle\n");

    // No VTOL mode here. Calling enter() on one with no quadplane available
    // segfaults -- Plane::set_mode's VTOL guard is what normally stands
    // between a Q mode request and that null dereference, and a harness
    // calling enter() directly walks straight past it.
    Mode *modes[] = {
        (Mode *)&plane.mode_manual,
        (Mode *)&plane.mode_fbwa,
        (Mode *)&plane.mode_cruise,
        (Mode *)&plane.mode_training,
        (Mode *)&plane.mode_acro,
    };
    const int16_t pitches[] = {-4500, -137, 0, 250, 8999};

    int idx = 0;
    for (unsigned m = 0; m < 5; m++)
      for (unsigned p = 0; p < 5; p++) {
          // Every field written with a distinct value that is not its reset
          // value, so anything enter() leaves alone is visible.
          plane.auto_state.inverted_flight = true;
          plane.takeoff_state.waiting_for_rudder_neutral = true;
          plane.auto_state.next_wp_crosstrack = true;
          plane.auto_state.checked_for_autoland = true;
          plane.steer_state.locked_course = true;
          plane.steer_state.locked_course_err = 1.5f;
          plane.crash_state.is_crashed = true;
          plane.crash_state.impact_detected = true;
          plane.auto_state.highest_airspeed = 33.25f;
          plane.auto_state.initial_pitch_cd = -9999;
          plane.auto_state.fbwa_tdrag_takeoff_mode = true;
          plane.auto_state.rotation_complete = true;
          plane.loiter.start_time_ms = 123456;
          plane.auto_state.vtol_mode = true;
          plane.auto_state.vtol_loiter = true;
          plane.new_airspeed_cm = 777;
          plane.long_failsafe_pending = true;
          plane.auto_state.idle_mode = true;
          plane.throttle_suppressed = false;

          // The attitude initial_pitch_cd is seeded from.
          plane.ahrs.pitch_sensor = pitches[p];

          // A known starting mode, so enter() is entered from somewhere.
          plane.control_mode = (Mode *)&plane.mode_manual;

          const bool ok = modes[m]->enter();

          printf("%d,%d,%d,%d,%d,"
                 "%d,%d,%d,"
                 "%d,%d,%u,"
                 "%d,%d,%u,%d,"
                 "%d,%d,%u,%d,"
                 "%d,%d,%d,%d,"
                 "%d,%d\n",
                 idx++, (int)modes[m]->mode_number(), (int)pitches[p],
                 (int)modes[m]->is_vtol_mode(), ok ? 1 : 0,
                 (int)plane.auto_state.inverted_flight,
                 (int)plane.takeoff_state.waiting_for_rudder_neutral,
                 (int)plane.auto_state.next_wp_crosstrack,
                 (int)plane.auto_state.checked_for_autoland,
                 (int)plane.steer_state.locked_course,
                 fbits(plane.steer_state.locked_course_err),
                 (int)plane.crash_state.is_crashed,
                 (int)plane.crash_state.impact_detected,
                 fbits(plane.auto_state.highest_airspeed),
                 (int)plane.auto_state.initial_pitch_cd,
                 (int)plane.auto_state.fbwa_tdrag_takeoff_mode,
                 (int)plane.auto_state.rotation_complete,
                 (unsigned)plane.loiter.start_time_ms,
                 (int)plane.auto_state.vtol_mode,
                 (int)plane.auto_state.vtol_loiter,
                 (int)plane.new_airspeed_cm,
                 (int)plane.long_failsafe_pending,
                 (int)plane.auto_state.idle_mode,
                 (int)plane.throttle_suppressed,
                 (int)modes[m]->does_auto_throttle());
      }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = plane_link.objects(stage_dir="/tmp/plane_entry_parity/vehicle")
    build(HARNESS, objects, BUILD, "ArduPlane/Plane.cpp",
          link_flags=plane_link.LINK_FLAGS)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
