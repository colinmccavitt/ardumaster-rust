"""Record Plane::set_mode and Plane::in_fence_recovery from the firmware.

Links ArduPlane rather than ArduCopter -- see tools/parity/plane_link.py.

# What is observed

The refusal messages, via GCS::send_text, which ArduPlane/system.cpp.o
references as undefined (checked with nm -u before relying on it; see the note
in tools/parity/gen_mode_entry.py for what happens when it is not). Plane's
three pre-entry refusals each send a distinct one.

And the four pieces of mode state, read directly after the call, so the
rollback path is recorded rather than reasoned about: a mode whose enter()
fails must leave control_mode, previous_mode and both reasons exactly as they
were.

# Forcing an enter() failure

Most modes enter successfully under any conditions a harness can arrange, so
the rollback path would never be recorded. TAKEOFF is used because its enter()
refuses without a valid home and mission state, giving rows on both sides of
the branch. Whether it refused is not assumed: the row records the mode the
vehicle ended up in, which is what makes the rollback visible.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import plane_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/plane-fw-rust")
OUT = ROOT / "fixtures/plane_mode.csv"
BUILD = Path("/tmp/plane_mode_parity/harness")

SEND_TEXT = "_ZN3GCS9send_textE12MAV_SEVERITYPKcz"

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

static char g_text[192];
static int g_text_calls = 0;

extern "C" void __wrap__ZN3GCS9send_textE12MAV_SEVERITYPKcz(
    void *self, int severity, const char *fmt, ...);
extern "C" void __wrap__ZN3GCS9send_textE12MAV_SEVERITYPKcz(
    void *self, int severity, const char *fmt, ...)
{
    (void)self; (void)severity;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_text, sizeof(g_text), fmt, ap);
    va_end(ap);
    g_text_calls++;
}

// The recorded reason is the tail of upstream's message, so a reordering of
// the ladder changes the fixture.
static const char *classify(void)
{
    if (g_text_calls == 0) {
        return "-";
    }
    if (strstr(g_text, "in fence recovery") != NULL) {
        return "in fence recovery";
    }
    if (strstr(g_text, "GCS entry disabled") != NULL) {
        return "GCS entry disabled";
    }
    if (strstr(g_text, "Q_ENABLE 0") != NULL ||
        strstr(g_text, "HAL_QUADPLANE_ENABLED=0") != NULL) {
        return "vtol unavailable";
    }
    if (strstr(g_text, "Flight mode change failed") != NULL) {
        return "enter failed";
    }
    return "other";
}

static int reason_num(ModeReason r) { return (int)r; }

int main()
{
    AP::scheduler().init(nullptr, 0, 0);

    // ---- in_fence_recovery ----
    //
    // Driven through the two reasons it reads. control_mode is left as
    // something other than AUTO so the early return is not taken; the AUTO
    // branch needs a mission and is covered by reasoning in the test.
    printf("#fence_recovery\n");
    printf("idx,control_reason,previous_reason,out\n");
    {
        const ModeReason reasons[] = {
            ModeReason::UNKNOWN,
            ModeReason::FENCE_BREACHED,
            ModeReason::GCS_COMMAND,
            ModeReason::RTL_COMPLETE_SWITCHING_TO_FIXEDWING_AUTOLAND,
            ModeReason::RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL,
            ModeReason::QRTL_INSTEAD_OF_RTL,
            ModeReason::QLAND_INSTEAD_OF_RTL,
            ModeReason::INITIALISED,
        };
        plane.control_mode = &plane.mode_manual;
        int idx = 0;
        for (unsigned a = 0; a < 8; a++)
          for (unsigned b = 0; b < 8; b++) {
              plane.control_mode_reason = reasons[a];
              plane.previous_mode_reason = reasons[b];
              printf("%d,%d,%d,%d\n", idx++,
                     reason_num(reasons[a]), reason_num(reasons[b]),
                     (int)plane.in_fence_recovery());
          }
    }

    // ---- set_mode ----
    printf("#set_mode\n");
    printf("idx,target,reason,gcs_enabled,is_vtol,soft_armed,fence_enabled,"
           "fence_option,fence_breached,in_fence_recovery,"
           "before_control,before_previous,"
           "before_control_reason,before_previous_reason,ok,"
           "after_control,after_previous,after_control_reason,"
           "after_previous_reason,message\n");
    {
        Mode *targets[] = {
            (Mode *)&plane.mode_manual,
            (Mode *)&plane.mode_fbwa,
            (Mode *)&plane.mode_cruise,
            (Mode *)&plane.mode_takeoff,
            // A VTOL mode, so the quadplane check is reachable. No quadplane
            // is configured here, so it should refuse.
            (Mode *)&plane.mode_qhover,
            // AUTO is deliberately absent. Its enter() succeeds and then the
            // mode immediately re-enters RTL because there is no mission, so
            // the recorded after-state would be a different mode change than
            // the one under test. GUIDED does not do this.
            (Mode *)&plane.mode_guided,
        };
        const ModeReason reasons[] = {
            ModeReason::RC_COMMAND,
            ModeReason::GCS_COMMAND,
            ModeReason::INITIALISED,
            ModeReason::FENCE_BREACHED,
        };
        Mode *starts[] = {
            (Mode *)&plane.mode_manual,
            (Mode *)&plane.mode_fbwa,
        };

        int idx = 0;
        for (unsigned t = 0; t < 6; t++)
          for (unsigned r = 0; r < 4; r++)
            for (unsigned s = 0; s < 2; s++)
              for (unsigned g = 0; g < 2; g++)
                for (unsigned fen = 0; fen < 2; fen++)
                  for (unsigned pr = 0; pr < 2; pr++) {
                    plane.control_mode = starts[s];
                    plane.previous_mode = (Mode *)&plane.mode_manual;
                    // A fence row needs in_fence_recovery() true, and the
                    // simplest way there is the current mode having been
                    // entered because of the breach.
                    plane.control_mode_reason = fen
                        ? ModeReason::FENCE_BREACHED
                        : ModeReason::RC_COMMAND;
                    plane.previous_mode_reason = pr
                        ? ModeReason::FENCE_BREACHED
                        : ModeReason::UNKNOWN;

                    // The four fence conditions, together or not at all.
                    hal.util->set_soft_armed(fen != 0);
                    plane.fence._enabled_fences = fen ? 0xFF : 0;
                    plane.fence._breached_fences = fen ? 0xFF : 0;
                    plane.fence._options.set(fen ? 1 : 0);   // DISABLE_MODE_CHANGE

                    // FLTMODE_GCSBLOCK: block CRUISE on odd rows.
                    plane.flight_mode_GCS_block.set(g ? (1 << 7) : 0);   // CRUISE

                    // Read before the call, because set_mode moves most
                    // of this. in_fence_recovery in particular is a function
                    // of the two reasons the call is about to overwrite.
                    const int gcs_enabled =
                        (int)plane.gcs_mode_enabled(targets[t]->mode_number());
                    const int is_vtol = (int)targets[t]->is_vtol_mode();
                    const int soft_armed = (int)hal.util->get_soft_armed();
                    const int fence_enabled = (int)plane.fence.enabled();
                    const int fence_option = (int)plane.fence.option_enabled(
                        AC_Fence::OPTIONS::DISABLE_MODE_CHANGE);
                    const int fence_breached = plane.fence.get_breaches() ? 1 : 0;
                    const int recovering = (int)plane.in_fence_recovery();

                    const int before_control =
                        (int)plane.control_mode->mode_number();
                    const int before_previous =
                        (int)plane.previous_mode->mode_number();
                    const int before_cr = reason_num(plane.control_mode_reason);
                    const int before_pr = reason_num(plane.previous_mode_reason);

                    g_text[0] = '\0';
                    g_text_calls = 0;
                    const bool ok = plane.set_mode(*targets[t], reasons[r]);

                    printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s\n",
                           idx++,
                           (int)targets[t]->mode_number(),
                           reason_num(reasons[r]),
                           gcs_enabled,
                           is_vtol, soft_armed, fence_enabled,
                           fence_option, fence_breached, recovering,
                           before_control, before_previous, before_cr, before_pr,
                           ok ? 1 : 0,
                           (int)plane.control_mode->mode_number(),
                           (int)plane.previous_mode->mode_number(),
                           reason_num(plane.control_mode_reason),
                           reason_num(plane.previous_mode_reason),
                           classify());
                }
    }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = plane_link.objects(stage_dir="/tmp/plane_mode_parity/vehicle")
    flags = list(plane_link.LINK_FLAGS) + ["-Wl,--wrap=" + SEND_TEXT]
    build(HARNESS, objects, BUILD, "ArduPlane/Plane.cpp", link_flags=flags)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
