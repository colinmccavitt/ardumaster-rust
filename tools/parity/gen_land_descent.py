"""Record Mode::land_run_vertical_control's descent demand from the firmware.

# Observing a local

The climb rate this function computes is a local variable. It leaves only as
an argument to `pos_control->D_set_pos_target_from_climb_rate_ms`, and that
method immediately shapes it through the jerk limiter, so reading the position
controller's state afterwards recovers the shaped value rather than the demand.

Transcribing the arithmetic into the harness to observe it would compare the
port against a C++ copy of itself, which is what vehicle_link exists to
prevent. So the call is intercepted instead: `-Wl,--wrap` on the mangled
symbol replaces every call site's target with `__wrap_...`, which records the
arguments and then forwards to `__real_...`. The firmware objects are not
modified and the real function still runs; what is recorded is the actual
argument at the actual call boundary.

# What is and is not swept

Precision landing is compiled in -- AC_PRECLAND_ENABLED defaults to 1 -- but
`precland.target_acquired()` is false without a backend and a target, so the
recorded rows are the branch taken when precision landing is not active. That
is the path every ordinary landing takes. The precland adjustment is a
separate slice and is not ported here; see the module documentation on
ap_copter::land for where the boundary is drawn.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/plane-fw-rust")
OUT = ROOT / "fixtures/land_descent.csv"
BUILD = Path("/tmp/land_descent_parity/harness")

WRAPPED = "_ZN13AC_PosControl35D_set_pos_target_from_climb_rate_msEfb"

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

// The interception. --wrap redirects every call to the real symbol here; the
// real one is still reachable as __real_. Nothing in the firmware is edited,
// and the real function still runs afterwards, so the vehicle state evolves
// exactly as it would have.
extern "C" void __real__ZN13AC_PosControl35D_set_pos_target_from_climb_rate_msEfb(
    void *self, float climb_rate_ms, bool ignore_descent_limit);

// Declared before it is defined: the build treats a definition with no
// previous declaration as an error.
extern "C" void __wrap__ZN13AC_PosControl35D_set_pos_target_from_climb_rate_msEfb(
    void *self, float climb_rate_ms, bool ignore_descent_limit);

static float g_climb_rate_ms = 0.0f;
static bool g_ignore_descent_limit = false;
static int g_calls = 0;

extern "C" void __wrap__ZN13AC_PosControl35D_set_pos_target_from_climb_rate_msEfb(
    void *self, float climb_rate_ms, bool ignore_descent_limit)
{
    g_climb_rate_ms = climb_rate_ms;
    g_ignore_descent_limit = ignore_descent_limit;
    g_calls++;
    __real__ZN13AC_PosControl35D_set_pos_target_from_climb_rate_msEfb(
        self, climb_rate_ms, ignore_descent_limit);
}

int main()
{
    AP::scheduler().init(nullptr, 0, 0);
    copter.allocate_motors();
    copter.motors->_throttle_hover_learn.set(0);

    AC_PosControl *pos_control = copter.pos_control;
    if (pos_control == nullptr) {
        fprintf(stderr, "pos_control null\n");
        return 1;
    }

    // The vertical position controller's gains. Read back and recorded rather
    // than assumed: the demand is a sqrt_controller over them, and a harness
    // that left them at zero would record a controller that always returns
    // zero -- which is how four attitude tests were once silently no-ops.
    pos_control->_vel_max_down_ms = 1.5f;

    // The loop period. It is a reference into Copter, and nothing sets it
    // without the scheduler loop running; left at zero the sqrt controller
    // would be asked for a step it cannot take.
    copter.mode_land.G_Dt = 0.0025f;

    printf("#descent\n");
    printf("idx,pause,alt_agl,land_alt_low,land_speed_high,land_speed,"
           "max_speed_down,kp,accel_max,dt,land_complete_maybe,"
           "climb_rate,ignore_limit\n");

    // Heights relative to the *effective* slowdown height, so that the band
    // where the demand is neither clamped to the floor nor to the ceiling is
    // actually visited. The offsets straddle the point where sqrt_controller
    // leaves its linear region, which sits at accel / (2 * kP^2).
    const float offsets[] = {-4.0f, -1.0f, 0.0f, 0.05f, 0.2f, 0.5f,
                             1.0f, 1.3f, 2.0f, 4.0f, 9.0f};
    // Slowdown heights either side of the hard floor of 1 metre the code
    // imposes with MAX(land_alt_low_m, 1).
    const float lows[] = {0.0f, 1.0f, 3.0f, 10.0f};
    // No separate high-speed descent (falls back to the controller's own
    // limit), one slower than the final speed, and one fast enough to leave
    // the square-root region room before the ceiling cuts it off.
    const float highs[] = {0.0f, 0.35f, 2.0f, 6.0f};
    const float finals[] = {0.2f, 1.5f};
    const float kps[] = {0.5f, 1.0f, 2.0f};
    const float accels[] = {1.0f, 2.5f};

    int idx = 0;
    for (unsigned p = 0; p < 2; p++)
      for (unsigned a = 0; a < 11; a++)
        for (unsigned l = 0; l < 4; l++)
          for (unsigned h = 0; h < 4; h++)
            for (unsigned s = 0; s < 2; s++)
              for (unsigned k = 0; k < 3; k++)
                for (unsigned c = 0; c < 2; c++)
                  for (unsigned m = 0; m < 2; m++) {
                  const bool pause = (p != 0);
                  const bool maybe = (m != 0);

                  copter.mode_land.land_alt_low_m.set(lows[l]);
                  copter.mode_land.land_speed_high_ms.set(highs[h]);
                  copter.mode_land.land_speed_ms.set(finals[s]);
                  copter.ap.land_complete_maybe = maybe;
                  pos_control->_p_pos_d_m.kP().set(kps[k]);
                  pos_control->_accel_max_d_mss = accels[c];

                  // The height above ground comes from current_loc with no
                  // rangefinder and no terrain, which is the flat-earth
                  // fallback: altitude in centimetres times 0.01.
                  const float alt = MAX(lows[l], 1.0f) + offsets[a];
                  copter.current_loc.set_alt_cm((int32_t)(alt * 100.0f),
                                                Location::AltFrame::ABOVE_HOME);

                  const float agl = copter.mode_land.get_alt_above_ground_m();

                  g_calls = 0;
                  copter.mode_land.land_run_vertical_control(pause);
                  if (g_calls != 1) {
                      fprintf(stderr, "row %d: %d calls, expected 1\n",
                              idx, g_calls);
                      return 1;
                  }

                  printf("%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%d,%u,%d\n", idx++,
                         (int)pause,
                         fbits(agl),
                         fbits(copter.mode_land.get_land_alt_low_m()),
                         fbits(copter.mode_land.get_land_speed_high_ms()),
                         fbits(copter.mode_land.get_land_speed_ms()),
                         fbits(pos_control->get_max_speed_down_ms()),
                         fbits(pos_control->D_get_pos_p().kP()),
                         fbits(pos_control->D_get_max_accel_mss()),
                         fbits(copter.mode_land.G_Dt),
                         (int)maybe,
                         fbits(g_climb_rate_ms),
                         (int)g_ignore_descent_limit);
              }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/land_descent_parity/vehicle")
    flags = list(vehicle_link.LINK_FLAGS) + ["-Wl,--wrap=" + WRAPPED]
    build(HARNESS, objects, BUILD, "ArduCopter/Copter.cpp", link_flags=flags)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
