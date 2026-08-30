"""Record the pilot's part in a landing: cancel, reposition, precland handover.

Three observations, none of which is a return value -- the function returns
nothing and does its work through vehicle state and controller calls.

  * The landing cancel really changes mode, so the flight mode after the call
    is what shows it. Copter::set_mode is defined in mode.cpp and called from
    mode.cpp, so --wrap cannot see it (the trap from gen_mode_entry.py); the
    mode change is allowed to happen and observed instead.

  * land_repo_active and prec_land_active are Copter flags, read directly.

  * AC_PrecLand::target_acquired and AC_PosControl::NE_soften_for_landing are
    undefined references in mode.cpp.o (checked with nm -u), so the precland
    branch can be driven and the softening observed without a precision
    landing system.

The pilot's repositioning velocity comes from the real conversion, driven
through radio_in as in gen_pilot_input.py -- not injected, because whether a
given stick counts as "repositioning" is part of what is being recorded.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/ardumaster-rust")
OUT = ROOT / "fixtures/copter_land_horizontal.csv"
BUILD = Path("/tmp/land_h_parity/harness")

TARGET_ACQUIRED = "_ZN11AC_PrecLand15target_acquiredEv"
SOFTEN = "_ZN13AC_PosControl21NE_soften_for_landingEv"

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

static bool g_target_acquired = false;
static int g_soften_calls = 0;

extern "C" bool __wrap__ZN11AC_PrecLand15target_acquiredEv(void *self);
extern "C" bool __wrap__ZN11AC_PrecLand15target_acquiredEv(void *self)
{ (void)self; return g_target_acquired; }

extern "C" void __wrap__ZN13AC_PosControl21NE_soften_for_landingEv(void *self);
extern "C" void __wrap__ZN13AC_PosControl21NE_soften_for_landingEv(void *self)
{ (void)self; g_soften_calls++; }

int main()
{
    AP::scheduler().init(nullptr, 0, 0);
    copter.allocate_motors();
    copter.motors->_throttle_hover_learn.set(0);
    setvbuf(stdout, NULL, _IOLBF, 0);

    copter.channel_roll = rc().channel(0);
    copter.channel_pitch = rc().channel(1);
    copter.channel_throttle = rc().channel(2);
    copter.channel_yaw = rc().channel(3);
    for (uint8_t i = 0; i < 4; i++) {
        RC_Channel *ch = rc().channel(i);
        ch->radio_min.set(1000);
        ch->radio_trim.set(1500);
        ch->radio_max.set(2000);
        ch->dead_zone.set(20);
    }
    copter.channel_roll->set_angle(4500);
    copter.channel_pitch->set_angle(4500);
    rc()._has_ever_seen_rc_input = true;

    printf("#land_horizontal\n");
    printf("idx,land_complete_maybe,thr_behave,thr_filtered,repositioning,"
           "roll_in,repo_active_before,target_acquired,allow_after_repo,"
           "mode_after,repo_active_after,prec_land_active,softened\n");

    // Centre stick and a clear deflection, so "is the pilot repositioning"
    // varies.
    const int16_t sticks[] = {1500, 1800};
    const float throttles[] = {100.0f, 699.0f, 701.0f, 950.0f};
    const int behaves[] = {0, 2};

    int idx = 0;
    for (unsigned lcm = 0; lcm < 2; lcm++)
      for (unsigned bh = 0; bh < 2; bh++)
        for (unsigned th = 0; th < 4; th++)
          for (unsigned rp = 0; rp < 2; rp++)
            for (unsigned st = 0; st < 2; st++)
              for (unsigned ra = 0; ra < 2; ra++)
                for (unsigned ta = 0; ta < 2; ta++)
                  for (unsigned ap = 0; ap < 2; ap++) {
                      // A landing in progress, every time.
                      copter.flightmode = (Mode *)&copter.mode_land;
                      copter.ap.land_complete_maybe = (lcm != 0);
                      copter.g.throttle_behavior.set(behaves[bh]);
                      copter.rc_throttle_control_in_filter.reset(throttles[th]);
                      copter.g.land_repositioning.set(rp != 0);
                      copter.channel_roll->set_radio_in(sticks[st]);
                      copter.channel_pitch->set_radio_in(1500);
                      copter.ap.land_repo_active = (ra != 0);
                      g_target_acquired = (ta != 0);
                      copter.precland._options.set(ap ? 2 : 0);   // PLND_OPTION_PRECLAND_AFTER_REPOSITION is 1<<1

                      g_soften_calls = 0;
                      copter.mode_land.land_run_horizontal_control();

                      printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", idx++,
                             (int)(lcm != 0), behaves[bh], (int)throttles[th],
                             (int)(rp != 0), (int)sticks[st],
                             (int)(ra != 0), (int)(ta != 0),
                             (int)copter.precland.allow_precland_after_reposition(),
                             (int)copter.flightmode->mode_number(),
                             (int)copter.ap.land_repo_active,
                             (int)copter.ap.prec_land_active,
                             g_soften_calls > 0 ? 1 : 0);
                  }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/land_h_parity/vehicle")
    flags = list(vehicle_link.LINK_FLAGS) + [
        "-Wl,--wrap=" + TARGET_ACQUIRED,
        "-Wl,--wrap=" + SOFTEN,
    ]
    build(HARNESS, objects, BUILD, "ArduCopter/Copter.cpp", link_flags=flags)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
