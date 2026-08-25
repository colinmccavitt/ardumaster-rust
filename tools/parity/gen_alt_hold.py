"""Record ArduCopter's altitude-hold state machine from the real firmware.

The machine is a pure function of six flags and a climb rate, so the sweep is
exhaustive over the flags rather than sampled: 2^5 flag combinations against
all five spool states and a set of climb rates chosen to sit on both sides of
the two different zero thresholds.

Two things are recorded rather than assumed. The spool state is set directly
on the motors, because the machine reads where the motors *are* and the
command it issues does not move them. And `takeoff.running()` is read back
after being set, so a row records what the firmware's own predicate answered
rather than what the harness intended.

The desired spool state is captured by reading `motors->_spool_desired` before
and after the call: a row where it did not move is a branch that issued no
command, which is a fact about the machine and not an absence of data.

What lands in that member is the *constrained* request, not the mode's ask.
`AP_MotorsMulticopter::set_desired_spool_state` shuts down whatever it is
handed while disarmed or while the motor interlock is open, so the recording
is of the two layers composed. The Rust side composes the same two, which is
the point: it verifies the constraint has a home in the port rather than
having been dropped between the crates.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/plane-fw-rust")
OUT = ROOT / "fixtures/alt_hold_state.csv"
BUILD = Path("/tmp/alt_hold_parity/harness")

HARNESS = r'''
#include <AP_HAL/AP_HAL.h>

// Visibility only, for this translation unit; the firmware objects linked
// against are untouched. See tools/parity/gen_pilot_input.py.
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

int main()
{
    AP::scheduler().init(nullptr, 0, 0);
    copter.allocate_motors();
    if (copter.motors == nullptr) {
        fprintf(stderr, "motors still null after allocate_motors\n");
        return 1;
    }

    // Disarming otherwise saves the learned hover throttle, and that save
    // waits on a clock a harness never advances. See the module docstring.
    copter.motors->_throttle_hover_learn.set(
        (uint8_t)AP_MotorsMulticopter::HOVER_LEARN_DISABLED);

    Mode *mode = &copter.mode_stabilize;

    // Both thresholds the machine tests around zero, from both sides:
    // triggered_ms rejects <= 0, the landed branch commands ground idle on
    // < 0, so exact zero is the row that tells them apart.
    const float rates[] = {-2.5f, -0.001f, 0.0f, 0.001f, 2.5f};

    const AP_Motors::SpoolState spools[] = {
        AP_Motors::SpoolState::SHUT_DOWN,
        AP_Motors::SpoolState::GROUND_IDLE,
        AP_Motors::SpoolState::SPOOLING_UP,
        AP_Motors::SpoolState::THROTTLE_UNLIMITED,
        AP_Motors::SpoolState::SPOOLING_DOWN,
    };

    printf("#alt_hold\n");
    printf("idx,armed,spool,takeoff_running,auto_armed,land_complete,"
           "using_interlock,motor_interlock,rate,state,desired,commanded\n");

    int idx = 0;
    for (unsigned flags = 0; flags < 64; flags++) {
        const bool armed           = (flags & 1) != 0;
        const bool takeoff_running = (flags & 2) != 0;
        const bool auto_armed      = (flags & 4) != 0;
        const bool land_complete   = (flags & 8) != 0;
        const bool interlock       = (flags & 16) != 0;
        // The motors' own interlock, distinct from copter.ap.using_interlock:
        // this one gates what set_desired_spool_state will accept, the other
        // is a Copter flag the state machine reads. Both are swept.
        const bool motor_interlock = (flags & 32) != 0;

        for (unsigned s = 0; s < 5; s++) {
            for (unsigned r = 0; r < 5; r++) {
                copter.motors->armed(armed);
                copter.motors->set_interlock(motor_interlock);
                copter.motors->_spool_state = spools[s];
                Mode::takeoff._running = takeoff_running;
                copter.ap.auto_armed = auto_armed;
                copter.ap.land_complete = land_complete;
                copter.ap.using_interlock = interlock;

                // A sentinel the machine never commands, so "did not move" is
                // distinguishable from "commanded what was already there".
                copter.motors->_spool_desired =
                    (AP_Motors::DesiredSpoolState)7;

                const Mode::AltHoldModeState st =
                    mode->get_alt_hold_state_D_ms(rates[r]);

                const int desired = (int)copter.motors->_spool_desired;
                const int commanded = (desired == 7) ? 0 : 1;

                printf("%d,%d,%d,%d,%d,%d,%d,%d,%u,%d,%d,%d\n", idx++,
                       (int)armed, (int)spools[s],
                       (int)Mode::takeoff.running(),
                       (int)auto_armed, (int)land_complete, (int)interlock,
                       (int)motor_interlock,
                       fbits(rates[r]), (int)st,
                       commanded ? desired : -1, commanded);
            }
        }
    }

    // Safe the vehicle before leaving, so the exit path is not the one
    // reached with the motors armed.
    copter.motors->armed(false);
    copter.motors->set_interlock(false);

    // Every row is written by this point. What follows a plain return is the
    // destruction of a global Copter and whatever the SITL HAL started behind
    // it, which spins rather than exiting and is no part of what is being
    // recorded. Leave without it.
    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/alt_hold_parity/vehicle")
    build(HARNESS, objects, BUILD, "ArduCopter/Copter.cpp",
          link_flags=vehicle_link.LINK_FLAGS)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
