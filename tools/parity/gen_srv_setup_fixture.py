"""Dump the per-function default output shapes from the real firmware.

Its own binary rather than a section of the motors harness, and that is the
whole point. A channel latches `type_setup` the first time it is given a
default shape and nothing public clears it, so this sweep needs channels
nothing else has touched. Sharing a process with the motor fixtures meant
either claiming every channel -- which drove `invalid_mask` to zero and
silently removed a case another test depends on -- or claiming most of them,
which moved the motor mask `update_throttle_range` measures. Sections of one
harness are not independent.

A second binary is cheap now: the harness has no stubs to duplicate. It links
the same firmware objects through vehicle_link and consists of includes, main,
and the sweep.

# What it measures

`high_out` and `type_angle` are private, so the shape is not readable. It is
recovered through the conversion instead, which is what the shape is for: with
SERVOn_MIN 1100, MAX 1900 and TRIM 1500, a range maps 0 to the minimum and an
angle maps 0 to the trim, and the slope separates the groups.
"""
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/ardumaster-rust")
OUT = ROOT / "fixtures/srv_setup_sweep.csv"
PREDICATES_OUT = ROOT / "fixtures/srv_predicates.csv"
BUILD = Path("/tmp/srv_setup_parity/harness")

# The functions to sweep, chosen from the source by pick_funcs.
PICKED = subprocess.run(
    [sys.executable, str(Path(__file__).parent / "pick_funcs.py"), "--list"],
    capture_output=True,
    text=True,
    check=True,
).stdout.strip()

HARNESS = r'''
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <SRV_Channel/SRV_Channel.h>
#include <cstdarg>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace AP_HAL {
    void panic(const char *m, ...)
    {
        va_list ap;
        va_start(ap, m);
        vfprintf(stderr, m, ap);
        va_end(ap);
        fputc('\n', stderr);
        abort();
    }
}

extern const AP_HAL::HAL &hal;

static uint32_t fbits(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }

int main(void)
{
    // Nothing has touched SRV_Channels in this process, so every channel is
    // still at its default and none has latched a shape.
    AP::scheduler().init(nullptr, 0, 0);

    // The classification predicates first: they are statics that read no
    // channel state, so they cost nothing and cannot disturb the sweep below.
    printf("#predicates\n");
    printf("function,is_motor,e_stop,is_surface\n");
    for (unsigned fn = 0; fn < 190; fn++) {
        const SRV_Channel::Function f = (SRV_Channel::Function)fn;
        printf("%u,%d,%d,%d\n", fn,
               SRV_Channel::is_motor(f) ? 1 : 0,
               SRV_Channel::should_e_stop(f) ? 1 : 0,
               SRV_Channel::is_control_surface(f) ? 1 : 0);
    }

    printf("#shapes\n");
    printf("function,scaled,pwm\n");

    static const uint16_t FUNCS[] = { __FUNCS__ };
    const unsigned nf = sizeof(FUNCS) / sizeof(FUNCS[0]);

    // Enough probes to separate every group by slope as well as by origin.
    static const float PROBES[] = { 0.0f, 25.0f, 50.0f, 500.0f, 1000.0f };
    const unsigned np = sizeof(PROBES) / sizeof(PROBES[0]);

    for (unsigned i = 0; i < nf; i++) {
        SRV_Channels::set_default_function(
            (uint8_t)i, (SRV_Channel::Function)FUNCS[i]);
    }
    SRV_Channels::update_aux_servo_function();

    for (unsigned pi = 0; pi < np; pi++) {
        for (unsigned i = 0; i < nf; i++) {
            SRV_Channels::set_output_scaled(
                (SRV_Channel::Function)FUNCS[i], PROBES[pi]);
        }
        SRV_Channels::calc_pwm();
        for (unsigned i = 0; i < nf; i++) {
            const SRV_Channel *c = SRV_Channels::srv_channel((uint8_t)i);
            printf("%u,%u,%u\n", (unsigned)FUNCS[i], fbits(PROBES[pi]),
                   (unsigned)(c ? c->get_output_pwm() : 0));
        }
    }

    return 0;
}
'''.replace("__FUNCS__", PICKED)


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/srv_setup_parity/vehicle")
    build(HARNESS, objects, BUILD, "SRV_Channel/SRV_Channels.cpp",
          link_flags=vehicle_link.LINK_FLAGS)
    text = run(BUILD)

    marker = "#shapes\n"
    head, shapes = text.split(marker, 1)
    PREDICATES_OUT.write_text(head)
    OUT.write_text(marker + shapes)

    for path, body in ((PREDICATES_OUT, head), (OUT, shapes)):
        rows = sum(1 for l in body.splitlines()
                   if l and not l.startswith("#") and not l.startswith("function,"))
        print("wrote %s: %d rows" % (path.name, rows))


main()
