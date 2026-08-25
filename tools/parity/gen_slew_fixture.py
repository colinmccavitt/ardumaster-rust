"""Dump the SRV_Channels slew limiter from the real firmware.

Its own binary. The slew list is a file-scope static that persists for the
process, and it is only ever appended to -- there is no removal -- so a harness
sharing it with any other SRV_Channels sequence would inherit whatever entries
that sequence installed.

The two readers are deliberately interleaved. get_slew_limited_output_scaled
peeks without advancing the history while calc_pwm both enforces and advances
it, so a recording that called only one of them would not show the difference.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/plane-fw-rust")
OUT = ROOT / "fixtures/srv_slew.csv"
BUILD = Path("/tmp/slew_parity/harness")

HARNESS = r'''
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
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

static uint32_t fbits(float f)
{
    uint32_t u;
    memcpy(&u, &f, sizeof(u));
    return u;
}

int main()
{

    const SRV_Channel::Function THR = SRV_Channel::k_throttle;
    const SRV_Channel::Function FLAP = SRV_Channel::k_flap;
    const SRV_Channel::Function ELEV = SRV_Channel::k_elevator;

    printf("#functions\n");
    printf("throttle,flap,elevator\n");
    printf("%d,%d,%d\n", (int)THR, (int)FLAP, (int)ELEV);

    // ---- a slew-limited step, read both ways ----
    //
    // Throttle takes a real limit, flap takes zero (an entry is still made,
    // and it must keep tracking), and elevator gets none at all -- three
    // states the code treats differently.
    printf("#slew\n");
    printf("step,rate,demand,peek_thr,after_thr,peek_flap,after_flap,"
           "peek_elev,after_elev\n");
    {
        const float dt = 0.02f;
        SRV_Channels::set_slew_rate(THR, 60.0f, 100, dt);
        SRV_Channels::set_slew_rate(FLAP, 0.0f, 100, dt);

        for (int i = 0; i < 300; i++) {
            // A square wave, so the limiter is asked to chase a step it
            // cannot reach in one go, in both directions.
            float demand = 0.0f;
            if (i < 40)        demand = 0.0f;
            else if (i < 120)  demand = 90.0f;
            else if (i < 200)  demand = -70.0f;
            else               demand = 20.0f;

            // Plane calls this every loop from set_servos, so the harness
            // does too. The rate steps down partway and then to zero, which
            // exercises updating an existing entry as well as creating it,
            // and covers a limit switched off while the entry keeps tracking.
            float rate = 60.0f;
            if (i >= 250)      rate = 0.0f;
            else if (i >= 150) rate = 8.0f;
            SRV_Channels::set_slew_rate(THR, rate, 100, dt);

            SRV_Channels::set_output_scaled(THR, demand);
            SRV_Channels::set_output_scaled(FLAP, demand);
            SRV_Channels::set_output_scaled(ELEV, demand);

            // Peek BEFORE calc_pwm: this must not advance the history.
            const float peek_thr = SRV_Channels::get_slew_limited_output_scaled(THR);
            const float peek_flap = SRV_Channels::get_slew_limited_output_scaled(FLAP);
            const float peek_elev = SRV_Channels::get_slew_limited_output_scaled(ELEV);

            SRV_Channels::calc_pwm();

            printf("%d,%u,%u,%u,%u,%u,%u,%u,%u\n", i,
                   fbits(rate), fbits(demand),
                   fbits(peek_thr),
                   fbits(SRV_Channels::get_output_scaled(THR)),
                   fbits(peek_flap),
                   fbits(SRV_Channels::get_output_scaled(FLAP)),
                   fbits(peek_elev),
                   fbits(SRV_Channels::get_output_scaled(ELEV)));
        }
    }

    // ---- peeking repeatedly must not move anything ----
    printf("#peek\n");
    printf("idx,peek\n");
    {
        SRV_Channels::set_output_scaled(THR, 500.0f);
        for (int i = 0; i < 8; i++) {
            printf("%d,%u\n", i,
                   fbits(SRV_Channels::get_slew_limited_output_scaled(THR)));
        }
    }

    return 0;
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/slew_parity/vehicle")
    build(HARNESS, objects, BUILD,
          "SRV_Channel/SRV_Channels.cpp",
          link_flags=vehicle_link.LINK_FLAGS)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
