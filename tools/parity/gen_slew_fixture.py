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
#include <AP_Scheduler/AP_Scheduler.h>
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
    AP::scheduler().init(nullptr, 0, 0);


    const SRV_Channel::Function THR = SRV_Channel::k_throttle;
    const SRV_Channel::Function FLAP = SRV_Channel::k_flap;
    const SRV_Channel::Function ELEV = SRV_Channel::k_elevator;

    // The override sequence below drives this channel; it is set up here so
    // its conversion limits can be recorded alongside everything else the
    // port needs, in one place.
    const uint8_t CHAN = 4;
    SRV_Channels::set_default_function(CHAN, ELEV);
    SRV_Channels::update_aux_servo_function();
    const SRV_Channel *cfg = SRV_Channels::srv_channel(CHAN);

    printf("#functions\n");
    printf("throttle,flap,elevator,num_servo_channels,loop_period_us,"
           "chan,servo_min,servo_trim,servo_max,servo_reversed\n");
    printf("%d,%d,%d,%d,%u,%d,%u,%u,%u,%d\n",
           (int)THR, (int)FLAP, (int)ELEV,
           (int)NUM_SERVO_CHANNELS,
           (unsigned)AP::scheduler().get_loop_period_us(),
           (int)CHAN,
           (unsigned)cfg->get_output_min(),
           (unsigned)cfg->get_trim(),
           (unsigned)cfg->get_output_max(),
           cfg->get_reversed() ? 1 : 0);

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

    // ---- the override counter ----
    //
    // The timeout is in milliseconds but the mechanism counts loops, so the
    // conversion rounds up: a request shorter than one loop still gets one.
    // A timeout of zero is documented as clearing the override -- it sets the
    // flag anyway and lets the next calc_pwm clear it, so the override lasts
    // the rest of the current loop and no longer.
    //
    // Neither the counter nor the flag is reachable from outside
    // SRV_Channels, so the pulse width is what is recorded. That is the point
    // of the mechanism, and the scaled value sweeps underneath it so the
    // moment the override lapses the output visibly returns to tracking it.
    printf("#override\n");
    printf("step,request_ms,pwm,scaled,out_pwm\n");
    {

        for (int i = 0; i < 60; i++) {
            // Sweeping, so an overridden channel is visibly not following it.
            const float scaled = -4000.0f + 150.0f * i;
            SRV_Channels::set_output_scaled(ELEV, scaled);

            uint16_t request = 0;
            bool ask = false;
            if (i == 5)       { request = 20;  ask = true; }
            else if (i == 25) { request = 0;   ask = true; }
            else if (i == 35) { request = 1;   ask = true; }
            else if (i == 45) { request = 60000; ask = true; }

            if (ask) {
                SRV_Channels::set_output_pwm_chan_timeout(CHAN, 1200 + i, request);
            }

            SRV_Channels::calc_pwm();

            uint16_t out = 0;
            SRV_Channels::get_output_pwm_chan(CHAN, out);

            printf("%d,%d,%d,%u,%u\n", i,
                   ask ? (int)request : -1,
                   ask ? (int)(1200 + i) : -1,
                   fbits(scaled),
                   (unsigned)out);
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
