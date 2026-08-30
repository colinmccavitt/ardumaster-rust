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

ROOT = Path("/srv/ardumaster/ports/ardumaster-rust")
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

    // ---- the function-scoped setters ----
    //
    // Two channels share ELEV, one reversed, because every setter here
    // resolves endpoints per channel: the same Limit sends a reversed channel
    // and an upright one in opposite directions, which is the whole point and
    // is invisible if they both point the same way.
    printf("#setters\n");
    printf("case,chan,reversed,servo_min,servo_trim,servo_max,out_pwm\n");
    {
        const uint8_t A = 6;
        const uint8_t B = 7;
        SRV_Channels::set_default_function(A, ELEV);
        SRV_Channels::set_default_function(B, ELEV);
        SRV_Channels::update_aux_servo_function();

        SRV_Channel *ca = SRV_Channels::srv_channel(A);
        SRV_Channel *cb = SRV_Channels::srv_channel(B);
        ca->set_output_min(1100);
        ca->set_output_max(1900);
        cb->set_output_min(1000);
        cb->set_output_max(2000);
        cb->reversed_set_and_save_ifchanged(true);
        SRV_Channels::set_trim_to_pwm_for(ELEV, 1500);

        struct Step { int kind; int arg; };
        const Step steps[] = {
            {0, (int)SRV_Channel::Limit::TRIM},
            {0, (int)SRV_Channel::Limit::MIN},
            {0, (int)SRV_Channel::Limit::MAX},
            {0, (int)SRV_Channel::Limit::ZERO_PWM},
            {1, 0},
            {2, 1234},
            {3, 0},
            {0, (int)SRV_Channel::Limit::MIN},
            {3, 1},
            {0, (int)SRV_Channel::Limit::MIN},
        };

        for (unsigned s = 0; s < sizeof(steps)/sizeof(steps[0]); s++) {
            switch (steps[s].kind) {
            case 0:
                SRV_Channels::set_output_limit(ELEV, (SRV_Channel::Limit)steps[s].arg);
                break;
            case 1:
                SRV_Channels::set_output_to_trim(ELEV);
                break;
            case 2:
                SRV_Channels::set_trim_to_pwm_for(ELEV, (int16_t)steps[s].arg);
                break;
            case 3:
                SRV_Channels::set_trim_to_min_for(ELEV, steps[s].arg != 0);
                break;
            default:
                break;
            }

            const uint8_t chans[2] = {A, B};
            for (int k = 0; k < 2; k++) {
                SRV_Channel *c = SRV_Channels::srv_channel(chans[k]);
                printf("%u,%u,%d,%u,%u,%u,%u\n",
                       s, (unsigned)chans[k],
                       c->get_reversed() ? 1 : 0,
                       (unsigned)c->get_output_min(),
                       (unsigned)c->get_trim(),
                       (unsigned)c->get_output_max(),
                       (unsigned)c->get_output_pwm());
            }
        }
    }

    // ---- the normalised read ----
    //
    // Driven by scaled value, because the aggregate recomputes the width from
    // it before reading -- so this exercises the conversion and the
    // normalisation together, which is how the function is actually used.
    //
    // Two functions rather than two channels, because the aggregate reads the
    // FIRST channel carrying a function. AIL is reversed with asymmetric
    // travel, so the two halves scale differently and a port using one divisor
    // for both is caught.
    printf("#norm\n");
    printf("idx,func,chan,reversed,servo_min,servo_trim,servo_max,"
           "scaled,pwm,norm\n");
    {
        // Two functions of its own. ELEV is carried by channel 4 from the
        // override section above and is still frozen there, so reading it
        // would report a width set by a different sequence entirely.
        const SRV_Channel::Function UP = SRV_Channel::k_rudder;
        const SRV_Channel::Function REV = SRV_Channel::k_aileron;
        const uint8_t C = 9;
        const uint8_t D = 10;
        SRV_Channels::set_default_function(C, UP);
        SRV_Channels::set_default_function(D, REV);
        SRV_Channels::update_aux_servo_function();

        SRV_Channel *cc = SRV_Channels::srv_channel(C);
        cc->set_output_min(1100);
        cc->set_output_max(1900);
        SRV_Channels::set_trim_to_pwm_for(UP, 1500);

        SRV_Channel *cd = SRV_Channels::srv_channel(D);
        cd->set_output_min(1000);
        cd->set_output_max(2000);
        cd->reversed_set_and_save_ifchanged(true);
        SRV_Channels::set_trim_to_pwm_for(REV, 1600);

        const SRV_Channel::Function funcs[2] = {UP, REV};
        int idx = 0;
        for (int k = 0; k < 2; k++) {
            for (int s = -5000; s <= 5000; s += 250) {
                SRV_Channels::set_output_scaled(funcs[k], (float)s);
                const float norm = SRV_Channels::get_output_norm(funcs[k]);
                uint16_t pwm = 0;
                SRV_Channels::get_output_pwm(funcs[k], pwm);
                const uint8_t cn = (k == 0) ? C : D;
                const SRV_Channel *cfgc = SRV_Channels::srv_channel(cn);
                printf("%d,%d,%u,%d,%u,%u,%u,%d,%u,%u\n", idx++,
                       (int)funcs[k], (unsigned)cn,
                       cfgc->get_reversed() ? 1 : 0,
                       (unsigned)cfgc->get_output_min(),
                       (unsigned)cfgc->get_trim(),
                       (unsigned)cfgc->get_output_max(),
                       s, (unsigned)pwm, fbits(norm));
            }
        }
    }

    // ---- adjust_trim ----
    //
    // Held in one direction long enough to walk the trim into its bound and
    // then reversed, because the bounds are asymmetric -- up stops at 60% of
    // travel, down at 40% -- and a sequence that only pushes one way sees one
    // of them.
    //
    // Two channels again, one reversed, since the reversal flips the sense
    // before any of the bounds are consulted. Magnitudes vary deliberately:
    // only the sign is read, so a port scaling by v would drift away
    // immediately.
    printf("#adjtrim\n");
    printf("step,v,chan,servo_trim,trimmed_mask\n");
    {
        const SRV_Channel::Function TR = SRV_Channel::k_flap_auto;
        const uint8_t E = 11;
        const uint8_t F = 12;
        SRV_Channels::set_default_function(E, TR);
        SRV_Channels::set_default_function(F, TR);
        SRV_Channels::update_aux_servo_function();

        SRV_Channel *ce = SRV_Channels::srv_channel(E);
        SRV_Channel *cf = SRV_Channels::srv_channel(F);
        ce->set_output_min(1000);
        ce->set_output_max(2000);
        cf->set_output_min(1000);
        cf->set_output_max(2000);
        cf->reversed_set_and_save_ifchanged(true);
        SRV_Channels::set_trim_to_pwm_for(TR, 1500);

        for (int i = 0; i < 500; i++) {
            // Long enough in each direction that BOTH channels reach both
            // bounds -- 200 steps of travel between them, plus slack. The
            // zero stretch must move nothing at all.
            float v;
            if (i < 150)      v = 0.25f + 0.01f * (i % 7);
            else if (i < 160) v = 0.0f;
            else if (i < 400) v = -3.0f - 0.5f * (i % 5);
            else              v = 0.75f;

            AP::srv().adjust_trim(TR, v);

            const uint8_t chans[2] = {E, F};
            for (int k = 0; k < 2; k++) {
                printf("%d,%u,%u,%u,%u\n", i, fbits(v),
                       (unsigned)chans[k],
                       (unsigned)SRV_Channels::srv_channel(chans[k])->get_trim(),
                       0u);
            }
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
