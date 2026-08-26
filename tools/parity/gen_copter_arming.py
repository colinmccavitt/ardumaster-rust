"""Record ArduCopter's pre-arm checks from the firmware.

Each check is a method on AP_Arming_Copter returning a bool, so they are
called directly with the state driven around them. What is NOT recorded from
the return value is *which* refusal fired -- every one of them returns false
-- so the message is captured too, the same way b26e64c captured mode-change
refusals: GCS::send_text is an undefined reference in AP_Arming_Copter.cpp.o
and carries the text check_failed formats.

Verified with nm -u before relying on it, which is now the first step rather
than the diagnosis.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/plane-fw-rust")
OUT = ROOT / "fixtures/copter_arming.csv"
BUILD = Path("/tmp/arming_parity/harness")

# Both overloads of check_failed; they are what carries the refusal.
CHECK_FAILED = "_ZNK9AP_Arming12check_failedEbPKcz"
CHECK_FAILED_TAGGED = "_ZNK9AP_Arming12check_failedENS_5CheckEbPKcz"

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

static char g_text[192];
static int g_text_calls = 0;

// Both overloads of check_failed. GCS::send_text is not referenced from this
// translation unit, so wrapping that would have captured nothing; these carry
// the reason directly, before it is formatted for the pilot.
extern "C" void __wrap__ZNK9AP_Arming12check_failedEbPKcz(
    const void *self, bool report, const char *fmt, ...);
extern "C" void __wrap__ZNK9AP_Arming12check_failedEbPKcz(
    const void *self, bool report, const char *fmt, ...)
{
    (void)self; (void)report;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_text, sizeof(g_text), fmt, ap);
    va_end(ap);
    g_text_calls++;
}

extern "C" void __wrap__ZNK9AP_Arming12check_failedENS_5CheckEbPKcz(
    const void *self, int check, bool report, const char *fmt, ...);
extern "C" void __wrap__ZNK9AP_Arming12check_failedENS_5CheckEbPKcz(
    const void *self, int check, bool report, const char *fmt, ...)
{
    (void)self; (void)check; (void)report;
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_text, sizeof(g_text), fmt, ap);
    va_end(ap);
    g_text_calls++;
}

static const char *reason(void)
{
    return g_text_calls == 0 ? "-" : g_text;
}

int main()
{
    AP::scheduler().init(nullptr, 0, 0);
    copter.allocate_motors();
    copter.motors->_throttle_hover_learn.set(0);
    setvbuf(stdout, NULL, _IOLBF, 0);

    copter.channel_throttle = rc().channel(2);
    copter.channel_throttle->radio_min.set(1000);
    copter.channel_throttle->radio_max.set(2000);

    AP_Arming_Copter &arming = copter.arming;

    // ---- rc_throttle_failsafe_checks ----
    printf("#rc_failsafe\n");
    printf("idx,rc_check_enabled,fs_thr,had_receiver,had_override,"
           "radio_in,fs_thr_value,passed,reason\n");
    {
        const uint8_t fs_thr[] = {0, 1, 2};
        const uint16_t radio_ins[] = {0, 900, 975, 1000, 1500};
        const uint16_t thresholds[] = {975, 1100};
        int idx = 0;
        for (unsigned ce = 0; ce < 2; ce++)
          for (unsigned ft = 0; ft < 3; ft++)
            for (unsigned hr = 0; hr < 2; hr++)
              for (unsigned ho = 0; ho < 2; ho++)
                for (unsigned ri = 0; ri < 5; ri++)
                  for (unsigned th = 0; th < 2; th++) {
                      // checks_to_skip is a SKIP mask, so setting the RC
                      // bit (1<<6) disables the RC check. Zero enables
                      // everything.
                      arming.checks_to_skip.set(ce ? 0 : (1 << 6));
                      copter.g.failsafe_throttle.set(fs_thr[ft]);
                      rc()._has_had_rc_receiver = (hr != 0);
                      rc()._has_had_override = (ho != 0);
                      copter.channel_throttle->radio_in = radio_ins[ri];
                      copter.g.failsafe_throttle_value.set(thresholds[th]);

                      g_text[0] = '\0';
                      g_text_calls = 0;
                      const bool ok = arming.rc_throttle_failsafe_checks(true);

                      printf("%d,%d,%d,%d,%d,%d,%d,%d,%s\n", idx++,
                             (int)arming.check_enabled(AP_Arming::Check::RC),
                             (int)fs_thr[ft], (int)(hr != 0), (int)(ho != 0),
                             (int)radio_ins[ri], (int)thresholds[th],
                             ok ? 1 : 0, reason());
                  }
    }

    // ---- gcs_failsafe_check ----
    printf("#gcs\n");
    printf("gcs_failsafe,passed,reason\n");
    for (unsigned g = 0; g < 2; g++) {
        copter.failsafe.gcs = (g != 0);
        g_text[0] = '\0';
        g_text_calls = 0;
        const bool ok = arming.gcs_failsafe_check(true);
        printf("%d,%d,%s\n", (int)(g != 0), ok ? 1 : 0, reason());
    }

    // ---- alt_checks ----
    printf("#alt\n");
    printf("mode,manual_throttle,passed,reason\n");
    {
        Mode *modes[] = {
            (Mode *)&copter.mode_stabilize,
            (Mode *)&copter.mode_althold,
            (Mode *)&copter.mode_loiter,
            (Mode *)&copter.mode_land,
        };
        for (unsigned m = 0; m < 4; m++) {
            copter.flightmode = modes[m];
            g_text[0] = '\0';
            g_text_calls = 0;
            const bool ok = arming.alt_checks(true);
            printf("%d,%d,%d,%s\n",
                   (int)modes[m]->mode_number(),
                   (int)modes[m]->has_manual_throttle(),
                   ok ? 1 : 0, reason());
        }
    }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/arming_parity/vehicle")
    flags = list(vehicle_link.LINK_FLAGS) + [
        "-Wl,--wrap=" + CHECK_FAILED,
        "-Wl,--wrap=" + CHECK_FAILED_TAGGED,
    ]
    build(HARNESS, objects, BUILD, "ArduCopter/Copter.cpp", link_flags=flags)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
