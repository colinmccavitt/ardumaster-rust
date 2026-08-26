"""Record parameter_checks: a ladder where one rung warns without blocking.

The FS_GCS_ENABLE=2 rung calls check_failed and does NOT return, so the
operator is told their parameter is obsolete and arming continues. Every
other rung returns false. A recording of return values alone could not tell
that rung from a passing check, so the reason text is captured as well -- via
AP_Arming::check_failed, both overloads, which are undefined references in
AP_Arming_Copter.cpp.o.

Because that rung does not return, one call can produce TWO messages. The
harness records the first and the last, so a row that warned and then refused
is distinguishable from one that only refused.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/plane-fw-rust")
OUT = ROOT / "fixtures/copter_param_checks.csv"
BUILD = Path("/tmp/param_parity/harness")

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

static char g_first[160];
static char g_last[160];
static int g_calls = 0;

static void record(const char *fmt, va_list ap)
{
    char buf[160];
    vsnprintf(buf, sizeof(buf), fmt, ap);
    if (g_calls == 0) {
        snprintf(g_first, sizeof(g_first), "%s", buf);
    }
    snprintf(g_last, sizeof(g_last), "%s", buf);
    g_calls++;
}

extern "C" void __wrap__ZNK9AP_Arming12check_failedEbPKcz(
    const void *self, bool report, const char *fmt, ...);
extern "C" void __wrap__ZNK9AP_Arming12check_failedEbPKcz(
    const void *self, bool report, const char *fmt, ...)
{
    (void)self; (void)report;
    va_list ap; va_start(ap, fmt); record(fmt, ap); va_end(ap);
}

extern "C" void __wrap__ZNK9AP_Arming12check_failedENS_5CheckEbPKcz(
    const void *self, int check, bool report, const char *fmt, ...);
extern "C" void __wrap__ZNK9AP_Arming12check_failedENS_5CheckEbPKcz(
    const void *self, int check, bool report, const char *fmt, ...)
{
    (void)self; (void)check; (void)report;
    va_list ap; va_start(ap, fmt); record(fmt, ap); va_end(ap);
}

int main()
{
    AP::scheduler().init(nullptr, 0, 0);
    copter.allocate_motors();
    copter.motors->_throttle_hover_learn.set(0);
    setvbuf(stdout, NULL, _IOLBF, 0);

    copter.channel_throttle = rc().channel(2);
    AP_Arming_Copter &arming = copter.arming;

    printf("#parameters\n");
    printf("idx,enabled,fs_thr,rc3_min,fs_thr_value,fs_gcs,"
           "acro_roll,acro_pitch,angle_roll_p,angle_pitch_p,pilot_spd_up,"
           "adsb,passed,calls,messages\n");
    // The two messages are joined by a pipe and placed last: upstream's text
    // contains commas, so it cannot be a comma-separated field.

    // A configuration that passes everything, perturbed one axis at a time
    // plus the two-message case.
    const uint16_t rc3_mins[] = {1000, 915};
    const uint16_t fs_values[] = {905, 975};
    const uint8_t fs_thrs[] = {0, 1};
    const uint8_t fs_gcss[] = {0, 2};
    const float acro_rolls[] = {1.0f, -1.0f, 99.0f};
    const float spd_ups[] = {2.5f, 0.0f, -1.0f};

    int idx = 0;
    for (unsigned en = 0; en < 2; en++)
      for (unsigned ft = 0; ft < 2; ft++)
        for (unsigned rm = 0; rm < 2; rm++)
          for (unsigned fv = 0; fv < 2; fv++)
            for (unsigned fg = 0; fg < 2; fg++)
              for (unsigned ar = 0; ar < 3; ar++)
                for (unsigned su = 0; su < 3; su++)
                  for (unsigned ad = 0; ad < 2; ad++) {
                      // checks_to_skip is a SKIP mask: PARAMETERS is 1<<5.
                      arming.checks_to_skip.set(en ? 0 : (1 << 5));
                      copter.g.failsafe_throttle.set(fs_thrs[ft]);
                      copter.channel_throttle->radio_min.set(rc3_mins[rm]);
                      copter.g.failsafe_throttle_value.set(fs_values[fv]);
                      copter.g.failsafe_gcs.set(fs_gcss[fg]);
                      copter.g.acro_balance_roll.set(acro_rolls[ar]);
                      copter.g.acro_balance_pitch.set(1.0f);
                      copter.g2.pilot_speed_up_ms.set(spd_ups[su]);
                      copter.failsafe.adsb = (ad != 0);

                      // Keep the RTL terrain rung out of the way; it has its
                      // own inputs and is covered by reasoning in the test.
                      copter.g.rtl_alt_type.set(ModeRTL::RTLAltType::RELATIVE);

                      g_first[0] = g_last[0] = '\0';
                      g_calls = 0;
                      const bool ok = arming.parameter_checks(true);

                      printf("%d,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%d,%d,%d,%s|%s\n",
                             idx++,
                             (int)arming.check_enabled(AP_Arming::Check::PARAMETERS),
                             (int)fs_thrs[ft], (int)rc3_mins[rm],
                             (int)fs_values[fv], (int)fs_gcss[fg],
                             *(unsigned *)&acro_rolls[ar],
                             *(unsigned *)&(const float &)copter.g.acro_balance_pitch,
                             *(unsigned *)&(const float &)copter.attitude_control->get_angle_roll_p().kP(),
                             *(unsigned *)&(const float &)copter.attitude_control->get_angle_pitch_p().kP(),
                             *(unsigned *)&spd_ups[su],
                             (int)(ad != 0),
                             ok ? 1 : 0, g_calls,
                             g_calls ? g_first : "-",
                             g_calls ? g_last : "-");
                  }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/param_parity/vehicle")
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
