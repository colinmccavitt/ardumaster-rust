"""Record the final arm checks.

The property worth recording is the ORDER, specifically that three checks sit
above the skip-all shortcut. An operator disabling ARMING_CHECK still cannot
arm a vehicle whose estimator is unhealthy or whose mode refuses -- and if the
shortcut ever moved to the top of the function nothing about the return value
would change for a healthy vehicle. So the sweep sets skip-all against each of
those three failing, and the message says which rung answered.

The AHRS health and the yaw source come from outside the flight code, so both
are wrapped. The mode's own allows_arming is left real: which modes refuse
which methods is part of what is being recorded.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/plane-fw-rust")
OUT = ROOT / "fixtures/copter_arm_checks.csv"
BUILD = Path("/tmp/armchk_parity/harness")

WRAPS = [
    "_ZNK9AP_Arming12check_failedEbPKcz",
    "_ZNK9AP_Arming12check_failedENS_5CheckEbPKcz",
    "_ZN3GCS10send_textvE12MAV_SEVERITYPKcP13__va_list_tag",
    "_ZN3GCS9send_textE12MAV_SEVERITYPKcz",
    "_ZNK7AP_AHRS7healthyEv",
    "_ZNK7AP_AHRS24using_noncompass_for_yawEv",
]

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

static char g_first[192];
static int g_calls = 0;
static bool g_ahrs_healthy = true;
static bool g_noncompass_yaw = true;

static void record(const char *fmt, va_list ap)
{
    char buf[192];
    vsnprintf(buf, sizeof(buf), fmt, ap);
    if (g_calls == 0) {
        const char *pre = strstr(buf, "PreArm: ");
        if (pre != NULL) {
            snprintf(g_first, sizeof(g_first), "%s", pre + 8);
        } else {
            const char *arm = strstr(buf, "Arm: ");
            snprintf(g_first, sizeof(g_first), "%s", arm ? arm + 5 : buf);
        }
    }
    g_calls++;
}

extern "C" void __wrap__ZNK9AP_Arming12check_failedEbPKcz(
    const void *self, bool report, const char *fmt, ...);
extern "C" void __wrap__ZNK9AP_Arming12check_failedEbPKcz(
    const void *self, bool report, const char *fmt, ...)
{ (void)self; (void)report; va_list ap; va_start(ap, fmt); record(fmt, ap); va_end(ap); }

extern "C" void __wrap__ZNK9AP_Arming12check_failedENS_5CheckEbPKcz(
    const void *self, int check, bool report, const char *fmt, ...);
extern "C" void __wrap__ZNK9AP_Arming12check_failedENS_5CheckEbPKcz(
    const void *self, int check, bool report, const char *fmt, ...)
{ (void)self; (void)check; (void)report; va_list ap; va_start(ap, fmt); record(fmt, ap); va_end(ap); }

extern "C" void __wrap__ZN3GCS10send_textvE12MAV_SEVERITYPKcP13__va_list_tag(
    void *self, int severity, const char *fmt, va_list ap);
extern "C" void __wrap__ZN3GCS10send_textvE12MAV_SEVERITYPKcP13__va_list_tag(
    void *self, int severity, const char *fmt, va_list ap)
{ (void)self; (void)severity; record(fmt, ap); }

extern "C" void __wrap__ZN3GCS9send_textE12MAV_SEVERITYPKcz(
    void *self, int severity, const char *fmt, ...);
extern "C" void __wrap__ZN3GCS9send_textE12MAV_SEVERITYPKcz(
    void *self, int severity, const char *fmt, ...)
{ (void)self; (void)severity; va_list ap; va_start(ap, fmt); record(fmt, ap); va_end(ap); }

extern "C" bool __wrap__ZNK7AP_AHRS7healthyEv(const void *self);
extern "C" bool __wrap__ZNK7AP_AHRS7healthyEv(const void *self)
{ (void)self; return g_ahrs_healthy; }

extern "C" bool __wrap__ZNK7AP_AHRS24using_noncompass_for_yawEv(const void *self);
extern "C" bool __wrap__ZNK7AP_AHRS24using_noncompass_for_yawEv(const void *self)
{ (void)self; return g_noncompass_yaw; }

int main()
{
    AP::scheduler().init(nullptr, 0, 0);
    copter.allocate_motors();
    copter.motors->_throttle_hover_learn.set(0);
    setvbuf(stdout, NULL, _IOLBF, 0);

    copter.channel_throttle = rc().channel(2);
    copter.channel_throttle->set_range(1000);
    AP_Arming_Copter &arming = copter.arming;

    printf("#arm_checks\n");
    printf("idx,mode,method,ahrs_healthy,skip_all,ins_enabled,rc_enabled,"
           "adsb,throttle_in,mode_allows_arming,passed,first\n");

    Mode *modes[] = {
        (Mode *)&copter.mode_stabilize,
        (Mode *)&copter.mode_loiter,
        (Mode *)&copter.mode_land,
    };
    const AP_Arming::Method methods[] = {
        AP_Arming::Method::RUDDER,
        AP_Arming::Method::MAVLINK,
        AP_Arming::Method::SCRIPTING,
    };
    const int16_t throttles[] = {0, 400};

    int idx = 0;
    for (unsigned m = 0; m < 3; m++)
      for (unsigned me = 0; me < 3; me++)
        for (unsigned ah = 0; ah < 2; ah++)
          for (unsigned sk = 0; sk < 2; sk++)
            for (unsigned ad = 0; ad < 2; ad++)
              for (unsigned th = 0; th < 2; th++) {
                  copter.flightmode = modes[m];
                  g_ahrs_healthy = (ah != 0);
                  // Skip the compass rung: it needs a real compass driver.
                  g_noncompass_yaw = true;
                  // checks_to_skip is a SKIP mask.
                  arming.checks_to_skip.set(sk ? 0x7FFFFFFF : 0);
                  copter.failsafe.adsb = (ad != 0);
                  copter.channel_throttle->set_control_in(throttles[th]);
                  copter.motors->armed(false);

                  g_first[0] = '\0';
                  g_calls = 0;
                  const bool ok = arming.arm_checks(methods[me]);

                  printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s\n", idx++,
                         (int)modes[m]->mode_number(),
                         (int)methods[me],
                         (int)(ah != 0),
                         (int)arming.should_skip_all_checks(),
                         (int)arming.check_enabled(AP_Arming::Check::INS),
                         (int)arming.check_enabled(AP_Arming::Check::RC),
                         (int)(ad != 0),
                         (int)throttles[th],
                         (int)modes[m]->allows_arming(methods[me]),
                         ok ? 1 : 0,
                         g_calls ? g_first : "-");
              }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/armchk_parity/vehicle")
    flags = list(vehicle_link.LINK_FLAGS) + [
        "-Wl,--wrap=" + sym for sym in WRAPS
    ]
    build(HARNESS, objects, BUILD, "ArduCopter/Copter.cpp", link_flags=flags)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
