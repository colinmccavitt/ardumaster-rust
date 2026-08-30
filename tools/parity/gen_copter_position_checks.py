"""Record mandatory_position_checks and the GPS HDOP check.

The message is the observation again, because every rung returns the same
false. check_failed is called from AP_Arming_Copter.cpp here -- unlike the RC
calibration check, which delegates into AP_Arming.cpp -- so wrapping
check_failed is the right symbol for these. Both are wrapped anyway, along
with GCS::send_textv, so a message raised from either translation unit is
captured; getting this wrong is what cost the previous slice a cycle.

position_ok, the AHRS pre-arm check, the filter status and the variances all
come from the EKF, so each is wrapped and answered by the sweep. What is
recorded is the ladder: which rung fires first for a given combination.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/ardumaster-rust")
OUT = ROOT / "fixtures/copter_position_checks.csv"
BUILD = Path("/tmp/pos_parity/harness")

WRAPS = [
    "_ZNK9AP_Arming12check_failedEbPKcz",
    "_ZNK9AP_Arming12check_failedENS_5CheckEbPKcz",
    "_ZN3GCS10send_textvE12MAV_SEVERITYPKcP13__va_list_tag",
    "_ZN3GCS9send_textE12MAV_SEVERITYPKcz",
    "_ZNK6Copter11position_okEv",
    # Without these the AHRS rung refuses first every time and nothing
    # below it is reachable.
    "_ZNK7AP_AHRS13pre_arm_checkEbPch",
    "_ZNK7AP_AHRS17get_filter_statusER17nav_filter_status",
    "_ZNK7AP_AHRS13get_variancesERfS0_S0_R7Vector3IfES0_",
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
static bool g_position_ok = true;
static bool g_ahrs_pre_arm_ok = true;
static bool g_filter_status_ok = true;
static bool g_gps_glitching = false;
static float g_compass_var = 0.0f;
static float g_position_var = 0.0f;
static float g_velocity_var = 0.0f;
static float g_height_var = 0.0f;

static void record(const char *fmt, va_list ap)
{
    char buf[192];
    vsnprintf(buf, sizeof(buf), fmt, ap);
    if (g_calls == 0) {
        const char *tail = strstr(buf, "PreArm: ");
        snprintf(g_first, sizeof(g_first), "%s", tail ? tail + 8 : buf);
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

extern "C" bool __wrap__ZNK6Copter11position_okEv(const void *self);
extern "C" bool __wrap__ZNK6Copter11position_okEv(const void *self)
{ (void)self; return g_position_ok; }

extern "C" bool __wrap__ZNK7AP_AHRS13pre_arm_checkEbPch(
    const void *self, bool requires_position, char *failure_msg, unsigned char len);
extern "C" bool __wrap__ZNK7AP_AHRS13pre_arm_checkEbPch(
    const void *self, bool requires_position, char *failure_msg, unsigned char len)
{
    (void)self; (void)requires_position;
    if (!g_ahrs_pre_arm_ok) {
        snprintf(failure_msg, len, "EKF3 not started");
        return false;
    }
    return true;
}

extern "C" bool __wrap__ZNK7AP_AHRS17get_filter_statusER17nav_filter_status(
    const void *self, nav_filter_status &status);
extern "C" bool __wrap__ZNK7AP_AHRS17get_filter_statusER17nav_filter_status(
    const void *self, nav_filter_status &status)
{
    (void)self;
    status.value = 0;
    status.flags.gps_glitching = g_gps_glitching;
    return g_filter_status_ok;
}

extern "C" void __wrap__ZNK7AP_AHRS13get_variancesERfS0_S0_R7Vector3IfES0_(
    const void *self, float &vel, float &pos, float &hgt, Vector3f &mag, float &tas);
extern "C" void __wrap__ZNK7AP_AHRS13get_variancesERfS0_S0_R7Vector3IfES0_(
    const void *self, float &vel, float &pos, float &hgt, Vector3f &mag, float &tas)
{
    (void)self;
    vel = g_velocity_var;
    pos = g_position_var;
    hgt = g_height_var;
    // The compass variance is compared as a vector length.
    mag.x = g_compass_var;
    mag.y = 0.0f;
    mag.z = 0.0f;
    tas = 0.0f;
}

int main()
{
    AP::scheduler().init(nullptr, 0, 0);
    copter.allocate_motors();
    copter.motors->_throttle_hover_learn.set(0);
    setvbuf(stdout, NULL, _IOLBF, 0);

    AP_Arming_Copter &arming = copter.arming;

    printf("#position\n");
    printf("idx,mode,requires_position,require_location,position_ok,"
           "ahrs_ok,glitching,thresh_tenths,var_index,passed,calls,first\n");
    {
        // A mode that needs position and one that does not.
        Mode *modes[] = {
            (Mode *)&copter.mode_stabilize,
            (Mode *)&copter.mode_loiter,
        };
        const float threshes[] = {0.0f, 0.8f};
        int idx = 0;
        for (unsigned m = 0; m < 2; m++)
          for (unsigned pk = 0; pk < 2; pk++)
            for (unsigned rl = 0; rl < 2; rl++)
              for (unsigned th = 0; th < 2; th++)
                for (unsigned ah = 0; ah < 2; ah++)
                  for (unsigned gl = 0; gl < 2; gl++)
                    for (unsigned vi = 0; vi < 5; vi++) {
                  copter.flightmode = modes[m];
                  g_position_ok = (pk != 0);
                  g_ahrs_pre_arm_ok = (ah != 0);
                  g_gps_glitching = (gl != 0);
                  g_filter_status_ok = true;

                  // vi 0 leaves every variance clear; 1..4 pushes one of
                  // them onto the threshold, in upstream's report order, so
                  // which one is named can be checked.
                  g_compass_var = g_position_var = 0.0f;
                  g_velocity_var = g_height_var = 0.0f;
                  const float over = 0.9f;
                  if (vi == 1) g_compass_var = over;
                  if (vi == 2) g_position_var = over;
                  if (vi == 3) g_velocity_var = over;
                  if (vi == 4) g_height_var = over;

                  arming.require_location.set(rl
                      ? AP_Arming::RequireLocation::YES
                      : AP_Arming::RequireLocation::NO);
                  copter.g.fs_ekf_thresh.set(threshes[th]);

                  g_first[0] = '\0';
                  g_calls = 0;
                  const bool ok = arming.mandatory_position_checks(true);

                  printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s\n", idx++,
                         (int)modes[m]->mode_number(),
                         (int)modes[m]->requires_position(),
                         (int)(rl != 0), (int)(pk != 0),
                         (int)(ah != 0), (int)(gl != 0),
                         (int)(threshes[th] * 10.0f), (int)vi,
                         ok ? 1 : 0, g_calls,
                         g_calls ? g_first : "-");
              }
    }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/pos_parity/vehicle")
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
