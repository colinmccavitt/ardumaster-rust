"""Record Copter::set_mode's veto ladder from the real firmware.

# Why the message and not just the return value

Every veto returns the same false. A recording of return values alone would be
satisfied by any permutation of the ladder, which is precisely the part that
matters: each rung sends the pilot a different explanation, and the order
decides which one arrives. So the explanation is recorded.

Getting at it took two attempts. `Copter::mode_change_failed` is defined in
mode.cpp and called from mode.cpp, so the compiler binds those calls directly
and --wrap never sees them -- 1536 rows of empty reasons is what that looks
like. `GCS::send_text` is an undefined reference in mode.cpp.o and is
wrappable, and `AP_Vehicle::notify_no_such_mode` covers the one rung that
does not go through mode_change_failed. Between them every rung is
distinguished, and the strings are upstream's own, so a reordering shows up as
a changed fixture rather than as nothing at all.

# Inputs that cannot be brought up

`position_ok` and `ekf_alt_ok` consult the EKF, which needs a running
estimator with an origin. Both are wrapped and answered by the sweep. The
ladder itself is the firmware's, unmodified.

The fence rungs are not swept: reaching them needs a breach recovery in
progress, which needs a configured fence and a real breach. Rows are recorded
with the fence quiet, so the fixture pins every other rung and the fence rung
is covered by reasoning in the test rather than by recording. That gap is
stated rather than hidden.

# A mode change that succeeds really happens

The vehicle is put back into a known mode before each row, because a
successful row leaves it somewhere else and the next row's "already in this
mode" answer would otherwise depend on the previous row.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/plane-fw-rust")
OUT = ROOT / "fixtures/mode_entry.csv"
BUILD = Path("/tmp/mode_entry_parity/harness")

SEND_TEXT = "_ZN3GCS9send_textE12MAV_SEVERITYPKcz"
NO_SUCH_MODE = "_ZN10AP_Vehicle19notify_no_such_modeEh"
POSITION_OK = "_ZNK6Copter11position_okEv"
EKF_ALT_OK = "_ZNK6Copter10ekf_alt_okEv"

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
#include <math.h>
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

// ---- the observation ----
//
// Copter::mode_change_failed is defined in mode.cpp and called from mode.cpp,
// so the compiler binds those calls directly and --wrap cannot see them. The
// text it produces does cross the boundary, though: mode.cpp.o carries an
// undefined reference to GCS::send_text. That is what is intercepted.
static char g_reason[160];
static int g_failed_calls = 0;

extern "C" void __wrap__ZN3GCS9send_textE12MAV_SEVERITYPKcz(
    void *self, int severity, const char *fmt, ...);
extern "C" void __wrap__ZN3GCS9send_textE12MAV_SEVERITYPKcz(
    void *self, int severity, const char *fmt, ...)
{
    (void)self; (void)severity;
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    // Only the mode-change refusal is of interest; a mode's init may send
    // text of its own and that is not a veto.
    const char *marker = strstr(buf, " failed: ");
    if (strncmp(buf, "Mode change to ", 15) == 0 && marker != NULL) {
        snprintf(g_reason, sizeof(g_reason), "%s", marker + 9);
        g_failed_calls++;
    }
    // Not forwarded: there is no GCS backend in a harness, and nothing in the
    // ladder reads the effects of sending.
}

extern "C" void __wrap__ZN10AP_Vehicle19notify_no_such_modeEh(
    void *self, unsigned char mode_number);
extern "C" void __wrap__ZN10AP_Vehicle19notify_no_such_modeEh(
    void *self, unsigned char mode_number)
{
    (void)self; (void)mode_number;
    snprintf(g_reason, sizeof(g_reason), "no such mode");
    g_failed_calls++;
}

// ---- injected inputs ----
static bool g_position_ok = true;
static bool g_ekf_alt_ok = true;

extern "C" bool __wrap__ZNK6Copter11position_okEv(const void *self);
extern "C" bool __wrap__ZNK6Copter11position_okEv(const void *self)
{
    (void)self;
    return g_position_ok;
}

extern "C" bool __wrap__ZNK6Copter10ekf_alt_okEv(const void *self);
extern "C" bool __wrap__ZNK6Copter10ekf_alt_okEv(const void *self)
{
    (void)self;
    return g_ekf_alt_ok;
}

struct Candidate {
    Mode::Number number;
    const char *label;
};

int main()
{
    AP::scheduler().init(nullptr, 0, 0);
    copter.allocate_motors();
    copter.motors->_throttle_hover_learn.set(0);

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
    copter.channel_throttle->set_range(1000);
    rc()._has_ever_seen_rc_input = true;

    // Modes reachable without a mission, a fence or a companion computer,
    // plus one number that no mode carries.
    const Candidate candidates[] = {
        { Mode::Number::STABILIZE,  "STABILIZE"  },
        { Mode::Number::ALT_HOLD,   "ALT_HOLD"   },
        { Mode::Number::LOITER,     "LOITER"     },
        { Mode::Number::LAND,       "LAND"       },
        { Mode::Number::ACRO,       "ACRO"       },
        { (Mode::Number)77,         "NONEXISTENT"},
    };

    const ModeReason reasons[] = {
        ModeReason::RC_COMMAND,
        ModeReason::GCS_COMMAND,
    };

    printf("#entry\n");
    printf("idx,mode,target_is_current,reason_is_gcs,gcs_enabled,mode_exists,"
           "armed,land_complete,new_manual_thr,new_is_drift,cur_manual_thr,"
           "pilot_thr,non_takeoff_thr,new_requires_pos,position_ok,"
           "ekf_alt_ok,in_rc_failsafe,new_allows_rc_fs,result,reason\n");

    int idx = 0;
    for (unsigned c = 0; c < 6; c++)
      for (unsigned r = 0; r < 2; r++)
        for (unsigned armed = 0; armed < 2; armed++)
          for (unsigned lc = 0; lc < 2; lc++)
            for (unsigned thr = 0; thr < 2; thr++)
              for (unsigned pk = 0; pk < 2; pk++)
                for (unsigned ek = 0; ek < 2; ek++)
                  for (unsigned fs = 0; fs < 2; fs++)
                    for (unsigned start = 0; start < 2; start++) {
                        // Put the vehicle in a known mode first. Two starting
                        // points: a manual-throttle mode and one that is not,
                        // because two rungs read the *current* mode.
                        copter.flightmode = start ? (Mode *)&copter.mode_stabilize
                                                  : (Mode *)&copter.mode_loiter;

                        copter.motors->armed(armed != 0);
                        copter.ap.land_complete = (lc != 0);
                        g_position_ok = (pk != 0);
                        g_ekf_alt_ok = (ek != 0);
                        copter.failsafe.radio = (fs != 0);

                        // A throttle stick at the bottom, and one well above
                        // the no-takeoff threshold.
                        copter.channel_throttle->set_control_in(thr ? 900 : 0);

                        // FLTMODE_GCSBLOCK: block LOITER on odd rows so the
                        // GCS rung is reachable, leave the rest open.
                        copter.flight_mode_GCS_block.set(
                            (r == 1) ? (1 << 5) : 0);

                        Mode *before = copter.flightmode;
                        const bool cur_manual = before->has_manual_throttle();

                        Mode *target = copter.mode_from_mode_num(candidates[c].number);
                        const bool exists = (target != nullptr);
                        const bool target_is_current = exists && (target == before);

                        float pilot_thr = 0.0f;
                        bool new_manual = false, new_requires_pos = false;
                        bool new_allows_rc_fs = true;
                        if (exists) {
                            pilot_thr = target->get_pilot_desired_throttle();
                            new_manual = target->has_manual_throttle();
                            new_requires_pos = target->requires_position();
                            new_allows_rc_fs = target->allows_entry_in_rc_failsafe();
                        }

                        g_reason[0] = '\0';
                        g_failed_calls = 0;
                        const bool ok = copter.set_mode(candidates[c].number,
                                                        reasons[r]);

                        // 0 refused, 1 entered, 2 already in the mode.
                        int result;
                        if (!ok) {
                            result = 0;
                        } else if (target_is_current) {
                            result = 2;
                        } else {
                            result = 1;
                        }

                        printf("%d,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%d,%d,%d,%d,%d,%d,%s\n",
                               idx++, candidates[c].label,
                               (int)target_is_current,
                               (int)(reasons[r] == ModeReason::GCS_COMMAND),
                               (int)copter.gcs_mode_enabled(candidates[c].number),
                               (int)exists,
                               (int)(armed != 0),
                               (int)(lc != 0),
                               (int)new_manual,
                               0,
                               (int)cur_manual,
                               fbits(pilot_thr),
                               fbits(copter.get_non_takeoff_throttle()),
                               (int)new_requires_pos,
                               (int)(pk != 0),
                               (int)(ek != 0),
                               (int)(fs != 0),
                               (int)new_allows_rc_fs,
                               result,
                               g_failed_calls ? g_reason : "-");
                    }

    // ---- the throttle rung's boundary ----
    //
    // Found by bisection: pilot(hover) - hover/2 is continuous in the hover
    // throttle, so where it changes sign there is a crossing. Bisecting to one
    // representable step gives the two adjacent hover values that straddle it,
    // which is as close to the comparison as floats allow.
    printf("#boundary\n");
    printf("idx,mode,target_is_current,reason_is_gcs,gcs_enabled,mode_exists,"
           "armed,land_complete,new_manual_thr,new_is_drift,cur_manual_thr,"
           "pilot_thr,non_takeoff_thr,new_requires_pos,position_ok,"
           "ekf_alt_ok,in_rc_failsafe,new_allows_rc_fs,result,reason\n");
    {
        Mode *target = (Mode *)&copter.mode_stabilize;

        const int16_t controls[] = {150, 200, 250, 275, 300, 325,
                                    350, 400, 450, 500};
        int bidx = 0;

        for (unsigned a = 0; a < 10; a++) {
            copter.channel_throttle->set_control_in(controls[a]);

            // g(hover) = pilot(hover) - hover/2, at the ends of the clamp.
            float lo = 0.125f, hi = 0.6875f;
            copter.motors->_throttle_hover.set(lo);
            const float g_lo = target->get_pilot_desired_throttle()
                               - copter.get_non_takeoff_throttle();
            copter.motors->_throttle_hover.set(hi);
            const float g_hi = target->get_pilot_desired_throttle()
                               - copter.get_non_takeoff_throttle();

            if ((g_lo < 0.0f) == (g_hi < 0.0f)) {
                continue;   // no crossing for this stick position
            }

            // Bisect to one representable step.
            for (int it = 0; it < 200; it++) {
                const float mid = lo + (hi - lo) * 0.5f;
                if (fbits(mid) == fbits(lo) || fbits(mid) == fbits(hi)) {
                    break;
                }
                copter.motors->_throttle_hover.set(mid);
                const float g = target->get_pilot_desired_throttle()
                                - copter.get_non_takeoff_throttle();
                if ((g < 0.0f) == (g_lo < 0.0f)) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }

            // Record both sides of the crossing, and one step beyond each, so
            // the comparison is pinned from both directions.
            const float hovers[] = {
                nextafterf(lo, 0.0f), lo, hi, nextafterf(hi, 1.0f),
            };

            for (unsigned k = 0; k < 4; k++) {
                if (hovers[k] < 0.125f || hovers[k] > 0.6875f) {
                    continue;
                }
                copter.flightmode = (Mode *)&copter.mode_loiter;
                copter.motors->armed(true);
                copter.motors->set_interlock(true);
                copter.ap.land_complete = true;
                copter.failsafe.radio = false;
                g_position_ok = true;
                g_ekf_alt_ok = true;
                copter.flight_mode_GCS_block.set(0);
                copter.motors->_throttle_hover.set(hovers[k]);

                const float pilot = target->get_pilot_desired_throttle();
                const bool cur_manual = copter.flightmode->has_manual_throttle();

                g_reason[0] = '\0';
                g_failed_calls = 0;
                const bool ok = copter.set_mode(Mode::Number::STABILIZE,
                                                ModeReason::RC_COMMAND);

                printf("%d,STABILIZE,0,0,1,1,1,1,%d,0,%d,%u,%u,%d,1,1,0,1,%d,%s\n",
                       bidx++,
                       (int)target->has_manual_throttle(),
                       (int)cur_manual,
                       fbits(pilot),
                       fbits(copter.get_non_takeoff_throttle()),
                       (int)target->requires_position(),
                       ok ? 1 : 0,
                       g_failed_calls ? g_reason : "-");
            }
        }
    }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/mode_entry_parity/vehicle")
    flags = list(vehicle_link.LINK_FLAGS) + [
        "-Wl,--wrap=" + SEND_TEXT,
        "-Wl,--wrap=" + NO_SUCH_MODE,
        "-Wl,--wrap=" + POSITION_OK,
        "-Wl,--wrap=" + EKF_ALT_OK,
    ]
    build(HARNESS, objects, BUILD, "ArduCopter/Copter.cpp", link_flags=flags)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
