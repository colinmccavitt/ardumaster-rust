"""Record the ground-handling and mode-exit decisions from the firmware.

Five small decisions attached to large side effects. Each is observed where
it leaves the function rather than inferred from what a controller looked
like afterwards:

  * the spool command lands in the motors' `_spool_desired`, after
    AP_MotorsMulticopter's own safety constraint -- so the Rust side composes
    both layers, exactly as with the altitude-hold machine.
  * `reset_yaw_target_and_rate` and `set_accel_throttle_I_from_pilot_throttle`
    are undefined references in mode.cpp.o, so both are wrapped and counted.
    Checked with `nm -u` first: a wrap on a symbol the linker does not
    resolve fires silently never, which cost a whole recording cycle on
    gen_mode_entry.py.
  * the EKF reset method is a plain member the setter assigns, so it is read
    back directly.

`make_safe_ground_handling` and `zero_throttle_and_relax_ac` also drive the
position and attitude controllers. Those calls are left to run; nothing here
reads their effects, and letting them run keeps the vehicle in the state the
next row would find it in anyway.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/ardumaster-rust")
OUT = ROOT / "fixtures/ground_handling.csv"
BUILD = Path("/tmp/ground_parity/harness")

RESET_YAW = "_ZN18AC_AttitudeControl25reset_yaw_target_and_rateEb"
SEED_THROTTLE = "_ZN6Copter40set_accel_throttle_I_from_pilot_throttleEv"

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

// ---- observations ----
static int g_reset_yaw_calls = 0;
static int g_seed_throttle_calls = 0;

extern "C" void __real__ZN18AC_AttitudeControl25reset_yaw_target_and_rateEb(
    void *self, bool reset_rate);

extern "C" void __wrap__ZN18AC_AttitudeControl25reset_yaw_target_and_rateEb(
    void *self, bool reset_rate);
extern "C" void __wrap__ZN18AC_AttitudeControl25reset_yaw_target_and_rateEb(
    void *self, bool reset_rate)
{
    g_reset_yaw_calls++;
    __real__ZN18AC_AttitudeControl25reset_yaw_target_and_rateEb(self, reset_rate);
}

extern "C" void __wrap__ZN6Copter40set_accel_throttle_I_from_pilot_throttleEv(
    void *self);
extern "C" void __wrap__ZN6Copter40set_accel_throttle_I_from_pilot_throttleEv(
    void *self)
{
    (void)self;
    g_seed_throttle_calls++;
    // Not forwarded: it writes into the position controller's integrator, and
    // whether it was called is the whole observation.
}

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

    Mode *mode = &copter.mode_stabilize;

    const AP_Motors::SpoolState spools[] = {
        AP_Motors::SpoolState::SHUT_DOWN,
        AP_Motors::SpoolState::GROUND_IDLE,
        AP_Motors::SpoolState::SPOOLING_UP,
        AP_Motors::SpoolState::THROTTLE_UNLIMITED,
        AP_Motors::SpoolState::SPOOLING_DOWN,
    };

    // ---- is_disarmed_or_landed ----
    printf("#disarmed_or_landed\n");
    printf("idx,armed,auto_armed,land_complete,out\n");
    {
        int idx = 0;
        for (unsigned a = 0; a < 2; a++)
          for (unsigned b = 0; b < 2; b++)
            for (unsigned c = 0; c < 2; c++) {
                copter.motors->armed(a != 0);
                copter.ap.auto_armed = (b != 0);
                copter.ap.land_complete = (c != 0);
                printf("%d,%d,%d,%d,%d\n", idx++, a, b, c,
                       (int)mode->is_disarmed_or_landed());
            }
    }

    // ---- make_safe_ground_handling ----
    printf("#ground_handling\n");
    printf("idx,force_unlimited,spool,armed,interlock,desired,reset_yaw\n");
    {
        int idx = 0;
        for (unsigned f = 0; f < 2; f++)
          for (unsigned s = 0; s < 5; s++)
            for (unsigned ar = 0; ar < 2; ar++)
              for (unsigned il = 0; il < 2; il++) {
                  copter.motors->armed(ar != 0);
                  copter.motors->set_interlock(il != 0);
                  copter.motors->_spool_state = spools[s];
                  copter.motors->_spool_desired =
                      (AP_Motors::DesiredSpoolState)7;

                  g_reset_yaw_calls = 0;
                  mode->make_safe_ground_handling(f != 0);

                  printf("%d,%d,%d,%d,%d,%d,%d\n", idx++,
                         f, (int)spools[s], ar, il,
                         (int)copter.motors->_spool_desired,
                         g_reset_yaw_calls > 0 ? 1 : 0);
              }
    }

    // ---- zero_throttle_and_relax_ac ----
    printf("#zero_throttle\n");
    printf("idx,spool_up,armed,interlock,desired\n");
    {
        int idx = 0;
        for (unsigned u = 0; u < 2; u++)
          for (unsigned ar = 0; ar < 2; ar++)
            for (unsigned il = 0; il < 2; il++) {
                copter.motors->armed(ar != 0);
                copter.motors->set_interlock(il != 0);
                copter.motors->_spool_desired =
                    (AP_Motors::DesiredSpoolState)7;

                mode->zero_throttle_and_relax_ac(u != 0);

                printf("%d,%d,%d,%d,%d\n", idx++, u, ar, il,
                       (int)copter.motors->_spool_desired);
            }
    }

    // ---- exit_mode's throttle handover ----
    printf("#exit_mode\n");
    printf("idx,old_manual,new_manual,armed,land_complete,seeded\n");
    {
        Mode *manual = (Mode *)&copter.mode_stabilize;
        Mode *automatic = (Mode *)&copter.mode_loiter;
        int idx = 0;
        for (unsigned o = 0; o < 2; o++)
          for (unsigned n = 0; n < 2; n++)
            for (unsigned ar = 0; ar < 2; ar++)
              for (unsigned lc = 0; lc < 2; lc++) {
                  Mode *old_mode = o ? manual : automatic;
                  Mode *new_mode = n ? manual : automatic;
                  copter.motors->armed(ar != 0);
                  copter.ap.land_complete = (lc != 0);

                  g_seed_throttle_calls = 0;
                  copter.exit_mode(old_mode, new_mode);

                  printf("%d,%d,%d,%d,%d,%d\n", idx++,
                         (int)old_mode->has_manual_throttle(),
                         (int)new_mode->has_manual_throttle(),
                         ar, lc,
                         g_seed_throttle_calls > 0 ? 1 : 0);
              }
    }

    // ---- update_flight_mode's EKF reset method ----
    printf("#ekf_reset\n");
    printf("idx,move_vehicle,method\n");
    {
        Mode *modes[] = { (Mode *)&copter.mode_stabilize,
                          (Mode *)&copter.mode_loiter,
                          (Mode *)&copter.mode_land,
                          (Mode *)&copter.mode_acro };
        int idx = 0;
        for (unsigned m = 0; m < 4; m++) {
            copter.flightmode = modes[m];
            copter.pos_control->_ekf_reset_method =
                AC_PosControl::EKFResetMethod::MoveTarget;

            copter.update_flight_mode();

            printf("%d,%d,%d\n", idx++,
                   (int)modes[m]->move_vehicle_on_ekf_reset(),
                   (int)copter.pos_control->_ekf_reset_method);
        }
    }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/ground_parity/vehicle")
    flags = list(vehicle_link.LINK_FLAGS) + [
        "-Wl,--wrap=" + RESET_YAW,
        "-Wl,--wrap=" + SEED_THROTTLE,
    ]
    build(HARNESS, objects, BUILD, "ArduCopter/Copter.cpp", link_flags=flags)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
