"""Record Mode's roll and pitch stick conversions from the real firmware.

Both read the AHRS: the velocity conversion rotates by its yaw, and the lean
angle conversion does not but shares the failsafe guard. The yaw is set
directly on `_cos_yaw` and `_sin_yaw` rather than through an attitude update,
because those are the members `body_to_earth2D` reads and driving them
directly is the only way to sweep the rotation without running an estimator.

The values swept are consistent pairs from a real angle, so the rotation stays
a rotation; a harness that set them independently would be recording a shear.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/plane-fw-rust")
OUT = ROOT / "fixtures/stick_nav.csv"
BUILD = Path("/tmp/stick_nav_parity/harness")

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

int main()
{
    AP::scheduler().init(nullptr, 0, 0);
    copter.allocate_motors();

    // Disarming saves the learned hover throttle, and that save waits on a
    // clock a harness never advances. See tools/parity/gen_alt_hold.py.
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
    copter.channel_roll->set_angle(4500);
    copter.channel_pitch->set_angle(4500);

    // Both conversions return neutral until the radio has been seen once.
    rc()._has_ever_seen_rc_input = true;

    Mode *mode = &copter.mode_stabilize;

    // radio_in values: the extremes, inside and outside the dead zone, and
    // centre. norm_input_dz is what both conversions read.
    const int16_t sticks[] = {1000, 1300, 1490, 1500, 1515, 1700, 2000};

    // ---- lean angles ----
    printf("#lean\n");
    printf("idx,roll_norm,pitch_norm,angle_max,angle_limit,roll_out,pitch_out\n");
    {
        // A conventional 30 degrees, a shallow limit and a limit above the
        // maximum, so the branch that ignores the limit is reached too.
        const float maxes[] = {0.3f, 0.5236f, 1.0472f};
        const float limits[] = {0.1745f, 0.5236f, 1.5708f};
        int idx = 0;
        for (unsigned a = 0; a < 7; a++)
          for (unsigned b = 0; b < 7; b++)
            for (unsigned c = 0; c < 3; c++)
              for (unsigned d = 0; d < 3; d++) {
                  copter.channel_roll->set_radio_in(sticks[a]);
                  copter.channel_pitch->set_radio_in(sticks[b]);

                  float roll_out = 0.0f, pitch_out = 0.0f;
                  mode->get_pilot_desired_lean_angles_rad(
                      roll_out, pitch_out, maxes[c], limits[d]);

                  printf("%d,%u,%u,%u,%u,%u,%u\n", idx++,
                         fbits(copter.channel_roll->norm_input_dz()),
                         fbits(copter.channel_pitch->norm_input_dz()),
                         fbits(maxes[c]), fbits(limits[d]),
                         fbits(roll_out), fbits(pitch_out));
              }
    }

    // ---- earth-frame velocity ----
    printf("#velocity\n");
    printf("idx,roll_norm,pitch_norm,vel_max,cos_yaw,sin_yaw,vel_n,vel_e\n");
    {
        // Consistent cos/sin pairs from real headings, including the axes and
        // the diagonals where the square-to-circle scaling does the most work.
        const float yaws[] = {0.0f, 0.7853982f, 1.5707963f, 2.3561945f,
                              3.1415927f, -0.7853982f, -2.0f};
        const float vel_maxes[] = {1.0f, 5.0f, 12.5f};
        int idx = 0;
        for (unsigned a = 0; a < 7; a++)
          for (unsigned b = 0; b < 7; b++)
            for (unsigned y = 0; y < 7; y++)
              for (unsigned v = 0; v < 3; v++) {
                  copter.channel_roll->set_radio_in(sticks[a]);
                  copter.channel_pitch->set_radio_in(sticks[b]);
                  copter.ahrs._cos_yaw = cosf(yaws[y]);
                  copter.ahrs._sin_yaw = sinf(yaws[y]);

                  const Vector2f vel = mode->get_pilot_desired_velocity(
                      vel_maxes[v]);

                  printf("%d,%u,%u,%u,%u,%u,%u,%u\n", idx++,
                         fbits(copter.channel_roll->norm_input_dz()),
                         fbits(copter.channel_pitch->norm_input_dz()),
                         fbits(vel_maxes[v]),
                         fbits(copter.ahrs._cos_yaw),
                         fbits(copter.ahrs._sin_yaw),
                         fbits(vel.x), fbits(vel.y));
              }
    }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/stick_nav_parity/vehicle")
    build(HARNESS, objects, BUILD, "ArduCopter/Copter.cpp",
          link_flags=vehicle_link.LINK_FLAGS)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
