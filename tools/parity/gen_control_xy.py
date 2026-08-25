"""Dump the 2-D shaping family from the real firmware.

Pure functions of their arguments, so this is a sweep rather than a sequence
for the limiters, and a stepped trajectory for the shapers, whose whole
purpose is to carry state forward.

The cornering limiter is the interesting one: it answers the same
budget question two different ways depending on whether the vehicle is
braking, so the sweep has to cross that boundary rather than sit on one side.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import vehicle_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/plane-fw-rust")
OUT = ROOT / "fixtures/control_xy.csv"
BUILD = Path("/tmp/controlxy_parity/harness")

HARNESS = r'''
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/control.h>
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
    // ---- limit_accel_xy and limit_accel_corner_xy ----
    //
    // Swept over the angle between velocity and acceleration, because that
    // angle is what both functions decide on: the cornering one switches
    // regime at 90 degrees, where the along-track component changes sign.
    printf("#limits\n");
    printf("idx,vel_x,vel_y,acc_x,acc_y,accel_max,"
           "plain_x,plain_y,plain_ret,corner_x,corner_y,corner_ret\n");
    {
        const float speeds[] = {0.0f, 0.5f, 4.0f, 20.0f};
        // The two middle magnitudes matter: with accel_max at 3 and 12 they
        // land between the limit and the limit divided by sin(angle), which
        // is the only window where the cross-track fits AND the along-track
        // clamp binds. Outside it either nothing is limited or the
        // cross-track alone consumes the whole budget, and the remaining
        // along-track allowance is never computed.
        const float mags[] = {0.5f, 2.5f, 5.0f, 9.0f, 20.0f, 40.0f};
        const float maxes[] = {0.0f, 3.0f, 12.0f};
        int idx = 0;
        for (unsigned s = 0; s < sizeof(speeds)/sizeof(speeds[0]); s++) {
            for (unsigned m = 0; m < sizeof(mags)/sizeof(mags[0]); m++) {
                for (unsigned x = 0; x < sizeof(maxes)/sizeof(maxes[0]); x++) {
                    for (int a = 0; a < 12; a++) {
                        const float ang = a * (2.0f * M_PI / 12.0f);
                        const Vector2f vel{speeds[s], 0.0f};
                        const Vector2f acc{mags[m] * cosf(ang), mags[m] * sinf(ang)};

                        Vector2f plain = acc;
                        const bool pr = limit_accel_xy(vel, plain, maxes[x]);
                        Vector2f corner = acc;
                        const bool cr = limit_accel_corner_xy(vel, corner, maxes[x]);

                        printf("%d,%u,%u,%u,%u,%u,%u,%u,%d,%u,%u,%d\n", idx++,
                               fbits(vel.x), fbits(vel.y),
                               fbits(acc.x), fbits(acc.y), fbits(maxes[x]),
                               fbits(plain.x), fbits(plain.y), pr ? 1 : 0,
                               fbits(corner.x), fbits(corner.y), cr ? 1 : 0);
                    }
                }
            }
        }
    }

    // ---- the shapers, stepped ----
    //
    // shape_accel_xy and shape_vel_accel_xy carry acceleration forward, so
    // what matters is the trajectory. The velocity demand turns a corner
    // partway, which is where the cornering limiter earns its keep.
    printf("#shape\n");
    printf("step,dt,accel_max,jerk_max,vel_des_x,vel_des_y,acc_ff_x,acc_ff_y,"
           "vel_x,vel_y,limit_total,sa_x,sa_y,sva_x,sva_y,3d_x,3d_y,3d_z,guard_x,guard_y\n");
    {
        const float dt = 0.0025f;
        const float accel_max = 6.0f;
        const float jerk_max = 30.0f;

        Vector2f accel_a{0.0f, 0.0f};
        Vector2f accel_b{0.0f, 0.0f};
        // The three-dimensional spelling shapes only the horizontal pair; its
        // z must come out untouched, which is the whole reason it exists.
        Vector3f accel_3d{0.0f, 0.0f, -7.5f};
        Vector2f accel_guard{1.25f, -0.75f};

        for (int i = 0; i < 600; i++) {
            const float ts = i * dt;

            // A demand that turns ninety degrees partway, so the shaped
            // acceleration has to swing through the cornering regimes.
            Vector2f vel_des;
            if (ts < 0.5f)      vel_des = Vector2f{8.0f, 0.0f};
            else if (ts < 1.0f) vel_des = Vector2f{0.0f, 8.0f};
            else                vel_des = Vector2f{-6.0f, -2.0f};

            // For the first two thirds the velocity lags badly, which
            // saturates the correction. For the last third it nearly matches
            // the demand, which puts the sqrt controller in its linear region
            // -- the only place the gain and the length normalisation decide
            // the answer rather than feeding a limiter that clips them to the
            // same value either way.
            Vector2f vel;
            if (ts < 1.0f) {
                vel = Vector2f{4.0f * cosf(1.5f * ts), 4.0f * sinf(1.5f * ts)};
            } else {
                const Vector2f nudge{0.02f * cosf(11.0f * ts), 0.02f * sinf(9.0f * ts)};
                vel = vel_des + nudge;
            }
            const Vector2f accel_ff{0.5f * sinf(3.0f * ts), -0.4f * cosf(2.0f * ts)};
            const bool limit_total = (i / 150) % 2 == 0;

            shape_accel_xy(vel_des, accel_a, jerk_max, dt);

            const Vector3f des_3d{vel_des.x, vel_des.y, 99.0f};
            shape_accel_xy(des_3d, accel_3d, jerk_max, dt);

            // The degenerate-argument guard is NOT recorded here: upstream
            // raises an internal error, which aborts this harness. That the
            // port returns quietly instead is a difference in reporting, not
            // in the value, and it is covered by a test against the port.
            shape_vel_accel_xy(vel_des, accel_ff, vel, accel_b,
                               accel_max, jerk_max, dt, limit_total);

            printf("%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", i,
                   fbits(dt), fbits(accel_max), fbits(jerk_max),
                   fbits(vel_des.x), fbits(vel_des.y),
                   fbits(accel_ff.x), fbits(accel_ff.y),
                   fbits(vel.x), fbits(vel.y), limit_total ? 1 : 0,
                   fbits(accel_a.x), fbits(accel_a.y),
                   fbits(accel_b.x), fbits(accel_b.y),
                   fbits(accel_3d.x), fbits(accel_3d.y), fbits(accel_3d.z),
                   fbits(accel_guard.x), fbits(accel_guard.y));
        }
    }

    // ---- update_vel_accel_xy and update_pos_vel_accel_xy ----
    //
    // The limit vector is the whole point: these suppress motion that would
    // worsen an error in a direction the vehicle cannot go. The sweep varies
    // the limit, the errors, and the sign of the current velocity, because
    // the suppression tests all three.
    printf("#update\n");
    printf("idx,dt,pos_x,pos_y,vel_x,vel_y,acc_x,acc_y,lim_x,lim_y,"
           "perr_x,perr_y,verr_x,verr_y,"
           "uva_vel_x,uva_vel_y,upva_pos_x,upva_pos_y,upva_vel_x,upva_vel_y\n");
    {
        const float dt = 0.01f;
        const Vector2f limits[] = {{0.0f, 0.0f}, {1.0f, 0.0f}, {-1.0f, 0.0f}, {0.6f, 0.8f}};
        const Vector2f errs[] = {{0.0f, 0.0f}, {2.0f, 0.0f}, {-2.0f, 0.0f}, {1.0f, 1.0f}};
        const float vels[] = {-3.0f, 0.0f, 3.0f};
        int idx = 0;
        for (unsigned l = 0; l < 4; l++) {
            for (unsigned pe = 0; pe < 4; pe++) {
                for (unsigned ve = 0; ve < 4; ve++) {
                    for (unsigned v = 0; v < 3; v++) {
                        const Vector2f accel{2.0f, -1.0f};
                        const Vector2f vel0{vels[v], 1.0f};
                        const Vector2p pos0{5.0, -2.0};

                        Vector2f vel_a = vel0;
                        update_vel_accel_xy(vel_a, accel, dt, limits[l], errs[ve]);

                        Vector2p pos_b = pos0;
                        Vector2f vel_b = vel0;
                        update_pos_vel_accel_xy(pos_b, vel_b, accel, dt,
                                                limits[l], errs[pe], errs[ve]);

                        // Position printed as a decimal, not float bits:
                        // it is postype_t and a float round-trip would hide
                        // exactly the precision this type exists for.
                        printf("%d,%u,%.17g,%.17g,%u,%u,%u,%u,%u,%u,"
                               "%u,%u,%u,%u,%u,%u,%.17g,%.17g,%u,%u\n", idx++,
                               fbits(dt),
                               (double)pos0.x, (double)pos0.y,
                               fbits(vel0.x), fbits(vel0.y),
                               fbits(accel.x), fbits(accel.y),
                               fbits(limits[l].x), fbits(limits[l].y),
                               fbits(errs[pe].x), fbits(errs[pe].y),
                               fbits(errs[ve].x), fbits(errs[ve].y),
                               fbits(vel_a.x), fbits(vel_a.y),
                               (double)pos_b.x, (double)pos_b.y,
                               fbits(vel_b.x), fbits(vel_b.y));
                    }
                }
            }
        }
    }

    return 0;
}
'''


def main():
    objects = vehicle_link.objects(stage_dir="/tmp/controlxy_parity/vehicle")
    build(HARNESS, objects, BUILD, "AP_Math/control.cpp",
          link_flags=vehicle_link.LINK_FLAGS)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
