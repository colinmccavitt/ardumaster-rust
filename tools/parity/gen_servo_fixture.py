#!/usr/bin/env python3
"""Parity fixture for SRV_Channel's output conversion (FW-018).

These are pure functions of the channel's configuration, so they can be
compared directly against upstream's compiled code rather than needing a
flight. That is the stronger test anyway: a flight exercises whatever
configuration it happened to have, while this sweeps the awkward ones on
purpose.

The grid deliberately includes the cases the code special-cases or divides by:

  * a maximum not above the minimum, and a zero range or angle, which upstream
    answers with the minimum or the trim rather than dividing
  * a trim hard against the minimum and against the maximum, so one half of an
    angle output has zero span
  * scaled values beyond the limits in both directions
  * reversed channels, which negate before constraining for angle outputs and
    after for range outputs -- not the same thing

`#define private public` reaches the conversion functions and the AP_Int16
members, which are protected.
"""
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import parity_build  # noqa: E402

OUT = Path("/srv/ardumaster/ports/ardumaster-rust/fixtures/servo_pwm.csv")

# (servo_min, servo_trim, servo_max)
CONFIGS = [
    (1000, 1500, 2000),   # symmetric, the usual case
    (1000, 1200, 2000),   # trim below centre
    (1000, 1800, 2000),   # trim above centre
    (1000, 1000, 2000),   # trim on the minimum: the lower half has no span
    (1000, 2000, 2000),   # trim on the maximum: the upper half has no span
    (1100, 1500, 1900),   # narrower travel
    (2000, 1500, 1000),   # max not above min: upstream returns the minimum
    (1500, 1500, 1500),   # degenerate
]
HIGH_OUT = [4500, 100, 1, 0]
SCALED = [0.0, 1.0, -1.0, 0.5, -0.5, 45.0, -45.0, 100.0, -100.0,
          4500.0, -4500.0, 9000.0, -9000.0, 0.4999, -0.4999, 2249.9, -2249.9]

HARNESS = r"""
#define private public
#define protected public
#include <SRV_Channel/SRV_Channel.h>
#undef private
#undef protected

#include <stdio.h>

static const int mins[]  = {%(mins)s};
static const int trims[] = {%(trims)s};
static const int maxs[]  = {%(maxs)s};
static const int highs[] = {%(highs)s};
static const float vals[] = {%(vals)s};

// The constructor calls AP_Param::setup_object_defaults, which cannot run
// outside a vehicle. These conversions are const members reading six plain
// fields, so the harness lays down zeroed storage and sets those fields
// directly rather than constructing.
alignas(SRV_Channel) static unsigned char storage[sizeof(SRV_Channel)];

int main()
{
    SRV_Channel &c = *reinterpret_cast<SRV_Channel *>(storage);
    const unsigned ncfg = sizeof(mins)/sizeof(mins[0]);
    const unsigned nhigh = sizeof(highs)/sizeof(highs[0]);
    const unsigned nval = sizeof(vals)/sizeof(vals[0]);

    for (unsigned ci = 0; ci < ncfg; ci++) {
        c.servo_min.set(mins[ci]);
        c.servo_trim.set(trims[ci]);
        c.servo_max.set(maxs[ci]);
        for (unsigned hi = 0; hi < nhigh; hi++) {
            for (unsigned rv = 0; rv < 2; rv++) {
                c.reversed.set(rv);
                for (unsigned vi = 0; vi < nval; vi++) {
                    c.set_angle(highs[hi]);
                    unsigned a = c.pwm_from_angle(vals[vi]);
                    c.set_range(highs[hi]);
                    unsigned r = c.pwm_from_range(vals[vi]);
                    printf("%%d,%%d,%%d,%%d,%%d,%%.9g,%%u,%%u\n",
                           mins[ci], trims[ci], maxs[ci], highs[hi], rv,
                           (double)vals[vi], a, r);
                }
            }
        }
    }
    return 0;
}
""" % {
    "mins": ",".join(str(c[0]) for c in CONFIGS),
    "trims": ",".join(str(c[1]) for c in CONFIGS),
    "maxs": ",".join(str(c[2]) for c in CONFIGS),
    "highs": ",".join(str(h) for h in HIGH_OUT),
    "vals": ",".join(repr(v) + "f" for v in SCALED),
}

out_dir = Path("/tmp/parity_servo")
binary = parity_build.build(
    HARNESS,
    [
        "build/sitl/libraries/SRV_Channel/SRV_Channel.cpp.0.o",
        # Both conversions call constrain_float, which is
        # constrain_value_line<float> and lives in AP_Math. Left dangling by
        # --unresolved-symbols it becomes a call to address zero, which is a
        # segfault before the first printf rather than a link error.
        "build/sitl/libraries/AP_Math/AP_Math.cpp.0.o",
    ],
    out_dir / "servo_pwm",
    "SRV_Channel/SRV_Channel.cpp",
    link_flags=["-Wl,--unresolved-symbols=ignore-all"],
)
text = parity_build.run(binary)

rows = [l.split(",") for l in text.splitlines() if l.strip()]
if not rows:
    sys.exit("harness produced nothing")

OUT.parent.mkdir(parents=True, exist_ok=True)
with open(OUT, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["servo_min", "servo_trim", "servo_max", "high_out", "reversed",
                "scaled", "pwm_angle", "pwm_range"])
    for r in rows:
        w.writerow(r)
print("wrote %s (%d cases)" % (OUT.name, len(rows)))
print("sample: %s" % (rows[0],))
