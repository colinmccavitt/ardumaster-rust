#!/usr/bin/env python3
"""Parity fixture for Location's geodesics (FW-006 slice, needed by FW-016).

These three functions decide where the navigation controller thinks the
aircraft is relative to its waypoints, so an error here moves the whole flight
path rather than jittering a servo. They are also the first ported code whose
result depends on `ftype` being double: SITL sets HAL_WITH_EKF_DOUBLE, so
longitude_scale and get_bearing compute in double while LOCATION_SCALING_FACTOR
stays a float and get_distance_NE returns a Vector2f.

Reading that off the source is exactly the kind of thing that goes wrong
quietly, so the values come from upstream's own compiled code.

The grid deliberately includes the antimeridian and both poles, since
diff_longitude has a separate 64-bit path for coordinates that straddle the
sign boundary and longitude_scale is floored near the poles.
"""
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import parity_build  # noqa: E402

OUT = Path("/srv/ardumaster/ports/plane-fw-rust/fixtures/location_geo.csv")

# lat, lng in 1e-7 degrees
POINTS = [
    (0, 0),
    (-353632621, 1491652374),        # the SITL home location
    (-353642621, 1491662374),        # ~100 m away
    (376194500, -1224280000),        # northern hemisphere, western longitude
    (900000000, 0),                  # north pole
    (-900000000, 0),                 # south pole
    (0, 1799999999),                 # just west of the antimeridian
    (0, -1799999999),                # just east of it
    (450000000, 1799999000),
    (450000000, -1799999000),
    (100000, 100000),
    (-1, -1),
]

HARNESS = r"""
#include <AP_Common/Location.h>
#include <stdio.h>

static const int32_t lats[] = {%(lats)s};
static const int32_t lngs[] = {%(lngs)s};

int main()
{
    const unsigned n = sizeof(lats)/sizeof(lats[0]);
    for (unsigned i = 0; i < n; i++) {
        printf("scale,%%d,%%.17g\n", (int)lats[i],
               (double)Location::longitude_scale(lats[i]));
    }
    for (unsigned i = 0; i < n; i++) {
        for (unsigned j = 0; j < n; j++) {
            Location a; a.lat = lats[i]; a.lng = lngs[i];
            Location b; b.lat = lats[j]; b.lng = lngs[j];
            Vector2f ne = a.get_distance_NE(b);
            printf("pair,%%d,%%d,%%d,%%d,%%d,%%.9g,%%.9g,%%.17g,%%d\n",
                   (int)lats[i], (int)lngs[i], (int)lats[j], (int)lngs[j],
                   (int)Location::diff_longitude(lngs[j], lngs[i]),
                   (double)ne.x, (double)ne.y,
                   (double)a.get_bearing(b),
                   (int)a.get_bearing_to(b));
        }
    }
    return 0;
}
""" % {
    "lats": ",".join(str(p[0]) for p in POINTS),
    "lngs": ",".join(str(p[1]) for p in POINTS),
}

out_dir = Path("/tmp/parity_locgeo")
binary = parity_build.build(
    HARNESS,
    ["build/sitl/libraries/AP_Common/Location.cpp.0.o"],
    out_dir / "location_geo",
    "AP_Common/Location.cpp",
    link_flags=["-Wl,--unresolved-symbols=ignore-all"],
)
text = parity_build.run(binary)

scales, pairs = [], []
for line in text.splitlines():
    f = line.split(",")
    if f[0] == "scale":
        scales.append(f[1:])
    elif f[0] == "pair":
        pairs.append(f[1:])

if not pairs:
    sys.exit("harness produced nothing")

OUT.parent.mkdir(parents=True, exist_ok=True)
with open(OUT, "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["kind", "a", "b", "c", "d", "e", "f", "g", "h"])
    for s in scales:
        w.writerow(["scale"] + s + [""] * (8 - len(s)))
    for p in pairs:
        w.writerow(["pair"] + p)
print("wrote %s (%d scales, %d pairs)" % (OUT.name, len(scales), len(pairs)))
print("sample: %s" % (pairs[1],))
