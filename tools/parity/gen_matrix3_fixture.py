#!/usr/bin/env python3
r"""Parity fixture for Matrix3::rotate (FW-002, needed by FW-008).

The DCM integration step is `_dcm_matrix.rotate(omega * dt)`, so this is the
one piece of genuinely new arithmetic the AHRS slice introduced. It is a pure
function of a matrix and a vector, and it lives in AP_Math -- where the
archive-linking harness already works -- so there is no excuse for not
comparing it against upstream.

Links a small explicit object list and does NOT pass
--unresolved-symbols=ignore-all. That flag turns a missing symbol into a
silent call to address zero -- which is what made the AP_AHRS_DCM attempt jump
back into startup code and present as a static-initialisation fault. Without
it the link names the symbol and a stub can be written for it.

The grid mixes matrices that are attitudes with ones that are not, and
rotation vectors from the tiny (a realistic gyro step) to the absurd, because
the function is first-order and its error grows with the angle -- reproducing
that growth is the point.
"""
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import parity_build  # noqa: E402

OUT = Path("/srv/ardumaster/ports/ardumaster-rust/fixtures/matrix3_rotate.csv")

MATRICES = [
    [1, 0, 0, 0, 1, 0, 0, 0, 1],
    [0.7071, -0.7071, 0, 0.7071, 0.7071, 0, 0, 0, 1],
    [0.866, 0, 0.5, 0, 1, 0, -0.5, 0, 0.866],
    [0.99, 0.02, 0.01, -0.02, 0.99, 0.03, -0.01, -0.03, 0.99],
    [1.05, 0.1, -0.2, 0.05, 0.95, 0.3, -0.15, 0.25, 1.1],
    [0, 1, 0, -1, 0, 0, 0, 0, 1],
]
# rotation vectors: a 400 Hz gyro step, a slow roll, a fast one, and far past
# where a first-order approximation is meaningful
ROTATIONS = [
    [0, 0, 0],
    [0.0025, 0, 0],
    [0, 0.0025, 0],
    [0, 0, 0.0025],
    [0.0025, -0.005, 0.00125],
    [0.05, 0.05, 0.05],
    [0.5, -0.25, 0.1],
    [1.0, 0, 0],
    [-1.5, 2.0, -0.5],
]


def lit(v):
    """A C float literal that always carries a decimal point."""
    return repr(float(v)) + "f"


MATS = ",\n".join("    {" + ", ".join(lit(v) for v in m) + "}" for m in MATRICES)
ROTS = ",\n".join("    {" + ", ".join(lit(v) for v in r) + "}" for r in ROTATIONS)

HARNESS = r"""
#include <AP_Math/AP_Math.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_CustomRotations/AP_CustomRotations.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdlib.h>

// Aborting stubs, the same set gen_vector3_fixture.py uses. Nothing under
// test reaches any of them; the convention is that a stub aborts rather than
// returning a value upstream never produced.
void AP_InternalError::error(const AP_InternalError::error_t, uint16_t) {}
namespace AP {
AP_InternalError &internalerror() { static AP_InternalError d; return d; }
AP_CustomRotations &custom_rotations() { abort(); }
}
void AP_HAL::panic(const char *m, ...) { fputs(m, stderr); abort(); }
void AP_CustomRotations::rotate(enum Rotation, Vector3f &) { abort(); }
void AP_CustomRotations::rotate(enum Rotation, Vector3d &) { abort(); }

static const float mats[][9] = {
__MATS__
};
static const float rots[][3] = {
__ROTS__
};

int main()
{
    const unsigned nm = sizeof(mats)/sizeof(mats[0]);
    const unsigned nr = sizeof(rots)/sizeof(rots[0]);
    for (unsigned i = 0; i < nm; i++) {
        for (unsigned j = 0; j < nr; j++) {
            const float *m = mats[i];
            Matrix3f M(Vector3f(m[0], m[1], m[2]),
                       Vector3f(m[3], m[4], m[5]),
                       Vector3f(m[6], m[7], m[8]));
            M.rotate(Vector3f(rots[j][0], rots[j][1], rots[j][2]));
            printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                   "%.9g,%.9g,%.9g,"
                   "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
                   (double)m[0], (double)m[1], (double)m[2],
                   (double)m[3], (double)m[4], (double)m[5],
                   (double)m[6], (double)m[7], (double)m[8],
                   (double)rots[j][0], (double)rots[j][1], (double)rots[j][2],
                   (double)M.a.x, (double)M.a.y, (double)M.a.z,
                   (double)M.b.x, (double)M.b.y, (double)M.b.z,
                   (double)M.c.x, (double)M.c.y, (double)M.c.z);
        }
    }
    return 0;
}
""".replace("__MATS__", MATS).replace("__ROTS__", ROTS)

out_dir = Path("/tmp/parity_matrix3")
# A small explicit object list and NO --unresolved-symbols=ignore-all. That
# flag turns a missing symbol into a silent call to address zero, which is what
# made the AP_AHRS_DCM attempt jump back into startup code and look like a
# static-initialisation fault. Without it the link fails loudly and names the
# symbol.
binary = parity_build.build(
    HARNESS,
    [
        "build/sitl/libraries/AP_Math/matrix3.cpp.0.o",
        "build/sitl/libraries/AP_Math/vector3.cpp.0.o",
        "build/sitl/libraries/AP_Math/vector2.cpp.0.o",
        "build/sitl/libraries/AP_Math/AP_Math.cpp.0.o",
    ],
    out_dir / "matrix3_rotate",
    "AP_Math/matrix3.cpp",
)
text = parity_build.run(binary)

rows = [l.split(",") for l in text.splitlines() if l.strip()]
if not rows:
    sys.exit("harness produced nothing")

OUT.parent.mkdir(parents=True, exist_ok=True)
cols = (["in_%d" % i for i in range(9)] + ["rot_%d" % i for i in range(3)]
        + ["out_%d" % i for i in range(9)])
with open(OUT, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(cols)
    for r in rows:
        w.writerow(r)
print("wrote %s (%d cases)" % (OUT.name, len(rows)))
