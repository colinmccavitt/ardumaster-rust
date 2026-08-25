#!/usr/bin/env python3
r"""Parity fixture for AP_AHRS_DCM's matrix maintenance (FW-008).

`normalize` and `renorm` are pure functions of the matrix, so they are compared
against upstream's compiled code rather than needing a flight -- which also
lets the grid include matrices a flight would never produce.

The harness lays down zeroed storage and treats it as an AP_AHRS_DCM rather
than constructing one: the constructor reaches the INS, GPS, compass and
barometer, none of which these two functions touch. AP_Math is linked because
`constrain_value_line` and the wrap helpers live there, and a dangling call
becomes a jump to address zero -- which presents as a segfault before the
first printf and looks nothing like a link problem.

Inputs stay inside the accept band on purpose. Outside it upstream calls
`reset(true)`, which reads the HAL and the INS and cannot run here; that path
is covered by the port's own tests instead.
"""
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import parity_build  # noqa: E402

OUT = Path("/srv/ardumaster/ports/plane-fw-rust/fixtures/dcm_normalize.csv")

# Row-major 3x3 matrices, chosen to exercise the correction rather than to look
# like attitudes: already orthonormal, drifted, uniformly scaled, and skewed by
# amounts the integration could plausibly produce.
MATRICES = [
    [1, 0, 0, 0, 1, 0, 0, 0, 1],
    [1, 0.01, 0, -0.01, 1, 0, 0, 0, 1],
    [1, 0.1, 0, -0.1, 1, 0, 0, 0, 1],
    [0.99, 0.02, 0.01, -0.02, 0.99, 0.03, -0.01, -0.03, 0.99],
    [1.05, 0, 0, 0, 1.05, 0, 0, 0, 1.05],
    [0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5],
    [0.7071, 0.7071, 0, -0.7071, 0.7071, 0, 0, 0, 1],
    [0.866, 0.5, 0, -0.5, 0.866, 0, 0, 0, 1],
    [0.9, 0.3, 0.1, -0.3, 0.9, 0.2, 0.05, -0.25, 0.95],
    [1.2, 0.4, -0.1, -0.35, 1.1, 0.2, 0.15, -0.2, 1.3],
    [0.6, 0.1, 0.02, -0.1, 0.6, 0.05, 0.0, -0.05, 0.6],
    [1, 0.5, 0, -0.5, 1, 0, 0, 0, 1],
]


def lit(v):
    """A C float literal that always carries a decimal point, so `f` parses."""
    return repr(float(v)) + "f"


MATS = ",\n".join(
    "    {" + ", ".join(lit(v) for v in m) + "}" for m in MATRICES
)

HARNESS = r"""
#define private public
#define protected public
#include <AP_AHRS/AP_AHRS_DCM.h>
#undef private
#undef protected

#include <stdio.h>

static const float mats[][9] = {
__MATS__
};

// The constructor reaches the INS, GPS, compass and barometer; normalize and
// renorm touch none of them, so the harness lays down zeroed storage instead.
alignas(AP_AHRS_DCM) static unsigned char storage[sizeof(AP_AHRS_DCM)];

int main()
{
    AP_AHRS_DCM &d = *reinterpret_cast<AP_AHRS_DCM *>(storage);
    const unsigned n = sizeof(mats)/sizeof(mats[0]);
    for (unsigned i = 0; i < n; i++) {
        const float *m = mats[i];
        d._dcm_matrix.a = Vector3f(m[0], m[1], m[2]);
        d._dcm_matrix.b = Vector3f(m[3], m[4], m[5]);
        d._dcm_matrix.c = Vector3f(m[6], m[7], m[8]);
        d._renorm_val_sum = 0;
        d._renorm_val_count = 0;

        fprintf(stderr, "row %u\n", i);
        d.normalize();

        printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
               "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u\n",
               (double)m[0], (double)m[1], (double)m[2],
               (double)m[3], (double)m[4], (double)m[5],
               (double)m[6], (double)m[7], (double)m[8],
               (double)d._dcm_matrix.a.x, (double)d._dcm_matrix.a.y, (double)d._dcm_matrix.a.z,
               (double)d._dcm_matrix.b.x, (double)d._dcm_matrix.b.y, (double)d._dcm_matrix.b.z,
               (double)d._dcm_matrix.c.x, (double)d._dcm_matrix.c.y, (double)d._dcm_matrix.c.z,
               (double)d._renorm_val_sum, (unsigned)d._renorm_val_count);
    }
    return 0;
}
""".replace("__MATS__", MATS)

out_dir = Path("/tmp/parity_dcm")
binary = parity_build.build(
    HARNESS,
    [
        "build/sitl/libraries/AP_AHRS/AP_AHRS_DCM.cpp.4.o",
        "build/sitl/libraries/AP_Math/AP_Math.cpp.0.o",
    ],
    out_dir / "dcm_normalize",
    "AP_AHRS/AP_AHRS_DCM.cpp",
    link_flags=["-Wl,--unresolved-symbols=ignore-all"],
)
text = parity_build.run(binary)

rows = [l.split(",") for l in text.splitlines() if l.strip()]
if not rows:
    sys.exit("harness produced nothing")

OUT.parent.mkdir(parents=True, exist_ok=True)
cols = (["in_%d" % i for i in range(9)] + ["out_%d" % i for i in range(9)]
        + ["renorm_sum", "renorm_count"])
with open(OUT, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(cols)
    for r in rows:
        w.writerow(r)
print("wrote %s (%d matrices)" % (OUT.name, len(rows)))
print("first corrected row: %s" % (rows[1][9:12],))
