#!/usr/bin/env python3
"""Build the vertical-slice fixtures from one flight.

Three streams, all from the same run, which is the whole point: the existing
L1 and roll fixtures came from different flights and cannot be composed.

    vslice_l1.csv    L1I + L1O   navigation, 10 Hz
    vslice_roll.csv  RCTI + RCTO attitude control, 50 Hz
    vslice_join.csv  PLNR        the vehicle glue between them

17 significant digits throughout. Nine is enough to round-trip a float32 and
NOT enough for a ten-digit integer -- longitudes and microsecond counters both
overflow it, and that cost an afternoon on the L1 replay.
"""
import csv
import sys
from pathlib import Path

sys.path.insert(0, "/srv/ardumaster/ports/plane-fw-rust/tools/sitl_diff")
from extract_fixtures import read_series  # noqa: E402

from pymavlink import mavutil  # noqa: E402

LOG = Path("/srv/ardumaster/upstream/plane-4.7.0/logs/00000002.BIN")
OUT_DIR = Path("/srv/ardumaster/ports/plane-fw-rust/fixtures")

STREAMS = {
    "vslice_l1": {
        "a": ("L1I", ["us", "ms", "la", "ln", "gx", "gy", "yw", "ys", "pt", "e2",
                      "pa", "po", "na", "no", "dm"]),
        "b": ("L1O", ["lad", "nrc", "nbr", "ber", "xte", "tbc", "l1d", "xti",
                      "enu", "exi"]),
    },
    "vslice_roll": {
        "a": ("RCTI", ["ae", "sc", "di", "gm", "gy", "as", "e2t", "dt", "ig"]),
        "b": ("RCTO", ["out", "tgt", "act", "P", "I", "D", "F", "DF"]),
    },
}

OUT_DIR.mkdir(parents=True, exist_ok=True)

for name, spec in STREAMS.items():
    an, acols = spec["a"]
    bn, bcols = spec["b"]
    a = dict(read_series(LOG, an, acols))
    b = dict(read_series(LOG, bn, bcols))
    common = sorted(set(a) & set(b))
    print("%s %d  %s %d  ->  %d joined" % (an, len(a), bn, len(b), len(common)))
    if not common:
        sys.exit("no rows for %s -- is the patch applied?" % name)
    path = OUT_DIR / (name + ".csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_us"] + ["in_" + c for c in acols] + ["out_" + c for c in bcols])
        for t in common:
            w.writerow([t]
                       + ["{:.17g}".format(v) for v in a[t]]
                       + ["{:.17g}".format(v) for v in b[t]])
    print("  wrote %s" % path.name)

# the join is a single message; its fields are all "outputs" for the reader
JOIN = ["nrc", "rlc", "nav", "rs", "ae", "pt"]
rows = read_series(LOG, "PLNR", JOIN)
print("PLNR %d record(s)" % len(rows))
if not rows:
    sys.exit("no PLNR rows -- is the vehicle patch applied?")
path = OUT_DIR / "vslice_join.csv"
with open(path, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time_us"] + ["out_" + c for c in JOIN])
    for t, vals in rows:
        w.writerow([t] + ["{:.17g}".format(v) for v in vals])
print("  wrote %s" % path.name)

# RCOU is logged by default, so the chain reaches the actual servo signal
# without another patch. ARM comes with it because a disarmed vehicle holds
# its servos at trim regardless of what the controllers ask for.
m = mavutil.mavlink_connection(str(LOG))
rcou, arm = [], []
while True:
    x = m.recv_match(type=["RCOU", "ARM"])
    if x is None:
        break
    if x.get_type() == "RCOU":
        rcou.append((x.TimeUS, x.C1, x.C2, x.C3, x.C4))
    else:
        arm.append((x.TimeUS, x.ArmState))
print("RCOU %d record(s), %d arm event(s)" % (len(rcou), len(arm)))
if not rcou:
    sys.exit("no RCOU records")


def armed_at(t):
    state = 0
    for at, s in arm:
        if at <= t:
            state = s
        else:
            break
    return state


with open(OUT_DIR / "vslice_rcou.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time_us", "out_c1", "out_c2", "out_c3", "out_c4", "out_armed"])
    for row in rcou:
        w.writerow([row[0]] + ["{:.17g}".format(v) for v in row[1:]] + [armed_at(row[0])])
print("  wrote vslice_rcou.csv")

m = mavutil.mavlink_connection(str(LOG))
params = {}
while True:
    msg = m.recv_match(type="PARM")
    if msg is None:
        break
    params[msg.Name] = msg.Value
path = OUT_DIR / "vslice_params.csv"
with open(path, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["name", "value"])
    for k in sorted(params):
        w.writerow([k, "{:.9g}".format(params[k])])
print("  wrote %s (%d parameters)" % (path.name, len(params)))
