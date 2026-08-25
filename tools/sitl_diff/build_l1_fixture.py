#!/usr/bin/env python3
"""Build the L1 navigation replay fixture: L1I + L1O joined on TimeUS.

Both messages are written from the same update_waypoint call with the same
timestamp, so the join is exact rather than interpolated.

Only update_waypoint is logged. The vehicle also calls update_loiter,
update_heading_hold and update_level_flight, and all four share state, so the
gap histogram below is worth reading: a run of waypoint calls at the loop rate
is replayable continuously, and a break in it means one of the other entry
points ran in between and moved the state.
"""
import csv
import sys
from pathlib import Path

sys.path.insert(0, "/srv/ardumaster/ports/plane-fw-rust/tools/sitl_diff")
from extract_fixtures import read_series  # noqa: E402

from pymavlink import mavutil  # noqa: E402

LOG = Path("/srv/ardumaster/upstream/plane-4.7.0/logs/00000002.BIN")
OUT = Path("/srv/ardumaster/ports/plane-fw-rust/fixtures/l1_replay.csv")
PARAMS = Path("/srv/ardumaster/ports/plane-fw-rust/fixtures/l1_replay_params.csv")

L1I = ["us", "ms", "la", "ln", "gx", "gy", "yw", "ys", "pt", "e2",
       "pa", "po", "na", "no", "dm"]
L1O = ["lad", "nrc", "nbr", "ber", "xte", "tbc", "l1d", "xti", "enu", "exi"]

i = dict(read_series(LOG, "L1I", L1I))
o = dict(read_series(LOG, "L1O", L1O))

common = sorted(set(i) & set(o))
print("L1I {:,}  L1O {:,}  ->  {:,} exact-joined".format(len(i), len(o), len(common)))
if not common:
    sys.exit("no joined rows -- is the L1I/L1O patch applied, and did the flight navigate?")

OUT.parent.mkdir(parents=True, exist_ok=True)
with open(OUT, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time_us"] + ["in_" + x for x in L1I] + ["out_" + x for x in L1O])
    for t in common:
        w.writerow(
            [t]
            # 17 significant digits, not 9. Nine is exactly enough to
            # round-trip a float32 and NOT enough for a ten-digit integer:
            # a longitude of 1491652374 was being written as 1491652370,
            # four units of 1e-7 degrees, about four centimetres.
            + ["{:.17g}".format(v) for v in i[t]]
            + ["{:.17g}".format(v) for v in o[t]]
        )
print("wrote {} ({} input cols, {} output cols)".format(OUT.name, len(L1I), len(L1O)))

# Runs of consecutive calls are what can be replayed continuously; a break
# means another entry point ran and moved the shared state.
gaps = {}
prev = None
for t in common:
    if prev is not None:
        g = round((t - prev) * 1e-6, 3)
        gaps[g] = gaps.get(g, 0) + 1
    prev = t
print("gaps between logged waypoint calls:")
for g, n in sorted(gaps.items(), key=lambda kv: -kv[1])[:6]:
    print("  {:>8.3f}s  x{:,}".format(g, n))

m = mavutil.mavlink_connection(str(LOG))
params = {}
while True:
    msg = m.recv_match(type="PARM")
    if msg is None:
        break
    params[msg.Name] = msg.Value
with open(PARAMS, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["name", "value"])
    for k in sorted(params):
        w.writerow([k, "{:.9g}".format(params[k])])
print("wrote {} ({} parameters)".format(PARAMS.name, len(params)))

for k in ["NAVL1_PERIOD", "NAVL1_DAMPING", "NAVL1_XTRACK_I", "NAVL1_LIM_BANK"]:
    print("  {:<16} {}".format(k, params.get(k, "<absent>")))
