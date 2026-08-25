#!/usr/bin/env python3
"""Build the roll-controller replay fixture: RCTI + RCTO joined on TimeUS.

Both messages are written from the same `get_servo_out` call with the same
timestamp, so the join is exact rather than interpolated. Any row missing from
either stream is dropped and reported rather than approximated.

RCTI carries the integrator as it stood BEFORE the update. That is a check on
the replay, not a seed for it: the controller also carries unlogged filter
state, so the rows must be replayed in order and the recorded integrator is
what proves that state evolved identically. Holes therefore matter here as much
as they did for TECS, and the replay segments at them.

The parameters are extracted from the same log, so the replay configures itself
from the flight rather than from values written down by hand. Every parameter
the TECS work wrote by hand turned out to be wrong.
"""
import csv
import sys
from pathlib import Path

sys.path.insert(0, "/srv/ardumaster/ports/plane-fw-rust/tools/sitl_diff")
from extract_fixtures import read_series  # noqa: E402

from pymavlink import mavutil  # noqa: E402

LOG = Path("/srv/ardumaster/upstream/plane-4.7.0/logs/00000002.BIN")
OUT = Path("/srv/ardumaster/ports/plane-fw-rust/fixtures/roll_replay.csv")
PARAMS = Path("/srv/ardumaster/ports/plane-fw-rust/fixtures/roll_replay_params.csv")

RCTI = ["ae", "sc", "di", "gm", "gy", "as", "e2t", "dt", "ig"]
RCTO = ["out", "tgt", "act", "P", "I", "D", "F", "DF"]

i = dict(read_series(LOG, "RCTI", RCTI))
o = dict(read_series(LOG, "RCTO", RCTO))

common = sorted(set(i) & set(o))
print("RCTI {:,}  RCTO {:,}  ->  {:,} exact-joined".format(len(i), len(o), len(common)))
dropped = max(len(i), len(o)) - len(common)
if dropped:
    print("  {} row(s) unmatched and dropped".format(dropped))

OUT.parent.mkdir(parents=True, exist_ok=True)
with open(OUT, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time_us"] + ["in_" + x for x in RCTI] + ["out_" + x for x in RCTO])
    for t in common:
        w.writerow(
            [t]
            + ["{:.9g}".format(v) for v in i[t]]
            + ["{:.9g}".format(v) for v in o[t]]
        )
print("wrote {} ({} input cols, {} output cols)".format(OUT.name, len(RCTI), len(RCTO)))

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
    for name in sorted(params):
        w.writerow([name, "{:.9g}".format(params[name])])
print("wrote {} ({} parameters)".format(PARAMS.name, len(params)))

KEY = ["RLL_RATE_P", "RLL_RATE_I", "RLL_RATE_D", "RLL_RATE_FF", "RLL_RATE_IMAX",
       "RLL_RATE_FLTT", "RLL_RATE_FLTE", "RLL_RATE_FLTD", "RLL_RATE_SMAX",
       "RLL2SRV_TCONST", "RLL2SRV_RMAX", "AIRSPEED_MIN"]
print("roll gains from the flight:")
for k in KEY:
    print("  {:<16} {}".format(k, params.get(k, "<absent>")))
