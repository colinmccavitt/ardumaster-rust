#!/usr/bin/env python3
"""Build the longitudinal-slice fixtures from one flight.

    vpslice_pitch.csv  PCTI + PCTO  the pitch controller
    vpslice_join.csv   PLNP         the TECS-to-controller join

17 significant digits: nine round-trips a float32 and truncates a ten-digit
integer, which cost an afternoon on the L1 replay.
"""
import csv
import sys
from pathlib import Path

sys.path.insert(0, "/srv/ardumaster/ports/plane-fw-rust/tools/sitl_diff")
from extract_fixtures import read_series  # noqa: E402

from pymavlink import mavutil  # noqa: E402

LOG = Path("/srv/ardumaster/upstream/plane-4.7.0/logs/00000002.BIN")
OUT_DIR = Path("/srv/ardumaster/ports/plane-fw-rust/fixtures")

PCTI = ["ae", "sc", "di", "gm", "gy", "as", "e2t", "dt", "ig", "rr", "pr", "rs", "ps"]
PCTO = ["out", "tgt", "act", "P", "I", "D", "F", "DF"]
PLNP = ["tpd", "pmin", "pmax", "nav", "trm", "thr", "kff", "dem", "ps", "ae"]

i = dict(read_series(LOG, "PCTI", PCTI))
o = dict(read_series(LOG, "PCTO", PCTO))
common = sorted(set(i) & set(o))
print("PCTI %d  PCTO %d  ->  %d joined" % (len(i), len(o), len(common)))
if not common:
    sys.exit("no pitch rows -- is add_pcti.py applied?")

OUT_DIR.mkdir(parents=True, exist_ok=True)
with open(OUT_DIR / "vpslice_pitch.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time_us"] + ["in_" + c for c in PCTI] + ["out_" + c for c in PCTO])
    for t in common:
        w.writerow([t] + ["{:.17g}".format(v) for v in i[t]]
                       + ["{:.17g}".format(v) for v in o[t]])
print("  wrote vpslice_pitch.csv")

rows = read_series(LOG, "PLNP", PLNP)
print("PLNP %d record(s)" % len(rows))
if not rows:
    sys.exit("no PLNP rows -- is add_plnp.py applied?")
with open(OUT_DIR / "vpslice_join.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time_us"] + ["out_" + c for c in PLNP])
    for t, vals in rows:
        w.writerow([t] + ["{:.17g}".format(v) for v in vals])
print("  wrote vpslice_join.csv")

m = mavutil.mavlink_connection(str(LOG))
params = {}
while True:
    msg = m.recv_match(type="PARM")
    if msg is None:
        break
    params[msg.Name] = msg.Value
with open(OUT_DIR / "vpslice_params.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["name", "value"])
    for k in sorted(params):
        w.writerow([k, "{:.9g}".format(params[k])])
print("  wrote vpslice_params.csv (%d parameters)" % len(params))
