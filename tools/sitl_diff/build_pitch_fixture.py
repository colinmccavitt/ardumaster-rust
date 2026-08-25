#!/usr/bin/env python3
"""Build the pitch-controller replay fixture: PCTI + PCTO joined on TimeUS.

Both messages are written from the same `get_servo_out` call with the same
timestamp, so the join is exact rather than interpolated. Any row missing from
either stream is dropped and reported rather than approximated.

The rows must be replayed in order. PCTI carries the integrator entering the
call, but the controller also holds unlogged low-pass state, so the recorded
integrator is a check that the state evolved identically, not a seed. See
patches/add_pcti.py.

Parameters come from the log's own PARM records.
"""
import csv
import sys
from pathlib import Path

sys.path.insert(0, "/srv/ardumaster/ports/plane-fw-rust/tools/sitl_diff")
from extract_fixtures import read_series  # noqa: E402

from pymavlink import mavutil  # noqa: E402

LOG = Path("/srv/ardumaster/upstream/plane-4.7.0/logs/00000002.BIN")
OUT = Path("/srv/ardumaster/ports/plane-fw-rust/fixtures/pitch_replay.csv")
PARAMS = Path("/srv/ardumaster/ports/plane-fw-rust/fixtures/pitch_replay_params.csv")

PCTI = ["ae", "sc", "di", "gm", "gy", "as", "e2t", "dt", "ig", "rr", "pr", "rs", "ps"]
PCTO = ["out", "tgt", "act", "P", "I", "D", "F", "DF"]

i = dict(read_series(LOG, "PCTI", PCTI))
o = dict(read_series(LOG, "PCTO", PCTO))

common = sorted(set(i) & set(o))
print("PCTI {:,}  PCTO {:,}  ->  {:,} exact-joined".format(len(i), len(o), len(common)))
dropped = max(len(i), len(o)) - len(common)
if dropped:
    print("  {} row(s) unmatched and dropped".format(dropped))
if not common:
    sys.exit("no joined rows -- is the PCTI/PCTO patch applied to the reference build?")

OUT.parent.mkdir(parents=True, exist_ok=True)
with open(OUT, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["time_us"] + ["in_" + x for x in PCTI] + ["out_" + x for x in PCTO])
    for t in common:
        w.writerow(
            [t]
            + ["{:.9g}".format(v) for v in i[t]]
            + ["{:.9g}".format(v) for v in o[t]]
        )
print("wrote {} ({} input cols, {} output cols)".format(OUT.name, len(PCTI), len(PCTO)))

# The loop period is logged per record; a gap that is not one period means a
# call went unrecorded and the replay has to segment there.
dts = sorted({round(v[PCTI.index("dt")], 6) for v in i.values()})
print("logged loop period(s): {}".format(dts))
gaps = {}
prev = None
for t in common:
    if prev is not None:
        gaps[round((t - prev) * 1e-6, 4)] = gaps.get(round((t - prev) * 1e-6, 4), 0) + 1
    prev = t
for g, n in sorted(gaps.items(), key=lambda kv: -kv[1])[:5]:
    print("  gap {:>8.4f}s  x{:,}".format(g, n))

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

KEY = ["PTCH_RATE_P", "PTCH_RATE_I", "PTCH_RATE_D", "PTCH_RATE_FF",
       "PTCH_RATE_IMAX", "PTCH_RATE_FLTT", "PTCH_RATE_FLTE", "PTCH_RATE_FLTD",
       "PTCH_RATE_SMAX", "PTCH2SRV_TCONST", "PTCH2SRV_RMAX_UP",
       "PTCH2SRV_RMAX_DN", "PTCH2SRV_RLL", "ROLL_LIMIT_DEG",
       "AIRSPEED_MIN", "AIRSPEED_MAX"]
print("pitch gains from the flight:")
for k in KEY:
    print("  {:<18} {}".format(k, params.get(k, "<absent>")))
