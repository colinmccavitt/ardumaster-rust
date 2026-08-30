#!/usr/bin/env python3
"""Build the yaw-damper and steering replay fixtures from one reference flight.

Both controllers log a single message rather than the input/output pair roll
and pitch needed, so there is no join and no dropped-row accounting: each
record is already complete.

Neither controller is driven by a scheduler period. Both measure their own loop
period from AP_HAL::millis(), which is logged as `ms`, so the replay is driven
by the same millisecond count upstream saw rather than by the record timestamp.

Parameters come from the log's own PARM records.
"""
import csv
import sys
from pathlib import Path

sys.path.insert(0, "/srv/ardumaster/ports/ardumaster-rust/tools/sitl_diff")
from extract_fixtures import read_series  # noqa: E402

from pymavlink import mavutil  # noqa: E402

LOG = Path("/srv/ardumaster/upstream/plane-4.7.0/logs/00000002.BIN")
OUT_DIR = Path("/srv/ardumaster/ports/ardumaster-rust/fixtures")

SERIES = {
    "steer_replay": {
        "msg": "STCI",
        "inputs": ["ms", "dr", "gs", "yr", "rv", "ig"],
        "outputs": ["out", "tgt", "act", "P", "I", "D", "F"],
    },
    "yaw_replay": {
        "msg": "YCTI",
        "inputs": ["ms", "sc", "di", "rr", "as", "yr", "ay", "ab", "ig"],
        "outputs": ["out", "I", "D"],
    },
}

OUT_DIR.mkdir(parents=True, exist_ok=True)
wrote_any = False

for name, spec in SERIES.items():
    cols = spec["inputs"] + spec["outputs"]
    rows = read_series(LOG, spec["msg"], cols)
    print("%s: %d record(s)" % (spec["msg"], len(rows)))
    if not rows:
        print("  SKIPPED -- is the patch applied, and does the flight reach this code?")
        continue
    wrote_any = True

    path = OUT_DIR / (name + ".csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(
            ["time_us"]
            + ["in_" + c for c in spec["inputs"]]
            + ["out_" + c for c in spec["outputs"]]
        )
        for t, vals in rows:
            w.writerow([t] + ["{:.9g}".format(v) for v in vals])
    print("  wrote %s (%d input cols, %d output cols)"
          % (path.name, len(spec["inputs"]), len(spec["outputs"])))

    # gap histogram, so a hole is noticed here rather than in the replay
    gaps = {}
    prev = None
    for t, _ in rows:
        if prev is not None:
            g = round((t - prev) * 1e-6, 4)
            gaps[g] = gaps.get(g, 0) + 1
        prev = t
    for g, n in sorted(gaps.items(), key=lambda kv: -kv[1])[:4]:
        print("    gap {:>8.4f}s  x{:,}".format(g, n))

if not wrote_any:
    sys.exit("no fixtures written")

m = mavutil.mavlink_connection(str(LOG))
params = {}
while True:
    msg = m.recv_match(type="PARM")
    if msg is None:
        break
    params[msg.Name] = msg.Value

pm = OUT_DIR / "yaw_steer_replay_params.csv"
with open(pm, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["name", "value"])
    for k in sorted(params):
        w.writerow([k, "{:.9g}".format(params[k])])
print("wrote %s (%d parameters)" % (pm.name, len(params)))

KEY = ["YAW2SRV_DAMP", "YAW2SRV_INT", "YAW2SRV_SLIP", "YAW2SRV_RLL",
       "YAW2SRV_IMAX", "STEER2SRV_TCONST", "STEER2SRV_P", "STEER2SRV_I",
       "STEER2SRV_D", "STEER2SRV_FF", "STEER2SRV_IMAX", "STEER2SRV_MINSPD",
       "STEER2SRV_DRTSPD", "STEER2SRV_DRTFCT", "STEER2SRV_DRTMIN",
       "AIRSPEED_MIN", "AIRSPEED_MAX"]
print("gains from the flight:")
for k in KEY:
    print("  {:<18} {}".format(k, params.get(k, "<absent>")))
