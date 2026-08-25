#!/usr/bin/env python3
"""Extract the reference flight's parameters from its log into the fixture set.

The replay test previously hard-coded parameter values from memory. Every one
of them was wrong: TRIM_THROTTLE is 50 not 45, AIRSPEED_CRUISE 22 not 12,
PTCH_LIM_MIN_DEG -20 not -25. The 45-vs-50 error alone put a constant 0.05
offset on every throttle sample, because throttle feed-forward is seeded from
`nomThr = throttle_cruise * 0.01`.

Guessing parameters is the same failure as guessing the FlightStage
discriminants. The fix is the same: read them from the source of truth. The log
records a PARM message for every parameter the vehicle was running, so the
replay can configure itself from the flight it is replaying.

Emits every parameter, not just the TECS ones, so later module replays
(attitude, navigation) can use the same file.
"""
import csv
from pathlib import Path

from pymavlink import mavutil

LOG = Path("/srv/ardumaster/upstream/plane-4.7.0/logs/00000002.BIN")
OUT = Path("/srv/ardumaster/ports/plane-fw-rust/fixtures/tecs_replay_params.csv")

m = mavutil.mavlink_connection(str(LOG))
params = {}
while True:
    msg = m.recv_match(type="PARM")
    if msg is None:
        break
    # later values win: a parameter set mid-flight should be reflected
    params[msg.Name] = msg.Value

OUT.parent.mkdir(parents=True, exist_ok=True)
with open(OUT, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["name", "value"])
    for name in sorted(params):
        w.writerow([name, "{:.9g}".format(params[name])])

print("wrote {} ({} parameters)".format(OUT.name, len(params)))

# report the ones the TECS replay depends on, so a change upstream is visible
KEY = [
    "TRIM_THROTTLE", "AIRSPEED_MIN", "AIRSPEED_MAX", "AIRSPEED_CRUISE",
    "PTCH_LIM_MAX_DEG", "PTCH_LIM_MIN_DEG", "STALL_PREVENTION", "THR_SLEWRATE",
]
print("key values:")
for k in KEY:
    print("  {:<18} {}".format(k, params.get(k, "<absent>")))
