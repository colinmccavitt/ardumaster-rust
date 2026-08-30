#!/usr/bin/env python3
"""FW-048: run the tick-injection-patched reference SITL binary N times.

Conventions lifted directly from `run_lua_runs.py` (FW-007): headless launch,
no MAVProxy/GCS client, a plain socket that only drains SERIAL0 so SITL does
not stall waiting for a connection.

The one structural difference from `run_lua_runs.py`: there is no Lua driver
to install into `scripts/`, because the command schedule is now baked into
the patched `AP_Scheduler::loop()` itself (see
`patches/add_tick_injection.py`). This runner's only job is to launch the
identical patched binary, drain its serial socket, and collect the dataflash
log - it issues no commands of its own.

TERMINATION - measured, not assumed, and different from run_lua_runs.py's
approach. run_lua_runs.py detects "the flight is over" by waiting for the
dataflash log to stop growing (LOG_DISARMED=0 means a disarmed vehicle
writes nothing further, so the file size goes flat). That does not work
here: the reference-build-only PLNR/RCTI/RCTO/TECI logging this run also
carries (added so the comparison table has the same fields as ADR-0008's
own Lua table) is written with `WriteCritical`/`WriteStreaming` from inside
the control loops themselves, which keep executing at their normal rate
even while disarmed (only the actuator outputs are suppressed) - so the log
NEVER goes flat, and log-size-stabilization would spin until the wall-clock
backstop on every run. A smoke run confirmed this directly: the schedule's
arm (tick 2000) through TAKEOFF (tick 3200) flew a real climb/loiter/RTL
profile that autoland-disarmed on its own around simulated t=963 s, and
after that the log kept growing from PLNR/TECI/RCTI/RCTO alone. So instead
this runner uses a fixed WALL-CLOCK run length chosen with generous margin
over that measured ~963 s simulated flight (`RUN_WALL_SECONDS` below,
comfortably longer at the configured speedup), then simply terminates the
process. The comparison step keeps its own window well inside that span
(see `tick_injection_tolerance_study.py`), so the arbitrary point at which
this runner's kill signal lands is never itself part of what gets compared.

NOISE: SIM_GYR1_RND / SIM_ACC1_RND are set to the same nonzero values
ArduPilot's own autotest uses for its "enable a noisy gyro" scenarios
(`Tools/autotest/arducopter.py`, e.g. line ~7779: SIM_GYR1_RND=20,
SIM_ACC1_RND=5). Upstream's own compiled-in default for both is 0 (see
`libraries/SITL/SITL.cpp` AP_GROUPINFO defaults) - autotest's Plane suite
does not turn them on by default, so FW-007's Lua runs effectively flew with
zero motor-vibration noise. ADR-0014's own open risk is specifically about
whether the `rand()` stream (shared, global, unseeded via `srand`) can drift
between runs of a deterministic command driver; leaving noise at the
compiled-in zero would still exercise the same number of `rand()` draws per
tick (`SIM_Aircraft.cpp` calls `rand_normal()` unconditionally and multiplies
by the noise scale after) but would make any resulting drift invisible in
the output telemetry. So noise here is deliberately turned ON, not left at
its convenient-but-blind default.
"""
import argparse
import shutil
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path("/srv/ardumaster")
HOME_LOC = "-35.363261,149.165230,584,353"
PORT = 5760
SPEEDUP = 20
# Measured directly (see TERMINATION above): a representative run's own
# TAKEOFF -> loiter -> RTL -> autoland profile disarms around simulated
# t=963 s. 110 wall seconds at 20x speedup is ~2200 s simulated - comfortable
# margin over that, so the kill always lands well after the flight (and any
# comparison window) is over.
RUN_WALL_SECONDS = 110


def drain(sock, stop):
    sock.settimeout(0.5)
    while not stop.is_set():
        try:
            if not sock.recv(4096):
                break
        except socket.timeout:
            continue
        except OSError:
            break


def run_once(rundir: Path, binary: Path) -> Path | None:
    if rundir.exists():
        shutil.rmtree(rundir)
    rundir.mkdir(parents=True)

    params = rundir / "params.parm"
    params.write_text("\n".join([
        # Realistic, nonzero motor-vibration noise - see module docstring.
        "SIM_GYR1_RND 20",
        "SIM_ACC1_RND 5",
        "LOG_DISARMED 0",
        "LOG_REPLAY 0",
        # The schedule calls arm_force(), which itself bypasses pre-arm
        # checks; ARMING_CHECK is cleared too so nothing re-imposes them
        # and a slow-settling EKF can never keep the schedule from arming
        # exactly on its scheduled tick.
        "ARMING_CHECK 0",
        "",
    ]))

    cmd = [
        str(binary),
        "--model", "plane",
        "--home", HOME_LOC,
        "--speedup", str(SPEEDUP),
        "--defaults", str(params),
        "--wipe",
    ]
    proc = subprocess.Popen(
        cmd, cwd=str(rundir),
        stdout=(rundir / "sitl.log").open("w"),
        stderr=subprocess.STDOUT,
    )

    stop = threading.Event()
    sock = None
    for _ in range(60):
        try:
            sock = socket.create_connection(("127.0.0.1", PORT), timeout=1)
            break
        except OSError:
            time.sleep(0.5)
    if sock is None:
        proc.kill()
        print("  could not attach to SERIAL0")
        return None

    t = threading.Thread(target=drain, args=(sock, stop), daemon=True)
    t.start()

    # Fixed wall-clock run length (see TERMINATION in the module docstring) -
    # not a "did it finish" detector, just a generous, measured-safe cutoff.
    time.sleep(RUN_WALL_SECONDS)

    stop.set()
    try:
        sock.close()
    except OSError:
        pass
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()

    logs = sorted((rundir / "logs").glob("*.BIN")) if (rundir / "logs").exists() else []
    if not logs:
        print("  no log produced")
        return None
    biggest = max(logs, key=lambda p: p.stat().st_size)
    print("  log %s (%d bytes)" % (biggest.name, biggest.stat().st_size))
    return biggest


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bin", required=True, type=Path,
                     help="path to the tick-injection-patched arduplane binary "
                          "(built inside the isolated worktree, never the shared tree)")
    ap.add_argument("--runs", type=int, default=4)
    ap.add_argument("--work", type=Path,
                     default=ROOT / "reference/tick_injection")
    args = ap.parse_args()

    if not args.bin.exists():
        sys.exit("missing binary: %s" % args.bin)
    args.work.mkdir(parents=True, exist_ok=True)

    produced = []
    for run in range(1, args.runs + 1):
        print("=== run %d of %d ===" % (run, args.runs))
        produced.append(run_once(args.work / ("run%d" % run), args.bin))

    if not all(produced):
        print("\nat least one run produced no usable log; nothing to compare")
        return 2

    print("\nall %d runs produced logs:" % args.runs)
    for p in produced:
        print("  %s %d bytes" % (p, p.stat().st_size))
    return 0


if __name__ == "__main__":
    sys.exit(main())
