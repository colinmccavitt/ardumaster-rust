#!/usr/bin/env python3
"""FW-007: run SITL twice under the Lua mission driver, with no MAVLink client.

The whole point is that nothing external issues a command. SITL still needs a
SERIAL0 connection before it will run, so a socket attaches and drains bytes
and does nothing else -- the same approach determinism_test.py used for the
idle case.

Termination is on SIMULATED time, not wall time. The first version of this
got it wrong in the way slice 1 predicted: the driver's schedule ended
without stopping the vehicle, an RTL loiter never stops producing log data,
and both runs ran to the wall-clock backstop -- 1380 seconds simulated
against the 150 intended.

Now the driver disarms at a fixed simulated moment and LOG_DISARMED is off,
so the log closes when the vehicle does. The runner still waits for the log
to stop growing, but that now happens because of a simulated-time event
rather than because a timer expired. The wall-clock budget remains only as a
backstop, and a run that hits it should be treated as failed rather than
compared.
"""
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path("/srv/ardumaster")
BIN = ROOT / "upstream/plane-4.7.0/build/sitl/bin/arduplane"
DEFAULTS = ROOT / "upstream/plane-4.7.0/Tools/autotest/default_params/plane.parm"
DRIVER = Path(__file__).with_name("mission_driver.lua")
WORK = ROOT / "reference/lua_driver"
HOME_LOC = "-35.363261,149.165230,584,353"
PORT = 5760
SPEEDUP = 20
# The driver finishes at 120 s simulated; allow a margin beyond it.
SIM_SECONDS = 150


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


def run_once(rundir: Path) -> Path | None:
    if rundir.exists():
        shutil.rmtree(rundir)
    (rundir / "scripts").mkdir(parents=True)
    shutil.copy(DRIVER, rundir / "scripts" / "mission_driver.lua")

    params = rundir / "params.parm"
    text = DEFAULTS.read_text() if DEFAULTS.exists() else ""
    text += "\n".join([
        "",
        "SCR_ENABLE 1",
        # Off deliberately: the log must close when the driver disarms, so
        # the run ends on a simulated-time event rather than a stopwatch.
        "LOG_DISARMED 0",
        "LOG_REPLAY 0",
        # The pre-arm checks depend on sensor settling and EKF health, which
        # is the wall-clock-shaped variability being removed. arm_force skips
        # them, but ARMING_CHECK is cleared too so nothing else re-imposes it.
        "ARMING_CHECK 0",
        "",
    ])
    params.write_text(text)

    cmd = [
        str(BIN),
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

    # Wall-clock budget derived from the simulated target and the speedup,
    # with slack. The run is stopped when the log stops growing, not when the
    # budget expires -- the budget is only a backstop.
    budget = SIM_SECONDS / SPEEDUP + 60
    deadline = time.time() + budget
    logdir = rundir / "logs"
    last_size, stable_for = -1, 0.0
    hit_backstop = True
    while time.time() < deadline:
        time.sleep(2.0)
        logs = sorted(logdir.glob("*.BIN")) if logdir.exists() else []
        size = logs[-1].stat().st_size if logs else 0
        if size > 0 and size == last_size:
            stable_for += 2.0
            if stable_for >= 10.0:
                hit_backstop = False
                break
        else:
            stable_for = 0.0
        last_size = size

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
    if hit_backstop:
        print("  WARNING: hit the wall-clock backstop, so this run did not end")
        print("  on a simulated-time event. Do not compare it.")
        return None
    return biggest


def main():
    if not BIN.exists():
        sys.exit("missing %s" % BIN)
    WORK.mkdir(parents=True, exist_ok=True)

    produced = []
    for run in (1, 2):
        print("=== run %d ===" % run)
        produced.append(run_once(WORK / ("run%d" % run)))

    if not all(produced):
        print("\nat least one run produced no log; nothing to compare")
        return 2

    print("\nboth runs produced logs:")
    for p in produced:
        print("  %s %d bytes" % (p, p.stat().st_size))
    return 0


if __name__ == "__main__":
    sys.exit(main())
