#!/usr/bin/env python3
"""FW-007 task 1: is upstream SITL replay-deterministic?

ADR-0005 assumes it is, and the whole golden-trajectory approach depends on
that. This runs upstream ArduPlane SITL twice with identical inputs and
compares the resulting dataflash logs.

Two things SITL needs that are not obvious:
  - it blocks on "Waiting for connection" until something attaches to the
    SERIAL0 TCP port, so we attach a socket that just drains bytes
  - it does not write a dataflash log while disarmed unless LOG_DISARMED=1
"""
import argparse
import filecmp
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
WORK = ROOT / "reference/determinism"
HOME_LOC = "-35.363261,149.165230,584,353"
PORT = 5760


def drain(sock, stop):
    """Keep the SERIAL0 connection alive; SITL will not run without it."""
    sock.settimeout(0.5)
    while not stop.is_set():
        try:
            if not sock.recv(4096):
                break
        except socket.timeout:
            continue
        except OSError:
            break


def run_once(rundir: Path, runtime: float, speedup: int) -> Path | None:
    rundir.mkdir(parents=True, exist_ok=True)

    # LOG_DISARMED so a log exists without flying a mission
    params = rundir / "params.parm"
    # There is no base plane.parm upstream - the plain plane model uses
    # built-in defaults - so only add what this test needs.
    text = DEFAULTS.read_text() if DEFAULTS.exists() else ""
    text += "\nLOG_DISARMED 1\nLOG_REPLAY 0\n"
    params.write_text(text)

    cmd = [
        str(BIN),
        "--model", "plane",
        "--home", HOME_LOC,
        "--speedup", str(speedup),
        "--defaults", str(params),
        "--wipe",
    ]
    with open(rundir / "sitl.out", "w") as out, open(rundir / "sitl.err", "w") as err:
        proc = subprocess.Popen(cmd, cwd=str(rundir), stdout=out, stderr=err)

        sock = None
        stop = threading.Event()
        thread = None
        # SITL binds the port a moment after launch
        for _ in range(50):
            try:
                sock = socket.create_connection(("127.0.0.1", PORT), timeout=1.0)
                break
            except OSError:
                time.sleep(0.2)

        if sock is not None:
            thread = threading.Thread(target=drain, args=(sock, stop), daemon=True)
            thread.start()
        else:
            print("  WARNING: could not attach to SERIAL0")

        time.sleep(runtime)

        stop.set()
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        if thread is not None:
            thread.join(timeout=2)
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()

    logs = sorted(rundir.rglob("*.BIN")) + sorted(rundir.rglob("*.bin"))
    return logs[0] if logs else None


def compare(a: Path, b: Path) -> None:
    sa, sb = a.stat().st_size, b.stat().st_size
    print("\n=== log sizes ===")
    print("  run1: {:,} bytes  {}".format(sa, a.name))
    print("  run2: {:,} bytes  {}".format(sb, b.name))

    if filecmp.cmp(str(a), str(b), shallow=False):
        print("\nRESULT: byte-identical - SITL is bit-for-bit replay-deterministic")
        return

    da, db = a.read_bytes(), b.read_bytes()
    n = min(len(da), len(db))
    first = next((i for i in range(n) if da[i] != db[i]), None)
    diff = sum(1 for i in range(n) if da[i] != db[i])

    print("\nRESULT: logs DIFFER")
    print("  size delta      : {}".format(sa - sb))
    print("  first difference: byte {}".format(first))
    print("  differing bytes : {:,} of {:,} compared ({:.2f}%)".format(
        diff, n, 100.0 * diff / n if n else 0.0))
    if first is not None:
        lo = max(0, first - 8)
        print("  run1 @{}: {}".format(lo, da[lo:lo + 24].hex()))
        print("  run2 @{}: {}".format(lo, db[lo:lo + 24].hex()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runtime", type=float, default=20.0)
    ap.add_argument("--speedup", type=int, default=20)
    args = ap.parse_args()

    if not BIN.exists():
        sys.exit("SITL binary not built: {}".format(BIN))

    if WORK.exists():
        shutil.rmtree(WORK)

    logs = []
    for i in (1, 2):
        print("=== run {} ({}s wall, speedup {}) ===".format(i, args.runtime, args.speedup))
        log = run_once(WORK / "run{}".format(i), args.runtime, args.speedup)
        if log is None:
            rd = WORK / "run{}".format(i)
            print("  NO LOG PRODUCED")
            print("  --- stdout tail ---")
            print("  " + "\n  ".join((rd / "sitl.out").read_text().splitlines()[-12:]))
            print("  --- stderr tail ---")
            print("  " + "\n  ".join((rd / "sitl.err").read_text().splitlines()[-8:]))
            sys.exit(2)
        print("  log: {} ({:,} bytes)".format(log.name, log.stat().st_size))
        logs.append(log)

    compare(logs[0], logs[1])


if __name__ == "__main__":
    main()
