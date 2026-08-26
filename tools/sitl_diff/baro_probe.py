#!/usr/bin/env python3
"""Explain the cold-eeprom BARO enumeration anomaly (FW-004, carried from slice 3a).

The note says: on a cold eeprom the BARO group enumerates 1 parameter instead
of ~36. Slice 4 is storage conversion, which cares about first-boot behaviour,
so this has to be understood before it rather than after.

Two variables, so a 2x2 rather than a single reproduction:

  eeprom   cold (no eeprom.bin in cwd) vs warm (one written by a prior boot)
  dump     early (AP_PARAM_DUMP_NOW set in the environment, so the hook at the
           top of load_all() fires on the FIRST call, during startup) vs late
           (the one_second_loop hook sets it after 15s and calls load_all()
           again, so the walk happens with the vehicle fully up)

If the anomaly tracks the eeprom, it is about storage. If it tracks the dump
timing, it is about initialisation order and the fixture was simply recorded
too early. Those need different answers in slice 4, which is why it is worth
one run rather than one guess.
"""
import os
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path("/srv/ardumaster/upstream/plane-4.7.0")
BIN = ROOT / "build/sitl/bin/arduplane"
WORK = Path("/tmp/baro_probe")
WARM_SEED = WORK / "warm_seed_eeprom.bin"


def run_case(name, cold, early, timeout=90):
    """Boot the reference build once and capture its parameter dump."""
    d = WORK / name
    if d.exists():
        shutil.rmtree(d)
    d.mkdir(parents=True)

    if not cold:
        if not WARM_SEED.exists():
            return {"name": name, "error": "no warm seed yet"}
        shutil.copy(WARM_SEED, d / "eeprom.bin")

    out = d / "dump.txt"
    err = d / "dump.err"
    env = dict(os.environ, AP_PARAM_DUMP="1")
    if early:
        # Fire the hook at the top of load_all() on the first call.
        env["AP_PARAM_DUMP_NOW"] = "1"

    with open(out, "wb") as fo, open(err, "wb") as fe:
        proc = subprocess.Popen(
            [str(BIN), "--home", "-35.36,149.16,585,354", "--model", "plane"],
            cwd=str(d), env=env, stdout=fo, stderr=fe,
        )

    sock = None
    deadline = time.time() + timeout
    while time.time() < deadline:
        if sock is None:
            try:
                sock = socket.create_connection(("127.0.0.1", 5760), timeout=2)
            except OSError:
                time.sleep(0.4)
                continue
        if proc.poll() is not None:
            break
        if out.exists() and b"END_PARAMS" in out.read_bytes():
            break
        time.sleep(0.4)

    if sock is not None:
        sock.close()
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)

    text = out.read_text(errors="replace") if out.exists() else ""
    lines = text.splitlines()

    params = [l for l in lines if l.startswith("P,")]
    baro = [l for l in params if l.split(",")[1].startswith("BARO")]
    # The structure walk is independent of the enumeration: if the tables are
    # intact but the walk is short, the fault is in the walk, not the tables.
    struct_lines = [l for l in lines if l.startswith("G,")]
    v_lines = [l for l in lines if l.startswith("V,")]
    uptime = next((l.split(",")[1] for l in lines if l.startswith("U,")), "?")
    frame = next((l.split(",")[1] for l in lines if l.startswith("F,")), "?")

    return {
        "name": name,
        "cold": cold,
        "early": early,
        "complete": "END_PARAMS" in text,
        "uptime_ms": uptime,
        "frame_flags": frame,
        "v": len(v_lines),
        "g": len(struct_lines),
        "params": len(params),
        "baro": len(baro),
        "baro_names": [l.split(",")[1] for l in baro][:6],
        "eeprom_after": (d / "eeprom.bin").stat().st_size
        if (d / "eeprom.bin").exists() else 0,
        "dir": str(d),
    }


def main():
    WORK.mkdir(parents=True, exist_ok=True)
    if not BIN.exists():
        sys.exit("reference build missing: %s" % BIN)

    # Seed a warm eeprom the way a real second boot would get one: let a cold
    # boot run and keep what it wrote.
    print("== seeding a warm eeprom from a cold boot ==")
    seed = run_case("seed", cold=True, early=False)
    src = Path(seed["dir"]) / "eeprom.bin"
    if src.exists():
        shutil.copy(src, WARM_SEED)
        print("   seeded %d bytes" % WARM_SEED.stat().st_size)
    else:
        print("   WARNING: cold boot wrote no eeprom.bin")

    cases = [
        ("cold_late", True, False),
        ("warm_late", False, False),
        ("cold_early", True, True),
        ("warm_early", False, True),
    ]
    results = [seed]
    for name, cold, early in cases:
        print("== %s ==" % name)
        r = run_case(name, cold, early)
        results.append(r)
        print("   %s" % r)

    print()
    print("%-12s %-6s %-6s %-9s %-8s %-7s %-6s %s"
          % ("case", "cold", "early", "complete", "uptime", "params", "BARO", "v/g"))
    for r in results:
        if "error" in r:
            print("%-12s ERROR %s" % (r["name"], r["error"]))
            continue
        print("%-12s %-6s %-6s %-9s %-8s %-7d %-6d %d/%d"
              % (r["name"], r["cold"], r["early"], r["complete"],
                 r["uptime_ms"], r["params"], r["baro"], r["v"], r["g"]))


main()
