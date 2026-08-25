#!/usr/bin/env python3
"""Run the reference build with AP_PARAM_DUMP set and capture the tables.

SITL's SERIAL0 is a TCP listener that blocks until something connects, and the
vehicle's setup() -- and therefore load_all(), where the dump hook lives -- does
not run until it does. So this starts the binary, connects to 5760, holds the
connection open until the dump appears, and then stops the process.
"""
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path("/srv/ardumaster/upstream/plane-4.7.0")
OUT = Path("/tmp/paramdump.txt")
ERR = Path("/tmp/paramdump.err")

for p in (OUT, ERR):
    if p.exists():
        p.unlink()

env = dict(os.environ, AP_PARAM_DUMP="1")
with open(OUT, "wb") as fo, open(ERR, "wb") as fe:
    proc = subprocess.Popen(
        [
            str(ROOT / "build/sitl/bin/arduplane"),
            "--home", "-35.36,149.16,585,354",
            "--model", "plane",
        ],
        cwd=str(ROOT), env=env, stdout=fo, stderr=fe,
    )

sock = None
deadline = time.time() + 90
while time.time() < deadline:
    if sock is None:
        try:
            sock = socket.create_connection(("127.0.0.1", 5760), timeout=2)
            print("connected to SERIAL0")
        except OSError:
            time.sleep(0.5)
            continue
    if proc.poll() is not None:
        break
    if OUT.exists() and b"END_PARAMS" in OUT.read_bytes():
        break
    time.sleep(0.5)

if sock is not None:
    sock.close()
if proc.poll() is None:
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()

text = OUT.read_text(errors="replace") if OUT.exists() else ""
v = sum(1 for l in text.splitlines() if l.startswith("V,"))
g = sum(1 for l in text.splitlines() if l.startswith("G,"))
p = sum(1 for l in text.splitlines() if l.startswith("P,"))
print("V=%d G=%d P=%d  (END_PARAMS present: %s)" % (v, g, p, "END_PARAMS" in text))
if p == 0:
    print("--- last stdout ---")
    print("\n".join(text.splitlines()[-6:]))
    print("--- last stderr ---")
    if ERR.exists():
        print("\n".join(ERR.read_text(errors="replace").splitlines()[-6:]))
    sys.exit(1)
for line in text.splitlines():
    if line.startswith(("V,", "G,", "P,")):
        print("sample:", line)
        break
