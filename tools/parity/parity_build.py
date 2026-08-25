#!/usr/bin/env python3
"""Compile a parity harness against upstream's own build configuration.

Shared by the per-module fixture generators. The CRC harness could compile
`crc.cpp` standalone with one stub header; nothing else in AP_Math can — the
include tree reaches AP_Common, AP_HAL and the board configuration, and
stubbing those by hand risks the harness compiling against different
definitions than the vehicle does, which would silently invalidate the whole
comparison.

Instead this reads `build/sitl/compile_commands.json`, takes the exact flags
waf used for a real translation unit, and links the object files waf already
produced. The harness is then built the same way the firmware was.
"""
import json
import shlex
import subprocess
import sys
from pathlib import Path

ROOT = Path("/srv/ardumaster/upstream/plane-4.7.0")
CC_JSON = ROOT / "build/sitl/compile_commands.json"


def flags_for(source_suffix):
    """The compile flags waf used for the translation unit ending in `suffix`."""
    if not CC_JSON.exists():
        sys.exit("%s missing -- build the reference SITL first (./waf plane)" % CC_JSON)
    entries = json.loads(CC_JSON.read_text())
    hit = None
    for e in entries:
        if e.get("file", "").endswith(source_suffix):
            hit = e
            break
    if hit is None:
        sys.exit("no compile command for %s" % source_suffix)

    # waf emits the "arguments" array form; the "command" string form is the
    # other half of the compile_commands spec, so accept either
    if "arguments" in hit:
        argv = list(hit["arguments"])
    else:
        argv = shlex.split(hit["command"])

    out = []
    skip_next = False
    for i, a in enumerate(argv):
        if skip_next:
            skip_next = False
            continue
        if i == 0:
            continue  # the compiler itself
        if a in ("-c", "-o"):
            skip_next = a == "-o"
            continue
        if a.endswith(".cpp") or a.endswith(".o"):
            continue
        out.append(a)
    return hit["directory"], out


def make_archive(path):
    """Bundle every object waf built into one static archive.

    Loose `.o` files on a link line are all included unconditionally; archive
    members are pulled only when something references them. That is what keeps
    a harness for one small module from dragging in the entire HAL just because
    a comparison operator reaches `is_equal`.
    """
    objs = sorted(str(p) for p in (ROOT / "build/sitl/libraries").rglob("*.o"))
    if not objs:
        sys.exit("no built objects under build/sitl/libraries -- run ./waf plane")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    r = subprocess.run(["ar", "rcs", str(path)] + objs, capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit("ar failed:" + "\n" + r.stderr)
    return path


def build(harness_src, objects, out_path, source_suffix, link_flags=()):
    """Compile `harness_src` with upstream's flags and link `objects`.

    `link_flags` is for harnesses that link a single translation unit rather
    than the whole archive. AP_Param.cpp, for instance, references the
    filesystem, the GCS, the logger and the HAL, none of which its pure
    encoding functions touch -- and linking the full archive to satisfy them
    drags in the vehicle, and then the Lua bindings, and fails.

    Passing `-Wl,--unresolved-symbols=ignore-all` there leaves those references
    dangling. That is safe only because the harness never calls them: if it
    ever did, the process would die immediately rather than return a wrong
    answer, which is the same guarantee the aborting stubs elsewhere give.
    """
    cwd, flags = flags_for(source_suffix)
    src = Path(str(out_path) + ".cpp")
    src.parent.mkdir(parents=True, exist_ok=True)
    src.write_text(harness_src)

    resolved = [o if Path(o).is_absolute() else str(ROOT / o) for o in objects]
    cmd = ["g++"] + flags + [str(src)] + resolved + list(link_flags) + ["-o", str(out_path)]
    r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit("harness compile failed:" + "\n" + r.stderr[-4000:])
    if r.stderr.strip():
        print("compiler warnings:" + "\n" + r.stderr.strip()[:2000])
    return out_path


def run(binary):
    r = subprocess.run([str(binary)], capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit("harness failed:" + "\n" + r.stderr[-4000:])
    return r.stdout
