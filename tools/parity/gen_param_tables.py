#!/usr/bin/env python3
"""Turn the reference build's parameter dump into two fixtures (FW-004 slice 2).

  param_structure.csv    the var_info tree: what the port is given
  param_enumeration.csv  upstream's own first()/next() walk: what it must produce

Keeping these separate is what stops the test being circular. If the port's
tables were generated from the enumeration, the test would only confirm that a
lookup finds what was put into it. Generated from the structure and compared
against the enumeration, it tests the traversal, the name concatenation and the
group_id encoding -- which is what actually decides where a parameter is stored.

The dump prints each group entry against its PARENT's path, in array order, so
sibling order is recovered by accumulating per parent rather than being stated
explicitly.
"""
import csv
import sys
from pathlib import Path

DUMP = Path("/tmp/paramdump.txt")
OUT_DIR = Path("/srv/ardumaster/ports/plane-fw-rust/fixtures")

if not DUMP.exists():
    sys.exit("%s missing -- run tools/sitl_diff/run_param_dump.py first" % DUMP)

structure = []   # (path, index_in_parent, key, idx, type, flags, name)
enumeration = []
seen_children = {}
frame_type_flags = None

for line in DUMP.read_text(errors="replace").splitlines():
    if line.startswith("F,"):
        frame_type_flags = int(line.split(",", 1)[1])
    elif line.startswith("V,"):
        # V,<i>,<key>,<type>,<flags>,<name>
        f = line.split(",", 5)
        i, key, ptype, flags, name = int(f[1]), int(f[2]), int(f[3]), int(f[4]), f[5]
        structure.append(("", i, key, 0, ptype, flags, name))
    elif line.startswith("G,"):
        # G,<parent_path>,<idx>,<type>,<flags>,<name>
        f = line.split(",", 5)
        path, idx, ptype, flags, name = f[1], int(f[2]), int(f[3]), int(f[4]), f[5]
        pos = seen_children.get(path, 0)
        seen_children[path] = pos + 1
        structure.append((path, pos, 0, idx, ptype, flags, name))
    elif line.startswith("P,"):
        # P,<name>,<key>,<idx>,<group_element>,<type>,<default>
        f = line.split(",")
        if len(f) < 8:
            continue
        enumeration.append(
            [f[1], int(f[2]), int(f[3]), int(f[4]), int(f[5]), f[6], f[7].strip()]
        )

if not structure or not enumeration:
    sys.exit("dump did not contain both sections")
if frame_type_flags is None:
    sys.exit("dump did not record _frame_type_flags -- reapply add_param_dump.py")

OUT_DIR.mkdir(parents=True, exist_ok=True)

sp = OUT_DIR / "param_structure.csv"
with open(sp, "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["parent_path", "pos", "key", "idx", "type", "flags", "name"])
    for row in structure:
        w.writerow(row)
print("wrote %s (%d entries: %d top level, %d group)"
      % (sp.name, len(structure),
         sum(1 for r in structure if r[0] == ""),
         sum(1 for r in structure if r[0] != "")))

fp = OUT_DIR / "param_frame.csv"
with open(fp, "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["name", "value"])
    w.writerow(["frame_type_flags", frame_type_flags])
print("wrote %s (frame_type_flags=%d)" % (fp.name, frame_type_flags))

ep = OUT_DIR / "param_enumeration.csv"
with open(ep, "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["name", "key", "idx", "group_element", "type", "default", "value"])
    for row in enumeration:
        w.writerow(row)
print("wrote %s (%d parameters)" % (ep.name, len(enumeration)))

# a few facts worth stating out loud, since they shape the port's traversal
by_type = {}
for r in enumeration:
    by_type[r[4]] = by_type.get(r[4], 0) + 1
print("parameters by type tag: %s" % dict(sorted(by_type.items())))
vec = [r for r in enumeration if r[4] == 5]
print("Vector3f entries: %d (each also yields three float components)" % len(vec))
longest = max(enumeration, key=lambda r: len(r[0]))
print("longest name: %r (%d chars, buffer is 16)" % (longest[0], len(longest[0])))
depth = max((r[0].count(".") + 1) for r in structure if r[0] != "")
print("deepest group nesting seen: %d level(s)" % depth)
