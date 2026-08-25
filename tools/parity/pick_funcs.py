"""Choose which output functions a default-shape sweep should cover.

Picks representatives of every distinct group in `aux_servo_function_setup`,
all six actuators (the GCC case-range that is easiest to misparse), and
functions with no default so the leave-alone case is covered too.

Derived from the source rather than written down: an earlier hand-picked list
used remembered enum values and several were wrong -- function 27 came back a
range where an elevon was expected -- which makes coverage of every group a
hope rather than a fact.

NOT WIRED UP YET. The sweep it feeds needs channels that have never been given
a default shape, because `type_setup` latches on first use and nothing public
clears it. Adding it to the motors harness perturbed every later section:
claiming all 32 channels drove `invalid_mask` to zero for the whole run (caught
by the aux-function test's coverage guard), and claiming 26 changed the motor
mask that `update_throttle_range` measures. It needs its own harness binary,
which is now cheap -- that harness has no stubs left to duplicate. See COP-030.
"""
import re
from pathlib import Path

SRC = Path("/srv/ardumaster/upstream/plane-4.7.0/libraries/SRV_Channel/SRV_Channel.h")
AUX = SRC.parent / "SRV_Channel_aux.cpp"

# Function name -> value.
enum_body = SRC.read_text()
enum_body = enum_body[enum_body.index("typedef enum"):]
enum_body = enum_body[:enum_body.index("k_nr_aux_servo_functions")]
by_name = {m.group(1): int(m.group(2))
           for m in re.finditer(r"^\s*(k_\w+)\s*=\s*(\d+)\s*,", enum_body, re.M)}

# The setup switch, grouped.
body = AUX.read_text()
body = body[body.index("void SRV_Channel::aux_servo_function_setup"):]
body = body[:body.index("default:")]

groups = {}
pending = []
for line in body.splitlines():
    s = line.strip()
    m = re.match(r"case (k_\w+)(?:\s*\.\.\.\s*(k_\w+))?\s*:", s)
    if m:
        lo, hi = m.group(1), m.group(2)
        if hi is None:
            pending.append(by_name[lo])
        else:
            pending.extend(range(by_name[lo], by_name[hi] + 1))
        continue
    m = re.match(r"set_(range|angle)\((-?\d+)\);", s)
    if m and pending:
        groups.setdefault((m.group(1), int(m.group(2))), []).extend(pending)
        pending = []

assert groups, "no groups parsed"
with_default = sorted({v for vs in groups.values() for v in vs})

picked = []
# Every actuator: the case-range worth proving.
actuators = sorted(groups.get(("angle", 1), []))
picked.extend(actuators)
# Two representatives of every other group.
for key in sorted(groups):
    if key == ("angle", 1):
        continue
    picked.extend(sorted(groups[key])[:2])
# Fill the rest with functions that have no default, so "left alone" is covered.
no_default = [v for v in sorted(by_name.values()) if v not in with_default]
for v in no_default:
    if len(picked) >= 32:
        break
    picked.append(v)

picked = picked[:32]
assert len(picked) == 32, "need exactly 32, got %d" % len(picked)
assert len(set(picked)) == 32, "duplicates in the swept functions"

print("groups: %s" % sorted(groups))
print("picked: %s" % picked)

# Patch the harness.
p = Path("/srv/ardumaster/ports/plane-fw-rust/tools/parity/gen_motors_fixture.py")
t = p.read_text()
start = t.index("        static const uint16_t FUNCS[32] = {")
end = t.index("};", start) + 2
new = ("        // Chosen by tools/parity/pick_funcs.py from the same parse that\n"
       "        // builds the Rust table: every actuator, two representatives of\n"
       "        // each other group, and functions with no default at all. Picking\n"
       "        // these by hand got several wrong.\n"
       "        static const uint16_t FUNCS[32] = {\n            "
       + ", ".join(str(v) for v in picked) + ",\n        };")
p.write_text(t[:start] + new + t[end:])
print("harness updated")
