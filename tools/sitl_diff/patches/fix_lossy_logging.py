#!/usr/bin/env python3
"""Make the reference-build logging non-lossy, and log the controller's own _DT.

The height-stage comparison showed the port's climb-rate scaler decaying faster
than upstream's. Working the arithmetic back from the logged values:

    upstream  0.9539 -> 0.9231 -> 0.9256   two steps of dt=0.1
    port      0.9539 ---------> 0.8943     one step of dt=0.2

Both are correct for the dt they were given. The fault is in the fixture: a
timestamp histogram over the 1,911 rows shows

    100 ms  1715 rows
    101 ms    96
    200 ms    98      <-- a record was dropped
  16600 ms     1      (a genuine pause)

`WriteStreaming` is explicitly droppable — it discards the record when the log
buffer is full rather than blocking. About 5% of records were lost, and the
replay integrated one 200 ms step wherever upstream had taken two 100 ms ones.

`WriteCritical` is the non-droppable variant, so all four reference messages
move to it. `_DT` is added to TECL so the assumption can be checked rather than
trusted: the replay can now assert that upstream's own timestep matches the
timestamp delta, and fail loudly if a record is ever dropped again.

REFERENCE BUILD ONLY, never the port.
"""
from pathlib import Path

TARGET = Path("/srv/ardumaster/upstream/plane-4.7.0/libraries/AP_TECS/AP_TECS.cpp")
text = TARGET.read_text()

# --- 1. the four reference messages become non-droppable ---
changed = 0
for msg in ("TECI", "TECJ", "TECK", "TECL"):
    needle = 'AP::logger().WriteStreaming(\n        "%s"' % msg
    if needle in text:
        text = text.replace(needle, 'AP::logger().WriteCritical(\n        "%s"' % msg, 1)
        changed += 1
    else:
        # TECI has a comment line between the call and the name
        alt = 'AP::logger().WriteStreaming(\n        // Labels must fit'
        if msg == "TECI" and alt in text:
            text = text.replace(alt, 'AP::logger().WriteCritical(\n        // Labels must fit', 1)
            changed += 1
        else:
            raise SystemExit("could not find the %s write call" % msg)
print("switched %d reference messages to WriteCritical" % changed)

# --- 2. add _DT to TECL so the timestep can be verified, not assumed ---
OLD_LABELS = '"TECL", "TimeUS,hdin,hdip,hrtl,hlpf,mcs,mss,crl,srl,pto,tcs,scs,pdu",\n        "QfffffffffBBf",'
NEW_LABELS = '"TECL", "TimeUS,hdin,hdip,hrtl,hlpf,mcs,mss,crl,srl,pto,tcs,scs,pdu,dt",\n        "QfffffffffBBff",'
assert OLD_LABELS in text, "TECL header not found"
text = text.replace(OLD_LABELS, NEW_LABELS, 1)

OLD_TAIL = """        (float)_pitch_dem_unc);
    // ---- end reference-build-only logging ----"""
NEW_TAIL = """        (float)_pitch_dem_unc,
        (float)_DT);
    // ---- end reference-build-only logging ----"""
assert OLD_TAIL in text
text = text.replace(OLD_TAIL, NEW_TAIL, 1)

labels = "TimeUS,hdin,hdip,hrtl,hlpf,mcs,mss,crl,srl,pto,tcs,scs,pdu,dt"
fmt = "QfffffffffBBff"
assert len(labels) <= 64, "label string is %d chars" % len(labels)
assert len(labels.split(",")) == len(fmt), "%d labels vs %d type chars" % (
    len(labels.split(",")), len(fmt))

TARGET.write_text(text)
print("TECL now logs _DT (%d labels, %d chars)" % (len(labels.split(",")), len(labels)))
