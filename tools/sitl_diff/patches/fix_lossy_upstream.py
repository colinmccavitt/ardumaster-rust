#!/usr/bin/env python3
"""Make every TECS log message non-lossy in the reference build.

The reference-build messages (TECI/TECJ/TECK/TECL) were switched to
WriteCritical, but upstream's own TECS, TEC2, TEC3 and TEC4 are still
WriteStreaming and dropped 97 of 2,015 records. The fixture is an exact join
across all seven streams, so a drop in any one of them loses the row, and the
replay then integrates one long step where upstream took several short ones.

Logging volume is not a concern: this build exists only to produce reference
fixtures and runs in SITL. The alternative is a fixture with 5% of its steps
silently missing.

Every WriteStreaming in the file is switched rather than a named list, since
the indentation of each call differs and matching them individually is
needlessly brittle.

REFERENCE BUILD ONLY, never the port.
"""
from pathlib import Path

TARGET = Path("/srv/ardumaster/upstream/plane-4.7.0/libraries/AP_TECS/AP_TECS.cpp")
text = TARGET.read_text()

n = text.count("AP::logger().WriteStreaming(")
text = text.replace("AP::logger().WriteStreaming(", "AP::logger().WriteCritical(")
TARGET.write_text(text)

print("switched %d WriteStreaming calls to WriteCritical" % n)
print("WriteStreaming remaining: %d" % text.count("WriteStreaming"))
