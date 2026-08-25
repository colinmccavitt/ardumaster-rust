#!/usr/bin/env python3
r"""Log AP_L1_Control's waypoint law, reference build only (FW-016).

Two messages, because the inputs alone fill a Write() record: the format
string lives in a 17-byte field, so 16 fields including TimeUS is the ceiling.

    L1I  everything update_waypoint reads, including both waypoints
    L1O  what it produced, plus the state it CARRIED IN

The carried state matters. Only update_waypoint is logged, but the vehicle
also calls update_loiter, update_heading_hold and update_level_flight, and all
four share `_last_Nu`, `_L1_dist` and the capture latch. So a replay that just
runs the logged calls in order would be evolving state that upstream had
changed underneath it. Recording `_last_Nu` and `_L1_xtrack_i` as they stood
entering each call lets the replay notice exactly where that happened, reseed,
and say how often -- rather than silently drifting.

`nav_roll_cd()` is called here too. It is const and side-effect free, and it
is the number the roll controller actually receives, so logging it makes the
L1-to-aileron chain comparable end to end.

REFERENCE BUILD ONLY, never the port.
"""
import argparse
import sys
from pathlib import Path

TARGET = Path(
    "/srv/ardumaster/upstream/plane-4.7.0/libraries/AP_L1_Control/AP_L1_Control.cpp"
)

IN_LABELS = "TimeUS,us,ms,la,ln,gx,gy,yw,ys,pt,e2,pa,po,na,no,dm"
# TimeUS us ms la ln gx gy yw ys pt e2 pa po na no dm
IN_FMT = "QIIiifffiffiiiif"
OUT_LABELS = "TimeUS,lad,nrc,nbr,ber,xte,tbc,l1d,xti,enu,exi"
OUT_FMT = "Qfffffiffff"

ANCHOR = """    _data_is_stale = false; // status are correctly updated with current waypoint data
}

// update L1 control for loitering"""

PATCH = """    _data_is_stale = false; // status are correctly updated with current waypoint data

    // ---- reference-build-only logging ----
    AP_Logger *l1_logger = AP_Logger::get_singleton();
    if (l1_logger != nullptr) {
        l1_logger->WriteCritical(
            "L1I", "%s", "%s",
            AP_HAL::micros64(),
            (uint32_t)now,
            (uint32_t)AP_HAL::millis(),
            (int32_t)_current_loc.lat,
            (int32_t)_current_loc.lng,
            (float)_groundspeed_vector.x,
            (float)_groundspeed_vector.y,
            (float)get_yaw(),
            (int32_t)get_yaw_sensor(),
            (float)AP::ahrs().get_pitch_rad(),
            (float)AP::ahrs().get_EAS2TAS(),
            (int32_t)prev_WP.lat,
            (int32_t)prev_WP.lng,
            (int32_t)next_WP.lat,
            (int32_t)next_WP.lng,
            (float)dist_min);
        l1_logger->WriteCritical(
            "L1O", "%s", "%s",
            AP_HAL::micros64(),
            (float)_latAccDem,
            (float)nav_roll_cd(),
            (float)_nav_bearing,
            (float)_bearing_error,
            (float)_crosstrack_error,
            (int32_t)_target_bearing_cd,
            (float)_L1_dist,
            (float)_L1_xtrack_i,
            (float)l1_entry_last_Nu,
            (float)l1_entry_xtrack_i);
    }
    // ---- end reference-build-only logging ----
}

// update L1 control for loitering""" % (IN_LABELS, IN_FMT, OUT_LABELS, OUT_FMT)

ENTRY_ANCHOR = """    uint32_t now = AP_HAL::micros();
    float dt = (now - _last_update_waypoint_us) * 1.0e-6f;"""
ENTRY_PATCH = """    uint32_t now = AP_HAL::micros();
    // reference-build-only: the state carried into this call
    const float l1_entry_last_Nu = _last_Nu;
    const float l1_entry_xtrack_i = _L1_xtrack_i;
    float dt = (now - _last_update_waypoint_us) * 1.0e-6f;"""


def check(labels, fmt, name):
    n = len(labels.split(","))
    if len(fmt) > 16:
        sys.exit("%s: %d fields -- a Write() format field holds at most 16"
                 % (name, len(fmt)))
    if len(labels) > 64:
        sys.exit("%s: label string is %d chars, limit 64" % (name, len(labels)))
    if n != len(fmt):
        sys.exit("%s: %d labels vs %d type chars (%r)" % (name, n, len(fmt), fmt))
    print("  %s: %d fields, %d chars of labels" % (name, n, len(labels)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()

    if not TARGET.exists():
        sys.exit("target not found")
    text = TARGET.read_text()

    if args.revert:
        if PATCH not in text:
            print("L1I/L1O not applied")
            return
        text = text.replace(PATCH, ANCHOR).replace(ENTRY_PATCH, ENTRY_ANCHOR)
        TARGET.write_text(text)
        print("reverted L1I/L1O")
        return

    if PATCH in text:
        print("L1I/L1O already applied")
        return
    for name, a in (("body", ANCHOR), ("entry", ENTRY_ANCHOR)):
        if text.count(a) != 1:
            sys.exit("%s anchor matched %d times, expected 1" % (name, text.count(a)))

    check(IN_LABELS, IN_FMT, "L1I")
    check(OUT_LABELS, OUT_FMT, "L1O")

    text = text.replace(ANCHOR, PATCH, 1).replace(ENTRY_ANCHOR, ENTRY_PATCH, 1)
    marker = '#include "AP_L1_Control.h"\n'
    if marker not in text:
        sys.exit("include anchor not found")
    if "AP_Logger/AP_Logger.h" not in text:
        text = text.replace(marker, marker + "#include <AP_Logger/AP_Logger.h>\n", 1)

    TARGET.write_text(text)
    print("applied L1I/L1O logging")


if __name__ == "__main__":
    main()
