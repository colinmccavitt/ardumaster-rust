"""Record which source Mode::update_target_altitude picks.

# Observing a branch that has no return value

The function returns nothing and every branch writes the same field, so the
branch is identified by *which* call it makes. Each one has a distinguishing
call, and all of them are undefined references in mode.cpp.o (checked with
nm -u first):

    flare / loiter / fall-through -> set_target_altitude_location(next_WP)
    approach                      -> setup_landing_glide_slope
    landing target                -> set_target_altitude_location(other)
    soaring                       -> reset_offset_altitude
    terrain                       -> set_target_altitude_proportion_terrain
    proportional                  -> set_target_altitude_proportion

Three branches make the same call with the same argument, and that is not a
gap in the recording -- it is a fact about the firmware. Those three do the
same thing, so the port collapses them into one outcome and the recording
compares outcomes rather than branch numbers.

# Driving the inputs

The landing and soaring predicates need subsystems a harness cannot bring up,
so they are wrapped and answered by the sweep, as is the terrain attempt. The
ladder itself is the firmware's, unmodified. `offset_cm` and the waypoint's
terrain flag are plain fields and are set directly.

The soaring branch is the exception and is NOT driven. soaring_controller's
predicates do not appear among mode.cpp.o's undefined symbols -- they are
inlined -- so there is nothing for --wrap to redirect, and standing up a real
soaring controller is a different slice. Every recorded row therefore has
soaring inactive, and the test says so rather than letting the column look
like coverage.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from parity_build import build, run  # noqa: E402
import plane_link  # noqa: E402

ROOT = Path("/srv/ardumaster/ports/ardumaster-rust")
OUT = ROOT / "fixtures/plane_target_altitude.csv"
BUILD = Path("/tmp/plane_alt_parity/harness")

WRAPS = {
    "IS_FLARING": "_ZNK10AP_Landing10is_flaringEv",
    "IS_ON_APPROACH": "_ZNK10AP_Landing14is_on_approachEv",
    "LANDING_TARGET": "_ZN10AP_Landing28get_target_altitude_locationER8Location",
    "GLIDE_SLOPE": "_ZN10AP_Landing25setup_landing_glide_slopeERK8LocationS2_S2_Ri",
    "LOITER_TARGET": "_ZN5Plane21reached_loiter_targetEv",
    "RESET_OFFSET": "_ZN5Plane21reset_offset_altitudeEv",
    "SET_LOCATION": "_ZN5Plane28set_target_altitude_locationERK8Location",
    "SET_PROPORTION": "_ZN5Plane30set_target_altitude_proportionERK8Locationf",
    "CONSTRAIN": "_ZN5Plane34constrain_target_altitude_locationERK8LocationS2_",
    "TERRAIN": "_ZN5Plane38set_target_altitude_proportion_terrainEv",
    "PAST_FINISH": "_ZNK8Location25past_interval_finish_lineERKS_S1_",
    # The approach branch also calls this, and it reads rangefinder
    # state a harness has not brought up.
    "BUMP": "_ZN10AP_Landing41adjust_landing_slope_for_rangefinder_bumpERN12AP_FixedWing17Rangefinder_StateER8LocationS4_RKS3_fRi",
}

HARNESS = r'''
#include <AP_HAL/AP_HAL.h>

#define private public
#define protected public
#include "/srv/ardumaster/upstream/plane-4.7.0/ArduPlane/Plane.h"
#undef private
#undef protected

#include <cstdarg>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

namespace AP_HAL {
void panic(const char *errormsg, ...)
{
    va_list ap;
    va_start(ap, errormsg);
    vfprintf(stderr, errormsg, ap);
    va_end(ap);
    fprintf(stderr, "\n");
    abort();
}
}

extern const AP_HAL::HAL &hal;

// ---- injected predicates ----
static bool g_flaring = false;
static bool g_on_approach = false;
static bool g_landing_target = false;
static bool g_loiter_reached = false;
static bool g_terrain_ok = false;
static bool g_past_finish = false;

// ---- observations ----
static int g_glide_slope = 0;
static int g_reset_offset = 0;
static int g_set_location = 0;
static int32_t g_set_location_alt = 0;
static int g_set_proportion = 0;
static int g_constrain = 0;
static int g_terrain_calls = 0;

extern "C" bool __wrap__ZNK10AP_Landing10is_flaringEv(const void *s);
extern "C" bool __wrap__ZNK10AP_Landing10is_flaringEv(const void *s)
{ (void)s; return g_flaring; }

extern "C" bool __wrap__ZNK10AP_Landing14is_on_approachEv(const void *s);
extern "C" bool __wrap__ZNK10AP_Landing14is_on_approachEv(const void *s)
{ (void)s; return g_on_approach; }

extern "C" bool __wrap__ZN10AP_Landing28get_target_altitude_locationER8Location(
    void *s, Location &loc);
extern "C" bool __wrap__ZN10AP_Landing28get_target_altitude_locationER8Location(
    void *s, Location &loc)
{
    (void)s;
    if (!g_landing_target) {
        return false;
    }
    loc = plane.current_loc;
    loc.set_alt_cm(4242, Location::AltFrame::ABSOLUTE);
    return true;
}

extern "C" void __wrap__ZN10AP_Landing25setup_landing_glide_slopeERK8LocationS2_S2_Ri(
    void *s, const Location &a, const Location &b, const Location &c, int &d);
extern "C" void __wrap__ZN10AP_Landing25setup_landing_glide_slopeERK8LocationS2_S2_Ri(
    void *s, const Location &a, const Location &b, const Location &c, int &d)
{ (void)s; (void)a; (void)b; (void)c; (void)d; g_glide_slope++; }

extern "C" bool __wrap__ZN5Plane21reached_loiter_targetEv(void *s);
extern "C" bool __wrap__ZN5Plane21reached_loiter_targetEv(void *s)
{ (void)s; return g_loiter_reached; }

extern "C" void __wrap__ZN5Plane21reset_offset_altitudeEv(void *s);
extern "C" void __wrap__ZN5Plane21reset_offset_altitudeEv(void *s)
{ (void)s; g_reset_offset++; }

extern "C" void __wrap__ZN5Plane28set_target_altitude_locationERK8Location(
    void *s, const Location &loc);
extern "C" void __wrap__ZN5Plane28set_target_altitude_locationERK8Location(
    void *s, const Location &loc)
{
    (void)s;
    g_set_location++;
    // The altitude, so the landing-target branch is distinguishable from the
    // next-waypoint branches. Counting calls alone cannot tell them apart.
    g_set_location_alt = loc.alt;
}

extern "C" void __wrap__ZN5Plane30set_target_altitude_proportionERK8Locationf(
    void *s, const Location &loc, float p);
extern "C" void __wrap__ZN5Plane30set_target_altitude_proportionERK8Locationf(
    void *s, const Location &loc, float p)
{ (void)s; (void)loc; (void)p; g_set_proportion++; }

extern "C" void __wrap__ZN5Plane34constrain_target_altitude_locationERK8LocationS2_(
    void *s, const Location &a, const Location &b);
extern "C" void __wrap__ZN5Plane34constrain_target_altitude_locationERK8LocationS2_(
    void *s, const Location &a, const Location &b)
{ (void)s; (void)a; (void)b; g_constrain++; }

extern "C" bool __wrap__ZN5Plane38set_target_altitude_proportion_terrainEv(void *s);
extern "C" bool __wrap__ZN5Plane38set_target_altitude_proportion_terrainEv(void *s)
{ (void)s; g_terrain_calls++; return g_terrain_ok; }

extern "C" void __wrap__ZN10AP_Landing41adjust_landing_slope_for_rangefinder_bumpERN12AP_FixedWing17Rangefinder_StateER8LocationS4_RKS3_fRi(
    void *s, void *rf, Location &a, Location &b, const Location &c,
    float d, int &e);
extern "C" void __wrap__ZN10AP_Landing41adjust_landing_slope_for_rangefinder_bumpERN12AP_FixedWing17Rangefinder_StateER8LocationS4_RKS3_fRi(
    void *s, void *rf, Location &a, Location &b, const Location &c,
    float d, int &e)
{ (void)s; (void)rf; (void)a; (void)b; (void)c; (void)d; (void)e; }

extern "C" bool __wrap__ZNK8Location25past_interval_finish_lineERKS_S1_(
    const void *s, const Location &a, const Location &b);
extern "C" bool __wrap__ZNK8Location25past_interval_finish_lineERKS_S1_(
    const void *s, const Location &a, const Location &b)
{ (void)s; (void)a; (void)b; return g_past_finish; }

int main()
{
    AP::scheduler().init(nullptr, 0, 0);
    setvbuf(stdout, NULL, _IOLBF, 0);

    Mode *mode = (Mode *)&plane.mode_fbwa;

    printf("#target_altitude\n");
    printf("idx,flaring,on_approach,landing_target,soaring,loiter_reached,"
           "terrain_alt,terrain_ok,offset_cm,past_finish,"
           "glide_slope,reset_offset,set_location,set_proportion,constrain,"
           "terrain_calls,set_location_alt\n");

    const int32_t offsets[] = {0, -1500, 2500};

    int idx = 0;
    for (unsigned fl = 0; fl < 2; fl++)
      for (unsigned ap = 0; ap < 2; ap++)
        for (unsigned lt = 0; lt < 2; lt++)
          for (unsigned lo = 0; lo < 2; lo++)
            for (unsigned ta = 0; ta < 2; ta++)
              for (unsigned tok = 0; tok < 2; tok++)
                for (unsigned off = 0; off < 3; off++)
                  for (unsigned pf = 0; pf < 2; pf++) {
                      g_flaring = (fl != 0);
                      g_on_approach = (ap != 0);
                      g_landing_target = (lt != 0);
                      g_loiter_reached = (lo != 0);
                      g_terrain_ok = (tok != 0);
                      g_past_finish = (pf != 0);

                      plane.next_WP_loc.terrain_alt = (ta != 0);
                      // A distinctive altitude on the waypoint, different
                      // from the 4242 the landing wrapper reports, so the two
                      // set_target_altitude_location branches are separable.
                      plane.next_WP_loc.alt = 7700;
                      plane.target_altitude.offset_cm = offsets[off];

                      g_glide_slope = 0;
                      g_reset_offset = 0;
                      g_set_location = 0;
                      g_set_proportion = 0;
                      g_constrain = 0;
                      g_terrain_calls = 0;
                      g_set_location_alt = -1;

                      mode->update_target_altitude();

                      printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
                             "%d,%d,%d,%d,%d,%d,%d\n",
                             idx++,
                             (int)g_flaring, (int)g_on_approach,
                             (int)g_landing_target, 0,
                             (int)g_loiter_reached,
                             (int)(ta != 0), (int)g_terrain_ok,
                             (int)offsets[off], (int)g_past_finish,
                             g_glide_slope, g_reset_offset, g_set_location,
                             g_set_proportion, g_constrain, g_terrain_calls,
                             (int)g_set_location_alt);
                  }

    fflush(stdout);
    _exit(0);
}
'''


def main():
    objects = plane_link.objects(stage_dir="/tmp/plane_alt_parity/vehicle")
    flags = list(plane_link.LINK_FLAGS) + [
        "-Wl,--wrap=" + sym for sym in WRAPS.values()
    ]
    build(HARNESS, objects, BUILD, "ArduPlane/Plane.cpp", link_flags=flags)
    text = run(BUILD)
    OUT.write_text(text)
    rows = sum(1 for l in text.splitlines()
               if l and not l.startswith("#") and not l[0].isalpha())
    print("wrote %s: %d rows" % (OUT.name, rows))


main()
