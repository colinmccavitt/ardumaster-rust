#!/usr/bin/env python3
"""Parity fixture for AP_Param's on-storage format (FW-004 slice 1, ADR-0010).

Byte compatibility is the whole point of ADR-0010, and the layout depends on
how the compiler packs a bitfield:

    uint32_t key_low : 8;
    uint32_t type    : 5;
    uint32_t key_high: 1;
    uint32_t group_element : 18;

Bitfield allocation order is implementation-defined, so reading the C++ does
not settle it. This has upstream's own compiled code build the headers and
dumps the raw words, so the port is compared against what the firmware
actually writes rather than against an assumption about GCC.

`#define private public` is used to reach `get_key`, `set_key` and
`is_sentinel`, which are private statics. That is a blunt instrument, but the
alternative -- redeclaring the struct in the harness -- would measure the
compiler's packing of MY declaration rather than upstream's, which is exactly
the thing in question.
"""
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import parity_build  # noqa: E402

OUT = Path("/srv/ardumaster/ports/ardumaster-rust/fixtures/param_header.csv")
CONSTS = Path("/srv/ardumaster/ports/ardumaster-rust/fixtures/param_format.csv")

KEYS = [0, 1, 2, 127, 128, 255, 256, 257, 383, 511]
TYPES = [0, 1, 2, 3, 4, 5, 6, 0x1F]
GROUPS = [0, 1, 63, 64, 255, 4095, 65535, 262143]
# words fed straight to is_sentinel, to cover the power-off fill values
RAW = [0x00000000, 0xFFFFFFFF, 0x003FFFFF, 0x00000001, 0x12345678]

HARNESS = r"""
#define private public
#define protected public
#include <AP_Param/AP_Param.h>
#undef private
#undef protected

#include <stdio.h>
#include <string.h>

static const uint16_t keys[] = {%(keys)s};
static const uint8_t  types[] = {%(types)s};
static const uint32_t groups[] = {%(groups)s};
static const uint32_t raws[] = {%(raws)s};

int main()
{
    // --- the format's fixed points ---
    printf("const,eeprom_header_size,%%u\n", (unsigned)sizeof(AP_Param::EEPROM_header));
    printf("const,param_header_size,%%u\n", (unsigned)sizeof(AP_Param::Param_header));
    printf("const,magic0,%%u\n", (unsigned)AP_Param::k_EEPROM_magic0);
    printf("const,magic1,%%u\n", (unsigned)AP_Param::k_EEPROM_magic1);
    printf("const,revision,%%u\n", (unsigned)AP_Param::k_EEPROM_revision);
    printf("const,sentinel_key,%%u\n", (unsigned)AP_Param::_sentinel_key);
    printf("const,sentinel_type,%%u\n", (unsigned)AP_Param::_sentinel_type);
    printf("const,sentinel_group,%%u\n", (unsigned)AP_Param::_sentinel_group);
    printf("const,group_level_shift,%%u\n", (unsigned)AP_Param::_group_level_shift);
    printf("const,group_bits,%%u\n", (unsigned)AP_Param::_group_bits);

    // the header write_sentinel() builds, constructed the same way but without
    // touching storage
    {
        struct AP_Param::Param_header phdr;
        memset(&phdr, 0, sizeof(phdr));
        phdr.type = AP_Param::_sentinel_type;
        AP_Param::set_key(phdr, AP_Param::_sentinel_key);
        phdr.group_element = AP_Param::_sentinel_group;
        uint32_t w; memcpy(&w, &phdr, 4);
        printf("const,sentinel_word,%%u\n", (unsigned)w);
    }

    // --- type sizes ---
    for (unsigned t = 0; t <= 6; t++) {
        printf("size,%%u,%%u\n", t, (unsigned)AP_Param::type_size((enum ap_var_type)t));
    }

    // --- header encoding ---
    for (unsigned ki = 0; ki < sizeof(keys)/sizeof(keys[0]); ki++) {
        for (unsigned ti = 0; ti < sizeof(types)/sizeof(types[0]); ti++) {
            for (unsigned gi = 0; gi < sizeof(groups)/sizeof(groups[0]); gi++) {
                struct AP_Param::Param_header phdr;
                memset(&phdr, 0, sizeof(phdr));
                phdr.type = types[ti];
                AP_Param::set_key(phdr, keys[ki]);
                phdr.group_element = groups[gi];
                uint32_t w; memcpy(&w, &phdr, 4);
                printf("hdr,%%u,%%u,%%u,%%u,%%u,%%u\n",
                       (unsigned)keys[ki], (unsigned)types[ti], (unsigned)groups[gi],
                       (unsigned)w,
                       (unsigned)AP_Param::get_key(phdr),
                       (unsigned)AP_Param::is_sentinel(phdr));
            }
        }
    }

    // --- is_sentinel on raw words, including the storage fill values ---
    for (unsigned i = 0; i < sizeof(raws)/sizeof(raws[0]); i++) {
        struct AP_Param::Param_header phdr;
        memcpy(&phdr, &raws[i], 4);
        printf("raw,%%u,%%u,%%u\n",
               (unsigned)raws[i],
               (unsigned)AP_Param::get_key(phdr),
               (unsigned)AP_Param::is_sentinel(phdr));
    }
    return 0;
}
""" % {
    "keys": ",".join(str(k) for k in KEYS),
    "types": ",".join(str(t) for t in TYPES),
    "groups": ",".join(str(g) for g in GROUPS),
    "raws": ",".join("0x%08X" % r for r in RAW),
}

out_dir = Path("/tmp/parity_param")
# Only AP_Param's own object, not the whole archive: AP_Param.cpp references
# the filesystem, the GCS and the HAL, and satisfying those from the archive
# drags in the vehicle and then the Lua bindings. The four functions called
# here -- get_key, set_key, is_sentinel, type_size -- touch none of it.
AP_PARAM_O = [
    "build/sitl/libraries/AP_Param/AP_Param.cpp.0.o",
    # AP_Param has a file-scope HAL_Semaphore whose constructor runs before
    # main, so its vtable has to be real rather than dangling.
    "build/sitl/libraries/AP_HAL_SITL/Semaphores.cpp.0.o",
    # and file-scope StorageAccess objects, whose constructors reach into
    # StorageManager tables
    "build/sitl/libraries/StorageManager/StorageManager.cpp.4.o",
    # and the save queue, an ObjectBuffer whose constructor builds a ByteBuffer
    "build/sitl/libraries/AP_HAL/utility/RingBuffer.cpp.0.o",
]
binary = parity_build.build(
    HARNESS,
    AP_PARAM_O,
    out_dir / "param_header",
    "AP_Param.cpp",
    link_flags=["-Wl,--unresolved-symbols=ignore-all"],
)
text = parity_build.run(binary)

consts, sizes, hdrs, raws = [], [], [], []
for line in text.splitlines():
    if not line.strip():
        continue
    f = line.split(",")
    if f[0] == "const":
        consts.append((f[1], int(f[2])))
    elif f[0] == "size":
        sizes.append((int(f[1]), int(f[2])))
    elif f[0] == "hdr":
        hdrs.append([int(x) for x in f[1:]])
    elif f[0] == "raw":
        raws.append([int(x) for x in f[1:]])

OUT.parent.mkdir(parents=True, exist_ok=True)
with open(OUT, "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["key", "type", "group_element", "word", "get_key", "is_sentinel"])
    for r in hdrs:
        w.writerow(r)
print("wrote %s (%d header encodings)" % (OUT.name, len(hdrs)))

with open(CONSTS, "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["kind", "name", "value"])
    for name, v in consts:
        w.writerow(["const", name, v])
    for t, sz in sizes:
        w.writerow(["type_size", t, sz])
    for r in raws:
        w.writerow(["is_sentinel_raw", r[0], "%d;%d" % (r[1], r[2])])
print("wrote %s (%d constants, %d type sizes, %d raw words)"
      % (CONSTS.name, len(consts), len(sizes), len(raws)))

for name, v in consts:
    print("  %-20s %d (0x%X)" % (name, v, v))
