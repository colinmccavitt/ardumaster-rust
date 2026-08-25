#!/usr/bin/env python3
r"""Dump the vehicle's parameter tables, reference build only (FW-004 slice 2).

Slice 2 ports the descriptor tables and the name/key/group_element mapping, and
that mapping cannot be tested without the tables it maps over. They live in the
vehicle -- Parameters.cpp plus every library's var_info -- and are only assembled
at runtime, so a linked harness would have to link the whole firmware. Same
answer as ADR-0008: have the reference build tell us what it has.

Two sections, and the difference between them is what makes the test honest:

  V / G lines  the STRUCTURE: a recursive walk of var_info emitting each entry's
               name, idx, type and flags, and its position in the tree. This is
               the input the port is given.

  P lines      the RESULT: upstream's own first()/next() enumeration, giving the
               full name, key, idx and group_element of every parameter. This is
               the oracle the port has to reproduce.

Generating the port's tables from the flattened enumeration would make the test
circular -- it would check that a lookup finds what was put in it. Generating
them from the structure and comparing against the enumeration tests the
traversal and the group_id encoding, which is the part that actually decides
where a parameter is stored.

The structure walk lives inside load_all() rather than in a free function
because _num_vars and var_info() are private and this patch must not touch the
header. The group recursion is a free function, because GroupInfo is public.

Gated on the AP_PARAM_DUMP environment variable and exits immediately after, so
the binary can be run directly without flying anything.

REFERENCE BUILD ONLY, never the port.
"""
import argparse
import sys
from pathlib import Path

TARGET = Path("/srv/ardumaster/upstream/plane-4.7.0/libraries/AP_Param/AP_Param.cpp")

ANCHOR = """bool AP_Param::load_all()
{"""

PATCH = r'''// ---- reference-build-only: dump the parameter tables for FW-004 ----
static void ap_param_dump_group(const struct AP_Param::GroupInfo *gi,
                                const char *path, uint8_t depth)
{
    if (gi == nullptr || depth > 4) {
        return;
    }
    for (uint8_t i = 0; gi[i].type != AP_PARAM_NONE; i++) {
        char sub[64];
        snprintf(sub, sizeof(sub), "%s.%u", path, (unsigned)i);
        printf("G,%s,%u,%u,%u,%s\n",
               path,
               (unsigned)gi[i].idx,
               (unsigned)gi[i].type,
               (unsigned)gi[i].flags,
               gi[i].name ? gi[i].name : "");
        if (gi[i].type == AP_PARAM_GROUP) {
            const struct AP_Param::GroupInfo *child;
            if (gi[i].flags & AP_PARAM_FLAG_INFO_POINTER) {
                child = gi[i].group_info_ptr ? *gi[i].group_info_ptr : nullptr;
            } else {
                child = gi[i].group_info;
            }
            ap_param_dump_group(child, sub, depth + 1);
        }
    }
}
// ---- end reference-build-only ----

bool AP_Param::load_all()
{
    // ---- reference-build-only ----
    if (getenv("AP_PARAM_DUMP") != nullptr) {
        printf("F,%u\n", (unsigned)_frame_type_flags);
        printf("BEGIN_STRUCTURE\n");
        for (uint16_t i = 0; i < _num_vars; i++) {
            const struct Info &info = var_info(i);
            char path[16];
            snprintf(path, sizeof(path), "%u", (unsigned)i);
            printf("V,%u,%u,%u,%u,%s\n",
                   (unsigned)i,
                   (unsigned)info.key,
                   (unsigned)info.type,
                   (unsigned)info.flags,
                   info.name ? info.name : "");
            if (info.type == AP_PARAM_GROUP) {
                const struct GroupInfo *child;
                if (info.flags & AP_PARAM_FLAG_INFO_POINTER) {
                    child = info.group_info_ptr ? *info.group_info_ptr : nullptr;
                } else {
                    child = info.group_info;
                }
                ap_param_dump_group(child, path, 1);
            }
        }
        printf("END_STRUCTURE\n");

        printf("BEGIN_PARAMS\n");
        ParamToken token;
        enum ap_var_type type;
        float def = 0;
        for (AP_Param *ap = first(&token, &type, &def);
             ap != nullptr;
             ap = next(&token, &type, false, &def)) {
            char name[AP_MAX_NAME_SIZE + 1];
            name[AP_MAX_NAME_SIZE] = 0;
            ap->copy_name_token(token, name, AP_MAX_NAME_SIZE, true);
            printf("P,%s,%u,%u,%u,%u,%.9g\n",
                   name,
                   (unsigned)token.key,
                   (unsigned)token.idx,
                   (unsigned)token.group_element,
                   (unsigned)type,
                   def);
        }
        printf("END_PARAMS\n");
        fflush(stdout);
        exit(0);
    }
    // ---- end reference-build-only ----
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--revert", action="store_true")
    args = ap.parse_args()

    if not TARGET.exists():
        sys.exit("target not found")
    text = TARGET.read_text()

    if args.revert:
        if PATCH not in text:
            print("param dump not applied")
            return
        TARGET.write_text(text.replace(PATCH, ANCHOR))
        print("reverted the param dump")
        return

    if PATCH in text:
        print("param dump already applied")
        return
    if text.count(ANCHOR) != 1:
        sys.exit("anchor matched %d times, expected 1" % text.count(ANCHOR))

    text = text.replace(ANCHOR, PATCH, 1)
    if "#include <stdlib.h>" not in text:
        marker = '#include "AP_Param.h"\n'
        if marker not in text:
            sys.exit("include anchor not found")
        text = text.replace(marker, marker + "#include <stdlib.h>\n", 1)

    TARGET.write_text(text)
    print("applied the parameter table dump")


if __name__ == "__main__":
    main()
