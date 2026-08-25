"""Generate the Rust multirotor frame tables from upstream's C++.

The tables are 700 lines of angles across seven `setup_*_matrix` functions, in
three call shapes: a `MotorDef` array (angle, yaw, order), a `MotorDefRaw`
array (roll, pitch, yaw, order), and bare five-argument `add_motor` calls that
give roll and pitch as separate angles. Transcribing that by hand is exactly
the kind of job where one transposed digit hides for years, so it is generated
here and then checked against `fixtures/motors_frames.csv`, which was measured
from the compiled object rather than read off the source.

The generator does not have to be airtight. It has to be *checkable*: anything
it gets wrong shows up as a bit-exact mismatch in the parity test.

Some frame types sit behind `#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)` -- they
belong to quadplanes, not to Copter. Those are excluded here and listed
separately, because the fixture is a Copter build and would flag them as frames
the port invents.
"""
import json
import re
import sys
from pathlib import Path

SRC = Path("/srv/ardumaster/upstream/plane-4.7.0/libraries/AP_Motors/AP_MotorsMatrix.cpp")

CLASSES = [
    (1, "Quad", "setup_quad_matrix"),
    (2, "Hexa", "setup_hexa_matrix"),
    (3, "Octa", "setup_octa_matrix"),
    (4, "OctaQuad", "setup_octaquad_matrix"),
    (5, "Y6", "setup_y6_matrix"),
    (12, "DodecaHexa", "setup_dodecahexa_matrix"),
    (14, "Deca", "setup_deca_matrix"),
]

YAW = {
    "AP_MOTORS_MATRIX_YAW_FACTOR_CW": "-1.0",
    "AP_MOTORS_MATRIX_YAW_FACTOR_CCW": "1.0",
}

MOT = re.compile(r"AP_MOTORS_MOT_(\d+)")

TABLE = re.compile(
    r"static\s+const\s+(?:AP_MotorsMatrix::)?(MotorDef|MotorDefRaw)\s+\w+\s*\[\]"
    r"\s*=?\s*\{(.*?)\n\s*\};",
    re.S,
)

plane_only = []


def num(tok):
    """Render a C numeric or named constant as a Rust f32 literal."""
    tok = tok.strip()
    if tok in YAW:
        return YAW[tok]
    m = MOT.fullmatch(tok)
    if m:
        return str(int(m.group(1)) - 1)  # MOT_1 is motor index 0
    if not re.fullmatch(r"[-+]?[0-9]*\.?[0-9]+f?", tok):
        raise ValueError("unrecognised token %r" % tok)
    tok = tok.rstrip("f").lstrip("+")
    return tok if "." in tok else tok + ".0"


def body_of(text, fn):
    start = text.find("AP_MotorsMatrix::%s(" % fn)
    if start < 0:
        return None
    open_brace = text.index("{", start)
    depth = 0
    for i in range(open_brace, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[open_brace + 1 : i]
    raise ValueError("unbalanced braces in %s" % fn)


def preprocess(body, fn):
    """Resolve the conditionals the way an ArduCopter build resolves them.

    `AP_MOTORS_FRAME_*_ENABLED` are all on in the reference build, so those
    blocks are kept. `APM_BUILD_TYPE(APM_BUILD_ArduPlane)` is off, so those are
    dropped -- and recorded, since a quadplane port will need them.
    """
    out = []
    stack = []  # True = emitting
    for line in body.splitlines():
        s = line.lstrip()
        if s.startswith("#if"):
            cond = s
            keep = "APM_BUILD_ArduPlane" not in cond
            stack.append(keep)
            continue
        if s.startswith("#else"):
            if stack:
                stack[-1] = not stack[-1]
            continue
        if s.startswith("#endif"):
            if stack:
                stack.pop()
            continue
        if s.startswith("#"):
            continue
        if all(stack):
            out.append(line)
        else:
            m = re.search(r"case\s+MOTOR_FRAME_TYPE_(\w+)\s*:", line)
            if m:
                plane_only.append("%s/%s" % (fn, m.group(1)))
    return "\n".join(out)


def parse_cases(body):
    """Split into (frame_type_names, case_body). Bare labels share the next body.

    `default:` is carried through as the pseudo-name DEFAULT. Only Y6 has a
    productive one -- every other class returns false there, which parses to no
    motors and drops out -- but a class that grows one later must not be missed
    silently.
    """
    parts = re.split(r"\b(case\s+MOTOR_FRAME_TYPE_\w+|default)\s*:", body)
    out = []
    pending = []
    for label, chunk in zip(parts[1::2], parts[2::2]):
        m = re.match(r"case\s+MOTOR_FRAME_TYPE_(\w+)", label)
        pending.append(m.group(1) if m else "DEFAULT")
        if chunk.strip().strip("{").strip():
            out.append((pending, chunk))
            pending = []
    return out


# Frame-local post-processing. Two co-rotating X8 frames scale their top layer
# so the two rotor layers do not beat against each other; the constant lives in
# AP_Motors_config.h and is a double, which matters for the arithmetic.
SCALE = re.compile(
    r"for\s*\(uint8_t\s+i\s*=\s*0;\s*i\s*<\s*(\d+);\s*"
    r"(?:i\+\+|i\s*\+=\s*(\d+))\)\s*\{"
    r"[^}]*?_roll_factor\[i\]\s*\*=\s*(\w+);",
    re.S,
)
CONSTS = {"AP_MOTORS_FRAME_OCTAQUAD_COROTATING_SCALE_FACTOR": "0.9"}


def parse_scale(chunk):
    """Return (limit, step, value) if this frame scales part of its motors.

    The two co-rotating frames disagree about which motors are the top layer:
    X_COR takes the first four, CW_X_COR takes every other one of eight. Both
    are the same loop with a different stride, so the stride is carried rather
    than assumed.
    """
    m = SCALE.search(chunk)
    if not m:
        return None
    name = m.group(3)
    if name not in CONSTS:
        raise ValueError("unknown scale constant %r" % name)
    step = int(m.group(2)) if m.group(2) else 1
    return [int(m.group(1)), step, CONSTS[name]]


def parse_case(chunk):
    """Return a list of (kind, args) for one frame type."""
    motors = []
    tbl = TABLE.search(chunk)
    if tbl:
        kind = tbl.group(1)
        for row in re.finditer(r"\{([^{}]*)\}", tbl.group(2)):
            fields = [f for f in row.group(1).split(",") if f.strip()]
            if kind == "MotorDef":
                assert len(fields) == 3, fields
                motors.append(("angle", [num(f) for f in fields]))
            else:
                assert len(fields) == 4, fields
                motors.append(("raw", [num(f) for f in fields]))
        return motors

    for call in re.finditer(r"add_motor\(([^;]*?)\);", chunk, re.S):
        fields = [f.strip() for f in call.group(1).split(",")]
        assert len(fields) == 5, fields
        motors.append(("rollpitch", [num(f) for f in fields]))
    return motors


def main():
    text = SRC.read_text()
    blocks = []

    for cls_num, cls_name, fn in CLASSES:
        body = body_of(text, fn)
        if body is None:
            print("MISSING: %s (%s)" % (cls_name, fn), file=sys.stderr)
            continue
        entries = []
        for names, chunk in parse_cases(preprocess(body, fn)):
            motors = parse_case(chunk)
            if motors:
                entries.append((names, motors, parse_scale(chunk)))
        blocks.append((cls_num, cls_name, entries))

    total = 0
    for cls_num, cls_name, entries in blocks:
        n = sum(len(e[1]) for e in entries)
        names = sum(len(e[0]) for e in entries)
        print("class %-11s (%2d): %2d bodies, %2d frame types, %3d motors"
              % (cls_name, cls_num, len(entries), names, n))
        total += n
    print("total motor entries: %d" % total)
    if plane_only:
        print("excluded (ArduPlane-only): %s" % ", ".join(plane_only))

    Path("/tmp/frames.json").write_text(json.dumps(blocks, indent=1))
    print("wrote /tmp/frames.json")


main()
