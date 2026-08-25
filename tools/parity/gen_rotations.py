#!/usr/bin/env python3
"""Emit the Rotation enum and Vector3::rotate from upstream's own source.

49 rotations over a 246-line switch. Hand-transcribing coordinate permutations
is the kind of work where one swapped sign yields a rotation that is still
orthonormal, still length-preserving, and simply the wrong one. So the switch is
extracted rather than retyped, and every case is then checked against upstream
exhaustively.

The delicate part is float precision, and it is not what the C standard alone
implies. ArduPilot compiles with **-fsingle-precision-constant**, so every
floating literal in the source is a `float` rather than a `double`. There is
therefore no promotion to double anywhere in this switch: the literals are
single precision, the components are the element type, and the whole expression
evaluates in the element type.

Read without that flag, `HALF_SQRT_2 * (ftype)(x - y)` looks like a double
multiply narrowed once on assignment, and an earlier version of this generator
emitted exactly that. 43 of the 44 rotations agreed anyway -- single rounding
and double rounding usually land on the same float -- and
ROTATION_ROLL_90_PITCH_68_YAW_293 disagreed in the last bit. That one case is
how the flag was found. It is a good argument for bit-exact parity over a
tolerance: at any tolerance loose enough to be comfortable, the wrong model
would have passed.

The `(ftype)` casts are dropped, since `ftype` and the element type are both
float in the reference build.

Not generated: `ROTATION_CUSTOM_1/2` delegate to `AP_CustomRotations`, which is
not ported, and `ROTATION_MAX/CUSTOM_OLD/CUSTOM_END` are enum bookkeeping
rather than orientations. Both are rejected rather than silently ignored.
"""
import re
from pathlib import Path

UP = Path("/srv/ardumaster/upstream/plane-4.7.0/libraries/AP_Math")
OUT = Path("/srv/ardumaster/ports/plane-fw-rust/crates/ap-math/src/rotations_gen.rs")

HALF_SQRT_2 = "0.70710678118654752440084436210485"

NON_ROTATIONS = {"ROTATION_MAX", "ROTATION_CUSTOM_OLD", "ROTATION_CUSTOM_END"}
CUSTOM = {"ROTATION_CUSTOM_1", "ROTATION_CUSTOM_2"}

VAR = re.compile(r"\b(x|y|z|tmp|tmpx|tmpy|tmpz|sin_pitch|cos_pitch)\b")
LITERAL = re.compile(r"(?<![\w.])\d+\.\d+(?![\w.])")
CAST = re.compile(r"\((?:ftype|float)\)\s*\(([^()]*)\)")


def camel(name):
    """ROTATION_ROLL_90_PITCH_68_YAW_293 -> Roll90Pitch68Yaw293"""
    return "".join(
        p if p.isdigit() else p.capitalize()
        for p in name[len("ROTATION_"):].split("_")
    )


def tvar(m):
    """x/y/z are fields of the vector; everything else is a local."""
    n = m.group(1)
    return "v." + n if n in ("x", "y", "z") else n


def parse_enum():
    text = (UP / "rotations.h").read_text()
    body = text[text.index("enum Rotation"):]
    body = body[: body.index("};")]
    # ROTATION_MAX and ROTATION_CUSTOM_END carry no explicit value, so they take
    # the previous one plus 1 -- 44 and 103.
    out, nxt = [], 0
    for m in re.finditer(r"(ROTATION_\w+)\s*(?:=\s*(\d+))?\s*,", body):
        v = int(m.group(2)) if m.group(2) is not None else nxt
        out.append((m.group(1), v))
        nxt = v + 1
    return out


def parse_switch():
    text = (UP / "vector3.cpp").read_text()
    i = text.index("void Vector3<T>::rotate(enum Rotation rotation)")
    body = text[i: text.index("\n}", i)]

    cases, labels, stmts = [], [], []
    for raw in body.split("\n"):
        line = raw.strip()
        if line.startswith("case "):
            if stmts:
                cases.append((labels, stmts))
                labels, stmts = [], []
            labels.append(line[len("case "):].split(":")[0].strip())
            continue
        if not labels:
            continue
        if line in ("return;", "break;"):
            cases.append((labels, stmts))
            labels, stmts = [], []
            continue
        if line in ("{", "}", "") or line.startswith("//") or line.startswith("#"):
            continue
        stmts.append(line)
    if labels:
        cases.append((labels, stmts))
    return cases


def rust_expr(expr):
    """Translate one C++ right-hand side.

    ArduPilot builds with **-fsingle-precision-constant**, so every floating
    literal in the source is a `float`, not a `double`. There is therefore no
    promotion to double anywhere in this switch: the literals are single
    precision, the vector components are `T`, and the whole expression
    evaluates in `T`.

    That is worth stating because it is not what the C standard alone implies.
    Reading the source without the flag suggests `HALF_SQRT_2 * (ftype)(x - y)`
    multiplies in double and narrows once, and an earlier version of this
    generator emitted exactly that. It disagreed with upstream in the last bit
    on ROTATION_ROLL_90_PITCH_68_YAW_293, which is how the flag was found.

    The `(ftype)` casts are dropped: with `ftype` and `T` both float they are
    no-ops. See the module docs for the one case where that is not true.
    """
    e = expr.replace("HALF_SQRT_2", HALF_SQRT_2)
    e = CAST.sub(lambda m: "(" + m.group(1) + ")", e)
    e = LITERAL.sub(lambda m: "T::from_f64(%s)" % m.group(0), e)
    e = VAR.sub(tvar, e)
    return " ".join(e.split())


def rust_stmts(stmts):
    out = []
    for s in stmts:
        # `const T sin_pitch = 0.12...; // sinF(pitch);` -- the trailing comment
        # would otherwise split into a statement of its own
        for part in [p.strip() for p in s.split("//")[0].split(";") if p.strip()]:
            m = re.match(r"^const\s+T\s+(\w+)\s*=\s*(.+)$", part)
            if m:
                # shape 3: narrowed to the element type before any arithmetic
                out.append("let %s = T::from_f64(%s);" % (m.group(1), m.group(2).strip()))
                continue
            m = re.match(r"^T\s+(\w+)\s*=\s*(.+)$", part)
            if m:
                out.append("let %s = %s;" % (m.group(1), rust_expr(m.group(2))))
                continue
            m = re.match(r"^(\w+)\s*=\s*(.+)$", part)
            if m:
                lhs, rhs = m.group(1), rust_expr(m.group(2))
                if lhs in ("x", "y", "z"):
                    out.append("v.%s = %s;" % (lhs, rhs))
                else:
                    # `tmp` is reassigned within an arm; shadowing is fine
                    out.append("let %s = %s;" % (lhs, rhs))
                continue
            raise SystemExit("unhandled statement: %r" % part)
    return out


def main():
    enum = parse_enum()
    cases = parse_switch()
    known = {n for n, _ in enum}
    for labels, _ in cases:
        for l in labels:
            if l not in known:
                raise SystemExit("case %s is not in the enum" % l)

    L = [
        "//! The `Rotation` enum and `Vector3::rotate`, ported from",
        "//! `AP_Math/rotations.h` and `AP_Math/vector3.cpp`.",
        "//!",
        "//! GENERATED by `tools/parity/gen_rotations.py` -- do not edit. 49",
        "//! rotations over a 246-line switch: one swapped sign would give a",
        "//! rotation that is still orthonormal, still length-preserving, and",
        "//! simply the wrong one. Every case is checked against upstream",
        "//! exhaustively by `tests/rotations_parity.rs`.",
        "//!",
        "//! Upstream compiles with `-fsingle-precision-constant`, so the literals",
        "//! here are single precision and the whole switch evaluates in the",
        "//! element type -- there is no promotion to double, despite what the",
        "//! C source looks like. See the generator's docstring; getting this",
        "//! wrong changed exactly one rotation, in its last bit.",
        "",
        "#![allow(",
        "    clippy::excessive_precision,",
        "    reason = \"upstream's literals, reproduced verbatim; trimming them \\",
        "would be editing upstream's source\"",
        ")]",
        "#![allow(",
        "    clippy::approx_constant,",
        "    reason = \"HALF_SQRT_2 is near FRAC_1_SQRT_2 but is not it; \\",
        "substituting the named constant changes the last bits and breaks the \\",
        "bit-exact parity this module is verified against\"",
        ")]",
        "#![allow(",
        "    clippy::manual_swap,",
        "    reason = \"this file corresponds statement for statement with \\",
        "upstream's switch so the two can be diffed; mem::swap would be \\",
        "equivalent but would break that correspondence\"",
        ")]",
        "",
        "use crate::scalar::Real;",
        "use crate::vector3::Vector3;",
        "",
        "/// Sensor orientation, upstream `enum Rotation`.",
        "///",
        "/// Discriminants are upstream's: drivers and parameters carry the raw",
        "/// value, and MAVLink's `MAV_SENSOR_ORIENTATION` is expected to match.",
        "#[derive(Debug, Clone, Copy, PartialEq, Eq)]",
        "#[repr(u8)]",
        "pub enum Rotation {",
    ]
    for name, val in enum:
        L.append("    /// Upstream `%s`." % name)
        L.append("    %s = %d," % (camel(name), val))
    L += ["}", "", "impl Rotation {"]
    L += [
        "    /// From the raw discriminant a driver or parameter carries.",
        "    ///",
        "    /// `None` for a value that names no rotation.",
        "    #[must_use]",
        "    pub fn from_u8(v: u8) -> Option<Self> {",
        "        Some(match v {",
    ]
    for name, val in enum:
        L.append("            %d => Self::%s," % (val, camel(name)))
    L += [
        "            _ => return None,",
        "        })",
        "    }",
        "}",
        "",
        "/// Why a rotation could not be applied.",
        "#[derive(Debug, Clone, Copy, PartialEq, Eq)]",
        "pub enum BadRotation {",
        "    /// `ROTATION_CUSTOM_1` or `_2`, which upstream delegates to",
        "    /// `AP_CustomRotations` -- a separate library that is not ported, and",
        "    /// which upstream itself compiles out unless",
        "    /// `AP_CUSTOMROTATIONS_ENABLED`.",
        "    CustomUnsupported,",
        "    /// `ROTATION_MAX`, `ROTATION_CUSTOM_OLD` or `ROTATION_CUSTOM_END`:",
        "    /// enum bookkeeping, not orientations. Upstream reports",
        "    /// `INTERNAL_ERROR(bad_rotation)` and leaves the vector alone.",
        "    NotARotation,",
        "}",
        "",
        "/// Rotate `v` in place, upstream `Vector3<T>::rotate`.",
        "///",
        "/// # Errors",
        "///",
        "/// See [`BadRotation`]. Upstream returns `void` and reports an internal",
        "/// error; here the caller is told, and the vector is left unrotated",
        "/// either way.",
        "pub fn rotate<T: Real>(v: &mut Vector3<T>, rotation: Rotation) "
        "-> Result<(), BadRotation> {",
        "    match rotation {",
    ]

    handled = set()
    for labels, stmts in cases:
        concrete = [l for l in labels if l not in NON_ROTATIONS and l not in CUSTOM]
        if not concrete:
            continue
        handled.update(concrete)
        L.append("        %s => {" % " | ".join("Rotation::" + camel(l) for l in concrete))
        for s in rust_stmts(stmts):
            L.append("            %s" % s)
        L.append("        }")

    L += [
        "        Rotation::Custom1 | Rotation::Custom2 => {",
        "            return Err(BadRotation::CustomUnsupported);",
        "        }",
        "        Rotation::Max | Rotation::CustomOld | Rotation::CustomEnd => {",
        "            return Err(BadRotation::NotARotation);",
        "        }",
        "    }",
        "    Ok(())",
        "}",
        "",
        "/// Undo a rotation, upstream `Vector3<T>::rotate_inverse`.",
        "///",
        "/// Builds the rotation matrix by applying `rotation` to the three basis",
        "/// vectors, then multiplies by its transpose -- which for an orthonormal",
        "/// matrix is its inverse. Reproduced as upstream writes it rather than",
        "/// inverted analytically, so the arithmetic matches.",
        "///",
        "/// # Errors",
        "///",
        "/// As [`rotate`].",
        "pub fn rotate_inverse<T: Real>(v: &mut Vector3<T>, rotation: Rotation) "
        "-> Result<(), BadRotation> {",
        "    let mut x_vec = Vector3::new(T::one(), T::zero(), T::zero());",
        "    let mut y_vec = Vector3::new(T::zero(), T::one(), T::zero());",
        "    let mut z_vec = Vector3::new(T::zero(), T::zero(), T::one());",
        "",
        "    rotate(&mut x_vec, rotation)?;",
        "    rotate(&mut y_vec, rotation)?;",
        "    rotate(&mut z_vec, rotation)?;",
        "",
        "    let m = crate::matrix3::Matrix3::new(",
        "        x_vec.x, y_vec.x, z_vec.x,",
        "        x_vec.y, y_vec.y, z_vec.y,",
        "        x_vec.z, y_vec.z, z_vec.z,",
        "    );",
        "    *v = m.mul_transpose(*v);",
        "    Ok(())",
        "}",
        "",
    ]

    OUT.write_text("\n".join(L))
    missing = known - handled - NON_ROTATIONS - CUSTOM
    print("wrote %s" % OUT.name)
    print("  %d enum variants, %d concrete rotations" % (len(enum), len(handled)))
    if missing:
        raise SystemExit("rotations with no case body: %s" % sorted(missing))


if __name__ == "__main__":
    main()
