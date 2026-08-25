"""Emit crates/ap-motors/src/frames.rs from the extracted frame tables."""
import json
from pathlib import Path

OUT = Path("/srv/ardumaster/ports/plane-fw-rust/crates/ap-motors/src/frames.rs")
blocks = json.loads(Path("/tmp/frames.json").read_text())

TYPE_NUM = {
    "PLUS": 0, "X": 1, "V": 2, "H": 3, "VTAIL": 4, "ATAIL": 5, "PLUSREV": 6,
    "Y6B": 10, "Y6F": 11, "BF_X": 12, "DJI_X": 13, "CW_X": 14, "I": 15,
    "NYT_PLUS": 16, "NYT_X": 17, "BF_X_REV": 18, "Y4": 19, "X_COR": 20,
    "CW_X_COR": 21,
}


def ival(s):
    """A field the C++ writes as an integer but the extractor floated."""
    return s[:-2] if s.endswith(".0") else s


tables = []   # (const_name, rust_type, len, rows)
arms = []     # (class_num, name, [(type_num, const, variant, scale)], fallback)

for cls_num, cls_name, entries in blocks:
    upper = cls_name.upper()
    per_type = []
    fallback = None

    for names, motors, scale in entries:
        kind = motors[0][0]
        assert all(m[0] == kind for m in motors), (cls_name, names)

        label = "DEFAULT" if "DEFAULT" in names else names[0]
        const = "%s_%s" % (upper, label)

        if kind == "angle":
            rty = "Angle"
            rows = ["    (%s, %s, %s)," % (a[0], a[1], ival(a[2]))
                    for _, a in motors]
        elif kind == "raw":
            rty = "Raw"
            rows = ["    (%s, %s, %s, %s)," % (a[0], a[1], a[2], ival(a[3]))
                    for _, a in motors]
        else:
            rty = "ByAngles"
            rows = ["    (%s, %s, %s, %s, %s)."[:-1] % (
                ival(a[0]), a[1], a[2], a[3], ival(a[4])) + ","
                for _, a in motors]

        tables.append((const, rty, len(motors), rows))

        sc = ("None" if scale is None
              else "Some((%d, %d, %s))" % (scale[0], scale[1], scale[2]))
        if "DEFAULT" in names:
            fallback = (const, rty, sc)
        for n in names:
            if n == "DEFAULT":
                continue
            per_type.append((TYPE_NUM[n], const, rty, sc))

    arms.append((cls_num, cls_name, sorted(per_type), fallback))

L = []
L.append('//! Multirotor frame tables — which motors sit where, per frame.')
L.append('//!')
L.append('//! Generated from `AP_MotorsMatrix.cpp` by `tools/parity/gen_frames.py`.')
L.append('//! Do not edit by hand: regenerate, then run the `motors_frames` parity')
L.append('//! test, which checks every table against factors measured from the')
L.append('//! compiled ArduCopter object rather than read off the source.')
L.append('//!')
L.append('//! Frames guarded by `APM_BUILD_TYPE(APM_BUILD_ArduPlane)` are absent here.')
L.append('//! They are quadplane layouts (NYT_PLUS and NYT_X on the quad class), and')
L.append('//! they belong with the quadplane port, not with Copter.')
L.append('')
L.append('/// A motor given as one position angle: `(angle_deg, yaw_factor, order)`.')
L.append('pub(crate) type Angle = (f32, f32, u8);')
L.append('')
L.append('/// A motor given as factors outright: `(roll, pitch, yaw, order)`.')
L.append('pub(crate) type Raw = (f32, f32, f32, u8);')
L.append('')
L.append('/// A motor whose arms are asymmetric, so roll and pitch get their own')
L.append('/// angles: `(motor_num, roll_deg, pitch_deg, yaw_factor, order)`.')
L.append('pub(crate) type ByAngles = (i8, f32, f32, f32, u8);')
L.append('')
L.append('/// One frame\'s worth of motors, in whichever of the three shapes upstream')
L.append('/// wrote it. Keeping the shapes apart rather than pre-converting to factors')
L.append('/// means a table stays comparable to the source it came from.')
L.append('pub(crate) enum Layout {')
L.append('    Angle(&\'static [Angle]),')
L.append('    Raw(&\'static [Raw]),')
L.append('    ByAngles(&\'static [ByAngles]),')
L.append('}')
L.append('')
L.append('/// A frame: its motors, plus whatever upstream does to them afterwards.')
L.append('pub(crate) struct Frame {')
L.append('    pub layout: Layout,')
L.append('    /// Scale part of the motors, as `(limit, step, factor)`: indices')
L.append('    /// `0, step, 2*step, ...` below `limit`, across all four axes.')
L.append('    ///')
L.append('    /// The two co-rotating X8 frames shrink their top rotor layer so the')
L.append('    /// layers do not beat against each other, and they disagree about')
L.append('    /// which motors that is: X_COR takes the first four, CW_X_COR takes')
L.append('    /// every other one of eight. Same loop, different stride.')
L.append('    ///')
L.append('    /// `f32`, not `f64`, even though upstream writes the constant as a')
L.append('    /// bare `0.9`. ArduPilot compiles with `-fsingle-precision-constant`,')
L.append('    /// so that literal is a float and the multiply never promotes to')
L.append('    /// double. Reading it as a double puts this frame two ulp out.')
L.append('    pub top_layer_scale: Option<(usize, usize, f32)>,')
L.append('}')
L.append('')

for const, rty, n, rows in tables:
    L.append('const %s: [%s; %d] = [' % (const, rty, n))
    L.extend(rows)
    L.append('];')
    L.append('')

L.append('/// The table for a frame class and type, or `None` where upstream reports')
L.append('/// the combination unsupported.')
L.append('///')
L.append('/// Both arguments are plain integers, not enums, because that is what the')
L.append('/// FRAME_CLASS and FRAME_TYPE parameters hold — a value outside the enum is')
L.append('/// a configuration a real vehicle can be booted with, and the Y6 class')
L.append('/// answers for every one of them through its fallback.')
L.append('pub(crate) fn layout(frame_class: u8, frame_type: u8) -> Option<Frame> {')
L.append('    match frame_class {')

for cls_num, cls_name, per_type, fallback in arms:
    L.append('        // %s' % cls_name)
    L.append('        %d => match frame_type {' % cls_num)
    for tnum, const, rty, sc in per_type:
        L.append('            %d => Some(Frame { layout: Layout::%s(&%s), top_layer_scale: %s }),'
                 % (tnum, rty, const, sc))
    if fallback:
        const, rty, sc = fallback
        L.append('            // Y6 is the one class with a productive `default:` —')
        L.append('            // any other type gets the standard layout, not a refusal.')
        L.append('            _ => Some(Frame { layout: Layout::%s(&%s), top_layer_scale: %s }),'
                 % (rty, const, sc))
    else:
        L.append('            _ => None,')
    L.append('        },')

L.append('        _ => None,')
L.append('    }')
L.append('}')
L.append('')

OUT.write_text("\n".join(L))
print("wrote %s: %d tables, %d lines" % (OUT, len(tables), len(L)))
