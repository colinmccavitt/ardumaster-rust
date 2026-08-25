//! Multirotor frame tables — which motors sit where, per frame.
//!
//! Generated from `AP_MotorsMatrix.cpp` by `tools/parity/gen_frames.py`.
//! Do not edit by hand: regenerate, then run the `motors_frames` parity
//! test, which checks every table against factors measured from the
//! compiled ArduCopter object rather than read off the source.
//!
//! Frames guarded by `APM_BUILD_TYPE(APM_BUILD_ArduPlane)` are absent here.
//! They are quadplane layouts (NYT_PLUS and NYT_X on the quad class), and
//! they belong with the quadplane port, not with Copter.

/// A motor given as one position angle: `(angle_deg, yaw_factor, order)`.
pub(crate) type Angle = (f32, f32, u8);

/// A motor given as factors outright: `(roll, pitch, yaw, order)`.
pub(crate) type Raw = (f32, f32, f32, u8);

/// A motor whose arms are asymmetric, so roll and pitch get their own
/// angles: `(motor_num, roll_deg, pitch_deg, yaw_factor, order)`.
pub(crate) type ByAngles = (i8, f32, f32, f32, u8);

/// One frame's worth of motors, in whichever of the three shapes upstream
/// wrote it. Keeping the shapes apart rather than pre-converting to factors
/// means a table stays comparable to the source it came from.
pub(crate) enum Layout {
    Angle(&'static [Angle]),
    Raw(&'static [Raw]),
    ByAngles(&'static [ByAngles]),
}

/// A frame: its motors, plus whatever upstream does to them afterwards.
pub(crate) struct Frame {
    pub layout: Layout,
    /// Scale part of the motors, as `(limit, step, factor)`: indices
    /// `0, step, 2*step, ...` below `limit`, across all four axes.
    ///
    /// The two co-rotating X8 frames shrink their top rotor layer so the
    /// layers do not beat against each other, and they disagree about
    /// which motors that is: X_COR takes the first four, CW_X_COR takes
    /// every other one of eight. Same loop, different stride.
    ///
    /// `f32`, not `f64`, even though upstream writes the constant as a
    /// bare `0.9`. ArduPilot compiles with `-fsingle-precision-constant`,
    /// so that literal is a float and the multiply never promotes to
    /// double. Reading it as a double puts this frame two ulp out.
    pub top_layer_scale: Option<(usize, usize, f32)>,
}

const QUAD_PLUS: [Angle; 4] = [
    (90.0, 1.0, 2),
    (-90.0, 1.0, 4),
    (0.0, -1.0, 1),
    (180.0, -1.0, 3),
];

const QUAD_X: [Angle; 4] = [
    (45.0, 1.0, 1),
    (-135.0, 1.0, 3),
    (-45.0, -1.0, 4),
    (135.0, -1.0, 2),
];

const QUAD_BF_X: [Angle; 4] = [
    (135.0, -1.0, 2),
    (45.0, 1.0, 1),
    (-135.0, 1.0, 3),
    (-45.0, -1.0, 4),
];

const QUAD_BF_X_REV: [Angle; 4] = [
    (135.0, 1.0, 2),
    (45.0, -1.0, 1),
    (-135.0, -1.0, 3),
    (-45.0, 1.0, 4),
];

const QUAD_DJI_X: [Angle; 4] = [
    (45.0, 1.0, 1),
    (-45.0, -1.0, 4),
    (-135.0, 1.0, 3),
    (135.0, -1.0, 2),
];

const QUAD_CW_X: [Angle; 4] = [
    (45.0, 1.0, 1),
    (135.0, -1.0, 2),
    (-135.0, 1.0, 3),
    (-45.0, -1.0, 4),
];

const QUAD_V: [Angle; 4] = [
    (45.0, 0.7981, 1),
    (-135.0, 1.0000, 3),
    (-45.0, -0.7981, 4),
    (135.0, -1.0000, 2),
];

const QUAD_H: [Angle; 4] = [
    (45.0, -1.0, 1),
    (-135.0, -1.0, 3),
    (-45.0, 1.0, 4),
    (135.0, 1.0, 2),
];

const QUAD_VTAIL: [ByAngles; 4] = [
    (0, 60.0, 60.0, 0.0, 1),
    (1, 0.0, -160.0, -1.0, 3),
    (2, -60.0, -60.0, 0.0, 4),
    (3, 0.0, 160.0, 1.0, 2),
];

const QUAD_ATAIL: [ByAngles; 4] = [
    (0, 60.0, 60.0, 0.0, 1),
    (1, 0.0, -160.0, 1.0, 3),
    (2, -60.0, -60.0, 0.0, 4),
    (3, 0.0, 160.0, -1.0, 2),
];

const QUAD_PLUSREV: [Angle; 4] = [
    (90.0, -1.0, 2),
    (-90.0, -1.0, 4),
    (0.0, 1.0, 1),
    (180.0, 1.0, 3),
];

const QUAD_Y4: [Raw; 4] = [
    (-1.0, 1.000, 1.0, 1),
    (0.0, -1.000, -1.0, 2),
    (0.0, -1.000, 1.0, 3),
    (1.0, 1.000, -1.0, 4),
];

const HEXA_PLUS: [Angle; 6] = [
    (0.0, -1.0, 1),
    (180.0, 1.0, 4),
    (-120.0, -1.0, 5),
    (60.0, 1.0, 2),
    (-60.0, 1.0, 6),
    (120.0, -1.0, 3),
];

const HEXA_X: [Angle; 6] = [
    (90.0, -1.0, 2),
    (-90.0, 1.0, 5),
    (-30.0, -1.0, 6),
    (150.0, 1.0, 3),
    (30.0, 1.0, 1),
    (-150.0, -1.0, 4),
];

const HEXA_H: [Raw; 6] = [
    (-1.0, 0.0, -1.0, 2),
    (1.0, 0.0, 1.0, 5),
    (1.0, 1.0, -1.0, 6),
    (-1.0, -1.0, 1.0, 3),
    (-1.0, 1.0, 1.0, 1),
    (1.0, -1.0, -1.0, 4),
];

const HEXA_DJI_X: [Angle; 6] = [
    (30.0, 1.0, 1),
    (-30.0, -1.0, 6),
    (-90.0, 1.0, 5),
    (-150.0, -1.0, 4),
    (150.0, 1.0, 3),
    (90.0, -1.0, 2),
];

const HEXA_CW_X: [Angle; 6] = [
    (30.0, 1.0, 1),
    (90.0, -1.0, 2),
    (150.0, 1.0, 3),
    (-150.0, -1.0, 4),
    (-90.0, 1.0, 5),
    (-30.0, -1.0, 6),
];

const OCTA_PLUS: [Angle; 8] = [
    (0.0, -1.0, 1),
    (180.0, -1.0, 5),
    (45.0, 1.0, 2),
    (135.0, 1.0, 4),
    (-45.0, 1.0, 8),
    (-135.0, 1.0, 6),
    (-90.0, -1.0, 7),
    (90.0, -1.0, 3),
];

const OCTA_X: [Angle; 8] = [
    (22.5, -1.0, 1),
    (-157.5, -1.0, 5),
    (67.5, 1.0, 2),
    (157.5, 1.0, 4),
    (-22.5, 1.0, 8),
    (-112.5, 1.0, 6),
    (-67.5, -1.0, 7),
    (112.5, -1.0, 3),
];

const OCTA_V: [Raw; 8] = [
    (0.83, 0.34, -1.0, 7),
    (-0.67, -0.32, -1.0, 3),
    (0.67, -0.32, 1.0, 6),
    (-0.50, -1.00, 1.0, 4),
    (1.00, 1.00, 1.0, 8),
    (-0.83, 0.34, 1.0, 2),
    (-1.00, 1.00, -1.0, 1),
    (0.50, -1.00, -1.0, 5),
];

const OCTA_H: [Raw; 8] = [
    (-1.0, 1.0, -1.0, 1),
    (1.0, -1.0, -1.0, 5),
    (-1.0, 0.333, 1.0, 2),
    (-1.0, -1.0, 1.0, 4),
    (1.0, 1.0, 1.0, 8),
    (1.0, -0.333, 1.0, 6),
    (1.0, 0.333, -1.0, 7),
    (-1.0, -0.333, -1.0, 3),
];

const OCTA_I: [Raw; 8] = [
    (0.333, -1.0, -1.0, 5),
    (-0.333, 1.0, -1.0, 1),
    (1.0, -1.0, 1.0, 6),
    (0.333, 1.0, 1.0, 8),
    (-0.333, -1.0, 1.0, 4),
    (-1.0, 1.0, 1.0, 2),
    (-1.0, -1.0, -1.0, 3),
    (1.0, 1.0, -1.0, 7),
];

const OCTA_DJI_X: [Angle; 8] = [
    (22.5, 1.0, 1),
    (-22.5, -1.0, 8),
    (-67.5, 1.0, 7),
    (-112.5, -1.0, 6),
    (-157.5, 1.0, 5),
    (157.5, -1.0, 4),
    (112.5, 1.0, 3),
    (67.5, -1.0, 2),
];

const OCTA_CW_X: [Angle; 8] = [
    (22.5, 1.0, 1),
    (67.5, -1.0, 2),
    (112.5, 1.0, 3),
    (157.5, -1.0, 4),
    (-157.5, 1.0, 5),
    (-112.5, -1.0, 6),
    (-67.5, 1.0, 7),
    (-22.5, -1.0, 8),
];

const OCTAQUAD_PLUS: [Angle; 8] = [
    (0.0, 1.0, 1),
    (-90.0, -1.0, 7),
    (180.0, 1.0, 5),
    (90.0, -1.0, 3),
    (-90.0, 1.0, 8),
    (0.0, -1.0, 2),
    (90.0, 1.0, 4),
    (180.0, -1.0, 6),
];

const OCTAQUAD_X: [Angle; 8] = [
    (45.0, 1.0, 1),
    (-45.0, -1.0, 7),
    (-135.0, 1.0, 5),
    (135.0, -1.0, 3),
    (-45.0, 1.0, 8),
    (45.0, -1.0, 2),
    (135.0, 1.0, 4),
    (-135.0, -1.0, 6),
];

const OCTAQUAD_V: [Angle; 8] = [
    (45.0, 0.7981, 1),
    (-45.0, -0.7981, 7),
    (-135.0, 1.0000, 5),
    (135.0, -1.0000, 3),
    (-45.0, 0.7981, 8),
    (45.0, -0.7981, 2),
    (135.0, 1.0000, 4),
    (-135.0, -1.0000, 6),
];

const OCTAQUAD_H: [Angle; 8] = [
    (45.0, -1.0, 1),
    (-45.0, 1.0, 7),
    (-135.0, -1.0, 5),
    (135.0, 1.0, 3),
    (-45.0, -1.0, 8),
    (45.0, 1.0, 2),
    (135.0, -1.0, 4),
    (-135.0, 1.0, 6),
];

const OCTAQUAD_CW_X: [Angle; 8] = [
    (45.0, 1.0, 1),
    (45.0, -1.0, 2),
    (135.0, -1.0, 3),
    (135.0, 1.0, 4),
    (-135.0, 1.0, 5),
    (-135.0, -1.0, 6),
    (-45.0, -1.0, 7),
    (-45.0, 1.0, 8),
];

const OCTAQUAD_BF_X: [Angle; 8] = [
    (135.0, -1.0, 3),
    (45.0, 1.0, 1),
    (-135.0, 1.0, 5),
    (-45.0, -1.0, 7),
    (135.0, 1.0, 4),
    (45.0, -1.0, 2),
    (-135.0, -1.0, 6),
    (-45.0, 1.0, 8),
];

const OCTAQUAD_BF_X_REV: [Angle; 8] = [
    (135.0, 1.0, 3),
    (45.0, -1.0, 1),
    (-135.0, -1.0, 5),
    (-45.0, 1.0, 7),
    (135.0, -1.0, 4),
    (45.0, 1.0, 2),
    (-135.0, 1.0, 6),
    (-45.0, -1.0, 8),
];

const OCTAQUAD_X_COR: [Angle; 8] = [
    (45.0, 1.0, 1),
    (-45.0, -1.0, 7),
    (-135.0, 1.0, 5),
    (135.0, -1.0, 3),
    (-45.0, -1.0, 8),
    (45.0, 1.0, 2),
    (135.0, -1.0, 4),
    (-135.0, 1.0, 6),
];

const OCTAQUAD_CW_X_COR: [Angle; 8] = [
    (45.0, 1.0, 1),
    (45.0, 1.0, 2),
    (135.0, -1.0, 3),
    (135.0, -1.0, 4),
    (-135.0, 1.0, 5),
    (-135.0, 1.0, 6),
    (-45.0, -1.0, 7),
    (-45.0, -1.0, 8),
];

const Y6_Y6B: [Raw; 6] = [
    (-1.0, 0.500, -1.0, 1),
    (-1.0, 0.500, 1.0, 2),
    (0.0, -1.000, -1.0, 3),
    (0.0, -1.000, 1.0, 4),
    (1.0, 0.500, -1.0, 5),
    (1.0, 0.500, 1.0, 6),
];

const Y6_Y6F: [Raw; 6] = [
    (0.0, -1.000, 1.0, 3),
    (-1.0, 0.500, 1.0, 1),
    (1.0, 0.500, 1.0, 5),
    (0.0, -1.000, -1.0, 4),
    (-1.0, 0.500, -1.0, 2),
    (1.0, 0.500, -1.0, 6),
];

const Y6_DEFAULT: [Raw; 6] = [
    (-1.0, 0.666, 1.0, 2),
    (1.0, 0.666, -1.0, 5),
    (1.0, 0.666, 1.0, 6),
    (0.0, -1.333, -1.0, 4),
    (-1.0, 0.666, -1.0, 1),
    (0.0, -1.333, 1.0, 3),
];

const DODECAHEXA_PLUS: [Angle; 12] = [
    (0.0, 1.0, 1),
    (0.0, -1.0, 2),
    (60.0, -1.0, 3),
    (60.0, 1.0, 4),
    (120.0, 1.0, 5),
    (120.0, -1.0, 6),
    (180.0, -1.0, 7),
    (180.0, 1.0, 8),
    (-120.0, 1.0, 9),
    (-120.0, -1.0, 10),
    (-60.0, -1.0, 11),
    (-60.0, 1.0, 12),
];

const DODECAHEXA_X: [Angle; 12] = [
    (30.0, 1.0, 1),
    (30.0, -1.0, 2),
    (90.0, -1.0, 3),
    (90.0, 1.0, 4),
    (150.0, 1.0, 5),
    (150.0, -1.0, 6),
    (-150.0, -1.0, 7),
    (-150.0, 1.0, 8),
    (-90.0, 1.0, 9),
    (-90.0, -1.0, 10),
    (-30.0, -1.0, 11),
    (-30.0, 1.0, 12),
];

const DECA_PLUS: [Angle; 10] = [
    (0.0, 1.0, 1),
    (36.0, -1.0, 2),
    (72.0, 1.0, 3),
    (108.0, -1.0, 4),
    (144.0, 1.0, 5),
    (180.0, -1.0, 6),
    (-144.0, 1.0, 7),
    (-108.0, -1.0, 8),
    (-72.0, 1.0, 9),
    (-36.0, -1.0, 10),
];

const DECA_X: [Angle; 10] = [
    (18.0, 1.0, 1),
    (54.0, -1.0, 2),
    (90.0, 1.0, 3),
    (126.0, -1.0, 4),
    (162.0, 1.0, 5),
    (-162.0, -1.0, 6),
    (-126.0, 1.0, 7),
    (-90.0, -1.0, 8),
    (-54.0, 1.0, 9),
    (-18.0, -1.0, 10),
];

/// The table for a frame class and type, or `None` where upstream reports
/// the combination unsupported.
///
/// Both arguments are plain integers, not enums, because that is what the
/// FRAME_CLASS and FRAME_TYPE parameters hold — a value outside the enum is
/// a configuration a real vehicle can be booted with, and the Y6 class
/// answers for every one of them through its fallback.
pub(crate) fn layout(frame_class: u8, frame_type: u8) -> Option<Frame> {
    match frame_class {
        // Quad
        1 => match frame_type {
            0 => Some(Frame {
                layout: Layout::Angle(&QUAD_PLUS),
                top_layer_scale: None,
            }),
            1 => Some(Frame {
                layout: Layout::Angle(&QUAD_X),
                top_layer_scale: None,
            }),
            2 => Some(Frame {
                layout: Layout::Angle(&QUAD_V),
                top_layer_scale: None,
            }),
            3 => Some(Frame {
                layout: Layout::Angle(&QUAD_H),
                top_layer_scale: None,
            }),
            4 => Some(Frame {
                layout: Layout::ByAngles(&QUAD_VTAIL),
                top_layer_scale: None,
            }),
            5 => Some(Frame {
                layout: Layout::ByAngles(&QUAD_ATAIL),
                top_layer_scale: None,
            }),
            6 => Some(Frame {
                layout: Layout::Angle(&QUAD_PLUSREV),
                top_layer_scale: None,
            }),
            12 => Some(Frame {
                layout: Layout::Angle(&QUAD_BF_X),
                top_layer_scale: None,
            }),
            13 => Some(Frame {
                layout: Layout::Angle(&QUAD_DJI_X),
                top_layer_scale: None,
            }),
            14 => Some(Frame {
                layout: Layout::Angle(&QUAD_CW_X),
                top_layer_scale: None,
            }),
            18 => Some(Frame {
                layout: Layout::Angle(&QUAD_BF_X_REV),
                top_layer_scale: None,
            }),
            19 => Some(Frame {
                layout: Layout::Raw(&QUAD_Y4),
                top_layer_scale: None,
            }),
            _ => None,
        },
        // Hexa
        2 => match frame_type {
            0 => Some(Frame {
                layout: Layout::Angle(&HEXA_PLUS),
                top_layer_scale: None,
            }),
            1 => Some(Frame {
                layout: Layout::Angle(&HEXA_X),
                top_layer_scale: None,
            }),
            3 => Some(Frame {
                layout: Layout::Raw(&HEXA_H),
                top_layer_scale: None,
            }),
            13 => Some(Frame {
                layout: Layout::Angle(&HEXA_DJI_X),
                top_layer_scale: None,
            }),
            14 => Some(Frame {
                layout: Layout::Angle(&HEXA_CW_X),
                top_layer_scale: None,
            }),
            _ => None,
        },
        // Octa
        3 => match frame_type {
            0 => Some(Frame {
                layout: Layout::Angle(&OCTA_PLUS),
                top_layer_scale: None,
            }),
            1 => Some(Frame {
                layout: Layout::Angle(&OCTA_X),
                top_layer_scale: None,
            }),
            2 => Some(Frame {
                layout: Layout::Raw(&OCTA_V),
                top_layer_scale: None,
            }),
            3 => Some(Frame {
                layout: Layout::Raw(&OCTA_H),
                top_layer_scale: None,
            }),
            13 => Some(Frame {
                layout: Layout::Angle(&OCTA_DJI_X),
                top_layer_scale: None,
            }),
            14 => Some(Frame {
                layout: Layout::Angle(&OCTA_CW_X),
                top_layer_scale: None,
            }),
            15 => Some(Frame {
                layout: Layout::Raw(&OCTA_I),
                top_layer_scale: None,
            }),
            _ => None,
        },
        // OctaQuad
        4 => match frame_type {
            0 => Some(Frame {
                layout: Layout::Angle(&OCTAQUAD_PLUS),
                top_layer_scale: None,
            }),
            1 => Some(Frame {
                layout: Layout::Angle(&OCTAQUAD_X),
                top_layer_scale: None,
            }),
            2 => Some(Frame {
                layout: Layout::Angle(&OCTAQUAD_V),
                top_layer_scale: None,
            }),
            3 => Some(Frame {
                layout: Layout::Angle(&OCTAQUAD_H),
                top_layer_scale: None,
            }),
            12 => Some(Frame {
                layout: Layout::Angle(&OCTAQUAD_BF_X),
                top_layer_scale: None,
            }),
            14 => Some(Frame {
                layout: Layout::Angle(&OCTAQUAD_CW_X),
                top_layer_scale: None,
            }),
            18 => Some(Frame {
                layout: Layout::Angle(&OCTAQUAD_BF_X_REV),
                top_layer_scale: None,
            }),
            20 => Some(Frame {
                layout: Layout::Angle(&OCTAQUAD_X_COR),
                top_layer_scale: Some((4, 1, 0.9)),
            }),
            21 => Some(Frame {
                layout: Layout::Angle(&OCTAQUAD_CW_X_COR),
                top_layer_scale: Some((8, 2, 0.9)),
            }),
            _ => None,
        },
        // Y6
        5 => match frame_type {
            10 => Some(Frame {
                layout: Layout::Raw(&Y6_Y6B),
                top_layer_scale: None,
            }),
            11 => Some(Frame {
                layout: Layout::Raw(&Y6_Y6F),
                top_layer_scale: None,
            }),
            // Y6 is the one class with a productive `default:` —
            // any other type gets the standard layout, not a refusal.
            _ => Some(Frame {
                layout: Layout::Raw(&Y6_DEFAULT),
                top_layer_scale: None,
            }),
        },
        // DodecaHexa
        12 => match frame_type {
            0 => Some(Frame {
                layout: Layout::Angle(&DODECAHEXA_PLUS),
                top_layer_scale: None,
            }),
            1 => Some(Frame {
                layout: Layout::Angle(&DODECAHEXA_X),
                top_layer_scale: None,
            }),
            _ => None,
        },
        // Deca
        14 => match frame_type {
            0 => Some(Frame {
                layout: Layout::Angle(&DECA_PLUS),
                top_layer_scale: None,
            }),
            1 => Some(Frame {
                layout: Layout::Angle(&DECA_X),
                top_layer_scale: None,
            }),
            14 => Some(Frame {
                layout: Layout::Angle(&DECA_X),
                top_layer_scale: None,
            }),
            _ => None,
        },
        _ => None,
    }
}
