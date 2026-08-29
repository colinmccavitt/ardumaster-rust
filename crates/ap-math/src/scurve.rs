//! Snap-limited trajectory segments, upstream `AP_Math/SCurve`. COP-002.
//!
//! Copter flies between waypoints along a path whose *snap* — the rate of
//! change of jerk — is bounded, so the commanded acceleration is smooth rather
//! than merely continuous. Fixed-wing uses L1/NPFG instead and never executes
//! any of this.
//!
//! # Why bound snap and not just acceleration
//!
//! A trajectory with a step in jerk asks the motors for an instantaneous
//! change in torque. Bounding jerk fixes that but leaves a step in *snap*,
//! which shows up as a jolt in the airframe. The answer here is a raised-cosine
//! jerk profile — jerk rises as `Jm/2·(1 − cos(πt/tj))` — so every derivative
//! up to snap is continuous, and the whole trajectory is built from three
//! kinds of segment carrying that shape.
//!
//! # The three segment types
//!
//! - **Constant jerk**: plain polynomial integration.
//! - **Increasing jerk**: the raised-cosine rising from zero to `Jm`.
//! - **Decreasing jerk**: the same shape falling back, evaluated as the
//!   second half of a full cosine period so the two halves join exactly.
//!
//! A whole leg is 23 of these: one for the initial state, seven to accelerate,
//! seven to change speed, one at constant velocity, and seven to decelerate.
//!
//! # This slice
//!
//! `add_segments` and `calculate_track` — turn origin, destination and the
//! kinematic limits into a 23-segment track. `calculate_path` already
//! produces the five durations; this slice lays them into the array and
//! seats the 3-D chord (straight or the arc's NE geometry).
//!
//! `advance_target_along_track`, speed-change (`set_speed_max`, origin /
//! dest speed), arc projection and the `move_*` helpers are the rest of
//! the file.

use crate::control::kinematic_limit;
use crate::scalar::{
    is_negative, is_positive, is_zero, radians, safe_sqrt, sq, wrap_pi, Real,
};
use crate::vector2::Vector2f;
use crate::vector3::Vector3f;

/// Segments in a full track, upstream `segments_max`.
///
/// Segment 0 holds the initial state; 1–7 accelerate; 8–14 change speed; 15
/// is constant velocity; 16–22 decelerate.
pub const SEGMENTS_MAX: usize = 23;

/// Initial / empty-path index, upstream `SEG_INIT`.
pub const SEG_INIT: usize = 0;
/// Constant-accel hold of the accel half, upstream `SEG_ACCEL_MAX`.
pub const SEG_ACCEL_MAX: usize = 4;
/// Last accel-half segment, upstream `SEG_ACCEL_END`.
pub const SEG_ACCEL_END: usize = 7;
/// Last of the seven speed-change slots, upstream `SEG_SPEED_CHANGE_END`.
pub const SEG_SPEED_CHANGE_END: usize = 14;
/// Constant-velocity cruise, upstream `SEG_CONST` / `SEG_DECEL_START`.
pub const SEG_CONST: usize = 15;
/// Last decel-half segment, upstream `SEG_DECEL_END`.
pub const SEG_DECEL_END: usize = 22;

/// What the jerk is doing across a segment, upstream `SCurve::SegmentType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SegmentType {
    /// Jerk is constant. Plain polynomial integration.
    #[default]
    ConstantJerk,
    /// Jerk rises from zero to its reference on a raised cosine.
    PositiveJerk,
    /// Jerk falls back to zero on the same shape.
    NegativeJerk,
}

/// One segment's end state, upstream's anonymous `segment[]` struct.
#[derive(Debug, Clone, Copy, Default)]
pub struct Segment {
    /// Jerk reference: the value at the beginning, middle or end depending on
    /// the segment type.
    pub jerk_ref: f32,
    /// What the jerk is doing across this segment.
    pub seg_type: SegmentType,
    /// Time at the end of the segment.
    pub end_time: f32,
    /// Acceleration at the end of the segment.
    pub end_accel: f32,
    /// Velocity at the end of the segment.
    pub end_vel: f32,
    /// Position at the end of the segment.
    pub end_pos: f32,
}

/// Jerk, acceleration, velocity and position at an instant.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Javp {
    /// Jerk.
    pub jerk: f32,
    /// Acceleration.
    pub accel: f32,
    /// Velocity.
    pub vel: f32,
    /// Position.
    pub pos: f32,
}

/// A segment sequence being built or evaluated.
#[derive(Debug, Clone, Copy)]
pub struct SegmentTrack {
    segments: [Segment; SEGMENTS_MAX],
    num_segs: usize,
}

impl Default for SegmentTrack {
    fn default() -> Self {
        Self {
            segments: [Segment::default(); SEGMENTS_MAX],
            num_segs: 0,
        }
    }
}

/// State at a point in a segment, for the closed-form evaluators.
#[derive(Debug, Clone, Copy, Default)]
pub struct SegmentStart {
    /// Acceleration at the segment's start.
    pub accel: f32,
    /// Velocity at the segment's start.
    pub vel: f32,
    /// Position at the segment's start.
    pub pos: f32,
}

/// Jerk, acceleration, velocity and position after `t` of constant jerk,
/// upstream `calc_javp_for_segment_const_jerk`.
///
/// Plain polynomial integration: the textbook constant-jerk kinematics.
#[must_use]
pub fn javp_const_jerk(t: f32, j0: f32, s: SegmentStart) -> Javp {
    Javp {
        jerk: j0,
        accel: s.accel + j0 * t,
        vel: s.vel + s.accel * t + 0.5 * j0 * (t * t),
        pos: s.pos + s.vel * t + 0.5 * s.accel * (t * t) + (1.0 / 6.0) * j0 * (t * t * t),
    }
}

/// The same, for a segment whose jerk is rising on a raised cosine. Upstream
/// `calc_javp_for_segment_incr_jerk`.
///
/// `tj` is the segment's duration and `jm` the jerk it reaches. A non-positive
/// `tj` returns the start state untouched with zero jerk — there is no segment
/// to integrate over.
#[must_use]
pub fn javp_incr_jerk(t: f32, tj: f32, jm: f32, s: SegmentStart) -> Javp {
    if !is_positive(tj) {
        return Javp {
            jerk: 0.0,
            accel: s.accel,
            vel: s.vel,
            pos: s.pos,
        };
    }
    let alpha = jm * 0.5;
    let beta = core::f32::consts::PI / tj;

    Javp {
        jerk: alpha * (1.0 - Real::cos(beta * t)),
        accel: s.accel + alpha * t - (alpha / beta) * Real::sin(beta * t),
        vel: s.vel
            + s.accel * t
            + (alpha * 0.5) * (t * t)
            + (alpha / (beta * beta)) * Real::cos(beta * t)
            - alpha / (beta * beta),
        pos: s.pos
            + s.vel * t
            + 0.5 * s.accel * (t * t)
            + (-alpha / (beta * beta)) * t
            + alpha * (t * t * t) / 6.0
            + (alpha / (beta * beta * beta)) * Real::sin(beta * t),
    }
}

/// The same, for a segment whose jerk is falling. Upstream
/// `calc_javp_for_segment_decr_jerk`.
///
/// Evaluated as the *second half* of the same full cosine period the rising
/// segment used — every term carries `t + tj` — which is what makes the two
/// halves join exactly rather than approximately. The `AT`, `VT` and `PT`
/// terms subtract the state the first half would have accumulated.
#[must_use]
pub fn javp_decr_jerk(t: f32, tj: f32, jm: f32, s: SegmentStart) -> Javp {
    if !is_positive(tj) {
        return Javp {
            jerk: 0.0,
            accel: s.accel,
            vel: s.vel,
            pos: s.pos,
        };
    }
    let alpha = jm * 0.5;
    let beta = core::f32::consts::PI / tj;
    let at = alpha * tj;
    let vt = alpha * ((tj * tj) * 0.5 - 2.0 / (beta * beta));
    let pt = alpha * ((-1.0 / (beta * beta)) * tj + (1.0 / 6.0) * (tj * tj * tj));

    let tp = t + tj;
    Javp {
        jerk: alpha * (1.0 - Real::cos(beta * tp)),
        accel: (s.accel - at) + alpha * tp - (alpha / beta) * Real::sin(beta * tp),
        vel: (s.vel - vt)
            + (s.accel - at) * t
            + 0.5 * alpha * tp * tp
            + (alpha / (beta * beta)) * Real::cos(beta * tp)
            - alpha / (beta * beta),
        pos: (s.pos - pt)
            + (s.vel - vt) * t
            + 0.5 * (s.accel - at) * (t * t)
            + (-alpha / (beta * beta)) * tp
            + (alpha / 6.0) * tp * tp * tp
            + (alpha / (beta * beta * beta)) * Real::sin(beta * tp),
    }
}

impl SegmentTrack {
    /// An empty track.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// How many segments have been added.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.num_segs
    }

    /// Whether no segments have been added.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.num_segs == 0
    }

    /// One segment, by index.
    #[must_use]
    pub fn segment(&self, i: usize) -> Option<Segment> {
        self.segments.get(i).copied()
    }

    /// Append a segment, upstream `add_segment`.
    ///
    /// Silently does nothing when the array is full. Upstream indexes without
    /// checking and relies on its callers adding exactly 23; the bound is kept
    /// here because flight code must not write past an array whatever its
    /// callers do.
    pub fn add_segment(&mut self, seg: Segment) {
        if let Some(slot) = self.segments.get_mut(self.num_segs) {
            *slot = seg;
            self.num_segs += 1;
        }
    }

    fn last(&self) -> Segment {
        match self
            .num_segs
            .checked_sub(1)
            .and_then(|i| self.segments.get(i))
        {
            Some(s) => *s,
            None => Segment::default(),
        }
    }

    /// Append a constant-jerk segment of duration `tj`, upstream
    /// `add_segment_const_jerk`.
    ///
    /// A non-positive duration copies the previous segment's end state at the
    /// same time, so the sequence stays 23 long and the evaluator's indexing
    /// still works.
    pub fn add_segment_const_jerk(&mut self, tj: f32, j0: f32) {
        let p = self.last();
        if !is_positive(tj) {
            self.add_segment(Segment {
                jerk_ref: j0,
                seg_type: SegmentType::ConstantJerk,
                end_time: p.end_time,
                end_accel: p.end_accel,
                end_vel: p.end_vel,
                end_pos: p.end_pos,
            });
            return;
        }
        self.add_segment(Segment {
            jerk_ref: j0,
            seg_type: SegmentType::ConstantJerk,
            end_time: p.end_time + tj,
            end_accel: p.end_accel + j0 * tj,
            end_vel: p.end_vel + p.end_accel * tj + 0.5 * j0 * sq(tj),
            end_pos: p.end_pos
                + p.end_vel * tj
                + 0.5 * p.end_accel * sq(tj)
                + (1.0 / 6.0) * j0 * Real::powf(tj, 3.0),
        });
    }

    /// Append a rising-jerk segment, upstream `add_segment_incr_jerk`.
    pub fn add_segment_incr_jerk(&mut self, tj: f32, jm: f32) {
        let p = self.last();
        if !is_positive(tj) {
            self.add_segment(Segment {
                jerk_ref: 0.0,
                seg_type: SegmentType::ConstantJerk,
                end_time: p.end_time,
                end_accel: p.end_accel,
                end_vel: p.end_vel,
                end_pos: p.end_pos,
            });
            return;
        }
        let beta = core::f32::consts::PI / tj;
        let alpha = jm * 0.5;
        let at = alpha * tj;
        let vt = alpha * (sq(tj) * 0.5 - 2.0 / sq(beta));
        let pt = alpha * ((-1.0 / sq(beta)) * tj + (1.0 / 6.0) * Real::powf(tj, 3.0));

        self.add_segment(Segment {
            jerk_ref: jm,
            seg_type: SegmentType::PositiveJerk,
            end_time: p.end_time + tj,
            end_accel: p.end_accel + at,
            end_vel: p.end_vel + p.end_accel * tj + vt,
            end_pos: p.end_pos + p.end_vel * tj + 0.5 * p.end_accel * sq(tj) + pt,
        });
    }

    /// Append a falling-jerk segment, upstream `add_segment_decr_jerk`.
    pub fn add_segment_decr_jerk(&mut self, tj: f32, jm: f32) {
        let p = self.last();
        if !is_positive(tj) {
            self.add_segment(Segment {
                jerk_ref: 0.0,
                seg_type: SegmentType::ConstantJerk,
                end_time: p.end_time,
                end_accel: p.end_accel,
                end_vel: p.end_vel,
                end_pos: p.end_pos,
            });
            return;
        }
        let beta = core::f32::consts::PI / tj;
        let alpha = jm * 0.5;
        let at = alpha * tj;
        let vt = alpha * (sq(tj) * 0.5 - 2.0 / sq(beta));
        let pt = alpha * ((-1.0 / sq(beta)) * tj + (1.0 / 6.0) * Real::powf(tj, 3.0));
        let a2t = jm * tj;
        let v2t = jm * sq(tj);
        let p2t = alpha * ((-1.0 / sq(beta)) * 2.0 * tj + (4.0 / 3.0) * Real::powf(tj, 3.0));

        self.add_segment(Segment {
            jerk_ref: jm,
            seg_type: SegmentType::NegativeJerk,
            end_time: p.end_time + tj,
            end_accel: (p.end_accel - at) + a2t,
            end_vel: (p.end_vel - vt) + (p.end_accel - at) * tj + v2t,
            end_pos: (p.end_pos - pt)
                + (p.end_vel - vt) * tj
                + 0.5 * (p.end_accel - at) * sq(tj)
                + p2t,
        });
    }

    /// Rise, hold, fall — one jerk pulse. Upstream `add_segments_jerk`.
    pub fn add_segments_jerk(&mut self, tj: f32, jm: f32, tcj: f32) {
        self.add_segment_incr_jerk(tj, jm);
        self.add_segment_const_jerk(tcj, jm);
        self.add_segment_decr_jerk(tj, jm);
    }

    fn segment_mut(&mut self, i: usize) -> Option<&mut Segment> {
        self.segments.get_mut(i)
    }

    /// Lay a full 23-segment track of length `length`, upstream `add_segments`.
    ///
    /// The init segment (index 0) must already be there — [`SCurve::init`]
    /// puts it in. A zero length leaves the track untouched.
    ///
    /// `calculate_path` is asked for half the length because the accel and
    /// decel halves are mirrors. The seven speed-change slots stay empty
    /// here; `set_speed_max` (later leftover) rewrites them. `t15` is the
    /// cruise that fills whatever of `length` the two halves did not use.
    ///
    /// The accel-end and decel-end accel (and the decel-end vel) are forced
    /// to zero after the sums: floating-point drift would otherwise leave
    /// a path that never quite stops, and [`valid`] rejects that.
    pub fn add_segments(
        &mut self,
        snap_max: f32,
        jerk_max: f32,
        accel_max: f32,
        vel_max: f32,
        length: f32,
    ) {
        if is_zero(length) {
            return;
        }
        if self.num_segs == 0 {
            self.add_segment(Segment::default());
        }

        let p = calculate_path(snap_max, jerk_max, 0.0, accel_max, vel_max, length * 0.5);

        self.add_segments_jerk(p.tj, p.jm, p.t2);
        self.add_segment_const_jerk(p.t4, 0.0);
        self.add_segments_jerk(p.tj, -p.jm, p.t6);

        if let Some(s) = self.segment_mut(SEG_ACCEL_END) {
            s.end_accel = 0.0;
        }

        for _ in 0..7 {
            self.add_segment_const_jerk(0.0, 0.0);
        }

        let (end_pos, end_vel) = self
            .segment(SEG_SPEED_CHANGE_END)
            .map(|s| (s.end_pos, s.end_vel))
            .unwrap_or((0.0, 0.0));
        let t15 = cpp_max(0.0, (length - 2.0 * end_pos) / end_vel);
        self.add_segment_const_jerk(t15, 0.0);

        self.add_segments_jerk(p.tj, -p.jm, p.t6);
        self.add_segment_const_jerk(p.t4, 0.0);
        self.add_segments_jerk(p.tj, p.jm, p.t2);

        if let Some(s) = self.segment_mut(SEG_DECEL_END) {
            s.end_accel = 0.0;
            s.end_vel = 0.0;
        }
    }

    /// True when the 23-segment array is a usable path, upstream `valid`.
    ///
    /// Every stored number must be finite, velocity never negative, time
    /// and position never go backwards, and the last segment must finish
    /// at zero acceleration — otherwise the vehicle would keep thrusting
    /// after the path said it was done.
    #[must_use]
    pub fn valid(&self) -> bool {
        if self.num_segs != SEGMENTS_MAX {
            return false;
        }
        for i in 0..self.num_segs {
            let Some(s) = self.segments.get(i) else {
                return false;
            };
            if !s.jerk_ref.is_finite()
                || !s.end_time.is_finite()
                || !s.end_accel.is_finite()
                || !s.end_vel.is_finite()
                || is_negative(s.end_vel)
                || !s.end_pos.is_finite()
            {
                return false;
            }
            if i >= 1 {
                let Some(prev) = self.segments.get(i - 1) else {
                    return false;
                };
                if is_negative(s.end_time - prev.end_time)
                    || is_negative(s.end_pos - prev.end_pos)
                {
                    return false;
                }
            }
        }
        match self.segments.get(self.num_segs - 1) {
            Some(last) if is_zero(last.end_accel) => true,
            _ => false,
        }
    }

    /// Evaluate the track at a time, upstream
    /// `get_jerk_accel_vel_pos_at_time`.
    ///
    /// Returns zeros unless the track is completely built. Upstream guards
    /// with `num_segs != segments_max` and returns silently, because a
    /// half-built track has no meaningful state — the segments after the last
    /// one added still hold whatever was there before.
    ///
    /// Position is clamped at zero: it is distance along the track, and a
    /// negative one would mean the vehicle was behind its own origin.
    #[must_use]
    pub fn javp_at_time(&self, time_now: f32) -> Javp {
        if self.num_segs != SEGMENTS_MAX {
            return Javp::default();
        }

        // The earliest segment that has not ended yet.
        let mut pnt = self.num_segs;
        for i in 0..self.num_segs {
            let idx = self.num_segs - 1 - i;
            if let Some(seg) = self.segments.get(idx) {
                if time_now < seg.end_time {
                    pnt = idx;
                }
            }
        }

        let (jtype, jm, tj, t0, start) = if pnt == 0 {
            let s = self.segment(0).unwrap_or_default();
            (
                SegmentType::ConstantJerk,
                0.0,
                0.0,
                s.end_time,
                SegmentStart {
                    accel: s.end_accel,
                    vel: s.end_vel,
                    pos: s.end_pos,
                },
            )
        } else if pnt == self.num_segs {
            let s = self.segment(pnt - 1).unwrap_or_default();
            (
                SegmentType::ConstantJerk,
                0.0,
                0.0,
                s.end_time,
                SegmentStart {
                    accel: s.end_accel,
                    vel: s.end_vel,
                    pos: s.end_pos,
                },
            )
        } else {
            let cur = self.segment(pnt).unwrap_or_default();
            let prev = self.segment(pnt - 1).unwrap_or_default();
            (
                cur.seg_type,
                cur.jerk_ref,
                cur.end_time - prev.end_time,
                prev.end_time,
                SegmentStart {
                    accel: prev.end_accel,
                    vel: prev.end_vel,
                    pos: prev.end_pos,
                },
            )
        };

        let t = time_now - t0;
        let mut out = match jtype {
            SegmentType::ConstantJerk => javp_const_jerk(t, jm, start),
            SegmentType::PositiveJerk => javp_incr_jerk(t, tj, jm, start),
            SegmentType::NegativeJerk => javp_decr_jerk(t, tj, jm, start),
        };
        out.pos = out.pos.max(0.0);
        out
    }
}

/// Segment durations for a trigonometric S-curve, upstream
/// `SCurve::calculate_path`.
///
/// `tj` is the raised-cosine jerk rise (and fall) time. `t2` is the
/// constant-jerk stretch that holds `Jm` after the rise. `t4` is the
/// constant-acceleration stretch. `t6` is the mirror of `t2` on the way
/// back down. `jm` is the jerk the profile actually uses — it can be
/// smaller than the caller's limit when the path is too short to reach it.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PathTimes {
    /// Jerk the profile will use, m/s³.
    pub jm: f32,
    /// Raised-cosine jerk rise time, seconds.
    pub tj: f32,
    /// Constant-jerk hold after the rise, seconds.
    pub t2: f32,
    /// Constant-acceleration hold, seconds.
    pub t4: f32,
    /// Constant-jerk hold on the way down, seconds.
    pub t6: f32,
}

/// C++ `MIN` / `MAX` macros: a strict `<` / `>` ternary, not IEEE minNum.
///
/// A NaN comparison is false, so `MIN(finite, NaN)` returns the NaN and
/// `MIN(NaN, finite)` returns the finite value. `f32::min` does the other
/// thing, and that would hide a cubic that overflowed.
#[inline]
fn cpp_min(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}

#[inline]
fn cpp_max(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}

/// The `t4` closed form shared by solutions 2 and 7. Upstream writes this
/// expression twice; both copies are the same polynomial.
///
/// Two roots of the distance-under-constant-accel quadratic, then the
/// speed-limit cap, then a floor at zero so a numerical undershoot cannot
/// schedule a negative hold.
fn t4_for_speed_change(am: f32, jm: f32, v0: f32, vm: f32, tj: f32, length: f32) -> f32 {
    let disc = (am * am * am * am) * (1.0 / 4.0)
        + (jm * jm) * (v0 * v0)
        + (am * am) * (jm * jm) * (tj * tj) * (1.0 / 4.0)
        + am * (jm * jm) * length * 2.0
        - (am * am) * jm * v0
        + (am * am * am) * jm * tj * (1.0 / 2.0)
        - am * (jm * jm) * v0 * tj;
    let root = safe_sqrt(disc);
    let t4_a = ((am * am) * (-3.0 / 2.0) + root - jm * v0 - am * jm * tj * (3.0 / 2.0)) / (am * jm);
    let t4_b = ((am * am) * (-3.0 / 2.0) - root - jm * v0 - am * jm * tj * (3.0 / 2.0)) / (am * jm);
    let t4_c = -(v0 - vm + am * tj + (am * am) / jm) / am;
    cpp_max(cpp_min(t4_c, cpp_max(t4_a, t4_b)), 0.0)
}

/// Cardano reduction of the short-path cubic that solution 5 solves for `Am`.
///
/// The two cube-root terms are the same value added twice in the C++ — once
/// as `p / u` and once as `u` — which is the `u + p/u` form of a depressed
/// cubic whose roots multiply to `p`.
fn am_for_short_path(am: f32, jm: f32, v0: f32, vm: f32, tj: f32, length: f32) -> f32 {
    let span = safe_sqrt((v0 * -4.0 + vm * 4.0 + jm * (tj * tj)) / jm);
    let from_vel = cpp_max(
        jm * (tj + span) * (-1.0 / 2.0),
        jm * (tj - span) * (-1.0 / 2.0),
    );
    let p = (jm * jm) * (tj * tj) * (1.0 / 9.0) - jm * v0 * (2.0 / 3.0);
    let q_body = -(jm * jm) * length * (1.0 / 2.0) + (jm * jm * jm) * (tj * tj * tj) * (8.0 / 27.0)
        - jm * tj * ((jm * jm) * (tj * tj) + jm * v0 * 2.0) * (1.0 / 3.0)
        + (jm * jm) * v0 * tj;
    let cbrt_arg = safe_sqrt(Real::powf(q_body, 2.0) - Real::powf(p, 3.0))
        + (jm * jm) * length * (1.0 / 2.0)
        - (jm * jm * jm) * (tj * tj * tj) * (8.0 / 27.0)
        + jm * tj * ((jm * jm) * (tj * tj) + jm * v0 * 2.0) * (1.0 / 3.0)
        - (jm * jm) * v0 * tj;
    let u = Real::powf(cbrt_arg, 1.0 / 3.0);
    let from_cubic = jm * tj * (-2.0 / 3.0) + p * 1.0 / u + u;
    cpp_min(cpp_min(am, from_vel), from_cubic)
}

/// Segment times for a snap-limited path of length `length` starting at
/// speed `v0`, upstream `SCurve::calculate_path`.
///
/// `sm` is snap, `jm` jerk, `am` acceleration, `vm` speed — every limit
/// the vehicle is allowed. Returns zeros when the inputs cannot make a
/// path: non-positive limits, a start already at or above `vm`, or a
/// length too short to accelerate at all.
///
/// The four solutions (0 / 2 / 5 / 7) are which of `t2`, `t4`, `t6` are
/// live. A short hop from rest never reaches the jerk or accel limits and
/// is all raised-cosine (`t2 = t4 = t6 = 0`). A long cruise uses every
/// segment (`t2`, `t4`, `t6` all positive).
///
/// Invalid outputs — NaN, infinity, or a negative duration — are zeroed
/// the same way upstream's `INTERNAL_ERROR` path is, except `tj` is left
/// alone: that is what the C++ does.
#[must_use]
pub fn calculate_path(
    sm: f32,
    mut jm: f32,
    v0: f32,
    mut am: f32,
    vm: f32,
    length: f32,
) -> PathTimes {
    if !is_positive(sm)
        || !is_positive(jm)
        || !is_positive(am)
        || !is_positive(vm)
        || !is_positive(length)
    {
        return PathTimes::default();
    }
    if v0 >= vm {
        return PathTimes::default();
    }

    // C++ `Jm * M_PI / (2.0f * Sm)`: left-associative, `M_PI` is double.
    let mut tj = jm * core::f32::consts::PI / (2.0 * sm);
    let at = cpp_min(
        cpp_min(am, (vm - v0) / (2.0 * tj)),
        (length - 4.0 * v0 * tj) / (4.0 * sq(tj)),
    );
    if !is_positive(at) {
        return PathTimes::default();
    }

    let (t2, t4, t6);
    if at.abs() < jm * tj {
        if is_zero(v0) {
            // No closed form for a non-zero start on this branch, so from
            // rest we shrink `tj` until snap, speed and accel all fit.
            tj = cpp_min(
                cpp_min(
                    cpp_min(
                        tj,
                        Real::powf((length * core::f32::consts::PI) / (8.0 * sm), 1.0 / 4.0),
                    ),
                    Real::powf((vm * core::f32::consts::PI) / (4.0 * sm), 1.0 / 3.0),
                ),
                safe_sqrt((am * core::f32::consts::PI) / (2.0 * sm)),
            );
            // C++ `2.0f * Sm * tj / M_PI`.
            jm = 2.0 * sm * tj / core::f32::consts::PI;
            am = jm * tj;
        } else {
            // Speed change: keep `tj`, drop `Jm` so the small `At` fits.
            am = at;
            jm = am / tj;
        }
        if vm <= v0 + 2.0 * am * tj || length <= 4.0 * v0 * tj + 4.0 * am * sq(tj) {
            // solution 0 — t6 t4 t2 = 0 0 0
            t2 = 0.0;
            t4 = 0.0;
            t6 = 0.0;
        } else {
            // solution 2 — t6 t4 t2 = 0 1 0
            t2 = 0.0;
            t4 = t4_for_speed_change(am, jm, v0, vm, tj, length);
            t6 = 0.0;
        }
    } else if vm < v0 + am * tj + (am * am) / jm
        || length
            < 1.0 / (jm * jm) * (am * am * am + am * jm * (v0 * 2.0 + am * tj * 2.0))
                + v0 * tj * 2.0
                + am * (tj * tj)
    {
        // solution 5 — t6 t4 t2 = 1 0 1
        am = am_for_short_path(am, jm, v0, vm, tj, length);
        t2 = am / jm - tj;
        t4 = 0.0;
        t6 = t2;
    } else {
        // solution 7 — t6 t4 t2 = 1 1 1
        t2 = am / jm - tj;
        t4 = t4_for_speed_change(am, jm, v0, vm, tj, length);
        t6 = t2;
    }

    let out = PathTimes { jm, tj, t2, t4, t6 };
    if !out.jm.is_finite()
        || is_negative(out.jm)
        || !out.tj.is_finite()
        || is_negative(out.tj)
        || !out.t2.is_finite()
        || is_negative(out.t2)
        || !out.t4.is_finite()
        || is_negative(out.t4)
        || !out.t6.is_finite()
        || is_negative(out.t6)
    {
        // Upstream zeroes Jm/t2/t4/t6 and leaves tj as computed.
        return PathTimes {
            tj: out.tj,
            ..PathTimes::default()
        };
    }
    out
}

/// Circular-arc geometry in the NE plane, upstream's anonymous `arc` struct.
///
/// The scalar S-curve still runs on path length. Projecting that motion
/// onto this circle is a later leftover (`project_scurve_onto_track`).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Arc {
    /// Signed central angle, radians. Upstream: +CCW, −CW, 0 = straight.
    pub angle_rad: f32,
    /// Horizontal arc length `R · |θ|`.
    pub length_ne: f32,
    /// Arc radius in the NE plane.
    pub radius_ne: f32,
    /// Circle centre relative to the start point, NE.
    pub center_ne: Vector2f,
}

/// A snap-limited 3-D track between two points, upstream `SCurve`.
///
/// This slice builds the track. Advancing along it, changing speed mid-leg,
/// and projecting the scalar path onto an arc are later leftovers.
#[derive(Debug, Clone)]
pub struct SCurve {
    snap_max: f32,
    jerk_max: f32,
    accel_max: f32,
    accel_z_max: f32,
    vel_max: f32,
    time: f32,
    track: SegmentTrack,
    is_arc_segment: bool,
    seg_delta: Vector3f,
    seg_length: f32,
    arc: Arc,
}

impl Default for SCurve {
    fn default() -> Self {
        Self::new()
    }
}

impl SCurve {
    /// An empty path: one init segment, no motion.
    #[must_use]
    pub fn new() -> Self {
        let mut s = Self {
            snap_max: 0.0,
            jerk_max: 0.0,
            accel_max: 0.0,
            accel_z_max: 0.0,
            vel_max: 0.0,
            time: 0.0,
            track: SegmentTrack::new(),
            is_arc_segment: false,
            seg_delta: Vector3f::zero(),
            seg_length: 0.0,
            arc: Arc::default(),
        };
        s.init();
        s
    }

    /// Clear the path and put the empty init segment in, upstream `init`.
    ///
    /// `accel_z_max` is left alone: that is what the C++ does.
    pub fn init(&mut self) {
        self.snap_max = 0.0;
        self.jerk_max = 0.0;
        self.accel_max = 0.0;
        self.vel_max = 0.0;
        self.time = 0.0;
        self.track = SegmentTrack::new();
        self.track.add_segment(Segment::default());
        self.is_arc_segment = false;
        self.seg_delta = Vector3f::zero();
        self.seg_length = 0.0;
        self.arc = Arc::default();
    }

    /// Build a track from `origin` to `destination`, upstream `calculate_track`.
    ///
    /// Origin and destination are NED offsets from the EKF origin. An arc
    /// smaller than one degree is treated as a straight chord — below that
    /// the circle centre is a numerical fiction. Invalid limits (non-positive
    /// snap, jerk, accel or speed) leave a zero-length path rather than a
    /// track that would divide by zero. A path that fails [`SegmentTrack::valid`]
    /// is thrown away the same way.
    #[allow(
        clippy::too_many_arguments,
        reason = "matches upstream calculate_track's argument list"
    )]
    pub fn calculate_track(
        &mut self,
        origin: Vector3f,
        destination: Vector3f,
        arc_ang_rad: f32,
        mut speed_xy: f32,
        mut speed_up: f32,
        mut speed_down: f32,
        mut accel_xy: f32,
        mut accel_z: f32,
        mut accel_c: f32,
        snap_maximum: f32,
        jerk_maximum: f32,
    ) {
        self.init();

        speed_xy = speed_xy.abs();
        speed_up = speed_up.abs();
        speed_down = speed_down.abs();
        accel_xy = accel_xy.abs();
        accel_z = accel_z.abs();

        self.seg_delta = destination - origin;
        if self.seg_delta.is_zero() || is_zero(self.seg_delta.length_squared()) {
            self.seg_delta = Vector3f::zero();
            return;
        }

        let chord = self.seg_delta.xy();
        let chord_length = chord.length();
        if !is_positive(chord_length) || wrap_pi(arc_ang_rad).abs() < radians(1.0) {
            self.set_straight(chord_length);
        } else {
            self.is_arc_segment = true;
            self.arc.angle_rad = arc_ang_rad;
            self.arc.radius_ne =
                (chord_length / (2.0 * Real::sin(self.arc.angle_rad * 0.5).abs())).abs();
            let center_offset = safe_sqrt(sq(self.arc.radius_ne) - sq(chord_length * 0.5));
            let turn_dir = if is_negative(self.arc.angle_rad) {
                -1.0
            } else {
                1.0
            };
            let center_side = if is_positive(wrap_pi(self.arc.angle_rad.abs())) {
                1.0
            } else {
                -1.0
            };
            if !is_zero(self.arc.radius_ne) && !is_zero(chord_length) {
                self.arc.center_ne = chord * 0.5
                    + Vector2f::new(-chord.y, chord.x)
                        * (center_side * turn_dir * center_offset / chord_length);
                self.arc.length_ne = self.arc.radius_ne * self.arc.angle_rad.abs();
                self.seg_length = safe_sqrt(sq(self.seg_delta.z) + sq(self.arc.length_ne));
                accel_c = if is_positive(accel_c) {
                    accel_c
                } else {
                    accel_xy
                };
                speed_xy = cpp_min(speed_xy, safe_sqrt(accel_c * self.arc.radius_ne));
            } else {
                self.set_straight(chord_length);
            }
        }

        if is_zero(self.seg_length) {
            self.seg_delta = Vector3f::zero();
            return;
        }

        self.snap_max = snap_maximum;
        self.jerk_max = jerk_maximum;
        self.set_kinematic_limits(
            origin,
            destination,
            speed_xy,
            speed_up,
            speed_down,
            accel_xy,
            accel_z,
        );

        if !is_positive(self.snap_max)
            || !is_positive(self.jerk_max)
            || !is_positive(self.accel_max)
            || !is_positive(self.vel_max)
        {
            return;
        }

        self.track.add_segments(
            self.snap_max,
            self.jerk_max,
            self.accel_max,
            self.vel_max,
            self.seg_length,
        );

        if !self.track.valid() {
            self.init();
        }
    }

    fn set_straight(&mut self, chord_length: f32) {
        self.is_arc_segment = false;
        self.arc.angle_rad = 0.0;
        self.arc.length_ne = chord_length;
        self.arc.radius_ne = 0.0;
        self.arc.center_ne = Vector2f::zero();
        self.seg_length = self.seg_delta.length();
    }

    /// Speed and accel along the track from the 3-D direction.
    /// Upstream `set_kinematic_limits`.
    fn set_kinematic_limits(
        &mut self,
        origin: Vector3f,
        destination: Vector3f,
        speed_xy: f32,
        speed_up: f32,
        speed_down: f32,
        accel_xy: f32,
        accel_z: f32,
    ) {
        let direction = destination - origin;
        self.vel_max = kinematic_limit(direction, speed_xy, speed_up, speed_down);
        self.accel_max = kinematic_limit(direction, accel_xy, accel_z, accel_z);
        self.accel_z_max = accel_z;
    }

    /// The 23-segment array, or the lone init segment on an empty path.
    #[must_use]
    pub const fn track(&self) -> &SegmentTrack {
        &self.track
    }

    /// Displacement origin → destination, NED.
    #[must_use]
    pub const fn seg_delta(&self) -> Vector3f {
        self.seg_delta
    }

    /// Scalar path length (straight 3-D length, or arc-length ⊕ Δz).
    #[must_use]
    pub const fn seg_length(&self) -> f32 {
        self.seg_length
    }

    /// True when [`calculate_track`] stored a circular NE arc.
    #[must_use]
    pub const fn is_arc_segment(&self) -> bool {
        self.is_arc_segment
    }

    /// Arc geometry last computed by [`calculate_track`].
    #[must_use]
    pub const fn arc(&self) -> Arc {
        self.arc
    }

    /// Speed limit along the track after kinematic limiting.
    #[must_use]
    pub const fn vel_max(&self) -> f32 {
        self.vel_max
    }

    /// Accel limit along the track after kinematic limiting.
    #[must_use]
    pub const fn accel_max(&self) -> f32 {
        self.accel_max
    }

    /// Vertical accel limit stored by [`set_kinematic_limits`].
    #[must_use]
    pub const fn accel_z_max(&self) -> f32 {
        self.accel_z_max
    }

    /// Snap limit last passed to [`calculate_track`].
    #[must_use]
    pub const fn snap_max(&self) -> f32 {
        self.snap_max
    }

    /// Jerk limit last passed to [`calculate_track`].
    #[must_use]
    pub const fn jerk_max(&self) -> f32 {
        self.jerk_max
    }

    /// Elapsed path time. Advanced by a later leftover.
    #[must_use]
    pub const fn time(&self) -> f32 {
        self.time
    }

    /// True when the 23-segment array is a usable path, upstream `valid`.
    #[must_use]
    pub fn valid(&self) -> bool {
        self.track.valid()
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "a zero-length segment must leave the state EXACTLY as it was; an \
epsilon would accept a drift, which is the failure"
    )]

    use super::*;

    /// Constant jerk is the textbook polynomial, and each derivative
    /// integrates to the next.
    #[test]
    fn constant_jerk_integrates_as_a_polynomial() {
        let s = SegmentStart {
            accel: 1.0,
            vel: 2.0,
            pos: 3.0,
        };
        let r = javp_const_jerk(2.0, 0.5, s);
        assert_eq!(r.jerk, 0.5);
        assert_eq!(r.accel, 1.0 + 0.5 * 2.0);
        assert_eq!(r.vel, 2.0 + 1.0 * 2.0 + 0.5 * 0.5 * 4.0);
        assert!(
            (r.pos - (3.0 + 2.0 * 2.0 + 0.5 * 1.0 * 4.0 + (1.0 / 6.0) * 0.5 * 8.0)).abs() < 1e-5
        );
    }

    /// The rising segment starts at zero jerk and reaches the reference at
    /// the end. That is what makes snap finite at the join.
    #[test]
    fn a_rising_segment_starts_and_ends_at_the_right_jerk() {
        let s = SegmentStart::default();
        let (tj, jm) = (0.5_f32, 4.0_f32);

        let at_start = javp_incr_jerk(0.0, tj, jm, s);
        assert!(
            at_start.jerk.abs() < 1e-5,
            "jerk starts at zero: {}",
            at_start.jerk
        );

        let at_end = javp_incr_jerk(tj, tj, jm, s);
        assert!(
            (at_end.jerk - jm).abs() < 1e-4,
            "and reaches the reference: {}",
            at_end.jerk
        );
    }

    /// The falling segment is the mirror: it begins at the reference and
    /// returns to zero.
    #[test]
    fn a_falling_segment_returns_the_jerk_to_zero() {
        let s = SegmentStart::default();
        let (tj, jm) = (0.5_f32, 4.0_f32);

        let at_start = javp_decr_jerk(0.0, tj, jm, s);
        assert!(
            (at_start.jerk - jm).abs() < 1e-4,
            "begins at the reference: {}",
            at_start.jerk
        );

        let at_end = javp_decr_jerk(tj, tj, jm, s);
        assert!(
            at_end.jerk.abs() < 1e-4,
            "and returns to zero: {}",
            at_end.jerk
        );
    }

    /// A rising segment followed by a falling one is one continuous cosine
    /// period, so the acceleration handed over at the join must match.
    #[test]
    fn the_two_halves_join_exactly() {
        let (tj, jm) = (0.5_f32, 4.0_f32);
        let s = SegmentStart {
            accel: 0.5,
            vel: 1.0,
            pos: 2.0,
        };

        let end_of_rise = javp_incr_jerk(tj, tj, jm, s);
        let start_of_fall = javp_decr_jerk(
            0.0,
            tj,
            jm,
            SegmentStart {
                accel: end_of_rise.accel,
                vel: end_of_rise.vel,
                pos: end_of_rise.pos,
            },
        );

        assert!((start_of_fall.accel - end_of_rise.accel).abs() < 1e-4);
        assert!((start_of_fall.vel - end_of_rise.vel).abs() < 1e-4);
        assert!((start_of_fall.jerk - end_of_rise.jerk).abs() < 1e-3);
    }

    /// A zero-length segment leaves the state exactly alone. Upstream relies
    /// on this to keep the array 23 long when a phase is not needed.
    #[test]
    fn a_zero_length_segment_changes_nothing() {
        let s = SegmentStart {
            accel: 1.5,
            vel: -2.5,
            pos: 7.0,
        };
        for r in [
            javp_incr_jerk(0.3, 0.0, 4.0, s),
            javp_decr_jerk(0.3, 0.0, 4.0, s),
        ] {
            assert_eq!(r.jerk, 0.0);
            assert_eq!(r.accel, s.accel);
            assert_eq!(r.vel, s.vel);
            assert_eq!(r.pos, s.pos);
        }
    }

    /// The velocity a rising segment produces is the integral of its jerk
    /// twice over — checked numerically rather than by restating the formula.
    #[test]
    fn the_closed_form_agrees_with_numerical_integration() {
        let (tj, jm) = (0.4_f32, 6.0_f32);
        let s = SegmentStart::default();

        let steps = 20_000;
        let dt = tj / steps as f32;
        let (mut accel, mut vel, mut pos) = (0.0_f32, 0.0_f32, 0.0_f32);
        for i in 0..steps {
            let t = i as f32 * dt;
            let j = javp_incr_jerk(t, tj, jm, s).jerk;
            pos += vel * dt;
            vel += accel * dt;
            accel += j * dt;
        }

        let closed = javp_incr_jerk(tj, tj, jm, s);
        assert!(
            (closed.accel - accel).abs() < 0.01,
            "accel: closed {} numeric {accel}",
            closed.accel
        );
        assert!(
            (closed.vel - vel).abs() < 0.01,
            "vel: closed {} numeric {vel}",
            closed.vel
        );
        // Position is the most integrated of the three, so it carries the most
        // Euler error; the bound is looser for that reason and not because the
        // agreement is worse.
        assert!(
            (closed.pos - pos).abs() < 0.01,
            "pos: closed {} numeric {pos}",
            closed.pos
        );
    }

    /// A track built segment by segment evaluates to each segment's own end
    /// state at that segment's end time.
    #[test]
    fn a_track_evaluates_to_its_segment_endpoints() {
        let mut track = SegmentTrack::new();
        track.add_segment(Segment::default()); // segment 0, the initial state
        track.add_segment_incr_jerk(0.25, 8.0);
        track.add_segment_const_jerk(0.5, 8.0);
        track.add_segment_decr_jerk(0.25, 8.0);
        // pad to a full track so the evaluator will run
        while track.len() < SEGMENTS_MAX {
            track.add_segment_const_jerk(0.1, 0.0);
        }

        for i in 1..4 {
            let seg = track.segment(i).expect("segment");
            let at_end = track.javp_at_time(seg.end_time);
            assert!(
                (at_end.vel - seg.end_vel).abs() < 1e-3,
                "segment {i}: evaluated {} against stored {}",
                at_end.vel,
                seg.end_vel
            );
        }
    }

    /// A half-built track evaluates to nothing. Upstream returns silently
    /// because the untouched segments hold whatever was there before.
    #[test]
    fn a_half_built_track_evaluates_to_zero() {
        let mut track = SegmentTrack::new();
        track.add_segment(Segment::default());
        track.add_segment_incr_jerk(0.25, 8.0);
        assert_eq!(track.javp_at_time(0.1), Javp::default());
    }

    /// The array cannot be written past its end, whatever the caller does.
    #[test]
    fn the_segment_array_is_bounded() {
        let mut track = SegmentTrack::new();
        for _ in 0..100 {
            track.add_segment_const_jerk(0.1, 1.0);
        }
        assert_eq!(track.len(), SEGMENTS_MAX);
    }

    /// Position along a track is never negative — it is distance travelled,
    /// and behind the origin is not a place on the path.
    #[test]
    fn position_along_the_track_is_never_negative() {
        let mut track = SegmentTrack::new();
        track.add_segment(Segment::default());
        while track.len() < SEGMENTS_MAX {
            track.add_segment_const_jerk(0.1, -5.0);
        }
        let mut t = 0.0_f32;
        while t < 3.0 {
            assert!(track.javp_at_time(t).pos >= 0.0);
            t += 0.01;
        }
    }
    /// Non-positive limits cannot make a path. Upstream logs INTERNAL_ERROR
    /// and returns zeros; we skip the log and return the same zeros.
    #[test]
    fn calculate_path_rejects_non_positive_limits() {
        for (sm, jm, am, vm, length) in [
            (0.0, 8.0, 5.0, 10.0, 50.0),
            (100.0, 0.0, 5.0, 10.0, 50.0),
            (100.0, 8.0, 0.0, 10.0, 50.0),
            (100.0, 8.0, 5.0, 0.0, 50.0),
            (100.0, 8.0, 5.0, 10.0, 0.0),
            (-1.0, 8.0, 5.0, 10.0, 50.0),
        ] {
            assert_eq!(
                calculate_path(sm, jm, 0.0, am, vm, length),
                PathTimes::default()
            );
        }
    }

    /// Already at or above cruise: there is no speed change to schedule.
    #[test]
    fn calculate_path_is_empty_when_already_at_cruise() {
        assert_eq!(
            calculate_path(100.0, 8.0, 10.0, 5.0, 10.0, 50.0),
            PathTimes::default()
        );
        assert_eq!(
            calculate_path(100.0, 8.0, 15.0, 5.0, 10.0, 50.0),
            PathTimes::default()
        );
    }

    /// A long cruise from rest uses every segment: rise, hold jerk, hold
    /// accel, and the mirror on the way down.
    #[test]
    fn a_long_cruise_from_rest_uses_every_segment() {
        let p = calculate_path(100.0, 8.0, 0.0, 5.0, 15.0, 200.0);
        assert!(is_positive(p.jm), "jm {}", p.jm);
        assert!(is_positive(p.tj), "tj {}", p.tj);
        assert!(is_positive(p.t2), "t2 {}", p.t2);
        assert!(is_positive(p.t4), "t4 {}", p.t4);
        assert!(is_positive(p.t6), "t6 {}", p.t6);
        assert_eq!(p.t2, p.t6, "solution 7 mirrors t2 and t6");
    }

    /// A short hop from rest never reaches the jerk or accel limits, so
    /// the constant-jerk and constant-accel holds stay at zero.
    #[test]
    fn a_short_hop_from_rest_is_all_raised_cosine() {
        let p = calculate_path(100.0, 40.0, 0.0, 10.0, 20.0, 1.0);
        assert!(is_positive(p.jm) || p == PathTimes::default());
        if p != PathTimes::default() {
            assert_eq!(p.t2, 0.0);
            assert_eq!(p.t4, 0.0);
            assert_eq!(p.t6, 0.0);
            assert!(is_positive(p.tj));
        }
    }

    /// Durations the solver returns are never negative. A negative hold
    /// would walk the vehicle backwards along its own track.
    #[test]
    fn calculate_path_never_returns_a_negative_duration() {
        for sm in [10.0, 100.0, 400.0] {
            for jm in [1.0, 8.0, 40.0] {
                for v0 in [0.0, 5.0] {
                    for am in [1.0, 5.0] {
                        for vm in [5.0, 15.0] {
                            for length in [1.0, 50.0, 200.0] {
                                let p = calculate_path(sm, jm, v0, am, vm, length);
                                assert!(
                                    p.jm >= 0.0
                                        && p.tj >= 0.0
                                        && p.t2 >= 0.0
                                        && p.t4 >= 0.0
                                        && p.t6 >= 0.0
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    fn init_track() -> SegmentTrack {
        let mut track = SegmentTrack::new();
        track.add_segment(Segment::default());
        track
    }

    /// A zero length is a no-op: the init segment stays alone.
    #[test]
    fn add_segments_of_zero_length_changes_nothing() {
        let mut track = init_track();
        track.add_segments(100.0, 8.0, 5.0, 15.0, 0.0);
        assert_eq!(track.len(), 1);
        assert!(!track.valid());
    }

    /// A long cruise fills all 23 slots, finishes at rest, and the last
    /// stored position is the path length — that is the whole point of
    /// asking `calculate_path` for half and putting a cruise in the middle.
    #[test]
    fn add_segments_builds_a_full_track() {
        let length = 200.0;
        let mut track = init_track();
        track.add_segments(100.0, 8.0, 5.0, 15.0, length);
        assert_eq!(track.len(), SEGMENTS_MAX);
        assert!(track.valid(), "long cruise must be a valid path");

        let last = track.segment(SEG_DECEL_END).expect("decel end");
        assert!(is_zero(last.end_accel), "accel {}", last.end_accel);
        assert!(is_zero(last.end_vel), "vel {}", last.end_vel);
        assert!(
            (last.end_pos - length).abs() < 0.05,
            "end_pos {} against length {length}",
            last.end_pos
        );

        let cruise = track.segment(SEG_CONST).expect("cruise");
        let speed_end = track.segment(SEG_SPEED_CHANGE_END).expect("speed-change end");
        assert!(
            cruise.end_time > speed_end.end_time,
            "a 200 m cruise at 15 m/s must spend time at constant speed"
        );
    }

    /// A short hop never reaches cruise, but it is still 23 segments and
    /// still valid — the empty slots are zero-duration copies.
    #[test]
    fn add_segments_short_hop_is_still_twenty_three() {
        let length = 2.0;
        let mut track = init_track();
        track.add_segments(100.0, 40.0, 10.0, 20.0, length);
        assert_eq!(track.len(), SEGMENTS_MAX);
        assert!(track.valid());
        let last = track.segment(SEG_DECEL_END).expect("decel end");
        assert!(is_zero(last.end_vel));
        assert!(
            (last.end_pos - length).abs() < 0.05,
            "end_pos {} against length {length}",
            last.end_pos
        );
    }

    /// The evaluator on a built track agrees with the stored endpoints
    /// and finishes at rest.
    #[test]
    fn add_segments_evaluates_to_rest_at_the_end() {
        let mut track = init_track();
        track.add_segments(100.0, 8.0, 5.0, 15.0, 80.0);
        let last = track.segment(SEG_DECEL_END).expect("decel end");
        let at_end = track.javp_at_time(last.end_time);
        assert!(at_end.vel.abs() < 1e-3, "vel {}", at_end.vel);
        assert!(at_end.accel.abs() < 1e-3, "accel {}", at_end.accel);
        assert!((at_end.pos - last.end_pos).abs() < 1e-3);
    }

    /// Equal origin and destination leave a zero-length path.
    #[test]
    fn calculate_track_equal_points_is_empty() {
        let mut s = SCurve::new();
        s.calculate_track(
            Vector3f::zero(),
            Vector3f::zero(),
            0.0,
            15.0,
            5.0,
            5.0,
            5.0,
            5.0,
            5.0,
            100.0,
            8.0,
        );
        assert!(!s.valid());
        assert_eq!(s.track().len(), 1);
        assert!(is_zero(s.seg_length()));
        assert!(s.seg_delta().is_zero());
    }

    /// A 100 m east leg is a straight 23-segment track whose length is
    /// the chord.
    #[test]
    fn calculate_track_straight_horizontal() {
        let mut s = SCurve::new();
        let dest = Vector3f::new(100.0, 0.0, 0.0);
        s.calculate_track(
            Vector3f::zero(),
            dest,
            0.0,
            15.0,
            5.0,
            5.0,
            5.0,
            5.0,
            5.0,
            100.0,
            8.0,
        );
        assert!(s.valid());
        assert!(!s.is_arc_segment());
        assert!((s.seg_length() - 100.0).abs() < 1e-5);
        assert_eq!(s.seg_delta(), dest);
        assert_eq!(s.vel_max(), 15.0);
        assert_eq!(s.accel_max(), 5.0);
        let last = s.track().segment(SEG_DECEL_END).expect("decel end");
        assert!((last.end_pos - 100.0).abs() < 0.05);
    }

    /// An arc smaller than one degree is a straight chord. Below that
    /// the circle centre is a numerical fiction.
    #[test]
    fn calculate_track_tiny_arc_is_straight() {
        let mut s = SCurve::new();
        s.calculate_track(
            Vector3f::zero(),
            Vector3f::new(50.0, 0.0, 0.0),
            radians(0.5),
            15.0,
            5.0,
            5.0,
            5.0,
            5.0,
            5.0,
            100.0,
            8.0,
        );
        assert!(s.valid());
        assert!(!s.is_arc_segment());
        assert!(is_zero(s.arc().angle_rad));
        assert!(is_zero(s.arc().radius_ne));
    }

    /// A 90° arc stores the circle (radius = chord / √2) and still
    /// builds a valid scalar track of the arc length. Projection onto
    /// the circle is the later leftover.
    #[test]
    fn calculate_track_arc_sets_radius() {
        let mut s = SCurve::new();
        let dest = Vector3f::new(100.0, 0.0, 0.0);
        s.calculate_track(
            Vector3f::zero(),
            dest,
            core::f32::consts::FRAC_PI_2,
            15.0,
            5.0,
            5.0,
            5.0,
            5.0,
            5.0,
            100.0,
            8.0,
        );
        assert!(s.is_arc_segment());
        let want_r = 100.0 / core::f32::consts::SQRT_2;
        assert!(
            (s.arc().radius_ne - want_r).abs() < 1e-3,
            "radius {} against {want_r}",
            s.arc().radius_ne
        );
        let want_len = want_r * core::f32::consts::FRAC_PI_2;
        assert!((s.arc().length_ne - want_len).abs() < 1e-3);
        assert!((s.seg_length() - want_len).abs() < 1e-3);
        assert!(s.valid());
        let last = s.track().segment(SEG_DECEL_END).expect("decel end");
        assert!((last.end_pos - s.seg_length()).abs() < 0.05);
    }

    /// Non-positive snap / jerk / accel / speed leave the init-only path.
    /// Upstream logs INTERNAL_ERROR; we skip the log.
    #[test]
    fn calculate_track_rejects_non_positive_limits() {
        for (snap, jerk, accel, speed) in [
            (0.0, 8.0, 5.0, 15.0),
            (100.0, 0.0, 5.0, 15.0),
            (100.0, 8.0, 0.0, 15.0),
            (100.0, 8.0, 5.0, 0.0),
        ] {
            let mut s = SCurve::new();
            s.calculate_track(
                Vector3f::zero(),
                Vector3f::new(40.0, 0.0, 0.0),
                0.0,
                speed,
                5.0,
                5.0,
                accel,
                accel,
                accel,
                snap,
                jerk,
            );
            assert!(!s.valid(), "snap={snap} jerk={jerk} accel={accel} speed={speed}");
            assert_eq!(s.track().len(), 1);
        }
    }

    /// Straight up (NED −Z) uses the climb speed / accel, not the
    /// horizontal ones. That is `kinematic_limit` on a vertical direction.
    #[test]
    fn calculate_track_vertical_uses_climb_limit() {
        let mut s = SCurve::new();
        s.calculate_track(
            Vector3f::zero(),
            Vector3f::new(0.0, 0.0, -20.0),
            0.0,
            15.0,
            3.0,
            4.0,
            5.0,
            2.0,
            5.0,
            100.0,
            8.0,
        );
        assert!(s.valid());
        assert!(!s.is_arc_segment());
        assert!((s.seg_length() - 20.0).abs() < 1e-5);
        assert_eq!(s.vel_max(), 3.0);
        assert_eq!(s.accel_max(), 2.0);
        assert_eq!(s.accel_z_max(), 2.0);
    }
}
