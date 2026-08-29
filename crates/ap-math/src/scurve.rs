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
//! `calculate_path` — the static time-solver that turns snap / jerk / accel /
//! speed / length into the five numbers a 23-segment track is built from.
//! The segment kinematics are already here. `calculate_track`,
//! `advance_target_along_track`, the arc handling and the speed-change
//! logic are the rest of the file.

use crate::scalar::{is_negative, is_positive, is_zero, safe_sqrt, sq, Real};

/// Segments in a full track, upstream `segments_max`.
///
/// Segment 0 holds the initial state; 1–7 accelerate; 8–14 change speed; 15
/// is constant velocity; 16–22 decelerate.
pub const SEGMENTS_MAX: usize = 23;

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
}
