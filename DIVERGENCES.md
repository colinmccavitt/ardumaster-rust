# Divergence register

Every intentional difference between this port and ArduPilot `Plane-4.7.0`.

Governed by **ADR-0007**. Behavioral equivalence is the default; each entry here is a
documented exception. A difference from upstream that is *not* listed here is a **defect**,
not a feature — that distinction is what keeps `sitl-diff` meaningful.

Rules:

- Reference the `D-NNN` id in the code at the point of difference.
- Pin it with a test that asserts the new behavior **and** records the upstream behavior it
  replaces, so nobody "restores parity" by accident.
- `proposed` entries are **not applied**. High-blast-radius changes need explicit sign-off.

Status counts: **2 applied · 1 proposed · 1 rejected · 3 reproduced (not bugs)**

---

## D-001 — `Vector3::angle_to` returns 0 for antiparallel vectors

- **status**: `applied`
- **upstream**: `AP_Math/vector3.cpp`, `angle()` — `if (cosv >= 1 || cosv <= -1) return 0;`
- **ported**: returns `PI` for the antiparallel case, matching `Vector2::angle_to`
- **why**: A defect, and internally inconsistent. `vector2.cpp:145` handles the two
  out-of-domain ends separately and correctly (`cosv >= 1 → 0`, `cosv <= -1 → M_PI`);
  `vector3.cpp` collapses them into one `return 0`. The angle between opposed vectors is π,
  not 0. Upstream's own Vector3 `angle` test expects `M_PI` — and is **commented out**
  (`tests/test_vector3.cpp:140-149`), strong evidence the test was written correctly, failed
  against the implementation, and was disabled rather than fixed.
- **risk**: Low in practice, but **not zero — and the consequence is safety-relevant.**

  An initial "no callers" assessment was **wrong**. The two-argument `Vector3::angle` has a
  real caller: `AP_Compass.cpp:2243`, inside `Compass::consistent()`:

  ```cpp
  const float xyz_ang_diff = mag_field.angle(primary_mag_field);
  if (xyz_ang_diff > AP_COMPASS_MAX_XYZ_ANG_DIFF) { return false; }
  ```

  With the upstream bug, two compasses pointing in **exactly opposite directions** yield
  `xyz_ang_diff == 0` — reported as perfectly consistent when they are maximally
  inconsistent. A compass wired or oriented backwards could pass this gross-misalignment
  check.

  In practice the defect is **masked** at that call site: an antiparallel 3D pair is also
  antiparallel in xy, and the following `xy_ang_diff` check uses `Vector2::angle`, which
  handles the antiparallel case correctly and rejects it. The zero-xy escape is guarded
  earlier by `mag_field_xy.is_zero()`. So the bug is latent there rather than exploitable —
  but it is latent behind a coincidence, not a design.

  Fixing it makes `Compass::consistent()` reject such a pair on the xyz check directly,
  which is the intended behavior.
- **sitl_impact**: None on the fixed-wing control path — no caller in `ArduPlane`,
  `AP_TECS`, `AP_L1_Control`, `APM_Control`, `AP_AHRS` or `AP_Navigation`. Relevant when
  FW-014 (`AP_Compass`) is ported: the ported `consistent()` will reject an antiparallel pair
  one check earlier than upstream. Same verdict, different branch.
- **pinned by**: `vector3::tests::d001_angle_to_antiparallel_returns_pi`

## D-002 — `normalized()` on a zero vector produces NaN

- **status**: `applied`
- **upstream**: `AP_Math/vector2.cpp`, `vector3.h` — `return *this / length();` with no guard.
  A zero vector yields NaN components, which then propagate silently.
- **ported**: `normalized()` returns `Option<Self>`, `None` for a zero-length vector.
  `normalize()` returns `bool`. A `normalized_or_zero()` helper covers callers that genuinely
  want the old lenient shape without NaN.
- **why**: Normalizing a zero vector is mathematically undefined; producing NaN and continuing
  is the worst available option, because it corrupts downstream state with no signal at the
  point of failure. Upstream itself disagrees with itself here: `QuaternionT::normalize`
  *does* guard the zero case and raises `INTERNAL_ERROR(flow_of_control)`. The vector types
  simply lack that guard. Making it visible in the type is the single clearest reason to port
  to Rust at all.
- **risk**: Medium — API shape change, affects every future caller. Applied now precisely
  because there are no downstream consumers yet; the cost only grows.
- **sitl_impact**: None directly. Prevents NaN propagation that upstream would silently carry
  into a trajectory, so if it ever fires, the port stops where upstream would have continued
  with corrupt state. That is the intended difference.
- **pinned by**: `vector2::tests::d002_normalized_zero_is_none`,
  `vector3::tests::d002_normalized_zero_is_none`

---

## D-003 — `is_zero()` compares doubles against `FLT_EPSILON`

- **status**: `rejected` — reproduce upstream, do not change
- **upstream**: `AP_Math/ftype.h:70` — `is_zero(double x)` returns `fabs(x) < FLT_EPSILON`,
  i.e. `1.19e-7`, not `DBL_EPSILON` (`2.22e-16`).
- **considered**: comparing against the value's own type epsilon, making `is_zero` consistent
  with `is_equal`, which uses the common type's epsilon (`AP_Math.cpp:32`).
- **why it looked like a defect**: A double carrying `1e-10` is reported as zero, nine orders
  of magnitude above `DBL_EPSILON`, while `is_equal(1e-10, 0.0)` returns `false`. The same
  library appears to contradict itself.
- **why it is REJECTED**: The inconsistency is protective, and "fixing" it would move
  numerical behavior in the dangerous direction.

  `is_zero` underpins `is_positive`, which gates divide-by-zero guards throughout the
  library. The prevailing shape is:

  ```cpp
  const T len = length();
  if ((len > max_length) && is_positive(len)) {
      x *= (max_length / len);   // guarded division
  }
  ```

  A **looser** threshold means more near-zero values are treated as zero, so the guard fires
  more often and the division is **skipped**. Tightening to `DBL_EPSILON` would let values
  around `1e-10` pass `is_positive`, reach the division, and produce scale factors on the
  order of `1e10` instead of a skipped branch.

  So upstream's choice is the conservative one. The apparent inconsistency with `is_equal` is
  real but benign: `is_equal` answers "are these the same number", a precision question, while
  `is_zero`/`is_positive` answer "is this safe to divide by", a physical-magnitude question.
  Different questions legitimately take different thresholds.

  Supporting evidence that it is deliberate: the double branch explicitly writes `FLT_EPSILON`
  under `#if AP_MATH_ALLOW_DOUBLE_FUNCTIONS`, rather than inheriting a default.
- **risk if applied**: High, and in the wrong direction — numerical blow-up where upstream
  skips a branch, across the estimator and controllers.
- **decision**: Reproduce upstream. Revisit only with a concrete estimator problem that traces
  to this threshold, and treat any future change as a controls decision rather than a
  correctness cleanup.

## D-004 — no internal-error channel for reported-but-unhandled conditions

- **status**: `proposed` — needs a design decision
- **upstream**: raises `INTERNAL_ERROR(...)` at conditions that are "shouldn't happen" but
  recoverable: `constrain_value` receiving NaN (`AP_Math.cpp:288`), `QuaternionT::normalize`
  on a zero quaternion.
- **would become**: a real error-reporting channel so these are observable rather than
  silently swallowed.
- **why**: The port currently drops the report. `constrain_value` still returns the midpoint
  and `normalize` still returns `false`, so behavior is preserved — but the *diagnostic* is
  lost, and upstream treats these as conditions worth recording.
- **risk**: Low to implement, but it is a cross-cutting design decision (`no_std`, no
  allocator, no singletons per ADR-0004) and should be decided once rather than per-site.
- **sitl_impact**: None. Diagnostics only.
- **recommendation**: Design it before FW-008 (AHRS), which is where these conditions start
  mattering in flight logic. Two call sites already need it.

---

## Reproduced deliberately — surprising, but not defects

Recorded so nobody re-litigates them.

- **`constrain_value` maps NaN to the midpoint** (`AP_Math.cpp:288`). Deliberate and
  load-bearing: it exists to stop float errors propagating through every consumer. Reproduced.
  Note Rust's `clamp()` cannot be substituted — it propagates NaN and panics when
  `low > high`.
- **`operator==` on vectors compares with `is_equal`, not exactly** (`vector2.cpp:133`).
  Deliberate; upstream's own tests depend on it. It does mean equality is not transitive, so
  the ported types are intentionally not `Eq`.
- **`is_unit_length` uses a literal `1e-3` tolerance on the squared length**
  (`quaternion.cpp`). Far looser than any epsilon, but clearly chosen. Reproduced.

---

## Upstream test-quality issues — no port change

Defects in upstream's *tests*, not its code. The ported code is correct; these are recorded
because they weaken the unit-parity oracle.

- **`tests/test_vector3.cpp` lines 140-149 and 175-361 are commented out**, disabling `angle`,
  `Project`, `reflect`, `Offset_bearing`, `Perpendicular`, `closest_point`,
  `closest_distance`, `segment_intersectionx`, `circle_segment_intersectionx` and
  `point_on_segmentx`. Those methods have **no live upstream oracle**. Port tests covering
  them are labelled `PORT-DERIVED` to make the weaker evidence visible.
- **The disabled `reflect` case is itself wrong**: it expects `(-3,-3,-3)` for reflecting
  `(3,3,3)` about `(1,1,-1)` and calls them orthogonal, but their dot product is 3, not 0, and
  upstream's own algorithm yields `(-1,-1,-5)`.
- **`tests/test_matrix3.cpp` names its fixtures backwards**: `invertible[]` holds the `det==0`
  singular matrix and `non_invertible[]` holds the invertible ones, with the
  `INSTANTIATE_TEST_CASE_P` names swapped to match. The logic branches on `det == 0`, so it
  still tests the right thing.
