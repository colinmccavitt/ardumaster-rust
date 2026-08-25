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

Status counts: **7 applied · 1 proposed · 1 rejected · 3 reproduced (not bugs)**

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

## D-005 — `DigitalLPF::initialised` is read uninitialised (undefined behavior)

- **status**: `applied`
- **upstream**: `Filter/LowPassFilter.h:72` declares `bool initialised;` with no default member
  initializer, and the constructor (`LowPassFilter.cpp:17-20`) sets only `output`:

  ```cpp
  template <class T>
  DigitalLPF<T>::DigitalLPF() {
    // built in initialization
    output = T();          // initialised is NOT set
  }
  ```

  It is then read at `LowPassFilter.cpp:26` (`if (!initialised)`). Reading an indeterminate
  `bool` is undefined behavior in C++.
- **ported**: `DigitalLpf::default()` sets `initialised: false` explicitly. Rust cannot express
  the bug — every field must be initialised — so the port is correct by construction.
- **why**: A defect, and not a subtle one. Upstream's intent is unambiguous: `reset()` sets the
  flag `false` specifically so the next sample re-seeds the filter. The constructor simply
  fails to establish that starting state.

  The consequence when the indeterminate byte is nonzero: the filter skips first-sample
  seeding and instead treats its zero-initialised `output` as a real previous value, ramping
  from **0** toward the signal at the filter's time constant. That is a startup transient in a
  control filter, not a cosmetic difference.

  It often works in practice because many filter instances are members of objects with static
  storage duration, which are zero-initialised before construction. Instances on the stack or
  heap have no such guarantee.
- **risk**: Low to apply — the port simply has one defined behavior instead of two possible
  ones. Fixed-wing callers affected: `LowPassFilterConstDtFloat` (3 sites),
  `LowPassFilterFloat` (2), `LowPassFilterVector3f` (1).
- **sitl_impact**: None expected. SITL builds construct these as members of statically-stored
  vehicle objects, so upstream's flag is zero-initialised there and already behaves as the port
  does. The divergence appears only where upstream's UB resolves the other way, which SITL is
  unlikely to exhibit.
- **pinned by**: `lowpass::tests::d005_fresh_filter_is_deterministically_unseeded`

## D-006 — `SlewLimiter` leaves thirteen members uninitialised

- **status**: `applied`
- **upstream**: `Filter/SlewLimiter.cpp:33` — the constructor initialises only the two
  parameter references and configures the internal filter:

  ```cpp
  SlewLimiter::SlewLimiter(const float &_slew_rate_max, const float &_slew_rate_tau) :
      slew_rate_max(_slew_rate_max),
      slew_rate_tau(_slew_rate_tau)
  {
      slew_filter.set_cutoff_frequency(DERIVATIVE_CUTOFF_FREQ);
      slew_filter.reset(0.0);
  }
  ```

  `SlewLimiter.h:36-48` then declares, with no default member initialisers:
  `_output_slew_rate`, `_modifier_slew_rate`, `last_sample`, `_max_pos_slew_rate`,
  `_max_neg_slew_rate`, `_max_pos_slew_event_ms`, `_max_neg_slew_event_ms`,
  `_pos_event_index`, `_neg_event_index`, `_pos_event_ms[2]`, `_neg_event_ms[2]`,
  `_pos_event_stored`, `_neg_event_stored`. All are read on the first `modifier()` call.
- **ported**: every field is zeroed by `Default`. Rust cannot express the bug.
- **why**: Same class as D-005, larger blast radius. `SlewLimiter` produces the gain
  multiplier that reduces PID gains when a controller oscillates, with three call sites on
  the fixed-wing path.

  The dangerous field is `last_sample`. The first call computes
  `(sample - last_sample) / dt`; with a garbage `last_sample` that derivative can be
  enormous, which latches `_max_pos_slew_rate` high. Since
  `mod = slew_rate_max / (slew_rate_max + 1.5 * (modifier_slew_rate - slew_rate_max))`,
  a large latched slew rate drives the multiplier toward **zero** — PID gains crushed at
  startup, decaying back only over `slew_rate_tau`.

  A garbage `_max_pos_slew_event_ms` compounds it, since the decay branch is gated on
  `now_ms - _max_pos_slew_event_ms > WINDOW_MS`.

  As with D-005 this usually works in practice: these limiters live inside PID objects owned
  by statically-stored vehicle objects, which are zero-initialised. Stack or heap
  construction has no such guarantee.
- **risk**: Low to apply — the port has one defined behavior where upstream has many possible
  ones. Fixed-wing call sites: 3, all in `APM_Control` PID gain limiting.
- **sitl_impact**: None expected, for the same reason as D-005 — SITL constructs these within
  statically-stored objects, so upstream is already zero-initialised there.
- **pinned by**: `slew::tests::d006_state_is_deterministically_zeroed`, which also asserts
  that a signal at rest yields a multiplier of exactly 1.0 — the check a garbage
  `last_sample` would fail.

## D-007 — `Storage` block access cannot report an out-of-range offset

- **status**: `applied`
- **upstream**: `AP_HAL/Storage.h:10-11`

  ```cpp
  virtual void read_block(void *dst, uint16_t src, size_t n) = 0;
  virtual void write_block(uint16_t dst, const void* src, size_t n) = 0;
  ```

  Both return `void` and take a raw pointer plus a separate length, so an
  offset past the end of the storage region cannot be reported. What happens is
  backend-defined: clamp, read adjacent memory, or silently do nothing.
- **ported**: both take slices, which carry their own length, and return
  `Result<()>`. An out-of-range or overflowing offset is `Err`, and a rejected
  write applies nothing.
- **why**: This backs the parameter system (FW-004) and mission storage. A
  silently truncated or misdirected parameter write is the kind of fault that
  surfaces much later as an inexplicable configuration change, with nothing in
  the logs tying it to the write that caused it.

  This is a shape change more than a behavior change — backends that currently
  succeed still succeed — but it is registered rather than filed as a doc note,
  because converting silent undefined behavior into a reported error **is**
  observable to a caller, and that is the bar ADR-0007 sets for the register.
- **risk**: Low. No ported caller exists yet; FW-004 will be the first, and it
  gets the checked API from the start rather than being retrofitted.
- **sitl_impact**: None. Upstream's SITL backend is file-backed and sized to the
  parameter region, so in-range access is the only case exercised.
- **pinned by**: `storage::tests::d007_out_of_range_access_is_reported_not_silent`,
  which also asserts a rejected write leaves the region untouched.

## D-008 — `_update_height_demand` divides by an unguarded time constant

- **status**: `applied`
- **upstream**: `AP_TECS.cpp`, in `_update_height_demand`. Two *adjacent* lines
  treat the same value differently:

  ```cpp
  const float coef = MIN(_DT / (_DT + MAX(_hgt_dem_tconst, _DT)), 1.0f);
  _hgt_rate_dem = (_hgt_dem_rate_ltd - _hgt_dem_lpf) / _hgt_dem_tconst;
  ```

  The first guards the denominator with `MAX(_hgt_dem_tconst, _DT)`. The second
  divides by the raw parameter.
- **ported**: applies the same `MAX(tconst, dt)` guard on the second line.
- **why**: `TECS_HDEM_TCONST` is documented `@Range: 1.0 5.0`, but `AP_Param`
  does not enforce ranges at runtime — the range is GCS metadata. A value of
  zero therefore reaches the division and produces `inf` in `_hgt_rate_dem`,
  which feeds the pitch demand.

  The guard on the immediately preceding line is the strongest possible
  evidence this is an oversight rather than a decision: the author guarded the
  same quantity one line earlier, in the same function, for the same reason.
- **risk**: **Very low, and provably so.** For any `tconst` at or above `dt`,
  `MAX(tconst, dt) == tconst`, so the arithmetic is bit-identical to upstream.
  With the documented range starting at 1.0 and a typical `dt` around 0.02, no
  in-range configuration is affected. The divergence exists only where upstream
  divides by approximately zero.
- **sitl_impact**: None. Reference flights use in-range parameters, where the
  guard is a no-op. Confirmed by test: the in-range case is asserted to match
  upstream's arithmetic exactly.
- **pinned by**: `height::tests::d008_height_rate_demand_survives_zero_time_constant`,
  which asserts both that a zero `tconst` yields a finite result **and** that an
  in-range `tconst` produces upstream's exact value.

## D-009 — takeoff minimum pitch is converted from centidegrees twice

- **status**: `applied`
- **upstream**: `AP_TECS.cpp:1562`, in `_update_pitch_limits`:

  ```cpp
  // Apply TAKEOFF minimum pitch
  if (_flight_stage == TAKEOFF || _flight_stage == ABORT_LANDING) {
      _PITCHminf = cd_to_rad(ptchMinCO_cd);      // centidegrees -> RADIANS
  }
  ...
  // convert to radians
  _PITCHminf = radians(_PITCHminf);              // applied again, as if degrees
  ```

- **ported**: `ptchMinCO_cd * 0.01` — centidegrees to **degrees** — so the single
  trailing `radians()` conversion is correct.
- **why**: The whole function works in degrees and converts once at the end.
  Every other assignment to `_PITCHminf`/`_PITCHmaxf` is degrees:

  | source | units |
  |---|---|
  | `aparm.pitch_limit_min` / `_max` | `@Units: deg` |
  | `TECS_PITCH_MIN` / `_MAX` | degrees |
  | `pitch_limit_deg` in the flare | degrees, `0.01 * get_pitch_cd()` |
  | `_PITCHminf_ext` / `_PITCHmaxf_ext` | `-90.0` / `90.0`, degrees |
  | `flare_pitch_range` | `20`, degrees |

  The takeoff branch is the only one producing radians, and it is then treated
  as degrees. The result is scaled by `pi/180`: **a configured 10° climbout
  minimum becomes 0.175°**, a factor of 57.3 too small.

  This also feeds the takeoff pitch bias in `_update_pitch`
  (`SEBdot_dem_total += _PITCHminf * gainInv`), which becomes negligible instead
  of biasing the demand toward the climbout minimum.

- **risk**: **Latent in normal operation, but safety-relevant when it binds.**
  During a normal takeoff the height demand is far above the aircraft, so TECS
  commands high pitch anyway and the *minimum* limit never becomes active. The
  bug matters exactly when TECS wants to lower the nose during climbout — an
  overspeed, for instance — which is precisely the case the climbout minimum
  exists to prevent.

  Applying the fix restores the configured limit. That is a real behaviour
  change during TAKEOFF and ABORT_LANDING, not a no-op like D-008.

- **sitl_impact**: **Expected, and must not be misread as a port defect.** The
  reference flight (`test.Plane.ClimbBeforeTurn`) includes a takeoff, so the
  replay may diverge during that phase. Any divergence there should be checked
  against this entry *first*. Outside TAKEOFF and ABORT_LANDING the ported code
  is identical to upstream.

- **pinned by**: `limits::tests::d009_takeoff_pitch_min_uses_degrees`, which
  asserts the ported value matches the configured climbout minimum in radians,
  and separately shows what upstream's double conversion produces.

- **note**: This is the first divergence found that changes behaviour in a
  normal flight phase. Worth reporting upstream if the user chooses to; the
  register is the artifact for that.

---

## D-010 — `crc_crc64` byte order depends on the compilation target

- **status**: `applied`
- **upstream**: `AP_Math/crc.cpp`, in `crc_crc64`:

  ```cpp
  uint32_t value = *data++;
  for (uint8_t j = 0; j < 4; j++) {
      uint8_t byte = ((uint8_t *)&value)[j];
  ```

  Reading a `uint32_t` through a `uint8_t*` yields the host's byte order, so
  the checksum this function computes is a property of the machine it was
  compiled for, not of the data.
- **ported**: `value.to_le_bytes()`, which states little-endian explicitly.
- **why**: the function's comment says it matches the PX4 bootloader, and that
  format is little-endian. Every ArduPilot target is little-endian, so
  little-endian is what upstream has always computed and what the format means.
  Rust has no equivalent of the C aliasing trick that would carry the
  target-dependence over, and reproducing it deliberately would mean writing
  target-conditional code to preserve an outcome nobody wants.
- **risk**: **None on any supported target.** On little-endian — which is every
  board ArduPilot builds for — the two are byte-identical. They differ only on
  a big-endian target, where upstream would produce a value no PX4 bootloader
  would accept, and the port produces the correct one.
- **sitl_impact**: None. SITL hosts are little-endian.
- **pinned by**: `crc_parity::every_crc_matches_upstream`, which compares
  against values produced by compiling and running upstream's own `crc.cpp` on
  this (little-endian) host — so the equivalence is measured, not assumed.

## D-011 — `Polygon_closest_distance_line` iterates its edges with a `uint8_t`

- **status**: `applied`
- **upstream**: `AP_Math/polygon.cpp`, in `Polygon_closest_distance_line`:

  ```cpp
  float Polygon_closest_distance_line(const Vector2f *V, unsigned N, ...)
  {
      ...
      for (uint8_t i=0; i<N-1; i++) {
          const Vector2f &v1 = V[i];
          const Vector2f &v2 = V[i+1];
  ```

  `N` is `unsigned`; `i` is `uint8_t`. Two ways that goes wrong:

  1. **`N == 0`** makes `N-1` wrap to `UINT_MAX`. The loop reads past the end
     of the array and cannot terminate.
  2. **`N >= 257`** makes `i` wrap from 255 back to 0 before it ever reaches
     `N-1`, so the loop cannot terminate. At `N == 256` it happens to be
     correct, which is what makes this easy to miss.
- **ported**: iterates the slice's consecutive pairs. Neither wrap exists, and
  with fewer than two points there are simply no edges.
- **why**: an infinite loop in the flight path. `AP_OABendyRuler::calc_margin_
  from_inclusion_and_exclusion_polygons` reads its count as
  `uint16_t num_points` straight from `AC_PolyFence_loader::get_inclusion_
  polygon()` and passes it here, so a boundary of more than 256 points is
  representable by the types involved rather than being ruled out upstream.
  Whether a given airframe's fence storage can hold one is a separate question
  — the point is that nothing between the fence loader and this loop prevents
  it, and the failure mode is a hang rather than a wrong answer.
- **risk**: **None for any input upstream handles correctly.** For
  `2 <= N <= 256` the port walks exactly the same edges in the same order and
  returns bit-identical results — confirmed across the whole parity fixture.
  The divergence exists only where upstream hangs or reads out of bounds.
- **sitl_impact**: None. Reference fences are far below 256 points.
- **pinned by**: `polygon::tests::d011_closest_distance_line_terminates_without_edges`
  and `polygon::tests::d011_closest_distance_line_handles_more_than_256_points`,
  which cover `N == 0`, `N == 1` and `N == 400`. Both would hang under
  upstream's loop rather than fail, which is the point.

### Related, not separately registered

`Polygon_intersects` has the same `uint8_t` counter against an `unsigned N`, so
it too cannot terminate for `N >= 257`. The port's slice iteration fixes it by
construction, and it is covered by the same reasoning and the same fixture, so
it is recorded here rather than given its own id.

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

## Open questions — not divergences, but not resolved either

Suspected upstream defects where the **correct** behaviour cannot be determined
from the source. The port reproduces upstream exactly, so there is no divergence
to register; these are recorded so the question is not lost.

- **`crc8_table_rds02uf` has a one-byte collision.** The table is bijective
  except that `0x06` appears at both index 128 and index 202, and `0xA6` appears
  nowhere. One duplicate and one omission in an otherwise-bijective table is
  what a single mistyped nibble looks like — `0xA6` entered as `0x06`.

  It cannot be repaired from the source. Either index 128 or index 202 should
  hold `0xA6`, and nothing decides which: the table is a vendor S-box rather
  than a polynomial table — verified, it is not GF(2)-linear, either as
  published or with the candidate correction — so its entries cannot be
  re-derived the way the other six tables can. Changing the wrong one would
  break interoperability with hardware that works today, so the port changes
  nothing.

  Upstream cannot observe it: `AP_RangeFinder_RDS02UF` and the SITL model
  `SIM_RF_RDS02UF` both call `crc8_rds02uf`, so simulated frames always validate
  against the same table that produced them, whatever it contains. The effect on
  real hardware would be that frames whose running CRC passes through the
  affected index are rejected.

  Resolving it needs the RDS02UF vendor protocol document. Pinned meanwhile by
  `crc::tests::rds02uf_table_has_upstreams_one_byte_collision`, which asserts
  the exact anomaly, so an upstream edit to the table fails the build rather
  than passing unnoticed.

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
