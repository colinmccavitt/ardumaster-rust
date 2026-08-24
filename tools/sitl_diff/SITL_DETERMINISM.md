# SITL determinism — measured, not assumed

**Status:** validated 2026-08-24. Resolves the open assumption in ADR-0005.
**Verdict:** SITL **is** replay-deterministic in simulated state, but **only** when
compared on decoded state keyed by simulated time. Raw log bytes are not reproducible.

## Why this had to be measured

ADR-0005 built the golden-trajectory approach on the assumption that SITL runs are
replayable: build upstream once, record reference trajectories, delete the build, and
compare the port against the recorded logs. The whole `sitl-diff` verification method
(ADR-0003) rests on it, and it had never been tested.

Three modules — `LowPassFilter`, `SlewLimiter` and the entire scheduler — are ported with
port-derived tests only, and every controller ticket ahead declares `sitl-diff`. If the
assumption were false, that verification plan would need replacing before more work piled
on it.

## Method

`tools/sitl_diff/determinism_test.py` runs upstream `arduplane` SITL twice with identical
inputs — same home, same speedup, same parameters, no GCS traffic beyond a socket that
drains bytes so SITL will start at all — then compares the dataflash logs.

Two things SITL needs that are not obvious, both encoded in the script:

- It blocks on `Waiting for connection` until something attaches to the SERIAL0 TCP port.
- It writes no dataflash log while disarmed unless `LOG_DISARMED=1`.

## Result 1 — raw bytes are NOT reproducible

| | |
|---|---|
| log sizes | 17,289,216 bytes both runs — **identical** |
| first difference | byte 181 |
| differing bytes | 14,179,820 of 17,289,216 — **82.02%** |

Identical size with differing content means the *structure* is reproducible — same message
count, same layout — while values differ. The bytes at the first difference decode as
microsecond timestamps around 3.04 s and 3.06 s: wall-clock metadata, not vehicle state.

**A byte-comparison oracle would have failed, and worse, would have looked like a
catastrophic port defect.**

## Result 2 — decoded state IS reproducible, exactly

`tools/sitl_diff/noise_floor.py` decodes both logs and compares state series keyed on
`TimeUS`, the simulated-time axis:

| message | shared samples | max abs delta |
|---|---|---|
| `ATT` (Roll, Pitch, Yaw) | 10,124 | **0** |
| `POS` (Lat, Lng, Alt) | 9,935 | **0** |
| `IMU` (3× gyro, 3× accel) | 20,247 | **0** |

Every field, every shared timestamp, exactly zero difference. The runs differ only in how
many samples they got before being cut off — 10,125 versus 10,124 — which is where the
process was terminated, not divergence.

**The self-divergence noise floor is zero.**

## Why it is deterministic

`AP_HAL_SITL/system.cpp:173` returns the *stopped clock* whenever it is non-zero and only
falls back to `CLOCK_MONOTONIC` otherwise:

```cpp
uint64_t stopped_usec = scheduler->stopped_clock_usec();
if (stopped_usec) { return stopped_usec; }
```

`SITL_State.cpp:79` sets it to 1 at init ("start with non-zero clock"), and
`SITL_State.cpp:250` sets it to the FDM timestamp after **every** physics step,
unconditionally. So the wall-clock branch is effectively dead in normal SITL operation, and
the flight code advances on simulated time only.

Note this is *not* the `lockstep` mechanism, which appears only in `SIM_JSON` for external
simulators. The built-in physics is stepped this way regardless.

## Consequences for the harness

1. **Compare decoded state, never bytes.** Keyed on `TimeUS`. This is now a hard rule.
2. **Exact comparison is available** for the deterministic part, which is stronger than
   ADR-0005 anticipated. Tolerances are needed only where the *port* legitimately differs
   from upstream — float semantics per ADR-0004 decision 6, and registered divergences —
   not to absorb SITL jitter, because there is none.
3. **The golden-trajectory plan works.** Reference logs can be recorded once and the
   upstream build removed, exactly as ADR-0005 proposed.
4. **Terminate on simulated time, not wall time.** The sample-count difference came from
   killing the process on a wall clock. Reference runs should end on a sim-time condition
   so the series are the same length.

## Caveats not yet tested

- Only tested **idle on the ground**, disarmed, no mission. Determinism through an armed
  flight with mode changes and mission execution is untested; a chaotic flight regime could
  still amplify a difference the idle case never exposes.
- Only tested at `--speedup 20` on one machine. Speedup should not matter given the clock
  is simulated, but that is inference, not measurement.
- Only one duration, ~20 s wall. Long-run drift is untested.

These are recorded rather than assumed away. The next FW-007 slice should re-run this
across an armed mission before reference logs are trusted for controller verification.
