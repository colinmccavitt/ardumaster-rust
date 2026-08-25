//! Log-replay differential testing support (ADR-0008).
//!
//! Loads fixtures extracted from an upstream dataflash log, drives a ported
//! module with the recorded inputs, and compares its outputs against
//! upstream's own recorded outputs.
//!
//! This crate is **test support**, not flight code: it is `std`, it reads
//! files, and nothing in the flight path may depend on it. That is why it is a
//! separate crate rather than a module inside `ap-math` — the `no_std`
//! guarantee in ADR-0004 should not need a `cfg` escape hatch to hold.
//!
//! # Why fixtures rather than a live simulator
//!
//! FW-007 measured that two runs of the *same* upstream binary flying the same
//! autotest diverge by up to 349° of yaw, because MAVLink commands land at
//! wall-clock times and closed-loop dynamics amplify the jitter. A recorded
//! fixture has no such problem: the inputs are fixed data, so a replay is
//! reproducible by construction and any output difference is attributable to
//! the port.
//!
//! # Fixtures must come from a single atomic message
//!
//! A fixture is only sound when inputs and outputs were logged in the same
//! record at the same instant. Joining two streams by nearest timestamp does
//! not work — `XKQ` and `ATT` are both 5 Hz but unsynchronised, giving ~40 ms
//! of skew, over which the aircraft genuinely rotates. Such a comparison
//! measures the skew, not the port.

mod params;

pub use params::Params;

use std::collections::HashMap;
use std::fs;
use std::path::Path;

/// One row of a replay fixture: a timestamp, named inputs, named outputs.
#[derive(Debug, Clone)]
pub struct Row {
    /// Simulated time of the record, in microseconds.
    pub time_us: u64,
    /// Input fields, keyed without the `in_` prefix.
    pub inputs: HashMap<String, f64>,
    /// Upstream's own output fields, keyed without the `out_` prefix.
    pub outputs: HashMap<String, f64>,
}

impl Row {
    /// An input field, or a clear panic naming the missing column.
    pub fn input(&self, name: &str) -> f64 {
        *self
            .inputs
            .get(name)
            .unwrap_or_else(|| panic!("fixture has no input column '{name}'"))
    }

    /// An upstream output field.
    pub fn output(&self, name: &str) -> f64 {
        *self
            .outputs
            .get(name)
            .unwrap_or_else(|| panic!("fixture has no output column '{name}'"))
    }
}

/// A loaded fixture.
#[derive(Debug, Clone)]
pub struct Fixture {
    /// Where it came from, for failure messages.
    pub name: String,
    /// Records in log order.
    pub rows: Vec<Row>,
}

impl Fixture {
    /// Load a CSV fixture produced by `tools/sitl_diff/extract_fixtures.py`.
    ///
    /// Columns are `time_us`, then `in_*` and `out_*`. Parsed by hand rather
    /// than with a CSV crate: the format is fixed, and a test-support
    /// dependency that can break the build is not worth the convenience.
    pub fn load(path: impl AsRef<Path>) -> Result<Self, String> {
        let path = path.as_ref();
        let text = fs::read_to_string(path)
            .map_err(|e| format!("cannot read fixture {}: {e}", path.display()))?;

        let mut lines = text.lines();
        let header = lines
            .next()
            .ok_or_else(|| format!("fixture {} is empty", path.display()))?;
        let cols: Vec<&str> = header.split(',').map(|s| s.trim()).collect();

        let mut rows = Vec::new();
        for (n, line) in lines.enumerate() {
            if line.trim().is_empty() {
                continue;
            }
            let vals: Vec<&str> = line.split(',').map(|s| s.trim()).collect();
            if vals.len() != cols.len() {
                return Err(format!(
                    "{}: row {} has {} fields, header has {}",
                    path.display(),
                    n + 2,
                    vals.len(),
                    cols.len()
                ));
            }

            let mut time_us = 0u64;
            let mut inputs = HashMap::new();
            let mut outputs = HashMap::new();
            for (c, v) in cols.iter().zip(vals.iter()) {
                if *c == "time_us" {
                    time_us = v.parse().map_err(|e| {
                        format!("{}: row {}: bad time_us '{v}': {e}", path.display(), n + 2)
                    })?;
                } else if let Some(k) = c.strip_prefix("in_") {
                    inputs.insert(k.to_string(), parse_f64(v, path, n + 2)?);
                } else if let Some(k) = c.strip_prefix("out_") {
                    outputs.insert(k.to_string(), parse_f64(v, path, n + 2)?);
                }
            }
            rows.push(Row {
                time_us,
                inputs,
                outputs,
            });
        }

        Ok(Self {
            name: path
                .file_stem()
                .map(|s| s.to_string_lossy().into_owned())
                .unwrap_or_else(|| "fixture".into()),
            rows,
        })
    }

    /// Number of records.
    pub fn len(&self) -> usize {
        self.rows.len()
    }

    /// Whether the fixture has no records.
    pub fn is_empty(&self) -> bool {
        self.rows.is_empty()
    }
}

fn parse_f64(v: &str, path: &Path, line: usize) -> Result<f64, String> {
    v.parse::<f64>()
        .map_err(|e| format!("{}: line {}: bad float '{v}': {e}", path.display(), line))
}

/// Accumulates per-field differences between the port and upstream.
///
/// Reports the **worst** case and where it happened, rather than failing on the
/// first mismatch: knowing that throttle diverges by 0.3 at t=41.2 s is far
/// more useful than knowing row 12 differed.
#[derive(Debug, Default)]
pub struct Comparison {
    field: String,
    tolerance: f64,
    compared: usize,
    exceeded: usize,
    worst: f64,
    worst_at_us: u64,
    worst_expected: f64,
    worst_actual: f64,
}

impl Comparison {
    /// Compare one output field at the given absolute tolerance.
    pub fn new(field: &str, tolerance: f64) -> Self {
        Self {
            field: field.to_string(),
            tolerance,
            ..Default::default()
        }
    }

    /// Record one sample: what upstream logged, and what the port produced.
    pub fn sample(&mut self, time_us: u64, expected: f64, actual: f64) {
        self.compared += 1;
        let d = (expected - actual).abs();
        if d > self.tolerance {
            self.exceeded += 1;
        }
        if d > self.worst {
            self.worst = d;
            self.worst_at_us = time_us;
            self.worst_expected = expected;
            self.worst_actual = actual;
        }
    }

    /// How many samples were compared.
    ///
    /// Callers should assert this is non-trivial: a comparison that saw no
    /// samples reports `passed()` as true, which is vacuous.
    pub fn compared(&self) -> usize {
        self.compared
    }

    /// Whether every sample stayed inside tolerance.
    pub fn passed(&self) -> bool {
        self.exceeded == 0
    }

    /// A report suitable for an assertion message.
    pub fn report(&self) -> String {
        format!(
            "{}: {} samples, {} exceeded tol {:.6}; worst |delta| {:.6} at t={:.3}s \
             (upstream {:.6}, port {:.6})",
            self.field,
            self.compared,
            self.exceeded,
            self.tolerance,
            self.worst,
            self.worst_at_us as f64 / 1e6,
            self.worst_expected,
            self.worst_actual,
        )
    }

    /// Assert the comparison passed, reporting the worst case if not.
    pub fn assert_passed(&self) {
        assert!(self.passed(), "{}", self.report());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn fixtures_dir() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .parent()
            .and_then(|p| p.parent())
            .map(|p| p.join("fixtures"))
            .expect("workspace root")
    }

    /// The TECS fixture is the first real replay target (FW-015). Proving it
    /// loads and is well-formed now means that ticket lands with its oracle
    /// already in place rather than having to build one.
    #[test]
    fn tecs_fixture_loads_and_is_well_formed() {
        let path = fixtures_dir().join("tecs.csv");
        if !path.exists() {
            eprintln!("skipping: {} not present", path.display());
            return;
        }
        let f = Fixture::load(&path).expect("tecs fixture should load");
        assert!(
            f.len() > 500,
            "expected a substantial flight, got {}",
            f.len()
        );

        // every row carries the full TECS input set and both outputs
        for r in &f.rows {
            for k in ["h", "dh", "hdem", "spdem", "sp", "pmin", "pmax"] {
                assert!(r.inputs.contains_key(k), "row missing input {k}");
            }
            for k in ["th", "ph"] {
                assert!(r.outputs.contains_key(k), "row missing output {k}");
            }
        }

        // timestamps must advance, or the fixture is not a time series
        let mut prev = 0u64;
        for r in &f.rows {
            assert!(r.time_us > prev, "timestamps must be strictly increasing");
            prev = r.time_us;
        }

        // upstream throttle is a normalised demand
        for r in &f.rows {
            let th = r.output("th");
            assert!(
                (-1.0..=1.0).contains(&th),
                "throttle {th} outside [-1,1] at t={}",
                r.time_us
            );
        }
    }

    #[test]
    fn pid_fixture_loads() {
        let path = fixtures_dir().join("pid_roll.csv");
        if !path.exists() {
            eprintln!("skipping: {} not present", path.display());
            return;
        }
        let f = Fixture::load(&path).expect("pid fixture should load");
        assert!(f.len() > 500);
        let first = f.rows.first().expect("fixture should have rows");
        assert!(first.inputs.contains_key("Tar"));
        assert!(first.outputs.contains_key("Dmod"));
    }

    /// The comparator must report the WORST case and where, not just pass/fail.
    #[test]
    fn comparison_reports_worst_case_and_location() {
        let mut c = Comparison::new("throttle", 0.01);
        c.sample(1_000_000, 0.50, 0.505);
        c.sample(2_000_000, 0.60, 0.900);
        c.sample(3_000_000, 0.70, 0.702);

        assert!(!c.passed());
        let r = c.report();
        assert!(r.contains("throttle"), "{r}");
        assert!(r.contains("3 samples"), "{r}");
        assert!(r.contains("1 exceeded"), "{r}");
        // worst is the 0.3 at t=2s, not the first failure encountered
        assert!(
            r.contains("2.000"),
            "worst location should be t=2.000s: {r}"
        );
    }

    #[test]
    fn comparison_passes_within_tolerance() {
        let mut c = Comparison::new("pitch", 0.05);
        c.sample(1_000_000, 0.10, 0.12);
        c.sample(2_000_000, 0.20, 0.18);
        assert!(c.passed());
        c.assert_passed();
    }

    #[test]
    fn malformed_fixture_is_reported_not_panicked() {
        let dir = std::env::temp_dir().join("ap_replay_test");
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("bad.csv");
        std::fs::write(&p, "time_us,in_a,out_b\n1,2\n").unwrap();
        let err = Fixture::load(&p).unwrap_err();
        assert!(err.contains("has 2 fields"), "{err}");
    }
}
