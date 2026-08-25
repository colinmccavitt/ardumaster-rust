//! Loader for a reference flight's parameter set.
//!
//! An ArduPilot log records a `PARM` message for every parameter the vehicle
//! was running. Extracting them into the fixture lets a replay configure itself
//! from the flight it is replaying, rather than from values written down by
//! hand — which is how a constant 0.05 throttle offset went unnoticed, from
//! `TRIM_THROTTLE` being recorded as 45 when the flight used 50.

use std::collections::BTreeMap;
use std::path::Path;

/// A reference flight's parameters, keyed by upstream parameter name.
#[derive(Debug, Clone, Default)]
pub struct Params {
    values: BTreeMap<String, f64>,
}

impl Params {
    /// Load a two-column `name,value` CSV as written by `extract_params.py`.
    pub fn load(path: &Path) -> Result<Self, String> {
        let text =
            std::fs::read_to_string(path).map_err(|e| format!("{}: {}", path.display(), e))?;
        let mut values = BTreeMap::new();
        for (n, line) in text.lines().enumerate().skip(1) {
            let line = line.trim();
            if line.is_empty() {
                continue;
            }
            let (name, raw) = line
                .split_once(',')
                .ok_or_else(|| format!("{}:{}: expected 'name,value'", path.display(), n + 1))?;
            let value: f64 = raw
                .trim()
                .parse()
                .map_err(|e| format!("{}:{}: {:?}: {}", path.display(), n + 1, raw, e))?;
            values.insert(name.trim().to_string(), value);
        }
        if values.is_empty() {
            return Err(format!("{}: no parameters", path.display()));
        }
        Ok(Self { values })
    }

    /// A parameter's value, or `None` if the flight did not have it.
    pub fn get(&self, name: &str) -> Option<f64> {
        self.values.get(name).copied()
    }

    /// A parameter's value as `f32`.
    ///
    /// # Panics
    ///
    /// If the parameter is absent. A replay that silently substituted a default
    /// for a missing parameter would be comparing against a different vehicle
    /// than the one that flew, so this is deliberately fatal.
    pub fn f32(&self, name: &str) -> f32 {
        self.req(name) as f32
    }

    /// A parameter's value as `i8`, for upstream's `AP_Int8` parameters.
    ///
    /// # Panics
    ///
    /// If the parameter is absent, or its value does not fit in an `i8`.
    pub fn i8(&self, name: &str) -> i8 {
        let v = self.req(name);
        let r = v.round();
        assert!(
            r >= i8::MIN as f64 && r <= i8::MAX as f64,
            "parameter {} = {} does not fit in i8",
            name,
            v
        );
        r as i8
    }

    /// A parameter's value as `i32`, for upstream's `AP_Int32` parameters.
    ///
    /// # Panics
    ///
    /// If the parameter is absent, or its value does not fit in an `i32`.
    pub fn i32(&self, name: &str) -> i32 {
        let v = self.req(name);
        let r = v.round();
        assert!(
            r >= i32::MIN as f64 && r <= i32::MAX as f64,
            "parameter {} = {} does not fit in i32",
            name,
            v
        );
        r as i32
    }

    /// A parameter's value as a boolean, non-zero being true.
    ///
    /// # Panics
    ///
    /// If the parameter is absent.
    pub fn bool(&self, name: &str) -> bool {
        self.req(name) != 0.0
    }

    /// How many parameters were loaded.
    pub fn len(&self) -> usize {
        self.values.len()
    }

    /// Whether no parameters were loaded.
    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    fn req(&self, name: &str) -> f64 {
        match self.values.get(name) {
            Some(v) => *v,
            None => panic!(
                "reference flight has no parameter {:?}; \
                 the fixture may predate a rename (Plane 4.6 renamed \
                 ARSPD_FBW_MIN/MAX to AIRSPEED_MIN/MAX and TRIM_ARSPD_CM to \
                 AIRSPEED_CRUISE)",
                name
            ),
        }
    }
}
