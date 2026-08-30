//! Minimal JSON object parser for `Frame::load_frame_params` and
//! `SimPlane::load_coeffs`. C++ counterpart `sim_json.hpp` (CCP-046/047).
//! Original uses AP_JSON (nlohmann wrapper). Supports objects, arrays,
//! numbers, strings, bools, null, and `//` line comments. Enough for
//! `Tools/autotest/models/*.json`.

#![allow(missing_docs)]

use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

use crate::sim_plane::Vec3;

#[derive(Debug, Clone, PartialEq)]
pub enum JsonValue {
    Null,
    Number(f64),
    String(String),
    Bool(bool),
    Array(Vec<JsonValue>),
    Object(BTreeMap<String, JsonValue>),
}

impl JsonValue {
    pub fn is_null(&self) -> bool {
        matches!(self, Self::Null)
    }
    pub fn is_number(&self) -> bool {
        matches!(self, Self::Number(_))
    }
    pub fn is_array(&self) -> bool {
        matches!(self, Self::Array(_))
    }
    pub fn get(&self, key: &str) -> Option<&JsonValue> {
        match self {
            Self::Object(m) => m.get(key),
            _ => None,
        }
    }
}

pub struct JsonParser<'a> {
    s: &'a str,
    i: usize,
    err: String,
}

impl<'a> JsonParser<'a> {
    pub fn new(text: &'a str) -> Self {
        Self {
            s: text,
            i: 0,
            err: String::new(),
        }
    }
    pub fn error(&self) -> &str {
        &self.err
    }
    pub fn parse(&mut self) -> Option<JsonValue> {
        self.skip();
        let v = self.parse_value()?;
        self.skip();
        if self.i >= self.s.len() {
            Some(v)
        } else {
            self.err = "trailing junk".into();
            None
        }
    }
    fn skip(&mut self) {
        let b = self.s.as_bytes();
        while self.i < b.len() {
            let c = b[self.i];
            if c == b' ' || c == b'\n' || c == b'\r' || c == b'\t' {
                self.i += 1;
                continue;
            }
            if c == b'/' && self.i + 1 < b.len() && b[self.i + 1] == b'/' {
                self.i += 2;
                while self.i < b.len() && b[self.i] != b'\n' {
                    self.i += 1;
                }
                continue;
            }
            break;
        }
    }
    fn parse_value(&mut self) -> Option<JsonValue> {
        self.skip();
        if self.i >= self.s.len() {
            self.err = "unexpected eof".into();
            return None;
        }
        let c = self.s.as_bytes()[self.i];
        match c {
            b'{' => self.parse_object(),
            b'[' => self.parse_array(),
            b'"' => self.parse_string().map(JsonValue::String),
            b't' | b'f' => self.parse_bool(),
            b'n' => self.parse_null(),
            _ => self.parse_number(),
        }
    }
    fn parse_object(&mut self) -> Option<JsonValue> {
        self.i += 1;
        self.skip();
        let mut map = BTreeMap::new();
        if self.i < self.s.len() && self.s.as_bytes()[self.i] == b'}' {
            self.i += 1;
            return Some(JsonValue::Object(map));
        }
        while self.i < self.s.len() {
            self.skip();
            let key = self.parse_string()?;
            self.skip();
            if self.i >= self.s.len() || self.s.as_bytes()[self.i] != b':' {
                self.err = "expected :".into();
                return None;
            }
            self.i += 1;
            let child = self.parse_value()?;
            map.insert(key, child);
            self.skip();
            if self.i < self.s.len() && self.s.as_bytes()[self.i] == b',' {
                self.i += 1;
                continue;
            }
            if self.i < self.s.len() && self.s.as_bytes()[self.i] == b'}' {
                self.i += 1;
                return Some(JsonValue::Object(map));
            }
            self.err = "expected }".into();
            return None;
        }
        self.err = "unterminated object".into();
        None
    }
    fn parse_array(&mut self) -> Option<JsonValue> {
        self.i += 1;
        self.skip();
        let mut arr = Vec::new();
        if self.i < self.s.len() && self.s.as_bytes()[self.i] == b']' {
            self.i += 1;
            return Some(JsonValue::Array(arr));
        }
        while self.i < self.s.len() {
            let child = self.parse_value()?;
            arr.push(child);
            self.skip();
            if self.i < self.s.len() && self.s.as_bytes()[self.i] == b',' {
                self.i += 1;
                continue;
            }
            if self.i < self.s.len() && self.s.as_bytes()[self.i] == b']' {
                self.i += 1;
                return Some(JsonValue::Array(arr));
            }
            self.err = "expected ]".into();
            return None;
        }
        self.err = "unterminated array".into();
        None
    }
    fn parse_string(&mut self) -> Option<String> {
        self.skip();
        if self.i >= self.s.len() || self.s.as_bytes()[self.i] != b'"' {
            self.err = "expected string".into();
            return None;
        }
        self.i += 1;
        let mut out = String::new();
        let b = self.s.as_bytes();
        while self.i < b.len() {
            let c = b[self.i];
            self.i += 1;
            if c == b'"' {
                return Some(out);
            }
            if c == b'\\' && self.i < b.len() {
                out.push(b[self.i] as char);
                self.i += 1;
                continue;
            }
            out.push(c as char);
        }
        self.err = "unterminated string".into();
        None
    }
    fn parse_number(&mut self) -> Option<JsonValue> {
        self.skip();
        let start = self.i;
        let b = self.s.as_bytes();
        if self.i < b.len() && (b[self.i] == b'-' || b[self.i] == b'+') {
            self.i += 1;
        }
        while self.i < b.len() {
            let c = b[self.i];
            if c.is_ascii_digit() || c == b'.' || c == b'e' || c == b'E' || c == b'+' || c == b'-' {
                self.i += 1;
            } else {
                break;
            }
        }
        if self.i == start {
            self.err = "expected number".into();
            return None;
        }
        let n = self.s[start..self.i].parse::<f64>().unwrap_or(0.0);
        Some(JsonValue::Number(n))
    }
    fn parse_bool(&mut self) -> Option<JsonValue> {
        if self.s[self.i..].starts_with("true") {
            self.i += 4;
            return Some(JsonValue::Bool(true));
        }
        if self.s[self.i..].starts_with("false") {
            self.i += 5;
            return Some(JsonValue::Bool(false));
        }
        self.err = "expected bool".into();
        None
    }
    fn parse_null(&mut self) -> Option<JsonValue> {
        if self.s[self.i..].starts_with("null") {
            self.i += 4;
            return Some(JsonValue::Null);
        }
        self.err = "expected null".into();
        None
    }
}

pub fn load_json_file(path: &Path) -> Result<JsonValue, String> {
    let text = fs::read_to_string(path).map_err(|e| format!("cannot open {}: {e}", path.display()))?;
    let mut p = JsonParser::new(&text);
    p.parse().ok_or_else(|| p.error().to_string())
}

pub fn json_get_float(obj: &JsonValue, key: &str, dest: &mut f32) -> bool {
    match obj.get(key) {
        Some(JsonValue::Number(n)) => {
            *dest = *n as f32;
            true
        }
        _ => false,
    }
}

pub fn json_get_vector3(obj: &JsonValue, key: &str, dest: &mut Vec3) -> bool {
    match obj.get(key) {
        Some(JsonValue::Array(a)) if a.len() >= 3 => match (&a[0], &a[1], &a[2]) {
            (JsonValue::Number(x), JsonValue::Number(y), JsonValue::Number(z)) => {
                dest.x = *x as f32;
                dest.y = *y as f32;
                dest.z = *z as f32;
                true
            }
            _ => false,
        },
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_object_number_array_comment() {
        let text = r#"{
            // model
            "mass": 4.5,
            "cg": [ -0.1, 0.0, -0.05 ],
            "ok": true
        }"#;
        let mut p = JsonParser::new(text);
        let v = p.parse().expect("parse");
        let mut mass = 0.0f32;
        assert!(json_get_float(&v, "mass", &mut mass));
        assert!((mass - 4.5).abs() < 1e-6);
        let mut cg = Vec3::zero();
        assert!(json_get_vector3(&v, "cg", &mut cg));
        assert!((cg.x + 0.1).abs() < 1e-6);
        assert_eq!(v.get("ok"), Some(&JsonValue::Bool(true)));
    }
}
