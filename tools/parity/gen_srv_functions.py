"""Generate the SRV_Channel function table from upstream's header.

`SRV_Channel::Function` is ~170 entries naming every output a channel can be
assigned to, and the numbers are stored in the `SERVOn_FUNCTION` parameters --
so a transposed value does not fail to compile, it silently reassigns a
vehicle's outputs. Transcribing that by hand is exactly the wrong kind of work.

Emitted as a newtype over `u8` with associated constants rather than a Rust
enum. The values are what the parameters hold, and a parameter can hold a
number this build does not name; an enum would have to decide what to do with
that, while upstream simply carries it and lets `valid_function` judge.
"""
import re
from pathlib import Path

SRC = Path("/srv/ardumaster/upstream/plane-4.7.0/libraries/SRV_Channel/SRV_Channel.h")
OUT = Path("/srv/ardumaster/ports/plane-fw-rust/crates/ap-servo/src/function.rs")

text = SRC.read_text()
start = text.index("typedef enum")
end = text.index("k_nr_aux_servo_functions", start)
body = text[start:end]

entries = []
for m in re.finditer(r"^\s*(k_[A-Za-z0-9_]+)\s*=\s*(\d+)\s*,", body, re.M):
    entries.append((m.group(1), int(m.group(2))))

assert entries, "no enum entries found"
values = [v for _, v in entries]
assert len(set(values)) == len(values), "duplicate function values"
count = max(values) + 1

# Sanity: the motor split get_motor_function relies on must hold.
by_name = dict(entries)
for a, b in (("k_motor1", "k_motor9"), ("k_motor9", "k_motor13")):
    assert a in by_name and b in by_name, "motor anchors missing"

L = []
L.append("//! Output functions a servo channel can be assigned to.")
L.append("//!")
L.append("//! Generated from `SRV_Channel.h` by `tools/parity/gen_srv_functions.py`.")
L.append("//! Do not edit by hand.")
L.append("//!")
L.append("//! These numbers live in the `SERVOn_FUNCTION` parameters, so a wrong one")
L.append("//! does not fail to compile -- it silently reassigns a vehicle's outputs.")
L.append("//! That is why they are generated rather than typed.")
L.append("")
L.append("/// One past the highest function this build defines, upstream")
L.append("/// `k_nr_aux_servo_functions`. Also the size of the function registry.")
L.append("pub const NR_AUX_SERVO_FUNCTIONS: usize = %d;" % count)
L.append("")
L.append("/// What a channel is for, upstream `SRV_Channel::Function`.")
L.append("///")
L.append("/// A newtype rather than an enum, because the value comes from a parameter")
L.append("/// and a parameter can hold a number this build does not name. Upstream")
L.append("/// carries such a value and lets [`Self::valid`] judge it; an enum would")
L.append("/// have to reject it at the boundary instead.")
L.append("#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]")
L.append("pub struct Function(pub u8);")
L.append("")
L.append("impl Function {")
for name, value in entries:
    const = name[2:].upper()
    L.append("    /// Upstream `%s`." % name)
    L.append("    pub const %s: Self = Self(%d);" % (const, value))
L.append("")
L.append("    /// Whether this build defines this function, upstream")
L.append("    /// `valid_function`.")
L.append("    ///")
L.append("    /// Upstream also tests the lower bound against `k_none`. That half is")
L.append("    /// vacuous here -- `k_none` is zero and the value is unsigned -- so it")
L.append("    /// is asserted at compile time below instead, which is stronger: if")
L.append("    /// `k_none` ever stopped being zero this would fail to build rather")
L.append("    /// than quietly accept every value.")
L.append("    #[must_use]")
L.append("    pub const fn valid(self) -> bool {")
L.append("        (self.0 as usize) < NR_AUX_SERVO_FUNCTIONS")
L.append("    }")
L.append("")
L.append("    /// The function driving a zero-based motor channel, upstream")
L.append("    /// `SRV_Channels::get_motor_function`.")
L.append("    ///")
L.append("    /// Three ranges, not one. Motors 1-8 are contiguous, then 9-12 sit")
L.append("    /// elsewhere in the enum, then 13 onward somewhere else again --")
L.append("    /// because the later motor functions were added long after the first")
L.append("    /// eight and had to take whatever numbers were free. A port that")
L.append("    /// assumed `k_motor1 + channel` throughout would quietly drive the")
L.append("    /// wrong outputs on anything with more than eight motors.")
L.append("    #[must_use]")
L.append("    pub const fn motor(channel: u8) -> Self {")
L.append("        if channel < 8 {")
L.append("            Self(Self::MOTOR1.0 + channel)")
L.append("        } else if channel < 12 {")
L.append("            Self(Self::MOTOR9.0 + (channel - 8))")
L.append("        } else {")
L.append("            Self(Self::MOTOR13.0 + (channel - 12))")
L.append("        }")
L.append("    }")
L.append("}")
L.append("")
L.append("/// The premise `valid` relies on: `k_none` is the lowest function value,")
L.append("/// so an unsigned value can only fail the upper bound.")
L.append("const _: () = assert!(Function::NONE.0 == 0);")
L.append("")


# --- should_e_stop -------------------------------------------------------
SRC_CPP = SRC.parent / "SRV_Channel.cpp"
cpp = SRC_CPP.read_text()
estop_body = cpp[cpp.index("bool SRV_Channel::should_e_stop"):]
estop_body = estop_body[:estop_body.index("default:")]

estop = []
for m in re.finditer(r"case Function::(k_\w+)(?:\s*\.\.\.\s*Function::(k_\w+))?\s*:",
                     estop_body):
    lo_name, hi_name = m.group(1), m.group(2)
    assert lo_name in by_name, "unknown e-stop function %s" % lo_name
    if hi_name is None:
        estop.append(by_name[lo_name])
    else:
        # GCC case-range: every value between the two, inclusive.
        assert hi_name in by_name, "unknown e-stop function %s" % hi_name
        lo, hi = by_name[lo_name], by_name[hi_name]
        assert lo <= hi, "inverted case range %s ... %s" % (lo_name, hi_name)
        estop.extend(range(lo, hi + 1))

assert estop, "no e-stop functions found"
estop = sorted(set(estop))

L.append("/// Functions an emergency stop must be able to zero, upstream")
L.append("/// `SRV_Channel::should_e_stop`.")
L.append("///")
L.append("/// Motors, throttles, engine starters and the heli rotor speed")
L.append("/// controllers. Generated from the switch, because missing one means an")
L.append("/// E-stop that leaves something spinning -- the exact failure the feature")
L.append("/// exists to prevent.")
L.append("///")
L.append("/// Sorted, so the lookup can binary search.")
L.append("const E_STOP: [u8; %d] = %s;" % (len(estop), "[" + ", ".join(str(v) for v in estop) + "]"))
L.append("")
L.append("impl Function {")
L.append("    /// Whether an emergency stop must zero this function, upstream")
L.append("    /// `should_e_stop`.")
L.append("    #[must_use]")
L.append("    pub fn should_e_stop(self) -> bool {")
L.append("        E_STOP.binary_search(&self.0).is_ok()")
L.append("    }")
L.append("}")
L.append("")


# --- aux_servo_function_setup -------------------------------------------
AUX_CPP = SRC.parent / "SRV_Channel_aux.cpp"
aux = AUX_CPP.read_text()
setup_body = aux[aux.index("void SRV_Channel::aux_servo_function_setup"):]
setup_body = setup_body[:setup_body.index("default:")]

setup = []          # (function value, "Range"|"Angle", amount)
pending = []
for line in setup_body.splitlines():
    s = line.strip()
    m = re.match(r"case (k_\w+)(?:\s*\.\.\.\s*(k_\w+))?\s*:", s)
    if m:
        lo_name, hi_name = m.group(1), m.group(2)
        assert lo_name in by_name, "unknown setup function %s" % lo_name
        if hi_name is None:
            pending.append(by_name[lo_name])
        else:
            # GCC case-range, as in `case k_actuator1 ... k_actuator6:`.
            assert hi_name in by_name, "unknown setup function %s" % hi_name
            lo, hi = by_name[lo_name], by_name[hi_name]
            assert lo <= hi, "inverted case range %s ... %s" % (lo_name, hi_name)
            pending.extend(range(lo, hi + 1))
        continue
    m = re.match(r"set_(range|angle)\((-?\d+)\);", s)
    if m:
        assert pending, "set_%s with no cases above it" % m.group(1)
        kind = "Range" if m.group(1) == "range" else "Angle"
        for v in pending:
            setup.append((v, kind, int(m.group(2))))
        pending = []

assert setup, "no setup entries found"
assert not pending, "case labels with no set_range/set_angle after them"
setup.sort()

L.append("/// A channel's default output shape, upstream's `set_range` and")
L.append("/// `set_angle`.")
L.append("#[derive(Debug, Clone, Copy, PartialEq, Eq)]")
L.append("pub enum DefaultOutput {")
L.append("    /// One-sided: 0 to this value.")
L.append("    Range(u16),")
L.append("    /// Two-sided about the trim: plus and minus this value.")
L.append("    Angle(i16),")
L.append("}")
L.append("")
L.append("/// Sorted by function, so the lookup can binary search.")
L.append("#[allow(")
L.append("    clippy::type_complexity,")
L.append('    reason = "a generated lookup table; naming the tuple would not make it clearer"')
L.append(")]")
L.append("const DEFAULT_OUTPUT: [(u8, DefaultOutput); %d] = [" % len(setup))
for value, kind, amount in setup:
    L.append("    (%d, DefaultOutput::%s(%d))," % (value, kind, amount))
L.append("];")
L.append("")
L.append("impl Function {")
L.append("    /// The output shape this function defaults to, upstream")
L.append("    /// `aux_servo_function_setup`.")
L.append("    ///")
L.append("    /// `None` for a function with no default, which upstream leaves at")
L.append("    /// whatever the channel already had -- its `default:` does nothing")
L.append("    /// rather than picking a fallback.")
L.append("    ///")
L.append("    /// Upstream applies this only when the channel has not already been")
L.append("    /// set up, and the caller is responsible for that check; this is the")
L.append("    /// table, not the guard.")
L.append("    #[must_use]")
L.append("    pub fn default_output(self) -> Option<DefaultOutput> {")
L.append("        DEFAULT_OUTPUT")
L.append("            .binary_search_by_key(&self.0, |&(f, _)| f)")
L.append("            .ok()")
L.append("            .and_then(|i| DEFAULT_OUTPUT.get(i))")
L.append("            .map(|&(_, out)| out)")
L.append("    }")
L.append("}")
L.append("")


# --- is_control_surface, is_motor, motor_num ----------------------------
surf_body = cpp[cpp.index("bool SRV_Channel::is_control_surface"):]
surf_body = surf_body[:surf_body.index("default:")]

surfaces = []
for m in re.finditer(r"case Function::(k_\w+)(?:\s*\.\.\.\s*Function::(k_\w+))?\s*:",
                     surf_body):
    lo_name, hi_name = m.group(1), m.group(2)
    assert lo_name in by_name, "unknown surface function %s" % lo_name
    if hi_name is None:
        surfaces.append(by_name[lo_name])
    else:
        assert hi_name in by_name, "unknown surface function %s" % hi_name
        surfaces.extend(range(by_name[lo_name], by_name[hi_name] + 1))

assert surfaces, "no control-surface functions found"
surfaces = sorted(set(surfaces))

L.append("/// Functions that move an aerodynamic surface, upstream")
L.append("/// `SRV_Channel::is_control_surface`.")
L.append("///")
L.append("/// Sorted, so the lookup can binary search.")
L.append("const CONTROL_SURFACE: [u8; %d] = %s;"
         % (len(surfaces), "[" + ", ".join(str(v) for v in surfaces) + "]"))
L.append("")
L.append("impl Function {")
L.append("    /// Whether this drives a multicopter motor, upstream `is_motor`.")
L.append("    ///")
L.append("    /// The same three ranges as [`Self::motor`], expressed against the")
L.append("    /// generated constants so they move together if upstream")
L.append("    /// renumbers.")
L.append("    #[must_use]")
L.append("    pub const fn is_motor(self) -> bool {")
L.append("        (self.0 >= Self::MOTOR1.0 && self.0 <= Self::MOTOR8.0)")
L.append("            || (self.0 >= Self::MOTOR9.0 && self.0 <= Self::MOTOR12.0)")
L.append("            || (self.0 >= Self::MOTOR13.0 && self.0 <= Self::MOTOR32.0)")
L.append("    }")
L.append("")
L.append("    /// Whether this moves an aerodynamic surface, upstream")
L.append("    /// `is_control_surface`.")
L.append("    #[must_use]")
L.append("    pub fn is_control_surface(self) -> bool {")
L.append("        CONTROL_SURFACE.binary_search(&self.0).is_ok()")
L.append("    }")
L.append("")
L.append("    /// The zero-based motor this function drives, upstream")
L.append("    /// `get_motor_num`. `None` for anything that is not a motor.")
L.append("    ///")
L.append("    /// The exact inverse of [`Self::motor`], written once rather than")
L.append("    /// transcribed a second time -- a round-trip test pins the pair")
L.append("    /// together, which no amount of re-reading the three ranges would.")
L.append("    #[must_use]")
L.append("    pub const fn motor_num(self) -> Option<u8> {")
L.append("        if self.0 >= Self::MOTOR1.0 && self.0 <= Self::MOTOR8.0 {")
L.append("            Some(self.0 - Self::MOTOR1.0)")
L.append("        } else if self.0 >= Self::MOTOR9.0 && self.0 <= Self::MOTOR12.0 {")
L.append("            Some(8 + (self.0 - Self::MOTOR9.0))")
L.append("        } else if self.0 >= Self::MOTOR13.0 && self.0 <= Self::MOTOR32.0 {")
L.append("            Some(12 + (self.0 - Self::MOTOR13.0))")
L.append("        } else {")
L.append("            None")
L.append("        }")
L.append("    }")
L.append("}")
L.append("")

OUT.write_text("\n".join(L))
print("wrote %s: %d functions, %d e-stop, %d defaults, %d surfaces, NR_AUX_SERVO_FUNCTIONS = %d"
      % (OUT.name, len(entries), len(estop), len(setup), len(surfaces), count))
