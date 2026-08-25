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

OUT.write_text("\n".join(L))
print("wrote %s: %d functions, %d e-stop, NR_AUX_SERVO_FUNCTIONS = %d"
      % (OUT.name, len(entries), len(estop), count))
