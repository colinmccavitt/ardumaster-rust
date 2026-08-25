#!/bin/sh
# Run the mutation gate for one crate, or the workspace if none is named.
#
# Usage: tools/mutants.sh [package]
#
# Surviving mutants are printed and the script exits non-zero. Each one is a
# line the tests cannot tell from a wrong line -- either write a test that
# distinguishes it, or record why it cannot be distinguished. Both outcomes
# are useful; silently ignoring it is not.
set -eu
export PATH="$HOME/.cargo/bin:$PATH"
cd "$(dirname "$0")/.."

# --test-workspace: mutants are judged by the whole workspace's tests, not
# only the tests in their own crate. Crates here are written to be called from
# other crates -- ap-motors' spool safety constraint is exercised by
# ap-copter's alt-hold recording -- and a per-crate run reports those as
# untested, which is the opposite of the truth. It belongs on the command line
# because cargo-mutants 27.1 does not read the equivalent config key.
if [ $# -gt 0 ]; then
    pkg=$1
    shift
    exec cargo mutants --package "$pkg" --test-workspace true --no-shuffle -j 4 "$@"
fi
exec cargo mutants --test-workspace true --no-shuffle -j 4
