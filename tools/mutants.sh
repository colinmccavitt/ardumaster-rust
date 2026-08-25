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

if [ $# -gt 0 ]; then
    exec cargo mutants --package "$1" --no-shuffle -j 4 "$@"
fi
exec cargo mutants --no-shuffle -j 4
