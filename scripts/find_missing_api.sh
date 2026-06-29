#!/usr/bin/env bash
# List public API functions declared B3_API in include/box3d that have no definition.
# Heuristic keyed to Box2D/Box3D style: a definition opens at column 0 as
# "<ret> name(" with no trailing ';'. The prototype variant ends in ';'.
# Run from anywhere; resolves paths from its own dir.
#
# Limitation: a definition hidden behind #if 0 or a disabled platform block reads
# as defined. For an authoritative cross-check, diff against the built static lib:
# dumpbin /symbols build/bin/Debug/box3d.lib (MSVC) or nm (gcc/clang).
set -uo pipefail
cd "$(dirname "$0")/.."

# Declared public API: last token before first '(' on B3_API lines.
# #define B3_API ... lines are indented, so ^B3_API skips them.
decl=$(grep -hE '^B3_API' include/box3d/*.h \
  | sed -E 's/\(.*$//' | awk '{print $NF}' | sort -u)

# Defined anywhere: column-0 definition-opening lines (not ';'-terminated).
# Scan .c and .h so a rare inline-in-header definition isn't false-flagged.
def=$(grep -rhE '^[A-Za-z_].*\(' src include --include='*.c' --include='*.h' \
  | grep -vE ';[[:space:]]*$' \
  | sed -E 's/\(.*$//' | awk '{print $NF}' | sort -u)

missing=$(comm -23 <(echo "$decl") <(echo "$def"))

if [ -z "$missing" ]; then
  echo "All B3_API functions have definitions."
else
  echo "Declared B3_API with no definition in src/include:"
  echo "$missing" | sed 's/^/  /'
fi
