#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PRESET=${1:-debug}
BUILD_FIRST="${2:-}"

if [[ "$BUILD_FIRST" == "--build" ]]; then
    ./scripts/build.sh "$PRESET" test
else
    echo "Skipping build (pass --build to compile tests first)."
fi

echo "Running tests: $PRESET"
ctest --test-dir "build/$PRESET" --build-config "$PRESET" --output-on-failure
