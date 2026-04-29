#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PRESET=${1:-debug}

./scripts/build.sh "$PRESET"

echo "Running tests: $PRESET"
ctest --test-dir "build/$PRESET" --build-config "$PRESET" --output-on-failure