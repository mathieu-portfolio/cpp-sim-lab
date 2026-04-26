#!/usr/bin/env bash
set -e

PRESET=${1:-debug}

./scripts/build.sh "$PRESET"

echo "Running tests: $PRESET"
ctest --test-dir "build/$PRESET" -C "$PRESET" --output-on-failure
