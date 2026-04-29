#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="build"
BUILD_TYPE="${1:-Debug}"
BUILD_FIRST="${2:-}"

if [[ "$BUILD_FIRST" == "--build" ]]; then
    # Build only test-related target graph
    cmake --build "$BUILD_DIR" --target test --config "$BUILD_TYPE"
else
    echo "Skipping build (pass --build to compile tests first)."
fi

# Run all tests via CTest
ctest --test-dir "$BUILD_DIR" -C "$BUILD_TYPE" --output-on-failure -V
