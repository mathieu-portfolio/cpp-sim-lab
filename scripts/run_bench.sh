#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

source "$ROOT_DIR/scripts/find_bin.sh"

if [[ $# -lt 1 ]]; then
    echo "Usage: ./scripts/run_bench.sh <benchmark_name> [preset] [--build]"
    echo "Example: ./scripts/run_bench.sh boids_naive_neighbors release --build"
    exit 1
fi

BENCH_NAME="$1"
PRESET="${2:-release}"
BUILD_FIRST="${3:-}"

BUILD_DIR="build/$PRESET"
BENCH_SRC_DIR="benchmarks/simulations/$BENCH_NAME"
OUT_DIR="$BENCH_SRC_DIR/results"
OUT="$OUT_DIR/results.csv"
BENCH_EXE="${BENCH_NAME}_bench"

if [[ ! -d "$BENCH_SRC_DIR" ]]; then
    echo "Error: benchmark directory not found for $BENCH_NAME"
    exit 1
fi

if [[ "$BUILD_FIRST" == "--build" ]]; then
    echo "Building benchmark target ($PRESET): $BENCH_EXE"
    cmake --build --preset "$PRESET" --target "$BENCH_EXE"
else
    echo "Skipping build (pass --build to compile $BENCH_EXE first)."
fi

if [[ ! -d "$BUILD_DIR" ]]; then
    echo "Error: build directory not found: $BUILD_DIR"
    exit 1
fi

BIN="$(
    find_bin "$BUILD_DIR" "$BENCH_EXE" \
        "$BUILD_DIR/benchmarks/simulations/$BENCH_NAME" \
        "$BUILD_DIR/benchmarks/simulations" \
        "$BUILD_DIR/bin"
)"

if [[ -z "$BIN" ]]; then
    echo "Error: could not find benchmark binary: $BENCH_EXE"
    echo "Searched under: $BUILD_DIR"
    exit 1
fi

mkdir -p "$OUT_DIR"

echo "Running $BENCH_NAME..."
"$BIN" > "$OUT"

echo "Saved to $OUT"
