#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
    echo "Usage: ./scripts/run_bench.sh <benchmark_name> [preset]"
    echo "Example: ./scripts/run_bench.sh boids_naive_neighbors release"
    exit 1
fi

BENCH_NAME=$1
PRESET=${2:-release}
BUILD_DIR="build/$PRESET"

BENCH_DIR=$(find benchmarks -type d -path "*/$BENCH_NAME" | head -n 1)

if [ -z "$BENCH_DIR" ]; then
    echo "Error: benchmark directory not found for $BENCH_NAME"
    exit 1
fi

echo "Building ($PRESET)..."
cmake --build --preset "$PRESET"

echo "Locating benchmark binary..."
BIN=$(find "$BUILD_DIR" -type f \( -name "${BENCH_NAME}_bench" -o -name "${BENCH_NAME}_bench.exe" \) | head -n 1)

if [ -z "$BIN" ]; then
    echo "Error: could not find benchmark binary for $BENCH_NAME"
    exit 1
fi

OUT_DIR="$BENCH_DIR/results"
OUT="$OUT_DIR/results.csv"

mkdir -p "$OUT_DIR"

echo "Running $BENCH_NAME..."
"$BIN" > "$OUT"

echo "Saved to $OUT"
