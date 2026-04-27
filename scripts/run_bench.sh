#!/usr/bin/env bash
set -e

if [ $# -lt 2 ]; then
    echo "Usage: ./scripts/run_bench.sh <benchmark_name> <preset>"
    echo "Example: ./scripts/run_bench.sh boids_naive_neighbors release"
    exit 1
fi

BENCH_NAME=$1
PRESET=$2

BUILD_DIR="build/$PRESET"

echo "Building ($PRESET)..."
cmake --build --preset "$PRESET"

echo "Locating benchmark binary..."
BIN=$(find "$BUILD_DIR" -type f -name "${BENCH_NAME}_bench.exe" | head -n 1)

if [ -z "$BIN" ]; then
    echo "Error: could not find benchmark binary for $BENCH_NAME"
    exit 1
fi

echo "Found: $BIN"

# Infer output directory from benchmark location
# e.g. benchmarks/simulations/boids_cpu/boids_naive_neighbors/results
OUT_DIR=$(echo "$BIN" | sed -E "s|build/[^/]+/(benchmarks/.*/)$BENCH_NAME/.*|\1$BENCH_NAME/results|")

mkdir -p "$OUT_DIR"
OUT="$OUT_DIR/results.csv"

echo "Running $BENCH_NAME..."
"$BIN" > "$OUT"

echo "Saved to $OUT"