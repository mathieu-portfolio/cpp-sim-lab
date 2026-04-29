#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [[ $# -lt 1 ]]; then
    echo "Usage: ./scripts/run_bench.sh <benchmark_name> [preset]"
    echo "Example: ./scripts/run_bench.sh boids_naive_neighbors release"
    exit 1
fi

BENCH_NAME=$1
PRESET=${2:-release}
BUILD_DIR="build/$PRESET"
OUT_DIR="benchmarks/simulations/$BENCH_NAME/results"
OUT="$OUT_DIR/results.csv"

if [[ ! -d "benchmarks/simulations/$BENCH_NAME" ]]; then
    echo "Error: benchmark directory not found for $BENCH_NAME"
    exit 1
fi

echo "Building ($PRESET)..."
cmake --build --preset "$PRESET"

BIN="$BUILD_DIR/benchmarks/simulations/$BENCH_NAME/${BENCH_NAME}_bench"
if [[ ! -x "$BIN" ]]; then
    BIN="$BIN.exe"
fi

if [[ ! -x "$BIN" ]]; then
    echo "Error: could not find benchmark binary at expected path: $BIN"
    exit 1
fi

mkdir -p "$OUT_DIR"

echo "Running $BENCH_NAME..."
"$BIN" > "$OUT"

echo "Saved to $OUT"
