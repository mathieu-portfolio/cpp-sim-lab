#!/usr/bin/env bash
set -e

if [ $# -lt 2 ]; then
    echo "Usage: ./scripts/run_bench.sh <benchmark_name> <preset>"
    echo "Example: ./scripts/run_bench.sh particles_collision_compare release"
    exit 1
fi

BENCH_NAME=$1
PRESET=$2

BIN=build/$PRESET/benchmarks/$BENCH_NAME/Release/${BENCH_NAME}_bench.exe
OUT=benchmarks/$BENCH_NAME/results/results.csv

mkdir -p benchmarks/$BENCH_NAME/results

echo "Building ($PRESET)..."
cmake --build --preset $PRESET

echo "Running $BENCH_NAME..."
"$BIN" > "$OUT"

echo "Saved to $OUT"
