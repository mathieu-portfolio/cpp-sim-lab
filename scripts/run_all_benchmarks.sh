#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PRESET=${1:-release}
BUILD_FIRST="${2:-}"

echo "Running all benchmarks with preset: $PRESET"

for bench_dir in benchmarks/simulations/*/*; do
    [[ -d "$bench_dir" ]] || continue
    [[ -f "$bench_dir/main.cpp" ]] || continue

    BENCH_NAME="${bench_dir##*/}"

    echo "----------------------------------"
    echo "Benchmark: $BENCH_NAME"

    if [[ -n "$BUILD_FIRST" ]]; then
        ./scripts/run_and_plot.sh "$BENCH_NAME" "$PRESET" "$BUILD_FIRST"
    else
        ./scripts/run_and_plot.sh "$BENCH_NAME" "$PRESET"
    fi
done

echo "All benchmarks done."