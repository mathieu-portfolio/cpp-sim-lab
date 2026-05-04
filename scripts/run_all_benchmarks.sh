#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PRESET=${1:-release}
BUILD_FIRST="${2:-}"
BENCHMARK_TIME_LIMIT_SEC="${BENCHMARK_TIME_LIMIT_SEC:-0}"
TOTAL_START_TS=$(date +%s)

if ! [[ "$BENCHMARK_TIME_LIMIT_SEC" =~ ^[0-9]+$ ]]; then
    echo "Error: BENCHMARK_TIME_LIMIT_SEC must be a non-negative integer (seconds)."
    exit 1
fi

echo "Running all benchmarks with preset: $PRESET"

for bench_dir in benchmarks/simulations/*/*; do
    [[ -d "$bench_dir" ]] || continue
    [[ -f "$bench_dir/main.cpp" ]] || continue

    BENCH_NAME="${bench_dir##*/}"

    if (( BENCHMARK_TIME_LIMIT_SEC > 0 )); then
        NOW_TS=$(date +%s)
        ELAPSED_SEC=$((NOW_TS - TOTAL_START_TS))
        if (( ELAPSED_SEC >= BENCHMARK_TIME_LIMIT_SEC )); then
            echo "----------------------------------"
            echo "Skipping $BENCH_NAME (time threshold reached: ${ELAPSED_SEC}s >= ${BENCHMARK_TIME_LIMIT_SEC}s)"
            continue
        fi
    fi

    echo "----------------------------------"
    echo "Benchmark: $BENCH_NAME"

    if [[ -n "$BUILD_FIRST" ]]; then
        ./scripts/run_and_plot.sh "$BENCH_NAME" "$PRESET" "$BUILD_FIRST"
    else
        ./scripts/run_and_plot.sh "$BENCH_NAME" "$PRESET"
    fi
done

echo "All benchmarks done."
