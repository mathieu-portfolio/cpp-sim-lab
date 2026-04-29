#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

PRESET=${1:-release}

echo "Running all benchmarks with preset: $PRESET"

while IFS= read -r cmake_file; do
    BENCH_DIR=$(dirname "$cmake_file")
    BENCH_NAME=$(basename "$BENCH_DIR")

    if [[ ! -f "$BENCH_DIR/main.cpp" ]]; then
        continue
    fi

    echo "----------------------------------"
    echo "Benchmark: $BENCH_NAME"

    ./scripts/run_and_plot.sh "$BENCH_NAME" "$PRESET"
done < <(find benchmarks -type f -name CMakeLists.txt | sort)

echo "All benchmarks done."