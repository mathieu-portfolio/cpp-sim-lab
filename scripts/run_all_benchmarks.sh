#!/usr/bin/env bash
set -euo pipefail

PRESET=${1:-release}

echo "Running all benchmarks with preset: $PRESET"

while IFS= read -r cmake_file; do
    BENCH_DIR=$(dirname "$cmake_file")
    BENCH_NAME=$(basename "$BENCH_DIR")

    # Only leaf benchmark directories have a main.cpp.
    if [ ! -f "$BENCH_DIR/main.cpp" ]; then
        continue
    fi

    echo "----------------------------------"
    echo "Benchmark: $BENCH_NAME"

    ./scripts/run_and_plot.sh "$BENCH_NAME" "$PRESET"
done < <(find benchmarks -type f -name CMakeLists.txt | sort)

echo "All benchmarks done."
