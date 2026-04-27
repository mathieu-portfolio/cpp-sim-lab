#!/usr/bin/env bash
set -e

if [ $# -lt 1 ]; then
    echo "Usage: ./scripts/plot_bench.sh <benchmark_name>"
    exit 1
fi

BENCH_NAME=$1
CSV=benchmarks/$BENCH_NAME/results/results.csv
PLOT=$(find benchmarks -type f -path "*/$BENCH_NAME/plot.py" | head -n 1)

if [ -z "$PLOT" ]; then
    echo "Warning: plot.py not found for $BENCH_NAME"
    exit 0
fi

echo "Plotting $BENCH_NAME..."
python "$PLOT" "$CSV"