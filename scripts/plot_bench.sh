#!/usr/bin/env bash
set -e

if [ $# -lt 1 ]; then
    echo "Usage: ./scripts/plot_bench.sh <benchmark_name>"
    exit 1
fi

BENCH_NAME=$1
CSV=benchmarks/$BENCH_NAME/results/results.csv
PLOT=benchmarks/$BENCH_NAME/plot.py

if [ ! -f "$CSV" ]; then
    echo "No results found at $CSV"
    exit 1
fi

echo "Plotting $BENCH_NAME..."
python "$PLOT" "$CSV"
