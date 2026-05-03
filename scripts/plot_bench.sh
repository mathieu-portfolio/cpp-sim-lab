#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [[ $# -lt 1 ]]; then
    echo "Usage: ./scripts/plot_bench.sh <benchmark_name>"
    exit 1
fi

BENCH_NAME=$1
BENCH_DIR="benchmarks/simulations/*/$BENCH_NAME"
BENCH_DIR=$(echo $BENCH_DIR)

if [[ ! -d "$BENCH_DIR" ]]; then
    echo "Error: benchmark directory not found for $BENCH_NAME"
    exit 1
fi

CSV="$BENCH_DIR/results/results.csv"
PLOT="$BENCH_DIR/plot.py"

if [[ ! -f "$CSV" ]]; then
    echo "Error: results CSV not found: $CSV"
    exit 1
fi

if [[ ! -f "$PLOT" ]]; then
    echo "Warning: plot.py not found for $BENCH_NAME"
    exit 0
fi

echo "Plotting $BENCH_NAME..."
python "$PLOT" "$CSV"